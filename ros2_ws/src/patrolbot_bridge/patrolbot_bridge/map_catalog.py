"""Content-addressed local maps; no ROS dependency and no arbitrary map URLs."""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile

DEFAULT_MAP_ID = 'cmuq-floor2'
DOCK_MAP_ID = DEFAULT_MAP_ID


def describe_map(path: Path) -> dict:
    fields = {}
    for line in path.read_text().splitlines():
        key, sep, value = line.split('#', 1)[0].partition(':')
        if sep:
            fields[key.strip()] = value.strip()
    image = (path.parent / fields['image']).resolve()
    if image.parent != path.parent.resolve():
        raise ValueError('map image must be beside its YAML')
    origin = json.loads(fields['origin'])
    meta = dict(resolution=float(fields['resolution']), origin=origin,
                negate=int(fields['negate']),
                occupied_thresh=float(fields['occupied_thresh']),
                free_thresh=float(fields['free_thresh']))
    if (len(origin) != 3 or not all(math.isfinite(v) for v in origin)
            or not math.isfinite(meta['resolution']) or meta['resolution'] <= 0
            or meta['negate'] not in (0, 1)
            or not 0 <= meta['free_thresh'] < meta['occupied_thresh'] <= 1):
        raise ValueError('invalid map geometry or thresholds')
    data = image.read_bytes()
    position, tokens = 0, []
    while len(tokens) < 4:
        match = re.compile(rb'\s*(?:#[^\n]*\n\s*)*(\S+)').match(data, position)
        if match is None:
            raise ValueError('invalid PGM header')
        tokens.append(match.group(1)); position = match.end()
    width, height = int(tokens[1]), int(tokens[2])
    if (tokens[0] != b'P5' or tokens[3] != b'255' or width <= 0 or height <= 0
            or len(data[position + 1:]) != width * height):
        raise ValueError('invalid PGM raster')
    meta.update(width=width, height=height)
    digest = hashlib.sha256(json.dumps(meta, sort_keys=True, separators=(',', ':')).encode())
    digest.update(b'\0'); digest.update(data[position + 1:])
    return dict(meta, revision='sha256:' + digest.hexdigest(),
                image_sha256=hashlib.sha256(data).hexdigest())


class MapCatalog:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        manifest = json.loads((self.directory / 'catalog.json').read_text())
        if manifest['schema_version'] != 1 or manifest['default_map_id'] != DEFAULT_MAP_ID:
            raise ValueError('unsupported map catalog')
        self.maps = {}
        for entry in manifest['maps']:
            key = entry['map_id']
            path = (self.directory / entry['yaml']).resolve()
            if path.parent != self.directory or key in self.maps:
                raise ValueError('invalid catalog path or duplicate identity')
            actual = describe_map(path)
            if any(entry.get(k) != v for k, v in actual.items()):
                raise ValueError('map content revision mismatch: ' + key)
            self.maps[key] = dict(entry, path=str(path))
        if set(self.maps) != {'cmuq-floor1', 'cmuq-floor2'}:
            raise ValueError('incomplete map catalog')

    def require(self, map_id, revision):
        entry = self.maps.get(map_id)
        if entry is None or entry['revision'] != revision:
            raise ValueError('unknown map or map revision mismatch')
        return entry

    def contains(self, entry, x, y):
        if not all(math.isfinite(v) for v in (x, y)):
            return False
        ox, oy, yaw = entry['origin']
        dx, dy = x - ox, y - oy
        mx = math.cos(yaw) * dx + math.sin(yaw) * dy
        my = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return (0 <= mx < entry['width'] * entry['resolution'] and
                0 <= my < entry['height'] * entry['resolution'])


class ActiveMapStore:
    """Only a missing file in an available durable directory is first install.

    The caller persists only after map_server AND AMCL acknowledgement. A read
    or write error must keep the motion hold, never silently choose Floor 2.
    """
    def __init__(self, directory, catalog):
        self.directory = Path(directory)
        self.catalog = catalog
        self.path = self.directory / 'active-map.json'

    def load(self):
        if not self.directory.is_dir():
            raise ValueError('durable robot state directory unavailable')
        try:
            with self.path.open() as stream:
                state = json.load(stream)
        except FileNotFoundError:
            if self.path.is_symlink():
                raise ValueError('broken active map state link')
            return self.catalog.maps[DEFAULT_MAP_ID]
        if state.get('schema_version') != 1:
            raise ValueError('unsupported active map state')
        return self.catalog.require(state['map_id'], state['revision'])

    def save(self, map_id, revision):
        self.catalog.require(map_id, revision)
        data = dict(schema_version=1, map_id=map_id, revision=revision)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=self.directory,
                                             prefix='.active-map-', delete=False) as stream:
                temporary = stream.name
                json.dump(data, stream, sort_keys=True)
                stream.flush(); os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            fd = os.open(self.directory, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
