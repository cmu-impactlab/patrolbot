"""
Strict, bounded YAML loading for robot-owned tuning configuration.

This module exists because the tuning store writes and re-reads YAML that the
robot later trusts as configuration. Ordinary ``yaml.safe_load`` is not enough:
it silently accepts anchors/aliases (the basis of "billion laughs" and merge-key
bombs), it keeps the *last* value of a duplicate key, and it happily returns
non-finite floats. A corrupted or hostile file must fail closed, never be
interpreted leniently.

Everything here is pure Python plus PyYAML. There are no ROS imports, so the
loader and its limits are unit-tested on a plain development host.

The loader only ever produces ``dict[str, ...]``/``list``/``str``/``int``/
``float``/``bool``/``None``. Custom tags are rejected by construction; there is
no ``!python/...`` path.
"""

from __future__ import annotations

import math
from typing import Any

import yaml

# Bounded, generous enough for the generated consumer overlays (the largest is
# the Nav2 overlay) but small enough that an oversized file cannot become an
# unbounded memory or CPU problem.
MAX_YAML_BYTES = 256 * 1024
MAX_YAML_DEPTH = 24
MAX_YAML_NODES = 20_000
MAX_SCALAR_CHARS = 4_096


class StrictYAMLError(ValueError):
    """Raised when YAML is unsafe, malformed or outside the accepted bounds."""


class _StrictSafeLoader(yaml.SafeLoader):
    """SafeLoader that refuses anchors/aliases and duplicate keys."""

    def compose_node(self, parent, index):  # type: ignore[override]
        if self.check_event(yaml.AliasEvent):
            raise StrictYAMLError('YAML aliases are not allowed')
        if getattr(self.peek_event(), 'anchor', None) is not None:
            raise StrictYAMLError('YAML anchors are not allowed')
        node = super().compose_node(parent, index)
        if getattr(node, 'anchor', None) is not None:
            raise StrictYAMLError('YAML anchors are not allowed')
        return node

    def construct_mapping(self, node, deep=False):  # type: ignore[override]
        if not isinstance(node, yaml.MappingNode):
            raise StrictYAMLError('expected a YAML mapping')
        seen: set[Any] = set()
        for key_node, _value_node in node.value:
            key = self.construct_object(key_node, deep=True)
            if not isinstance(key, str):
                raise StrictYAMLError('mapping keys must be strings')
            if key in seen:
                raise StrictYAMLError('duplicate mapping key: ' + key)
            seen.add(key)
        return super().construct_mapping(node, deep=deep)


def _walk(value: Any, depth: int, budget: list[int]) -> None:
    budget[0] -= 1
    if budget[0] < 0:
        raise StrictYAMLError('YAML node count exceeds limit')
    if depth > MAX_YAML_DEPTH:
        raise StrictYAMLError('YAML nesting depth exceeds limit')
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise StrictYAMLError('mapping keys must be strings')
            if len(key) > MAX_SCALAR_CHARS:
                raise StrictYAMLError('mapping key too long')
            _walk(item, depth + 1, budget)
    elif isinstance(value, list):
        for item in value:
            _walk(item, depth + 1, budget)
    elif isinstance(value, str):
        if len(value) > MAX_SCALAR_CHARS:
            raise StrictYAMLError('string scalar too long')
    elif isinstance(value, bool) or value is None:
        return
    elif isinstance(value, float):
        if not math.isfinite(value):
            raise StrictYAMLError('non-finite float is not allowed')
    elif isinstance(value, int):
        return
    else:  # pragma: no cover - SafeLoader never yields other types
        raise StrictYAMLError('unsupported scalar type: ' + type(value).__name__)


def strict_yaml_load(text: Any, *, max_bytes: int = MAX_YAML_BYTES) -> Any:
    """
    Parse ``text`` as bounded, tag-free, duplicate-free YAML.

    Raises :class:`StrictYAMLError` on any violation. Only a ``str`` is
    accepted; ``bytes`` and everything else is a structured error rather than
    implicit decoding work.
    """
    if not isinstance(text, str):
        raise StrictYAMLError('expected a text document')
    if '\x00' in text:
        raise StrictYAMLError('NUL byte in document')
    if len(text) > max_bytes:
        raise StrictYAMLError('document exceeds byte limit')
    try:
        encoded = text.encode('utf-8')
    except UnicodeError as exc:
        raise StrictYAMLError('document is not valid UTF-8') from exc
    if len(encoded) > max_bytes:
        raise StrictYAMLError('document exceeds byte limit')
    try:
        value = yaml.load(text, Loader=_StrictSafeLoader)
    except StrictYAMLError:
        raise
    except yaml.YAMLError as exc:
        raise StrictYAMLError('invalid YAML: ' + exc.__class__.__name__) from exc
    except RecursionError as exc:
        raise StrictYAMLError('YAML nesting too deep to parse') from exc
    _walk(value, 0, [MAX_YAML_NODES])
    return value


def strict_yaml_load_file(path, *, max_bytes: int = MAX_YAML_BYTES) -> Any:
    """Read and strictly parse a file, never following a symlink."""
    import os

    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0)
    try:
        fd = os.open(path, flags)
    except OSError as exc:
        raise StrictYAMLError('cannot open config file: ' + exc.__class__.__name__) from exc
    try:
        raw = os.read(fd, max_bytes + 1)
    finally:
        os.close(fd)
    if len(raw) > max_bytes:
        raise StrictYAMLError('document exceeds byte limit')
    try:
        text = raw.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise StrictYAMLError('config file is not UTF-8') from exc
    return strict_yaml_load(text, max_bytes=max_bytes)


def strict_yaml_dump(data: Any) -> str:
    """Render ``data`` deterministically: sorted keys, no anchors, no aliases."""
    _walk(data, 0, [MAX_YAML_NODES])
    return yaml.safe_dump(
        data,
        default_flow_style=False,
        sort_keys=True,
        allow_unicode=True,
        width=1000,
    )
