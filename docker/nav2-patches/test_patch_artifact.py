#!/usr/bin/env python3
"""Check that the patch's added helper exactly matches its canonical source."""

from hashlib import sha256
from pathlib import Path


ROOT = Path(__file__).parent
PATCH = ROOT / '0001-costmap-stale-warning-policy.patch'
HEADER = ROOT / 'nav2_costmap_2d/src/stale_warning_policy.hpp'


def added_file_from_patch(text):
    marker = '+++ b/nav2_costmap_2d/src/stale_warning_policy.hpp\n'
    start = text.index(marker) + len(marker)
    lines = text[start:].splitlines(keepends=True)
    added = []
    for line in lines:
        if line.startswith('@@ '):
            continue
        if line.startswith('+'):
            added.append(line[1:])
    return ''.join(added)


def digest(text):
    return sha256(text.encode('utf-8')).hexdigest()


def main():
    patch_text = PATCH.read_text(encoding='utf-8')
    canonical = HEADER.read_text(encoding='utf-8')
    applied = added_file_from_patch(patch_text)
    assert digest(applied) == digest(canonical), 'embedded helper differs from canonical helper'
    assert 'a/nav2_costmap_2d/src/observation_buffer.cpp' in patch_text
    assert 'stale checks: %zu' in patch_text
    print('patch artifact header hash and source hunks verified')


if __name__ == '__main__':
    main()
