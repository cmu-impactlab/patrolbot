"""Small bounded JSON codec for the trusted tuning boundary (no ROS imports)."""

import json
import math


def loads(text, limit=20 * 1024):
    if not isinstance(text, str) or len(text) > limit or len(text.encode('utf-8')) > limit:
        raise ValueError('invalid or oversized JSON text')
    depth, quoted, escaped = 0, False, False
    for char in text:
        if quoted:
            if escaped:
                escaped = False
            elif char == '\\':
                escaped = True
            elif char == '"':
                quoted = False
        elif char == '"':
            quoted = True
        elif char in '[{':
            depth += 1
            if depth > 32:
                raise ValueError('JSON is too deeply nested')
        elif char in ']}':
            depth -= 1

    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate JSON key')
            result[key] = value
        return result

    def number(value):
        result = float(value)
        if not math.isfinite(result):
            raise ValueError('non-finite JSON number')
        return result

    def integer(value):
        if len(value) > 20:
            raise ValueError('oversized JSON integer')
        result = int(value)
        if abs(result) > 2**63 - 1:
            raise ValueError('oversized JSON integer')
        return result

    def constant(value):
        raise ValueError('non-finite JSON constant')

    return json.loads(text, object_pairs_hook=pairs, parse_constant=constant,
                      parse_float=number, parse_int=integer)
