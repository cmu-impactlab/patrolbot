#!/usr/bin/env python3
"""Offline validator and consistency checks for the robot-tuning contract.

This module is portable: it has NO ROS imports and only needs ``jsonschema``
(plus stdlib) to run. PyYAML is used only by the robot-repo source-consistency
checks in ``test_robot_tuning_sources.py``.

Slice 1 is contract-only. Nothing here contacts a robot, publishes ROS traffic,
writes YAML, or claims that the runtime capability exists or is active. This is
a standalone shape checker: it does NOT know the robot's authoritative desired
revision or session unless the caller passes them in ``context``. With no
context it reports ``validation_only: true`` and never implies authorization.

Things JSON Schema cannot express, enforced here:

* Non-finite numbers: JSON has no NaN/Infinity literals, but Python accepts
  ``NaN``/``Infinity`` and ``1e9999`` overflows float; the strict parser rejects
  both literal constants and non-finite float results, and every loaded or
  in-memory payload is scanned for non-finite numbers anywhere, including
  snapshots.
* Boolean-vs-number: the registry type decides; ``true`` is never a number and
  ``1000.0`` is never accepted for an integer field.
* Duplicate object keys, oversized/deep/cyclic payloads: rejected with bounded
  traversal so a malformed input cannot stall the validator.
* Dependency coherence: every dependency edge incident to a changed parameter
  (either endpoint) is checked against the merged defaults + supplied base +
  the whole valid patch, so a one-sided edit cannot bypass a constraint.
"""

from __future__ import annotations

import json
import math
import re
import sys
import uuid
from pathlib import Path
from typing import Any

try:
    import jsonschema
except ImportError as exc:  # pragma: no cover - exercised only without deps
    raise SystemExit(
        "jsonschema is required: pip install -r contracts/requirements-test.txt"
    ) from exc

CONTRACTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = CONTRACTS_DIR.parent

SCHEMA_PATH = CONTRACTS_DIR / "robot-tuning.schema.json"
REGISTRY_PATH = CONTRACTS_DIR / "robot-tuning.registry.json"
FIXTURES_PATH = CONTRACTS_DIR / "robot-tuning.fixtures.json"

CONTRACT = "patrolbot-robot-tuning"
CONTRACT_VERSION = 1
CAPABILITY = "robot_tuning_v1"

REQUEST_OPERATIONS = ("read", "save", "apply", "redeploy", "status", "restore")

# Separate, explicit bounds. Requests are small wire messages; artifacts (the
# schema/registry/fixtures) are larger but still bounded.
REQUEST_MAX_BYTES = 16 * 1024
ARTIFACT_MAX_BYTES = 512 * 1024
MAX_DEPTH = 64
MAX_ITEMS = 200_000
MAX_INT = 2 ** 63 - 1
JS_SAFE_INT = 2 ** 53 - 1

UUID_PATTERN = re.compile(
    r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")

# Severity-free codes shared with fixtures for precise negative tests.
CODE_MISSING_FIELD = "missing_field"
CODE_UNKNOWN_FIELD = "unknown_field"
CODE_BAD_TYPE = "bad_type"
CODE_BAD_CONST = "bad_const"
CODE_BAD_ENUM = "bad_enum"
CODE_BAD_PATTERN = "bad_pattern"
CODE_BAD_SHAPE = "bad_shape"
CODE_OUT_OF_BOUNDS = "out_of_bounds"
CODE_INVALID_JSON = "invalid_json"
CODE_NON_FINITE = "non_finite_number"
CODE_DUPLICATE_KEY = "duplicate_key"
CODE_SCHEMA_VIOLATION = "schema_violation"
CODE_PATH_INJECTION = "path_injection"
CODE_UNKNOWN_LOGICAL_ID = "unknown_logical_id"
CODE_PROTECTED_PARAMETER = "protected_parameter"
CODE_NOT_APPLICABLE = "not_applicable"
CODE_NOT_EDITABLE = "not_editable"
CODE_BOOL_AS_NUMBER = "bool_as_number"
CODE_DEPENDENCY_VIOLATION = "dependency_violation"
CODE_DEPENDENCY_OPERAND = "invalid_dependency_operand"
CODE_INVALID_BASE_VALUE = "invalid_base_value"
CODE_DUPLICATE_CHANGE_ID = "duplicate_change_id"
CODE_INCONSISTENT_ACTIVE_STATE = "inconsistent_active_state"
CODE_PENDING_FLAG_MISMATCH = "pending_flag_mismatch"
CODE_UNADVERTISED_VIOLATION = "unadvertised_violation"
CODE_UNSUPPORTED_OPERATION = "unsupported_operation"
CODE_PAYLOAD_TOO_LARGE = "payload_too_large"
CODE_PAYLOAD_TOO_DEEP = "payload_too_deep"
CODE_CYCLIC_PAYLOAD = "cyclic_payload"
CODE_INTEGER_TOO_LARGE = "integer_too_large"
CODE_STALE_REVISION = "stale_revision"
CODE_STALE_SESSION = "stale_session"
CODE_NON_CANONICAL_UUID = "non_canonical_uuid"
CODE_DUPLICATE_SNAPSHOT_ID = "duplicate_snapshot_id"
CODE_CLASSIFICATION_MISMATCH = "classification_mismatch"
CODE_INVALID_TEXT = "invalid_text"
CODE_SCHEMA_REF = "unsupported_schema_ref"
CODE_BAD_CONTEXT = "bad_context"

HARD_SCAN_CODES = frozenset({CODE_PAYLOAD_TOO_LARGE, CODE_PAYLOAD_TOO_DEEP,
                             CODE_CYCLIC_PAYLOAD})

SCHEMA_ERROR_CODES = {
    "required": CODE_MISSING_FIELD,
    "additionalProperties": CODE_UNKNOWN_FIELD,
    "type": CODE_BAD_TYPE,
    "const": CODE_BAD_CONST,
    "enum": CODE_BAD_ENUM,
    "pattern": CODE_BAD_PATTERN,
    "oneOf": CODE_BAD_SHAPE,
    "anyOf": CODE_BAD_SHAPE,
    "allOf": CODE_BAD_SHAPE,
    "format": CODE_BAD_PATTERN,
    "minItems": CODE_OUT_OF_BOUNDS,
    "maxItems": CODE_OUT_OF_BOUNDS,
    "minLength": CODE_OUT_OF_BOUNDS,
    "maxLength": CODE_OUT_OF_BOUNDS,
    "minimum": CODE_OUT_OF_BOUNDS,
    "maximum": CODE_OUT_OF_BOUNDS,
    "exclusiveMinimum": CODE_OUT_OF_BOUNDS,
    "exclusiveMaximum": CODE_OUT_OF_BOUNDS,
}

_PATH_SEPARATORS = ("/", "\\", "\x00")


class StrictJSONError(ValueError):
    """Raised when raw JSON text is not strict, plain, finite, bounded JSON."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


def canonical_uuid(value: Any) -> str | None:
    """Return the canonical lowercase UUID, or None if not canonical."""
    if not isinstance(value, str) or not UUID_PATTERN.match(value):
        return None
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError, TypeError):
        return None
    return value if str(parsed) == value else None


def _reject_constant(name: str) -> Any:
    raise StrictJSONError(CODE_NON_FINITE, f"non-finite number literal: {name}")


def _parse_int(text: str) -> int:
    try:
        value = int(text)
    except ValueError as exc:  # CPython's int-digit limit for huge literals
        raise StrictJSONError(CODE_INTEGER_TOO_LARGE,
                              "integer literal too large to parse") from exc
    if abs(value) > MAX_INT:
        raise StrictJSONError(CODE_INTEGER_TOO_LARGE, f"integer magnitude exceeds {MAX_INT}")
    return value


def _parse_float(text: str) -> float:
    value = float(text)  # may be +inf/-inf for 1e9999; never raises on overflow
    if not math.isfinite(value):
        raise StrictJSONError(CODE_NON_FINITE, f"non-finite number literal: {text}")
    return value


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    seen: dict[str, Any] = {}
    for key, value in pairs:
        if key in seen:
            raise StrictJSONError(CODE_DUPLICATE_KEY, f"duplicate object key: {key!r}")
        seen[key] = value
    return seen


def _utf8_size(text: str, limit: int) -> int:
    """Return the UTF-8 byte size, or raise a structured error if invalid/oversized."""
    if len(text) > limit:  # chars <= bytes for UTF-8, so this can reject early
        raise StrictJSONError(CODE_PAYLOAD_TOO_LARGE, f"payload exceeds {limit} bytes")
    try:
        encoded = text.encode("utf-8")
    except UnicodeEncodeError as exc:
        raise StrictJSONError(CODE_INVALID_TEXT, "payload is not valid UTF-8") from exc
    if len(encoded) > limit:
        raise StrictJSONError(CODE_PAYLOAD_TOO_LARGE, f"payload exceeds {limit} bytes")
    return len(encoded)


def _scan_text_structure(text: str, max_bytes: int) -> None:
    """Bounded pre-parse scan for byte size, nesting depth and item count."""
    _utf8_size(text, max_bytes)
    depth = 0
    items = 0
    in_string = False
    escaped = False
    for char in text:
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char in "[{":
            depth += 1
            items += 1
            if depth > MAX_DEPTH:
                raise StrictJSONError(CODE_PAYLOAD_TOO_DEEP, f"nesting exceeds {MAX_DEPTH}")
            if items > MAX_ITEMS:
                raise StrictJSONError(CODE_PAYLOAD_TOO_LARGE, f"items exceed {MAX_ITEMS}")
        elif char in "]}":
            depth -= 1


def strict_json_loads(text: str, max_bytes: int = ARTIFACT_MAX_BYTES) -> Any:
    """Parse JSON text rejecting non-finite/oversized/deep/duplicate content.

    Only a ``str`` is accepted; bytes/other types become a structured error so a
    caller cannot trigger an unbounded implicit decode.
    """
    if not isinstance(text, str):
        raise StrictJSONError(CODE_BAD_TYPE, "raw JSON text must be a string")
    _scan_text_structure(text, max_bytes)
    try:
        return json.loads(
            text,
            parse_constant=_reject_constant,
            parse_int=_parse_int,
            parse_float=_parse_float,
            object_pairs_hook=_reject_duplicate_keys,
        )
    except StrictJSONError:
        raise
    except RecursionError as exc:
        raise StrictJSONError(CODE_PAYLOAD_TOO_DEEP,
                              "payload nesting exceeded parser limits") from exc
    except json.JSONDecodeError as exc:
        raise StrictJSONError(CODE_INVALID_JSON, f"invalid JSON: {exc}") from exc
    except (ValueError, OverflowError) as exc:
        raise StrictJSONError(CODE_INVALID_JSON, f"invalid JSON value: {exc}") from exc


_SCALAR_BYTES = 8  # conservative accounting for numbers/bools/null/containers


def _text_cost(text: str, remaining: int) -> tuple[int | None, dict[str, str] | None]:
    """Size a string against the remaining byte budget without unbounded work."""
    if len(text) > remaining:
        return None, _error(CODE_PAYLOAD_TOO_LARGE, "payload exceeds the byte budget")
    try:
        size = len(text.encode("utf-8"))
    except UnicodeEncodeError:
        return len(text), _error(CODE_INVALID_TEXT, "string is not valid UTF-8")
    if size > remaining:
        return None, _error(CODE_PAYLOAD_TOO_LARGE, "payload exceeds the byte budget")
    return size, None


def _scan_in_memory(payload: Any, max_depth: int = MAX_DEPTH, max_items: int = MAX_ITEMS,
                    max_bytes: int = REQUEST_MAX_BYTES) -> list[dict[str, str]]:
    """Bounded scan of an in-memory payload for non-finite/cyclic/oversized data.

    Applies a byte budget *in addition* to depth and item limits, so an oversized
    in-memory payload is rejected before any expensive schema validation.
    """
    errors: list[dict[str, str]] = []
    stack: list[tuple[Any, int, bool]] = [(payload, 0, False)]
    on_path: set[int] = set()
    items = 0
    total_bytes = 0
    while stack:
        node, depth, leaving = stack.pop()
        if leaving:
            on_path.discard(id(node))
            continue
        if depth > max_depth:
            return errors + [_error(CODE_PAYLOAD_TOO_DEEP, f"nesting exceeds {max_depth}")]
        if isinstance(node, bool) or node is None:
            total_bytes += _SCALAR_BYTES
        elif isinstance(node, float):
            total_bytes += _SCALAR_BYTES
            if not math.isfinite(node):
                errors.append(_error(CODE_NON_FINITE, "non-finite number in payload"))
        elif isinstance(node, int):
            total_bytes += _SCALAR_BYTES
            if abs(node) > MAX_INT:
                errors.append(_error(CODE_INTEGER_TOO_LARGE, "integer magnitude out of range"))
        elif isinstance(node, str):
            size, error = _text_cost(node, max_bytes - total_bytes)
            if error is not None:
                if error["code"] == CODE_PAYLOAD_TOO_LARGE:
                    return errors + [error]
                errors.append(error)
                size = size or 0
            total_bytes += size or 0
        elif isinstance(node, dict):
            if id(node) in on_path:
                return errors + [_error(CODE_CYCLIC_PAYLOAD, "cyclic reference in payload")]
            on_path.add(id(node))
            stack.append((node, depth, True))
            total_bytes += _SCALAR_BYTES
            for key, value in node.items():
                items += 1
                if items > max_items:
                    return errors + [_error(CODE_PAYLOAD_TOO_LARGE, f"items exceed {max_items}")]
                if isinstance(key, str):
                    size, error = _text_cost(key, max_bytes - total_bytes)
                    if error is not None:
                        if error["code"] == CODE_PAYLOAD_TOO_LARGE:
                            return errors + [error]
                        errors.append(error)
                        size = size or 0
                    total_bytes += size or 0
                else:
                    total_bytes += _SCALAR_BYTES
                    errors.append(_error(CODE_BAD_TYPE, "object key must be a string"))
                stack.append((value, depth + 1, False))
        elif isinstance(node, (list, tuple)):
            if id(node) in on_path:
                return errors + [_error(CODE_CYCLIC_PAYLOAD, "cyclic reference in payload")]
            on_path.add(id(node))
            stack.append((node, depth, True))
            total_bytes += _SCALAR_BYTES
            for value in node:
                items += 1
                if items > max_items:
                    return errors + [_error(CODE_PAYLOAD_TOO_LARGE, f"items exceed {max_items}")]
                stack.append((value, depth + 1, False))
        else:
            errors.append(_error(CODE_BAD_TYPE, f"unsupported value type {type(node).__name__}"))
        if total_bytes > max_bytes:
            return errors + [_error(CODE_PAYLOAD_TOO_LARGE, "payload exceeds the byte budget")]
    return errors


def load_json(path: Path) -> Any:
    """Load a UTF-8 JSON artifact with strict, bounded parsing."""
    return strict_json_loads(Path(path).read_text(encoding="utf-8"),
                             max_bytes=ARTIFACT_MAX_BYTES)


def load_schema(path: Path = SCHEMA_PATH) -> dict[str, Any]:
    return load_json(path)


def load_registry(path: Path = REGISTRY_PATH) -> dict[str, Any]:
    return load_json(path)


def load_fixtures(path: Path = FIXTURES_PATH) -> dict[str, Any]:
    return load_json(path)


# ---------------------------------------------------------------------------
# Registry helpers
# ---------------------------------------------------------------------------

def registry_index(registry: Any) -> dict[str, dict[str, Any]]:
    """Tolerant index used at runtime; malformed shapes yield an empty index."""
    if not isinstance(registry, dict):
        return {}
    parameters = registry.get("parameters")
    if not isinstance(parameters, list):
        return {}
    index: dict[str, dict[str, Any]] = {}
    for entry in parameters:
        if isinstance(entry, dict) and isinstance(entry.get("id"), str):
            index[entry["id"]] = entry
    return index


def _classification(entry: dict[str, Any]) -> str:
    if entry.get("activation_block") == "commissioned_usable_lower_bound_unknown":
        return "uncommissioned"
    if entry.get("protected") or entry.get("not_applicable"):
        return "read_only"
    if entry.get("mutability") == "restart_required":
        return "restart_required"
    return "candidate_unverified"


def dependency_edges(index: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    """Flatten registry dependencies into edges source -> target."""
    edges: list[dict[str, Any]] = []
    for pid, entry in index.items():
        for dep in entry.get("dependencies", []):
            edges.append({
                "source": pid,
                "target": dep["target"],
                "kind": dep["kind"],
                "equals": dep.get("equals"),
                "tolerance": dep.get("tolerance", 1e-9),
            })
    return edges


def _is_valid_logical_id(pid: Any) -> bool:
    return isinstance(pid, str) and bool(
        re.fullmatch(r"[a-z][a-z0-9_]*(\.[a-z0-9_]+)*", pid)) and len(pid) <= 64


def _safe_relative_source_path(path: Any) -> bool:
    if not isinstance(path, str) or not path or path.startswith("/"):
        return False
    if "\\" in path or "\x00" in path:
        return False
    parts = path.split("/")
    return ".." not in parts and "" not in parts


def _value_errors(value: Any, declared_type: str) -> list[str]:
    if declared_type == "number":
        return [] if _is_finite_number(value) else ["default must be a finite number"]
    if declared_type == "integer":
        return [] if _is_strict_int(value) else ["default must be an integer"]
    if declared_type == "boolean":
        return [] if isinstance(value, bool) else ["default must be a boolean"]
    if declared_type == "string":
        return [] if isinstance(value, str) else ["default must be a string"]
    return ["unknown declared type"]


def check_registry(registry: Any) -> list[str]:
    """Return structural/safety problems in the registry (empty means clean).

    Fails fast with readable problems on malformed shapes rather than raising
    AttributeError/KeyError/TypeError.
    """
    problems: list[str] = []
    if not isinstance(registry, dict):
        return ["registry root must be an object"]
    if registry.get("contract") != CONTRACT:
        problems.append("registry contract mismatch")
    version = registry.get("version")
    if not _is_strict_int(version) or version != CONTRACT_VERSION:
        problems.append(f"registry version must be the integer {CONTRACT_VERSION}")

    capability = registry.get("capability")
    if not isinstance(capability, dict):
        problems.append("capability must be an object")
        capability = {}
    if capability.get("id") != CAPABILITY:
        problems.append("capability id mismatch")
    if capability.get("advertised") is not False:
        problems.append("capability.advertised must be strictly false")
    if not isinstance(capability.get("runtime_implemented"), bool):
        problems.append("capability.runtime_implemented must be a boolean, not authorization")

    envelope = registry.get("hard_envelope")
    if not isinstance(envelope, dict):
        problems.append("hard_envelope must be an object")
        envelope = {}
    for key, ref in (("max_linear_m_s", 0.30), ("max_angular_rad_s", 0.70)):
        entry = envelope.get(key)
        if not isinstance(entry, dict):
            problems.append(f"hard_envelope.{key} must be an object")
            continue
        if entry.get("hard_upper") != ref:
            problems.append(f"hard_envelope.{key}.hard_upper must be {ref}")
        if entry.get("commissioned_usable_lower_bound") is not None:
            problems.append(f"hard_envelope.{key} must not invent a lower bound")

    groups = registry.get("groups")
    if not isinstance(groups, list):
        problems.append("groups must be a list")
        groups = []
    group_ids: list[Any] = []
    for group in groups:
        if not isinstance(group, dict):
            problems.append("group entries must be objects")
            continue
        group_ids.append(group.get("id"))
        if not _is_valid_logical_id(group.get("id")):
            problems.append(f"invalid group id {group.get('id')!r}")
        if not group.get("label"):
            problems.append(f"group {group.get('id')!r} missing label")
    if len(set(group_ids)) != len(group_ids):
        problems.append("duplicate group id")

    parameters = registry.get("parameters")
    if not isinstance(parameters, list):
        problems.append("parameters must be a list")
        parameters = []
    index = registry_index(registry)
    if len(index) != len(parameters):
        problems.append("duplicate or invalid parameter id")

    for entry in parameters:
        if not isinstance(entry, dict):
            problems.append("parameter entries must be objects")
            continue
        pid = entry.get("id")
        if not _is_valid_logical_id(pid):
            problems.append(f"invalid logical id {pid!r}")
            continue
        if entry.get("group") not in group_ids:
            problems.append(f"{pid}: unknown group {entry.get('group')!r}")
        declared = entry.get("type")
        if declared not in ("number", "integer", "boolean", "string"):
            problems.append(f"{pid}: invalid type {declared!r}")
        else:
            for message in _value_errors(entry.get("default"), declared):
                problems.append(f"{pid}: {message}")
        for flag in ("accepts_edits", "activatable", "protected", "not_applicable",
                     "live_verified", "exceeds_effective_ceiling"):
            if not isinstance(entry.get(flag), bool):
                problems.append(f"{pid}: {flag} must be a boolean")
        if entry.get("mutability") not in ("unverified", "restart_required"):
            problems.append(f"{pid}: mutability must be unverified/restart_required")
        if entry.get("mutability_fallback") != "restart_required":
            problems.append(f"{pid}: mutability_fallback must be restart_required")
        if entry.get("live_verified") is not False:
            problems.append(f"{pid}: slice 1 must not claim live verification")
        if entry.get("activatable") is not False:
            problems.append(f"{pid}: slice 1 must be non-activatable")
        if entry.get("protected") and entry.get("accepts_edits"):
            problems.append(f"{pid}: protected parameters must not accept edits")
        if entry.get("not_applicable") and entry.get("accepts_edits"):
            problems.append(f"{pid}: not-applicable parameters must not accept edits")
        if declared == "string" and entry.get("accepts_edits"):
            problems.append(f"{pid}: string values are not an editable wire type")

        scopes = entry.get("restart_scopes")
        if not isinstance(scopes, list) or not all(isinstance(s, str) for s in scopes):
            problems.append(f"{pid}: restart_scopes must be a list of strings")

        ref = entry.get("effective_ceiling_ref")
        if ref is not None and ref not in index:
            problems.append(f"{pid}: effective_ceiling_ref {ref!r} is not a parameter")

        bounds = entry.get("bounds")
        if not isinstance(bounds, dict):
            problems.append(f"{pid}: bounds must be an object")
        else:
            if bounds.get("status") not in ("hard_source", "candidate_unverified", "unknown"):
                problems.append(f"{pid}: invalid bounds status {bounds.get('status')!r}")
            for flag in ("lower_inclusive", "upper_inclusive"):
                if not isinstance(bounds.get(flag), bool):
                    problems.append(f"{pid}: bounds.{flag} must be a boolean")
            lower, upper = bounds.get("lower"), bounds.get("upper")
            for side, value in (("lower", lower), ("upper", upper)):
                if value is not None and not _is_finite_number(value):
                    problems.append(f"{pid}: bounds.{side} must be null or a finite number")
            if _is_finite_number(lower) and _is_finite_number(upper) and lower > upper:
                problems.append(f"{pid}: bounds lower must not exceed upper")

        dependencies = entry.get("dependencies")
        if not isinstance(dependencies, list):
            problems.append(f"{pid}: dependencies must be a list")
            dependencies = []
        for dep in dependencies:
            if not isinstance(dep, dict):
                problems.append(f"{pid}: dependency entries must be objects")
                continue
            if dep.get("kind") not in ("less_than_or_equal", "sum_equals"):
                problems.append(f"{pid}: unknown dependency kind {dep.get('kind')!r}")
            if dep.get("target") not in index:
                problems.append(
                    f"{pid}: dependency target {dep.get('target')!r} is not a parameter")
            if dep.get("kind") == "sum_equals":
                if not _is_finite_number(dep.get("equals")):
                    problems.append(f"{pid}: sum_equals dependency needs a finite 'equals'")
                tolerance = dep.get("tolerance", 1e-9)
                if not _is_finite_number(tolerance) or tolerance < 0:
                    problems.append(
                        f"{pid}: sum_equals tolerance must be a finite non-negative number")

        source = entry.get("source", {})
        if not isinstance(source, dict):
            problems.append(f"{pid}: source must be an object")
        else:
            if not source.get("node") or not source.get("key"):
                problems.append(f"{pid}: incomplete source node/key")
            if not _safe_relative_source_path(source.get("file")):
                problems.append(f"{pid}: source file must be a safe relative path")

        evidence = entry.get("evidence", {})
        if (not isinstance(evidence, dict)
                or not evidence.get("kind") or not evidence.get("detail")):
            problems.append(f"{pid}: incomplete evidence")
        elif evidence.get("native_proof") != "none":
            problems.append(f"{pid}: slice 1 evidence must state native_proof=none")

    for key, entry_id in (("max_linear_m_s", "max_linear_m_s"),
                          ("max_angular_rad_s", "max_angular_rad_s")):
        entry = index.get(entry_id)
        fixed = envelope.get(key)
        fixed_upper = fixed.get("hard_upper") if isinstance(fixed, dict) else None
        if entry is None:
            problems.append(f"missing ceiling parameter {entry_id}")
        else:
            bounds = entry.get("bounds")
            if not isinstance(bounds, dict) or bounds.get("upper") != fixed_upper:
                problems.append(f"{entry_id}: bounds.upper must equal the fixed hard upper")
    return problems


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def _internal_def_name(schema: dict[str, Any],
                       schema_ref: Any) -> tuple[str | None, dict[str, str] | None]:
    """Resolve an explicit schema_ref to an internal $defs name, or reject it.

    Only ``#/$defs/<name>`` pointers that exist in this document are accepted so
    ``jsonschema`` can never be asked to fetch a remote/relative $ref.
    """
    if schema_ref is None:
        return None, None
    if not isinstance(schema_ref, str) or not schema_ref.startswith("#/$defs/"):
        return None, _error(
            CODE_SCHEMA_REF, f"schema_ref must be an internal $defs pointer: {schema_ref!r}")
    name = schema_ref[len("#/$defs/"):]
    if "/" in name or "#" in name or name not in schema.get("$defs", {}):
        return None, _error(CODE_SCHEMA_REF, f"unresolved internal schema_ref: {schema_ref!r}")
    return name, None


def _validator(schema: dict[str, Any],
               schema_ref: str | None) -> jsonschema.Draft202012Validator:
    jsonschema.Draft202012Validator.check_schema(schema)
    if schema_ref is None:
        return jsonschema.Draft202012Validator(schema)
    name, _ = _internal_def_name(schema, schema_ref)
    return jsonschema.Draft202012Validator({"$ref": f"#/$defs/{name}", "$defs": schema["$defs"]})


def _schema_error_to_code(error: jsonschema.exceptions.ValidationError) -> str:
    return SCHEMA_ERROR_CODES.get(error.validator, CODE_SCHEMA_VIOLATION)


def _error(code: str, message: str, path: str = "") -> dict[str, str]:
    return {"code": code, "message": message, "path": path}


def _structural_errors(payload: Any, schema: dict[str, Any],
                       schema_ref: str | None) -> list[dict[str, str]]:
    _, ref_error = _internal_def_name(schema, schema_ref)
    if ref_error is not None:
        return [ref_error]
    validator = _validator(schema, schema_ref)
    return [_error(_schema_error_to_code(err), err.message,
                   "/".join(str(p) for p in err.path))
            for err in validator.iter_errors(payload)]


def _has_path_injection(text: str) -> bool:
    return ".." in text or any(sep in text for sep in _PATH_SEPARATORS)


def _is_strict_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_finite_number(value: Any) -> bool:
    if isinstance(value, bool):
        return False
    if isinstance(value, int):
        return True  # magnitude is bounded at parse/scan time
    if isinstance(value, float):
        return math.isfinite(value)
    return False


def _resolved_value(pid: str, resolved: dict[str, Any], index: dict[str, dict[str, Any]]) -> Any:
    if pid in resolved:
        return resolved[pid]
    entry = index.get(pid)
    return None if entry is None else entry.get("default")


def _check_bounds(entry: dict[str, Any], value: Any, path: str) -> list[dict[str, str]]:
    errors: list[dict[str, str]] = []
    bounds = entry.get("bounds", {})
    lower = bounds.get("lower")
    upper = bounds.get("upper")
    if lower is not None:
        if bounds.get("lower_inclusive", True):
            if value < lower:
                errors.append(_error(
                    CODE_OUT_OF_BOUNDS,
                    f"{entry['id']} below candidate lower {lower}", path))
        elif value <= lower:
            errors.append(_error(CODE_OUT_OF_BOUNDS, f"{entry['id']} must be > {lower}", path))
    if upper is not None:
        if bounds.get("upper_inclusive", True):
            if value > upper:
                errors.append(_error(
                    CODE_OUT_OF_BOUNDS,
                    f"{entry['id']} above candidate upper {upper}", path))
        elif value >= upper:
            errors.append(_error(CODE_OUT_OF_BOUNDS, f"{entry['id']} must be < {upper}", path))
    return errors


def _check_single_value(entry: dict[str, Any], value: Any, path: str) -> list[dict[str, str]]:
    """Type-check an edit value against the registry type (no bool-as-number)."""
    declared = entry["type"]
    if isinstance(value, bool):
        if declared == "boolean":
            return []
        return [_error(CODE_BOOL_AS_NUMBER,
                       f"{entry['id']} expects a {declared}, got boolean", path)]
    if declared in ("number", "integer"):
        if not _is_finite_number(value):
            return [_error(CODE_BAD_TYPE, f"{entry['id']} expects a finite {declared}", path)]
        if declared == "integer" and not _is_strict_int(value):
            return [_error(CODE_BAD_TYPE, f"{entry['id']} expects an integer", path)]
        return []
    return [_error(CODE_BAD_TYPE, f"{entry['id']} expects a {declared}", path)]


def _check_change(change: Any, index: dict[str, dict[str, Any]],
                  resolved: dict[str, Any], path: str) -> list[dict[str, str]]:
    if not isinstance(change, dict):
        return [_error(CODE_BAD_TYPE, "change must be an object", path)]
    pid = change.get("id")
    if not isinstance(pid, str):
        return [_error(CODE_BAD_TYPE, "change.id must be a string", path)]
    if _has_path_injection(pid):
        return [_error(CODE_PATH_INJECTION, f"logical id contains a path fragment: {pid!r}", path)]
    entry = index.get(pid)
    if entry is None:
        return [_error(CODE_UNKNOWN_LOGICAL_ID, f"unknown logical id: {pid!r}", path)]
    if entry.get("protected"):
        return [_error(CODE_PROTECTED_PARAMETER, f"{pid} is protected and never editable", path)]
    if entry.get("not_applicable"):
        return [_error(CODE_NOT_APPLICABLE, f"{pid} is not applicable to this robot", path)]
    if not entry.get("accepts_edits"):
        return [_error(CODE_NOT_EDITABLE, f"{pid} is read-only in slice 1", path)]

    value = change.get("value")
    errors = _check_single_value(entry, value, path)
    if errors:
        return errors
    resolved[pid] = value
    errors.extend(_check_bounds(entry, value, path))
    return errors


def _base_value_errors(base_values: Any, index: dict[str, dict[str, Any]]) -> list[dict[str, str]]:
    errors: list[dict[str, str]] = []
    if base_values is None:
        return errors
    if not isinstance(base_values, dict):
        return [_error(CODE_INVALID_BASE_VALUE, "base_values must be an object")]
    for pid, value in base_values.items():
        if not isinstance(pid, str) or _has_path_injection(pid) or pid not in index:
            errors.append(_error(CODE_INVALID_BASE_VALUE, f"unknown base id {pid!r}"))
            continue
        entry = index[pid]
        declared = entry["type"]
        if declared == "boolean":
            if not isinstance(value, bool):
                errors.append(_error(CODE_INVALID_BASE_VALUE, f"{pid} base must be boolean"))
        elif declared == "string":
            if not isinstance(value, str):
                errors.append(_error(CODE_INVALID_BASE_VALUE, f"{pid} base must be a string"))
        elif declared == "integer":
            if not _is_strict_int(value):
                errors.append(_error(CODE_INVALID_BASE_VALUE, f"{pid} base must be an integer"))
        else:
            if not _is_finite_number(value):
                errors.append(_error(CODE_INVALID_BASE_VALUE,
                                     f"{pid} base must be a finite number"))
    return errors


def _dependency_errors(changed_ids: list[str], index: dict[str, dict[str, Any]],
                       resolved: dict[str, Any]) -> list[dict[str, str]]:
    """Check every dependency edge incident to a changed parameter."""
    errors: list[dict[str, str]] = []
    changed = set(changed_ids)
    if not changed:
        return errors
    for edge in dependency_edges(index):
        if edge["source"] not in changed and edge["target"] not in changed:
            continue
        left = _resolved_value(edge["source"], resolved, index)
        right = _resolved_value(edge["target"], resolved, index)
        if not _is_finite_number(left) or not _is_finite_number(right):
            errors.append(_error(
                CODE_DEPENDENCY_OPERAND,
                f"dependency {edge['source']} -> {edge['target']} has a non-numeric operand"))
            continue
        if edge["kind"] == "less_than_or_equal":
            if left > right:
                errors.append(_error(
                    CODE_DEPENDENCY_VIOLATION,
                    f"{edge['source']}={left} must be <= {edge['target']}={right}"))
        elif edge["kind"] == "sum_equals":
            total = edge["equals"]
            if abs((left + right) - total) > edge["tolerance"]:
                errors.append(_error(
                    CODE_DEPENDENCY_VIOLATION,
                    f"{edge['source']} + {edge['target']} must equal {total} "
                    f"(got {left + right})"))
    return errors


def _uuid_errors(payload: dict[str, Any]) -> list[dict[str, str]]:
    errors: list[dict[str, str]] = []
    rid = payload.get("request_id")
    if isinstance(rid, str) and canonical_uuid(rid) is None:
        errors.append(_error(CODE_NON_CANONICAL_UUID,
                             "request_id must be a canonical lowercase UUID", "request_id"))
    job = payload.get("job_id")
    if isinstance(job, str) and canonical_uuid(job) is None:
        errors.append(_error(CODE_NON_CANONICAL_UUID,
                             "job_id must be a canonical lowercase UUID", "job_id"))
    generation = payload.get("generation")
    if isinstance(generation, dict):
        session = generation.get("session_id")
        if isinstance(session, str) and canonical_uuid(session) is None:
            errors.append(_error(CODE_NON_CANONICAL_UUID,
                                 "generation.session_id must be a canonical lowercase UUID",
                                 "generation/session_id"))
    return errors


CONTEXT_FIELDS = frozenset({"current_revision", "session_id"})


def _context_errors(payload: dict[str, Any], context: Any) -> tuple[list[dict[str, str]], bool]:
    """Return (errors, performed).

    ``performed`` is True only when a *complete and valid* authoritative context
    was supplied, so contextual CAS/session checks actually ran. ``None`` keeps
    the shape-only behavior; ``{}`` or a partial/extra/invalid context is an
    error and performs no checks.
    """
    if context is None:
        return [], False
    if not isinstance(context, dict):
        return [_error(CODE_BAD_CONTEXT, "validation context must be an object")], False

    errors: list[dict[str, str]] = []
    missing = CONTEXT_FIELDS - set(context)
    extra = set(context) - CONTEXT_FIELDS
    if missing:
        errors.append(_error(CODE_BAD_CONTEXT,
                             f"context is missing required fields: {sorted(missing)}"))
    if extra:
        errors.append(_error(CODE_BAD_CONTEXT,
                             f"context has unknown fields: {sorted(extra)}"))
    if "current_revision" in context:
        current = context["current_revision"]
        if not _is_strict_int(current) or not 0 <= current <= JS_SAFE_INT:
            errors.append(_error(
                CODE_BAD_CONTEXT,
                "context.current_revision must be a non-negative safe integer"))
    if "session_id" in context and canonical_uuid(context["session_id"]) is None:
        errors.append(_error(CODE_NON_CANONICAL_UUID,
                             "context.session_id must be a canonical lowercase UUID"))
    if errors:
        return errors, False

    current = context["current_revision"]
    session = context["session_id"]
    operation = payload.get("operation")
    generation = payload.get("generation")
    generation = generation if isinstance(generation, dict) else {}
    if isinstance(generation.get("session_id"), str) and generation["session_id"] != session:
        errors.append(_error(CODE_STALE_SESSION,
                             "generation.session_id does not match the authoritative session"))
    if operation == "save" and payload.get("base_revision") != current:
        errors.append(_error(CODE_STALE_REVISION,
                             "base_revision does not match the authoritative revision"))
    if operation in ("apply", "redeploy", "restore") and payload.get(
            "expected_revision") != current:
        errors.append(_error(CODE_STALE_REVISION,
                             "expected_revision does not match the authoritative revision"))
    return errors, True


def _revision_errors(payload: dict[str, Any]) -> list[dict[str, str]]:
    """Reject non-integer revision values, then out-of-range integers.

    JSON Schema ``integer`` accepts an integral float (``0.0``) and Python
    ``bool`` is an ``int``; both are rejected here explicitly. Snapshot
    revisions may legitimately be ``null``; request revisions may not.
    """
    errors: list[dict[str, str]] = []

    def check(value: Any, where: str, allow_none: bool) -> None:
        if value is None:
            if not allow_none:
                errors.append(_error(CODE_BAD_TYPE, f"{where} must be an integer revision"))
            return
        if isinstance(value, bool) or not isinstance(value, int):
            errors.append(_error(
                CODE_BAD_TYPE,
                f"{where} must be an integer revision, got {type(value).__name__}"))
            return
        if not 0 <= value <= JS_SAFE_INT:
            errors.append(_error(CODE_OUT_OF_BOUNDS, f"{where} must be a JS-safe revision"))

    for key in ("base_revision", "expected_revision", "restore_revision"):
        if key in payload:
            check(payload[key], key, allow_none=False)
    revisions = payload.get("revisions")
    if isinstance(revisions, dict):
        for key in ("desired", "active", "last_known_good"):
            if key in revisions:
                check(revisions[key], f"revisions.{key}", allow_none=True)
    return errors


def _restore_errors(payload: dict[str, Any]) -> list[dict[str, str]]:
    """A restore must bind an immutable, reviewed target revision."""
    errors: list[dict[str, str]] = []
    restore = payload.get("restore_revision")
    expected = payload.get("expected_revision")
    if _is_strict_int(restore) and _is_strict_int(expected) and restore > expected:
        errors.append(_error(CODE_OUT_OF_BOUNDS,
                             "restore_revision must not exceed expected_revision"))
    return errors


def _semantic_errors(payload: Any, registry: dict[str, Any],
                     base_values: dict[str, Any] | None,
                     context: Any = None) -> list[dict[str, str]]:
    index = registry_index(registry)
    errors: list[dict[str, str]] = []
    if not isinstance(payload, dict):
        return errors

    operation = payload.get("operation")
    if operation is not None and operation not in REQUEST_OPERATIONS:
        errors.append(_error(CODE_UNSUPPORTED_OPERATION, f"unsupported operation: {operation!r}"))
        return errors

    revision_errors = _revision_errors(payload)
    if payload.get("kind") == "tuning_state_snapshot":
        return revision_errors + _snapshot_errors(payload, index)

    errors.extend(_uuid_errors(payload))
    errors.extend(revision_errors)
    errors.extend(_context_errors(payload, context)[0])
    if operation == "restore":
        errors.extend(_restore_errors(payload))

    if operation == "save":
        changes = payload.get("changes")
        if not isinstance(changes, list):
            return errors
        errors.extend(_base_value_errors(base_values, index))
        resolved: dict[str, Any] = {}
        if isinstance(base_values, dict):
            resolved.update(base_values)
        changed_ids: list[str] = []
        seen: set[str] = set()
        for position, change in enumerate(changes):
            path = f"changes/{position}"
            before = len(errors)
            errors.extend(_check_change(change, index, resolved, path))
            accepted = len(errors) == before
            pid = change.get("id") if isinstance(change, dict) else None
            if accepted and isinstance(pid, str) and pid in index:
                if pid in seen:
                    errors.append(_error(
                        CODE_DUPLICATE_CHANGE_ID, f"duplicate change for {pid!r}", path))
                else:
                    seen.add(pid)
                    changed_ids.append(pid)
        errors.extend(_dependency_errors(changed_ids, index, resolved))
    return errors


def _registry_value_ok(value: Any, entry: dict[str, Any]) -> bool:
    if value is None:
        return True
    declared = entry["type"]
    if declared == "number":
        return _is_finite_number(value)
    if declared == "integer":
        return _is_strict_int(value)
    if declared == "boolean":
        return isinstance(value, bool)
    if declared == "string":
        return isinstance(value, str)
    return False


def _snapshot_errors(payload: dict[str, Any],
                     index: dict[str, dict[str, Any]]) -> list[dict[str, str]]:
    errors: list[dict[str, str]] = []
    if payload.get("advertised") is not False:
        errors.append(_error(CODE_UNADVERTISED_VIOLATION, "capability must stay unadvertised"))

    runtime = payload.get("runtime")
    revisions = payload.get("revisions") if isinstance(payload.get("revisions"), dict) else {}
    if runtime == "unknown":
        if revisions.get("known") is not False:
            errors.append(_error(
                CODE_INCONSISTENT_ACTIVE_STATE, "runtime unknown requires revisions.known=false"))
        if revisions.get("active") is not None:
            errors.append(_error(
                CODE_INCONSISTENT_ACTIVE_STATE, "runtime unknown requires revisions.active=null"))
    elif runtime == "known":
        if revisions.get("known") is not True:
            errors.append(_error(
                CODE_INCONSISTENT_ACTIVE_STATE, "runtime known requires revisions.known=true"))
        if revisions.get("active") is None:
            errors.append(_error(
                CODE_INCONSISTENT_ACTIVE_STATE,
                "runtime known requires a concrete revisions.active "
                "(no partial state in slice 1)"))

    fields = payload.get("fields")
    if not isinstance(fields, list):
        return errors
    seen: set[str] = set()
    for position, field in enumerate(fields):
        path = f"fields/{position}"
        if not isinstance(field, dict):
            continue
        pid = field.get("id")
        if isinstance(pid, str):
            if pid in seen:
                errors.append(_error(CODE_DUPLICATE_SNAPSHOT_ID,
                                     f"duplicate snapshot id {pid!r}", path))
            seen.add(pid)
        entry = index.get(pid) if isinstance(pid, str) else None
        if entry is None:
            if isinstance(pid, str):
                errors.append(_error(CODE_UNKNOWN_LOGICAL_ID,
                                     f"unknown snapshot id: {pid!r}", path))
            continue

        desired = field.get("desired")
        active = field.get("active")
        active_known = field.get("active_known")
        pending = field.get("pending")

        if not _registry_value_ok(desired, entry):
            errors.append(_error(CODE_BAD_TYPE, f"{pid} desired has the wrong type", path))
        if not _registry_value_ok(active, entry):
            errors.append(_error(CODE_BAD_TYPE, f"{pid} active has the wrong type", path))
        if entry["type"] == "integer":
            if desired is not None and _is_strict_int(desired):
                errors.extend(_check_bounds(entry, desired, path))
        elif entry["type"] == "number" and desired is not None and _is_finite_number(desired):
            errors.extend(_check_bounds(entry, desired, path))
        # active is deliberately NOT bounds-checked: a real runtime readback may
        # legitimately drift outside the current policy, and that drift must be
        # representable rather than rejected as "impossible".

        expected = _classification(entry)
        if field.get("classification") != expected:
            errors.append(_error(
                CODE_CLASSIFICATION_MISMATCH,
                f"{pid} classification must be {expected!r}", path))

        if active_known is True and active is None:
            errors.append(_error(CODE_INCONSISTENT_ACTIVE_STATE,
                                 "active_known true requires a concrete active", path))
        if active_known is False and active is not None:
            errors.append(_error(CODE_INCONSISTENT_ACTIVE_STATE,
                                 "active must be null when active_known is false", path))
        # `pending` is a PERSISTED promotion status owned by the robot manifest,
        # not a value inferred from a runtime readback: a saved change awaiting
        # activation stays pending across a reboot even while the runtime (and
        # thus `active`) is unreadable. Runtime unknown therefore does NOT clear
        # a truthful pending flag. `active` stays null/unknown and
        # safe_apply.allowed stays const false.
        if active_known is True and runtime == "unknown":
            errors.append(_error(CODE_INCONSISTENT_ACTIVE_STATE,
                                 "runtime unknown cannot have known active values", path))
        if active_known is True and desired != active and pending is False:
            errors.append(_error(CODE_PENDING_FLAG_MISMATCH,
                                 "desired differs from active but pending is false", path))
    return errors


def _result(errors: list[dict[str, str]], classifications: dict[str, str] | None = None,
            validation_only: bool = True, schema_valid: bool | None = None) -> dict[str, Any]:
    deduped: list[dict[str, str]] = []
    seen: set[tuple[str, str]] = set()
    for error in errors:
        key = (error["code"], error["path"])
        if key not in seen:
            seen.add(key)
            deduped.append(error)
    classifications = classifications or {}
    return {
        "valid": not deduped,
        "schema_valid": (not deduped) if schema_valid is None else schema_valid,
        "errors": deduped,
        "error_codes": sorted({e["code"] for e in deduped}),
        "classifications": classifications,
        "classification": next(iter(classifications.values()), None),
        "activatable": False,
        "validation_only": validation_only,
        "authorization_evaluated": False,
    }


def validate_message(
    payload: Any,
    registry: dict[str, Any] | None = None,
    schema: dict[str, Any] | None = None,
    schema_ref: str | None = None,
    base_values: dict[str, Any] | None = None,
    context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Validate one contract message against schema and semantics.

    ``context`` optionally supplies the authoritative current revision and
    session (``{"current_revision": int, "session_id": uuid}``) so stale
    CAS/session tokens can be rejected. It must be complete: any partial, empty
    or extra-field context is an error and performs no contextual checks.
    ``validation_only`` is true unless a complete, valid context actually ran
    the checks; ``authorization_evaluated`` is always false.
    """
    registry = registry if registry is not None else load_registry()
    schema = schema if schema is not None else load_schema()

    # Snapshots carry no generation/CAS fields, so an authoritative context is
    # meaningless for them and must not turn this into a "contextual" result.
    if isinstance(payload, dict) and payload.get("kind") == "tuning_state_snapshot":
        context = None

    scan = _scan_in_memory(payload)
    if base_values is not None:
        scan = scan + _scan_in_memory(base_values)
    if context is not None:
        scan = scan + _scan_in_memory(context)
    if any(error["code"] in HARD_SCAN_CODES for error in scan):
        return _result(scan, validation_only=True, schema_valid=False)

    structural = _structural_errors(payload, schema, schema_ref)
    semantic = _semantic_errors(payload, registry, base_values, context)
    errors = scan + structural + semantic

    # Contextual checks only count as performed for a complete, valid context.
    _, context_performed = _context_errors(payload if isinstance(payload, dict) else {},
                                           context)

    index = registry_index(registry)
    classifications: dict[str, str] = {}
    if isinstance(payload, dict) and payload.get("operation") == "save":
        changes = payload.get("changes")
        for change in changes if isinstance(changes, list) else []:
            pid = change.get("id") if isinstance(change, dict) else None
            if isinstance(pid, str) and pid in index:
                classifications[pid] = _classification(index[pid])

    return _result(errors, classifications,
                   validation_only=not context_performed,
                   schema_valid=not structural)


def validate_strict_text(text: str, schema_ref: str | None = None,
                         registry: dict[str, Any] | None = None,
                         schema: dict[str, Any] | None = None,
                         base_values: dict[str, Any] | None = None,
                         context: dict[str, Any] | None = None) -> dict[str, Any]:
    """Strict-parse raw request text (bounded), then validate it."""
    try:
        payload = strict_json_loads(text, max_bytes=REQUEST_MAX_BYTES)
    except StrictJSONError as exc:
        return _result([_error(exc.code, exc.message)], schema_valid=False)
    return validate_message(payload, registry=registry, schema=schema, schema_ref=schema_ref,
                            base_values=base_values, context=context)


# ---------------------------------------------------------------------------
# Fixture checks
# ---------------------------------------------------------------------------

def check_fixture(case: dict[str, Any], registry: dict[str, Any],
                  schema: dict[str, Any]) -> list[str]:
    """Return human-readable problems for a single fixture case."""
    problems: list[str] = []
    name = case.get("name", "<unnamed>")
    expect = case.get("expect")
    if expect not in ("valid", "invalid"):
        return [f"{name}: expect must be valid/invalid"]
    result = validate_message(case.get("payload"), registry=registry, schema=schema,
                              schema_ref=case.get("schema_ref"),
                              base_values=case.get("base_values"),
                              context=case.get("context"))
    if expect == "valid":
        if not result["valid"]:
            problems.append(f"{name}: expected valid, got {result['error_codes']}")
        root = validate_message(case.get("payload"), registry=registry, schema=schema,
                                base_values=case.get("base_values"), context=case.get("context"))
        if not root["valid"]:
            problems.append(f"{name}: not valid against root schema {root['error_codes']}")
    else:
        reason = case.get("expected_reason")
        if not reason:
            problems.append(f"{name}: invalid case needs expected_reason")
        elif reason not in result["error_codes"]:
            problems.append(f"{name}: expected {reason}, got {result['error_codes']}")
    return problems


def check_fixtures(fixtures: dict[str, Any], registry: dict[str, Any],
                   schema: dict[str, Any]) -> list[str]:
    problems: list[str] = []
    if fixtures.get("contract") != CONTRACT:
        problems.append("fixtures contract mismatch")
    if fixtures.get("version") != CONTRACT_VERSION:
        problems.append("fixtures version mismatch")

    names: set[str] = set()
    for case in fixtures.get("cases", []):
        name = case.get("name")
        if name in names:
            problems.append(f"duplicate fixture name: {name}")
        names.add(name)
        problems.extend(check_fixture(case, registry, schema))

    for case in fixtures.get("raw_cases", []):
        name = case.get("name")
        if name in names:
            problems.append(f"duplicate fixture name: {name}")
        names.add(name)
        if case.get("expect") != "invalid":
            problems.append(f"{name}: raw cases are negative by definition")
        result = validate_strict_text(case.get("text", ""), schema_ref=case.get("schema_ref"),
                                      registry=registry, schema=schema)
        reason = case.get("expected_reason")
        if reason not in result["error_codes"]:
            problems.append(f"{name}: expected {reason}, got {result['error_codes']}")
    return problems


def run() -> int:
    """Validate schema, registry and fixtures; print a short report."""
    schema = load_schema()
    registry = load_registry()
    fixtures = load_fixtures()

    failures: list[str] = []
    try:
        jsonschema.Draft202012Validator.check_schema(schema)
    except jsonschema.exceptions.SchemaError as exc:
        failures.append(f"schema is not valid Draft 2020-12: {exc.message}")
    failures.extend(check_registry(registry))
    failures.extend(check_fixtures(fixtures, registry, schema))

    if failures:
        print("robot-tuning contract: FAIL")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print("robot-tuning contract: OK "
          f"({len(registry.get('parameters', []))} parameters, "
          f"{len(fixtures.get('cases', [])) + len(fixtures.get('raw_cases', []))} fixtures)")
    return 0


if __name__ == "__main__":
    sys.exit(run())
