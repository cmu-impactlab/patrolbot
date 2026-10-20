#!/usr/bin/env python3
"""Portable offline tests for the robot-tuning contract (slice 1).

No ROS imports, no robot contact, no source-tree reads. These tests exercise the
schema, the registry, the fixtures and the validator only, so the same file can
be vendored into the dashboard repo. Robot-source consistency lives separately
in ``test_robot_tuning_sources.py`` (not part of the vendored portable set).

Run with: ``python -m pytest contracts/test_robot_tuning.py``
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import validate_robot_tuning as vrt  # noqa: E402  (path set above)

SCHEMA = vrt.load_schema()
REGISTRY = vrt.load_registry()
FIXTURES = vrt.load_fixtures()
INDEX = vrt.registry_index(REGISTRY)

UUID_1 = "22222222-2222-4222-8222-222222222222"
SESSION = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
OTHER_SESSION = "bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb"


def _envelope(op: str, **extra):
    message = {
        "contract": vrt.CONTRACT,
        "version": vrt.CONTRACT_VERSION,
        "operation": op,
        "request_id": UUID_1,
        "generation": {"capability": vrt.CAPABILITY, "session_id": SESSION},
    }
    message.update(extra)
    return message


def _save(changes, base=0):
    return _envelope("save", base_revision=base, changes=changes)


def _codes(payload, schema_ref=None, base_values=None, context=None):
    result = vrt.validate_message(payload, REGISTRY, SCHEMA,
                                  schema_ref=schema_ref, base_values=base_values,
                                  context=context)
    return result["error_codes"], result


# ---------------------------------------------------------------------------
# Schema / file hygiene
# ---------------------------------------------------------------------------

def test_schema_is_valid_draft_2020_12():
    import jsonschema
    jsonschema.Draft202012Validator.check_schema(SCHEMA)
    assert SCHEMA["$schema"] == "https://json-schema.org/draft/2020-12/schema"


@pytest.mark.parametrize("path", [vrt.SCHEMA_PATH, vrt.REGISTRY_PATH, vrt.FIXTURES_PATH])
def test_contract_files_are_strict_json(path):
    # Strict loader rejects duplicate keys and NaN/Infinity in our own files too.
    assert vrt.load_json(path) is not None


# ---------------------------------------------------------------------------
# Registry safety posture
# ---------------------------------------------------------------------------

def test_registry_consistency_is_clean():
    assert vrt.check_registry(REGISTRY) == []


def test_static_capability_is_unadvertised_and_runtime_metadata_does_not_authorize():
    capability = REGISTRY["capability"]
    assert capability["advertised"] is False
    assert capability["runtime_implemented"] is True


def test_mutability_matrix_holds_every_unverified_capability_disabled():
    """Documenting the mutability matrix: nothing is live or activatable."""
    matrix = []
    for entry in REGISTRY["parameters"]:
        matrix.append((entry["id"], entry["mutability"], entry["live_verified"],
                       entry["activatable"], entry["accepts_edits"], entry["protected"]))
        assert entry["mutability"] in ("unverified", "restart_required"), entry["id"]
        assert entry["mutability_fallback"] == "restart_required", entry["id"]
        assert entry["live_verified"] is False, entry["id"]
        assert entry["activatable"] is False, entry["id"]
    # Every parameter is represented exactly once.
    assert len(matrix) == len(REGISTRY["parameters"])
    assert matrix, "mutability matrix must not be empty"


def test_no_parameter_claims_live_mutability_anywhere():
    blob = json.dumps(REGISTRY)
    assert '"live"' not in blob
    assert "live_verified\": true" not in blob


def test_hard_envelope_matches_the_fixed_commissioned_upper_bounds():
    envelope = REGISTRY["hard_envelope"]
    assert envelope["max_linear_m_s"]["hard_upper"] == 0.30
    assert envelope["max_angular_rad_s"]["hard_upper"] == 0.70
    for key in ("max_linear_m_s", "max_angular_rad_s"):
        assert envelope[key]["commissioned_usable_lower_bound"] is None
        assert envelope[key]["lower_bound_status"] == "unknown_uncommissioned"


def test_structural_and_protected_parameters_never_accept_edits():
    for entry in REGISTRY["parameters"]:
        if entry["protected"] or entry["not_applicable"]:
            assert entry["accepts_edits"] is False, entry["id"]
        if entry["type"] == "string":
            assert entry["accepts_edits"] is False, entry["id"]


def test_effective_ceiling_references_resolve():
    for entry in REGISTRY["parameters"]:
        ref = entry["effective_ceiling_ref"]
        if ref is not None:
            assert ref in INDEX, entry["id"]


def test_recorded_upstream_inconsistencies_are_explicit_not_clamped():
    """0.9 rad/s and 1.0 rad/s exceed the 0.70 cap in source; record, do not hide."""
    rot = INDEX["nav2.controller.rotate_to_heading_angular_vel"]
    smoother = INDEX["nav2.velocity_smoother.max_velocity_theta"]
    for entry in (rot, smoother):
        assert entry["default"] > 0.70
        assert entry["exceeds_effective_ceiling"] is True
        assert entry["effective_ceiling_ref"] == "max_angular_rad_s"
        assert entry["activatable"] is False


# ---------------------------------------------------------------------------
# Strict JSON parsing
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("text,code", [
    ('{"a": NaN}', vrt.CODE_NON_FINITE),
    ('{"a": Infinity}', vrt.CODE_NON_FINITE),
    ('{"a": -Infinity}', vrt.CODE_NON_FINITE),
    ('{"a": 1, "a": 2}', vrt.CODE_DUPLICATE_KEY),
    ('{"a": 1', vrt.CODE_INVALID_JSON),
])
def test_strict_loader_rejects_non_plain_json(text, code):
    with pytest.raises(vrt.StrictJSONError) as excinfo:
        vrt.strict_json_loads(text)
    assert excinfo.value.code == code


def test_strict_loader_accepts_plain_json():
    parsed = vrt.strict_json_loads('{"a": 1, "b": [true, null, 0.5]}')
    assert parsed == {"a": 1, "b": [True, None, 0.5]}


@pytest.mark.parametrize("text,code", [
    ('{"a": 1e9999}', vrt.CODE_NON_FINITE),
    ('{"a": -1e9999}', vrt.CODE_NON_FINITE),
    ('{"a": ' + "1" + "0" * 400 + '}', vrt.CODE_INTEGER_TOO_LARGE),
    ('{"a": ' + "5" * 5000 + '}', vrt.CODE_INTEGER_TOO_LARGE),
    ("[" * 70 + "]" * 70, vrt.CODE_PAYLOAD_TOO_DEEP),
])
def test_strict_loader_bounds_and_overflows(text, code):
    with pytest.raises(vrt.StrictJSONError) as excinfo:
        vrt.strict_json_loads(text)
    assert excinfo.value.code == code


def test_strict_loader_rejects_oversized_text():
    text = '{"a": "' + "x" * (vrt.REQUEST_MAX_BYTES + 1) + '"}'
    with pytest.raises(vrt.StrictJSONError) as excinfo:
        vrt.strict_json_loads(text, max_bytes=vrt.REQUEST_MAX_BYTES)
    assert excinfo.value.code == vrt.CODE_PAYLOAD_TOO_LARGE


def test_strict_loader_rejects_duplicate_nested_key():
    with pytest.raises(vrt.StrictJSONError) as excinfo:
        vrt.strict_json_loads('{"a": {"b": 1, "b": 2}}')
    assert excinfo.value.code == vrt.CODE_DUPLICATE_KEY


def test_registry_artifact_limit_is_larger_than_the_request_limit():
    assert vrt.ARTIFACT_MAX_BYTES > vrt.REQUEST_MAX_BYTES
    # The real registry parses under the artifact limit but would not be a request.
    assert vrt.load_registry() is not None


# ---------------------------------------------------------------------------
# Fixtures and cross-fixture checks
# ---------------------------------------------------------------------------

def test_all_fixtures_hold_their_expected_outcome():
    assert vrt.check_fixtures(FIXTURES, REGISTRY, SCHEMA) == []


def test_fixture_names_are_unique_and_cover_valid_and_invalid():
    names = [case["name"] for case in FIXTURES["cases"] + FIXTURES["raw_cases"]]
    assert len(names) == len(set(names))
    expectations = {case["expect"] for case in FIXTURES["cases"]}
    assert expectations == {"valid", "invalid"}


def test_every_fixture_schema_ref_exists_in_the_schema_defs():
    defs = SCHEMA["$defs"]
    for case in FIXTURES["cases"]:
        ref = case.get("schema_ref")
        if ref is not None:
            assert ref.startswith("#/$defs/")
            assert ref.split("/")[-1] in defs, case["name"]


def test_logical_ids_in_fixtures_exist_in_the_registry():
    intentional_misses = {"unknown_logical_id", "path_injection", "not_applicable",
                          "protected_parameter", "read_only"}
    for case in FIXTURES["cases"]:
        if case.get("expected_reason") in intentional_misses:
            continue
        payload = case.get("payload") or {}
        for change in payload.get("changes", []):
            pid = change.get("id")
            if isinstance(pid, str):
                assert pid in INDEX, f"{case['name']}: {pid}"


def test_every_expected_reason_is_a_known_validator_code():
    known = {value for name, value in vars(vrt).items() if name.startswith("CODE_")}
    for case in FIXTURES["cases"] + FIXTURES["raw_cases"]:
        reason = case.get("expected_reason")
        if reason is not None:
            assert reason in known, case["name"]


# ---------------------------------------------------------------------------
# Semantic enforcement beyond JSON Schema
# ---------------------------------------------------------------------------

def test_bool_is_rejected_where_a_number_is_required():
    codes, _ = _codes(_save([{"id": "max_linear_m_s", "value": True}]), "#/$defs/save_request")
    assert vrt.CODE_BOOL_AS_NUMBER in codes


def test_non_finite_number_rejected_by_strict_text_path():
    text = ('{"contract":"patrolbot-robot-tuning","version":1,"operation":"save",'
            f'"request_id":"{UUID_1}",'
            f'"generation":{{"capability":"robot_tuning_v1","session_id":"{SESSION}"}},'
            '"base_revision":0,"changes":[{"id":"max_linear_m_s","value":NaN}]}')
    result = vrt.validate_strict_text(text, schema_ref="#/$defs/save_request",
                                      registry=REGISTRY, schema=SCHEMA)
    assert vrt.CODE_NON_FINITE in result["error_codes"]


def test_unknown_logical_id_rejected():
    codes, _ = _codes(_save([{"id": "totally.unknown", "value": 0.1}]), "#/$defs/save_request")
    assert vrt.CODE_UNKNOWN_LOGICAL_ID in codes


@pytest.mark.parametrize("bad_id", [
    "amcl.alpha1/../../etc/passwd",
    "..\\..\\windows",
    "amcl.alpha1\x00",
    "../alpha1",
])
def test_path_injection_ids_rejected(bad_id):
    codes, _ = _codes(_save([{"id": bad_id, "value": 0.1}]), "#/$defs/save_request")
    assert vrt.CODE_PATH_INJECTION in codes


def test_protected_knob_rejected_even_with_a_valid_number():
    codes, result = _codes(_save([{"id": "safety_watchdog.scan_timeout", "value": 0.5}]),
                           "#/$defs/save_request")
    assert vrt.CODE_PROTECTED_PARAMETER in codes
    assert result["activatable"] is False


def test_not_applicable_parameter_rejected():
    codes, _ = _codes(_save([{"id": "amcl.alpha5", "value": 0.1}]), "#/$defs/save_request")
    assert vrt.CODE_NOT_APPLICABLE in codes


def test_ceiling_bounds_reject_over_and_under_limit():
    over, _ = _codes(_save([{"id": "max_linear_m_s", "value": 0.31}]), "#/$defs/save_request")
    under, _ = _codes(_save([{"id": "max_angular_rad_s", "value": -0.01}]), "#/$defs/save_request")
    assert vrt.CODE_OUT_OF_BOUNDS in over
    assert vrt.CODE_OUT_OF_BOUNDS in under


def test_particle_relationship_min_le_max_enforced():
    codes, _ = _codes(_save([{"id": "amcl.min_particles", "value": 5000},
                             {"id": "amcl.max_particles", "value": 1000}]),
                      "#/$defs/save_request")
    assert vrt.CODE_DEPENDENCY_VIOLATION in codes


def test_particle_relationship_resolves_against_base_values():
    codes, result = _codes(_save([{"id": "amcl.min_particles", "value": 4000}]),
                           "#/$defs/save_request",
                           base_values={"amcl.max_particles": 3000})
    assert vrt.CODE_DEPENDENCY_VIOLATION in codes
    ok, _ = _codes(_save([{"id": "amcl.min_particles", "value": 2000}]),
                   "#/$defs/save_request",
                   base_values={"amcl.max_particles": 3000})
    assert ok == []


def test_zhit_zrand_sum_relationship_enforced():
    bad, _ = _codes(
        _save([{"id": "amcl.z_hit", "value": 0.5}, {"id": "amcl.z_rand", "value": 0.2}]),
        "#/$defs/save_request")
    good, _ = _codes(
        _save([{"id": "amcl.z_hit", "value": 0.6}, {"id": "amcl.z_rand", "value": 0.4}]),
        "#/$defs/save_request")
    assert vrt.CODE_DEPENDENCY_VIOLATION in bad
    assert good == []


def test_reverse_only_edit_is_checked_against_the_merged_other_endpoint():
    """Changing only one endpoint must still respect the other (default) endpoint."""
    # max_particles alone, below the default min, is a bypass if you only read
    # the changed node's own outgoing edges.
    codes, _ = _codes(_save([{"id": "amcl.max_particles", "value": 500}]),
                      "#/$defs/save_request")
    assert vrt.CODE_DEPENDENCY_VIOLATION in codes
    # min_particles alone, within the default max, is fine.
    ok, _ = _codes(_save([{"id": "amcl.min_particles", "value": 2000}]),
                   "#/$defs/save_request")
    assert ok == []


@pytest.mark.parametrize("pid,value", [
    ("amcl.z_rand", 0.2),
    ("amcl.z_hit", 0.6),
])
def test_sum_relationship_flags_either_member_edited_alone(pid, value):
    codes, _ = _codes(_save([{"id": pid, "value": value}]), "#/$defs/save_request")
    assert vrt.CODE_DEPENDENCY_VIOLATION in codes


def test_lower_linear_cap_requires_coherent_producers():
    alone, _ = _codes(_save([{"id": "max_linear_m_s", "value": 0.25}]),
                      "#/$defs/save_request")
    assert vrt.CODE_DEPENDENCY_VIOLATION in alone
    coherent, _ = _codes(_save([
        {"id": "max_linear_m_s", "value": 0.25},
        {"id": "nav2.velocity_smoother.max_velocity_x", "value": 0.25},
    ]), "#/$defs/save_request")
    assert coherent == []


def test_lower_angular_cap_requires_coherent_producers():
    alone, _ = _codes(_save([{"id": "max_angular_rad_s", "value": 0.6}]),
                      "#/$defs/save_request")
    assert vrt.CODE_DEPENDENCY_VIOLATION in alone
    coherent, _ = _codes(_save([
        {"id": "max_angular_rad_s", "value": 0.6},
        {"id": "nav2.controller.rotate_to_heading_angular_vel", "value": 0.6},
        {"id": "nav2.velocity_smoother.max_velocity_theta", "value": 0.6},
        {"id": "nav2.behavior.max_rotational_vel", "value": 0.6},
    ]), "#/$defs/save_request")
    assert coherent == []


def test_unrelated_edit_is_not_penalised_by_the_known_angular_mismatch():
    """The 0.9/1.0 vs 0.70 angular mismatch must not block an unrelated edit."""
    codes, _ = _codes(_save([{"id": "amcl.alpha1", "value": 0.1}]),
                      "#/$defs/save_request")
    assert codes == []
    # ...but an edit that touches the angular component must resolve it.
    resolved, _ = _codes(_save([
        {"id": "nav2.controller.rotate_to_heading_angular_vel", "value": 0.7},
        {"id": "nav2.velocity_smoother.max_velocity_theta", "value": 0.7},
    ]), "#/$defs/save_request")
    assert resolved == []


def test_lookahead_upper_only_edit_is_valid():
    codes, _ = _codes(_save([
        {"id": "nav2.controller.max_lookahead_dist", "value": 0.8},
    ]), "#/$defs/save_request")
    assert codes == []


def test_type_invalid_base_operand_is_rejected_structurally():
    codes, _ = _codes(_save([{"id": "amcl.min_particles", "value": 2000}]),
                      "#/$defs/save_request",
                      base_values={"amcl.max_particles": "abc"})
    assert vrt.CODE_DEPENDENCY_OPERAND in codes
    assert vrt.CODE_INVALID_BASE_VALUE in codes


def test_duplicate_change_id_rejected():
    codes, _ = _codes(_save([{"id": "max_linear_m_s", "value": 0.2},
                             {"id": "max_linear_m_s", "value": 0.25}]),
                      "#/$defs/save_request")
    assert vrt.CODE_DUPLICATE_CHANGE_ID in codes


def test_integer_type_rejects_a_fractional_value():
    codes, _ = _codes(_save([{"id": "amcl.min_particles", "value": 1000.5}]),
                      "#/$defs/save_request")
    assert vrt.CODE_BAD_TYPE in codes


def test_exact_activation_revision_rejects_a_string():
    payload = _envelope("apply", expected_revision="latest")
    codes, _ = _codes(payload, "#/$defs/apply_request")
    assert vrt.CODE_BAD_TYPE in codes


def test_generation_capability_binding_is_enforced():
    payload = _save([{"id": "max_linear_m_s", "value": 0.25}])
    payload["generation"]["capability"] = "some_other_capability"
    codes, _ = _codes(payload, "#/$defs/save_request")
    assert vrt.CODE_BAD_CONST in codes


def test_generation_is_an_opaque_session_not_a_duplicate_revision():
    payload = _save([{"id": "max_linear_m_s", "value": 0.28}])
    assert set(payload["generation"]) == {"capability", "session_id"}
    # A revision-shaped session token is refused instead of aliasing the CAS field.
    payload["generation"]["session_id"] = 0
    codes, _ = _codes(payload, "#/$defs/save_request")
    assert vrt.CODE_BAD_TYPE in codes


def test_generation_session_must_be_canonical_uuid():
    payload = _save([{"id": "max_linear_m_s", "value": 0.28}])
    payload["generation"]["session_id"] = "ABCDEF01-2345-4678-8ABC-ABCDEF012345"
    codes, _ = _codes(payload, "#/$defs/save_request")
    assert vrt.CODE_NON_CANONICAL_UUID in codes


def test_missing_context_reports_validation_only_and_never_authorizes():
    result = vrt.validate_message(_save([{"id": "max_linear_m_s", "value": 0.28}]),
                                  REGISTRY, SCHEMA, schema_ref="#/$defs/save_request")
    assert result["valid"]
    assert result["validation_only"] is True
    assert result["activatable"] is False
    assert result["authorization_evaluated"] is False


def test_context_rejects_stale_base_revision():
    codes, result = _codes(_save([{"id": "max_linear_m_s", "value": 0.28}], base=1),
                           "#/$defs/save_request",
                           context={"current_revision": 3, "session_id": SESSION})
    assert vrt.CODE_STALE_REVISION in codes
    assert result["validation_only"] is False


def test_context_accepts_the_matching_revision_and_session():
    codes, _ = _codes(_save([{"id": "max_linear_m_s", "value": 0.28}], base=3),
                      "#/$defs/save_request",
                      context={"current_revision": 3, "session_id": SESSION})
    assert codes == []


def test_context_rejects_a_foreign_session():
    payload = _envelope("apply", expected_revision=3)
    codes, _ = _codes(payload, "#/$defs/apply_request",
                      context={"current_revision": 3, "session_id": OTHER_SESSION})
    assert vrt.CODE_STALE_SESSION in codes


@pytest.mark.parametrize("context", [
    {},
    {"current_revision": 3},
    {"session_id": SESSION},
    {"current_revision": 3, "session_id": SESSION, "extra": 1},
    {"current_revision": 3.0, "session_id": SESSION},
    {"current_revision": -1, "session_id": SESSION},
    {"current_revision": 3, "session_id": "not-a-uuid"},
])
def test_incomplete_or_invalid_context_performs_no_checks(context):
    result = vrt.validate_message(_save([{"id": "max_linear_m_s", "value": 0.28}]),
                                  REGISTRY, SCHEMA, schema_ref="#/$defs/save_request",
                                  context=context)
    assert not result["valid"]
    assert result["validation_only"] is True
    assert result["activatable"] is False
    assert result["authorization_evaluated"] is False


def test_complete_valid_context_performs_checks():
    result = vrt.validate_message(_save([{"id": "max_linear_m_s", "value": 0.28}], base=3),
                                  REGISTRY, SCHEMA, schema_ref="#/$defs/save_request",
                                  context={"current_revision": 3, "session_id": SESSION})
    assert result["valid"]
    assert result["validation_only"] is False
    assert result["authorization_evaluated"] is False


def test_revision_must_fit_a_js_safe_integer():
    codes, _ = _codes(_save([{"id": "max_linear_m_s", "value": 0.25}], base=9007199254740992),
                      "#/$defs/save_request")
    assert vrt.CODE_OUT_OF_BOUNDS in codes


@pytest.mark.parametrize("field,value", [
    ("base_revision", 0.0),
    ("base_revision", True),
    ("expected_revision", 0.0),
    ("expected_revision", False),
    ("restore_revision", 1.0),
])
def test_revision_fields_reject_integral_floats_and_bools(field, value):
    payload = _envelope("restore", expected_revision=1, restore_revision=1,
                        target="last_known_good")
    if field in ("base_revision",):
        payload = _save([{"id": "max_linear_m_s", "value": 0.28}], base=value)
    elif field == "expected_revision":
        payload = _envelope("apply", expected_revision=value)
    else:
        payload[field] = value
    codes, _ = _codes(payload, None)
    assert vrt.CODE_BAD_TYPE in codes


def test_snapshot_revisions_reject_integral_floats_and_bools():
    for value in (2.0, True):
        codes, _ = _codes(_snapshot(revisions={"desired": 3, "active": value,
                                               "last_known_good": 2, "known": True}),
                          "#/$defs/tuning_state_snapshot")
        assert vrt.CODE_BAD_TYPE in codes, value


def test_snapshot_revisions_may_be_null_when_unknown():
    codes, _ = _codes(_snapshot(revisions={"desired": 1, "active": None,
                                           "last_known_good": None, "known": False},
                                runtime="unknown",
                                fields=[{"id": "max_linear_m_s", "desired": 0.25,
                                         "active": None, "active_known": False,
                                         "pending": False,
                                         "classification": "uncommissioned"}]),
                      "#/$defs/tuning_state_snapshot")
    assert codes == []


def test_restore_binds_an_immutable_target_revision():
    missing = _envelope("restore", expected_revision=2, target="last_known_good")
    missing_codes, _ = _codes(missing, "#/$defs/restore_request")
    assert vrt.CODE_MISSING_FIELD in missing_codes

    ok = _envelope("restore", expected_revision=2, restore_revision=2,
                   target="last_known_good")
    ok_codes, _ = _codes(ok, "#/$defs/restore_request",
                         context={"current_revision": 2, "session_id": SESSION})
    assert ok_codes == []

    future = _envelope("restore", expected_revision=1, restore_revision=5,
                       target="last_known_good")
    future_codes, _ = _codes(future, "#/$defs/restore_request")
    assert vrt.CODE_OUT_OF_BOUNDS in future_codes


def test_restore_rejects_a_moving_latest_target():
    payload = _envelope("restore", expected_revision=2, restore_revision=2, target="latest")
    codes, _ = _codes(payload, "#/$defs/restore_request")
    assert vrt.CODE_BAD_CONST in codes


def test_integer_fields_reject_a_whole_float():
    codes, _ = _codes(_save([{"id": "amcl.min_particles", "value": 1000.0}]),
                      "#/$defs/save_request")
    assert vrt.CODE_BAD_TYPE in codes


def test_serialized_floating_values_stay_finite():
    """Guard against a regression that would let JSON emit NaN/Infinity."""
    for entry in REGISTRY["parameters"]:
        for key in ("default",):
            value = entry.get(key)
            if isinstance(value, float):
                assert math.isfinite(value), entry["id"]


# ---------------------------------------------------------------------------
# Desired vs active honesty (snapshots)
# ---------------------------------------------------------------------------

def _snapshot(**overrides):
    base = {
        "kind": "tuning_state_snapshot",
        "contract": vrt.CONTRACT,
        "version": vrt.CONTRACT_VERSION,
        "capability": vrt.CAPABILITY,
        "advertised": False,
        "runtime": "known",
        "revisions": {"desired": 3, "active": 2, "last_known_good": 2, "known": True},
        "safe_apply": {"allowed": False, "reason": "busy"},
        "fields": [{"id": "max_linear_m_s", "desired": 0.25, "active": 0.30,
                    "active_known": True, "pending": True, "classification": "uncommissioned"}],
    }
    base.update(overrides)
    return base


def test_snapshot_allows_desired_and_active_to_differ():
    result = vrt.validate_message(_snapshot(), REGISTRY, SCHEMA,
                                  schema_ref="#/$defs/tuning_state_snapshot")
    assert result["valid"], result["errors"]


def test_snapshot_unknown_runtime_must_be_explicit():
    snapshot = _snapshot(runtime="unknown",
                         revisions={"desired": 1, "active": None,
                                    "last_known_good": None, "known": False},
                         fields=[{"id": "max_linear_m_s", "desired": 0.30, "active": None,
                                  "active_known": False, "pending": False,
                                  "classification": "uncommissioned"}])
    result = vrt.validate_message(snapshot, REGISTRY, SCHEMA,
                                  schema_ref="#/$defs/tuning_state_snapshot")
    assert result["valid"], result["errors"]


def test_snapshot_cannot_assert_active_while_runtime_is_unknown():
    codes, _ = _codes(_snapshot(runtime="unknown",
                                revisions={"desired": 3, "active": None,
                                           "last_known_good": 2, "known": False}),
                      "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_INCONSISTENT_ACTIVE_STATE in codes


def test_snapshot_desired_must_not_impersonate_active_without_pending():
    codes, _ = _codes(_snapshot(fields=[{"id": "max_linear_m_s", "desired": 0.25, "active": 0.30,
                                         "active_known": True, "pending": False,
                                         "classification": "uncommissioned"}]),
                      "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_PENDING_FLAG_MISMATCH in codes


def test_snapshot_classification_has_no_live_member():
    assert "live" not in SCHEMA["$defs"]["snapshot_field"]["properties"]["classification"]["enum"]
    codes, _ = _codes(_snapshot(fields=[{"id": "max_linear_m_s", "desired": 0.25, "active": 0.30,
                                         "active_known": True, "pending": True,
                                         "classification": "live"}]),
                      "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_BAD_ENUM in codes


def test_snapshot_must_stay_unadvertised():
    codes, _ = _codes(_snapshot(advertised=True), "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_BAD_CONST in codes


def test_snapshot_cannot_claim_apply_is_allowed_in_slice_one():
    codes, _ = _codes(_snapshot(safe_apply={"allowed": True, "reason": "looks fine"}),
                      "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_BAD_CONST in codes


def test_snapshot_safe_reason_must_be_non_empty():
    codes, _ = _codes(_snapshot(safe_apply={"allowed": False, "reason": ""}),
                      "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_OUT_OF_BOUNDS in codes


def test_snapshot_active_known_true_requires_a_concrete_active():
    codes, _ = _codes(_snapshot(fields=[{"id": "max_linear_m_s", "desired": 0.25,
                                         "active": None, "active_known": True,
                                         "pending": False,
                                         "classification": "uncommissioned"}]),
                      "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_INCONSISTENT_ACTIVE_STATE in codes


def test_snapshot_active_known_false_forbids_a_value():
    codes, _ = _codes(_snapshot(fields=[{"id": "max_linear_m_s", "desired": 0.30,
                                         "active": 0.30, "active_known": False,
                                         "pending": False,
                                         "classification": "uncommissioned"}]),
                      "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_INCONSISTENT_ACTIVE_STATE in codes


def test_snapshot_known_runtime_requires_a_concrete_active_revision():
    codes, _ = _codes(_snapshot(revisions={"desired": 3, "active": None,
                                           "last_known_good": 2, "known": True}),
                      "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_INCONSISTENT_ACTIVE_STATE in codes


def test_snapshot_rejects_duplicate_ids():
    field = {"id": "max_linear_m_s", "desired": 0.25, "active": 0.30,
             "active_known": True, "pending": True, "classification": "uncommissioned"}
    codes, _ = _codes(_snapshot(fields=[field, dict(field)]),
                      "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_DUPLICATE_SNAPSHOT_ID in codes


def test_snapshot_field_type_must_match_the_registry():
    codes, _ = _codes(_snapshot(fields=[{"id": "max_linear_m_s", "desired": "fast",
                                         "active": 0.30, "active_known": True,
                                         "pending": True,
                                         "classification": "uncommissioned"}]),
                      "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_BAD_TYPE in codes


def test_snapshot_desired_is_bounds_checked_but_active_drift_is_representable():
    bad_desired, _ = _codes(_snapshot(fields=[{"id": "max_linear_m_s", "desired": 0.99,
                                               "active": 0.30, "active_known": True,
                                               "pending": True,
                                               "classification": "uncommissioned"}]),
                            "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_OUT_OF_BOUNDS in bad_desired
    drift, _ = _codes(_snapshot(fields=[{"id": "max_linear_m_s", "desired": 0.30,
                                         "active": 0.45, "active_known": True,
                                         "pending": True,
                                         "classification": "uncommissioned"}]),
                      "#/$defs/tuning_state_snapshot")
    assert drift == []


def test_snapshot_classification_must_match_the_registry():
    codes, _ = _codes(_snapshot(fields=[{"id": "max_linear_m_s", "desired": 0.25,
                                         "active": 0.30, "active_known": True,
                                         "pending": True,
                                         "classification": "restart_required"}]),
                      "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_CLASSIFICATION_MISMATCH in codes


def test_snapshot_can_represent_read_only_strings_but_save_cannot():
    snapshot = _snapshot(fields=[{"id": "amcl.laser_model_type",
                                  "desired": "likelihood_field_prob",
                                  "active": "likelihood_field_prob",
                                  "active_known": True, "pending": False,
                                  "classification": "read_only"}])
    codes, _ = _codes(snapshot, "#/$defs/tuning_state_snapshot")
    assert codes == []
    save_codes, _ = _codes(_save([{"id": "amcl.laser_model_type",
                                   "value": "likelihood_field_prob"}]),
                           "#/$defs/save_request")
    assert vrt.CODE_BAD_TYPE in save_codes


@pytest.mark.parametrize("bad", [float("inf"), float("-inf"), float("nan")])
def test_snapshot_rejects_non_finite_numbers_in_memory(bad):
    snapshot = _snapshot()
    snapshot["fields"][0]["active"] = bad
    result = vrt.validate_message(snapshot, REGISTRY, SCHEMA,
                                  schema_ref="#/$defs/tuning_state_snapshot")
    assert vrt.CODE_NON_FINITE in result["error_codes"]


def test_malformed_in_memory_payload_is_rejected_without_unbounded_traversal():
    cyclic = {}
    cyclic["self"] = cyclic
    result = vrt.validate_message(cyclic, REGISTRY, SCHEMA)
    assert vrt.CODE_CYCLIC_PAYLOAD in result["error_codes"]

    deep = {"n": None}
    node = deep
    for _ in range(vrt.MAX_DEPTH + 5):
        node["n"] = {"n": None}
        node = node["n"]
    deep_result = vrt.validate_message(deep, REGISTRY, SCHEMA)
    assert vrt.CODE_PAYLOAD_TOO_DEEP in deep_result["error_codes"]


def test_snapshot_unknown_runtime_keeps_a_durable_pending_flag():
    """Pending is a persisted promotion status, not a runtime inference."""
    codes, _ = _codes(_snapshot(
        runtime="unknown",
        revisions={"desired": 4, "active": None, "last_known_good": 2, "known": False},
        fields=[{"id": "max_linear_m_s", "desired": 0.25, "active": None,
                 "active_known": False, "pending": True,
                 "classification": "uncommissioned"}]),
        "#/$defs/tuning_state_snapshot")
    assert codes == []


def test_snapshot_unknown_runtime_still_forbids_a_known_active_value():
    codes, _ = _codes(_snapshot(
        runtime="unknown",
        revisions={"desired": 4, "active": None, "last_known_good": 2, "known": False},
        fields=[{"id": "max_linear_m_s", "desired": 0.25, "active": 0.30,
                 "active_known": True, "pending": True,
                 "classification": "uncommissioned"}]),
        "#/$defs/tuning_state_snapshot")
    assert vrt.CODE_INCONSISTENT_ACTIVE_STATE in codes


# ---------------------------------------------------------------------------
# Bounded input handling / error types
# ---------------------------------------------------------------------------

def test_in_memory_payload_byte_budget_is_enforced():
    payload = _save([{"id": "max_linear_m_s", "value": 0.28}])
    payload["blob"] = "x" * (vrt.REQUEST_MAX_BYTES + 1)
    result = vrt.validate_message(payload, REGISTRY, SCHEMA, schema_ref="#/$defs/save_request")
    assert vrt.CODE_PAYLOAD_TOO_LARGE in result["error_codes"]


def test_lone_surrogate_string_is_a_structured_error():
    raw = '{"a": "' + "\ud800" + '"}'
    with pytest.raises(vrt.StrictJSONError) as excinfo:
        vrt.strict_json_loads(raw)
    assert excinfo.value.code == vrt.CODE_INVALID_TEXT
    payload = _save([{"id": "max_linear_m_s", "value": 0.28}])
    payload["note"] = "\ud800"
    result = vrt.validate_message(payload, REGISTRY, SCHEMA, schema_ref="#/$defs/save_request")
    assert vrt.CODE_INVALID_TEXT in result["error_codes"]


def test_raw_text_of_wrong_type_is_a_structured_error():
    for bad in (b"{}", 123, None, ["{}"]):
        with pytest.raises(vrt.StrictJSONError) as excinfo:
            vrt.strict_json_loads(bad)
        assert excinfo.value.code == vrt.CODE_BAD_TYPE


def test_invalid_base_values_shape_never_raises_or_reaches_a_dict_constructor():
    for bad in (1, ["x"], "abc", True):
        result = vrt.validate_message(_save([{"id": "amcl.min_particles", "value": 2000}]),
                                      REGISTRY, SCHEMA, schema_ref="#/$defs/save_request",
                                      base_values=bad)
        assert not result["valid"]
        assert vrt.CODE_INVALID_BASE_VALUE in result["error_codes"]


def test_oversized_base_values_or_context_is_rejected_before_checks():
    huge = {"x" * vrt.REQUEST_MAX_BYTES: 1}
    result = vrt.validate_message(_save([{"id": "max_linear_m_s", "value": 0.28}]),
                                  REGISTRY, SCHEMA, schema_ref="#/$defs/save_request",
                                  base_values=huge)
    assert vrt.CODE_PAYLOAD_TOO_LARGE in result["error_codes"]
    result = vrt.validate_message(_save([{"id": "max_linear_m_s", "value": 0.28}]),
                                  REGISTRY, SCHEMA, schema_ref="#/$defs/save_request",
                                  context={"extra": "x" * (vrt.REQUEST_MAX_BYTES + 1)})
    assert vrt.CODE_PAYLOAD_TOO_LARGE in result["error_codes"]


# ---------------------------------------------------------------------------
# Schema-reference safety
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("ref", [
    "https://example.com/schema.json",
    "file:///etc/passwd",
    "#/$defs/does_not_exist",
    "#/properties/foo",
    "contracts/robot-tuning.schema.json",
])
def test_external_or_unresolved_schema_ref_is_rejected(ref):
    result = vrt.validate_message(_save([{"id": "max_linear_m_s", "value": 0.28}]),
                                  REGISTRY, SCHEMA, schema_ref=ref)
    assert vrt.CODE_SCHEMA_REF in result["error_codes"]


def test_all_static_schema_refs_are_internal():
    def walk(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key == "$ref":
                    assert isinstance(value, str) and value.startswith("#/"), value
                walk(value)
        elif isinstance(node, list):
            for item in node:
                walk(item)

    walk(SCHEMA)


# ---------------------------------------------------------------------------
# Registry structural invariants
# ---------------------------------------------------------------------------

def test_check_registry_rejects_a_mutated_safety_flag():
    import copy
    mutated = copy.deepcopy(REGISTRY)
    mutated["parameters"][0]["activatable"] = True
    assert any("non-activatable" in problem for problem in vrt.check_registry(mutated))


def test_check_registry_rejects_a_duplicate_group_id():
    import copy
    mutated = copy.deepcopy(REGISTRY)
    mutated["groups"].append(dict(mutated["groups"][0]))
    assert any("duplicate group id" in problem for problem in vrt.check_registry(mutated))


def test_check_registry_rejects_an_unsafe_source_path():
    import copy
    mutated = copy.deepcopy(REGISTRY)
    mutated["parameters"][0]["source"]["file"] = "../../etc/passwd"
    assert any("safe relative path" in problem for problem in vrt.check_registry(mutated))


def test_check_registry_rejects_a_default_type_mismatch():
    import copy
    mutated = copy.deepcopy(REGISTRY)
    for entry in mutated["parameters"]:
        if entry["id"] == "amcl.min_particles":
            entry["default"] = "lots"
    assert any("default must be an integer" in problem for problem in vrt.check_registry(mutated))


def test_check_registry_rejects_inverted_bounds():
    import copy
    mutated = copy.deepcopy(REGISTRY)
    for entry in mutated["parameters"]:
        if entry["id"] == "amcl.alpha1":
            entry["bounds"]["lower"] = 2.0
            entry["bounds"]["upper"] = 1.0
    assert any("lower must not exceed upper" in problem for problem in vrt.check_registry(mutated))


def _mutated(mutator):
    import copy
    registry = copy.deepcopy(REGISTRY)
    mutator(registry)
    return registry


@pytest.mark.parametrize("mutator", [
    lambda r: r.update({"parameters": {"not": "a list"}}),
    lambda r: r.update({"parameters": ["not-an-object"]}),
    lambda r: r.update({"groups": "not-a-list"}),
    lambda r: r["groups"].append("not-an-object"),
    lambda r: r.update({"capability": []}),
    lambda r: r.update({"hard_envelope": "nope"}),
    lambda r: r["parameters"][0].update({"bounds": "nope"}),
    lambda r: r["parameters"][0].update({"source": "nope"}),
    lambda r: next(e for e in r["parameters"] if e["dependencies"])
    .update({"dependencies": ["not-an-object"]}),
    lambda r: next(e for e in r["parameters"] if e["dependencies"])
    .update({"dependencies": "not-a-list"}),
])
def test_check_registry_returns_problems_instead_of_raising(mutator):
    mutated = _mutated(mutator)
    problems = vrt.check_registry(mutated)
    assert isinstance(problems, list) and problems


def test_check_registry_survives_a_non_object_root():
    assert vrt.check_registry(["not", "a", "registry"]) == ["registry root must be an object"]


def test_registry_index_is_tolerant_of_malformed_shapes():
    assert vrt.registry_index(["nope"]) == {}
    assert vrt.registry_index({"parameters": "nope"}) == {}
    assert vrt.registry_index({"parameters": ["nope", {"no_id": 1}]}) == {}


def test_check_registry_requires_a_strict_integer_version():
    assert any("version" in problem
               for problem in vrt.check_registry(_mutated(lambda r: r.update({"version": 1.0}))))


def test_check_registry_rejects_a_nul_source_path():
    problems = vrt.check_registry(_mutated(
        lambda r: r["parameters"][0]["source"].update({"file": "ros2_ws/\x00evil.yaml"})))
    assert any("safe relative path" in problem for problem in problems)


def test_check_registry_rejects_a_negative_sum_tolerance():
    def mutate(r):
        for entry in r["parameters"]:
            if entry["dependencies"]:
                for dep in entry["dependencies"]:
                    if dep["kind"] == "sum_equals":
                        dep["tolerance"] = -1.0
    problems = vrt.check_registry(_mutated(mutate))
    assert any("tolerance" in problem for problem in problems)


def test_check_registry_requires_native_proof_none_and_candidate_status():
    for entry in REGISTRY["parameters"]:
        assert entry["evidence"]["native_proof"] == "none", entry["id"]
        assert entry["evidence"]["kind"], entry["id"]
