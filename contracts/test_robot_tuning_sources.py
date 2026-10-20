#!/usr/bin/env python3
"""Robot-repo source-consistency tests for the robot-tuning registry.

These tests read the actual core-repo source YAML/JSON/launch files and prove
the registry is honest about what it says it is: every recorded default matches
the file it names, the fixed envelope matches the final smoother and the web
wire contract, and every node name exists in a launch source.

They are NOT part of the portable cross-repo set: the dashboard repo does not
have this source tree, so only ``test_robot_tuning.py`` and the three JSON files
plus the validator are vendored there.

No ROS imports, no robot contact. Read-only.

Run with: ``python -m pytest contracts/test_robot_tuning_sources.py``
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

import validate_robot_tuning as vrt  # noqa: E402

REPO_ROOT = vrt.REPO_ROOT
REGISTRY = vrt.load_registry()
INDEX = vrt.registry_index(REGISTRY)

LAUNCH_SOURCES = [
    REPO_ROOT / "ros2_ws/src/patrolbot_navigation/launch/bringup.launch.py",
    REPO_ROOT / "ros2_ws/src/patrolbot_navigation/launch/patrolbot_localization_launch.py",
    REPO_ROOT / "ros2_ws/src/patrolbot_navigation/launch/patrolbot_navigation_launch.py",
    REPO_ROOT / "ros2_ws/src/patrolbot-launch/launch/bringup.xml",
    REPO_ROOT / "ros2_ws/src/patrolbot-launch/launch/mobile_base.xml",
    REPO_ROOT / "ros2_ws/src/patrolbot-launch/launch/smoother.xml",
]

YAML_SOURCES = [
    (entry["id"], entry["source"])
    for entry in REGISTRY["parameters"]
    if entry["source"]["file"].endswith(".yaml")
]


def _ros_value(source: dict):
    data = yaml.safe_load((REPO_ROOT / source["file"]).read_text(encoding="utf-8"))
    node = data
    for part in source["node"].split("."):
        node = node[part]
    value = node["ros__parameters"]
    for part in source["key"].split("."):
        value = value[part]
    if source.get("index") is not None:
        value = value[source["index"]]
    return value


def test_source_tree_is_present():
    assert REPO_ROOT.joinpath("ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml").exists()


def _values_match(recorded, declared):
    """Match value AND scalar type; only genuinely numeric values get approx.

    YAML ``false`` must never compare equal to numeric ``0`` via an approximate
    numeric comparison.
    """
    if isinstance(recorded, bool) or isinstance(declared, bool):
        return (isinstance(recorded, bool) and isinstance(declared, bool)
                and recorded == declared)
    if isinstance(recorded, (int, float)) and isinstance(declared, (int, float)):
        return recorded == pytest.approx(declared)
    return recorded == declared


@pytest.mark.parametrize("entry_id,source", YAML_SOURCES, ids=[item[0] for item in YAML_SOURCES])
def test_registry_default_matches_recorded_source(entry_id, source):
    entry = INDEX[entry_id]
    recorded = _ros_value(source)
    assert _values_match(recorded, entry["default"]), (
        f"{entry_id}: registry default {entry['default']!r} "
        f"({type(entry['default']).__name__}) != source {recorded!r}")


def test_every_registry_source_file_exists():
    missing = []
    for entry in REGISTRY["parameters"]:
        path = REPO_ROOT / entry["source"]["file"]
        if not path.exists():
            missing.append(f"{entry['id']}: {entry['source']['file']}")
    assert missing == []


def test_hard_envelope_matches_final_smoother_and_web_contract():
    smoother = yaml.safe_load(
        (REPO_ROOT / "ros2_ws/src/patrolbot-launch/param/defaults/smoother.yaml").read_text()
    )["teleop_velocity_smoother"]["ros__parameters"]
    web = json.loads((REPO_ROOT / "contracts/web-teleop.json").read_text())
    envelope = REGISTRY["hard_envelope"]
    assert envelope["max_linear_m_s"]["hard_upper"] == smoother["max_velocity"][0]
    assert envelope["max_angular_rad_s"]["hard_upper"] == smoother["max_velocity"][2]
    assert envelope["max_linear_m_s"]["hard_upper"] == web["velocity_limits"]["linear_m_s"]["max"]
    angular = web["velocity_limits"]["angular_rad_s"]["max"]
    assert envelope["max_angular_rad_s"]["hard_upper"] == angular
    # Reverse-symmetric reverse bound must not be invented as a lower ceiling.
    assert smoother["min_velocity"][0] == -smoother["max_velocity"][0]
    assert smoother["min_velocity"][2] == -smoother["max_velocity"][2]


def test_ceiling_parameters_share_the_fixed_hard_upper():
    envelope = REGISTRY["hard_envelope"]
    for pid in ("max_linear_m_s", "max_angular_rad_s"):
        assert INDEX[pid]["bounds"]["upper"] == envelope[pid]["hard_upper"], pid


def test_approach_scaling_edit_budget_tracks_yaml_costmap_geometry():
    source = REPO_ROOT / "ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml"
    params = yaml.safe_load(source.read_text())["local_costmap"]["local_costmap"]["ros__parameters"]
    bounds = INDEX["nav2.controller.approach_velocity_scaling_dist"]["bounds"]
    assert bounds["upper"] == params["width"] / 2
    assert bounds["upper_inclusive"] is False


# Nav2 hosts the local/global costmaps inside controller_server/planner_server
# as internally-created child nodes; they have no separate launch <node> entry.
INTERNALLY_HOSTED_NODES = {"local_costmap", "global_costmap"}


def test_registry_node_names_exist_in_launch_sources():
    launch_text = "\n".join(path.read_text(encoding="utf-8") for path in LAUNCH_SOURCES)
    node_names = {entry["source"]["node"] for entry in REGISTRY["parameters"]}
    for node in node_names:
        leaf = node.split(".")[-1]
        if leaf in INTERNALLY_HOSTED_NODES:
            assert "controller_server" in launch_text, leaf
            assert "planner_server" in launch_text, leaf
            continue
        quoted = f"'{leaf}'" in launch_text or f'"{leaf}"' in launch_text
        assert quoted or f'name="{leaf}"' in launch_text, node


def test_baseline_commit_is_recorded_not_assumed():
    assert REGISTRY["generated_from"]["baseline_commit"] == "37a4c1a"
    assert REGISTRY["generated_from"]["limitations"]


def test_portable_contract_modules_never_import_ros():
    for name in ("validate_robot_tuning.py", "test_robot_tuning.py"):
        text = (vrt.CONTRACTS_DIR / name).read_text(encoding="utf-8")
        assert "import rclpy" not in text
        assert "from rclpy" not in text
        assert "ros2" not in text.replace("ros2_ws", "")  # only source paths may mention ros2_


def test_slice1_has_no_yaml_write_interface():
    for name in ("validate_robot_tuning.py", "test_robot_tuning.py"):
        text = (vrt.CONTRACTS_DIR / name).read_text(encoding="utf-8")
        for forbidden in ("yaml.safe_dump", "yaml.dump", "yaml.emit"):
            assert forbidden not in text, f"{name} must not write YAML in slice 1"


def test_registry_does_not_claim_installed_package_versions():
    blob = json.dumps(REGISTRY)
    assert "package_version_evidence" in blob
    assert REGISTRY["package_version_evidence"]["status"] == "unverified"


def test_amcl_alpha5_matches_source_but_is_not_editable():
    source = INDEX["amcl.alpha5"]["source"]
    assert _ros_value(source) == INDEX["amcl.alpha5"]["default"]
    assert INDEX["amcl.alpha5"]["not_applicable"] is True
    assert INDEX["amcl.alpha5"]["accepts_edits"] is False
