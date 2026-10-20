"""Bootstrap preflight: honest preview, byte-identical state, no initialisation."""

import ast
import json
import os
from pathlib import Path
import subprocess
import sys

from patrolbot_bridge import config_bootstrap as bootstrap
from patrolbot_bridge.tuning_store import RevisionStore

REPO = Path(__file__).resolve().parents[4]
REGISTRY_PATH = REPO / 'contracts/robot-tuning.registry.json'
REGISTRY = json.loads(REGISTRY_PATH.read_text())
ACTOR = {'id': 1, 'username': 'fixture', 'role': 'administrator'}


def _tree(root):
    root = Path(root)
    if not root.exists():
        return None
    return {
        str(path.relative_to(root)) + ('/' if path.is_dir() else ''):
            (b'' if path.is_dir() else path.read_bytes())
        for path in sorted(root.rglob('*'))
    }


def test_clean_missing_state_previews_honestly_without_writing(tmp_path):
    state = tmp_path / 'config'
    report = bootstrap.build_plan(REGISTRY_PATH, state)
    assert report['bootstrap_status'] == 'clean_uninitialized'
    assert report['write_performed'] is False
    assert report['activatable'] is False
    assert report['initialization_supported'] is False
    assert report['live_readback'] is False
    assert report['migration_eligible'] is False
    assert report['state_parent_available'] is True
    assert 'preview is read-only' in report['initialization_blocked_reason']
    candidate = report['candidate_defaults']
    assert candidate['revision'] == 1
    assert candidate['validation'] == 'ok'
    assert candidate['reasons'] == []
    assert candidate['artifacts'] == ['bringup.yaml', 'navigation.yaml']
    assert candidate['defaults_digest'].startswith('sha256:')
    assert candidate['config_digest'].startswith('sha256:')
    # Reviewed angular producers above the cap are clamped, not a defect.
    assert candidate['clamped_producers']
    assert report['summary'].splitlines()[1].startswith('Status clean_uninitialized')
    assert not state.exists()
    assert list(tmp_path.iterdir()) == []
    blob = json.dumps(report, sort_keys=True, allow_nan=False)
    assert len(blob.encode('utf-8')) <= bootstrap.MAX_REPORT_BYTES


def test_existing_empty_state_dir_is_clean_and_left_untouched(tmp_path):
    state = tmp_path / 'config'
    state.mkdir()
    before = _tree(state)
    report = bootstrap.build_plan(REGISTRY_PATH, state)
    assert report['bootstrap_status'] == 'clean_uninitialized'
    assert _tree(state) == before == {}


def test_initialized_state_is_refused_and_left_byte_identical(tmp_path):
    state = tmp_path / 'config'
    RevisionStore(state, REGISTRY).initialize(actor=ACTOR, reason='fixture only')
    before = _tree(state)
    report = bootstrap.build_plan(REGISTRY_PATH, state)
    assert report['bootstrap_status'] == 'already_initialized'
    assert report['write_performed'] is False
    assert report['candidate_defaults']['revision'] is None
    assert report['current']['desired_revision'] == 1
    assert any('already_initialized' in item for item in report['refusals'])
    assert _tree(state) == before


def test_corrupt_state_is_refused_never_reset(tmp_path):
    state = tmp_path / 'config'
    RevisionStore(state, REGISTRY).initialize(actor=ACTOR, reason='fixture only')
    (state / 'initialized').unlink()
    before = _tree(state)
    report = bootstrap.build_plan(REGISTRY_PATH, state)
    assert report['bootstrap_status'] == 'damaged'
    assert any('corrupt_missing_sentinel' in item for item in report['refusals'])
    assert report['candidate_defaults']['revision'] is None
    assert _tree(state) == before


def test_symlinked_state_dir_is_refused(tmp_path):
    real = tmp_path / 'real'
    real.mkdir()
    link = tmp_path / 'config'
    link.symlink_to(real, target_is_directory=True)
    report = bootstrap.build_plan(REGISTRY_PATH, link)
    assert report['bootstrap_status'] == 'damaged'
    assert any('symlink' in item for item in report['refusals'])
    assert _tree(real) == {}


def test_oversized_registry_is_rejected(tmp_path):
    oversized = tmp_path / 'registry.json'
    oversized.write_bytes(b'{' + b' ' * bootstrap.MAX_REGISTRY_BYTES + b'}')
    report = bootstrap.build_plan(oversized, tmp_path / 'config')
    assert report['bootstrap_status'] == 'refused'
    assert any('registry_oversized' in item for item in report['refusals'])
    assert not (tmp_path / 'config').exists()


def test_duplicate_registry_keys_are_rejected(tmp_path):
    duplicate = tmp_path / 'registry.json'
    duplicate.write_text('{"contract": "patrolbot-robot-tuning", '
                         '"contract": "patrolbot-robot-tuning"}')
    report = bootstrap.build_plan(duplicate, tmp_path / 'config')
    assert report['bootstrap_status'] == 'refused'
    assert any('registry_invalid_json' in item for item in report['refusals'])


def test_non_contract_registry_is_rejected(tmp_path):
    bad = tmp_path / 'registry.json'
    bad.write_text(json.dumps({'contract': 'nope', 'version': 1, 'parameters': []}))
    report = bootstrap.build_plan(bad, tmp_path / 'config')
    assert report['bootstrap_status'] == 'refused'
    assert any('registry_invalid' in item for item in report['refusals'])


def test_preview_never_auto_initializes(tmp_path):
    state = tmp_path / 'config'
    bootstrap.build_plan(REGISTRY_PATH, state)
    assert not state.exists()
    assert RevisionStore(state, REGISTRY).state()['status'] == 'uninitialized'


def test_cli_prints_bounded_json_without_initializing(tmp_path, capsys):
    state = tmp_path / 'config'
    code = bootstrap.main(['--registry', str(REGISTRY_PATH), '--state-dir', str(state)])
    report = json.loads(capsys.readouterr().out)
    assert code == 0
    assert report['bootstrap_status'] == 'clean_uninitialized'
    assert not state.exists()


def test_module_never_imports_motion_or_ros():
    tree = ast.parse(Path(bootstrap.__file__).read_text())
    absolute, relative = set(), set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            absolute.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                relative.add(node.module or node.names[0].name)
            elif node.module:
                absolute.add(node.module.split('.')[0])
    assert absolute <= {'__future__', 'argparse', 'json', 'os', 'pathlib', 'stat'}
    assert relative <= {'tuning_json', 'tuning_spec', 'tuning_store'}


def test_import_pulls_no_motion_modules():
    code = (
        'import sys; import patrolbot_bridge.config_bootstrap; '
        'forbidden = ("rclpy", "patrolbot_bridge.bridge_node", '
        '"patrolbot_bridge.epoch_runtime", "patrolbot_bridge.undock_turn", '
        '"patrolbot_bridge.map_runtime", "patrolbot_bridge.tuning_activation"); '
        'print(",".join(name for name in forbidden if name in sys.modules))'
    )
    package_root = str(Path(bootstrap.__file__).resolve().parents[1])
    result = subprocess.run(
        [sys.executable, '-c', code], capture_output=True, text=True,
        env=dict(os.environ, PYTHONPATH=package_root))
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == ''


def test_malformed_registry_structure_is_a_refusal(tmp_path):
    malformed = dict(REGISTRY, parameters=[{}])
    path = tmp_path / 'registry.json'
    path.write_text(json.dumps(malformed))
    result = bootstrap.build_plan(path, tmp_path / 'config')
    assert result['bootstrap_status'] == 'refused'
    assert result['write_performed'] is False


def test_unavailable_candidate_artifacts_do_not_report_success(tmp_path, monkeypatch):
    def unavailable(*args):
        raise OSError('packaged defaults unavailable')

    monkeypatch.setattr(bootstrap.tuning_spec, 'generated_artifacts', unavailable)
    result = bootstrap.build_plan(REGISTRY_PATH, tmp_path / 'config')
    assert result['bootstrap_status'] == 'refused'
    assert result['candidate_defaults']['validation'] == 'unavailable'
    assert not (tmp_path / 'config').exists()


def test_report_budget_remains_bounded_for_many_candidate_reasons(tmp_path):
    report = bootstrap.build_plan(REGISTRY_PATH, tmp_path / 'config')
    report['candidate_defaults']['reasons'] = ['x' * 1000] * 100
    encoded = bootstrap._serialize(report)
    assert len(encoded.encode('utf-8')) <= bootstrap.MAX_REPORT_BYTES
    assert json.loads(encoded)['bootstrap_status'] == 'refused'


def test_fifo_registry_is_refused_without_waiting_for_a_writer(tmp_path):
    path = tmp_path / 'registry.pipe'
    os.mkfifo(path)
    package_root = str(Path(bootstrap.__file__).resolve().parents[1])
    result = subprocess.run(
        [sys.executable, '-m', 'patrolbot_bridge.config_bootstrap',
         '--registry', str(path), '--state-dir', str(tmp_path / 'config')],
        capture_output=True, text=True, timeout=5,
        env=dict(os.environ, PYTHONPATH=package_root))
    assert result.returncode == 2
    assert 'registry_not_regular' in json.loads(result.stdout)['refusals'][0]
