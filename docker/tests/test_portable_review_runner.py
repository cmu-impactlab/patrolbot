"""The portable runner must never inherit live test opt-ins or hide failures."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    'portable_runner', ROOT / 'docker/run-portable-review-tests.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def test_list_excludes_native_and_lint_without_running(monkeypatch, capsys):
    monkeypatch.setattr(runner.sys, 'argv', ['runner', '--list'])
    monkeypatch.setattr(runner.subprocess, 'run', lambda *a, **k: (_ for _ in ()).throw(
        AssertionError('list must not execute tests')))
    assert not runner.main()
    selected = capsys.readouterr().out
    for name in runner.EXCLUDED:
        assert name not in selected
    assert 'test_manual_undock_runtime.py' in selected
    assert 'test_no_gimbal.py' in selected


def test_failed_cpp_binary_does_not_hide_other_binaries(monkeypatch):
    calls = []
    monkeypatch.setattr(runner.sys, 'argv', ['runner'])
    monkeypatch.setenv('PATROLBOT_RUN_INTEGRATION', '1')
    monkeypatch.setenv('PYTEST_ADDOPTS', '--some-native-option')

    def run(command, **kwargs):
        calls.append(command)
        assert kwargs['env']['PATROLBOT_RUN_INTEGRATION'] == '0'
        assert kwargs['env']['PYTHONDONTWRITEBYTECODE'] == '1'
        assert 'PYTEST_ADDOPTS' not in kwargs['env']
        assert kwargs['env']['PYTHONPATH'].split(':')[0] == str(
            ROOT / 'ros2_ws/src/patrolbot_bridge')
        return SimpleNamespace(returncode=int(command[0].endswith('/tests/protocol_test')))

    monkeypatch.setattr(runner.subprocess, 'run', run)
    assert runner.main()
    binaries = [str(ROOT / 'hardware/patrolbot_hw_server' / name)
                for name in runner.HARDWARE_TESTS]
    assert [call[0] for call in calls[-len(binaries):]] == binaries
    pytest_calls = [call for call in calls if 'pytest' in call]
    assert len(pytest_calls) == len(runner.GROUPS)
    assert all('-B' in call and 'no:cacheprovider' in call for call in pytest_calls)
