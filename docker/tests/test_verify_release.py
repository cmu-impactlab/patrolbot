"""The release checker is read-only and refuses changed source or artifacts."""
import importlib.util
import json
from pathlib import Path
from unittest.mock import Mock


def checker():
    spec = importlib.util.spec_from_file_location(
        'verify_release', Path(__file__).parents[1] / 'verify_release.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_missing_source_refuses_without_remote_access(tmp_path, monkeypatch):
    module = checker()
    manifest = tmp_path / 'release.json'
    manifest.write_text(json.dumps({'files': [{
        'repository_path': 'this-file-does-not-exist', 'sha256': '0' * 64}]}))
    monkeypatch.setattr(module.sys, 'argv', ['verify', str(manifest)])
    remote = Mock(side_effect=AssertionError('offline verification used SSH'))
    monkeypatch.setattr(module.subprocess, 'run', remote)
    assert module.main() == 1
    remote.assert_not_called()


def test_live_mismatch_propagates_and_has_no_apply_option(tmp_path, monkeypatch):
    module = checker()
    manifest = tmp_path / 'release.json'
    manifest.write_text(json.dumps({'files': [{
        'repository_path': 'missing', 'sha256': '0' * 64, 'host': 'test-host'}]}))
    monkeypatch.setattr(module.sys, 'argv', ['verify', str(manifest), '--live'])
    remote = Mock(return_value=Mock(returncode=0, stdout='["image mismatch: test"]'))
    monkeypatch.setattr(module.subprocess, 'run', remote)
    assert module.main() == 1
    command = remote.call_args.args[0]
    assert command[:3] == ['/usr/bin/ssh', '-o', 'BatchMode=yes']
    assert 'docker create' not in command[-1]
    assert 'restart' not in command[-1]
