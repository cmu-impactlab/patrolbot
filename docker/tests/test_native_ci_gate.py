"""Exercise the real pytest child/evidence protocol without ROS or Docker."""

import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('gate', ROOT / 'docker/native_ci_gate.py')
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


def run(tmp_path, source, deadline=10):
    package = tmp_path / 'package'
    (package / 'test').mkdir(parents=True)
    (package / 'test/test_sample.py').write_text(source)
    return gate.run_suite('custom', package, tmp_path / 'evidence', deadline)


@pytest.mark.parametrize('compatibility_proxy', [False, True])
def test_configuration_metadata_uses_plugin_registry(
        tmp_path, monkeypatch, pytestconfig, compatibility_proxy):
    # pytest 7 wraps path hooks in functions; newer pytest may expose HookCaller directly.
    hook = pytestconfig.hook
    if compatibility_proxy:
        hook = SimpleNamespace(pytest_ignore_collect=lambda **kwargs:
                               pytestconfig.hook.pytest_ignore_collect(**kwargs))
        assert not hasattr(hook.pytest_ignore_collect, 'get_hookimpls')
    config = SimpleNamespace(
        hook=hook, pluginmanager=pytestconfig.pluginmanager,
        rootpath=tmp_path, inipath=tmp_path / 'pytest.ini',
        invocation_params=SimpleNamespace(args=('test',)),
        _native_exclusions=tuple(tmp_path / path for path in gate.EXCLUSIONS))
    events = tmp_path / 'events.jsonl'
    monkeypatch.setenv('PATROLBOT_NATIVE_EVENTS', str(events))
    gate.pytest_sessionstart(SimpleNamespace(config=config))
    implementations = config.pluginmanager.hook.pytest_ignore_collect.get_hookimpls()
    assert implementations
    assert json.loads(events.read_text()) == {
        'event': 'configuration', 'pytest_version': pytest.__version__,
        'python_version': sys.version, 'cwd': str(Path.cwd()),
        'rootdir': str(config.rootpath), 'configfile': str(config.inipath),
        'args': ['test'], 'exclusions': list(map(str, config._native_exclusions)),
        'plugins': sorted([dist.project_name, dist.version]
                          for _, dist in config.pluginmanager.list_plugin_distinfo()),
        'ignore_hooks': [{'plugin': str(impl.plugin_name), 'tryfirst': impl.tryfirst,
                          'trylast': impl.trylast, 'hookwrapper': impl.hookwrapper}
                         for impl in implementations],
    }


def test_clean_suite_records_collection_all_phases_and_junit(tmp_path):
    result = run(tmp_path, 'def test_ok():\n    assert True\n')
    assert result['issues'] == []
    directory = tmp_path / 'evidence/custom'
    events = [json.loads(line) for line in (directory / 'events.jsonl').read_text().splitlines()]
    assert next(item for item in events if item['event'] == 'collection')['nodeids'] == [
        'test/test_sample.py::test_ok']
    assert [item['when'] for item in events if item['event'] == 'report'] == [
        'setup', 'call', 'teardown']
    assert result['expected_exclusions'] == list(gate.EXCLUSIONS)
    assert result['allowed_skips'] == []
    cleanup = result['process_group_cleanup']
    assert cleanup['signals_sent'] == []
    assert not cleanup['forced_cleanup']
    assert cleanup['group_retired']
    assert (directory / 'pytest.log').is_file()
    assert (directory / 'ros').is_dir()
    assert gate.check_evidence(directory, ['test/missing.py::test_missing']) == [
        'missing required integration sentinel: test/missing.py::test_missing']
    (directory / 'junit.xml').unlink()
    assert any('JUnit' in issue for issue in gate.check_evidence(directory, ()))


@pytest.mark.parametrize('source,reason', [
    ('import pytest\npytest.skip("collection skip", allow_module_level=True)\n', 'collection skipped'),
    ('import pytest\n@pytest.fixture\ndef setup_skip():\n    pytest.skip("setup skip")\n'
     'def test_skip(setup_skip):\n    pass\n', 'setup: skipped'),
    ('import pytest\ndef test_skip():\n    pytest.skip("call skip")\n', 'call: skipped'),
    ('import pytest\n@pytest.fixture\ndef teardown_skip():\n    yield\n    pytest.skip("teardown skip")\n'
     'def test_skip(teardown_skip):\n    pass\n', 'teardown: skipped'),
    ('import pytest\n@pytest.mark.xfail\ndef test_xfail():\n    assert False\n', 'call: skipped'),
    ('import threading\ndef test_thread():\n'
     '    def crash():\n        raise RuntimeError("thread broke")\n'
     '    worker = threading.Thread(target=crash)\n    worker.start()\n    worker.join()\n',
     'PytestUnhandledThreadExceptionWarning'),
    ('import gc\ndef test_unraisable():\n'
     '    class Broken:\n        def __del__(self):\n            raise RuntimeError("unraisable broke")\n'
     '    del_me = Broken()\n    del del_me\n    gc.collect()\n',
     'PytestUnraisableExceptionWarning'),
    ('import gc\ndef test_unawaited():\n'
     '    async def handler():\n        pass\n'
     '    coroutine = handler()\n    del coroutine\n    gc.collect()\n',
     'was never awaited'),
    ('def test_teardown(request):\n'
     '    def fail():\n        raise RuntimeError("teardown broke")\n'
     '    request.addfinalizer(fail)\n', 'teardown: failed'),
    ('raise RuntimeError("collection broke")\n', 'collection failed'),
    ('# no tests\n', 'empty or missing collection'),
    ('import os\ndef test_exit():\n    os._exit(27)\n', 'pytest process exit: 27'),
    ('import os, signal\ndef test_crash():\n    os.kill(os.getpid(), signal.SIGTERM)\n',
     'pytest process exit: -15'),
])
def test_reliability_failures_are_never_green(tmp_path, source, reason):
    result = run(tmp_path, source)
    assert result['issues']
    assert any(reason in issue for issue in result['issues']), result
    assert (tmp_path / 'evidence/custom/summary.json').is_file()


def test_deadline_keeps_logs_and_partial_collection(tmp_path):
    result = run(tmp_path, 'import time\ndef test_hang():\n    time.sleep(60)\n', deadline=1.5)
    assert result['timed_out']
    assert any('deadline exceeded' in issue for issue in result['issues'])
    assert any('session finish' in issue for issue in result['issues'])
    assert (tmp_path / 'evidence/custom/events.jsonl').is_file()
    assert (tmp_path / 'evidence/custom/pytest.log').is_file()
    assert result['process_group_cleanup']['signals_sent'] == ['SIGTERM']
    assert not result['process_group_cleanup']['forced_cleanup']
    assert result['process_group_cleanup']['group_retired']


def test_never_awaited_warning_cannot_be_greenwashed_by_local_warning_filter(tmp_path):
    result = run(tmp_path,
                 'import gc, warnings\ndef test_unawaited():\n'
                 '    warnings.simplefilter("always", RuntimeWarning)\n'
                 '    async def handler():\n        pass\n'
                 '    coroutine = handler()\n    del coroutine\n    gc.collect()\n')
    assert result['returncode'] == 0
    assert any('RuntimeWarning:' in issue and 'was never awaited' in issue
               for issue in result['issues'])


def test_missing_or_corrupt_evidence_fails(tmp_path):
    assert 'missing or invalid pytest evidence' in gate.check_evidence(tmp_path, ())[0]
    (tmp_path / 'events.jsonl').write_text('{broken')
    assert 'missing or invalid pytest evidence' in gate.check_evidence(tmp_path, ())[0]


@pytest.mark.parametrize('different_rootdir', [False, True])
@pytest.mark.parametrize('installed_hook', [False, True])
def test_expected_lint_exclusions_happen_before_collection(
        tmp_path, monkeypatch, different_rootdir, installed_hook):
    package = tmp_path / 'package'
    (package / 'test').mkdir(parents=True)
    for name in gate.EXCLUSIONS:
        (package / name).write_text('raise RuntimeError("must never import lint")\n')
    (package / 'test/test_ok.py').write_text('def test_ok():\n    pass\n')
    # Same basenames under other paths remain functional gate inputs.
    (package / 'test/nested').mkdir()
    (package / 'test/nested/test_flake8.py').write_text('def test_nested():\n    pass\n')
    other = tmp_path / 'other/test'
    other.mkdir(parents=True)
    (other / 'test_pep257.py').write_text('def test_other_package():\n    pass\n')
    if installed_hook:
        (package / 'force_collect.py').write_text(
            'import pytest\n'
            'def pytest_configure(config):\n'
            '    names = [name for name, _ in config.pluginmanager.list_name_plugin()]\n'
            '    assert names.index("native_ci_gate") < names.index("force-collect")\n'
            '@pytest.hookimpl(tryfirst=True)\n'
            'def pytest_ignore_collect(collection_path, config):\n'
            '    if collection_path.suffix == ".py":\n        return False\n')
        dist = package / 'force_collect-1.0.dist-info'
        dist.mkdir()
        (dist / 'METADATA').write_text('Name: force-collect\nVersion: 1.0\n')
        (dist / 'entry_points.txt').write_text('[pytest11]\nforce-collect = force_collect\n')
        monkeypatch.setenv('PYTHONPATH', os.pathsep.join(
            filter(None, (str(package), os.environ.get('PYTHONPATH')))))
    rootdir = tmp_path if different_rootdir else package
    sentinel = f'{"package/" if different_rootdir else ""}test/test_ok.py::test_ok'
    monkeypatch.setitem(gate.SENTINELS, 'custom', (sentinel,))
    result = gate.run_suite('custom', package, tmp_path / 'evidence', 10,
                            (f'--rootdir={rootdir}', 'test', str(other)))
    assert result['issues'] == []
    directory = tmp_path / 'evidence/custom'
    assert 'must never import lint' not in (directory / 'pytest.log').read_text()
    events = [json.loads(line) for line in (directory / 'events.jsonl').read_text().splitlines()]
    exclusions = {str(package / name) for name in gate.EXCLUSIONS}
    assert {item['path'] for item in events if item['event'] == 'excluded'} == exclusions
    configuration = next(item for item in events if item['event'] == 'configuration')
    assert configuration['rootdir'] == str(rootdir)
    assert set(configuration['exclusions']) == exclusions
    assert configuration['pytest_version'] == pytest.__version__
    assert (['force-collect', '1.0'] in configuration['plugins']) == installed_hook
    hooks = {hook['plugin']: hook for hook in configuration['ignore_hooks']}
    assert hooks['native_ci_gate']['hookwrapper'] is True
    if installed_hook:
        assert hooks['force-collect'] == {
            'plugin': 'force-collect', 'tryfirst': True, 'trylast': False,
            'hookwrapper': False}
    collection = next(item for item in events if item['event'] == 'collection')
    assert set(collection['files']) == {
        str(package / 'test/test_ok.py'), str(package / 'test/nested/test_flake8.py'),
        str(other / 'test_pep257.py')}
    assert not any(item['event'] == 'deselected' for item in events)


@pytest.mark.parametrize('excluded', [False, True])
def test_ignore_wrapper_preserves_delegate_failure(tmp_path, excluded):
    package = tmp_path / 'package'
    (package / 'test').mkdir(parents=True)
    filename = gate.EXCLUSIONS[0] if excluded else 'test/test_ok.py'
    (package / filename).write_text('raise RuntimeError("must never import")\n')
    (package / 'conftest.py').write_text(
        'import pytest\n'
        '@pytest.hookimpl(tryfirst=True)\n'
        'def pytest_ignore_collect(collection_path, config):\n'
        f'    if collection_path.name == {Path(filename).name!r}:\n'
        '        raise RuntimeError("delegate hook broke")\n')
    result = gate.run_suite('custom', package, tmp_path / 'evidence', 10)
    assert result['returncode'] != 0
    assert result['issues']
    log = (tmp_path / 'evidence/custom/pytest.log').read_text()
    assert 'delegate hook broke' in log
    assert 'must never import' not in log


@pytest.mark.parametrize('result', [None, False, True])
def test_ignore_wrapper_preserves_other_path_result(tmp_path, result):
    class Delegate:
        @pytest.hookimpl(tryfirst=True)
        def pytest_ignore_collect(self, collection_path, config):
            return result

    manager = pytest.PytestPluginManager()
    manager.register(gate, 'native_ci_gate')
    manager.register(Delegate(), 'later_delegate')
    config = SimpleNamespace(
        _native_exclusions=tuple(tmp_path / path for path in gate.EXCLUSIONS))
    assert manager.hook.pytest_ignore_collect(
        collection_path=tmp_path / 'test/test_functional.py', config=config) is result


@pytest.mark.parametrize('addopts', ['--ignore=test/test_ok.py', '--ignore-glob=*ok.py'])
def test_extra_exclusions_are_rejected(tmp_path, addopts):
    package = tmp_path / 'package'
    (package / 'test').mkdir(parents=True)
    (package / 'pytest.ini').write_text(f'[pytest]\naddopts = {addopts}\n')
    (package / 'test/test_ok.py').write_text('def test_ok():\n    pass\n')
    result = gate.run_suite('custom', package, tmp_path / 'evidence', 10)
    assert result['issues']
    assert result['returncode'] != 0
    assert 'permits only its declared absolute exclusions' in (
        tmp_path / 'evidence/custom/pytest.log').read_text()


def test_evidence_rejects_excluded_collection_and_unknown_exclusions(tmp_path):
    assert run(tmp_path, 'def test_ok():\n    pass\n')['issues'] == []
    directory = tmp_path / 'evidence/custom'
    evidence = directory / 'events.jsonl'
    events = [json.loads(line) for line in evidence.read_text().splitlines()]
    exclusions = [str(tmp_path / 'package' / path) for path in gate.EXCLUSIONS]
    next(item for item in events if item['event'] == 'collection')['files'].append(exclusions[0])
    next(item for item in events if item['event'] == 'configuration')['exclusions'].append('extra')
    events.append({'event': 'excluded', 'path': 'extra'})
    evidence.write_text(''.join(json.dumps(item) + '\n' for item in events))
    assert gate.check_evidence(directory, (), exclusions) == [
        'pre-collection exclusions differ from declared package paths',
        'declared excluded files reached collection', 'unexpected pre-collection exclusion']


def test_main_aggregates_later_suites_after_failure(tmp_path, monkeypatch):
    for name in gate.SENTINELS:
        package = tmp_path / f'ros2_ws/src/patrolbot_{name}/test'
        package.mkdir(parents=True)
        (package / 'test_sample.py').write_text(
            f'def test_ok():\n    assert {name != "bridge"}\n')
    monkeypatch.setattr(gate, 'SENTINELS', dict.fromkeys(gate.SENTINELS, ()))
    monkeypatch.setattr(sys, 'argv', ['native_ci_gate.py'])
    monkeypatch.setenv('PATROLBOT_TEST_WORKSPACE', str(tmp_path))
    monkeypatch.setenv('PATROLBOT_TEST_EVIDENCE', str(tmp_path / 'evidence'))
    assert gate.main() == 1
    results = json.loads((tmp_path / 'evidence/summary.json').read_text())
    assert [result['suite'] for result in results] == ['bridge', 'dock_manager', 'navigation']
    assert [bool(result['issues']) for result in results] == [True, False, False]


def test_sentinels_reference_real_source_tests():
    import ast
    for name, sentinels in gate.SENTINELS.items():
        for nodeid in sentinels:
            filename, function = nodeid.split('::')
            source = ROOT / f'ros2_ws/src/patrolbot_{name}' / filename
            tree = ast.parse(source.read_text())
            assert any(isinstance(node, ast.FunctionDef) and node.name == function
                       for node in tree.body), nodeid


def test_leftover_native_child_fails_and_is_killed(tmp_path):
    result = run(tmp_path,
                 'import subprocess, sys\ndef test_leak():\n'
                 '    subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])\n')
    assert result['leaked_children']
    assert 'pytest left a live process group after shutdown' in result['issues']
    cleanup = result['process_group_cleanup']
    assert cleanup['signals_sent'][0] == 'SIGTERM'
    assert cleanup['elapsed_seconds'] < 8
    if not cleanup['group_retired']:
        assert 'suite process group did not retire within cleanup deadline' in result['issues']


@pytest.mark.parametrize('addopts', [
    '-k keep', '--deselect=test/test_sample.py::test_other', '-p no:native_ci_gate',
])
def test_inherited_pytest_selection_and_plugins_are_sanitized(tmp_path, monkeypatch, addopts):
    monkeypatch.setenv('PYTEST_ADDOPTS', addopts)
    monkeypatch.setenv('PYTEST_PLUGINS', 'nonexistent_host_plugin')
    monkeypatch.setenv('PYTEST_DISABLE_PLUGIN_AUTOLOAD', '1')
    monkeypatch.setitem(gate.SENTINELS, 'custom', ('test/test_sample.py::test_keep',))
    result = run(tmp_path,
                 'import os\ndef test_keep():\n    pass\n'
                 'def test_other():\n'
                 '    assert not any(key in os.environ for key in '
                 '("PYTEST_ADDOPTS", "PYTEST_PLUGINS", "PYTEST_DISABLE_PLUGIN_AUTOLOAD"))\n')
    assert result['issues'] == []
    events = (tmp_path / 'evidence/custom/events.jsonl').read_text()
    assert 'test/test_sample.py::test_other' in events
    assert '"event": "deselected"' not in events


@pytest.mark.parametrize('addopts', ['-k keep', '--deselect=test/test_sample.py::test_other'])
def test_actual_pytest_deselection_cannot_claim_full_coverage(tmp_path, monkeypatch, addopts):
    package = tmp_path / 'package'
    (package / 'test').mkdir(parents=True)
    (package / 'pytest.ini').write_text(f'[pytest]\naddopts = {addopts}\n')
    (package / 'test/test_sample.py').write_text(
        'def test_keep():\n    pass\ndef test_other():\n    pass\n')
    monkeypatch.setitem(gate.SENTINELS, 'bridge', ('test/test_sample.py::test_keep',))
    result = gate.run_suite('bridge', package, tmp_path / 'evidence', 10)
    assert result['returncode'] == 0
    assert result['issues'] == [
        "unexpected deselected tests: ['test/test_sample.py::test_other']"]
    events = (tmp_path / 'evidence/bridge/events.jsonl').read_text()
    assert '"event": "deselected"' in events


def test_explicit_diagnostic_path_still_runs(tmp_path):
    package = tmp_path / 'package'
    (package / 'test').mkdir(parents=True)
    (package / 'test/test_keep.py').write_text('def test_keep():\n    pass\n')
    (package / 'test/test_other.py').write_text('raise RuntimeError("not selected")\n')
    result = gate.run_suite('custom', package, tmp_path / 'evidence', 10,
                            ('test/test_keep.py',))
    assert result['issues'] == []


def test_missing_timeout_plugin_config_is_fatal(tmp_path):
    package = tmp_path / 'package'
    (package / 'test').mkdir(parents=True)
    (package / 'pytest.ini').write_text('[pytest]\ntimeout = 1\n')
    (package / 'test/test_ok.py').write_text('def test_ok():\n    pass\n')
    result = gate.run_suite('custom', package, tmp_path / 'evidence', 10,
                            ('-p', 'no:timeout', 'test'))
    assert result['issues']
    assert result['returncode'] != 0
    assert 'Unknown config option: timeout' in (
        tmp_path / 'evidence/custom/pytest.log').read_text()


def test_forced_cleanup_is_bounded_and_leaves_unrelated_process_alive(tmp_path):
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                                 start_new_session=True)
    try:
        result = run(tmp_path,
                     'import signal, time\ndef test_stubborn():\n'
                     '    signal.signal(signal.SIGTERM, signal.SIG_IGN)\n'
                     '    time.sleep(60)\n', deadline=1.5)
        cleanup = result['process_group_cleanup']
        assert cleanup['signals_sent'] == ['SIGTERM', 'SIGKILL']
        assert cleanup['forced_cleanup']
        assert cleanup['group_retired']
        assert cleanup['elapsed_seconds'] < 8
        assert 'suite process group required SIGKILL' in result['issues']
        assert unrelated.poll() is None
    finally:
        os.killpg(unrelated.pid, signal.SIGKILL)
        unrelated.wait(timeout=5)
