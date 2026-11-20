"""Keep required checks tied to unconditional PR jobs and the reviewed policy."""

import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
POLICY = json.loads((ROOT / '.github/main-protection.json').read_text())
WORKFLOWS = {
    path.name: yaml.load(path.read_text(), Loader=yaml.BaseLoader)
    for path in (ROOT / '.github/workflows').iterdir()
    if path.suffix in {'.yml', '.yaml'}
}


def test_required_checks_match_unconditional_pull_request_jobs():
    pr_workflows = {
        name: workflow for name, workflow in WORKFLOWS.items()
        if 'pull_request' in workflow['on']
    }
    assert set(pr_workflows) == {
        'robot-tuning-contract.yml', 'native-test-image.yml',
    }
    contexts = []
    for workflow in pr_workflows.values():
        # Empty event config forbids paths, branches, and event-type filters.
        assert workflow['on']['pull_request'] in ('', {})
        for job_id, job in workflow['jobs'].items():
            assert 'if' not in job
            assert 'strategy' not in job
            assert 'needs' not in job
            contexts.append(job.get('name', job_id))
    assert sorted(contexts) == ['contract', 'native-test-image']
    assert POLICY['required_status_checks'] == {
        'strict': True,
        'checks': [{'context': context, 'app_id': 15368}
                   for context in sorted(contexts)],
    }


def test_documentation_publication_is_not_required():
    docs = WORKFLOWS['deploy.yml']
    assert 'pull_request' not in docs['on']
    assert 'workflow_dispatch' in docs['on']
    assert docs['on']['push']['branches'] == ['main']
    assert docs['on']['push']['paths']
    required = {check['context']
                for check in POLICY['required_status_checks']['checks']}
    assert required.isdisjoint(
        job.get('name', job_id) for job_id, job in docs['jobs'].items())


def test_review_rules_and_remaining_flags_have_no_bypass():
    assert {key: value for key, value in POLICY.items()
            if key != 'required_status_checks'} == {
        'enforce_admins': True,
        'required_pull_request_reviews': {
            'required_approving_review_count': 1,
            'dismiss_stale_reviews': True,
            'require_last_push_approval': True,
            'require_code_owner_reviews': False,
            'bypass_pull_request_allowances': {
                'users': [], 'teams': [], 'apps': [],
            },
        },
        'required_conversation_resolution': True,
        'allow_force_pushes': False,
        'allow_deletions': False,
        'restrictions': None,
        'required_linear_history': False,
        'lock_branch': False,
    }
