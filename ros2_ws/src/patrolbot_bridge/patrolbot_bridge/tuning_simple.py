"""Reduced Apply: close input, hot-update or restart all Nav2, verify, reopen.

One existing core command gate owns maintenance. New input is discarded. No
per-node lifecycle sequence or automatic whole-robot redeploy is selected here.
Transport support is explicit and fails closed while end-to-end hooks are absent.
"""

from .tuning_activation import AdmissionRefused, Evidence, ReloadResult, evaluate_evidence
from .tuning_backend import NODES, parameter_plan
from .tuning_lease import context
from .tuning_plan import canonical_digest
from .tuning_simple_plan import build_plan


class SimpleApplyBackend:
    requires_review = True
    plan_strategy = 'hot_or_nav2_redeploy'
    detail = 'Verified hot updates or fixed navigation-only redeployment are required.'

    def __init__(self, store, transport, probe, *, enabled=False, generation=None):
        self.store, self.transport, self._probe = store, transport, probe
        self.enabled = enabled is True
        self.generation = generation or (lambda: None)
        self._context = None
        self._installed = None
        self._verified = None

    def preview_capabilities(self):
        return self.transport.simple_capabilities()

    @property
    def approved(self):
        return self.enabled and self.preview_capabilities().get('ready') is True

    def probe(self):
        return self._probe()

    def evaluate_admission(self, evidence):
        if self._context is None or not self.transport.input_gate_closed(self._context):
            return evaluate_evidence(evidence)
        # Release/intent observations are required BEFORE closing admission.
        # Once the gate is acknowledged closed, fresh button presses/commands
        # are ignored, not treated as a reason to interrupt installation. This
        # does not call them 'released' or fabricate any odometry evidence.
        if not isinstance(evidence, Evidence):
            return False, 'hardware/ownership evidence unavailable'
        for name in ('hardware_ok', 'dock_ok', 'faults_clear', 'command_buffers_zero',
                     'maps_clear', 'undock_clear', 'lifecycle_clear', 'autonomy_idle'):
            if getattr(evidence, name) is not True:
                return False, 'maintenance evidence unavailable or unsafe: ' + name
        return True, ''

    def preflight(self, revision, digest):
        state = self.store.load_revision(revision)
        if state['meta']['generated_digest'] != digest:
            raise ValueError('candidate digest changed')
        parameter_plan(state, self.store.index)

    def inspect(self, revision, digest):
        """Preserve the bridge's read-only ACTIVE observation API.

        This does not reserve input, write parameters, promote a revision or
        replace the stronger complete-job verification required for Apply.
        """
        state = self.store.load_revision(revision)
        if state['meta']['generated_digest'] != digest:
            raise ValueError('immutable revision digest mismatch')
        actual, _ = self.transport.inspect_parameters(parameter_plan(state, self.store.index))
        version = self.transport.get('patrolbot_joy_teleop', ['tuning_policy_version'])
        if type(version.get('tuning_policy_version')) is not int or version['tuning_policy_version'] != 1:
            raise ValueError('F710 cached-policy implementation is unavailable')
        return state, actual

    def _job(self):
        job = self.store.latest_job()
        if not job or job.get('state') not in ('pending', 'running'):
            raise ValueError('durable apply job required')
        plan = job.get('execution_plan')
        if (not isinstance(plan, dict) or plan.get('strategy') != self.plan_strategy
                or job.get('plan_hash') != plan.get('plan_hash')):
            raise ValueError('exact reduced Apply plan required')
        return job, plan, context({'job_id': job['job_id'], 'digest': job['digest'], 'plan_hash': job['plan_hash']})

    def reserve_navigation(self):
        job, plan, value = self._job()
        if self._context is not None:
            raise AdmissionRefused('another apply owns the input gate')
        self._context = value
        self.store.record_job(dict(job, input_gate_attempted=True))
        try:
            self.transport.block_inputs(plan, value)
        except AdmissionRefused:
            # A definitive refusal must have unwound any reversible admission.
            # An ambiguous block operation is NOT caught/retried/released here.
            self._context = None
            raise
        if not self.transport.input_gate_closed(value):
            raise RuntimeError('input gate closure was not acknowledged')

    def release_navigation_reservation(self):
        if self._context is None:
            return
        if self.transport.release_inputs(self._context) is not True:
            raise RuntimeError('input gate release outcome unknown')
        self._context = None

    def _bind(self, plan, revision, digest, job_id):
        job, recorded, value = self._job()
        if self._context != value or job_id != value['job_id'] or digest != value['digest'] or plan != recorded:
            raise ValueError('apply job/plan changed')
        if plan['revision'] != revision or plan['hardware_generation'] != canonical_digest(self.generation()):
            raise ValueError('apply revision or hardware generation changed')
        active = self.store.load_revision(plan['base_revision'])
        candidate = self.store.load_revision(revision)
        if active['meta']['generated_digest'] != plan['base_digest'] or candidate['meta']['generated_digest'] != digest:
            raise ValueError('immutable artifact binding changed')
        # Rebuild from the same captured state and source audit, not caller
        # supplied node names/parameters. Support/scope changes require review.
        capabilities = self.preview_capabilities()
        if capabilities.get('ready') is not True:
            # Long replacement boot can outlive the public read-only cache.
            # Refresh real read-only evidence in this worker, never extend age
            # or rely on the background policy reader blocked by Apply's writer.
            refresh = getattr(self.transport, 'poll_simple_capabilities', None)
            if callable(refresh):
                refresh()
                capabilities = self.preview_capabilities()
        if capabilities.get('ready') is not True:
            raise ValueError('native execution support is no longer current')
        rebuilt = build_plan(candidate=candidate, active=active, index=self.store.index,
            registry_digest=self.store.registry_digest, session_id=plan['session_id'], generation=self.generation(),
            desired_revision=plan['desired_revision'], operation=plan['operation'], states=plan['runtime_states'],
            hot_support=capabilities.get('hot_support', ()),
            nav2_redeploy_supported=capabilities.get('nav2_redeploy_supported', False),
            policy_support=capabilities.get('policy_support', False),
            implementation=capabilities.get('implementation', 'unverified'))
        if rebuilt != plan or not plan['executable']:
            raise ValueError('reduced Apply plan no longer supported')
        if not self.transport.input_gate_closed(value):
            raise ValueError('input gate is not held')
        allowed, reason = self.evaluate_admission(self.probe())
        if not allowed:
            raise ValueError(reason)
        return candidate

    def reload_plan(self, plan, revision, digest, *, job_id):
        try:
            self._bind(plan, revision, digest, job_id)
            self._installed = self._verified = None
            if plan['cache_consumers']:
                self.transport.stage_policies(plan, self._context)
            if plan['execution_kind'] == 'nav2_redeploy':
                # Exactly one fixed service restart; coordinator/SBC stay up.
                # Any timeout is unknown. There is no retry or wider fallback.
                self.transport.restart_navigation(plan, self._context)
            for step in plan['operations']:
                if step['kind'] == 'parameters':
                    self.transport.apply_hot(step['node'], step['parameters'], plan, self._context)
                elif step['kind'] != 'nav2_restart' or plan['execution_kind'] != 'nav2_redeploy':
                    raise ValueError('unexpected execution operation')
            self._bind(plan, revision, digest, job_id)
            self._installed = (job_id, revision, digest, plan['plan_hash'])
            return ReloadResult('ok', 'Parameters installed; verification pending.')
        except Exception as exc:
            return ReloadResult('unknown', 'Apply installation outcome unknown: ' + type(exc).__name__)

    def verify_plan(self, plan, revision, digest, *, job_id):
        candidate = self._bind(plan, revision, digest, job_id)
        if self._installed != (job_id, revision, digest, plan['plan_hash']):
            raise ValueError('installation did not complete')
        expected = parameter_plan(candidate, self.store.index)
        proof = self.transport.verify_application(expected, plan, self._context)
        self._bind(plan, revision, digest, job_id)
        if not isinstance(proof, dict) or proof.get('verified_nodes') != sorted(NODES):
            raise ValueError('actual consumer verification is incomplete')
        result = dict(proof, complete=True, job_id=job_id, revision=revision, digest=digest,
            plan_hash=plan['plan_hash'], hardware_generation=plan['hardware_generation'], scopes=['bringup', 'navigation'])
        self._verified = result
        return result

    def committed_plan(self, plan, *, job_id):
        if self._verified is None or self._verified['job_id'] != job_id or self._verified['plan_hash'] != plan['plan_hash']:
            raise ValueError('exact revision was not verified')
        self._bind(plan, plan['revision'], plan['digest'], job_id)
        if self.transport.commit_policies(plan, self._context) is not True:
            raise ValueError('committed policy binding unknown')
        self.release_navigation_reservation()
        return True
