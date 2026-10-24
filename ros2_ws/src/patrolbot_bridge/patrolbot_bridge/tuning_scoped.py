"""Exact-job scoped executor using existing lifecycle/motion authorities.

The ROS transport must attest the maintenance/cache protocol before this backend
advertises support. Parameter acceptance alone is never that attestation. Every
failure after possible mutation retains ownership for explicit recovery.
"""

from .tuning_activation import AdmissionRefused, ReloadResult, evaluate_evidence
from .tuning_backend import LifecycleBackend, NODES, matching, parameter_plan
from .tuning_lease import context


MANAGER_FOR = {'teleop_velocity_smoother': 'lifecycle_manager_script',
               'amcl': 'lifecycle_manager_localization',
               'controller_server': 'lifecycle_manager_navigation',
               'velocity_smoother': 'lifecycle_manager_navigation',
               'behavior_server': 'lifecycle_manager_navigation'}
PRODUCERS = ('navigation', 'physical', 'web', 'switch', 'undock', 'safety')


class ScopedLifecycleBackend(LifecycleBackend):
    def __init__(self, *args, owner, **kwargs):
        super().__init__(*args, **kwargs)
        self.owner = owner
        self._context = None
        self._reserved = []
        self._local_reserved = False
        self._completed = None
        self._proof = None

    @property
    def plan_support(self):
        capabilities = self.transport.scoped_capabilities()
        return capabilities.get('support', {}) if capabilities.get('ready') is True else {}

    @property
    def plan_implementation(self):
        capabilities = self.transport.scoped_capabilities()
        return capabilities.get('implementation', 'unverified') if capabilities.get('ready') is True else 'unverified'

    @property
    def plan_policy_support(self):
        capabilities = self.transport.scoped_capabilities()
        return capabilities.get('ready') is True and capabilities.get('policy_support') is True

    @property
    def approved(self):
        return self.enabled and self.transport.scoped_capabilities().get('ready') is True

    def _job(self):
        job = self.store.latest_job()
        if not isinstance(job, dict) or job.get('state') not in ('pending', 'running'):
            raise ValueError('durable exact-job ownership required')
        plan = job.get('execution_plan')
        if not isinstance(plan, dict) or job.get('plan_hash') != plan.get('plan_hash'):
            raise ValueError('durable reviewed execution plan required')
        value = context({'job_id': job['job_id'], 'digest': job['digest'], 'plan_hash': job['plan_hash']})
        return job, plan, value

    def preflight(self, revision, digest):
        state = self.store.load_revision(revision)
        if state['meta']['generated_digest'] != digest:
            raise ValueError('immutable revision digest mismatch')
        parameter_plan(state, self.store.index)
        # No selected-map check here. Only a plan that actually resets AMCL/map
        # needs a map-consumption step; a missing map is not a speed-only veto.

    def reserve_navigation(self):
        job, plan, value = self._job()
        if self._context is not None:
            raise AdmissionRefused('backend already owns an unsettled maintenance reservation')
        self._context = value
        if not self.owner.reserve_tuning(job['job_id']):
            self._context = None
            raise AdmissionRefused('local lifecycle or hardware mutation owner busy')
        self._local_reserved = True
        peers = list(PRODUCERS)
        peers.extend(sorted({MANAGER_FOR[node] for node in plan['nodes'] if node in MANAGER_FOR}))
        try:
            for peer in peers:
                # Journal each *attempt* before remote admission. A timeout can
                # mean ownership was accepted; never erase it on an exception.
                current = self.store.get_job(job['job_id'])
                attempts = list(current.get('reservation_attempts', [])) + [peer]
                self.store.record_job(dict(current, reservation_attempts=attempts))
                self.transport.maintenance(peer, 'acquire', value)
                self._reserved.append(peer)
        except AdmissionRefused:
            # A definitive peer refusal can unwind only the peers acknowledged
            # so far. A release timeout propagates unknown to the coordinator.
            self.release_navigation_reservation()
            raise

    def release_navigation_reservation(self):
        if self._context is None:
            return
        for peer in reversed(self._reserved):
            self.transport.maintenance(peer, 'release', self._context)
        self._reserved.clear()
        if self._local_reserved:
            if not self.owner.release_tuning_reservation(self._context['job_id']):
                raise RuntimeError('local maintenance release was not acknowledged')
            self._local_reserved = False
        self._context = None

    def _admission(self):
        allowed, reason = evaluate_evidence(self.probe())
        if not allowed:
            raise ValueError(reason)
        if self._context is None or not self._local_reserved:
            raise ValueError('exclusive local maintenance is not owned')
        for peer in self._reserved:
            self.transport.maintenance(peer, 'inspect', self._context)

    def _bind_plan(self, plan, revision, digest, job_id):
        from .tuning_plan import canonical_digest
        _, journalled, value = self._job()
        if (self._context != value or value['job_id'] != job_id or value['digest'] != digest
                or plan != journalled or plan['revision'] != revision
                or plan['hardware_generation'] != canonical_digest(self.generation())):
            raise ValueError('reviewed execution context changed')
        # Recompile the actual immutable old/new delta: never trust a stored node
        # list or hash as authority to write arbitrary ROS parameters.
        old = self.store.load_revision(plan['base_revision'])
        new = self.store.load_revision(revision)
        if old['meta']['generated_digest'] != plan['base_digest'] or new['meta']['generated_digest'] != digest:
            raise ValueError('reviewed artifact digest changed')
        before, after = parameter_plan(old, self.store.index), parameter_plan(new, self.store.index)
        expected = {node: {key: val for key, val in values.items() if val != before[node].get(key)}
                    for node, values in after.items()}
        expected = {node: values for node, values in expected.items() if values}
        steps = plan['operations']
        if (plan['nodes'] != sorted(expected) or len(steps) != len(expected)
                or {step['node']: step['parameters'] for step in steps} != expected
                or any(step['kind'] not in ('parameters', 'lifecycle') for step in steps)):
            raise ValueError('reviewed scope differs from compiled dependency closure')
        return after

    def reload_plan(self, plan, revision, digest, *, job_id):
        try:
            self._bind_plan(plan, revision, digest, job_id)
            self._admission()
            self._reload_generation = self.generation()
            self._completed = None
            self._proof = None
            # Staging only affects cache state under the acknowledged hold. The
            # transport verifies exact candidate policy/identity; it must never
            # make DESIRED ordinary startup or arm any producer.
            self.transport.stage_policies(plan, self._context)
            for step in plan['operations']:
                self._admission()
                node = step['node']
                if node not in NODES:
                    raise ValueError('unallowlisted scoped node')
                if step['kind'] == 'parameters':
                    self.transport.set(node, step['parameters'])
                else:
                    state = self.transport.state(node)
                    if state not in (1, 2, 3):
                        raise ValueError('unknown/transitional lifecycle state')
                    if state == 3:
                        self.transport.scoped_transition(node, 'DEACTIVATE', self._context)
                    if state in (2, 3):
                        self.transport.scoped_transition(node, 'CLEANUP', self._context)
                    self.transport.set(node, step['parameters'])
                    self.transport.scoped_transition(node, 'CONFIGURE', self._context)
                    if state == 3:
                        # Native manager decides whether activation is allowed;
                        # no navigation-ready/seed requirement is imposed here.
                        self.transport.scoped_transition(node, 'ACTIVATE', self._context)
                current = self.store.get_job(job_id)
                self.store.record_job(dict(current, completed_nodes=list(current.get('completed_nodes', [])) + [node]))
            if plan['seed_required']:
                # This hook journals a distinct new-seed barrier before release;
                # it is never called for unrelated changes or used to replay pose.
                self.transport.require_operator_seed(plan, self._context)
            self._admission()
            if self.generation() != self._reload_generation:
                raise ValueError('hardware generation changed during scoped mutation')
            self._completed = (job_id, revision, digest, plan['plan_hash'])
            return ReloadResult('ok', 'Scoped consumers updated; verification pending.')
        except Exception as exc:
            return ReloadResult('unknown', 'Scoped lifecycle/cache outcome unknown: ' + type(exc).__name__)

    def verify_plan(self, plan, revision, digest, *, job_id):
        from .tuning_plan import canonical_digest
        expected = self._bind_plan(plan, revision, digest, job_id)
        if self._completed != (job_id, revision, digest, plan['plan_hash']):
            raise ValueError('reviewed execution did not complete')
        self._admission()
        actual = {node: self.transport.get(node, sorted(values)) for node, values in expected.items()}
        if any(set(actual[node]) != set(values) or any(not matching(value, actual[node][key])
                   for key, value in values.items()) for node, values in expected.items()):
            raise ValueError('whole revision parameter mismatch')
        # Actual cache/policy/graph evidence is collected by the trusted native
        # transport. These hooks are intentionally mandatory, including for
        # unchanged nodes; no successful HTTP/SetParameters substitutes for them.
        cache_evidence = self.transport.verify_consumers(expected, plan, self._context)
        invariants = self.transport.verify_invariants(self._context)
        self._admission()
        if self.generation() != self._reload_generation:
            raise ValueError('generation changed during scoped verification')
        if cache_evidence.get('verified_nodes') != sorted(NODES):
            raise ValueError('native consumed-cache evidence incomplete')
        proof = dict(invariants, complete=True, job_id=job_id, revision=revision, digest=digest,
                     plan_hash=plan['plan_hash'], hardware_generation=canonical_digest(self.generation()),
                     scopes=['bringup', 'navigation'], parameters=actual,
                     verified_nodes=cache_evidence['verified_nodes'],
                     verified_policies=cache_evidence.get('verified_policies'),
                     cache_evidence=cache_evidence)
        self._proof = proof
        return proof

    def committed_plan(self, plan, *, job_id):
        if (self._proof is None or self._proof['job_id'] != job_id
                or self._proof['plan_hash'] != plan['plan_hash']
                or self.generation() != self._reload_generation):
            raise ValueError('reviewed verification is stale or absent')
        if self.transport.commit_policies(plan, self._context) is not True:
            raise ValueError('committed cache binding is unknown')
        self._admission()
        self.release_navigation_reservation()
        return True
