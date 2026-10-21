# Live observations and desired tuning are separate

The `readback` member of a tuning snapshot is a read-only observation lane. It
works before configuration-store initialization and while runtime activation is
disabled. It never creates a revision, sets a parameter, changes a lifecycle,
publishes velocity or supplies a localization pose.

Shape: `source: "ros_parameters"`, `age_s`, `hardware_session_generation`,
`values` (logical ID to observed scalar or null), and `limits` (observed final
smoother limits or null). Only fixed source nodes from the reviewed registry are
queried. Missing nodes/parameters remain null; packaged YAML is never substituted
for an observation. Readbacks may expose out-of-policy drift, but such drift may
not raise a display/policy ceiling. Observations expire after three seconds and
cannot cross hardware sessions. Their age includes the start of the whole query
batch, not merely the final service reply.

This observation does **not** establish `active_known` or a committed revision.
Those still require exact revision/digest/parameter verification. That read-only
verification no longer depends on permission to activate nodes.

Desired saves require administrator-derived signed authorization, an initialized
store and a fresh SBC link. They require neither motors enabled, localization,
stationarity nor navigation idle. Link evidence is checked again after writer
admission. Saves do not activate the desired values. Applying/redeploying retains
the existing safe-idle/lifecycle ownership and explicit localization-seed gates.

`write_disabled_reason` reports the precise store/key/link refusal. An
uninitialized snapshot remains readable with null session/revision and false
writable/activation flags. Enabling editing still requires separately reviewed
private-key provisioning and initialization against the actual deployed baseline.
