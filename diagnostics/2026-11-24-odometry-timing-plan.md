# Odometry continuity timing fix

## Evidence and scope

The deployed bridge reported real `heading-discontinuity` events during operation.
None of the passive captures included a heading rejection. A 680-frame capture
did reproduce the related position failure: consecutive transport messages about
44 ms apart sometimes repeat the same pose, then report a hardware pose step
accumulated over multiple transport intervals. The replay reported 36
position-discontinuity and 73 linear-speed-limit rejections. A 29.2 mm step
after 44.1 ms exceeded a 28.2 mm receive-time allowance; measured speed peaked
at 0.33 m/s against the separate 0.30 m/s check. The capture did not carry SIP
times, so a source-timed test reconstructs the reported step, not the entire
capture. Heading timing is plausible, not confirmed; this is not a fix for
genuine yaw jumps or reported overspeed.

## Implementation

1. Verify the supported ARIA API or standard-packet callback for the identity
   and monotonic timing of the *motor odometry update*. Capture it consistently
   with the pose and twist used in each SBC navigation frame. Do not substitute
   the SBC send-loop clock, a timestamp updated by unrelated packets, or pose
   equality for the controller update identity.
2. Add backward-compatible, documented wire metadata. Use the difference
   between successive hardware update times for the bridge's continuity
   allowance; use Pi receipt and freshness independently. Repeating a
   controller update must not move the validation reference or manufacture
   freshness. A genuinely new stationary update may have the same pose.
   Handle malformed, regressed, inconsistent, and stale metadata
   conservatively; keep explicit epoch resets and preserve old SBC behavior.
   Avoid creating a scan outage merely because several transport frames carry
   the same still-fresh odometry update.
3. Keep hard twist limits, epoch checks, and safety timeouts unchanged. Include
   rate-limited rejection evidence (measured step, elapsed update time, and
   limit, where available) to distinguish a real heading jump from a timing
   problem if the alert recurs.
4. Add replay/regression coverage for repeated source updates, valid delayed or
   batched pose steps, wrapped yaw, an actual yaw jump, invalid metadata,
   source time across an epoch change, and the legacy no-metadata path. Run the
   relevant bridge and hardware host tests. List any build or real-hardware
   acceptance still needed in the PR.

No live changes, deployment, threshold changes, or robot motion are part of
this PR. A controlled hardware acceptance run must be separately scheduled.

## Implementation findings

- The [ARIA motor packet path](https://github.com/reedhedges/AriaCoda/blob/master/src/ArRobot.cpp)
  accepts `0x32`/`0x33` and stores their receive time minus kinematics delay
  in `getLastOdometryTime()`, alongside its pose/twist update. The receive
  time and motor packet IDs were also verified in the cached 32-bit SBC
  headers/library. `getLastPacketTime()` accepts unrelated packets and
  `getLastOdometryTime()` is seeded on connection; neither is sufficient
  alone. The new observer corroborates the processed time against an actual
  SIP under the ARIA lock. Unlike the opt-in encoder diagnostic, it does
  not request `0x90` traffic and does not read or consume packet payloads.
- The versioned [wire extension](../docs/reference/wire-protocol.md#motor-odometry-sample-timing-optional-protocol-2-capability)
  ties source time to the existing odometry origin. Pi receipt elapsed caps
  the continuity interval even if source time claims more motion time;
  acquisition age and the local queue count against freshness. A duplicate
  cannot become a new baseline, refresh odometry, restore a withdrawn
  transport, or starve independently fresh scans while the pose is fresh.
- ARIA timestamps represent SBC receipt of SIPs, not a hardware clock. A
  same-millisecond collision is not evidence of a new observation; a
  conflicting payload for that timestamp is rejected. TCP transit is not
  measured. The live SBC build and controlled acceptance still need to
  confirm SIP cadence/age over the actual serial/TCP connection, clean
  stationary and moving streams, and scan/TF freshness. No robot was
  contacted for this PR.
