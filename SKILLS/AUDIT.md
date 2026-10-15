# PatrolBot Audit and Status-Tracking System

> October 15, 2026 policy: current STATUS/OPEN/COMPLETED and sanitized release
> manifests are tracked in Git. Raw historical evidence and private archives
> remain off-device/private. The old keep-two device snapshot policy is retired;
> temporary testing/transaction copies require verified off-device archival.
> GitHub describes tracker state; newer contradictory artifact evidence must be
> recorded here and proposed as a tracker correction, never silently discarded.

> Established 2026-08-01, when the scattered `TODO.md`, `*-TODO.md`, `*-PLAN.md`
> and eight parallel audit reports were consolidated. `audits/` is the single
> source of truth for project status. Read this before recording status
> anywhere.

---

## 1. Layout

```text
TODO.md                      pointer only — links, counts, robot state, next 3 things
audits/
  STATUS.md                  one-page index: every item, one row, one state
  COMPLETED.md               finished work: problem → implementation → evidence
  OPEN.md                    unfinished work: problem → progress → remaining → blocker
  2026-07-pb-audit/          original PB-AUDIT findings and raw evidence
  backlog/                   deferred, unscheduled work
  history/                   archived work log and session narratives
```

**`audits/` has exactly three top-level files.** Do not add a fourth. A new
audit campaign gets its own dated subdirectory (`audits/<YYYY-MM>-<slug>/`)
alongside `2026-07-pb-audit/`, and its findings are indexed in `STATUS.md` like
everything else.

## 2. What goes where

| File | Holds | Update when |
|---|---|---|
| `audits/STATUS.md` | Index tables: PB-AUDIT findings, other workstreams, counts | Any item changes state |
| `audits/COMPLETED.md` | Closed items, in full | An item closes |
| `audits/OPEN.md` | Everything unfinished, ordered by what actually gates progress | Progress, new item, or new blocker |
| `TODO.md` | Links table, counts, current robot state, next three things | Any of those four change |

`TODO.md` carries **no detail**. If you catch yourself writing a paragraph
there, it belongs in `OPEN.md`.

### Supporting directories

- **`audits/<campaign>/`** — original findings and raw evidence.
  **Append-only history.** Never rewrite a dated evidence record to match
  current reality, even when it has gone stale (old reports cite Pi backup paths
  that were since pruned — that is correct and deliberate). Corrections go in
  `OPEN.md` or `COMPLETED.md`, which say what superseded what.
- **`audits/backlog/`** — plans and backlogs that are not scheduled and block
  nothing. Each still gets a row in `STATUS.md`.
- **`audits/history/`** — archived, banner-marked, not maintained. Things move
  here when they are superseded, never deleted.

## 3. States

Three, and only three:

- ✅ **Done** — the original problem is resolved for its stated scope **and** the
  resolution was verified on the real system.
- 🟨 **In progress** — remediation exists and is at least partly verified, but a
  required acceptance step or dependency is outstanding.
- 🔲 **Not started** — investigated and written up, nothing built.

**Nothing is marked Done on a passing test suite alone.** Anything that changes
robot behaviour needs live evidence from the robot. Quote the actual readback,
image digest, or goal result. A green offline suite plus "no hardware claim is
made" is 🟨, not ✅.

Partial closure is normal and should be stated plainly: PB-AUDIT-009's
auto-enable removal is done and live-verified while its enabled-motor E-stop
cutoff is untested, so the finding is 🟨 with the closed half written up in
`COMPLETED.md` and the open half in `OPEN.md`, cross-linked.

## 4. Writing an entry

### `COMPLETED.md` — four parts

1. **Problem.** What was actually wrong, with the observation that proved it.
2. **Implementation.** *How* it was built, not just that it was. Name the files,
   services, states and parameters.
3. **Evidence.** The live proof. Image tag, source digest, commit SHA, goal
   UUID, measured distance, parameter readback, test counts.
4. **Tradeoffs, rejected alternatives, and residue.** The part that stops the
   next person re-breaking it. Record what was tried and reverted and why —
   e.g. the 0.20 m/s reverse that re-latched charge and cut the motors 0.24 m
   in, or AMCL random-particle injection set to `0.0` at the cost of
   kidnapped-robot recovery.

### `OPEN.md` — four questions, always all four

1. **Problem.** Same standard as above.
2. **Progress.** What exists now, and what it has been verified against.
3. **Remaining.** Concrete, ordered, with the measurement or test named.
4. **Blocked on.** Be specific. Most blockers here are physical — a supervised
   motion session, a tape measure, a maintenance-port window, a real electrical
   signal, hardware procurement. Say which.

Order `OPEN.md` by what actually gates progress, not by finding number.
Deployment and provenance items come first, because until they are resolved the
running robot and the reviewed source are not the same thing.

## 5. Non-negotiables

- **Carry do-not-do warnings forward verbatim** whenever you touch an entry.
  These are landmines that already cost a session each:
  - do not widen `max_position_variance` — it also gates general navigation;
  - do not flip `allow_reversing` — mutually exclusive with the enabled
    `use_rotate_to_heading`;
  - do not install a graceful-shutdown daemon with no real power-fail input;
  - do not change footprint and inflation together;
  - do not restore DDS discovery to `SUBNET`;
  - do not raise `SCAN_RANGE_MIN` to make the stop box see nearer.
- **Do not guess a physical value to unblock software.** Uncommissioned paths
  must keep failing closed. `HARDWARE_PROFILE_UNCOMMISSIONED` is working as
  designed.
- **Cite rollback.** Every deployment entry names its backup path and the
  previous image or digest.
- **Update counts in `STATUS.md` and `TODO.md` in the same edit.** They are the
  first thing anyone reads and the first thing to rot.
- **Update in the same response as the work.** Never leave the audit files
  stale, and never batch status updates for later.

## 6. Contradiction handling

When new evidence contradicts an existing record — and it has, repeatedly:

1. Leave the original record where it is. Do not silently rewrite it.
2. Write the correction in the current file (`OPEN.md` or `COMPLETED.md`),
   stating what was believed, what is now known, and the evidence that settled
   it.
3. If the correction re-opens something already marked ✅, move it back to 🟨 in
   `STATUS.md` and say why.

Worked example, preserved in `COMPLETED.md`: a `session_generation` advance
during motor enable was blamed on a base PWR-ON cycle, and
`allow_session_advance` was added to work around it. The SBC journal later
showed PID 1341 running continuously with exactly two TCP sessions all day while
thirteen `MOTOR_ENABLE` operations completed — nowhere near a reconnect. The real
cause was a bridge startup artifact. The original diagnosis is struck through
and superseded in place, not deleted, and the workaround was removed.

## 7. Cross-references

- `docs/known-gaps.md` points at `audits/STATUS.md` and `audits/OPEN.md`. Keep
  it consistent when an item closes.
- Relative links between the three files must resolve. Check them after any
  move.
