---
title: Documentation Standards
description: The standards PatrolBot documentation is written against — Diátaxis structure, REP-conformant technical content, RFC 2119 normative language, and the rule that the docs describe the deployed system and nothing else.
---

# Documentation Standards

This page defines how PatrolBot documentation is written. It is the specification the docs
are checked against; a page that violates a **MUST** here is a defect regardless of how well
it reads.

The key words **MUST**, **MUST NOT**, **REQUIRED**, **SHOULD**, **SHOULD NOT**, and **MAY** in
this document are to be interpreted as described in
[RFC 2119](https://www.rfc-editor.org/rfc/rfc2119) and
[RFC 8174](https://www.rfc-editor.org/rfc/rfc8174), when and only when they appear in all
capitals.

## The first rule: docs describe the deployed system

PatrolBot documentation describes **how the robot works as deployed**. It is not a status
report, a plan, or a work log.

Documentation **MUST NOT** contain:

| Excluded | Why |
|---|---|
| Audit identifiers (`PB-AUDIT-nnn`), audit findings, or links into `audits/` | Audit records track *work*. Docs describe *behavior*. A reader asking "what does the bridge publish?" is not served by a finding ID. |
| Work-in-progress, planned features, or roadmap items | Anything unbuilt has no observable behavior to document. It dates the page the moment priorities move. |
| "Not yet", "coming soon", "still open", "pending commissioning", "TODO" | These are status claims. They go stale silently and are the primary source of doc drift. |
| Dates that mean "as of" (`updated 2026-07-23`, "live-audited") | A page that needs a freshness date is asserting something time-varying, which belongs in a record, not a reference. |
| Remediation steps for known defects | If it is broken, either fix it or document the *behavior* the reader will observe. |

Documentation **MUST** state constraints as present-tense system behavior with no remediation
attached. The distinction is not cosmetic:

!!! failure "Not documentation — this is a work item"
    Commission the undock hardware profile (`undock_profile.conf`). Until then a production
    undock is refused. Unmeasured: `CHARGE 0` release timing, E-stop signal semantics.
    Tracked in `audits/OPEN.md`.

!!! success "Documentation — this is observable behavior"
    An undock request is refused with `HARDWARE_PROFILE_UNCOMMISSIONED` unless
    `undock_profile.conf` supplies a measured hardware profile. The profile is site-specific
    and is not shipped with the image.

Where the project's *status* is genuinely the question, the answer is the audit system
described in the repository's contributor instructions — not this site. The two **MUST NOT**
cross-reference each other.

This is a structural constraint, not a stylistic preference. The audit records are a **local
working copy**: their index and history files are not part of the published repository, and a
reader of this site is not expected to have them. A link or path reference into that material
is therefore a dangling reference by construction — it resolves for one person on one machine
and for nobody else. The documentation **MUST** be complete and self-contained without it.

## Structure: Diátaxis

Every page **MUST** serve exactly one of four purposes, and **MUST NOT** mix them.

| Mode | Answers | Voice | Lives under |
|---|---|---|---|
| **Tutorial** | "Get me running the first time" | Second person, imperative, guaranteed-to-work path | `getting-started/` |
| **How-to** | "Perform this task" | Imperative, goal-oriented, assumes competence | `deployment/`, `operations/`, `troubleshooting/` |
| **Reference** | "What exactly is this?" | Third person, declarative, exhaustive, dry | `ros2/`, `reference/`, `devices/`, `packages/` |
| **Explanation** | "Why is it built this way?" | Discursive, admits trade-offs and history | `architecture/`, `internals/` |

The most common violation in this project is **reference pages that drift into explanation**
— a topic table interrupted by three paragraphs on why the TCP seam exists. Move the prose to
`architecture/` and link to it.

A tutorial **MUST NOT** explain rationale beyond one sentence. A reference page **MUST NOT**
contain a numbered procedure. An explanation page **MUST NOT** be the only place a fact is
recorded.

## Required page shape

Every Markdown file under `docs/` **MUST** begin with YAML frontmatter:

```yaml
---
title: Communication Architecture
description: How the SBC and Pi exchange telemetry and commands over a single TCP socket, the line grammar, and why the two never share a ROS graph.
---
```

- `title` **MUST** be present and **MUST** match the `nav:` entry in `mkdocs.yml`.
- `description` **MUST** be one sentence, **MUST** state what the page covers, and **MUST NOT**
  exceed roughly 200 characters. It is used for search and social previews.
- The first heading **MUST** be a single `# H1` matching `title`.
- Headings **MUST NOT** skip levels.

Every file **SHOULD** carry an SPDX identifier as the last line:

```markdown
<!-- SPDX-License-Identifier: Apache-2.0 -->
```

## Technical content: REP conformance

PatrolBot is a ROS 2 system. Where a ROS Enhancement Proposal defines a convention, the docs
**MUST** use it and **MUST NOT** invent a parallel vocabulary.

| REP | Binds |
|---|---|
| [REP-103](https://www.ros.org/reps/rep-0103.html) | Units (SI, metres, radians, seconds) and axis orientation (x forward, y left, z up). Every quantity in a table **MUST** carry its unit. |
| [REP-105](https://www.ros.org/reps/rep-0105.html) | Frame names and the `map` → `odom` → `base_link` chain. Frame names **MUST** be written exactly as published, in `code` formatting. |
| [REP-144](https://www.ros.org/reps/rep-0144.html) | Package naming. Package names **MUST** be written in their real underscored form (`patrolbot_bridge`, not "the bridge package" in a reference table). |
| [REP-149](https://www.ros.org/reps/rep-0149.html) | `package.xml` field names, when documenting manifests. |

Interface names **MUST** be fully qualified on first use in a reference table:
`geometry_msgs/msg/Twist`, not `Twist`.

## Normative language

Operational and configuration docs **MUST** use RFC 2119 keywords, capitalized, for anything
load-bearing. Any page using them **MUST** include the boilerplate paragraph from the top of
this page.

This matters more here than in most projects. PatrolBot carries settings that look like
removable defaults but prevent specific crashes. Prose advice gets deleted; a **MUST NOT**
does not:

!!! failure "Deletable"
    You should keep `base_shift_correction` set to False, since changing it has caused
    problems with TF during reconnects.

!!! success "Normative"
    `base_shift_correction` **MUST** remain `False`. A reconnect opens a TF gap that aborts
    the Nav2 container when correction is enabled.

Every load-bearing value **MUST** be documented with the failure it prevents, not merely its
value. If the failure mode is unknown, the value **MUST NOT** be described as load-bearing.

## Diagrams

- Diagrams **MUST** be Mermaid in fenced ```mermaid blocks, never images of text.
- Every diagram **MUST** be preceded or followed by prose carrying the same information.
  A diagram is an aid, never the sole record of a fact.
- Node labels **MUST** use real identifiers (`/cmd_vel_nav`, `patrolbot_hw_server`).

## Paths, hosts, and commands

- On-robot absolute paths **MUST** follow the layout the machine actually uses and **SHOULD**
  be introduced with the owning machine: "on the Pi, `/opt/patrolbot_ws`".
- Paths that vary by site (home directories, mount points, backup roots) **MUST** be shown as
  the deployed value with a note that it is site-local, per
  [FHS](https://refspecs.linuxfoundation.org/fhs.shtml) conventions.
- Every command block **MUST** make clear which machine it runs on. Prefer a preceding
  sentence over a comment inside the block.
- Command blocks **MUST** be copy-pasteable: no leading `$`, no interleaved output. Output, when
  shown, goes in its own block.

## Cross-references

- Internal links **MUST** be relative Markdown paths to the `.md` file (`../ros2/topics.md`),
  so `mkdocs build --strict` catches breakage.
- A fact **SHOULD** have exactly one authoritative page. Other pages link to it rather than
  restating it. Restated facts drift independently — this is how the laser TF, the map
  resolution, and the launch mode each went stale.
- External standards **SHOULD** be linked on first reference.

## Section coverage

The site **MUST** cover at minimum the following. Sections marked *new* do not exist yet and
are required by this standard.

| Section | Mode | Covers |
|---|---|---|
| Getting Started | Tutorial | Installation, building, quickstart |
| Architecture | Explanation | Overview, software, hardware, SBC↔Pi communication, execution and data flow |
| ROS 2 Reference | Reference | Nodes, topics, services, actions, parameters, launch system |
| Devices | Reference | Sensors, actuators, controllers, interfaces |
| Packages | Reference | Per-package pages for every package in `ros2_ws/src/` and the SBC server |
| **Safety** *(new)* | Reference + Explanation | E-stop chain and recovery, velocity mux priorities, collision monitor, bumper policy, speed limits, the full command path from source to motor |
| **Operations** *(new)* | How-to | Startup and shutdown, docking and charging, undocking, teleop, running a patrol, monitoring a live robot |
| **Troubleshooting** *(new)* | How-to | Symptom → cause → resolution, organised by what the operator observes |
| **Maintenance** *(new)* | How-to | Battery care, RTC, backup rotation policy, image and log rotation |
| **Reference: Configuration** *(new)* | Reference | Every tunable in `nav2_params.yaml`, `mux.yaml`, `smoother.yaml`, and the undock profile — key, type, deployed value, and for load-bearing keys the failure prevented |
| **Reference: Wire protocol** *(new)* | Reference | The SBC↔Pi TCP line grammar as a formal reference, extracted from the architecture narrative |
| **Reference: Limitations** *(new)* | Reference | Permanent and behavioral constraints of the deployed system, stated as behavior. Replaces the former status-style gaps page. |
| Development | How-to | Workspace setup, coding standards, testing, debugging, profiling |
| Deployment | How-to | Robot deployment, Docker, network setup, remote operation, updates |
| Internals | Explanation | Repository structure, component breakdown, startup sequence, state machines, legacy components |
| Contributing | How-to | Contributing, pull requests, releases, this page |
| Glossary, FAQ | Reference | Terms and recurring questions |
| **Changelog** *(new)* | Reference | CalVer release history |

Safety, Operations, and Troubleshooting are the significant additions. Today that material is
scattered through `ros2/nodes.md`, `ros2/parameters.md`, and `development/debugging.md`, which
means an operator has to read developer reference pages to find out how to stop the robot.

## Versioning and history

- The documentation and robot deployments use [CalVer](https://calver.org/) in `YYYY.MM` form
  (for example `2026.08`). A release cut in the same month **MUST** add an incrementing
  suffix: `2026.08.1`.
- ROS package versions in `package.xml` follow their own upstream expectations and are **NOT**
  governed by this rule.
- `docs/changelog.md` **MUST** record each documentation release under its CalVer heading.
  Entries describe what changed **in the system**, not what was worked on.
- [Release Process](release-process.md) **MUST** describe cutting a CalVer release end to end:
  tagging, the image and package metadata it stamps, and the changelog entry it produces. It
  **MUST NOT** describe release readiness in terms of outstanding work.

## Commits

Documentation commits **MUST** follow
[Conventional Commits](https://www.conventionalcommits.org/):

```text
docs(ros2): document the AUX telemetry section
docs: split the wire protocol out of communication-architecture
fix(docs): correct the laser frame roll value
```

- `docs:` for documentation-only changes. A scope **SHOULD** name the section.
- `fix(docs):` where a doc change corrects a factual error a reader could have acted on.
- Commits touching robot code follow the repository's existing style and are unaffected.

## Review checklist

A documentation change is ready when all of the following hold.

- [ ] Frontmatter has `title` and a one-sentence `description`; `title` matches `mkdocs.yml`.
- [ ] The page serves exactly one Diátaxis mode.
- [ ] No audit IDs, status claims, roadmap items, or "as of" dates.
- [ ] Units and frame names conform to REP-103 and REP-105; message types fully qualified.
- [ ] Load-bearing values carry RFC 2119 keywords **and** the failure they prevent.
- [ ] Every command block states its machine and is copy-pasteable.
- [ ] Every diagram is Mermaid and is mirrored in prose.
- [ ] Facts are stated once and linked, not duplicated.
- [ ] `mkdocs build --strict` passes with no warnings.
- [ ] Commit message follows Conventional Commits.

<!-- SPDX-License-Identifier: Apache-2.0 -->
