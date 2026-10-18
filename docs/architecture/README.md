# Training, simulation and visualization architecture

Status: ARCH-01 through ARCH-09 independently accepted. ARCH-10's structural/correction subset is accepted at `f1bb879`; HC-11 final regression is independently accepted at evidence commit `66dc43d` on that frozen effective source (717/717 headless, 780/780 Release CTest entries). HC-38 to HC-43 and HC-20 are closed; earlier accepted decisions remain. HC-07 Viewer/manual/performance/joint gates still prevent complete ARCH-10/final-program acceptance. HC-44 records nonblocking future fingerprint hardening; HC-34/35/36 and other assigned carryovers remain. Source captures are dated evidence, with later documentation overlays identified explicitly.
HC-07 diagnostic evidence is independently accepted at `465b7af`; it remains open on HC-45's failed performance,
HC-06 combat prerequisites, HC-02's J6 and a human-operated session. The [review](validation/hc-07-review-2026-10-17.md#6-interaction-subset-and-hc-51-acceptance-at-5c80892) accepts the sampled window-input mouse/replay/resize subset at `2a09756` and closes HC-51 at `5c80892`; other panel/camera/option variants remain unverified. HC-52 is nonblocking resize-helper completeness work; HC-46 remains an option issue.
HC-47 helper-check logic is independently accepted and closed at `d6bfb54`; HC-48/49/50 remain closed at `1e9433b`. V2 live-path validation is retained before future automated-gate reuse only, not claimed by pure checks. HC-45/HC-07 stay open; see [acceptance](validation/hc-45-review-2026-10-17.md#7-hc-47-version-2-correction-acceptance-at-d6bfb54).
Planning date: 2026-10-16 (America/Denver).
Original planning audit baseline: main, d344e2b (refreshed after M8 was committed during the audit).
Working-tree condition: existing terminology edits are present. M8 generic is committed and awaits independent acceptance.

This directory is the single entry point for the three-domain architecture, its directory policy and the staged migration. New cross-domain architecture plans belong here. Existing SDK references, accepted feature contracts and historical evidence remain authoritative until their registered relocation is completed.

## Initial planning assessment (2026-10-16, d344e2b)

The following is the original planning audit, before ARCH-01 through ARCH-08. At that baseline the repository had working architectural boundaries but did not satisfy the intended three-domain directory and dependency policy. The current phase verdict is above; accepted migration records supersede these historical observations.

- The existing native architecture check accepts 19 modules and 367 source/public-header files, including optional rendering branches. No native module dependency cycle was found.
- A scoped scan of 16 Python modules finds no eager literal-import cycle, but finds two deferred cycles: fsim with fsim.training, and fsim.agent with fsim.world.
- Simulation, training and visualization responsibilities are spread across flat source directories and the SDK assembly. The directory src/world is visualization, while src/session/World is authoritative simulation.
- The public SDK assembly currently contains both simulation facades and training implementations. Moving files without resolving that dependency would create a training-to-SDK-to-training cycle.
- A native DAG does not establish clean Python imports, narrow include visibility, package independence or behavior preservation after migration.

See the [source audit](current-state/audit.md) and [machine-readable evidence](current-state/dependency-audit.json). These are static findings, not new simulation or GPU acceptance results.

## Document map

| Category | Canonical document | Purpose |
| --- | --- | --- |
| Current state | [Audit](current-state/audit.md) | Facts, graph, issues, evidence and limits |
| Design | [Boundaries](design/boundaries.md) | Responsibilities, contracts, permitted dependencies and data flow |
| Design | [Directory layout](design/directory-layout.md) | Exact destination roots and ownership rules |
| Simulation | [Simulation plan](modules/simulation/plan.md) | Runtime separation, model work and simulation acceptance |
| Training | [Training plan](modules/training/plan.md) | Episode/batch/gateway separation and training acceptance |
| Training review | [M8 generic review](modules/training/reviews/2026-10-16-m8-generic-d344e2b.md) | Independent results, five correction items and remaining acceptance |
| Training review | [M8 correction and test scoping review](modules/training/reviews/2026-10-16-m8-3802e7a-test-scoping.md) | Fast recheck, remaining M8 gaps and selector coverage findings |
| Visualization | [Visualization plan](modules/visualization/plan.md) | Scene, Viewer, Vision, VFX and GPU acceptance |
| Migration | [Execution roadmap](migration/roadmap.md) | Ordered work packages, exclusive windows and rollback |
| Migration | [Source path map](migration/path-map.csv) | Current-to-target source ownership |
| Migration | [Document register](migration/document-register.csv) | Authority, references and future relocation |
| Validation | [Acceptance matrix](validation/acceptance.md) | Structural, behavioral, compatibility and visual gates |
| Validation | [Target dependency policy](validation/target-dependencies.json) | Closed dependency DAGs for the intended architecture |
| Validation | [Document checker](validation/check_documents.py) | Validate this planning package without a native build |
| Validation | [Source audit utility](validation/audit_sources.py) | Refresh the scoped native/Python evidence with an explicit date |
| Validation | [ARCH-03 record](validation/arch-03-foundation-and-leaves.md) | Foundation and simulation leaves moved to their destinations, verification and the dated [baseline](validation/arch-03-baseline.json); include-path narrowing (HC-12) with its [include environments](validation/arch-03-include-views.json) |
| Validation | [ARCH-04 record](validation/arch-04-simulation-runtime.md) | Simulation runtime, modeled DMS and the lower simulation facade: ownership by state, targets, verification, the dated [baseline](validation/arch-04-baseline.json) and [include environments](validation/arch-04-include-views.json) |
| Validation | [ARCH-05 record](validation/arch-05-training-extraction.md) | Training extraction: VecEnv and the native episode owner (fsim_env), the mission implementation (fsim_training_mission) below the SDK, verification, the dated [baseline](validation/arch-05-baseline.json) and [include environments](validation/arch-05-include-views.json) |
| Validation | [ARCH-06 record](validation/arch-06-visualization.md) | Visualization in src/visualization, read-only scene/UI/Vision inputs, the Vision adapter, the Viewer and demo applications, before/after visual evidence, the dated [baseline](validation/arch-06-baseline.json) and [include environments](validation/arch-06-include-views.json) |
| Validation | [ARCH-07 record](validation/arch-07-sdk-and-python.md) | The thin SDK assembly in src/adapters/sdk, the extension modules' adapters, the private Python packages without import cycles, the camera core without the VecEnv, verification, the dated [baseline](validation/arch-07-baseline.json) and [include environments](validation/arch-07-include-views.json) |
| Validation | [ARCH-08 record](validation/arch-08-applications-tests-and-packaging.md) | The applications in src/applications, the demo publisher process and the stream's control-level names, tests, examples and tools by category with the gate's layout rule, verification, the dated [baseline](validation/arch-08-baseline.json) and [include environments](validation/arch-08-include-views.json) |
| Validation | [HC-45 performance comparison](validation/hc-45-performance-comparison.md) | The 4K gate on an idle host, the current Viewer against the one from before the migration (the same failures; neither a regression nor equivalence established), hypotheses for the failures and the `gpu worst` output ([evidence](validation/hc-45-evidence.json)) |
| Validation review | [HC-45 independent review and correction rechecks](validation/hc-45-review-2026-10-17.md) | HC-48/49/50 closed at 1e9433b; HC-47 check logic closed at d6bfb54 with future v2 live-path reuse qualification; HC-45 performance remains open |
| Validation | [HC-07 Viewer and joint verification](validation/hc-07-viewer-and-joint.md) | Viewer scenarios, reconnection, input-driven demo controls and the 4K performance gate ([evidence](validation/hc-07-evidence.json)); the combat cases' missing prerequisites |
| Validation review | [HC-07 independent review](validation/hc-07-review-2026-10-17.md) | Diagnostic subset accepted at 465b7af; HC-45 performance failure, combat prerequisites and interaction gaps keep HC-07 open; HC-46/47 qualified |
| Validation | [HC-11 final regression](validation/hc-11-final-regression.md) | The frozen, fingerprinted snapshot ([fingerprint](validation/hc-11-final-regression-fingerprint.json)) and each required full suite run once on it: 717/717 headless, 780/780 Release |
| Validation review | [HC-11 independent review](validation/hc-11-review-2026-10-17.md) | Accepted at 66dc43d: exact inventory/JUnit/log matches, scope/expected internal skips qualified, HC-44 recorded; no suite repeated |
| Validation | [ARCH-10 record](validation/arch-10-final-acceptance.md) | Final structural, configured-graph, compatibility and installed-consumer evidence; the resolved carryovers; the final [baseline](validation/arch-10-baseline.json) and [provenance](validation/arch-10-provenance.json); the checklist with its pending full-suite and visual gates |
| Validation | [ARCH-09 record](validation/arch-09-architecture-authority.md) | The registered architecture documents relocated into this directory, their redirects, links, owners' edits, the authority table and the deferred MA documents; HC-33's correction registers three documents in place |
| Validation | [ARCH-02 record](validation/arch-02-neutral-contracts.md) | Contracts target, public header inventory, scenario/options separation, compatibility evidence and the dated [baseline](validation/arch-02-baseline.json) |
| Validation | [ARCH-01 record](validation/arch-01-ownership-baseline.md) | Enforced ownership/domain/Python policy, negative fixtures and the dated [baseline](validation/arch-01-baseline.json) |
| Validation review | [ARCH-01 independent review](validation/arch-01-review-2026-10-16.md) | Accepted at main 7625529; all findings closed |
| Validation review | [ARCH-02 independent review](validation/arch-02-review-2026-10-16.md) | ARCH-02 accepted at dc09e8f; HC-10 follow-up accepted at 565ef3e |
| Validation review | [ARCH-03 independent review](validation/arch-03-review-2026-10-16.md) | Accepted at ad12a44; HC-12 include narrowing and ancestor-root findings closed |
| Validation review | [ARCH-04 independent review](validation/arch-04-review-2026-10-16.md) | Accepted at 6e79873 with explicit HC-15/P022 placement adjustment; future wrapper relocation remains tracked |
| Validation review | [ARCH-05 independent review](validation/arch-05-review-2026-10-16.md) | Accepted at 58f9297; training extraction, dependency closure, installed composition parity and carryovers reviewed |
| Validation review | [ARCH-06 independent review](validation/arch-06-review-2026-10-16.md) | Accepted at 74efb49; HC-13 closed, image evidence qualified and HC-16/17 decisions retained |
| Validation review | [ARCH-07 independent review](validation/arch-07-review-2026-10-17.md) | Accepted at a59b46f; HC-21/P056 and HC-23 closed, HC-22 decision retained and HC-24 assigned |
| Validation review | [ARCH-08 independent review](validation/arch-08-review-2026-10-17.md) | Accepted at dd7dc9f; HC-32 and HC-16 closed, 30-cycle native probe stable; scheduled carryovers and final acceptance gates retained |
| Validation review | [ARCH-09 independent review](validation/arch-09-review-2026-10-17.md) | Accepted at 6bb79cc with the inspected planning/index package; HC-33 closed; HC-34/35/36 and evidence limits retained |
| Validation review | [ARCH-10 independent review](validation/arch-10-review-2026-10-17.md) | Correction/structural subset accepted at f1bb879; HC-38 to HC-43 and HC-20 closed; source snapshot/review overlay and pending final-program gates distinguished |
| Validation | [Handoff carryover register](validation/handoff-carryover-register.md) | Reconciled self-reported issues, review findings, limits, external gates and historical closures |

## Architecture authority (document register)

Every registered architecture document, with its category, owner and status (the
[document register](migration/document-register.csv); relocated by ARCH-09, 2026-10-17). A relocated document's old
path holds only a redirect where code comments, generated data or prose still name it; it is not a second authority.
Three architecture documents outside this directory are registered where they are (D23-D25, ARCH-09's HC-33
correction): each is one body at its own path, with no copy or redirect elsewhere.

| Category | Document | Owner | Status |
| --- | --- | --- | --- |
| Index | This page and its [document map](#document-map) | architecture | Canonical (D01) |
| Design | [Platform baseline](design/platform-baseline.md) | platform_architecture | Relocated from `docs/FlightSim_System_Architecture_and_Design.md`, redirect kept (D02); historical milestones and evidence preserved |
| Design | [Agent boundary (ADR-31)](design/agent-boundary.md) | contracts | Relocated from `docs/agent-boundary.md`, redirect kept (D06) |
| Design | [Simulation interface boundaries](design/simulation-interface-boundaries.md) | contracts | Relocated from `docs/superpowers/specs/` (D10); frozen compatibility constraints preserved |
| Simulation | [ADR-30 mission systems](modules/simulation/adr-30-mission-systems.md) | simulation | Relocated from `docs/mission-systems.md`, redirect kept (D03); the original MS roadmap, distinct from training M0-M8 |
| Simulation | [Vehicle Interface](modules/simulation/vehicle-interface.md) | simulation | Relocated from `docs/vehicle-interface.md`, redirect kept (D04); the public SDK reference stays in `docs/sdk` |
| Simulation | [Flight autonomy](modules/simulation/flight-autonomy.md) | simulation | Relocated from `docs/flight-autonomy.md`, redirect kept (D05); dated domain acceptance preserved |
| Simulation | [Native engagement design](modules/simulation/native-engagement-design.md) | simulation | Relocated from `docs/superpowers/specs/` (D14); native delivery does not close full MS fidelity |
| Simulation | [ADR-26 control architecture](../control-architecture.md) | simulation | Current authority, canonical in place (D23): capability contracts over the control runtime, accepted 2026-09-26 and implemented; the public reference stays in `docs/sdk` |
| Simulation | [ADR-27 rotorcraft](../rotorcraft.md) | simulation | Retained domain reference in place (D24): accepted and implemented 2026-09-26; extends ADR-26, which stays in force |
| Training | [MA training design](../superpowers/specs/2026-10-12-ma-system-training-readiness-design.md) | training | Canonical at its current path; relocation deferred to the M8 feature handoff (D07) |
| Training | [MA training implementation plan](../superpowers/plans/2026-10-12-ma-system-training-readiness.md) | training | Canonical at its current path; relocation deferred to the M8 feature handoff (D08) |
| Training | [MA training work allocation](../superpowers/plans/2026-10-12-ma-system-training-work-allocation.md) | training | Canonical at its current path; relocation deferred to the M8 feature handoff (D09) |
| History | [Maintenance history A1-A6](migration/history/README.md) | architecture_history | Archived from `docs/superpowers` (D11-D13); historical, superseded where it differs |
| History | [Build/run and installed-SDK packaging plan](../superpowers/plans/2026-10-11-build-run-sdk-packaging.md) | architecture_history | Historical evidence in place (D25): the 2026-10-11 pass delivered in `f8cfec9`, before A1; the [development guide](../development.md) and the [ARCH-08 record](validation/arch-08-applications-tests-and-packaging.md) are current |
| Reference | [SDK API reference](../sdk/README.md) | public_api_reference | Retained in `docs/sdk`; architecture documents link to it rather than duplicate it (D15) |
| Reference | [Status](../status.md), [backlog](../backlog.md) | status, backlog | Retained (D16, D17) |
| Reference | [Development guide](../development.md), [resources](../resources.md) | development_guide, resource_guide | Retained (D18, D19) |
| Reference | [Hangar](../hangar.md), [effects record](../hangar-effects.md) | resource_reference | Retained (D21, D22) |
| Instructions | [AGENTS.md](../../AGENTS.md) | project_instructions | Released for link updates (D20); its links name no relocated document, so none changed |

## Decisions for implementation

1. Keep three domain roots: src/simulation, src/training and src/visualization.
2. Keep named shared support roots: src/contracts and src/foundation. They contain no domain orchestration.
3. Keep final SDK/CPython compatibility assembly in src/adapters and executable composition in src/applications.
4. Preserve include/fsim installed paths, existing DLL names, public namespaces, layouts, vtables, error semantics and Python runtime type owners.
5. Extract the lower simulation facade before extracting training code from the SDK. The final SDK assembly consumes both; neither domain consumes that final assembly internally.
6. Keep training and simulation independent of Vulkan/VSG/Flow. An optional outer application may compose training with rendered images through a provider contract.
7. Keep simulation the sole authority for physical state, sensing, message delivery, guidance, fuzes, collisions and damage.
8. Treat source relocation and new physical-model implementation as separate packages. Neither closes the other automatically.
9. Review the M8 generic handoff and release its shared write/build window before source migration. Full M8's pending combat/Viewer inputs remain separate gates.
10. No code move is complete until its source, build graph, imports, tests, package paths, documentation links and ownership policy agree.

## Scope and completion

The original planning request produced the architecture assessment and executable migration plans in this directory without moving source or changing models. Subsequent source migration and acceptance are recorded by package in the roadmap and the current status above. Remaining physical-model work, full-program gates and upper-layer MA work stay separate; planning alone does not claim their completion.

A completed architecture migration means all three domain implementations live under their assigned roots, allowed dependency graphs are acyclic, compatibility facades are thin, package/source imports obey policy, and fresh required acceptance has passed. Empty destination folders and an attractive diagram do not meet that condition.
