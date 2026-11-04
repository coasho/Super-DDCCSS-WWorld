# SIM-M03 Complete Quality Implementation Plan

> **For agentic workers:** Use subagent-driven development for independent audits and reviews. The root owns integration and the exclusive serial native configure/build/test and Viewer slots. Work only on main; do not create a branch or worktree.

**Goal:** Complete and deliver SIM-M03 under the user-confirmed SIM-M01/M02 engineering rule, with all required components, applicable functions, independently checked methods, native integration, public interfaces, fleet coverage and reproducible delivery. Push the verified result to main.

**Architecture:** The SDK owns every simulated weapon mechanism. JSBSim remains the sole flight integrator. Native sensor, guidance, communication, fuze and damage components produce the state consumed by public interfaces and the observational Viewer. Reuse the accepted M01 fusion and M02 flight plant without transferring generic gateway ownership.

**Tech stack:** Existing C++17/UCRT64 presets, JSBSim, native mission-system components, C99 ABI, CPython stable ABI, Hangar source generation and fullscreen 1x Viewer.

**Specification:** [Work allocation](../../architecture/modules/simulation/ms-work-allocation.md), [approved representative mechanisms](../specs/2026-10-20-sim-m03-representative-weapons-design.md), [complete-package matrix](../../sdk/sim-m03-reference-continuation.md), and the user's 2026-11-02 complete-delivery assignment.

## Authorization and acceptance

The user confirms M01 and M02 acceptance and assigns the complete M03 continuation,
including pushing and delivering the result. This assignment supersedes the
historical first-slice stop points for M03. It does not reopen the approved M02
rotor deferral, change M04 launch-zone scope, transfer generic gateway ownership,
or authorize invented equipment capability.

Actual equipment measurement calibration is a separate uncertainty ledger, as
in accepted M01/M02. An assumed or derived engineering model must execute its
required function, disclose its conditions and uncertainty, and pass independent
method and native tests. Missing implementations, failed tests, unexercised
variants and undelivered public products cannot be renamed uncalibrated.

The final M03 receipt must contain no unclosed engineering gates. Preserve raw
failures and original scenarios/tolerances. Neither a detonation nor native
crew loss constitutes mission success or a demonstrated comparison with DCS.

## Initial state and exclusive windows

- Main HEAD before this feature: `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`.
- Existing tracked and untracked changes are retained; record exact input bytes
  before changing an inherited file. Do not stage another feature's changes.
- The initial write allowlist is this plan only. Each implementation task below
  must record an exact file/symbol allowlist before its source edits.
- Read-only sensor/control, link/public-interface and effects/delivery audits may
  run independently. Shared World, API, CMake, card/generator and boundary files
  have one writer. Root serializes all native configuration, builds and tests.
- No native CMake/Ninja/CTest or Viewer process was active at initial inspection.
- Never access any `.codex` directory or another worktree. Scope all source scans
  to explicitly allowed paths and exclude `.codex` before recursion.
- Set TMP, TEMP and TORCH_EXTENSIONS_DIR to `D:/ENV/tmp`. Reuse existing tools.
- Keep all maintained artifacts and commit messages in English.

## Review focus

1. Retained/predicted data must not acquire a new measurement epoch through
   rereading, propagation or transmission. Reject future, stale, reordered and
   invalid input before mutating accepted state.
2. Sensor and guidance inputs must exclude referee target/decoy identities and
   hidden passive-sensor Cartesian information; finite sensing/transport must
   execute rather than appear only as configuration metadata.
3. Unsupported equipment directions/modes must refuse explicitly. A family name
   does not establish a fitted return link or a proprietary discrimination law.
4. All per-release mutable histories, queues, assessments and retained reports
   must obey ownership, deterministic streams, reset and generation lifetimes.
5. Actual public/installed consumers and actual complete-scene presentation must
   agree with native events. Old-header binaries and frozen schemas retain their
   contracts; unavailable data remain unavailable.

## Task 1: Reconcile the complete engineering population

Root evidence-tool write window, recorded before implementation: new
`tools/validation/weapon_quality/{contract.py,audit.py,acceptance.py,test_acceptance.py,campaign.py,installed.py,delivery.py,fleet.py,normal_entry.py,visual.py,navigation_reference.py,report.py,README.md,CMakeLists.txt,consumer.cpp,consumer.c}`;
new `docs/sdk/{sim-m03-completion.md,sim-m03-engineering-contract.md}`; new `docs/data/sim-m03-20261102-*`
receipts; this plan. Existing current-status/shared guide updates wait for the
final integration window. Source/API/effects/sensor files are not owned by this
evidence-tool window.

The integration continuation additionally owns new
`tools/validation/weapon_quality/{cadence.py,explosives.py,visual_explosives.py}`
and `cmake/ArchitectureChecks.cmake` only for the M03 admission-test registration.
Tools-only delegation is recorded in root handoffs; no worker executes a native
or GUI process. Reviewed inherited mandatory presentation prerequisites are
included only after source review and actual Release verification.

Final evidence-integration window at the same main HEAD additionally owns
`tools/validation/weapon_quality/{maps.py,test_maps.py,test_delivery.py,test_explosives.py}`
and the existing campaign, report and delivery tools. These bind the actual
start/end offline map tree, serialized generation audits, installed product
hashes and complete post-terminal World observations. Root registers their
repository checks in `cmake/ArchitectureChecks.cmake`. No simulation acceptance
predicate, scene, tolerance or native outcome is relaxed by this window.

### Observed integration corrections

- Checkpoint 5 retained the real PL-15 release refusal. Normal generation then
  exposed the new nested pulse-policy dictionary in a scalar positivity loop.
  The corrected generator's complete 16-method class passed at checkpoint 6.
  A further release check found that native plant availability still rejected
  implementation code4; it now admits code4 only for propulsion, with all
  other slots and mismatched fingerprints explicitly tested.
- The independent navigation review found continuous covariance applied to
  sampled held error. An augmented held-error state now matches realized
  exposure draws and fractional GNSS updates. Native tests and independent
  sampled matrix/continuum references passed after that correction.
- Causal damage invalidated an old comparison of every historical destruction
  flag against final crew state. The fixture now samples native crew state at
  each publication and preserves the actual hit, crew and functional predicates.
- Old immediate V2 observations remain explicit separate compatibility
  fixtures. Current V3 timing, reset and finite contact acceptance are separate;
  no current provider ID executes an ideal replacement to manufacture a pass.
- Independent review requires every captured target motion sample and emitted
  fragment coefficient to validate transactionally, plus real propagation-step
  convergence. These gates remain required until their actual root-run GREEN.

All failed logs remain in the dated architecture-review directory. These
corrections are integration history, not a completed M03 acceptance assertion.

Native RED registration window: `tests/CMakeLists.txt`, only new
`fsim_weapon_sensor_engineering_test` / `weapon_sensor_engineering` and
`fsim_weapon_links_test` / `weapon_links` registrations. Preserve inherited
Viewer registrations and scheduling changes. No implementation edits during
this build. All three workers confirmed their source-write freeze.

Root navigation window: new
`src/simulation/mission_systems/weapon/NavigationV1.{h,cpp}`,
`tests/simulation/test_weapon_navigation.cpp`,
`docs/sdk/weapon-navigation.md`, and their exact source/test registrations in
`src/simulation/mission_systems/CMakeLists.txt` and `tests/CMakeLists.txt`.
The link owner integrates native captured ownship samples and consumes the
estimate; no root World/API edits occur in the link owner's window. The model
simulates own inertial error and optional explicitly fitted GNSS aiding. True
target state is never a navigation input. Tests require unaided drift,
independent covariance equations, aiding/outage/reacquisition, fixed scheduled
streams, equal-epoch non-consumption and atomic malformed-input rejection.

Shared source registration window: add only the reviewed new sensor, weapon-link
and navigation source paths to `src/simulation/mission_systems/CMakeLists.txt`,
`src/simulation/runtime/CMakeLists.txt`, `src/simulation/api/CMakeLists.txt` and
the feature targets in `tests/CMakeLists.txt`. No inherited registrations are
removed or reduced. Native checkpoint builds require all worker write freezes.

Root commanded-motor integration window: `cmake/JsbsimPatches.cmake`, only the
opt-in `fsim/commanded-dual-pulse-v1` table's property-backed ignition timeline,
reset and remaining-propellant impulse bound. Preserve the legacy named-table
branch, FGRocket class layout, authored phase thrust/duration/Isp and existing
thrust buildup. Register only `weapon/PulseManagerV1.cpp` and its native fixture
test dependencies/definitions. The sensor/pulse owner supplies the reviewed
patch; root applies it and executes independent native resolution/fuel tests.

Checkpoint 4 reproduced a constructor-time access violation in every solid
rocket load. GDB `pulse-crash-gdb.log` identifies `FGRocket::ResetToIC` called
by `FGEngine::Load` from the derived constructor, before its body initializes
`ThrustTable`. Root adds pointer initialization in the existing constructor's
initializer list through the same CMake patch. No class layout, dependency
version or acceptance predicate changes. Native pulse/public/effects fixtures
retain this failing baseline and must pass after the repair.

Root native/visual campaign integration also owns
`tools/validation/viewer_perf/aam_campaign.py`: preserve the two existing frozen
encounter definitions, correct superseded calibration-blocker labels, discover
actual primary-display dimensions, and verify actual fullscreen foreground
presentation rather than only a configured framebuffer size. No windowed,
accelerated or reduced-effect route is added. Inherited bytes were captured in
the initial-window receipt before these edits.

- [ ] Independently derive the current store/airframe/fit rosters from maintained
  design, card and packaging inputs; reconcile all 24 current AAMs and applicable
  unguided-store effect paths without using representative coverage as a total.
- [ ] Audit each component and cross-component function against current source.
  Separate missing execution, numerical defects, supported approximation and
  optional equipment measurement calibration.
- [ ] Freeze an executable acceptance matrix with exact methods, independent
  references, native cases, public paths and visual evidence for every row.
- [ ] Record source/product hashes and preserve the inherited dirty state.

## Task 2: Complete sensing, navigation, guidance and physical response

- [ ] Add regressions demonstrating each missing function or concrete defect.
- [ ] Complete native measurement timing, stabilization/tracking response and
  relevant RF/IR measurement formation using sourced methods and explicit
  engineering inputs. Keep passive terminal observations bearing-only.
- [ ] Reuse accepted M01 processing/fusion and M02 finite-fin/propulsion plant
  where appropriate. Do not duplicate flight integration or copy fighter radar
  performance into a seeker without an explicit engineering derivation.
- [ ] Compare analytic/synthetic independent references, cadence convergence,
  sensor loss/reacquisition, countermeasures, saturation, failures and reset.
- [ ] Exercise every admitted weapon variant through real native release.

## Task 3: Complete modeled weapon communication and permitted feedback

- [ ] Freeze explicit fitted receive/return direction and permitted content per
  variant, with sourced facts separate from assumed timing/resource inputs.
- [ ] Add native endpoint-bound uplink delivery; only delivered launcher/fused
  knowledge may refresh navigation. Preserve sample and receive epochs,
  uncertainty, loss state and provenance from the accepted M01 seam.
- [ ] Implement assessment accept/refuse/cancel/expiry and periodic delivered
  return reports for supported profiles, including terminal retention and
  LOST/SOFT_LOST/END_OF_FLIGHT semantics distinct from seeker reacquisition.
- [ ] Deliver additive C++/C99/Python Agent access with non-consuming reads,
  permission/refusal, sized-buffer, identity, lifetime and deterministic tests.
- [ ] Preserve the frozen generic unavailable-only v1 schema and registry.
  Supply an explicit native extension handoff without claiming uninstalled
  generic handlers or wire-protocol conformance.

## Task 4: Complete fuze, effects and event-chain quality

- [ ] Audit safe-arm/contact/proximity/delay/expiry and one-shot initiation using
  captured native paths, including cadence and malformed-input cases.
- [ ] Verify fragment/blast energy and momentum accounting, geometry, vulnerable
  component response and native event meanings against independent methods.
- [ ] Repair missing native execution required by the M03 scope, including
  applicable detached-body/conservation behavior rather than hiding it in a
  successful screenshot.
- [ ] Exercise native chaff/flare lifetime, motion and sensor interaction with
  deterministic replay and clearly identified engineering limitations.

## Task 5: Complete public, installed, fleet and visual acceptance

- [ ] Close all implementation review findings, then freeze source inputs.
- [ ] Build both existing UCRT64 presets and discover the real full CTest
  inventories; execute every applicable case with bounded timeouts and
  `--no-tests=error`, retaining the unchanged approved M02 rotor scope only.
- [ ] Reconcile nested Python tests and any source-export Git-only bridge from
  actual logs. No new unexplained skips or historical totals count as passing.
- [ ] Install a fresh full SDK/Python/Viewer product; compile and run independent
  C++, C99 and Python consumers and record actually loaded module identity.
- [ ] Run every current AAM through frozen native scenarios and meaningful
  seeker/support/countermeasure/control/fuze/effect cases. Record misses as
  native results; no hit-percentage calibration is invented.
- [ ] Run actual primary-display fullscreen 1x Viewer cases with Earth imagery,
  terrain, clouds, sunlight, shadows and native exhaust/contrails/effects.
  Review all required captures and bind them to native records and binaries.
- [ ] Independently review complete theory, implementation, coverage and
  delivery. Fix all engineering findings before admission.

## Task 6: Reproducible delivery and push

- [ ] Produce one reproducible normal-entrypoint campaign, source/product/raw-log
  manifests, independent reference evidence, per-model matrix and uncertainty
  ledger. Verify from an ordinary allowed clean source export, not a worktree.
- [ ] Produce and verify a complete SDK/Viewer package and a single exact-byte
  source patch from the recorded main baseline. Verify patch replay.
- [ ] Update current status and M03 guides; date historical evidence honestly.
  The final receipt requires complete engineering gates and zero unclosed rows.
- [ ] Verify main, inspect feature-owned staged diff, commit with English text,
  inspect remote ancestry, push main, and verify the remote commit.
- [ ] Deliver concrete artifact paths/checksums, exact reproduction command,
  observed verification counts and the pushed commit; release all slots.
