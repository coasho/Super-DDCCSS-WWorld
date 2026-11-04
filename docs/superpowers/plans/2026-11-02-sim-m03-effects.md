# SIM-M03 effects quality implementation window

Main baseline: `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`.
Owner: effects feature, coordinated by the main delivery owner. Native configure,
build and test execution remains in the main owner's exclusive serial slot.

## Exact write allowlist

* `src/simulation/mission_systems/damage/Damage.h`, `Damage.cpp`: additive
  immutable detonation field, scene assessment, finite source and connected rod
  execution; preserve legacy single-target entrypoints.
* `src/simulation/mission_systems/damage/State.h`, `State.cpp`: detached membership
  construction and physically bounded component consequence helpers if required.
* `tests/simulation/test_damage.cpp`, `test_warhead_model.cpp`: independent
  conservation, shielding, finite source, rod continuity and detached membership
  fixtures before implementation.
* `tests/simulation/test_engagement_native.cpp`: delivery-owner-assigned
  diagnostic/orchestration correction after checkpoint5's observed historical
  crew-state comparison failure. Scene inputs and positive hit/functional/crew
  truth predicates remain in force; compare each record with its publication
  interval rather than a later final state.
* `docs/sdk/sim-m03-effects-quality.md`: source-backed model/uncertainty contracts,
  integration requirements and current evidence.
* This implementation allocation document.
* Additive provider sidecar (approved by delivery owner):
  `src/simulation/mission_systems/components/interfaces/WarheadAdapter.h`,
  `src/simulation/mission_systems/components/support/WarheadSnapshotV1.h`,
  `src/simulation/mission_systems/damage/WarheadAdapter.cpp`,
  `src/simulation/mission_systems/components/registration/WarheadProvidersV1.cpp`,
  `src/simulation/mission_systems/components/implementations/warhead/ZonedCaseWarheadV1.cpp`.
  Preserve the original adapter vtable; add a sibling scene capability and an
  explicit connected-ring provider selection.
* `src/simulation/runtime/world/WorldDamage.cpp` is assigned to effects only
  after the link owner's explicit release. `World.h` remains with link owner;
  additional Entry fields, if needed, require a recorded handoff.

Only the released `WorldDamage.cpp` window is assigned here; other World files,
CMake, public SDK interfaces, cards and generators remain with their recorded
owners. No other worktrees or excluded directories may be inspected. Shared
declarations and capture predicates are supplied as explicit source/symbol
instructions for their owner's serialized write window.

## Design and acceptance

One physical burst owns one immutable, mass-conserving finite fragment field.
Victim-specific random streams must not resample that field. Scene traversal
uses stable victim identity and orders physical crossings, retaining residual
fragment velocity and transferred impulse rather than giving each victim a new
projectile. Blast remains a separately declared free-air engineering fit.

Connected rods are an explicit source selection, only for configurations that
claim them. A closed expanding polygon preserves shared vertices until a
declared expansion bound; finite rod interception must cover material between
vertices. Independent rods remain a different declared mechanism.

Detached geometry keeps immutable original panel/component identities and owns
its membership/state separately from the parent. A departing section must not
shield or damage the parent while its independent body can receive a later hit.
World integration supplies captured body-origin poses, native atmosphere and
the complete eligible victim set.

Live-source causality additionally uses the delivery-owner-approved opaque
`WorldDamageContinuationV1` sidecar. It retains only immutable source correlation
and owned field state. The World declaration/capture seam stays with the link
owner. The field resumes actual captured intervals, retains crossed source faces
and full recipient generations, and publishes existing native damage meaning
after source removal without reusing a stream slot. Explicit synchronous
diagnostic calls retain their previous timing contract; the live/causal path is
selected explicitly at its dedicated seam.

Independent fixtures must establish source mass and finite bounds, a shared
field under victim reordering, downstream shielding, residual conservation,
Galilean invariance, connected rod interception between vertices and the central
ring gap, detached membership and unmodified legacy compatibility. Required
build/test operations are requested from the serial owner; no passing claim is
made before those results exist.

All 53 cards require explicit applicability: 26 explosive configurations and
27 not-applicable fuze/warhead configurations. Measured hardware constants stay
uncalibrated with declared engineering assumptions; missing generic mechanisms
are implementation work, never reclassified as calibration.
