# Native physical effects diagnostics and source lifetime

This diagnostic path inspects the maintained shared physical fragment field
and actual damage applications. It never emits extra particles, changes RNG
draws, applies forces, changes penetration or alters an engagement outcome.
No public Detonation layout, old applyDamage signature, WorldLayout v9 or
fsrec v8 field changes are required.

## Frozen source identity

Each causal `Pending` field captures its original weapon generation before
source removal or reset, alongside the existing immutable weapon/launcher
correlation and original stream slot. Its original name, store and actual
warhead charge/topology metadata are retained separately from any later Entry.
The internal `applyDamageWithSource` passes that frozen generation to the
diagnostic impact hook. The old applyDamage wrapper obtains the current
generation only for its original synchronous call.

The lightweight effects cache is keyed by **(weapon ID,generation)**.
Continuing an old field never looks up the generation of a replacement Entry.
A reset source can create a new field under its new generation without
blocking or overwriting the older field's display. Removed sources retain
their original identity; explicit selection of an old generation cannot
silently bind to a new generation. Cache metadata can restore the original
name/store even if the larger selected-frame cache has already been replaced.

The cache holds at most 128 lightweight effects records and expires them 30
simulation seconds after the last actual physical field/impact update.
Deterministic oldest-entry eviction changes diagnostics only. The separate
selected-frame history remains bounded to 8 total full frames. No Entry
pointer or FlightModel reference is retained by a causal field or cache.

## Particles, ordinals and epochs

`emitFragmentFieldV1` assigns each native emitted material primitive its
source ordinal 1..N after completing emission and validation. Assignment draws
no random values. Copies and causal continuation retain that ordinal through
flight, finite-line breakup and absorption; diagnostic code never regenerates
or reorders the physical population. Zero remains unknown for a legacy/custom
field that supplied no ordinal.

The immutable burst epoch is stored directly, not reconstructed repeatedly
by subtracting floating-point field age. Continuation particle positions and
velocities are copied at the actual captured native interval endpoint.
The sample timestamp is `simTime-(nativeDuration-endOffset)`; no prediction,
extrapolation or future assessed position is added by presentation code.
The existing synchronous diagnostic retains only the actual emitted source
at birth, rather than displaying its future fully assessed positions as now.

Source frame and delivery time are assigned when an actual new physical
cache revision is first published. Re-reading or publishing unchanged data
does not refresh them. Native positions have their own row sample epochs;
the Viewer labels them explicitly. Retained values become historical through
their original validity, even while the source's debris/fire remains visible.

One maintained weapon generation has one physical emitted warhead source.
An explicit manual diagnostic that emits independent burst epochs under that
same lifetime cannot be represented as one source: its diagnostic status is
rejected as ambiguous rather than mixing particles and impacts. This refusal
does not change that manual operation's physical execution.

## Counts and component meaning

`fragmentCount` is the bounded number of displayed **currently retained**
native particle rows. `totalFragmentCount` is the current native field's full
retained population, not its initial emitted count. Absorbed material is not
silently regenerated to maintain a constant display count.

`damageCount` is the number of distinct displayed component rows, keyed by
victim ID, victim generation and source component index. Repeated native
applications to that row accumulate actual hit count and impact energy.
`totalDamageCount` counts the valid positive-hit **component-impact records**
observed across those applications, including records whose distinct rows
exceeded the display bound. It is not a total distinct-component count and
must not be displayed as the denominator of a distinct-row coverage ratio.

A component row's timestamp is its latest actual native damage-application
boundary; it is not a fabricated per-fragment crossing time. Native Impact
supplies impact counts and energy. It does not supply continuous component
health or a per-component removed-mass attribution, so those availability
flags remain unset. The Viewer prints unknown values explicitly rather than
interpreting default zero as health/death or measured removed mass.

## Focused native source cases

`test_weapon_effects_diagnostics.cpp` emits a physical population, advances
and absorbs it, and compares labeled/unlabeled energy and momentum. Its
World case uses explicit diagnostic causal bursts, resets the same source ID,
emits under the second generation, removes the source, and checks both birth
epochs and unchanged publication epochs through sidecar reads. It is a native
diagnostic fixture, not an Agent/C2 release or natural-hit acceptance claim.
Root executes the normal native gate; source presence alone is not a pass.
