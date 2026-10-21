# Unchanged showcase coordinate/resource investigation

Baseline: main `dded7aed2c043a9f2f8b611bcc4c3f8fd9a9be69`.
The original `showcase_native` was reproduced on 2026-10-20: one burst,
zero fragment Hits. Its original scene, parameters and assertions stay fixed.
Existing unrelated and prior feature changes remain in place.

This user-assigned window owns temporary read-only tracing in
`World.cpp::createVehicle`, `JsbsimModel.cpp::load`,
`WorldEngagements.cpp::finishWeapons`, `WorldDamage.cpp::detonateAt`, and
`Damage.cpp::assess`. Trace the actual selected files, fixed origins,
captured CG/attitude/material points, contact identity and generated rays.
Compare actual loaded resources to their maintained sources and evaluate
intersections independently. Remove temporary instrumentation before delivery.
Evidence/scripts belong in `build/architecture-reviews/showcase-coordinate-chain/`.

Before a production correction, add and run a failing regression with an
independent expected result. Tests may extend `test_damage.cpp`,
`test_contact_fuze.cpp`, `test_showcase_native.cpp`, or a narrowly scoped
resource/transform regression. Record any proven-defect scope extension before
editing another implementation path. No equipment value, scene criterion,
generic MA/gateway/public layout, or generated aircraft is changed to force a Hit.

No native SDK configure/build/test process was active at acquisition. This
window owns the serial SDK slot; use existing UCRT64 presets/launcher and
`D:/ENV/tmp`, bounded tests and `--no-tests=error`. No GUI is needed.
AeroNumericsLab and other worktrees are outside the filesystem scope.

The actual loaded F-16C damage resource and both native flight-model paths
match the maintained sources. Independent reconstruction validates fixed-origin
and charge-point transforms. A fresh capture also misses all authored external
mesh triangles. Closed contact surfaces and damage/penetration boxes are
explicitly different existing contracts; their difference alone is not a defect
and does not authorize replacing the damage model.

Two analytic regressions reproduced a separate geometric kernel defect before
correction: nonzero ray components were classified as parallel by fixed angular
cutoffs. This loses real finite-face/thin-box intersections and also creates
thin-box false positives. The production correction is confined to
`Damage.cpp::{slab,quad}`: use the exact zero parallel condition and retain
finite extents/edge classification, original RNG draws and damage laws.
The existing upstream Geometric Tools line/box clipping implementation was
reviewed as a primary reference for directional zero handling; no upstream
code or dependency is imported. Both new regression cases failed before repair.

Both complete native presets build. Each discovers/executes 109 selected cases:
108 pass, only the original `showcase_native` fails, and zero skip. Both
recordings and engagement JSON remain identical to the pre-correction baseline.
Temporary trace statements are removed. Current sources, binaries, exact case
sets, independent reconstruction and protected-input comparisons are retained
in the evidence directory. The user-edited prior repair plan is preserved.
No commit, branch, worktree, remote publication or GUI operation is performed.
The SDK serial build/test slot is released at this handoff. The requested
original positive-Hit repair remains incomplete; replacing admitted physical
assumptions is not claimed by this software correction.

## Continued root-failure investigation, 2026-10-21

The user explicitly resumes work until the original showcase problem is solved.
Baseline HEAD and existing dirty-file boundaries remain unchanged. Reacquire
the serial SDK native slot after confirming no active native jobs. Extend the
temporary tracing window in `WorldEngagements.cpp::finishEngagements` to retain
the actual sensor/target captured paths, selected model and each proximity
result near contact. Independently replay stationary points and compare the
earliest proximity/contact event. Quantify ray-to-authored-geometry clearance
before attributing a miss to floating-point transforms.

If a mismatch is established, regress it before production repair in the
existing fuze/path/contact tests and preserve the original scenario and gate.
This does not authorize selecting a different physical model or changing a
card value to manufacture interception. Diagnostic artifacts remain in the
same explicit project evidence directory. AeroNumericsLab remains excluded.
