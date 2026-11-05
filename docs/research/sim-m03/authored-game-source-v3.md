# Explicit GAME source policies V3/V4

Numerical-domain qualification: relative mixture weights are canonically
rescaled by an exact power of two before constructing the CDF. Positive scaled
weights must remain normal and resolved both as prefix increments and at total
CDF scale; otherwise configuration refuses. This is a representability bound,
not a physical angular/detection threshold. Stable positive-component order is
preserved; zero-weight entries move before it so the unchanged native sampler's
defensive last-component fallback cannot select a zero-weight component. Common
normal source fields retain their element/RNG behaviour. Fixed tests compare
normal, subnormal and huge equal mixtures against the independent u<1/2 CDF.

The new ring decoder additionally checks finite positive expansion, breakup,
initial canonical chord, growth and material length using retained operations.
Its new typed provider validates every actually emitted material scalar under a
temporary RNG before publication, covering phase/captured-pose rounding that
intrinsic checks alone cannot guarantee. A refused actual population leaves the
caller RNG unchanged. Old V0/V1/V2 emitters and their semantics are untouched.

These additive native policies consume explicitly owned GAME emission inputs.
They do not construct a real device, infer explosive performance, calibrate
equipment or turn an unchanged encounter miss into a success. Existing V0/V1/V2
provider semantics, source mathematics, public layouts/vtables and defaults
remain preserved. Selected equipment cards adopt the new policies through
separately reviewed authoring changes; their original parameter records remain
intact.

`native.warhead.authored_field.v3` owns a finite annular-volume point source or
the existing disconnected finite-rod source selected by the existing card kind.
`native.warhead.authored_ring.v4` is a separate typed provider preserving
`emitFragmentFieldV1(...,connectedRod=true)`, its initial connected polygon,
material-limited unfolding and breakup. A connected ring is never routed
through the zoned/disconnected-rod provider.

## Independently owned inputs

Both policies require exact model version, total/charge/emitted-case/remaining
structure masses, independent blast equivalent, containing diameter/length,
source outer/inner radii, equivalent material length, integer element count,
presented area, source-relative speed, drag coefficient and fixed structural
station. No Gurney driver is owned; an unreferenced old card Gurney value cannot
change ejection or blast. The unused legacy snapshot Gurney field is0 for these
new policies. This is a versioned direct GAME input contract, not a new energy
conversion law.

V3 additionally requires1..32 explicit solid-angle mixture entries, each with
0<=from<to<=180 degrees, nonnegative finite relative weight and positive finite
total weight. Overlap retains its mixture meaning. It samples the existing
uniform cosine in a selected band and existing coupled radial/ejection azimuth.
It adds no forward/aft sectors. V4 rejects mixture roles because its retained
connected-ring emitter does not consume them; its exact topology binding is
`native.connected_ring.material_limited.v1`.

Every role must appear exactly once in the provider's resolved role set and
match its loaded card snapshot. Missing, extra, unit/provenance mismatch,
nonfinite, wrong version/topology, invalid count/geometry, overflow or empty
mixture refuses before emission. Count retains the existing1,000,000 resource
ceiling; connected topology requires at least3 elements. Positive material
length must permit the existing connected unfolding radius.

Total mass must equal charge+case+remaining structure within64 binary64 ULPs
of their scale, a numerical accounting bound. Independent blast equivalent is
not physical mass subtraction. The direct source preserves existing source
mass/count ownership: each emitted element owns case/count. Remaining structure,
gas, recoil and carrier continuation are not silently reclassified as emitted
fragments or component damage.

## Source geometry and frame equations

The coarse parser containment domain is the declared containing cylinder.
V3's source has axial half-support L/2 and radiusR. V4's INITIAL mathematical
centerline polygon has half-support0 and radiusR. Its equivalent material
length controls unfolding, not initial axial support; presented area is not a
wire radius. A centreline certificate is not a physical material-volume proof.

The reviewed authored19 configurations also have a continuous source-loft
certificate. Raw-knot PWL prior supports are explicit GAME surrogates whose
whole expanded intervals were independently certified inside the actual
Body.section-PCHIP/shape.loft-produced source primitive. The separate5 reference
uses those actual produced coefficients directly. These source-only proofs do
not claim a final mesher/IEEE-distance/hardware-compartment certificate.

Structural station s is aft/right/up from the authored nose. The existing
native launch seam converts it with S=diag(-1,1,-1):

```
p_source_body = p_nose_body + S*s
p_source_ECEF = p_CG_ECEF + R*(p_source_body-cg_body)
v_source_ECEF = R*(v_CG_body + omega_body cross (p_source_body-cg_body))
```

All inputs use the same native captured pose/CG/rate epoch. Unavailable fixed
point data refuses rather than substituting CG. Source position ownership is
independent of fuze acquisition radius, arming, delay, trigger time, guidance
and control. These policies change none of those values or laws.

## Per-model functional delta and limits

The19 former zoned-case selections retain their individual existing intrinsic
source mass, speed, count, area, angular distribution, Cd and blast magnitude.
The new functional changes are explicit input ownership, a fixed material
source frame instead of captured CG, strict bounded role decoding and one
shared finite-field assessment path. Each source's actual authored body/grain
layout yields its own station support, selected median and nonzero location
prior uncertainty. The prior is uniform in declared structural axial length;
it is not a hardware installation estimate or calibrated confidence interval.
The authoring receipt supplies each model's exact retained-input and station
derivation record. Shared laws with shared inputs are genuine equivalences,
not nineteen invented breakup equations.

AIM120C keeps its existing V2 selection and1.66m authored station. Four R27
ring source configurations now select their separate typed V4 policy through
the independently reviewed four-card authoring change. Their retained centerline
support and conditional legacy-equivalent-square dilation sensitivities stay
distinct from physical folded/slack material volume. Current native and World
execution remain separately required for each selected configuration.

Direct assessments use a temporary caller RNG, emit the actual population once,
and commit only if its complete scene assessment is valid. World scene emission
retains the existing copy-then-commit source RNG convention. No client performs
source simulation, no API observation permission changes and no native outcome
is synthesized.

## Verification boundary

The added native tests use existing registry/factory APIs, with new exact IDs as
data. Before production the positive sections must produce runtime native
unregistered-provider RED; a compile failure is not RED. After production they
compare every old/new element, endpoint, velocity, material field and final RNG
for identical synthetic source inputs, plus blast/driver independence and
strict domain/role/refusal controls.

Source authoring, provider unit tests and geometry calculations do not establish
actual per-model launch execution, installed/API delivery, Viewer/replay,
whole-device conservation or held-out equipment accuracy. Those require their
own current native/source-bound gates. The23 swept-path fuze and22 cascade
control rows remain separate mechanisms and are not admitted by these policies.
