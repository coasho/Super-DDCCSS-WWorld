# Warhead source, material effects and primary numerical references

Current-status update, 2026-11-03: the JSON now documents each current model
individually in theory, parameter and fidelity matrices. Its current `passed`
flag is false until a fresh actual native comparison under these source/card
hashes succeeds. Earlier72-row/836-constraint and older818-assertion results
are historical only and remain nested as `historical_evidence`. They do not
verify the new reticle receiver or the current scene-policy epoch. The separate
[72-cell parameterized table](per-model-effects-components.json) retains exact
source records and does not grant native/public/visual or calibration status.


The 24 AAM selectors resolve to three actual source mechanisms: zoned case,
finite zoned case V2 and connected-ring rod. The companion
[warhead.json](warhead.json) preserves each model's full parameter provenance,
uncertainty, selected implementation and mechanism delta. The common source
methods do not identify OEM explosive/case construction or establish calibrated
lethality. The two additional explosive cards are covered by the separate
26-card effects population; they are not invented AAM component rows here.

## Charge-to-source equations and assumptions

The primary basis is [R.W.Gurney, BRL Report405 (1943), equations2.1,2.3,2.4,
printed pp4-5](https://www.pdf-archive.com/2019/11/16/untitled-pdf-document/untitled-pdf-document.pdf),
an archived reproduction of the public-release report. Its uniform cylindrical
case assumes gas radial velocity increases linearly from zero on the axis to
case velocity. With case mass M, charge mass C and explosive specific energy E,

\[
EC=(M/2+C/4)v^2,\qquad
v=\frac{\sqrt{2E}}{\sqrt{M/C+1/2}}.
\]

The actual `gurney_ms` input is sqrt(2E). It is not TNT equivalence. The SDK
derives or explicitly resolves case mass and finite envelope according to the
selected source version. Current parameter source records retain whether case
geometry, charge or energetic properties are published, derived, assumed or
game references. The original report's uniform cylinder theory is not evidence
for our finite station, angular zones, case breakup, rod welds or any OEM
explosive constant.

For point fragments, the emitted population conserves case mass with equal
equivalent masses `M/N`; speed relative to the unrotating source is v. Its kinetic
energy is `Mv²/2`. Finite V2 resolves the authored annular case, station, axial
length and angular zones. Legacy zoned cards retain their established derived
case envelope and angular law. Zone sampling is uniform in cosine over the
chosen solid-angle band, with the declared zone shares; it is not a uniform
angle approximation. The War Thunder seven-zone reference remains a game
reference, not a fragment-arena measurement. Discrete rods span their full
equivalent finite length. Legacy rod speed retains its established1400m/s cap;
this explicit delta is compared rather than quietly removed.

The connected-ring selection is explicit, never inferred from the word `rod`.
N homogeneous line elements share adjacent polygon endpoints and endpoint
velocities. The initial radius is outer-case radius r; expansion rate is v/r.
Maximum radius is `N*length/(2*pi)`, so breakup age is
`(Rmax-r)/v`. Material length is the actual final chord, not a false assertion
that polygon chord and arc lengths are equal. Thereafter each element is a
finite independently flying rod. This is a homogenized source approximation,
not a claim that N is an OEM weld count.

The kinetic energy of an affine-velocity rod is evaluated by the exact line
integral

\[
K_i=\frac{m_i}{6}(v_{0i}\cdot v_{0i}+v_{0i}\cdot v_{1i}+v_{1i}\cdot v_{1i}).
\]

For an unrotating regular connected ring this gives
`K=(Mv²/2)*(2+cos(2*pi/N))/3`. Treating every segment as a point at an endpoint
would give the wrong energy. Total linear momentum integrates the two endpoints
with equal weights. Rigid-body source translation and spin are added using the
captured station/lever arm in the native path, preserving frame conventions.

## Material interaction, state, input/output and time

World owns one immutable emitted population for a physical burst, not a new
fragment draw per victim. Stable source ordinals and per-face/per-victim crossing
state survive continuation. Inputs are the resolved source, captured burst
pose/velocity/axis/angular velocity, RNG, attached native geometry/material,
captured recipient motion and finite environment. Outputs are emitted and
remaining populations, source identities, component/panel hits, perforation and
hole areas, transferred/retained/air/gravity momentum and body linear/angular
impulses. Source RNG consumption is transactional on refusal.

The causal scene assessor orders actual crossings across recipients, retains
residual fragments, and handles finite rods as segments rather than endpoint
rays. THOR aluminium2024-T3 residual velocity is an empirical steel-fragment
plate law with its original unit conversions and obliquity policy. It is not
universal composite/titanium/uranium penetration physics. Solid component
interception and finite-fluid transit have distinct native methods. Liquid
geometry/density/ownership and unresolved tumbling Cd have separate declared
assumptions and numerical gates; no wet component is counted as a perfect solid
absorber or as calibrated fuel ballistics.

Blast uses the declared TNT-equivalent input and the retained Swisdak/Kingery-
Bulmash fit, Sachs ambient scaling, facing/reflection and panel impulse law.
The implementation uses a hemispherical fit at half TNT mass as its declared
free-air engineering approximation. Empirical scaled-range clamps and ideal-gas
reflection are limits, not arbitrary far-field or obstacle/shielding fidelity.
The primary source is [Swisdak, Simplified Kingery Airblast Calculations,
1994, ADA526744](https://ntrl.ntis.gov/NTRL/dashboard/searchResults/titleDetail/ADA526744.xhtml).
This package's independent numerical comparison below targets charge/source
conservation, not an independent remeasurement of those blast/THOR fits. Their
equation domains and the current native effects regressions remain separately
required.

Scene point flight has one material-phase scheduler; fixed-phase smooth
integration resolves actual captured moving material-boundary roots. Finite
absolute clock progress, local-coordinate arithmetic and earliest real-work
timestamps are native contracts. Rod loads integrate affine material velocity
over owned intervals with global error budgets and exact shared-face ownership;
no double loading on adjacent cell faces. Continuation keeps physical age and
live material content; it does not expose future intersections. Scene angular
impulses are about each instantaneous moving structural origin in the initial
body axes; the World consumer must not add the legacy fixed-origin translation.
Legacy synchronous/external assessment keeps its own original convention.

## Independent numerical comparison and cohort-specific proof

The actual selected `WarheadAdapter` and additive scene capability are used for
each authored AAM. The fixture captures resolved Gurney/case/charge parameters,
native cylinder speed and source speed, then emits the real selected source at
fixed seed1943, zero translation/spin and axis(1,0,0). These conditions isolate
source conservation; they do not suppress any production coefficient or select
a substitute provider. All three mechanism cohorts have actual per-model
observations rather than one representative row.

The checker solves the primary energy balance independently with55-digit
Decimal arithmetic. It checks actual point/rod population count, emitted total
mass, energy, minimum/maximum speed, finite radial/axial envelope and segment
type. The connected cohort additionally checks exact adjacent endpoint closure,
zero net momentum within accumulated arithmetic budget, expansion rate, breakup
age and both material-chord extremes. These are additional geometric and
conservation derivations, explicitly our source-method controls rather than
measurements attributed to the Gurney paper.

Predeclared numerical budgets are2e-12 relative plus1e-12 absolute for Gurney;
1e-10 relative plus1e-6 J for population energy; mass accumulation64 eps*N;
and128 eps times source size for envelope arithmetic. Count, segment identity
and ring endpoint closure have zero tolerance. No tolerance was fitted to a
coupled hit, radius, lethality or card. Negative tests corrupt mass, energy,
population, finite envelope and topology and require refusal. Exact actual
inputs/outputs/differences are in [warhead.json](warhead.json) and
[effects-results.v1.json](references/effects-results.v1.json).

The original 26-card native effects population separately requires actual scene
emission, assessment and native damage, including fragment fuel holes rather
than blast-only destruction. Moving-origin, continuation, shared-face and
finite-clock/kernel regressions remain separate. A source-method comparison
does not replace those tests, per-component functional damage, installed public
execution, whole-chain fuze/pose transport or full visual evidence.

## Known fidelity limits

Unknown internal case geometry and energetic/fragment distributions remain
explicit assumptions with full per-model records. Uniform equivalent fragments,
homogenized rods, fixed angular laws, finite pose discretization, empirical
plate/blast laws, perfect solid-component absorption, unresolved fluid tumbling,
fire initiation and structural response are not a validated high-explosive
hydrocode or OEM vulnerability model. The source method has executable effects
and conservation checks; claiming aircraft-specific kill probability or greater
fidelity than another simulator would require distinct held-out measurements.

The historical pre-reticle Release actual native case passed818 assertions for the effects
observation fixture. Independent warhead comparisons historically admitted all24 selected
sources across all three mechanisms and292 numerical constraints, including
connected-ring energy/endpoint/length checks, without changing any original
budget or coefficient. Actual capture, pre-execution source epoch, case-level
JUnit and residuals are hash-bound in `warhead.json`. This source-method result
does not replace the separate original26 population's functional damage,
material integration, whole-chain or installed/visual requirements.

## Exact current model differences

The19 `zoned_case.v1` entries below are not identical warheads. Their own W, C, diameter and Gurney inputs produce different case mass, thickness, equivalent count, area and speed. This cohort includes the discrete rod selections R-60M, R-73 and R-77M; only the four explicitly selected R-27 variants use connected rings. A hardware label is never used to infer topology.

| Model | Selected source | W / C / case kg | Diameter / length / thickness m | Elements | Speed m/s |
| --- | --- | --- | --- | ---: | ---: |
| aim-120c | finite_zoned_case.v2 | 20 / 7.17 / 10.9055 | 0.16002 / 0.313216 / 0.0093717 | 1688 | 1828.91 |
| aim-120d | zoned_case.v1 | 20 / 7.17 / 10.9055 | 0.16002 / 0.313216 / 0.00882284 | 2023 | 1828.91 |
| aim-7m | zoned_case.v1 | 39.9 / 11.79 / 23.8935 | 0.1827 / 0.438502 / 0.0120935 | 1721 | 1446.97 |
| aim-9m | zoned_case.v1 | 9.43 / 3.58 / 4.9725 | 0.1143 / 0.297731 / 0.00592495 | 3045 | 1964.5 |
| aim-9x | zoned_case.v1 | 9.36 / 3.58 / 4.913 | 0.1143 / 0.296625 / 0.00587589 | 3085 | 1973.2 |
| asraam | zoned_case.v1 | 10 / 3.69 / 5.3635 | 0.1494 / 0.182215 / 0.00798901 | 1340 | 1860.22 |
| iris-t | zoned_case.v1 | 11.4 / 4.21 / 6.1115 | 0.1143 / 0.355035 / 0.00610676 | 3419 | 1861.11 |
| magic-2 | zoned_case.v1 | 12.7 / 6 / 5.695 | 0.1413 / 0.294394 / 0.0055514 | 4240 | 1993.66 |
| meteor | zoned_case.v1 | 12 / 4.43 / 6.4345 | 0.1602 / 0.190213 / 0.00856236 | 1306 | 1860.71 |
| mica-em | zoned_case.v1 | 12 / 3.7 / 7.055 | 0.144 / 0.216323 / 0.00918359 | 1160 | 1675.94 |
| mica-ir | zoned_case.v1 | 12 / 3.7 / 7.055 | 0.144 / 0.216323 / 0.00918359 | 1160 | 1675.94 |
| pl-10 | zoned_case.v1 | 11 / 4.06 / 5.899 | 0.144 / 0.215778 / 0.00769821 | 1647 | 1860.49 |
| pl-12 | zoned_case.v1 | 20 / 9.2 / 9.18 | 0.1827 / 0.273265 / 0.00745591 | 2821 | 2124.43 |
| pl-15 | zoned_case.v1 | 22.5 / 8.3 / 12.07 | 0.1827 / 0.274112 / 0.00977286 | 1647 | 1859.89 |
| pl-8 | zoned_case.v1 | 11 / 5 / 5.1 | 0.144 / 0.240366 / 0.00597469 | 3046 | 1946.66 |
| r-27er | connected_ring.v1 | 39 / 15 / 20.4 | 0.234 / 0.295713 / 0.0119543 | 61 | 1400 |
| r-27et | connected_ring.v1 | 39 / 15 / 20.4 | 0.234 / 0.295713 / 0.0119543 | 61 | 1400 |
| r-27r | connected_ring.v1 | 39 / 15 / 20.4 | 0.207 / 0.377887 / 0.010575 | 61 | 1400 |
| r-27t | connected_ring.v1 | 39 / 15 / 20.4 | 0.207 / 0.377887 / 0.010575 | 61 | 1400 |
| r-37m | zoned_case.v1 | 60 / 22.14 / 32.181 | 0.342 / 0.208634 / 0.0182881 | 670 | 1860.22 |
| r-60m | zoned_case.v1 | 3.5 / 1.35 / 1.8275 | 0.108 / 0.124762 / 0.00549962 | 62 | 1400 |
| r-73 | zoned_case.v1 | 8 / 3.75 / 3.6125 | 0.1485 / 0.167172 / 0.00590062 | 79 | 1400 |
| r-77m | zoned_case.v1 | 22.5 / 9.725 / 10.8588 | 0.18 / 0.306252 / 0.00798746 | 71 | 1400 |
| super-530d | zoned_case.v1 | 30 / 8.1 / 18.615 | 0.2367 / 0.189024 / 0.0168705 | 494 | 1614.09 |

### aim-120c

aim-120c: W=20kg,C=7.17kg,M=10.9055kg,D=0.16002m,L=0.313215656m,t=0.00937169949m,N=1688,v=1828.90539m/s; Only selected finite V2 AAM source and turning-CPA/station fuze. Explicit finite geometry differs from retained V0 cousins; no C7 hardware internal reconstruction is inferred.

Parameter/source matrix: 17 resolved roles; original basis counts {"assumed": 14, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `9b24592f378faaed949bc45ecee3ab8fb606af14540a48f2448e2ba56fd28853`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Only selected finite V2 AAM source and turning-CPA/station fuze. Explicit finite geometry differs from retained V0 cousins; no C7 hardware internal reconstruction is inferred. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### aim-120d

aim-120d: W=20kg,C=7.17kg,M=10.9055kg,D=0.16002m,L=0.313215656m,t=0.0088228384m,N=2023,v=1828.90539m/s; The card explicitly reuses C7 warhead information because exact D data are unpublished. Selected V0 source and CG sweep differ from AIM120C V2 despite equal total/charge/radius; do not equate the software constructions.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `edee1156a72b838e0b9460f8e4b7aa7ac0cc0a4557b2a0804c022a7ac58cb65f`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: The card explicitly reuses C7 warhead information because exact D data are unpublished. Selected V0 source and CG sweep differ from AIM120C V2 despite equal total/charge/radius; do not equate the software constructions. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### aim-7m

aim-7m: W=39.9kg,C=11.79kg,M=23.8935kg,D=0.1827m,L=0.438501839m,t=0.0120934517m,N=1721,v=1446.97298m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `2db284716d8ffd42f3a07c50cf87cdb3b24f56fdbb18ba67b130dfd800b9f4c5`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### aim-9m

aim-9m: W=9.43kg,C=3.58kg,M=4.9725kg,D=0.1143m,L=0.297731312m,t=0.00592494769m,N=3045,v=1964.49821m/s; Reticle receive law is newly selected; WDU17/B label does not establish the single-detector/spoke hardware. Historical passive-fin assertion abort was a fixture failure, not a warhead result.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "derived": 1, "published": 2}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `79a3c50125053e273acd31f32f55797991b2bbe5d84090afad265505f7a4ac9a`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Reticle receive law is newly selected; WDU17/B label does not establish the single-detector/spoke hardware. Historical passive-fin assertion abort was a fixture failure, not a warhead result. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### aim-9x

aim-9x: W=9.36kg,C=3.58kg,M=4.913kg,D=0.1143m,L=0.296624521m,t=0.00587589403m,N=3085,v=1973.19801m/s; Imaging selection and its own card mass differ from AIM9M; shared WDU17/B reference is explicitly inherited, not independent measured X case geometry.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `e2d6902af9477810d7d0de28cf5740a787087c562d975dbca98a32915c5f7664`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Imaging selection and its own card mass differ from AIM9M; shared WDU17/B reference is explicitly inherited, not independent measured X case geometry. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### asraam

asraam: W=10kg,C=3.69kg,M=5.3635kg,D=0.1494m,L=0.182215453m,t=0.00798900977m,N=1340,v=1860.21902m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 2, "published": 2}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `3a9ff045dcea44fa4b1634ac20d18377bc86066c02d09952fd16ead222ab1db8`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### iris-t

iris-t: W=11.4kg,C=4.21kg,M=6.1115kg,D=0.1143m,L=0.355035267m,t=0.0061067559m,N=3419,v=1861.10539m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 2, "published": 2}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `e06568f765da8fc98e01a63635d25df83678c75adff7e44f86ca30782fb23c4c`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### magic-2

magic-2: W=12.7kg,C=6kg,M=5.695kg,D=0.1413m,L=0.294394202m,t=0.00555140337m,N=4240,v=1993.66449m/s; Magic2 frontal detection and a historical multi-element description are not evidence for this scalar reticle. Pre-fragmented wording is represented by equivalent point fragments, not an authored preform map.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `fcd05a69e0d52c6d8ddaca13d5e82fc6cec6fc02ec610d00dd12bafa173d84c3`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Magic2 frontal detection and a historical multi-element description are not evidence for this scalar reticle. Pre-fragmented wording is represented by equivalent point fragments, not an authored preform map. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### meteor

meteor: W=12kg,C=4.43kg,M=6.4345kg,D=0.1602m,L=0.190212539m,t=0.00856236271m,N=1306,v=1860.71436m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 2, "published": 2}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `fe94f07868fd02c2191815d173742c422f04ff3a334854ec6424dc8026eebef6`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### mica-em

mica-em: W=12kg,C=3.7kg,M=7.055kg,D=0.144m,L=0.216322909m,t=0.0091835899m,N=1160,v=1675.9353m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `6b9671b53578c67a316fbaaf669744efa33105152bc06a12d99d2969e515f0e9`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### mica-ir

mica-ir: W=12kg,C=3.7kg,M=7.055kg,D=0.144m,L=0.216322909m,t=0.0091835899m,N=1160,v=1675.9353m/s; The actual receiver is dual-band imaging with separate channel and pairing policies, not the single-band or scalar-reticle receiver.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `be2ece63ca954c564bcd479cde7b251b13e3b9a424b939ace84944a34281d6a1`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: The actual receiver is dual-band imaging with separate channel and pairing policies, not the single-band or scalar-reticle receiver. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### pl-10

pl-10: W=11kg,C=4.06kg,M=5.899kg,D=0.144m,L=0.215777795m,t=0.00769820768m,N=1647,v=1860.48921m/s; Ambiguous fragmentation/expanding-rod description is resolved by the authored fragmentation kind and selected zoned_case.v1; this source does not implement an expanding connected rod.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 4}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `52bf1236bb2e3f25fd285f31de77bf9439a6da42be96c2b30d42bce366689012`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Ambiguous fragmentation/expanding-rod description is resolved by the authored fragmentation kind and selected zoned_case.v1; this source does not implement an expanding connected rod. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### pl-12

pl-12: W=20kg,C=9.2kg,M=9.18kg,D=0.1827m,L=0.273265038m,t=0.00745590788m,N=2821,v=2124.43111m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 2, "published": 2}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `3a1393c8c0c97feae60ee0dc765b14429cb8ded8a986d2f5a29fe0f853e02e76`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### pl-15

pl-15: W=22.5kg,C=8.3kg,M=12.07kg,D=0.1827m,L=0.274111632m,t=0.00977286114m,N=1647,v=1859.88876m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 3, "published": 1}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `be85e52fd52b8927b9576d532c1a1bd98e483fdf08ba1dbd0cb3230d3dde8034`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### pl-8

pl-8: W=11kg,C=5kg,M=5.1kg,D=0.144m,L=0.240365657m,t=0.00597469498m,N=3046,v=1946.65705m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `98854317952a183567c341ded5a403b96ce7564baa77a1b0c53dc64e8aeff11d`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-27er

r-27er: W=39kg,C=15kg,M=20.4kg,D=0.234m,L=0.295713106m,t=0.0119543016m,N=61,v=1400m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `11e9942ed61672a7780eba6cbfb987f5e117804fdded26844e32ea90ccb8b323`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-27et

r-27et: W=39kg,C=15kg,M=20.4kg,D=0.234m,L=0.295713106m,t=0.0119543016m,N=61,v=1400m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `6a65f01012ba423c3fb71cf5fae1dfa6fea30665d7a71d486324f85ffcc9c187`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-27r

r-27r: W=39kg,C=15kg,M=20.4kg,D=0.207m,L=0.377886691m,t=0.0105749591m,N=61,v=1400m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `3f2892a997e0b210303b068cca39fd2e8f6effba3eaf9be2b78ca7006b1d651f`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-27t

r-27t: W=39kg,C=15kg,M=20.4kg,D=0.207m,L=0.377886691m,t=0.0105749591m,N=61,v=1400m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `5c4bd24f272137c1eb6030b86e6b9146a4f86d7dd51e9a4f564ec310cf068fc2`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-37m

r-37m: W=60kg,C=22.14kg,M=32.181kg,D=0.342m,L=0.20863417m,t=0.0182880947m,N=670,v=1860.21902m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 2, "published": 2}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `336eeea078aa75bcedcc28ceac066c417b83a6c83607d40d2d18bc8c529b0fb6`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-60m

r-60m: W=3.5kg,C=1.35kg,M=1.8275kg,D=0.108m,L=0.12476169m,t=0.00549962215m,N=62,v=1400m/s; The card description mentions depleted-uranium rods, but the selected zoned-case source uses steel7850kg/m3 and discrete equivalent rods; no DU density, pyrophoricity or connected-ring capability is implemented by that name.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `4308f11295ae855004590be74303310362f3274574b5912725832c3c347b3efd`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: The card description mentions depleted-uranium rods, but the selected zoned-case source uses steel7850kg/m3 and discrete equivalent rods; no DU density, pyrophoricity or connected-ring capability is implemented by that name. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-73

r-73: W=8kg,C=3.75kg,M=3.6125kg,D=0.1485m,L=0.167172285m,t=0.00590061928m,N=79,v=1400m/s; Rod description maps to discrete finite rod elements under zoned_case.v1, not connected_ring.v1; exact weld topology and material construction are not inferred.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `1f74174d9fca580f4d0d89dde11d16862607079f77654160b3dbe4b54a5f7767`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Rod description maps to discrete finite rod elements under zoned_case.v1, not connected_ring.v1; exact weld topology and material construction are not inferred. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-77m

r-77m: W=22.5kg,C=9.725kg,M=10.85875kg,D=0.18m,L=0.306252113m,t=0.00798746018m,N=71,v=1400m/s; Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 2, "published": 2}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `2021298e53e0dd2ad34149d7aa8a3af2eca8d8cfc1d036ec74350b27b7a02886`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### super-530d

super-530d: W=30kg,C=8.1kg,M=18.615kg,D=0.2367m,L=0.189023917m,t=0.016870521m,N=494,v=1614.09247m/s; The card explicitly inherits Super530F warhead information; exact D case geometry and fragment distribution remain inferred.

Parameter/source matrix: 4 resolved roles; original basis counts {"assumed": 1, "published": 3}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `681f440b146a1e430c164b425b0e946a686f046c4fcc80a8f7a877a6cadb4290`; selected mechanism-source SHA256 `33bc9dc603490c5ed9c3a724a82c314104984d1df6d68b81554ff727d1e13431`.

Fidelity matrix: The card explicitly inherits Super530F warhead information; exact D case geometry and fragment distribution remain inferred. Case partition15%, diameter90%, steel density and equivalent shape are generic assumptions for V0; descriptor refs do not expose these as per-model measured inputs. Blast TNT equivalence differs from fragment-driving energy; legacy equivalence is inferred from Gurney, V2 has a separate explicit TNT record. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

## Actual material-law parameterization

For every row, current World scene emission uses `m_scene=M/N`; the retained
synchronous legacy assessor uses its pre-round nominal `fragmentKg`. These are
separate entry points. The scene emitter draws starts inside a finite inferred
shell even for V0: axial position in `[-L/2,L/2]`, azimuth uniform, and radius
squared uniform between `max(0,ro-t)^2` and `ro^2`. V2 instead uses its exact
annulus. Legacy synchronous V0 assessment retains its point-source angular law.
Discrete rod segments span the complete equivalent length parallel to the burst
axis; only the explicit connected selection shares polygon endpoints. Its
retained 85–95 degree zone record does not drive the connected-ring emitter.

The current THOR implementation, for plate thickness h in inches, presented area
A in square inches, fragment mass m in grains and speed v in ft/s, is
`v_out=max(v-10^7.047*(h*A)^1.029*m^-1.072*sec(theta)^1.251*v^-0.139,0)`.
The SDK clips angle magnitude to1.45rad and cos(theta) below by0.12, then converts
the result to m/s. Each model's JSON row carries its own fragment mass and area
in this same law. Those angular guards are numerical policy, not a calibrated
near-grazing validity extension. The original TR47 experimental domain has not
been independently re-established in this documentation pass; the coefficients
are accurately identified as the existing steel/2024-T3 empirical code, not a
newly verified universal material table. DU rods, composite delamination and
actual case material substitutions require distinct evidence.

The blast implementation evaluates `exp(sum(c_i*ln(Z)^i))`, using
`Z=R/(0.5*W_TNT)^(1/3)*(p_ambient/101325)^(1/3)`. Pressure scales with ambient
pressure; positive-phase impulse also scales with charge^(1/3), pressure^(2/3)
and sqrt(288.15/T). Scaled-range pressure clamps are0.2–198.5 and impulse clamps
0.2–158.7. Its ideal reflection formula is
`2*Ps*(7*Pa+4*Ps)/(7*Pa+Ps)`. [The NTIS Swisdak record](https://ntrl.ntis.gov/NTRL/dashboard/searchResults/titleDetail/ADA526744.xhtml)
identifies a simplified hemispherical surface-burst fit; the half-charge
free-air mapping is the SDK's explicit approximation, not a result supplied by
that reference. No fragment arena, plate campaign or blast measurement is
created by evaluating these equations.

## Parameter equality is a disclosed modeling gap

When complete numeric policy and selected mechanism/receiver coincide, the
JSON lists `identical_numeric_policy_models` explicitly. This is reused
engineering parameterization, not proof of identical hardware or separate
per-model tuning. Every model still needs its own actual capture. Different
equipment selection IDs, source notes and metadata hashes alone do not supply
different physical response, OEM internal construction, or calibration.

Sensitivity coefficients in the JSON are for the uncapped Gurney cylinder
speed. All seven current rod-source selections reach the retained1400m/s cap.
Their local source-speed derivatives with respect to case mass, charge and
Gurney constant are zero only inside that capped branch; the boundary is
nonsmooth. The cap does not remove uncertainty in fragment mass, count, geometry,
coverage or penetration. R-77M, like R-60M and R-73, selects discrete rods under
zoned_case.v1; only the four R-27 selections instantiate connected rings.
