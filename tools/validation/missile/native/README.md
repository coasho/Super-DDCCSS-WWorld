# Optional VSPAERO public-library regression

This directory contains a small geometry regression for OpenVSP 3.55.0 /
VSPAERO 7.2.2. It exercises public `VSP_GEOM::ReadFile` and `UpdateMeshes`, with
one quad, two native alternate triangles and a two-node trailing-edge chain.
It performs no flow solve and has no SDK dependency. Ordinary Python tests
mock every external process and never compile this fixture.

The native audit requires four cases: level 0 identity, level 1 identity, and
level 1 / level 2 with one unused vertex before the physical vertex IDs. The
unused point preserves the physical geometry but forces a nonidentity native
index map. All four cases must exist. An unavailable real level 2 or an identity
map in either unused-point case is invalid setup, rather than a skipped test.
The fixture translates the four physical vertices and checks root attachment,
exact physical-surface and other-node immutability, finite coordinates and
repeated-update idempotence. Surface membership comes from actual connectivity,
not a contiguous prefix inferred from `NumberOfSurfaceNodes()`.
Every case also calls both public zero-control `DeflectControlSurfaces` and
`UnDeflectControlSurfaces` paths. Each must preserve all finest-grid coordinates
and copy all eight internal trailing-vortex nodes from their fully mapped
finest-grid coordinates. The report retains distinct guards for both paths.

The original source was independently shown to pass the identity controls and
fail both nonidentity cases; the separately corrected source passed all four.
Those native runs used the retained preparation harness SHA256
`608df630302f73008ecd873ffd8795901222fd7e21cd73e6e1b5b51675b28240`.
The maintained copy makes level 2 mandatory, strengthens its nonidentity
precondition, and checks the two internal-copy paths. This extended fixture
SHA256 `02fc70c8b07fcf2ed8698329d54228cd3b7f26e54764f8964eb358096c7061de`
was then run against the root-attachment-only correction: both identity controls
passed, while both nonidentity cases failed the two internal-wake-copy guards.
Their physical-surface/root checks passed, isolating the additional defect.
Root then ran these same maintained fixture bytes against the complete
three-method correction: all four cases and every additional guard passed.
The maintained report parser independently accepts that actual green report.
The optional maintained runner was then executed on these same inputs: the
original library fails with native exit 2, and the explicit three-method override
passes all four cases. The manifests are
`build/architecture-reviews/sim-m02/aero/native-original-maintained/manifest.json`
and `build/architecture-reviews/sim-m02/aero/native-wake-mapping-maintained-valid/manifest.json`.
They retain actual compiler/run commands, input/executable hashes, raw logs and
strictly reassessed reports. A separate mistyped-library attempt remains failed
under `native-wake-mapping-maintained/`; it executed no compiler or fixture.
A successful geometry regression does not establish aerodynamic accuracy,
nonlinear wake convergence, equipment fidelity or SIM-M02 completion.

## Explicit execution

The optional runner requires an explicitly supplied GNU-compatible compiler,
the original pinned Solver include directory, and a compatible native library.
The supported local reproduction uses the existing Windows UCRT64 toolchain,
C++17 and OpenMP. `-iquote` avoids the upstream `time.H` shadowing system headers.
No compiler/library is discovered automatically, no patch is applied, and no
external source, library or SDK build input is modified.

```powershell
D:/ENV/python/3.13/python.exe -B tools/validation/missile/aero_vsp_native.py --compiler D:/ENV/DevLanguages/Cpp/msys2/ucrt64/bin/g++.exe --solver-include D:/ENV/physics/openvsp-3.55.0-source/OpenVSP-OpenVSP_3.55.0/src/vsp_aero/Solver --library D:/ENV/physics/openvsp-3.55.0-source/build/vsp_aero/Solver/libvspaero_lib.a --output build/architecture-reviews/sim-m02/native-original
```

For a separately reviewed corrected geometry object, explicitly add
`--geometry-object D:/ENV/physics/openvsp-3.55.0-wake-mapping-fix-v2/VSP_Geom.C.obj`
and choose another new output directory. This object is linked before the
explicit library. Its source/build provenance must be retained by its builder;
the runner hashes the object but does not infer or certify how it was built.

The output directory must not exist. Compile and run limits default to 120 and
30 seconds; explicit limits must be finite and in `(0, 300]`. The runner caps
OpenMP at two threads and sets `TMP`, `TEMP` and `TORCH_EXTENSIONS_DIR` to
`D:/ENV/tmp`. Hold the project's serial native build/test slot before invoking
it. The manifest retains commands, working directories, limits, raw logs,
compiler version output, direct source/header/library/object/compiler hashes,
executable hashes, raw report and input-change checks. It does not package the
compiler's entire sysroot or replace the external build's own provenance.

CLI exit 0 requires native exit 0 and all guards in all four actual report
cases. CLI exit 1 preserves a failed validation manifest, including missing or
mismatched external inputs. Invalid CLI syntax/time limits or an existing
output directory return exit 2 before starting external work.
The native fixture itself returns 2 for a detected regression and 3 for invalid
setup. An original-library regression is retained as failure evidence, never
promoted to model acceptance.

## Pinned upstream source and modification

Upstream: [OpenVSP 3.55.0](https://github.com/OpenVSP/OpenVSP/tree/5240ef210f41c5d6aeed0ba7687f6fab0ca0cafb),
commit `5240ef210f41c5d6aeed0ba7687f6fab0ca0cafb`.
The required original `src/vsp_aero/Solver/VSP_Geom.C` SHA256 is
`0dd0e712b0136dab67a5425a5c086207b4c26727b3334dd68caa53dade87f36f`.
The runner rejects a different original source even when an override object is
explicitly supplied.

`openvsp-3.55.0-wake-root.patch` changes only three index-copy sites in
`VSP_GEOM::UpdateMeshes(int)`, `DeflectControlSurfaces()` and
`UnDeflectControlSurfaces()`. Both TE and wake IDs belong to the solve grid.
The insertions map each through every immediate `FineGridNode()` link before
the existing finest-grid x/y/z assignment or read. This preserves level 0 and
identity maps and corrects nonidentity maps without changing public
layouts/signatures, geometry, wake equations, core law or solver settings.
The proposed complete source SHA256 is
`eb8e817e30ef188636fb02f87812508958e75a7d51c09361b37cdb7e09729aa2`.
The exact three-hunk patch SHA256 is
`b4e97d11d10418ef533ecfbf72830bc9298fa67a5797b95422210b6c1be9cd08`.
Independent byte comparison confirms exactly two eight-line copy-map insertions
and one nine-line root-map insertion; removing those restores the original
source exactly. Other index-copy paths and their independent acceptance remain
outside this patch. The runner never applies it or replaces the external source.

Modification originator: **Super-DDCCSS-WWorld**, dated **2026-10-18**.
Super-DDCCSS-WWorld identifies itself as the originator of this alteration and
consents to its characterization as a Modification derived, directly or
indirectly, from Original Software provided by the Government Agency, NASA.
This statement records the modification provenance required by NOSA 1.3
section 3.C. The exact upstream license and copyright notices accompany the
patch in [LICENSE.OpenVSP](LICENSE.OpenVSP); upstream headers and binaries are
not vendored here. The small public-API fixture and Python runner are separately
authored project tools; the patch's upstream source context retains its NASA
license and attribution.

## Exact triangle-intersection correction

`openvsp-3.55.0-exact-intersection.patch` addresses a separate geometry-core
defect before `TTri::SplitTri`. Twelve captured binary64 triangle pairs show
missing, collapsed or incorrect intersection segments. All supplied segments
survive the subsequent constraint cleanup and split triangulation; weakening
the authoritative `TTri::InTri` filter would not repair the missing segments.
The original kernel fails 64 of the 192 captured/boundary checks. Its other
128 checks and all 88 independent point/gap controls pass.

The patch changes only `src/external/tritri/tri_tri_intersect.cpp` and its
`CMakeLists.txt`. Finite binary64 coordinates import exactly into upstream's
already bundled `gte::BSRational<gte::UIntegerAP32>`. Plane cuts, duplicate
points, projection-axis selection and closed-interval overlap stay exact;
only accepted segment endpoints round to double. There is no overlap epsilon
or segment extension. The existing coplanar routine and its untouched endpoint
contract remain. Exact zero-area inputs delegate to the byte-preserved private
legacy kernel; nonfinite inputs explicitly return miss before rational
conversion. This is not a general certification of extreme floating ranges or
of the legacy degenerate/coplanar algorithm. No predicate initializer, new
binary dependency, SDK signature/layout or runtime model changes are added.

| File | Original SHA256 | Corrected SHA256 |
| --- | --- | --- |
| `tri_tri_intersect.cpp` | `57157c13bd9d42fd90e6a6ba88608d3471269a9df9ab61b5ca351c4ad15af37e` | `aa5cef485d52e58b5e6838dab57f73d386efd557810f7cf691aa089bd3466230` |
| `tritri/CMakeLists.txt` | `0306636c79275d5aa4d84385871f667c47876aebb5bd6f53c11198db920670dc` | `c71336dd7c58f8c53154eebda7d97d6bddf8087809702cf76e2674753b23cfcd` |

The patch SHA256 is
`1754739d9581d9c929beec32b182e486b7f67da6a1affe3e5f04ebd45e3ebfea`.
Actual forward/reverse application reproduced both proposed and original bytes,
including upstream's missing final CMake newline. A rejected first proposal
used rounded cut coordinates for overlap ordering. It failed all 44 independent
true-point contacts and 44 one-ulp positive gaps in a separately compiled
native binary; its evidence remains separate from the accepted correction.

The maintained fixtures retain the exact captured inputs and independently
computed rational endpoint expectations. `vsp_triangle_boundary.cpp` includes
the 112 captured/elementary cases and adds 80 boundary controls. It calls the
actual native `TTri::InTri` for positive noncoplanar endpoints.
`vsp_triangle_contact.cpp` adds the 88 independent point/gap cases. Its endpoint
rounding budget never changes a hit/miss decision. All 280 corrected guards
pass; the 104 positive segments in the original 112-case set match their
rounded exact oracle with zero endpoint error. The original control remains
64 failures, and its 88 point/gap guards remain passing.

`vsp_triangle_nonfinite.cpp` separately checks NaN and both infinities in each
of the 18 input coordinates: all 54 cases require a miss, cleared coplanarity,
unchanged input bits and untouched endpoint sentinels. Two finite dyadic
hit/gap controls prevent an always-miss implementation from passing. The
original kernel passes only those two finite controls; the correction passes
all 56 checks. These exercise the new rejection policy, not an intersection
oracle for invalid geometry or additional legacy-degeneracy guarantees.

For explicit external reproduction, append an `include` of
[OpenVspIntersectionFixture.cmake](OpenVspIntersectionFixture.cmake) to a separate
OpenVSP 3.55.0 headless source copy's `src/vsp/CMakeLists.txt`. Use upstream's
supported MSVC/C++17 geometry/API build and exact bundled dependencies.
The module adds only external fixture targets and never applies a patch or
enters the SDK build. `FSIM_OPENVSP_INTERSECTION_SOURCE` optionally selects an
explicit source override for the fixture executables; its SHA256 is printed
at configure time. Root used that option with the immutable original kernel
for RED, then removed it for the corrected library. Keep raw reports separately.
The selector is cached: explicitly clear it with
`-DFSIM_OPENVSP_INTERSECTION_SOURCE=` when configuring the corrected default,
then rebuild all three fixtures. An old original-source override would otherwise
keep exercising that source even after the external library is corrected.

```powershell
$env:TMP = $env:TEMP = $env:TORCH_EXTENSIONS_DIR = 'D:/ENV/tmp'
& D:/ENV/DevLanguages/Cpp/cmake-4.2.0-rc2-windows-x86_64/bin/cmake.exe --build D:/ENV/physics/openvsp-3.55.0-geometry-stage-build/core --target fsim_vsp_intersection_boundary fsim_vsp_intersection_contact fsim_vsp_intersection_nonfinite --parallel 2
& D:/ENV/DevLanguages/Cpp/cmake-4.2.0-rc2-windows-x86_64/bin/ctest.exe --test-dir D:/ENV/physics/openvsp-3.55.0-geometry-stage-build/core -R '^fsim_vsp_intersection_(boundary|contact|nonfinite)$' --no-tests=error --output-on-failure --timeout 30
```

The boundary/contact fixtures were actually rebuilt and rerun in both modes; their
raw JSON logs exactly match the corresponding frozen audit fixtures, and the
corrected external CTest selection passed **2/2**. Evidence is
`build/architecture-reviews/sim-m02/aero/source-build/maintained-intersection/`.
The main geometry executable and protected source inputs remained unchanged
through those fixture-override checks.

The final maintained selection, including the separate nonfinite fixture,
passes **3/3**, emitting **336 passing native checks** in 0.134 seconds of
controller wall time. Its build takes 0.491 seconds. Evidence is
`build/architecture-reviews/sim-m02/aero/source-build/maintained-guard-final/`.
The separate original/corrected nonfinite execution retains a controller
postcheck type error in its original failed record; `verified-result.json`
checks the completed native logs and frozen inputs without rerunning them.
The final maintained CTest run independently exercises all 56 checks again.

Both unchanged saved CAD inputs were also replayed with the corrected geometry
core, with identical dormant/enabled diagnostic output. Independent checks
find closed outward single-component bodies, simple primary polygons, exact
alternate partitioning, eight connected thin sheets, complete 48-node trailing
edges and unchanged source geometry. The former buried lobes disappear at the
earliest post-trim stage in all 24 snapshots, using unchanged native-rounding
budgets. This establishes the scoped geometry correction; downstream flow,
grid convergence and equipment calibration remain separate gates.

Modification originator: **Super-DDCCSS-WWorld**, dated **2026-10-18**.
Super-DDCCSS-WWorld identifies itself as the originator of this intersection
alteration and consents to its characterization as a Modification derived,
directly or indirectly, from Original Software provided by NASA, under NOSA 1.3
section 3.C. [LICENSE.OpenVSP](LICENSE.OpenVSP) retains upstream's exact notice.
GTE headers/libraries are neither copied nor redistributed here; the patch
consumes their existing upstream Boost Software License 1.0 distribution.
