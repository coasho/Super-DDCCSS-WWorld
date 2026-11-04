# SIM-M03 finite engineering sensing

Baseline: main `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`. Owner: sensing worker.
The root owns every build, CMake registration and World integration. Existing
dirty files are preserved. No public layout or legacy provider meaning changes.

## Exclusive write allowlist

- This plan; `docs/sdk/weapon-sensing-engineering.md`.
- `src/simulation/mission_systems/weapon/Seeker.h`, `Seeker.cpp`.
- `src/simulation/mission_systems/components/interfaces/SeekerExecutionV2.h`.
- `src/simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.h`, `EngineeringSeekerV3.cpp` (new).
- `src/simulation/mission_systems/components/registration/SeekerProvidersV1.cpp`.
- `tests/simulation/test_weapon_sensor_engineering.cpp` (new).
- `tests/simulation/test_weapon_seeker_models.cpp`: retain ideal V2 input-contract fixtures explicitly; actual-current-profile selection remains a separate all24 case. Existing semantic assertions are retained, and V3 physics has independent coverage.
- `tests/simulation/test_weapon_sensor_native.cpp` (new): exact-current24 V3 factories through real native releases at frameSkip1/2/4, with clock/pointing/variance/demand/outcome instrumentation.
- `tests/simulation/fixtures/weapon_v2_assets.py` (new): generate an isolated explicit V2 test asset tree from immutable current inputs with before/after hashes. Root CMake stages the unchanged original V2 native test executable and required DLLs beside that tree. No current card, installed package or original physical assertion is changed.
- Sensing descriptors/parameters only in `assets/stores/{aim-120c,aim-120d,aim-7m,aim-9m,aim-9x,asraam,iris-t,magic-2,meteor,mica-em,mica-ir,pl-10,pl-12,pl-15,pl-8,r-27er,r-27et,r-27r,r-27t,r-37m,r-60m,r-73,r-77m,super-530d}.json`.
- `tools/models/hangar/hangar/components.py`: versioned sensing descriptor recognition and native metadata stamping.
- `src/simulation/mission_systems/components/implementations/guidance/MeasuredPnGuidanceV2.cpp`: predict delayed angular measurement to the decision time without refreshing its measurement epoch.
- The same guidance file and `tests/simulation/test_weapon_sensor_engineering.cpp` own the delayed midcourse epoch correction: extrapolate the posterior from its original epoch exactly once; retain that original epoch in the input.
- Generated products are regenerated through the normal root-owned Hangar entrypoint.

## Method and acceptance

New `.v3` selections wrap the existing four `.v2` association mechanisms.
Legacy V2 numerical/binding behavior stays available. V3 performs physical
received-signal admission before association: monostatic/bistatic radar equation,
thermal receiver noise, finite angular-cell background rejection and point-source
IR irradiance with atmospheric extinction. Target/flare scene classes never enter
association. Passive range stays in sensor physics only.

Each receiver owns an inertial pointing vector with finite lag/rate and mechanical
field of regard, scheduled exposures, delayed delivery and a bounded history.
LOS estimation uses a finite response, tangent-plane error and declared angular
variance. Sample/receive epochs remain distinct. Repeated reads do not redraw
noise; reset clears all per-release state and independent streams.

References: Palumbo et al., Basic Principles of Homing Guidance, JHU/APL 29(1),
2010/revised 2018, pp.29-34; Jackson, Overview of Missile Flight Control Systems,
same issue, pp.9-14. Existing native radar/Detection physics is reused where its
measurement contract applies; no fighter profile or Python runtime is imported.
All new constants are explicit nonzero-uncertainty engineering assumptions,
not exact fitted equipment measurements. Noise samples represent repeatability;
parameter uncertainty is a separate disclosed screening interval.

RED first: new independent sensor cases must fail with unavailable V3 factory
before production changes. Root executes RED and subsequent native GREEN gates.
Cases cover independent inverse-power scaling, thermal noise, bistatic legs,
IR extinction/background, finite pointing, processing latency/epochs, filtered
rates, repeat reads, invalid/rewound input, reset and seeded reproducibility.
All24 native releases, installed public use and full regression remain root gates.
No pending engineering function may be labeled calibration-only.

Root observed the intended RED on 2026-11-02: one case, three assertions, one
failure at V3 configuration admission. Raw log:
`build/architecture-reviews/sim-m03-complete-20261102/sensor-red.log`.
Native build slot was released before production edits. Delayed attribution uses
the original sample epoch and original slot index; root retains the corresponding
bounded scene mapping. No entity IDs are introduced into detector association.

Root-authorized effects card handoff: only `r-27r`, `r-27er`, `r-27t`, `r-27et`
warhead descriptors change implementation ID to `native.warhead.connected_ring.v1`
and disclose homogenized uncalibrated connected-ring execution. All four scalar
role sets and bindings remain unchanged. The effects owner implements native
factory/registry behavior. The generator's `components.py` has no warhead ID
whitelist or warhead native metadata, so no generator warhead change is needed.
