# SIM-M03 physical implementation completion

Main baseline: `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`.
The user's continuation explicitly requires production functionality, theory and
model fidelity, rather than descriptive dispositions for missing mechanisms.

## Complete vertical packages

1. Radiometric IR/IIR: signed scene/background contrast, bounded focal-plane
   irradiance, optical PSF, pixel/electron/read noise and saturation, image-space
   extraction/association/tracking. Preserve finite gimbal/exposure/processing
   behavior and keep identity/truth outside association.
2. Weapon inertial navigation: gyro and accelerometer sampling, strapdown
   attitude/velocity/position mechanization, Earth/gravity terms, coupled error
   covariance and GNSS aiding/outage/reacquisition. No target truth or periodic
   ownship truth reset.
3. RF receiving: angular antenna response, finite measurement-cell clutter and
   detection processing using the accepted native radar processing where
   applicable. Preserve actual SARH illumination/path semantics and unavailable
   range rather than manufacturing passive Cartesian information.
4. Variant actuation and integration: implement missing source-backed TVC and
   other physically required mechanisms; use authored/source-backed carrier
   configurations through native sensors, consent and release. A compatibility
   mount is not a substitute for a required fitted model.

## Ownership and execution

Use the existing three agents for disjoint complete packages. Root owns shared
World/header/provider registration/card/CMake integration and the sole serial
native build/test and GUI slots. Agents may not execute native builds or viewers.
Record exact source/symbol windows at handoff. No branches or worktrees.

The first native write windows are new private sensing/image implementation and
interfaces for the sensing owner; new strapdown implementation and interfaces
for the navigation owner; source-backed fin/TVC/propulsion generator and model
mechanisms for the model owner. Shared paths remain root-owned until a recorded
exclusive handoff. Existing signatures/layouts/vtables remain unchanged.

Research and products are indexed at
[docs/research/sim-m03](../../research/sim-m03/README.md). Preserve primary source
versions, licenses and numerical references. Large original/run products belong
under `artifacts/sim-m03/`, with portable relative paths and exact hashes.

No missing function is closed by labeling it a model-order limitation. Existing
engineering assumptions remain explicit, but the required production mechanisms
must execute through the normal SDK and Viewer before final push/delivery.


## Active integration windows, 2026-11-02 continuation

Main HEAD verified before this window: `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`.

Root exclusively owns `World.h`, `World.cpp`, `WorldWeaponLinks.cpp`,
`WorldWeaponControls.cpp`, new `WorldWeaponDiagnostics.cpp`,
`weapon/NavigationConfigurationV2.{h,cpp}`, `GuidanceProvidersV1.cpp`,
all CMake registrations and native/GUI execution. Root added selected strapdown
navigation to the actual released-weapon lifecycle; own body-frame native
acceleration is resolved using `C * (UVWdot + PQR x UVW)`. Original V1 behavior
remains selected for old cards. Twenty-five authored guided stores explicitly
select V2 with recorded engineering noise/bias/initialization policy.

The sensing owner exclusively owns `WorldEngagements.cpp`,
`WorldWeaponSensorDiagnostics.cpp`, `ReceiverConfigurationV1.{h,cpp}`,
`SeekerProvidersV1.cpp`, image/RF implementations and tests, the additive
`SignalDiagnosticsV1.h`/optional internal `Signal.cpp` diagnostic path, and the
serialized sensor-parameter/binding write window in `assets/stores/*.json`.
It must preserve navigation parameters and existing radar processing behavior.

The diagnostics owner exclusively owns the new diagnostic contract/transport/UI,
`src/applications/viewer/main.cpp`, and their focused source tests. The actual
world PID, incarnation and published frame bind the new sidecar. Existing
WorldLayout v9 and fsrec v8 are unchanged. Pixel and RF units must match native
processor output; unavailable segmentation or thresholds cannot be invented.

The model owner exclusively owns passive rolleron generation and the relocated
`simulation/flight/PassiveRolleronV1.{h,cpp}`,
`JsbsimModel.{h,cpp}`, new native variant-actuation tests, V2 ducted-rocket
admission/provider/generation, and corresponding method documents. Its native
substep implementation must preserve World external reactions and old models.

The root retains sole serial configure/build/test and GUI slots. Focused native
execution first found and retained a thermal two-source segmentation failure
(four components instead of two); its original physical predicate is retained.
RF receiver and strapdown native cases executed successfully at that checkpoint.
These observations are development evidence, not final program acceptance.
