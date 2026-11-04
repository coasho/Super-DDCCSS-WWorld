# Native thermal image and quadrant receiver

Baseline: main `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`.
The root owns native execution and shared registration, configuration parsing,
cards, World and CMake. This owner changes only:

- `components/implementations/seeker/EngineeringSeekerV3.{h,cpp}`.
- New `components/interfaces/ThermalSeekerExecutionV1.h`.
- New `components/implementations/seeker/ThermalImageReceiverV1.{h,cpp}`.
- New `components/interfaces/SeekerNavigationPoseV1.h`.
- New `components/interfaces/RadioSeekerExecutionV1.h`.
- New `components/implementations/seeker/RadioReceiverV1.{h,cpp}`.
- New `tests/simulation/test_weapon_thermal_image.cpp`.
- New `tests/simulation/test_weapon_radio_receiver.cpp`.
- This plan and `docs/sdk/weapon-thermal-image.md`.
- New `docs/sdk/weapon-radio-receiver.md`.
- Root-granted `weapon/Seeker.cpp` descriptor-normalization hook and
  `components/registration/SeekerProvidersV1.cpp` physical receiver construction.
- New `components/support/ReceiverConfigurationV1.{h,cpp}`.
- Root-granted `radar/Signal.cpp` opt-in diagnostics only; new
  `radar/SignalDiagnosticsV1.h`. Existing `dwell` and `Snapshot` ABI retained.
- New `src/simulation/flight/NativeThermalSourceV1.h`; flight implementation is
  the separate current flight owner's exclusive window.
- Root-granted `src/simulation/runtime/world/WorldEngagements.cpp` live sensor
  scene/dispatch/navigation integration and new
  `WorldWeaponSensorDiagnostics.cpp` native-array diagnostic producer.
- Root-granted fixture-only retained-mode normalization/actual sibling
  forwarding in `tests/simulation/test_weapon_sensor_engineering.cpp`,
  `test_weapon_seeker_models.cpp` and `test_weapon_sensor_native.cpp`.
- Root-released 24 AAM sensor descriptor/parameter JSON window, plus the
  effects owner's exact Meteor propulsion-v2/residence handoff.

Source paths above are beneath `src/simulation/mission_systems/`.

An additive sibling interface receives explicit detector policy and an
epoch-aligned radiometric scene. Source patches contain direction, projected
support, spectral-band radiance, transmission, path radiance and finite range;
multiple patches can share separately carried referee attribution. The detector
and association never use attribution, IDs or semantic source classes.

Implementation forms a finite focal plane with sky/ground background, opaque
patch coverage, Gaussian optical spreading, finite exposure and pixel response,
photon/dark/read noise, full-well clipping and ADC quantization. Imaging derives
components and measured shapes from detector counts. Nonimaging derives signed
quadrant flux differences, and refuses unavailable/zero-contrast support. Both
feed the existing finite pointing, capture, tracking and delivery clocks.

Retained V2 and the separately selected scalar V3 path retain their meaning.
The new explicit optical binding has no silent valid detector defaults. Root
registers sources/tests and runs the actual native and integrated gates.

The root subsequently assigned the RF receiver to the same owner. It reuses
native `radar::signal::dwell` and `rf::Pattern`; existing Radar/Signal/Antenna
implementations are read-only. The additive radio seam accepts physical echoes,
terrain patches, interference and actual SARH illumination waveform. Both new
receiver selections require a current admitted navigation pose: physical scene
formation remains native, while cue and measured LOS reconstruction use the
estimated rotation/position/velocity. Actual image/spectrum diagnostics are
exposed through private sibling accessors for the separately owned UI.

## Actual SARH transmitter/listener clocks

The root authorized the following additional exact paths at the same main HEAD:
`radar/IlluminationClockV1.h`, the additive getter only in `radar/Radar.h`,
`components/implementations/seeker/RadioTemporalCoincidenceV1.{h,cpp}` and
`tests/simulation/test_weapon_radio_temporal.cpp`. The existing radio interface,
receiver, EngineeringSeekerV3, ReceiverConfigurationV1, SignalDiagnosticsV1 and
optional Signal implementation seam, WorldEngagements scene clock input, sensor
diagnostic producer and these two SDK documents remain this owner's scope.
Radar.cpp, cards and Python generation inputs remain frozen.

Root additionally assigned the physical ARH run condition in
`ActiveRadarSeekerV2.{h,cpp}` (nonvirtual shared phase preview only, no class
fields/vtable changes) and the existing EngineeringSeekerV3 private wrapper.
An authorized phase decision gates actual RF energy/noise, waits a full CPI
after activation and preserves a fresh family's launch estimate through first
configuration. The direct physical wrapper fixtures prove no standby cells,
finite first capture/delivery and unchanged first RF random realization after
standby. Native illuminator programs support the processor's 32 intervals;
the seeker's own configured waveform design remains bounded at five.

The production listener uses the actual native sequential PRF program and
clock, with no pre-observed illumination history. A short observation takes
only complete pulses actually within its finite window. Unequal interval energy
uses the actual pulse counts/PRFs. Measured folded peaks from distinct native
PRFs may coincide across exposures within the existing explicit track-memory
parameter. Each range/rate alias is advanced by its measured Doppler to the
newest capture epoch; delayed bearings retain their original navigation
reconstruction. Illumination loss and program/phase changes clear this history.
The configured M is retained, and no SARH Cartesian range/rate is published.

The independent fixtures enumerate a 90 ms transmitter's pulses against a
12 ms listener and require real three-PRF temporal agreement with measured
range migration and LOS motion. Native execution remains root-owned.
