# VIS-M14: Tactical & EMSO 3-D Overlays, Multi-Camera Director Suite, Vulkan PiP & Fleet VFX Parity

**Delivered:** 2026-11-27 (`main`, base `c331fb70`)
**Closes:** `B-43` (High-DPI `MonitorGui`), `B-57` (8-engine telemetry & per-engine `exhaust_k`), `B-59` (rotorcraft/propeller manifests, placed exhaust heat haze, rotor downwash & blade-tip condensation helices), plus offline tile leaf `PagedLOD` warning elimination.

---

## 1. Architecture & ABI-Preservation Contract

The Viewer remains strictly read-only with respect to simulation physics: zero RF link-budget, jamming, CRLB triangulation, DLZ, gun/bomb ballistics, SAR/weather, or propulsion equations are duplicated in client code.

- **Frozen Contracts Preserved Byte-for-Byte:** `WorldLayout v9`, `VehicleState` (`kMaxEngines = 4`), `VehicleSample`, `fsrec v8`, `DebrisLayoutV1`, `CombatVfxLayoutV1`, `FusionFrameV1`, `WeaponDiagnosticsV1`, `FlightModel` vtable, and all public C++, C99, and Python ABIs are unchanged.
- **Additive Sidecar Wire Layout (`src/contracts/world_stream/TacticalOverlayLayoutV1.h`):** Seven `alignas(8)` trivially copyable wire records with strict finite/bounds/reserved-zero validators:
  - `TacticalRadarVolumeWireV1` (`96 B`): scan sector, instantaneous beam, STT/TWS state, RCS-modulated detection range (`B-37`), live $10\log_{10}(1 + J_0/N_0)$ jamming ratio, and burn-through range.
  - `TacticalJammerLobeWireV1` (`96 B`): `AN/ALQ-99` / `Compass Call` / `AN/ALQ-135` directional/omni RF jammer origin, boresight, half-angle, ERP, frequency, bandwidth, and comms-jam flag.
  - `TacticalEsmRayWireV1` (`96 B`): `AN/ALR-56M` / `AN/ALQ-218` bearing-only DF rays, emitter mode (`Search`/`Lock`/`Jammer`/`Datalink`), lock warning flag, and native 3-D Gauss-Newton FIM CRLB geolocation fix + CEP50 ellipsoid radii when valid.
  - `TacticalLinkPulseWireV1` (`80 B`): `Dms` / Link 16 carrier pulses, C2 consent arcs, Mode 5 IFF `1030/1090 MHz` rings, and PNT GNSS jamming drift rings.
  - `TacticalWeaponCueWireV1` (`128 B`): 5-D flyout-derived `launchZoneV2` (`R_aero`, `R_max`, `R_ne`, `R_min`, NEZ flag), McCoy/STANAG 4355 gun LCOS lead pipper, and wind-compensated Mk 82 CCIP ground impact point.
  - `TacticalSwathWireV1` (`96 B`): `ASARS-2`/`EISS-SAR` spotlight footprint boxes and `AN/APN-241` weather radar reflectivity swaths (`dBZ`).
  - `TacticalPropulsionExtWireV1` (`192 B`): up to `8` independent native engine channels (`throttle`, `thrustN`, `engineN2`, `afterburner`, `fuelFlowKgS`), plus rotorcraft/propeller RPM, rotor radius, and disk loading (`B-57`/`B-59`).
- **Wait-Free IPC & Lazy Replay Sidecar (`src/foundation/transport/TacticalOverlaySegmentV1.{h,cpp}`):**
  - Live triple-buffered seqlock shared memory `fsim.tact.v1.<world>` published from `World::collectTacticalOverlayFrame()` alongside `WorldPublisher` / `WorldMirror`.
  - Lazy `<recordPath>.tact` (`FSTACT1`) companion recorder & replay reader (created only when a recording produces tactical/propulsion frames).
- **Additive Propulsion Extension (`FlightModel.h`, `JsbsimModel.{h,cpp}`):** Non-vtable-breaking `PropulsionExtendedSourceV1` interface exposing all 8 JSBSim engine channels on `B-52H` and rotor/prop telemetry on `UH-60`/`UH-1H`/`C-130J` without altering `VehicleState::kMaxEngines` (4).

---

## 2. Tactical & EMSO 3-D Overlays, Multi-Camera Director & Vulkan PiP

- **3-D Tactical & EMSO Renderer (`src/visualization/ui/TacticalOverlays.{h,cpp}`):**
  - Toggleable per layer via `[O]`, `--overlays all|none|radar,jammer,esm,links,weapons,swaths`, or `MonitorGui` checkboxes.
  - **Zero-cost OFF path:** returns immediately with `0` vertices and `0.000 ms` measured overhead when all overlay bits, Cockpit mode, and PiP are disabled.
- **Multi-Camera Director Suite (`src/visualization/scene/CameraController.{h,cpp}`, `KeyHandler.cpp`, `main.cpp`):**
  - `[C]` / `--camera chase|orbit|overview|free|weapon|cockpit` cycles all 6 modes:
    - `Weapon` (`Mode::Weapon`): auto-follows the newest active released munition (`aim120c`, `aim9x`, `mk82`, etc.) with fallback to selected vehicle.
    - `Cockpit` (`Mode::Cockpit`): pilot eye mount with own-aircraft mesh hidden (no canopy clipping) and Mil-Spec vector Fighter HUD (heading tape, airspeed/altitude/Mach/G/AoA/RALT/VVI tapes, pitch ladder & FPV, TD box, `RADAR JAMMED` alert, vertical 5-D DLZ bar + `SHOOT` cue, McCoy gun LCOS ring, CCIP bomb-fall line, and RWR threat azimuth compass).
- **Zero-Copy Offscreen Vulkan Picture-in-Picture (`src/visualization/render/Viewer.{h,cpp}`, `TacticalOverlays.cpp`):**
  - Toggleable via `[I]` or `--pip` (note: `[P]` remains reserved off-limits by `mandatory_presentation`).
  - Renders a `480×270` offscreen `RenderGraph` gated by `pipSwitch_` (`0.0 ms` GPU when OFF, `~0.39 ms` GPU when ON), excluding main-camera screen-space backdrops (`kNotInPip`) and main `GpuTimer` marks, and presenting directly in ImGui via a compiled `vsg::DescriptorSet` (`CompileDescriptorSet` `Compilable` wrapper) with selectable `Auto (Weapon/Target)` / `Weapon Follow` / `Tracked Target` director modes.

---

## 3. Fleet VFX Parity (`B-57`, `B-59`), High-DPI `MonitorGui` (`B-43`) & Offline Tile Pruning

- **`B-57` (8-Engine Propulsion & Per-Engine `exhaust_k`):**
  - `Exhaust::kPerSlot` expanded to `8`; `VehicleVisuals` maps `b52h.glb`'s 8 twin-pod nacelles to logical engines `0..7` driven by `TacticalPropulsionExtWireV1` for both plumes and 8-engine `Airflow` contrails.
  - Manifest `exhaust` entries accept an optional 8th `<exhaust_k>` turbine exit temperature (`Exhaust::reheatTotalK`), authored in `hangar/model3d.py` and manifests for `f22a` (`1280 K`), `mirage2000` (`1180 K`), and `b52h` (`950 K`).
- **`B-59` (Rotorcraft & Propeller VFX):**
  - `hangar/rotorcraft/model.py` and manifests (`uh1h`, `uh60`, `c130j`) now emit `exhaust` and `rotor` lines.
  - `Exhaust` / `VehicleVisuals` synthesize `exhaustStack` thermal plumes for placed exhaust entries without nozzle meshes so rotorcraft and turboprops drive `WingVapour` heat-haze distortion.
  - `GpuParticles::addRotorEffects()` emits radial ground-effect downwash dust/sea-spray rings when hovering near ground (`AGL < 2.5 × R_rotor`) and spinning blade-tip / propeller-tip condensation helices in humid air.
- **`B-43` (High-DPI Responsive `MonitorGui`):**
  - `MonitorGui` folds sections under `CollapsingHeader`s (`Camera & Director [C / I]`, `Tactical & EMSO Overlays [O]`, `Selected Vehicle [Tab]`, `World & Weather`), adapts layout compactness to `DisplaySize.y / FontGlobalScale`, and constrains window height so it fits `1600×900 @ 150% DPI` cleanly without clipping or overlapping the bottom-left `vehicles` table.
- **Offline Tile Leaf Pruning (`OfflineTiles.{h,cpp}`, `ElevatedTile.{h,cpp}`):**
  - Added `OfflineTiles::hasAnyChild(z, x, y)` and `ElevatedTile::pruneOfflineLeaves()` so offline `PagedLOD` nodes whose `z+1` subtiles do not exist in the local cache collapse into plain `vsg::CullGroup`s, reducing `vsg::tile::read_subtile(..) could not load any subtiles.` log warnings from continuous spam to **`0`**.

---

## 4. Verification Evidence (2026-11-27)

### Dual-Preset Builds & Automated CTest Suites (`0` `-Werror` warnings)
- **`ucrt64-release`:**
  - `fsim_render_tests "~[gpu]"`: **67/67 cases PASSED (5,341 assertions)** (includes 4 new `test_vis_m14.cpp` cases covering `B-57`, `B-59`, offline leaf pruning, and `Weapon`/`Cockpit` cameras).
  - CTest targeted & boundary suites: **7/7 PASSED** (`contracts`, `mandatory_presentation`, `vis_m14_ui`, `destruction_debris`, `combat_vfx`, `tactical_overlays`, `sdk_boundary`), plus `24/24` `include_view_*` compilation checks (`include_view_session` retains pre-existing unrelated Jolt empty `-D` macro issue; `architecture_boundary` has 0 VIS-M14 violations).
- **`ucrt64-headless`:**
  - CTest targeted & ABI suites: **8/8 PASSED** (`contracts`, `simulation_api_link`, `training_link`, `world_adapter_compatibility`, `c_abi`, `combat_vfx`, `tactical_overlays`, `sdk_boundary`).

### Native 4K Fullscreen Visual Acceptance (`3840×2160`, 1× real time, RTX 5070 Ti, `WinSta0\Default` via `desktop-gui` MCP)
All runs executed fullscreen at primary `3840×2160` with full Earth imagery, terrain, clouds, ocean, shadows, and effects enabled, producing **`0` `read_subtile` warnings** across all 8 viewer logs:

| Scenario | Flags | Steady 4K FPS | `tact` Overlay CPU | Total 4K GPU | Visual Checks Verified |
| --- | --- | --- | --- | --- | --- |
| Baseline OFF | `--replay emso_tactical_showcase.fsrec` | **159.9–160.1 fps** (`p50 6.24 ms`) | **`0.000 ms`** | `4.22–4.64 ms` | Zero overhead OFF, zero `read_subtile` warnings |
| Overview EMSO + PiP | `--camera overview --overlays all --pip` | **159.6–160.4 fps** (`p50 6.24 ms`) | **`0.028–0.033 ms`** | `4.29–4.46 ms` | Radar TWS wedges, EA jammer cone (`51 dBW \| 9.86 GHz`), live burn-through arcs (`13.5 km / 8.8 km`), ESM DF rays, live Vulkan PiP (`viper-2`), `B-43` `MonitorGui` |
| Cockpit HUD + PiP + Missile | `--camera cockpit --overlays all --pip` | **159.9 fps** (`frame avg 6.25 ms`) | **`0.033 ms`** | `5.43–5.49 ms` | Own-mesh hidden, Mil-Spec Fighter HUD (`RADAR JAMMED`, 5-D `AIM-120C` DLZ bar + `SHOOT`, gun `LCOS`, 2-threat `RWR` compass), live flare burst, Vulkan PiP `WEAPON CAM` (`viper-1-aim120c-5 \| M2.09`) |
| Weapon Director + PiP | `--camera weapon --overlays all --pip` | **160 fps** | **`~0.03 ms`** | `~4.4 ms` | Live `viper-1-aim120c-5` (`M2.10`) in flight + `WEAPON CAM` PiP + DLZ `Rmin` & `LCOS` overlays |
| Fleet VFX Parity (`B-57`/`B-59`) | `--replay fleet_vfx_parity.fsrec` | **160 fps** | `0.000 ms` | `~4.3 ms` | `B-52H` 8× TF33 twin-pod contrails at 10,200 m; `UH-60` 12 m AGL hover downwash dust ring + blade-tip condensation helices + twin turboshaft heat haze |
