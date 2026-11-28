# VIS-M14: 3-D Volumetric Tactical & EMSO Overlays, Fleet VFX Parity & High-DPI UI

**Delivered:** 2026-11-27 (`main`, base `c331fb70`, visual overhaul follow-up on `6ce733b1`)
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

## 2. True 3-D Vulkan Volumetric Overlays (`TacticalVolumes`) & Clean Controls

- **Depth-Tested 3-D Vulkan Volumetric Renderer (`src/visualization/vfx/effects/TacticalVolumes.{h,cpp}`, `assets/clouds/shaders/tactical_volume.{vert,frag}`):**
  - Eye-relative floating-origin SSBO geometry drawn in `overPass` with standard alpha blending (`SRC_ALPHA, ONE_MINUS_SRC_ALPHA`) for crisp, zero-whiteout contrast even over bright sunlit clouds, soft reverse-Z `sceneDepthTex` terrain/airframe occlusion, and terrain-intersection contour glow.
  - **Distinct Force & Layer Visual Coding:**
    - **Radar Sectors (Blue vs. Red Force):** Rich sapphire-blue (friendly) vs. crimson-red (hostile) curved holographic sector decks with anti-aliased 5/10 km concentric range rings, live bright sweeping scan beams, and sculpted dual-zone burn-through pockets (clear inner dome + razor-sharp luminous gold burn-through boundary arc vs. dimmed diagonal-static suppressed outer zone).
    - **EA Jamming Cones:** Distinct royal-purple translucent front-weighted 3-D cones with animated fiery amber-orange RF interference wavefront rings racing outward (amber-gold palette for comms jamming).
    - **ESM / RWR & 3-D CRLB Ellipsoids:** Tapered warm-gold directional bearing wedges with animated outward chevrons, plus true 3-D translucent geodesic CRLB uncertainty ellipsoids with equatorial/meridian contours when triangulated.
    - **5-D Weapon DLZ/WEZ & Gun/Bomb Cues:** Color-zoned 3-D tactical fan decks (emerald NEZ kill zone, amber `R_max`, crimson `R_min` collar) plus sleek volumetric LCOS/CCIP lead lines and rings for the selected aircraft.
    - **Link 16 / C2 / IFF / PNT & SAR / Weather:** Elevated parabolic energy arcs with animated traveling data packets, Mode 5 IFF ripple rings, PNT GNSS drift rings, and ground-hugging SAR scanline / NEXRAD `dBZ` footprints.
- **Anti-Overlap Glassmorphic Micro-Pills (`src/visualization/ui/TacticalOverlays.{h,cpp}`):**
  - Renders compact dark translucent pills with thin colored accent bars and screen-space collision avoidance that reserves the top-left `MonitorGui`, bottom-left `vehicles` list, and aircraft callsigns so annotations never overlap panels or each other.
- **Clean Camera & Input Discipline (`CameraController`, `KeyHandler`, `MonitorGui`):**
  - Preserves the clean original 4 camera modes (`chase`, `orbit`, `overview`, `free` via `[C]` / `--camera`) and original minimal hotkeys (`[Space]`, `[.]`, `[Tab]`, `[C]`, `[-/+]`, `[R]`, `[Esc]`) without extra camera modes, PiP clutter, or extraneous hotkeys.
  - Overlays toggle cleanly via `--overlays all|none|radar,jammer,esm,links,weapons,swaths` or color-swatched checkboxes in `MonitorGui` (**`0.000 ms`** CPU & GPU when OFF).

---

## 3. Fleet VFX Parity (`B-57`, `B-59`), High-DPI `MonitorGui` (`B-43`) & Offline Tile Pruning

- **`B-57` (8-Engine Propulsion & Per-Engine `exhaust_k`):**
  - `Exhaust::kPerSlot` expanded to `8`; `VehicleVisuals` maps `b52h.glb`'s 8 twin-pod nacelles to logical engines `0..7` driven by `TacticalPropulsionExtWireV1` for both plumes and 8-engine `Airflow` contrails.
  - Manifest `exhaust` entries accept an optional 8th `<exhaust_k>` turbine exit temperature (`Exhaust::reheatTotalK`), authored in `hangar/model3d.py` and manifests for `f22a` (`1280 K`), `mirage2000` (`1180 K`), and `b52h` (`950 K`).
- **`B-59` (Rotorcraft & Propeller VFX):**
  - `hangar/rotorcraft/model.py` and manifests (`uh1h`, `uh60`, `c130j`) emit `exhaust` and `rotor` lines.
  - `Exhaust` / `VehicleVisuals` synthesize `exhaustStack` thermal plumes for placed exhaust entries without nozzle meshes so rotorcraft and turboprops drive `WingVapour` heat-haze distortion.
  - `GpuParticles::addRotorEffects()` emits radial ground-effect downwash dust/sea-spray rings when hovering near ground (`AGL < 2.8 × R_rotor`) and spinning blade-tip / propeller-tip condensation helices in humid air.
- **`B-43` (High-DPI Responsive `MonitorGui`):**
  - `MonitorGui` folds sections under `CollapsingHeader`s (`Camera [C]`, `Tactical Overlays`, `Selected Vehicle [Tab]`, `World & Weather`), adapts layout compactness to `DisplaySize.y / FontGlobalScale`, and constrains window height so it fits `1600×900 @ 150% DPI` cleanly without clipping or overlapping the bottom-left `vehicles` table.
- **Offline Tile Leaf Pruning (`OfflineTiles.{h,cpp}`, `ElevatedTile.{h,cpp}`):**
  - Added `OfflineTiles::hasAnyChild(z, x, y)` and `ElevatedTile::pruneOfflineLeaves()` so offline `PagedLOD` nodes whose `z+1` subtiles do not exist in the local cache collapse into plain `vsg::CullGroup`s, eliminating `vsg::tile::read_subtile(..) could not load any subtiles.` log warnings (`0` warnings across all viewer runs).

---

## 4. Verification Evidence (2026-11-27)

### Dual-Preset Builds & Automated CTest Suites (`0` `-Werror` warnings, `glslangValidator` clean)
- **`ucrt64-release`:**
  - `fsim_render_tests "~[gpu]"`: **67/67 cases PASSED (5,346 assertions)** (includes 4 VIS-M14 cases covering `B-57`, `B-59`, offline leaf pruning, `TacticalVolumes::inferSide`, and 4-mode `CameraController`).
  - CTest suites: **8/8 PASSED** (`vis_m14_ui`, `mandatory_presentation`, `tactical_overlays`, `sdk_boundary`, `include_view_{scene,ui,render,viewer}`) plus `contracts`, `combat_vfx`, `destruction_debris`; `architecture_boundary` has 0 VIS-M14 violations.
- **`ucrt64-headless`:**
  - CTest targeted & ABI suites: **8/8 PASSED** (`contracts`, `simulation_api_link`, `training_link`, `world_adapter_compatibility`, `c_abi`, `combat_vfx`, `tactical_overlays`, `sdk_boundary`).

### Native 4K Fullscreen Visual Acceptance (`3840×2160`, 1× real time, RTX 5070 Ti, `WinSta0\Default` via `desktop-gui` MCP)
All runs executed fullscreen at primary `3840×2160` with full Earth imagery, terrain, clouds, ocean, shadows, and effects enabled, producing **`0` `read_subtile` warnings**:

| Scenario | Flags | Steady 4K FPS | Overlay CPU + 3-D GPU | Total 4K GPU | Visual Checks Verified |
| --- | --- | --- | --- | --- | --- |
| Baseline OFF | `--replay emso_tactical_showcase.fsrec` | **159.9–160.1 fps** (`p50 6.24 ms`) | **`0.000 ms` + `0.000 ms`** | `4.22–4.64 ms` | Exact zero overhead OFF, zero `read_subtile` warnings |
| Daylight 3-D EMSO ALL | `--sun-utc 19 --camera orbit --chase-distance 26000 --chase-elevation 36 --overlays all` | **159.8–161.5 fps** (`frame avg 6.19–6.26 ms`) | **`0.012 ms` CPU + `0.183–0.188 ms` GPU** (`0.20 ms` total) | `5.42–5.48 ms` | Sapphire-Blue vs. Crimson-Red 3-D radar sectors + live sweep beams + crisp gold burn-through arcs (`BT 6.9 km` / `BT 13.6 km`), royal-purple/amber animated EA jamming wavefront cone, tapered gold ESM wedges, live AIM-120C + flares, zero-overlap dark glass pills, clean 4-mode UI |
| Fleet VFX Parity (`B-57`/`B-59`) | `--replay fleet_vfx_parity.fsrec` | **160 fps** | `0.000 ms` | `~4.3 ms` | `B-52H` 8× TF33 twin-pod contrails at 10,200 m; `UH-60` 12 m AGL hover downwash dust ring + blade-tip condensation helices + twin turboshaft heat haze |
