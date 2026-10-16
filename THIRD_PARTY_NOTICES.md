# Third-party notices

flightsim is MIT licensed (see [LICENSE](LICENSE)). It builds on and ships
with the following components; each keeps its own licence.

| Component | Use | Licence | Shipped as |
| --- | --- | --- | --- |
| [JSBSim](https://github.com/JSBSim-Team/jsbsim) | flight dynamics (`src/simulation/flight`) and the aircraft data tree (`share/flightsim/jsbsim`) | LGPL-2.1 | `libJSBSim.dll` (shared library, vendored source in `third_party/jsbsim`), with project patches generated at build time - see below |
| [VulkanSceneGraph](https://github.com/vsg-dev/VulkanSceneGraph) | viewer and vision rendering | MIT (c) 2018 Robert Osfield | statically linked into `flightsim-viewer.exe` and `libfsim_vision.dll` |
| [vsgXchange](https://github.com/vsg-dev/vsgXchange) | model/image loading, tile fetching | MIT (c) 2019 Robert Osfield | statically linked |
| [vsgImGui](https://github.com/vsg-dev/vsgImGui), [Dear ImGui](https://github.com/ocornut/imgui), [ImPlot](https://github.com/epezent/implot) | viewer UI | MIT | statically linked into `flightsim-viewer.exe` |
| [stb_image / stb_image_write](https://github.com/nothings/stb) | PNG decode (elevation tiles), PNG write (camera images) | MIT / public domain | `third_party/stb` |
| [Open Asset Import Library (assimp)](https://github.com/assimp/assimp) | glTF loading through vsgXchange | BSD-3-Clause | `libassimp-6.dll` (MSYS2 package) |
| [curl](https://curl.se/) | tile downloads in the viewer (vsgXchange) | curl licence (MIT-style) | `libcurl-4.dll` and its dependencies (OpenSSL: Apache-2.0; nghttp2, ngtcp2, nghttp3: MIT; libssh2: BSD; libpsl, libidn2, libunistring: LGPL/MIT; brotli: MIT; zstd: BSD; zlib: zlib) - MSYS2 packages |
| [glslang](https://github.com/KhronosGroup/glslang), [SPIRV-Tools](https://github.com/KhronosGroup/SPIRV-Tools) | runtime shader compilation | BSD-3-Clause / Apache-2.0 | MSYS2 packages |
| [Vulkan loader](https://github.com/KhronosGroup/Vulkan-Loader) | | Apache-2.0 | `vulkan-1.dll` from the graphics driver / MSYS2 |
| MinGW-w64 / GCC runtime (`libstdc++-6.dll`, `libgcc_s_seh-1.dll`, `libwinpthread-1.dll`) | C++ runtime | GPL with runtime exception / MIT (winpthreads) | MSYS2 UCRT64 packages |
| [World Magnetic Model 2025](https://www.ncei.noaa.gov/products/world-magnetic-model) (NOAA National Centers for Environmental Information and the British Geological Survey) | the magnetic field and declination (`src/simulation/flight_control/Magnetic.cpp`): its coefficients (`WMM.COF` of 11/13/2024) built in; its test values (`tests/data/wmm2025_test_values.txt`) test it | a work of the US Government, public domain | compiled into `libfsim.dll`; the test values not shipped |
| [aeronautic](https://github.com/Romainlg29/aeronautic) (`@aeronautic/afterburner`, `@aeronautic/wing-vapor`, `@aeronautic/core`'s atmosphere) | the engines' exhaust (`src/world/Exhaust.cpp`) and the vapour round the aircraft (`src/world/WingVapour.cpp`): their jet and flow models and their shaders, translated from TypeScript and TSL to C++ and GLSL | MIT (c) 2026 Romain Le Gall | compiled into `flightsim-viewer.exe` and `libfsim_vision.dll` |
| [three-geospatial](https://github.com/takram-design-engineering/three-geospatial) (`@takram/three-clouds`, `@takram/three-atmosphere`, `@takram/three-geospatial`) | the clouds (`src/world/Clouds.cpp`, `assets/clouds`): their shaders, ported to Vulkan compute (changes marked `fsim`), their textures (`local_weather.png`, `shape.bin`, `shape_detail.bin`, `turbulence.png`, `stbn.bin` - NVIDIA's spatiotemporal blue noise as they ship it) and the precomputed atmosphere tables they ship | MIT (c) 2024 Shota Matsuda (`assets/clouds/LICENSE-three-geospatial.txt`); their cascaded shadow maps after three-csm, MIT (c) 2019 vtHawk | `share/flightsim/clouds` |
| [pycontrails](https://github.com/contrailcirrus/pycontrails) | the contrails (`src/world/Cocip.cpp`, `src/world/Airflow.cpp`): CoCiP's formation criterion, wake vortex, initial ice and time step, and its thermodynamics, ported from Python to C++ with its default parameters; its own functions made the parity test's reference values (`tools/contrails`) | Apache-2.0, (c) Contrails.org and the Breakthrough Energy Foundation (`third_party/licenses/LICENSE-pycontrails.txt`, `NOTICE-pycontrails.txt`) | compiled into `flightsim-viewer.exe` |
| [Precomputed Atmospheric Scattering](https://github.com/ebruneton/precomputed_atmospheric_scattering) (Eric Bruneton, as three-geospatial carries it) | the clouds' light (`assets/clouds/shaders/bruneton`) | BSD-3-Clause (c) 2017 Eric Bruneton, (c) 2008 INRIA - the notice is in each file | `share/flightsim/clouds` |
| [GodotOceanWaves](https://github.com/2Retr0/GodotOceanWaves) | the sea's waves (`assets/clouds/shaders/ocean`: `spectrum`, `modulate`, `fft`, `transpose`, `unpack`): its TMA spectrum with Hasselmann spreading, its Stockham FFT and transpose, its Jacobian foam, ported to one dispatch per pass for all cascades (changes marked `fsim`) | MIT (c) 2024 Ethan Truong (`assets/clouds/shaders/ocean/LICENSE-GodotOceanWaves.txt`) | `share/flightsim/clouds` |
| [Proland](https://proland.inrialpes.fr/) (Bruneton, Neyret & Holzschuch, "Real-time Realistic Ocean Lighting using Seamless Transitions from Geometry to BRDF", 2010) | the sea's light (`assets/clouds/shaders/ocean/ocean.frag`): its slope-statistics BRDF and averaged Fresnel, written from the paper | BSD-3-Clause (c) 2008-2015 INRIA - LJK (`assets/clouds/shaders/ocean/LICENSE-Proland.txt`) | `share/flightsim/clouds` |
| [NVIDIA Flow 2](https://github.com/NVIDIA-Omniverse/PhysX/tree/main/flow) (2.2.0, commit 6dc5358) | an engagement's fire and smoke solved as a fluid in the viewer (`src/world/FlowVolumes.cpp`, MS-7h): its combustion solver run through its C interface on its own Vulkan device, its NanoVDB fields read back and drawn by flightsim's shaders (`assets/clouds/shaders/flow.*`); PNanoVDB.h (its copy) reads them | BSD-3-Clause, (c) 2014-2025 NVIDIA Corporation & Affiliates | its headers and `nvflow.dll`, `nvflowext.dll` (built by `tools/flow/build-flow.sh`, unchanged) in `third_party/flow`, with its licence; the two libraries beside the viewer (`bin/`) |
| [Skybolt](https://github.com/Prograda/Skybolt) | how the clouds read a global coverage map from far off (`assets/clouds/shaders/fsim/coverage.glsl`): its method, nothing copied | MPL-2.0 | - |
| [NTIA P.528](https://github.com/NTIA/p528) and [NTIA ITM](https://github.com/NTIA/itm) (the U.S. reference implementations of ITU-R P.528-5 with P.676 and P.835, and of the Irregular Terrain Model) | the radio path (`src/simulation/mission_systems/rf`, ADR-30): air-to-air and air-to-ground loss, gases, terrain | works of the US Government, public domain in the US; NTIA's grant elsewhere (`third_party/ntia/*/LICENSE.md`); modified as `third_party/ntia/README.md` states | compiled into `libfsim.dll` |
| ITU-R Recommendations P.838-3 (rain), P.526-15 (diffraction), P.839 (rain height) | the radio path's rain and terrain diffraction (`src/simulation/mission_systems/rf/Rain.cpp`, `Path.cpp`): equations and coefficients, written from the Recommendations | ITU | compiled into `libfsim.dll` |
| [Stone Soup](https://github.com/dstl/Stone-Soup) | the trackers (`src/simulation/mission_systems/track`, ADR-30 MS-5): its motion and measurement models, Kalman filters, hypothesisers, associators, initiators, deleters and trackers, ported from Python to C++; Stone Soup itself made the cross-check's reference (`tools/track`) | MIT, (c) Crown Copyright 2017-2026 Dstl, (c) Crown Copyright 2018-2026 DRDC, (c) 2018-2026 University of Liverpool (`third_party/licenses/LICENSE-stonesoup.txt`) | compiled into `libfsim.dll` |
| [SciPy](https://github.com/scipy/scipy) (`linear_sum_assignment`: Crouse's shortest augmenting path) | the trackers' 2D assignment (`src/simulation/mission_systems/track/Assignment.cpp`), ported from C++ step for step | BSD-3-Clause, (c) 2001-2002 Enthought, Inc., 2003 SciPy Developers (`third_party/licenses/LICENSE-scipy.txt`) | compiled into `libfsim.dll` |
| [FERS](https://github.com/davidbits/FERS) (the Flexible Extensible Radar Simulator) | the signal-level radar's cross-check (`tools/fers`, ADR-30 MS-6): built from its source beside the platform and run as a separate program; its output, put through a receiver's processing, is the test's reference (`tests/data/signal/fers_crosscheck.json`). Nothing of it is linked or copied | GPL-2.0 | not shipped |
| [Catch2](https://github.com/catchorg/Catch2) | tests only | BSL-1.0 | not shipped |
| Cesium Air (`Cesium_Air.glb`) | sample aircraft model | Apache-2.0, (c) CesiumGS, Inc. and Contributors | downloaded at configure time (`assets/models/NOTICE.md`) |

## Data services

The viewer and the terrain physics fetch public tiles at run time and cache
them locally; using them is subject to the providers' terms:

- **Esri World Imagery** (`server.arcgisonline.com`): Esri, Maxar, Earthstar
  Geographics, and the GIS User Community. Attribution is required when
  imagery is shown or published.
- **AWS Terrain Tiles** (`s3.amazonaws.com/elevation-tiles-prod`, Terrarium
  encoding): Mapzen / Amazon Open Data; sources include USGS 3DEP, SRTM,
  GMTED2010, ETOPO1 and others - see the AWS Open Data registry entry.
- **OpenStreetMap** tiles (optional `--imagery osm`): (c) OpenStreetMap
  contributors, ODbL; the tile usage policy applies.
- **NASA Blue Marble: Clouds** (`eoimages.gsfc.nasa.gov`, `cloud_combined_8192.tif`, made into the map set's
  `clouds/earth-clouds-8192.png` by `tools/cloud_map/make_cloud_map.py`): NASA Earth Observatory / NASA Goddard
  Space Flight Center; a work of the US Government, public domain. The viewer's cloud cover over the globe.
- **GEBCO 2026 Grid** (`gebco.net`, `GEBCO_2026.nc` and `GEBCO_2026_TID.nc`, made into the map set's
  `ocean/gebco-ocean-2arcmin.png` by `tools/ocean_map/make_ocean_map.py`): GEBCO Compilation Group (2026) GEBCO 2026
  Grid; public domain, attribution requested. The sea's extent and depth.
- **OpenStreetMap land polygons** (`osmdata.openstreetmap.de`, `land-polygons-split-4326`, burnt into the map set's
  Terrarium elevation tiles by `tools/ocean_map/sea_mask_tiles.py`: which texels at or below sea level are land and
  which are sea): (c) OpenStreetMap contributors, ODbL. The sea's coastline.

## Changes to JSBSim

flightsim builds JSBSim 1.3.1 from the tracked vendored source in `third_party/jsbsim`, with
the following changes generated into the build tree by `cmake/JsbsimPatches.cmake`:

- In `src/models/FGLGear.cpp`, the projection of a wheel's strut on the ground normal is
  bounded at 45 degrees. Upstream divides the wheel's compression and ground force by that
  projection, so a wheel meeting the ground sideways, as in a cartwheel, got an unbounded force.
- In `src/models/propulsion/FGRotor.h` and `FGRotor.cpp` (the helicopter rotor,
  docs/rotorcraft.md): a reset restarts the rotor, which kept the last run's speed and
  inflow; a rotor may choose the classical force model (`<model>classical</model>`) - the
  in-plane forces and the torque of NASA TM-73254 (Talbot and Corliss, 1977), equations 1
  to 4 - instead of Heffley's, which stays the default - and with it the ground effect scales
  the steady inflow rather than the lagged one; and a start or a reset converges every
  rotor's inflow and its ground-height filter before the first step (upstream's begin from
  nothing and settle over the first seconds). Past that start, a rotor that does not ask for
  the classical model flies as upstream's does.
- In `src/models/propulsion/FGElectric.cpp`, an electric engine whose file gives a specific
  fuel consumption (`<bsfc>`, as a piston engine's does) burns that times its power from the
  tanks feeding it, and gives no power once they are empty (hangar's helicopters fly their
  turboshafts as such a governed power source). An electric engine without one burns
  nothing and runs as upstream's does.
- In `src/models/FGAccelerations.cpp`, an impact the step cannot resolve is made inelastic.
  The ground contacts' springs and dampers act for a whole step from the state it begins
  with, so an aircraft striking the ground faster than its contacts can take in a step
  (docs/rotorcraft.md: a 27 g quadrotor at 35 m/s) was sent back up faster than it came
  down. When the ground's forces would send a contact point back out more than 5 m/s faster
  than it came in, the step applies instead the forces that stop the contact points'
  approach, each no more than its contact's own, solved as JSBSim solves its friction, and
  bounds the friction by them. Otherwise the ground's forces are upstream's, bit for bit.
- In `src/models/FGAuxiliary.cpp`, the angle of attack's rate fades beyond 60 degrees of sideslip with the share of the
  flow in the aircraft's plane of symmetry, to nothing in a flow from the side. Upstream's grows without bound as that
  share falls to nothing: an aircraft stopped on a runway in a crosswind read 38 rad/s and was thrown off it by its rate
  terms (docs/flight-autonomy.md, 4.54). Within 60 degrees it is upstream's, bit for bit.
- A reset (`ResetToInitialConditions`) returns what a load leaves, where upstream's kept
  what the last run left: in `src/models/flight_control/FGActuator.cpp`, an actuator's first
  run after a reset passes its input through, as a new actuator's does; in
  `src/models/FGAerodynamics.cpp`, a reset has no lift from the last run (the next run's
  `aero/cl-squared`) and keeps the stall hysteresis's limits and the aerodynamic reference
  point's shift the aircraft loaded (upstream's cleared both for good); in
  `src/models/FGMassBalance.cpp`, the centre of gravity starts where a new model's does; in
  `src/models/propulsion/FGPropeller.cpp`, a constant-speed propeller's pitch starts at its
  least, where a load sets it; in `src/models/propulsion/FGTransmission.h`, a rotor's
  free-wheel unit starts locked, its lag at rest. A
  vehicle started on a reset model then starts as the same start on a new one, to the bit
  (docs/FlightSim_System_Architecture_and_Design.md, 7.2). Between resets nothing changes.
- In `src/models/propulsion/FGTurbine.cpp`, a staged afterburner (augmethod 2) lights only
  with the core at military speed (the 97 % upstream's augmethod 1 gates on) and is lit as far
  as its fuel has risen, at the engine's augmented fuel-flow rates, from the dry engine's flow
  toward full reheat's; its thrust adds that share of the reheat's, and its nozzle opens with
  it. Upstream added the whole reheat the step the lever passed military power, whatever the
  core was doing. An engine whose lever stays at or below military power is unchanged.

`cmake/JsbsimPatches.cmake` makes the changes: it writes the changed files into the build
tree, with each edit marked "flightsim patch" (two files the rotor's header reaches,
`FGEngine.cpp` and `FGTurboProp.cpp`, are compiled from there unchanged). Packages carry the
changed files and that script in `share/doc/flightsim/jsbsim-changes/`.
