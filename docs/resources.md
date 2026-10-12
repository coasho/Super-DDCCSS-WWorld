# Resource ownership and regeneration

This page identifies where resource facts are maintained, how derived files are made, and which copies the runtime reads. It is a maintenance map, not a capability summary; current implementation and milestone status belong in [status.md](status.md). Build and environment rules are in [development.md](development.md).

The 2026-10-11 documentation snapshot contains **35 aircraft designs and 53 store designs**. These are dated inventory observations, not permanent constants or acceptance thresholds. Recount from the current TOMLs/cards and tool inventory when reporting a later corpus. A file's presence does not establish successful generation, flight behavior, stability, or visual quality.

## Ownership table

| Resource | Editable source / owner | Generator or copy step | Output and runtime consumer |
| --- | --- | --- | --- |
| Aircraft geometry, mass, engine and physical applicability | Hand-maintained `aircraft/<name>/<name>.toml`, with cited facts and labeled assumptions | `tools/hangar/hangar/pipeline.py`; rotorcraft dispatch in `hangar/rotorcraft/` | JSBSim XML, engine files, model and stage evidence; simulation and viewer |
| Aircraft appearance | `aircraft/<name>/paint.toml` and design geometry | Hangar model stage | `<name>.glb` and `<name>.glb.manifest`; viewer and RCS geometry |
| Payload sensor facts and fits | `assets/payloads/*.json`, aircraft design's payload/station/loadout declarations | Hangar build and fit writer | `aircraft/<name>/payloads.json`; runtime equipment and loadouts |
| Store facts | `assets/stores/<id>.json`, with sources / basis per parameter | Missile or store design writer; store stages | Store designs, flight models and cards consumed by mission systems |
| Missile shape and assumed motor/layout details | `hangar/missiles.py` table plus store card facts | `python -m hangar missiles [<name> ...]` | Generated `aircraft/<name>/<name>.toml` and `paint.toml`; input to subsequent stages |
| Tank, pod, bomb and cargo shape | `hangar/storedesigns.py`, `hangar/stores.py` tables and cards | `python -m hangar storedesigns [<name> ...]` | Generated store TOML / paint; store stages read these |
| Released store flight physics | Store card and generated design; store build-up modules | `hangar/storestages.py`, `storemodels.py`, `storedrag.py` | `aircraft/stores/<name>/<name>.xml` and missile `Engines/`; carriage drag fields in cards |
| Stock-aircraft visual stand-ins | Aircraft `stands_in_for` declarations and generated geometry/CG | `python -m hangar register` | `aircraft/models.txt`; viewer stand-in mapping, not replacement stock flight physics |
| Flight/control profile | Design, calibration and stage observations | Hangar fly, autopilot, performance and profile writers | `autopilot.toml`, rotorcraft `hover.toml`, profile properties and performance/fuel tables embedded in XML |
| Damage and breakup geometry | Design mass/structure, geometry and vulnerability inputs | Hangar model stage, `damage.py` and `vulnerability.py` | `<name>.damage.json`, GLB sections and manifest components; world's damage chain and viewer |
| Radar cross section | Generated closed geometry; `aircraft/<name>/signature.json` and `assets/signature/treatments.json` | `tools/rcs` / `flightsim-rcs` | `aircraft/<name>/rcs.bin`; mission-system RCS lookup |
| Stage checks and aircraft report | The design and actual stage run | Hangar stages and HTML report writer | `aircraft/<name>/out/<stage>.json`, images, `report.html`; review evidence, not shipped design inputs |
| Store build report | Actual store generation run | `hangar stores --record docs/hangar-stores-build.md` | [hangar-stores-build.md](hangar-stores-build.md) and JSON companion; generated dated evidence |
| SDK principal inventory | [sdk/boundary.toml](sdk/boundary.toml) | `tools/boundary/boundary.py render`, against DLL exports and Python package | [sdk/boundary.md](sdk/boundary.md); generated inventory, checked by `sdk_boundary` |
| Cloud renderer assets | `assets/clouds/` shaders, textures, lookup tables and notices | `fsim_cloud_assets` build copy | `build/<preset>/share/flightsim/clouds`; renderer |
| Global cloud coverage | NASA Blue Marble cloud TIFF, optionally supplied locally | `tools/cloud_map/make_cloud_map.py` | `assets/maps/clouds/earth-clouds-8192.png`; viewer weather coverage |
| Imagery and terrain | `assets/config/offline-map-plan.json`, remote map providers and local tiles | `tile_prefetch` via `fetch-maps.cmd` | `assets/maps/`, installed viewer `maps/`; terrain, satellite imagery and offline package |
| Ocean depth and coastline | GEBCO grids and OpenStreetMap land polygons | `tools/ocean_map/make_ocean_map.py`, `sea_mask_tiles.py` | Ocean raster and sea-masked elevation tiles; world renderer/terrain |
| Stock JSBSim aircraft, engines and systems | Tracked vendored source and data in `third_party/jsbsim/` | Build/package copies; platform patches are owned by `cmake/JsbsimPatches.cmake` | Stock flight-model data; keep distinct from Hangar designs |
| Build/package configuration | `assets/config/viewer.json` and packaging code | `cmake/PackageConfig.cmake` for installed viewer settings | `config/viewer.json`; packaged settings differ deliberately from development settings |

## Edit inputs, then derive outputs

An ordinary aircraft TOML is authored directly. Its XML, GLB, manifest, generated control tuning, profile and report follow the design and the tool. Do not hand-edit generated output to fix a generator defect, or use a timestamped report as the source for model parameters.

Missile and other store TOMLs have an additional generation layer. Their headers name the writer. Published length, diameter/span and mass come from the JSON card; assumptions about shape, CG, fins and motor/layout come from the generator tables. Improve the card or the relevant table, then rewrite the TOML before running its stages. Editing only generated TOML loses the correction at the next design-generation run. The missile command and non-missile store command are distinct; the `stores` command processes both kinds through their appropriate stages.

Autopilot and performance files are derived aircraft data, not global controller presets. A change to aerodynamics, mass, propulsion, controls or calibration can make them stale; use the stage order documented in [hangar.md](hangar.md). Rotorcraft have their own stage dispatch and hover identification. Preserve generated XML dates when content is unchanged and compare GLB bytes for unexplained churn. Keep source citations, explicitly assumed values and generator provenance visible.

The following are **regeneration examples**, not commands to run for a documentation-only change. They assume the selected build already contains the Python SDK and dependencies. In PowerShell, use the build explicitly:

```powershell
$env:PYTHONPATH = "$PWD/tools/hangar;$PWD/build/ucrt64-headless/python"
$env:FSIM_BIN = "$PWD/build/ucrt64-headless/bin"
& build/ucrt64-headless/python/fsim-python.cmd -m hangar list
& build/ucrt64-headless/python/fsim-python.cmd -m hangar missiles aim120c
& build/ucrt64-headless/python/fsim-python.cmd -m hangar storedesigns tank370gal
& build/ucrt64-headless/python/fsim-python.cmd -m hangar aim120c
& build/ucrt64-headless/python/fsim-python.cmd -m hangar register
& build/ucrt64-headless/python/fsim-python.cmd -m hangar stores --jobs 1 --record docs/hangar-stores-build.md
& build/ucrt64-headless/python/fsim-python.cmd -m hangar effects
```

For a full aircraft campaign, `python tools/hangar/rebuild_all.py --jobs 6` runs each design in its own process and retains raw per-design logs in `build/hangar-rebuild`; `--only f16c c172` narrows the corpus. It is a potentially long regeneration, not a documentation validation step. For a long run, Hangar's native libraries stay loaded: point PYTHONPATH and FSIM_BIN at a deliberate copy of the selected build's Python package and bin directory if rebuilding that build concurrently. Record the copy's provenance.

The store report's `--force` option forces model and flight-model regeneration; use it only when that evidence is intended. The report rows and JSON companion belong to the tool. Update current status separately after reviewing the run, failures and generated diffs.

The generated effects inventory reports missing entries under the generator's current audit rules. Rotorcraft
exhaust/wake omissions are excluded from that count and remain B-59 in [backlog.md](backlog.md). A zero count does
not establish complete physical effect coverage. Showcase and plume-check recordings are scripted visual fixtures,
not native guidance or trajectory validation.

For the SDK inventory, edit TOML and render against the actual public binaries and staged Python package:

```powershell
& build/ucrt64-release/python/fsim-python.cmd tools/boundary/boundary.py render --fsim-dll build/ucrt64-release/bin/libfsim.dll --vision-dll build/ucrt64-release/bin/libfsim_vision.dll --objdump D:/ENV/DevLanguages/Cpp/msys2/ucrt64/bin/objdump.exe --cxxfilt D:/ENV/DevLanguages/Cpp/msys2/ucrt64/bin/c++filt.exe --python-path build/ucrt64-release/python
ctest --preset ucrt64-release -R sdk_boundary
```

Omit `--vision-dll` for a headless build and select that build's paths consistently. `check` uses the same arguments and verifies the rendered inventory. It imports the Python surface and reads exports, so a stale DLL is not evidence for a new API.

RCS generation is separate from Hangar and needs rendering dependencies plus a Vulkan ray-query device. Its executable accepts aircraft types, for example `build/ucrt64-release/bin/flightsim-rcs.exe f16c`; `--out <file>` selects output. Read [mission-systems.md](mission-systems.md) and the tool's settings before rebaking a corpus. A geometry or signature-treatment change can invalidate existing RCS tables; table presence alone does not establish current geometry provenance.

## Maps and cloud data

`fetch-maps.cmd --dry-run` inspects the configured tile plan; `fetch-maps.cmd` fetches to `assets/maps`. `--prune` is destructive: it removes tiles outside the plan. These commands need an already built `tile_prefetch.exe`. The wrapper can also generate the global cloud raster using whichever `python` is on PATH and Pillow, so ensure that is the intended environment.

`python tools/cloud_map/make_cloud_map.py --tif <local-cloud-tiff>` uses an existing TIFF; without `--tif` it downloads the NASA source when absent. Ocean generation accepts the two GEBCO grids and an output path as documented in `fetch-maps.cmd`. GEBCO and coastline source downloads are separate from the tile fetch. Keep environments and caches in `D:\ENV`; project map datasets live under `assets/maps` by design. Large map datasets are not committed. Preserve asset licenses and [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md) when redistributing.

## Staging and runtime search

Build staging and installation copy products; they do not run Hangar. [cmake/Assets.cmake](../cmake/Assets.cmake) stages model assets at configure time and copies clouds/cards on builds. [cmake/StageDesigns.cmake](../cmake/StageDesigns.cmake) copies aircraft XML, Engines, GLB/manifest, payload fit, RCS tables, released-store flight models and `models.txt`. It leaves design TOMLs, paint, signature inputs and `out/` behind. The existing copy list also **omits `<name>.damage.json`**: source-tree damage availability does not prove equivalent behavior in staged or installed layouts.

[cmake/Install.cmake](../cmake/Install.cmake) installs these data for viewer/tools; [python/CMakeLists.txt](../python/CMakeLists.txt) stages stock JSBSim data, designs and cards inside the Python package. The SDK component currently installs no such runtime data. Explicitly provision and test data for a standalone SDK consumer rather than assuming the exported CMake package includes it. The packaged viewer copies `assets/maps` to `maps/`; its packaged configuration is intended for offline tiles. A stale or incomplete package must be tested on its own, away from source-tree fallbacks.

Runtime ordering is defined in [src/io/AssetResolver.cpp](../src/io/AssetResolver.cpp) and the viewer:

1. Added asset search paths are prepended. The default asset roots are relative to the loaded module first (`../share/flightsim`, then `share`), then equivalent executable roots when the host differs, then the compiled development fallback.
2. Aircraft directories start with semicolon-separated `FSIM_AIRCRAFT_PATH`, then the compiled source `aircraft/` when it exists, then aircraft folders under asset roots. The source-first development ordering deliberately avoids a stale Python package hiding a newly rebuilt design.
3. Flight-model lookup lets an explicit `FSIM_AIRCRAFT_PATH` shadow stock models. Released-store models under `aircraft/stores` are checked before stock aircraft; otherwise the selected stock JSBSim root wins over a same-named ordinary design. Keep design names distinct from stock names when retaining them as references.
4. Viewer model lookup honors an explicit model path, then stand-in mappings and the configured model directories. Asset `models/` directories are added before aircraft design directories; stale `models/<type>.glb` can therefore shadow a design's model. The configure step prunes removed source model assets from staging, and Python model staging refreshes its copy.

Inspect the resolved model path in logs, not just the file you edited. Check both the development and isolated installed layout when validating packaging. Do not edit a staged or installed copy as a permanent fix; change its owner and repeat the copy/package step.
