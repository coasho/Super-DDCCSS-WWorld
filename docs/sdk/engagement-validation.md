# Missile Engagement Delivery Validation

Date: 2026-10-11. Local SDK history is preserved from starting HEAD `02d1683`. The game uses the staged SDK at `build/ucrt64-headless/python`; it contains no alternate missile simulation.

## Build and regression evidence

The Release headless build uses UCRT64 GCC 16.2, CMake/Ninja and CPython 3.13 on an AMD Ryzen 7 9700X (8 cores, 16 logical processors). Temporary files and dependency caches are under `D:/ENV`. `TMP`, `TEMP` and `TORCH_EXTENSIONS_DIR` are set to `D:/ENV/tmp` before compilation. Dependencies are tracked vendored sources, with their original licenses and upstream references retained.

| Check | Verified result |
| --- | --- |
| `cmake --build --preset ucrt64-headless -j10` | Successful final build and package staging |
| `ctest --preset ucrt64-headless` | Initial 572-test run passed every test except `control_alloc`; all 40 Hangar model tests passed |
| Allocation repair | Replaced a per-step snapshot allocation with a reusable World buffer; `fsim_control_bench alloc` reports zero platform allocations, with JSBSim allocations counted separately |
| `ctest --preset ucrt64-headless -LE hangar` | All 532 tests passed on the repaired final build, including allocation, fleet, native engagement, radar, C ABI and Python coverage; elapsed 308.91 s |
| Model regression | The 40 passing Hangar tests remain applicable: the subsequent repair changed snapshot storage, with no model geometry or mesher changes |
| English metadata staging | Rebuilt the staged package after translating three existing store cards' notes and source titles; physical parameters are unchanged |
| `ctest --preset ucrt64-headless -R '^python_sdk$'` | Complete Python SDK suite passed again after metadata staging; elapsed 81.64 s |
| SDK boundary | 1717 exported entries classified, with zero boundary errors |
| Source audit | Maintained first-party SDK and game code, documentation and UI use English; `git diff --check` clean |

Together, the repaired functional suite and unchanged model suite cover all 572 tests. This is not a claim that the initial full run had no failures. The allocation failure was reproduced, repaired and verified before delivery.

## Native encounter evidence

The seven `engagement_native` cases exercise actual JSBSim releases rather than a replacement kinematic integrator.

- A 5 km head-on encounter detonates with matching event sequences for one and two workers. Both report miss distance `3.39672e-05 m` and maximum speed `704.34 m/s`.
- A supported 30 km encounter activates, acquires, burns out and coasts to detonation. Miss distance is `8.94916 m`, maximum speed `788.603 m/s`. Native thrust is zero after burnout even when engine fuel-flow bookkeeping remains nonzero.
- A raw target entity cannot create midcourse sensor updates.
- A two-victim detonation safely appends native debris while assessing both victims.
- Infrared release retains the eligible on-rail contact rather than a nearer off-axis contact.
- A coarse-step swept encounter uses interpolated victim and terminal weapon positions for damage.
- Inert jettison never arms or emits motor thrust.

Encounter diagnostics were unchanged by the allocation repair. Guidance and countermeasure module tests additionally cover stale observations, illumination, decoy resistance, arming, bounded acceleration, inventory, interval, expiry and invalid-command refusal. Public-interface tests cover legacy sized result buffers, replay cursors, agent access restrictions and decoys outliving their carrier.

## Game integration evidence

The checks in this section were recorded against an external browser-game client. Its adapter tests and
`tools/server_smoke.py`, `tools/all_missions.py` and `tools/client_smoke.mjs` are not files in this SDK repository.
The original record did not identify the client checkout, commit or working directory, so these commands are
retained as historical evidence rather than advertised as reproducible commands here. Reverification requires
identifying that client revision first. This section does not validate the retired testbench tactical prototype
or the optional native Vulkan viewer.

| Check | Verified result |
| --- | --- |
| `python -m unittest discover -s tests -v` | Five adapter regressions passed: refusal, stale cues, lost stores, designation reuse and damage without automatic kills |
| `python tools/server_smoke.py` | HTTP and WebSocket checks passed against the final SDK; 51 snapshots at approximately 17/s, 1054 bytes per snapshot, 1.01 simulated seconds per wall second; x4 measured 3.99; pause holds time |
| `python tools/all_missions.py 180` | Ten mission/aircraft/skill combinations completed to 180 seconds or their native outcome, with releases and damage events; native player losses are valid outcomes |
| Recorded 180-second encounter and `node tools/client_smoke.mjs` | 2700 snapshots, 5400 rendered frames, 1875547 canvas calls, six loaded models and no JavaScript exception; three launches, one active-seeker event and three terminal events |
| Actual browser inspection | English menu/HUD, radar designation, guided release, chaff/flare, pause and native damage exercised; no browser console warnings or errors |
| Ownership audit | Missile adapters serialize SDK state; flight, seekers, launch-zone estimates, decoy response and damage remain in the SDK; receiver warnings use measured SDK contacts |

Browser evidence: `D:/ENV/tmp/bvr-native-engagement-browser.jpg`. The observed burst damaged an aircraft that continued to fly; the client displayed hits without awarding a fabricated kill. Terminal expiry/removal produces no detonation effect.

## Reproduction and limits

Raw logs are kept under `D:/ENV/tmp`: `bvr-sdk-delivery-build.log`, `bvr-sdk-delivery-tests.log`, `bvr-sdk-final-current.log`, `bvr-control-alloc-current.log`, `bvr-native-snapshot-green.log`, `bvr-sdk-metadata-build.log`, `bvr-sdk-metadata-python.log`, `bvr-adapter-delivery.log`, `bvr-server-delivery.log`, `bvr-missions-current.log` and `bvr-client-current.log`.

This verifies the native headless SDK and the actual browser client. The optional native Vulkan viewer was not rebuilt for this milestone. There is no comparable pre-migration game timing sample, so no performance improvement is claimed. Native engine/property diagnostics and SDK command refusals may appear in scenario logs; they are separate from browser console errors or test failures.

Guidance, fin response, sensor resistance, dispenser motion and launch-zone parameters are game assumptions, not calibrated real-world performance. See the [public engagement contract](engagement.md) for units, retention, access rules and modeling limits. Launch-zone envelopes are advisory and do not guarantee an intercept.
