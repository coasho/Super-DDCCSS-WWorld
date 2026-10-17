# ARCH-03 foundation and domain leaves

Status: relocation implemented at `30bc5ef`. The [independent review](arch-03-review-2026-10-16.md) found no behavioral
regression but held ARCH-03 incomplete without task 5 (HC-12). Task 5, include-path narrowing, is now implemented
([section 7](#7-include-path-narrowing-task-5-hc-12)), awaiting independent acceptance (2026-10-16). ARCH-04 has not
started.
Starting main HEAD: `565ef3e` (ARCH-02 accepted at `dc09e8f`; HC-10 accepted at `565ef3e`). This is a relocation only:
no source line changed except includes and path comments, and no model, RNG, timestep or default behavior changed.

## 1. What moved

| Module (target, alias unchanged) | From | To | Files |
| --- | --- | --- | --- |
| `platform` (`fsim_platform`) | `src/platform` | `src/foundation/platform` | 14 |
| `core` (`fsim_core`) | `src/core` | `src/foundation/core` | 19 |
| `io` (`fsim_io`) | `src/io` | `src/foundation/io` | 7 |
| transport (`ipc`, `fsim_ipc`) | `src/ipc` | `src/foundation/transport` | 11 |
| `sim` (`fsim_sim`) | `src/sim` | `src/simulation/flight` | 17 |
| `control` (`fsim_control`) | `src/control` | `src/simulation/flight_control` | 68 |
| `effects` (`fsim_effects`) | `src/effects` | `src/simulation/environment` | 5 |
| `comm` (`fsim_comm`) | `src/comm` | `src/simulation/communications/network` | 3 |
| `ms` (`fsim_ms`) | `src/ms` | `src/simulation/mission_systems` | 51 |

- **Moves:** 195 tracked files moved with `git mv`, each a rename, so history follows them. Before moving, I checked
  that no untracked, ignored or excluded entry sat in those directories.
- **Includes:** 591 literal includes in 257 files (`src`, `tests`, `examples`, `tools`, `python/src`) now name the
  destination paths, for example `"foundation/core/Log.h"` and `"simulation/mission_systems/radar/Radar.h"`. Each
  rewritten include was checked to name an existing file under `src`. Vendor includes were untouched.
- **Build and namespaces:** `src/CMakeLists.txt` adds the new subdirectories. Target names, aliases, namespaces
  (`fsim::core`, `fsim::sim`, `fsim::ms`, ...), link lists, `WHOLE_ARCHIVE` registration and per-target definitions are
  unchanged.
- **Separations the roadmap names:**
  - Physical effects (`simulation/environment`) stay apart from the rendered VFX in `src/world` (ARCH-06).
  - The generic network (`simulation/communications/network`) stays apart from modeled DMS (`src/session/Dms*`, ARCH-04).

## 2. Policy, gate and selector

- **Policy:** the moved modules no longer list legacy directories; their destinations own their sources. A new rule
  fails a legacy directory the policy still lists once it holds no source (`stale-legacy-directory`). Its fixture
  `test_a_moved_module_keeps_no_legacy_directory` fails on `565ef3e` and passes now.
- **Gate:** the architecture gate accepts the moved tree with the same counts as before: 20 modules, 374
  source/public files, the same two native and three public exceptions, and 41 classified public headers.
- **Selector:** the shared-core scope in `tools/testing/scopes.toml` names the new locations, so it claims what it
  claimed before.
- **Include roots (task 5):** at `30bc5ef` every target still exported the shared `src` include root, so the
  compiler saw every module's private headers; the review held this incomplete (HC-12). The narrowing is in
  [section 7](#7-include-path-narrowing-task-5-hc-12).

## 3. References

- **Updated:** comments and maintained documents that name the moved directories:
  - two public headers' comments (`ControlStack.h`, `MissionSystems.h`) and one source comment;
  - two test sources and the installed-consumer gate's comments;
  - the hangar and FERS tool comments;
  - `THIRD_PARTY_NOTICES.md`, `third_party/ntia/README.md`, and twelve maintained documents (the domain guides,
    `docs/sdk`, `resources.md`, `type-codes.md` and the architecture design document's source-tree table);
  - the current capability table in `docs/status.md`.

  Where another owner had uncommitted edits (the architecture design document, `docs/status.md`,
  `tools/architecture/README.md`), only these path changes are committed; their edits stay in the working tree.
- **HC-09's comment item:** `contracts/world_stream/WorldLayout.h`'s padding comment no longer says only two
  "published or recorded" records have padding. It now names the publisher's payload records, and notes that the
  recording's framing records have padding of their own, zeroed by the recording writer. This is a comment change only.
- **Kept as written:**
  - dated evidence (historical `docs/status.md` entries, the planning package's audit and plans);
  - the contracts headers' "had in `src/ipc`" history;
  - synthetic fixture paths in the architecture, boundary and selector tests;
  - the VSG vendor list.
- **Planning package:** the path map's moved units (P001-P009) now name their destinations, and the roadmap shows
  ARCH-03's status. These edits sit in the untracked planning package and are not committed with this change.

## 4. Verification

| Check | Result (2026-10-16) |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | 0 warnings. Headless compiled 250 units, every moved module from its new path; Release compiled 294, the renderer included |
| Fast regression tier: every test measured under 5 s on the last full runs, plus the architecture, contracts, layout boundary, C ABI, Vision, capture, showcase, screenshot and installed-package gates | Headless 568/568 in 98 s; Release 633/633 in 118 s |
| `tools/ma_contract/baseline.py compare`, both builds | 19 comparisons, 0 failures. The headless export list is byte-identical to the one captured before the move (1,456 exports); layouts, enumerators, vtables and Python inventories are unchanged |
| `tools/architecture/check.py`; `test_check.py`; `tools/testing/test_scope.py` | Accepted; 129 cases; 49 cases (1 skipped without a build) |
| After the `WorldLayout.h` comment change (comment only) | Both builds 0 warnings; `contracts`, `simulation_interface_boundary`, `architecture_boundary`, `architecture_checker`, `testing_scope_checker` 5/5 |
| Planning document checker | Accepted |

No full suite ran, as the request directs. The selector escalates this package to the full suite (shared core and
build files changed), and that run stays an open evidence gate (register HC-11). The dated baseline is
[arch-03-baseline.json](arch-03-baseline.json).

## 5. Rollback

Move the nine directories back with `git mv`, then revert:
- the include rewrite (the same prefix map, reversed);
- `src/CMakeLists.txt`, the policy's directories and the scope patterns;
- the listed comment and document lines.

No other source changed.

To roll back the include-path narrowing alone (section 7), revert:
- the ten `fsim_include_view` calls, back to the shared `src` root;
- the two consumers' own roots;
- `include(IncludeViews)` and the three `cmake/*IncludeView*` scripts;
- the probe registrations;
- the `broad-include-root` rule and its fixture.

The compiled objects are identical either way.

## 6. Next

ARCH-04 (the simulation runtime, its services and the lower facade) may start after independent acceptance of this
package. The open carryovers are listed in the [register](handoff-carryover-register.md).

## 7. Include-path narrowing (task 5, HC-12)

Starting main HEAD: `30bc5ef`. This is a build-configuration change: no source or header line changed.

### Include views

`cmake/IncludeViews.cmake` defines `fsim_include_view(<target>)`:
- It mirrors a migrated target's headers into `<build>/include-views/<target>`, at their src-relative paths.
- Each mirrored header is a one-line forwarder. It names the real header by the same absolute path the shared root
  produced.
- Each of the ten targets (the `contracts` interface target and the nine moved ones) exports its view instead of
  `${CMAKE_SOURCE_DIR}/src`, as `$<BUILD_INTERFACE:...>`.
- A target compiles with its own view plus the views its linked providers export.
- The public `include/` root stays where it was: `core`, `ms` and `contracts` export it.

Unchanged:
- include spelling (`"foundation/core/Log.h"`), `__FILE__` strings and include guards;
- target names, aliases, links, `WHOLE_ARCHIVE` registration and definitions;
- the installed package. Its exported targets name only `include/` and `fsim::sdk`; no view path appears.

Adding or removing a header re-runs CMake. A forwarder whose header is gone is removed; an unchanged forwarder is not
rewritten, so it triggers no rebuild. Both behaviors were checked by adding a header and then removing it.

### Resulting environments

The roots below follow the CMake link declarations, which match the policy's allowed-dependency closures. Vendor roots
(`stb` in `io`, JSBSim in `sim`, NTIA on two `ms` sources) are private and unchanged.

| Module (target) | Compiles with the views of (plus its own) |
| --- | --- |
| `contracts` (interface) | none; its consumers get its view and `include/` |
| `platform` | none |
| `core` | `platform`; `include/` |
| `io` | `core`, `platform`; `include/` |
| `ipc` (transport) | `contracts`, `core`, `platform`; `include/` |
| `sim` | `contracts`, `core`, `platform`, `io`; `include/` |
| `control`, `effects` | `sim`, `contracts`, `core`, `platform`, `io`; `include/` |
| `comm` | `control`, `sim`, `contracts`, `core`, `platform`, `io`; `include/` |
| `ms` | `core`, `platform`; `include/` |

[arch-03-include-views.json](arch-03-include-views.json) records these environments for both presets. In each preset,
all 98 sources the nine compiled targets build were checked against `compile_commands.json`. Each uses exactly these
roots, and no migrated target holds the shared `src` root.

### Consumers that borrowed the root

Narrowing exposed two non-migrated targets that reached their own headers only through the moved libraries' exported
`src` root:
- `flightsim`, for `app/Options.h`;
- `fsim_strike_capture_test`, for the viewer's `app/StrikeCapture.h`.

Each now declares the shared root itself (`PRIVATE`), which is the visibility it had before. No migrated module's own
source needed a header outside its closure. Non-migrated targets keep the shared root until their packages move them
(register HC-14).

### Compile-level fixtures

`include_view_<module>` is ten CTest tests with label `architecture`, run by `cmake/CheckIncludeView.cmake`. They need no
build:
- Each probe is a translation unit holding one include. It is compiled `-fsyntax-only` with the real compiler in the
  target's own environment: its include directories, definitions and options, read from the target's properties at
  generate time.
- **Allowed** headers (28: the target's own and each provider's) must compile.
- **Forbidden** headers (23) must fail with the compiler's not-found diagnostic naming that header. Each must then
  compile once the shared `src` root is added back. So the header is sound, only its visibility differs, and the
  configuration at `30bc5ef` fails these tests.

| Module | Forbidden probes (an unrelated module's private header) |
| --- | --- |
| `contracts` | `foundation/core/Units.h`, `simulation/flight/VehicleState.h` |
| `platform` | `foundation/core/Units.h`, `session/WorldOptions.h` |
| `core` | `foundation/io/AircraftFile.h`, `simulation/flight/VehicleState.h` |
| `io` | `foundation/transport/WorldRegistry.h`, `simulation/flight/VehicleState.h` |
| `ipc` | `simulation/flight/VehicleState.h` (the edge ARCH-02 removed), `foundation/io/AircraftFile.h` |
| `sim` | `simulation/flight_control/Pid.h`, `simulation/mission_systems/rf/Rain.h`, `foundation/transport/WorldRegistry.h` |
| `control` | `simulation/environment/Effect.h`, `simulation/mission_systems/rf/Rain.h`, `foundation/transport/WorldRegistry.h` |
| `effects` | `simulation/flight_control/Pid.h`, `simulation/communications/network/Comm.h` |
| `comm` | `simulation/mission_systems/rf/Rain.h`, `session/Dms.h` (modeled DMS stays apart from the network) |
| `ms` | `simulation/flight/VehicleState.h`, `simulation/flight_control/Pid.h`, `session/WorldOptions.h` |

`include_view_checker` (`cmake/TestCheckIncludeView.cmake`) runs the check on mutated copies of `ms`'s generated
check. The unmutated copy must pass. Each of five mutations must fail with its own reason:
- the shared root restored;
- an equivalent broader root (`src/.`), which the string guard does not catch and only the compile probes reject;
- the `core` provider's view removed;
- a stale probe header;
- an unsupported compiler.

The probes support GCC only, the compiler of every supported (UCRT64) preset; any other compiler is refused explicitly.

### Gate rule

The architecture gate rule `broad-include-root` covers a module at its destinations, with no legacy directory or file
claim. Such a module may declare no include directory under `src` outside its destinations.
- **Fixture:** `test_a_moved_module_exports_no_include_root_beyond_its_destination`. It rejects the shared root and a
  domain root, and accepts the module's own destination and its view. It fails with `30bc5ef`'s checker.
- **Effect:** on the `30bc5ef` tree the rule rejects all ten modules; it accepts the current tree.
- **Later packages:** the rule applies to each module automatically once a later package moves it.
- **Coverage:** the probes check representative headers. Completeness over every header comes from three sources: the
  build itself (every source compiles in its narrowed environment), the gate's static include closure, and this rule.

### Verification (2026-10-16)

| Check | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | 0 warnings; Release compiled 302 units, including the renderer, viewer and tools |
| Object identity, against the `30bc5ef` builds (sha256) | All 498 headless and 545 Release object files byte-identical: the compiled code is unchanged |
| `tools/ma_contract/baseline.py compare`, both builds | 19 comparisons, 0 failures; all 22 export, vtable and Python-inventory captures byte-identical to ARCH-03's |
| Relevant tests, both presets, `--no-tests=error`: the ten `include_view_*`, `include_view_checker`, `architecture_boundary`, `architecture_checker`, `sdk_guard_checker`, `testing_scope_checker`, `contracts`, `strike_capture`, `launcher_contract` | 18/18 headless, 18/18 Release (34 s each) |
| `test_check.py`; `tools/testing/test_scope.py` | 130 cases; 49 cases (1 skipped) |
| Architecture gate | Accepted with unchanged counts (20 modules, 374 files, 41 public headers, the same exceptions) |
| Environment cross-check | 98 compiled sources per preset use exactly the recorded roots |

No full suite ran, as the request directs. The selector escalates this change (build system) to the full suite of 702
headless tests. That run stays the open evidence gate HC-11. Logs are in `D:/ENV/tmp/hc12/`.
