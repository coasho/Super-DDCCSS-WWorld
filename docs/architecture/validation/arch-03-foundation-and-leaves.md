# ARCH-03 foundation and domain leaves

Status: implemented, awaiting independent acceptance (2026-10-16). ARCH-04 has not started.
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
- **Include roots (task 5):** each target keeps the shared `src` include root, which the destination-path includes
  need. The architecture gate, not the compiler search path, holds every include to its module's closure. Per-target
  narrowing would change the include style again; it is recorded as an explicit decision (register HC-12), not done
  here.

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

## 6. Next

ARCH-04 (the simulation runtime, its services and the lower facade) may start after independent acceptance of this
package. The open carryovers are listed in the [register](handoff-carryover-register.md).
