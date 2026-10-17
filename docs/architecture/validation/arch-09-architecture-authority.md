# ARCH-09 relocate existing architecture authority

Status: implemented on main (2026-10-17); awaiting independent acceptance. ARCH-10 has not started.
Starting main HEAD: `dd7dc9f` (ARCH-08 accepted).

This package applies the [document register](../migration/document-register.csv): each registered architecture document
moves once into `docs/architecture`, every Markdown link to it follows, and a redirect stays where anything else still
names the old path. Bodies are moved as written: their dates, achieved evidence and pending gates are unchanged, and
only their link targets were recomputed.

The user's decisions for this package (2026-10-17):
- **Other owners' edits:** files carrying another owner's uncommitted edits move with them; the edits stay uncommitted,
  at the new paths.
- **MA training authority:** D07-D09 stay in place until the M8 feature handoff.
- **References:** Markdown links are updated. Prose and code comments keep naming the old paths, which redirects
  answer.
- **Project instructions:** `AGENTS.md` and `.claude/CLAUDE.md` are released for link updates (D20).

## 1. What moved

| Row | From | To | Redirect |
| --- | --- | --- | --- |
| D02 | `docs/FlightSim_System_Architecture_and_Design.md` | [design/platform-baseline.md](../design/platform-baseline.md) | Yes |
| D03 | `docs/mission-systems.md` | [modules/simulation/adr-30-mission-systems.md](../modules/simulation/adr-30-mission-systems.md) | Yes |
| D04 | `docs/vehicle-interface.md` | [modules/simulation/vehicle-interface.md](../modules/simulation/vehicle-interface.md) | Yes |
| D05 | `docs/flight-autonomy.md` | [modules/simulation/flight-autonomy.md](../modules/simulation/flight-autonomy.md) | Yes |
| D06 | `docs/agent-boundary.md` | [design/agent-boundary.md](../design/agent-boundary.md) | Yes |
| D10 | `docs/superpowers/specs/2026-10-15-simulation-interface-boundaries.md` | [design/simulation-interface-boundaries.md](../design/simulation-interface-boundaries.md) | No |
| D11, D12 | The A1 design and plan in `docs/superpowers` | [migration/history/](../migration/history/README.md), basenames kept | No |
| D13 | The ten A2-A6 designs and plans in `docs/superpowers` | [migration/history/](../migration/history/README.md), basenames kept | No |
| D14 | `docs/superpowers/specs/2026-10-11-sdk-missile-engagement-design.md` | [modules/simulation/native-engagement-design.md](../modules/simulation/native-engagement-design.md) | No |

Further details:
- **Renames:** 19, all with `git mv`, committed apart from the redirects so that history follows each body (git
  detects all 19 as renames, 92-100 % similar).
- **Redirects:** a page at an old path holds only the new path and says it is no authority. One exists where something
  other than a Markdown link still names the path: code comments, generated aircraft data, build comments or prose. That
  is the five documents D02-D06, named by 4, 31, 51, 236 and 17 files. Nothing but links named D10-D14, so they have
  none.
- **History:** a new [index](../migration/history/README.md) lists the A1-A6 designs and plans, historical and
  superseded where they differ.
- **Deferred:** D07-D09, the MA training design, plan and allocation, stay canonical at their current paths (full M8 is
  pending, HC-06 and HC-07). Their links to relocated documents were updated.
- **Instructions (D20):** `AGENTS.md` and `.claude/CLAUDE.md` link only the deferred MA documents and the Viewer
  evidence, so neither changed.

## 2. Links and references (tasks 2 to 4)

- **Markdown links:** 368 link targets in 43 files were recomputed, anchors kept (the generator's and the checker's
  two included): links in the moved bodies to everything
  else, and links elsewhere to a moved body. One link in a moved body whose target began on the next line was fixed by
  hand; section 5 has why.
- **Generators and checkers:**
  - `boundary.py`'s rendered ADR-31 link changed with `docs/sdk/boundary.md`, which it renders;
  - the MA contract checker now validates the agent boundary at its new path (`LINKING`).
- **SDK reference:** `docs/sdk` keeps the public API reference, and the architecture documents link to it (D15); no
  contract body was duplicated.
- **Code comments:** they keep naming the old paths, as decided. No source, header, test or generated file changed, so
  no build input changed.

## 3. Owners' edits

Ten files carried another owner's uncommitted edits and changed in this package. Only this package's lines are
committed (HEAD-plus-mine blobs); the owners' edits stay uncommitted:
- **Moved with their edits:** the platform baseline, the agent boundary, the simulation interface boundaries and the
  A1 design. Their edits now sit at the new paths. In the agent boundary, one link in the owner's lines was recomputed
  with the others (11 links in the working file against 10 at HEAD).
- **Links updated:** `docs/sdk/agents.md`, `docs/sdk/boundary.md`, `docs/status.md`, the two deferred MA documents and
  `boundary.py`.

The other owners' 18 remaining files are untouched.

## 4. Index and register (tasks 1, 5 and 6)

- **Index:** the [architecture index](../README.md) has an authority table giving every registered document its
  category, owner and status.
- **Register:**
  - relocated rows name their new paths, with `relocated` or `archived` and where they came from;
  - D13 names the history glob;
  - D07-D09 record the deferral, and D20 its release;
  - every row has a status column.
- **No new architecture plan outside `docs/architecture`:** this package wrote none there, and its only page outside is
  the five redirects. Three architecture documents outside it are unregistered: ADR-26 (`docs/control-architecture.md`),
  ADR-27 (`docs/rotorcraft.md`) and the build/run packaging plan in `docs/superpowers` (HC-33).

## 5. Verification (2026-10-17)

| Check | Result |
| --- | --- |
| Every local Markdown link in the repository, anchors included (a scratch checker; `.codex` excluded) | Before: 97 files, 1,223 links, none broken. After the moves, before any redirect: none broken, so no link still pointed at an old path. With the redirects, the history index and this record: 104 files, 1,278 links, none broken |
| Planning document checker | Accepted: 49 Markdown documents reachable from the index, local links valid, 77 source units and 22 document units |
| Commit A's staged diff | 335 lines out, 335 in. Each differs only in a link target, except the split link's continuation and the checker's page path |
| Selector's plan for this package's 62 paths (documentation, SDK principals, MA contract scopes) | Headless 6/6 (8.9 s), Release 6/6 (8.3 s): `architecture_boundary`, `ma_contract` (12 pages, 330 local links, 0 errors), `ma_contract_checker`, `sdk_boundary` (the rendered `boundary.md` matches its template), `sdk_guard_checker`, `testing_scope_checker` |

Logs are in `D:/ENV/tmp/arch09/`. No native build ran; no build input changed.

Limits:
- **Full suite:** none ran (HC-11).
- **Link checks:** they cover local Markdown links. A path named in prose or a comment is answered by a redirect only
  where one exists; none is needed for D10-D14.
- **Split links:** the scratch checker first missed a link whose target began on the next line, as the first rewrite
  did; the planning checker caught it. Both scratch tools now accept the form, and the final counts above include it.

## 6. Open items

- **HC-33:** three architecture documents outside `docs/architecture` are unregistered: ADR-26, ADR-27 and the
  build/run packaging plan.
- **HC-34:** D07-D09's relocation is deferred to the M8 feature handoff. `CLAUDE.md` and `AGENTS.md` change with it.
- **HC-35:** a process breach, self-reported. One recursive search traversed the working tree, `.codex` included,
  before its output was filtered; it was stopped. Separately, `.codex` is not gitignored, so the untracked-file
  listings of this and earlier packages' scratch scripts could enumerate a `.codex` directory before filtering it out.
  Every listing now passes `--exclude=.codex`.
- **HC-24 to HC-27, HC-07, HC-11:** unchanged.

## 7. Rollback

1. Delete the five redirect pages and the history index.
2. Move the 19 bodies back with `git mv`, and revert the link changes, `boundary.py`'s link and the checker's page
   path.
3. Restore the register's rows and the index's authority table.

## 8. Next

ARCH-10 (final three-domain acceptance) requires the next user-relayed assignment.
