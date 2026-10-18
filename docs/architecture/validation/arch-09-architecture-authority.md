# ARCH-09 relocate existing architecture authority

Status: independently accepted at main `6bb79cc` plus the inspected uncommitted planning/index package (2026-10-17); HC-33 closed. HC-34 is the approved deferral; HC-35 remains pending human/instruction-owner disposition; HC-36 retains its maintenance owner. ARCH-10 has not started. Earlier sections retain their dated handoff findings; section 11 records final acceptance.
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
  A1 design. Their edits now sit at the new paths. Link targets in the owners' changed lines were recomputed with the
  rest, their wording untouched: in the platform baseline and the agent boundary (whose owner's lines add one link,
  11 against 10 at HEAD).
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

## 9. Independent review at 4d7a9be

The [independent review](arch-09-review-2026-10-17.md) accepts the delivered relocation subset: 19 moved bodies,
five redirect-only pages and the committed non-link text/reference mapping. **HC-33 (P2)** blocks the complete
ARCH-09 gate. Register and directly index ADR-26, ADR-27 and the packaging plan with category/owner/status;
their bodies can remain where they are. Merely assigning this metadata work to pre-ARCH-10 does not meet the gate.

Current exact fast checks pass 5/5 headless (5.86 s) and Release `sdk_boundary` 1/1 (2.61 s). The independent
pre-review scoped link audit finds 99 files/1,279 local links with no missing paths/anchors. These are separate from
the author's broader totals above. No native build, GUI run or full suite was repeated.

Owner-edited lines preserve their wording with authorized destination rewrites; exact pre-move uncommitted payload
identity remains author-reported because no before-body snapshot/hash was found. HC-34 stays approved; HC-35 is
not acknowledged on the user's behalf or closed by changing ignore policy. HC-36 records updating retained comment
references during the affected files' future maintenance. Evidence is under
`build/architecture-reviews/arch09-4d7a9be/`. Stop for the bounded metadata correction; do not start ARCH-10.

## 10. HC-33 correction (2026-10-17)

Corrects the gap the [independent review](arch-09-review-2026-10-17.md#1-hc-33-authority-metadata-is-required-now-p2)
found (HC-33, P2): three architecture documents outside `docs/architecture` had no category, owner, status or index
entry of their own. Each is now registered and indexed where it is. Starting HEAD: `4d7a9be`.

| Row | Document | Classification | Owner | Status |
| --- | --- | --- | --- | --- |
| D23 | [ADR-26 control architecture](../../control-architecture.md) | Current authority: capability contracts over the control runtime, accepted 2026-09-26 and implemented. ADR-27 and ADR-28 extend it; the public reference stays in `docs/sdk` | simulation | Canonical in place |
| D24 | [ADR-27 rotorcraft](../../rotorcraft.md) | Retained domain reference: the rotorcraft decision record, accepted and implemented 2026-09-26. It extends ADR-26, which stays in force | simulation | Retained reference |
| D25 | [Build/run and installed-SDK packaging plan](../../superpowers/plans/2026-10-11-build-run-sdk-packaging.md) | Historical evidence: the 2026-10-11 pass delivered in `f8cfec9`, before A1, with its execution record. The development guide and the ARCH-08 record are current | architecture_history | Historical in place |

- **Register:** rows D23-D25 have operations `retain-authority`, `retain-reference` and `retain-history` with phase
  `retain`, so no move is planned; a later move needs its own register update. The other rows are unchanged.
- **Index:** the [architecture index](../README.md)'s authority table links each directly, ADR-26 and ADR-27 under
  Simulation and the plan under History, and says that these three stay at their own paths.
- **Bodies:** none moved or changed, and none has a copy or redirect elsewhere. Their SHA-256 hashes match before and
  after.

| Check | Result |
| --- | --- |
| Planning document checker | Accepted: 50 Markdown documents reachable from the index, local links valid, 77 source units and 25 document units (22 before). A scratch gate check finds each of the 25 register rows with an owner, a status and an index entry naming it |
| Every local Markdown link in the repository, anchors included (the scratch checker; `.codex` excluded from the Git listing, before any access) | 105 files, 1,303 local links, none broken |
| Selector's plan for the six changed paths (documentation scope): headless only, no native build | Headless 3/3 (2.61 s): `architecture_boundary`, `ma_contract`, `sdk_guard_checker` |

Logs, the before copies and their hashes are in `D:/ENV/tmp/hc33/`. No native build, GUI run or full suite ran
(HC-11).

## 11. Independent HC-33 and ARCH-09 acceptance at 6bb79cc

The [final independent review](arch-09-review-2026-10-17.md#7-final-hc-33-and-arch-09-acceptance-at-6bb79cc)
closes HC-33 and accepts ARCH-09. D23-D25 have category/owner/status and direct authority-index links; all preceding
22 register rows are unchanged. ADR-27 remains in force for rotorcraft despite its retained-domain classification.
All three bodies match their before-snapshot SHA-256 values. Shared reviewer-edited record/status snapshots show
only additive correction sections. No body move is required.

Exact existing documentation gates pass 3/3 (3.10 s); the pre-review scoped link audit finds 100 files/1,302 local
inline links with no errors. Final documentation results and logs are recorded in the review and
`build/architecture-reviews/arch09-6bb79cc/`. These scoped totals are separate from the author's section 10 totals.
No native build, Release rerun, GUI or full suite ran. The original relocation's exact uncommitted-payload parity
remains author-reported where before snapshots were unavailable; section 9's limit is not retroactively closed.

Acceptance includes the inspected uncommitted index/register/roadmap, not just commit `6bb79cc`. HC-34's approved
M8 deferral, HC-35's pending human/instruction-owner disposition and HC-36's scheduled maintenance remain visible,
as do prior owners' final-program gates. No files were staged, committed or pushed by this review. ARCH-10 has
not started.
