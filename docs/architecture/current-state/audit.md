# Current-state architecture audit

## Baseline and method

Refreshed evidence (ARCH-10, 2026-10-17): the [evidence JSON](dependency-audit.json) now records the migrated tree (main 6bb79cc working tree): 28 native modules and 401 files, with no native or Python cycle. The assessment below is the dated pre-migration baseline; the [ARCH-10 record](../validation/arch-10-final-acceptance.md) states the final state.

Date: 2026-10-16. Reviewed main HEAD: d344e2b. The audit began at 0cf0150; Claude committed M8 generic during the audit, and the scoped source evidence was refreshed at d344e2b. This is a working-tree audit: prior terminology edits remain present. The architecture checker itself has a pending wording edit; its current policy and source snapshot are identified by the evidence file.

The audit inspected explicit project paths only: src, include/fsim, python/fsim, relevant CMake declarations, tools/architecture, and maintained project documents. Every recursive walk pruned directories named .codex before descent. No profile memory, other worktree, native configure/build, simulation run or Viewer run was used.

A normal-host, read-only invocation of the existing checker returned:

~~~text
Architecture accepted: 19 native modules, 367 source/public header files; all CMake branches scanned.
~~~

A restricted-process invocation did not discover the source declarations correctly and was discarded as invalid discovery evidence. The accepted result above used the normal host context.

The [evidence JSON](dependency-audit.json) records the actual direct target graph, checker result, policy hash, literal Python imports, deferred-edge classification and scan limitations.

## Verdict by question

| Question | Assessment |
| --- | --- |
| Is the repository entirely unstructured? | No. Native module ownership, a dependency policy, headless/render separation and compatibility gates exist. |
| Are the three requested domains obvious from the directory tree? | No. Source names and SDK assembly combine different responsibilities. |
| Are native module dependencies cyclic today? | No cycle found in the 19 registered native modules; the current checker passes. |
| Is every dependency clean and proven acyclic? | No. Python has two deferred import cycles, broad private include roots remain, and build/package/dynamic-import coverage is incomplete. |
| Is a three-domain relocation safe as a rename-only operation? | No. Contract, facade, registration, asset and package ownership must be addressed first. |

## Actual native dependency graph

Arrows mean consumer depends on provider. All conditional native branches are included; this is not the configured File API graph of one preset.

| Consumer | Direct project dependencies |
| --- | --- |
| platform | none |
| core | platform |
| io | core |
| sim | core, io |
| control | sim, core |
| effects | sim, core |
| comm | control, core, platform |
| ms | core |
| ipc | sim, core, platform |
| session | ms, control, effects, comm, ipc, sim, io, core |
| env | session, control, sim, io, core |
| sdk | env, session, ms, control, effects, comm, ipc, sim, io, core, platform |
| render | core |
| flow | core |
| world | sim, render, core, ipc, io, flow |
| ui | sim, render, world, ipc, core |
| vision | sdk, world, ipc, io, core, platform |
| app | sim, io, core, platform |
| viewer | ui, world, render, ipc, control, sim, io, core, platform |

JSBSim, graphics libraries and platform libraries are external edges and remain subject to their own support/licensing constraints.

The source policy distinguishes explicitly owned files in shared directories, including Flow in src/world and Viewer files in src/app. The native checker rejects undeclared module imports, cycles, private public-header leakage and headless graphics dependencies. Its deliberately limited parser and source ownership policy are documented in [the existing checker guide](../../../tools/architecture/README.md).

## Findings and destination decisions

| ID | Evidence | Meaning | Required action |
| --- | --- | --- | --- |
| A-01 | src/world/CMakeLists.txt; src/session/World.h | The word world names both scene construction and simulation authority | Move scene construction into visualization/scene; preserve authoritative World under simulation/runtime |
| A-02 | src/sdk/CMakeLists.txt; MissionGateway.h; MissionBatch.cpp | SDK links training and simulation and implements mission orchestration | Extract a lower simulation facade, then a training mission target; retain a thin final SDK assembly |
| A-03 | include/fsim/Scenario.h; session/Scenario.h | A public compatibility header combines simulation scenario and legacy VecEnv data | Separate internal value/recipe ownership below domains without changing the public aggregate or type identity |
| A-04 | training.py local imports of fsim; package __init__ importing training | Deferred facade cycle | Use a lower error/backend dependency; no private training implementation imports the package root |
| A-05 | agent.py importing world values; world.py locally importing Agent/Referee | Deferred simulation-wrapper cycle | Extract shared values/contracts and use one-direction factories or backend handles |
| A-06 | python/fsim/vision.py importing VecEnv | Visualization wrapper also contains training observation composition | Move the composition into an optional outer integration; camera rendering has no training dependency |
| A-07 | PUBLIC source-root include directories | Dependency checker compensates for broad compiler visibility | Narrow per-target includes incrementally; reject parent-relative imports that bypass ownership |
| A-08 | ipc/WorldLayout.h; ipc/Recording.h | Native state and frozen stream layouts are shared by producer and viewer | Keep protocol values below both domains; separate transport implementation from domain authority |
| A-09 | src/effects versus src/world/Flow* and Exhaust* | Physical disturbances and visual effects are different owners | Put physical effects under simulation/environment; rendered VFX under visualization/vfx |
| A-10 | viewer_main.cpp demo/live/replay branches | One application embeds demo simulation as well as passive viewing | Separate source adapters and the demo publisher; preserve command-line compatibility during transition |
| A-11 | FSIM_BUILDING, WHOLE_ARCHIVE, native Python registration | Registration/export/type ownership depends on assembly details | Keep one definition/registration owner and verify old binaries before changing target assembly |
| A-12 | docs root, sdk references, superpowers plans | Architecture decisions, feature plans and evidence are dispersed | Use this architecture index plus the document register; move old authority only with redirects and updated inbound links |

## Python import evidence

The scan covers 16 Python source modules and treats fsim._native and fsim._vision as compiled extension leaves. It excludes implicit package initialization from owner-level graph edges.

- Eager literal-import SCCs: none found.
- Literal imports including function-local imports: fsim with fsim.training; fsim.agent with fsim.world.
- The reverse edges are deferred, so these findings are not claims that current import initialization fails.
- Conditional branches are conservatively combined. Dynamic imports, native initialization, plugin loading and transitive external packages are not proven by this AST scan.
- Optional Gymnasium/SB3 modules are not eagerly imported by the root package. Preserve that property.

The target policy forbids deferred backedges too. Replacing an eager import with a local import does not close a target architecture violation.

## Existing acceptance and active work

M7 core at 0cf0150 passed independent acceptance. This includes the final EpisodeHold fix, independent focused gates and public preopened-handle probes; see the accepted [mission contract](../../sdk/mission-training.md) and the dated [status record](../../status.md). M8 generic was committed at d344e2b and awaits independent acceptance. This audit does not independently accept its runtime results. Review its handoff, record remaining gates and release its shared write/build window before source migration.

Current acceptance is not evidence that the original MS high-fidelity roadmap or all Viewer goals are complete. The simulation plan retains remaining MS work and the visualization plan retains dated graphics/performance limits.

## Limits and conclusions

The native source DAG is a useful established boundary, not a complete proof of package separation. This audit did not run the current native suite, GPU builds, runtime import stress, configured linker analysis or future-directory enforcement. Those gates belong to later exclusive windows.

The migration should retain the existing good dependency direction, remove the identified Python/facade backedges, make directory ownership obvious, and strengthen automated checks. It should not replace functioning simulation algorithms merely to make the tree symmetrical.
