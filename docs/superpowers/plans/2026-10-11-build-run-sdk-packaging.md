# Build, run and installed SDK consistency

## Goal

Make one selected preset govern configuration, builds, application launch, tests and distribution. Support the first build from an empty build tree, and make the SDK component usable by an installed consumer with its own runtime data.

## Scope and constraints

The approved maintenance scope is build/run/test entry points, installation, package acceptance and documentation. Preserve native modules, public APIs, simulation behavior, aircraft definitions and generated assets. Written files use English. Toolchains, caches and compilation temporary files stay under `D:/ENV`.

## Design

- Use `fsim.cmd [--preset NAME] COMMAND [ARGS]`, with explicit selection taking precedence over `FSIM_PRESET`, then `ucrt64-headless`. Do not select binaries by scanning unrelated builds.
- Delegate Windows launch behavior to a shared PowerShell entry point. Bootstrap configuration for `build` and `dist`; forward application arguments and process exit codes.
- Install the simulation data needed by SDK, tools and viewer components. Stage generated damage definitions and their referenced JSBSim piece models along with aircraft and stores.
- Test a separately installed SDK under a prefix containing spaces. Build a consumer against its CMake exports, verify the loaded library and complete data manifest, and prove that corrupt installed stock/custom/store models are rejected. Keep the development resolver's existing source fallback behavior unchanged.

## Tasks and acceptance

1. Launcher contract: demonstrate the old first-build failure using the real CMD entry; implement preset selection, forwarding and testbench consistency; run regression checks including spaces and failures.
2. SDK packaging: demonstrate the missing data failure using the installed-consumer gate; install complete runtime data and notices; ensure distribution builds all enabled install targets; run positive and negative installed-data probes.
3. Integration: register launcher and package acceptance in CTest, use matching presets in CI, set compilation temporary directories, and update README, development/resource guidance and current status.
4. Review and verification: run focused regressions, real headless build/run/test/dist commands and installed-consumer acceptance; review the complete maintenance diff and resolve material findings.

## Review focus

Check Windows argument quoting, environment precedence, configuration isolation, disabled optional targets, stale distribution files, relocatable package paths, runtime DLL dependencies and any accidental reliance on source data. Report render/GPU paths separately from headless evidence.

## Execution record

- First-round cleanup committed as `2b83f24`; second-round work starts on `codex/build-run-sdk-packaging` from that commit.
- Launcher and SDK tasks have separate file ownership. CTest registration, presets, CI and documentation belong to the coordinating task.
- Launcher and SDK regressions first reproduced the old failures, then passed. Additional review fixes aligned package exports with the fixed runtime layout and invalidated stale generated Python entries.
- Final headless configure/build/run/dist commands passed. The functional preset passed all 535 cases in 346.02 seconds; launcher acceptance contains 79 checks. SDK distribution matched the 1,230-file consumer manifest and tested DLL bytes.
- Independent review passed. The 40 unchanged Hangar cases retain the first pass's verification; rendering, render SDK installation, CPack ZIP and remote CI remain outside this pass's exercised scope. See [current status](../../status.md#second-maintenance-verification-2026-10-11) for evidence and limits.
