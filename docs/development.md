# Development workflow

This is the working guide for this checkout. [status.md](status.md) is the sole current capability and milestone summary; [resources.md](resources.md) identifies editable inputs and generated outputs. Design rationale remains in the architecture and ADR documents. Commands below run from the repository root in PowerShell unless stated otherwise.

## Environment and supported build

The maintained Windows build uses MSYS2 **UCRT64**, GCC, Ninja and CMake 3.25 or newer, with C++17. The presets and [toolchain](../cmake/toolchains/ucrt64.cmake) are the implementation of this contract. MSVC branches in CMake are not evidence of a verified alternative toolchain.

Keep installed environments, toolchains and dependency caches under `D:\ENV`. Reuse existing installations. Before compiling, set all temporary and extension-build directories explicitly:

```powershell
$env:TMP = 'D:\ENV\tmp'
$env:TEMP = 'D:\ENV\tmp'
$env:TORCH_EXTENSIONS_DIR = 'D:\ENV\tmp'
$env:PATH = 'D:\ENV\DevLanguages\Cpp\msys2\ucrt64\bin;' + $env:PATH
```

The toolchain defaults to `D:/ENV/DevLanguages/Cpp/msys2/ucrt64`. Override `FSIM_UCRT64_ROOT` for another UCRT64 installation under `D:\ENV`, and adjust PATH too: the preset's PATH still contains the default location. In an MSYS2 shell, the toolchain can fall back to `MSYSTEM_PREFIX` when the default compiler is absent.

The Python SDK requires 64-bit **CPython >= 3.11 from python.org or conda**, with `python3.dll` and `Python.h`. MSYS2 Python does not satisfy this stable-ABI integration. [python/CMakeLists.txt](../python/CMakeLists.txt) probes the interpreter named by `FSIM_PYTHON_EXECUTABLE`, or the Windows `py -3` launcher if none is supplied. A failed probe skips the Python SDK even when `FSIM_BUILD_PYTHON=ON`; read the configure output. Its native modules use the CPython 3.11 stable ABI. Hangar and its tests use numpy, scipy, matplotlib and Pillow; provision them in the selected environment under `D:\ENV`, with caches there too.

JSBSim and the VSG stack are tracked vendored sources in `third_party/`, as recorded in `.gitmodules`; there are no Git submodules to initialize. Configure a fresh checkout directly:

```powershell
cmake --preset ucrt64-headless -DFSIM_PYTHON_EXECUTABLE=D:/ENV/python/3.13/python.exe
cmake --build --preset ucrt64-headless
ctest --preset ucrt64-headless
```

The interpreter path is an example matching the local installation; select the actual supported interpreter. The three main presets are:

| Preset | Configuration | Renderer |
| --- | --- | --- |
| `ucrt64-headless` | Release | Off |
| `ucrt64-release` | Release | On |
| `ucrt64-debug` | Debug | On |

Tests, examples, command-line tools and the Python SDK are enabled by default. `FSIM_WITH_RENDER` defaults off outside presets. Use the build's own preset throughout; do not mix binaries from different build directories.

For rendering, the actual dependency entry point is the [deps superbuild](../deps/CMakeLists.txt), not a vcpkg recipe:

```powershell
cmake -S deps --preset deps-ucrt64
cmake --build build/deps-ucrt64
cmake --preset ucrt64-release -DFSIM_PYTHON_EXECUTABLE=D:/ENV/python/3.13/python.exe
cmake --build --preset ucrt64-release
ctest --preset ucrt64-release
```

It builds the checked-out vsg, vsgXchange and vsgImGui sequentially into `build/deps-ucrt64/install`, which the main presets put on `CMAKE_PREFIX_PATH`. The UCRT64 Vulkan headers/loader, glslang and the dependencies required by those upstream projects must be available. Consult each checked-out upstream README and its version support matrix before changing compiler, library or language-standard versions. Rendering also requires the bundled NVIDIA Flow headers and DLLs; [cmake/Flow.cmake](../cmake/Flow.cmake) rejects a renderer configure without them. An alternate built checkout can be selected with `FSIM_FLOW_DIR`; rebuilding Flow uses `tools/flow/build-flow.sh` and its pinned upstream version. Do not treat a render configure or a headless test as GPU validation.

## Direct commands and launcher limits

Configure and build directly with CMake on a fresh checkout. `fsim.cmd` checks for an existing application before dispatching any command, including `build`; consequently `fsim build` cannot bootstrap an empty build tree. It prefers release, then headless, packaged release viewer and debug binaries. Its `build`, `test` and `dist` actions always target release rather than the binary it selected. A headless selection has no viewer. These are current launcher limitations, not fixes completed by documentation.

After the relevant build exists, `fsim.cmd where`, `fsim.cmd python <script>` and `fsim.cmd hangar ...` are useful shortcuts. For an explicit build, use `build/<preset>/python/fsim-python.cmd` and put `tools/hangar` on PYTHONPATH. See [hangar.md](hangar.md) for stages; resource regeneration is a separate task from writing documentation.

```powershell
cmake --build --preset ucrt64-release --target dist
cmake --install build/ucrt64-release --component viewer --prefix D:/ENV/fsim-viewer
```

The `dist` target produces separate SDK, tools, examples, viewer (when enabled) and Python (when available) trees. The SDK component currently contains DLLs, headers, CMake exports and SDK documentation, but does **not** install the JSBSim data tree or designed aircraft/cards. A standalone SDK consumer therefore needs explicit runtime data provision; finding `fsim::sdk` alone does not prove a self-contained simulator package. See [resources.md](resources.md#staging-and-runtime-search).

## Change invariants and verification

Keep the existing public C++ API, C ABI and Python ownership/lifetime contracts aligned. A public surface change must update its principal classification in `docs/sdk/boundary.toml`, regenerate the Markdown inventory, and exercise the corresponding SDK and ABI tests. Agent permissions are part of behavior: do not broaden access to range operations or referee truth while adding convenience methods. [agent-boundary.md](agent-boundary.md) records the boundary design.

Preserve deterministic simulation streams, control trajectories and generated bytes outside the intended change. For flight/control/model work, compare the affected aircraft and the rest of the fleet, identifying named failures and digest differences. A clean process exit does not prove stability or conformance. For native mesher changes, compare generated GLB bytes across repeated runs and thread counts, and check the relevant corpus; do not reset unexpected generated differences to conceal them.

Use the existing layers in [tests/CMakeLists.txt](../tests/CMakeLists.txt) and [python/CMakeLists.txt](../python/CMakeLists.txt):

| Layer | Existing entry point | What it establishes |
| --- | --- | --- |
| Core, mission systems, integration | CTest cases, focused `-R` selection | Native behavior and scenario regressions |
| Public C++ and C ABI | SDK cases and `c_abi` | Consumer contracts against the shared library |
| Python and principals | `python_sdk`, `sdk_boundary` | Python behavior and classified exports |
| Control conformance | `ctest --preset ucrt64-headless -L conformance` | Lifecycle, allocation gate and adapter checks |
| Advertised fleet capabilities | `ctest --preset ucrt64-headless -L fleet` | Aircraft-specific acceptance cases; includes shards |
| Hangar methods and pipeline | `ctest --preset ucrt64-headless -L hangar` | Theory/data comparisons and stage checks |
| Renderer, RCS and cameras | Render preset cases; viewer screenshots/performance tools | Visual and GPU behavior on capable hardware |

Start with tests relevant to the change, then run the full applicable preset suite for behavioral changes. Record skips: camera tests may skip without a Vulkan device, and RCS tests may skip without ray queries. Viewer validation requires reading the captured image and confirming the model used in logs. A historical green suite, a workflow file, or a current test listing is not a fresh pass. The headless CI workflow itself notes that it was written before its first GitHub run; check actual run evidence before claiming CI success.

## Long-running work and evidence

A background command must write raw output directly to its log. Never end it with `| grep`, `| sed`, `| awk` or `| head`: block buffering can hide completed work until the process exits. Read and filter the saved log separately. If an unavoidable live pipeline needs filtering, use explicit line buffering (`grep --line-buffered`, `awk '{print; fflush()}'`, or `python -u`); `head` does not fix buffering.

Keep one background task entry per operation. Do not layer another background polling/wait task on a task that already reports completion. Use its existing completion notification and retain the original raw log.

Report the checkout/commit, preset, command, environment, elapsed time, tested scope, exit code, failures and skips. Keep before/after measurements under comparable conditions; separate implemented, measured, visually checked, and still unverified results. Documentation-only edits should be checked for links, command correctness, generated-file ownership and diff scope; they do not justify builds, asset generation or heavy flight campaigns.

## Documentation and skill maintenance

[status.md](status.md) owns current capability and milestone status. Keep architecture documents as design/rationale, roadmap documents as sequencing/history, [backlog.md](backlog.md) as unresolved work, and generated reports as dated evidence. Update cross-links instead of copying another live status table. A documented limitation stays unresolved until implementation and acceptance evidence support changing it.

`docs/sdk/boundary.toml` is the editable principal inventory; `docs/sdk/boundary.md` is rendered output. `docs/hangar-stores-build.md` and its JSON companion are generated build evidence; regenerate them with their owning tool rather than hand-editing rows. [resources.md](resources.md) lists the other owners.

The aircraft design skill is maintained at `.agents/skills/aircraft-design/SKILL.md`. `.claude/skills/aircraft-design/SKILL.md` is a byte-identical synchronized copy for Claude discovery. Change the maintenance source, sync the copy in the same change, and verify:

```powershell
Get-FileHash .agents/skills/aircraft-design/SKILL.md, .claude/skills/aircraft-design/SKILL.md -Algorithm SHA256
git diff --no-index .agents/skills/aircraft-design/SKILL.md .claude/skills/aircraft-design/SKILL.md
```

The hashes must match and the no-index diff must be empty (exit 0). Retain both frontmatter fields and the shared provenance note. This synchronization preserves the existing workflow; it does not authorize generating aircraft or change project acceptance criteria.
