# Compatibility baseline before the mission family

Dated record, 2026-10-12. Generic M0 of the
[MA training plan](../superpowers/plans/2026-10-12-ma-system-training-readiness.md) captured what existing consumers
of the SDK rely on, before any mission-episode code exists, so that every later package can be held to it: old
headers, old binaries and old Python must keep working against the new library.

| | |
| --- | --- |
| Checkout | main `e1945b98732586a31b8a8d05c276fc34f4a2d737` (delivered baseline `baf476f841970f1f3c90b8de0bf630e8a5478738` plus one documentation commit; no tracked source, header, test, CMake, asset or third-party file differed from HEAD) |
| Artifacts | `D:/ENV/tmp/ma-training-baseline` (outside the repository; schema `fsim.ma-baseline.v1`) |
| Manifest | `MANIFEST.json`, SHA-256 `205a724702b04091be8a72def93e214bf974c5b1bb7c8f6060ade8dbd81fcf46`, captured `2026-10-12T22:12:36Z`, 66 artifacts listed, 0 capture failures |
| Tool | [tools/validation/ma_contract/baseline.py](../../tools/validation/ma_contract/baseline.py): `capture`, `verify`, `run` |
| Toolchain | GCC 16.2.0 (MSYS2 UCRT64), CMake 4.4.3, Ninja 1.13.2, GNU objdump 2.47, CPython 3.13.12 |
| Builds read | `build/ucrt64-headless` and `build/ucrt64-release`, both up to date at that checkout before the capture |

The artifacts live under `D:/ENV/tmp`, which is not durable storage. The manifest's hash above identifies them; if
the directory is lost, `capture` at the same checkout reproduces every listed kind of artifact, and binaries then
differ only by their link timestamps.

## What was captured

| Kind | Where under the artifact directory | Result |
| --- | --- | --- |
| HEAD, branch, tree, status, dirty diff, every tracked blob id | `git/` | Dirty at capture: five pre-existing documentation files and this phase's own uncommitted pages, fixtures and tools. No tracked build input differed from HEAD |
| Public interface archive (`include/`, `python/fsim/`) at HEAD | `git/public-interface-e1945b9.zip` | SHA-256 `511841bcb58c44a78e8a44a4bc24c17f6a8617c7cf10c24e8b93eb13850a0456`. The supplied archive was taken at `c2641e9`; one comment in `MissionSystems.h` changed since |
| Protected runtime inputs (aircraft and store models, cards, JSBSim data, control sources) | `inputs/` | 2,357 files re-hashed from the working tree; all 2,357 equal the supplied manifest |
| Native exports | `exports/*.exports.txt`, `*.demangled.txt`, `*.vtables.txt` | `libfsim.dll`: 1,338 names (466 C, 872 C++; 68 vtable and typeinfo symbols), identical between the headless and release builds and equal to the supplied dump. `libfsim_vision.dll`: 57 (25 C, 32 C++), equal to the supplied dump. `_native.pyd` and `_vision.pyd`: their module initialiser only |
| Python surface | `exports/python-calls.*.txt`, `python-inventory.*.json` | Headless: 15 modules, 620 public names, 565 principal-classified calls. Release (with `fsim.vision`): 16, 629 and 580. The inventory records each call's signature, each enumeration's numbers and each record's fields |
| Test inventory | `tests/ctest-*.json`, `*.names.txt` | `ucrt64-headless`: 599 tests. `ucrt64-release`: 653 tests |
| C ABI layout | `types/c-abi-layout.json` | 120 structs with every member's offset, 81 enumerations with 415 enumerator values, from the two C headers compiled as strict C99 (`-pedantic-errors -Werror`) |
| C++ layout | `types/cpp-layout.json` | 267 `fsim::` types with size, bases and data-member offsets; 107 enumerations with their values |
| Vtables | `types/public-vtables.json`, `public-class-dump.txt` | 50 `fsim::` vtables with every slot, among them the plug-in interfaces `fsim::Task`, `ObservationBuilder`, `ActionMapper`, `AgentView`, `control::Controller`, `control::Behavior`, `effects::Effect`, `comm::Medium`, `Protocol`, `Transport`, `Codec` and `sim::GroundProvider` |
| Installed SDK prefix | `consumers/sdk-prefix` with `sdk-binaries.sha256`, `sdk-headers.sha256`, `sdk-runtime-data.sha256` | `cmake --install build/ucrt64-headless --component sdk`: 5 DLLs, 35 headers, 1,229 runtime data files and 33 installed documents, each by hash. The install copies `docs/sdk` from the working tree, so the capture put the two pages this phase edited back to HEAD's bytes and took out the three it added: the prefix is HEAD's package |
| Installed Python package | `consumers/python-dist` with `python-package.sha256` | `--component python`: the package, its DLLs and data, and its wheel |

## Frozen consumers

Each was built only against the installed prefix above and run with `PATH` reduced to that prefix's `bin` and the
Windows system directories, with `FSIM_AIRCRAFT_PATH` naming the prefix's own aircraft data.

| Consumer | Source at the baseline | Binary | Imports of `libfsim.dll` | Run at capture |
| --- | --- | --- | --- | --- |
| C++ | `tests/compatibility/package_consumer/main.cpp`, through `find_package(fsim)` | `sdk-prefix/bin/consumer.exe` | 15 names | `smoke`: exit 0; it verifies itself that both DLLs were loaded from the prefix |
| C99 | `tests/compatibility/test_c_abi.c`, `gcc -std=c99` against the installed headers and import library | `sdk-prefix/bin/c_abi_consumer.exe` | 388 names | exit 0 (`c abi ok`) |
| Python | The installed package and its stable-ABI module, with frozen copies of `examples/simulation/python/agent.py`, `links.py` and `c2.py` | `python-dist/fsim/_native.pyd` | 419 names | Three scripts, exit 0 each; each log starts with the paths the package, the module and both DLLs were really loaded from |

Every imported name is exported by the baseline library (`consumers/*.imports.txt` list them). The supplied frozen
C++ consumer under `D:/ENV/tmp/viewer-strike-delivery-20261012/compatibility` (31 binaries, 1,281 data hashes) was
re-hashed against its own manifest without a mismatch; its `libfsim.dll` is the release build's. It was not run
again: only its `bin` directory was supplied.

The Python examples cover an Agent's own-ship reports and commands, measured tracks and designation, DMS sends,
native publications, link tracks, a C2 node, roles, direct commands, overrides and an unauthorised sender. They are
smoke-level consumers, not a behavioural regression: the 599-test suite is that.

## Running the old binaries on a new library

    python tools/validation/ma_contract/baseline.py verify --out D:/ENV/tmp/ma-training-baseline
    python tools/validation/ma_contract/baseline.py run    --out D:/ENV/tmp/ma-training-baseline
    python tools/validation/ma_contract/baseline.py run    --out D:/ENV/tmp/ma-training-baseline --sdk-prefix <installed SDK of the new build>

`verify` re-hashes every listed artifact and both installed trees. `run` repeats the consumers as captured. With
`--sdk-prefix` the frozen executables are copied alone into a scratch directory and the frozen Python package is
copied without its DLLs and data, so the loader takes `libfsim.dll` from the other prefix and the library reads that
prefix's data; each log shows the library's real path. This is the check each later package repeats before it
commits a change to a public header, an adapter or the library.

On 2026-10-12, at the baseline: `verify` reported no changed artifact; `run` as captured exited 0 for all five
consumers; and `run --sdk-prefix "build/ucrt64-headless/package-consumer/installed SDK"` - the same library
installed separately by the `package_consumer` gate - exited 0 for all five, each loading `libfsim.dll` from that
prefix. That last run exercises the mechanism only: the library was the same code.

## Baseline test status

`ctest --preset ucrt64-headless --no-tests=error --output-on-failure`, started 2026-10-12T21:49:32Z at the checkout
above, before any M0 change to a build input: **599 of 599 passed, 0 failed, 0 skipped, 948.56 s wall time, exit 0**
(16 jobs; raw log `logs/03-headless-full-suite.log`, SHA-256
`11997cb39f2bc5bfbbce3a068e615668b808b5ca5d3953ebbbf553c25148633a`). There is no baseline failure to carry into the
MA packages. The release preset's suite, the GPU-dependent tests and the Viewer were not run for this record.

## Limits

- Layouts and vtables are GCC 16.2 on UCRT64's, the one maintained toolchain. They say nothing about another
  compiler.
- Only 8 base interfaces carry virtual-slot indices in the debug-information view (`cpp-layout.json`); the class
  dump (`public-vtables.json`) is the complete vtable record.
- Inline functions, templates and default arguments in public headers are compatibility surface that no export or
  layout table shows. The header archive and the frozen binaries stand for them.
- The C99 and Python consumer runs prove which library and package were loaded. They do not poison source-tree data
  the way the `package_consumer` gate does, so they are not by themselves proof that no source fallback was used.
- The capture does not include the release suite, renderer, vision device tests or any Viewer evidence. The
  [Viewer validation record](viewer-validation.md) remains the existing diagnostic evidence, unchanged.
- This record is a baseline. It is not acceptance of any mission-family interface: none exists yet
  ([mission training contract](mission-training.md)).
