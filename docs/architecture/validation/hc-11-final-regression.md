# HC-11 final regression

Status: both required full suites ran once on one frozen, fingerprinted snapshot, and both passed (2026-10-17).
Awaiting independent acceptance. HC-07's Viewer, manual-control, performance and joint gates stay pending, so
final-program acceptance is not claimed.

## 1. Preconditions

- **Review findings closed:** the [ARCH-10 review](arch-10-review-2026-10-17.md#9-hc-38-to-hc-43-correction-acceptance-at-f1bb879)
  closes HC-38 to HC-43 and HC-20 at `f1bb879`. No item in the [carryover register](handoff-carryover-register.md)
  is an open code defect. The open items are:
  - an evidence gap of the MA feature package: HC-02;
  - documented limits, arrangements or scheduled maintenance: HC-03 to HC-05, HC-17, HC-26, HC-27, HC-34, HC-36 and HC-37;
  - feature prerequisites: HC-06 and HC-07;
  - record consistency: HC-09;
  - a workspace observation: HC-18;
  - a human disposition: HC-35.
- **Related fast checks:** the selector's plan for the 10 paths changed since `197a059` passed on both presets:
  `ucrt64-headless` 29/29 (24.6 s) and `ucrt64-release` 35/35 (26.4 s).
- **Builds:** both presets were built incrementally before the freeze. Nothing compiled; only data, runtime DLLs and the
  Python package were staged. Ninja found every object current, so the objects date from earlier builds of unchanged
  inputs.
- **Slot:** no build or test job of this repository was running. One serial job ran at a time, and no other build,
  test or GUI job ran during the suites.

## 2. The frozen snapshot

| Item | Value |
| --- | --- |
| HEAD and tree | `f1bb879313fb3157515f7a8f916bf6784a2802cf`, tree `61e8c751dc2964f901254e9b4fddfe21f203bc0f` |
| Source snapshot digest (SHA-256 over every file's path and hash) | `e26241d67a879d4ee5ef417df9281dd34b0156477193629d3e44230f6d2b301d` |
| Files | 4,559 tracked and 28 untracked. 58 differ from HEAD: 30 tracked edits by other owners and the reviewer, and the 28 planning inputs. No `.codex` path was listed or read |
| `ucrt64-headless` | 717 tests (name digest `cd7a6fc9b694ec3c`), 17 staged executables, DLLs and extensions |
| `ucrt64-release` | 780 tests (name digest `fa9365ee533a5325`), 72 staged executables, DLLs and extensions |
| Staged data (`share/flightsim`, both presets) | 234 files, digest `603ea0553e18673f` |
| Toolchain | g++ 16.2.0 (MSYS2 UCRT64), CMake 4.4.3, Python 3.13.12 for the checks; NVIDIA GeForce RTX 5070 Ti, driver 596.36 |

[hc-11-final-regression-fingerprint.json](hc-11-final-regression-fingerprint.json) is the capture, committed byte for
byte (SHA-256 `2f90b85444c4e89a`). It holds every differing path's hash, each artifact's hash, the cache options and
the inventories by label.

The capture tool is a new one, not the ARCH-10 provenance helper: `D:/ENV/tmp/hc11/fingerprint.py`, SHA-256
`0f8f537cb3835456`. It meets the review's condition for any later capture:
- its directory walks prune `.codex`, `.git` and `__pycache__` before descending, and Git lists untracked files with
  `--exclude=.codex`;
- `verify` recomputes the whole set, so a missing, extra or changed file fails, as does a changed digest, artifact, data
  tree, inventory or cache option. Before the freeze, a trial rejected a changed digest, a dropped path and a changed
  artifact.

## 3. The full suites

Each ran once, `--no-tests=error --output-on-failure -j 16 --timeout 600 --output-junit`, the preset's own jobs:

| Preset | Result | Wall time |
| --- | --- | --- |
| `ucrt64-headless` | **717/717 passed**; none failed, skipped or not run | 1,123.4 s (08:06:34 to 08:25:18, -06:00) |
| `ucrt64-release` | **780/780 passed**; none failed, skipped or not run | 1,137.5 s (08:25:42 to 08:44:40, -06:00) |

Further details:
- **Labels, each on both presets unless noted:**
  - `python_sdk` 67;
  - `ma-training` 38, `hangar` 41, `stress` 10, `package` 4 and `launcher` 1;
  - `architecture` 27 headless and 33 Release;
  - `render` 63, on Release only.
- **Renderer tests:** `viewer_screenshot` ran and passed (JUnit status `run`); it was not skipped through its skip code.
  The demo publisher's process tests and the Vision tests passed too.
- **Timeouts:** 534 headless and 587 Release tests have no timeout of their own, and `--timeout 600` bounds them. In
  these runs the slowest of them took 168.1 s on headless (the adapter conformance case) and 289.6 s on Release (the
  vapour cone's bounds). The other tests keep their own timeouts, up to 3,600 s.
- **Slowest tests:** `hangar test_vulnerability.Dispensers` (1,041 s headless, 1,050 s Release, within its own
  timeout), then the same module's `Components` and three fleet shards.
- **Snapshot:** `verify` after each run reported all match: the same HEAD, snapshot digest, artifacts, data,
  inventories and cache options.

| Evidence (`D:/ENV/tmp/hc11/`) | SHA-256 |
| --- | --- |
| `full-headless.log` | `707518d701a9a43e` |
| `full-headless-junit.xml` | `34fd721874b2736a` |
| `full-release.log` | `1577bbc1307fd4e7` |
| `full-release-junit.xml` | `3648bf1a2d06bfc5` |
| `verify-after-headless.log`, `verify-after-release.log` (identical) | `e8297b12dafd2711` |
| `fast-run.log` and `fast-logs/` | `c2e91f6b38d7295b` (the run log) |
| `results.py` (the JUnit summary) | `95f86bff4e520c25` |

## 4. What this establishes

It establishes HC-11's final regression. Every test that the two required presets register passed once on one
unchanged snapshot, which covers:
- the native, C ABI and Python suites;
- the installed-consumer and launcher gates;
- the contention cases;
- the renderer tests.

It does not establish:
- **HC-07:** the complete fullscreen Viewer scenarios, manual pause, step, time-factor and quit, the 4K performance
  gate, and installed public-endpoint joint acceptance;
- **HC-06:** full M5-M8 combat criteria;
- physical-model quality;
- **`ucrt64-debug`:** not a required preset, and not run;
- **a clean rebuild:** the incremental build compiled nothing and was not part of this assignment.

## 5. After the runs

The second verification passed before any edit. Only documentation changed after it:
- **Committed:** this record, the fingerprint copy, section 13 of the [ARCH-10 record](arch-10-final-acceptance.md)
  and a status entry.
- **Uncommitted planning pages:** the carryover register, the architecture index, the roadmap and the acceptance page.

The build does not read any of them, and the fingerprint describes the snapshot before them.
