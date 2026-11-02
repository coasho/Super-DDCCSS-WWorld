# SIM-M01 engineering closure

Assignment: the user assigns the complete remaining SIM-M01 package, initial
push and final commit/push. The 2026-11-02 clarification adopts the SIM-M02
acceptance rule: engineering components, theory, complete declared-fit functions
and reproducible installed delivery must close; unavailable equipment accuracy
calibration is a separate uncertainty ledger. It cannot waive a missing function.

Base main HEAD after initial push: `b1e8a208da7fa344b4b9bee4324944678c293b94`.
No branch or worktree. The sole native build/test slot is acquired for this
window; no competing CMake/Ninja/CTest/Viewer process was found. Tools and caches
use D:/ENV; TMP, TEMP and TORCH_EXTENSIONS_DIR use D:/ENV/tmp.

Write allowlist and symbols:

- `src/simulation/mission_systems/track/Fusion.h`: `advance`, input validation
  and association/lifetime/capacity boundaries; `tests/simulation/test_track.cpp`.
- `tools/validation/radar/{audit,report,fleet,installed,delivery,performance}.py`,
  new reproducible campaign/checker and README in that directory, installed
  consumer fixtures and independent numerical fixtures under track validation.
- The SIM-M01 sections of `docs/status.md`, simulation `plan.md`,
  `docs/sdk/{sim-m01-completion,mission-systems}.md`, new dated M01 data receipts
  and this plan. Documentation reconciliation also owns only the B-66/B-68 rows
  of `docs/backlog.md`. Preserve mixed-owner guide/backlog changes with index-only
  hunks.
- Native radar/IMM files and their dedicated tests only if an observed defect
  requires repair. Public layouts, enum numbers, actor visibility, default
  permissions and existing physical thresholds remain unchanged.

Other dirty files, other owners' implementations and excluded directories are
outside scope. Preserve their bytes. Use a clean ordinary HEAD-plus-owned-file
export for the complete M01 dependency regression; do not absorb unrelated Viewer
changes or claim a whole-project regression from this feature selection.

1. Commit/push the reviewed existing M01 baseline; rerun its selected checks.
2. Reproduce boundary failures before repair. Check CI against an independent
   reference and retain unknown-correlation conservatism and invalid-input
   refusal. Keep legacy defaults and original numerical tolerances.
3. Exercise every declared air-radar fit: detection and signal search, applicable
   engagement modes, high-PRF ambiguity, power-off, coasting, loss, expiry,
   ownership and reproducibility across worker scheduling. Independently inventory
   unfitted airframes without claiming verified equipment absence.
4. Make acceptance join actual functional, method, regression, installation and
   provenance receipts; inventory or the calibration policy alone cannot pass it.
5. Freeze source, replay a clean export, build both existing presets serially,
   execute every case in the frozen common M01/RF/interface dependency selector,
   with discovered nonzero populations, bounded timeouts and
   --no-tests=error, install C++/C99/Python consumers and use the normal launcher.
   Record scoped rotor deferrals inherited from M02 separately from failures.
6. Report exact commands/counts/timing, uncertainties, current hashes and native
   provenance. Commit only reviewed owned changes, inspect staged diff, push main
   and release write/build slots. No new Viewer acceptance is inferred from SDK
   execution; visual work, if needed, follows fullscreen/full-effects/1x rules.

Method sources reviewed: Stone Soup's covariance-intersection example and
FilterPy's IMM implementation. Existing Stone Soup/FilterPy/FERS reference roles
and licenses remain unchanged; no new simulation dependency is introduced.

## All-round aperture correction

Post-campaign coverage review found that the MESA card declares an electronic
all-round scan while its gain is still evaluated in one forward-facing aperture.
Extend the owned paths to `radar/Radar.h::Spec`, the shared radar gain transform
and `assets/payloads/mesa.json`. Add an explicit, common fixed-face aperture
configuration with identity defaults; keep source-backed side/end-fire hardware
distinctions separate from assumed equivalent face gains/sectors. MESA's legacy
revisit clock is not a physically rotating antenna. The manufacturer confirms
fixed side-emitting and top-hat end-fire arrays and 360-degree coverage; it does
not supply admitted directional gain calibration. Test every body quadrant at
both detection and signal levels, away from the existing zero-Doppler rejection.
Retain the first completed campaign as superseded evidence and rerun both normal
presets, installation, complete fit sampling and the normal entry on the final
source. No acceptance predicate or original reference tolerance is relaxed.
