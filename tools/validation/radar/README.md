# SIM-M01 engineering verification

Run from the repository root on main, using the existing installed UCRT64
dependencies and CPython. This command configures/builds both existing presets,
runs every test in the common dependency selector, installs a new C++/C99/Python
SDK, exercises every declared air-radar fit twice, measures native throughput,
uses the normal SDK launcher and joins the actual receipts:

```powershell
$env:TMP = 'D:/ENV/tmp'
$env:TEMP = 'D:/ENV/tmp'
$env:TORCH_EXTENSIONS_DIR = 'D:/ENV/tmp'
& D:/ENV/python/3.13/python.exe tools/validation/radar/campaign.py `
    --output build/architecture-reviews/sim-m01-reproduction `
    --deps-prefix build/deps-ucrt64/install --jobs 8
```

The output directory must be new. Hold the exclusive native configure/build/test
slot for the whole command. A checkout without dependencies first follows
`docs/development.md`; this verifier does not install a different toolchain.
An ordinary clean source export can point `--deps-prefix` at the original
installed dependency prefix, and optionally pass `--catch2-source` for the
existing Catch2 source. No Git branch or worktree is needed. The published
campaign retains the exact arguments used.

`campaign.json` records each actual command, exit code, raw log and elapsed time.
`*-regression.json` joins the discovered inventory and unmodified JUnit output.
`installed.json` records fresh installation and standalone consumers, source and
product hashes, and DLLs loaded inside a native consumer process. The installed
fleet receipt records loaded DLLs in that very sampling process. `normal.json`
identifies the modules loaded through `tools/launch/fsim.ps1`, and
`sim-m01-delivery.json` is produced only after the joined gates pass.

## Common functional population

`audit.py` derives the airframe roster from the maintained Hangar design roster
and joins authored fits to every air-radar card. Its success is inventory
consistency only. An airframe without a declared air radar is explicitly listed;
it is not counted as exercised radar hardware or verified equipment absence.

`contract.py` is the single definition of applicable stages and the dependency
test selector. Every fighter fit runs detection search/fusion, signal volume
search, STT, TWS, fresh cued acquisition, signal and detection velocity search,
power-off, coasting, sample-age loss and bounded retention expiry. AEW fits run
their detection/signal surveillance, an actual rear-hemisphere signal return and explicit unsupported-engagement refusal,
then the same lifecycle stages. There are no model-name exemptions.

Each entire sequence is repeated with one and four workers at the same seed.
The fingerprint includes actual radar status, local track reports and fused
means/covariances/provenance; host timings are excluded. Velocity-only checks
require the exact pre-command update timestamps and reject new Cartesian tracks.
This avoids confusing floating-point clock equality with an actual update.

The performance gate uses the same eight-aircraft, four-worker scene with radar
off/on, 120 warmup and 600 measured steps each. Both cases must sustain at least
1x; live native radar must create fused knowledge, and no case may refuse native
measurement inputs. Mean/p99 and setup cost are retained, not inferred from the
simulation's configured speed.

## Theory and independent references

| Component | Native mechanism / reference | Verification and disclosed limit |
| --- | --- | --- |
| Antenna and radiometry | Authored aperture, taper, scan loss, radar equation and Swerling classes | Existing analytic/radar dependency cases; hardware parameters retain their individual published/derived/assumed labels |
| Propagation and clutter | Existing native radio path, weather/terrain, look-down clutter/notches | Existing path/radar checks; no copied fighter parameters or alternate client simulation |
| Receiver / engagement modes | Existing Doppler samples, monopulse, thresholds, multi-PRF M-of-N chain, native scheduling and resource limits | Signal/RF reference tests plus every declared fit; not a reconstructed classified processor |
| All-round electronic aperture | Explicit generic fixed-face body-to-aperture transform, used consistently for detection, signal echoes and clutter | Four-quadrant detection/signal fixtures plus installed rear-hemisphere AEW returns; MESA uses four equivalent look directions with assumed common face gains and uniform sectors, not four claimed physical panels |
| Ambiguity | Raw receiver covariance and all feasible temporal alias/prior associations | Competing aliases/shared returns remain unresolved; velocity-only folded range cannot update Cartesian tracks |
| Thermal/residual errors | SNR law and explicit floors, fixed/radome bias, stationary OU drift, finite-variance Student-t4 geometry-scaled glint, local-plane direct/image centroid | Analytic/statistical mechanism tests and floor sensitivity; optional residual coefficients retain identity defaults without equipment data; centroid is not a complete two-path receiver |
| Tracking | Existing Stone Soup-derived nonlinear UKF/GNN/JPDA; optional two-mode continuous-time IMM | Original Stone Soup fixtures and independent FilterPy 1.4.5 posteriors; original tolerances retained; cards keep legacy CV defaults unless explicitly authored |
| Fusion | Unknown-correlation covariance intersection on a fixed 33-weight logdet grid | Twelve independent Stone Soup 1.9.1 reducer cases including equal covariances, endpoints and anisotropic cross-covariance; grid is a bounded approximation to continuous weight optimization |
| Link covariance / latency | Conservative six-axis PSD upper bound from wire RMS summaries, propagated from sample epoch | Actual delivered native links; no independent-noise assumption or truth association; summary transport loses anisotropy |
| Lifetime / capacity | Sample-age validity, live/coasting/lost states, bounded terminal tail, recent provenance and explicit refusals | Failed-before/passed-after boundary fixtures; expired contributor history is reclaimed only when a new source needs its bounded slot |
| Public interfaces | Native C++/C99/Python V1 getters and own-Agent permissions | Installed consumers, exact repeated/count/short-buffer reads, C ABI and Python binding checks; no default actor callable-list expansion |

The upstream method implementations are
[Stone Soup covariance intersection](https://github.com/dstl/Stone-Soup/blob/v1.9.1/stonesoup/mixturereducer/gaussianmixture.py)
and [FilterPy IMM](https://github.com/rlabbe/filterpy/blob/1.4.5/filterpy/kalman/IMM.py).
They remain offline references, with original source hashes and versions in the
fixtures. No new runtime dependency is introduced. To regenerate the additional
CI fixture into a new file, use `tools/validation/track/fusion_reference.py` with
Stone Soup 1.9.1. Existing IMM and FERS reference roles remain unchanged.

MESA's actual 360-degree coverage is confirmed by its
[manufacturer](https://www.northropgrumman.com/what-we-do/mission-solutions/radars/multi-role-electronically-scanned-array-mesa-surveillance-radar).
The manufacturer's [2002 antenna release](https://www.globenewswire.com/news-release/2002/11/06/291813/189/en/northrop-grumman-rolls-out-first-mesa-radar-antenna-for-australia-s-wedgetail-aew-c-aircraft.html)
identifies side-emitting arrays and a top-hat end-fire array. The numerical
equivalent-face arrangement is explicitly assumed; neither source admits its
directional gains, actual sector transitions or tracking accuracy. The existing
`rotation_rpm` is a legacy all-round revisit clock for this fixed array.

Under the 2026-11-02 user clarification, unavailable real-equipment accuracy
calibration is a separate uncertainty ledger. It does not waive a component,
failed native function, missing fit stage or provenance defect. Method agreement
is not equipment validation. The selector covers M01 and its dependencies;
neither whole-project full regression nor new Viewer presentation acceptance is
claimed. SIM-M02's separate rotor deferrals are not used to skip an M01 fit.
