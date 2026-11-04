# Native countermeasure receiver coupling

The native fixture `Every configured AAM receives an actual native
countermeasure in its physical detector` covers every authored AAM card: eight
active radar, four semi-active radar, five imaging infrared, and seven
non-imaging infrared models. This is an engineering receiver-coupling gate.
It does not establish deception probability, countermeasure effectiveness,
equipment calibration, or a required hit or miss.

The original three native cadence scenes, their 40-second post-release limit,
tracking criteria, and their `summary-skip{1,2,4}.csv` and per-model trace paths
remain unchanged. The separate countermeasure scene uses frameSkip 4, 1/120 s
native steps, seed 17, one worker, the same aircraft/weapon geometry and native
control/sensing/release paths. Before acquisition it explicitly operates the
target's native dispenser. At or after 1.5 seconds of weapon release age it
requests exactly one native cartridge: a flare for infrared seekers and chaff
for radio seekers. The final native inventory must decrease by exactly one.
The scene preserves native tracking, miss distance, and all native end flags,
including `in_flight` at the frozen 40-second limit.

## Receiver experiment

Test-only receiver probes call the existing native detector kernels. They do
not reproduce thermal radiometry, noise, pulse processing, CFAR, association,
or tracking in a client. Each probe replays the actual policy, exposure epoch,
detector pointing, seed, and complete preceding exposure history. A copy of
the native receiver immediately before an exposure provides the paired
counterfactual. No probe mutates the actual provider or its random stream.
The full probe must reproduce the actual detector output exactly. Additional
probing stops once the required positive channel evidence is obtained.

The referee fixture identifies the counterfactual contribution by comparing
the actual weapon's physical position and an actual target-owned native
decoy's physical range and direction with the supplied detector scene. Range
must agree within 0.01 m and the unit direction within 1e-8. It rejects
ambiguous matches and ignores decoys younger than 0.1 s, when the flare and
aircraft CG may coincide. It also checks that the frame's physical origin is
the released weapon, preventing an on-rail receiver from fulfilling the gate.
The source join uses the native body's physical origin and matching current
World/release epochs. `SeekerFrameV2.position` is intentionally the onboard
navigation estimate and cannot replace that physical origin in the referee
experiment. The original range/direction tolerances remain unchanged.
No decoy ID, aircraft identity, or referee association is passed to measured
association. Referee attribution metadata retains its existing native role.

For thermal receivers the counterfactual removes only the geometrically
matched current flare patch. Both receivers retain the same preceding
detector-response history. The full probe's expected electron array must
exactly equal the actual image. A positive sum and maximum of current-flare
expected-charge differences proves detector coupling before ADC/full-well
clipping. The receipt records saturation and measured-image differences,
without requiring a measured difference when both images saturate. MICA IR
requires independent positive proofs in both physical bands, with the native
dual-band channel seed salts and each channel's own detector policy.

For radio receivers the probes retain actual body axes, pointing, exposure
history, illumination observations, and—for semi-active receivers—the actual
transmitter waveform and illumination clock. Deleting a radio echo would
change native random phase/fluctuation consumption. The control therefore
retains the same geometric echo and suppresses only its RCS by 1e-200. Both
native energy values must remain positive, with the declared suppression
ratio. This retains the native echo's random-draw slot and canonical geometric
ordering; its residual field amplitude is approximately 1e-100 of the actual
field. The control is a numerical source-suppression experiment, not a claim
of literally zero RF energy. It must change the retained first-interval native
range/Doppler power cells. That interval's noise and phase draws precede
outcome-dependent peak-noise draws that can affect later intervals. Exact
full-probe reproduction binds the experiment to the actual receiver. Missing
cell contribution fails honestly; inventory or an isolated RCS value cannot
substitute for this proof.

## Raw output and admission

Set `FSIM_SENSOR_NATIVE_DIR` to the normal native sensor evidence folder and
execute the new case through the existing UCRT64 native test entrypoint. The
root execution owner holds the exclusive native configure/build/test slot.
This feature does not introduce an alternative SDK launcher or increase the
existing cadence-test timeout. Its distinct artifacts are:

- `countermeasure-summary-skip4.csv`: exactly 24 raw model rows, no header;
  model, provider, frame skip, cartridge family, before/after inventory,
  release/request epochs, proof count/flags, actual sample/decision/tracking
  counts, native end flag, and optional native miss distance.
- `<card>-countermeasure-skip4.csv`: a header and exactly one physical-channel
  proof, or two for MICA IR; actual epochs/range, matched contribution count,
  full-probe replay error, thermal charge/saturation/image differences or RF
  retained-cell/native-energy differences, and actual illumination flags.

The strict admission command is:

```powershell
python tools/validation/weapon_quality/countermeasures.py `
  --coverage <current-coverage.json> `
  --source-epoch <countermeasure-source-epoch.json> `
  --output <fresh-countermeasure-receivers.json>
```

The source-epoch document uses schema
`fsim.sim-m03.countermeasure-source-epoch.v1`, `main_head`,
`coverage_sha256`, and `sources`, an array of repository-relative
`{path,sha256}` bindings. It must cover the test, validator and negative
controls; `WorldEngagements.cpp`; `Dispenser.cpp`; `EngineeringSeekerV3.cpp`;
`ThermalImageReceiverV1.cpp`; `RadioReceiverV1.cpp`; `Signal.cpp`; and all
24 exact current store cards. Additional source bindings are permitted and
verified. Coverage card/provider hashes must agree with those actual bytes.
The validator's exported `SOURCE_PATHS` set is the authoritative required
source list; it additionally includes the receiver interfaces, kernel headers,
temporal receiver and receiver-configuration sources and headers. Generate the
epoch from that set plus all coverage card paths so the header epoch cannot be
accidentally omitted.

Its `presets` array must contain exactly `ucrt64-headless` and
`ucrt64-release`. Each row contains `preset`, `binary`, nonempty `runtime`,
`summary`, `traces`, and `test_report`. Each artifact uses `{path,sha256}`;
each trace additionally has `card`. The binary must be the actual
`fsim_weapon_sensor_native_test.exe` under that preset's build directory. Runtime
bindings must include the staged SDK and JSBSim DLLs. The test report is an
actual Catch2 XML report, or the full CTest JUnit containing the exact new
`sensor_native.` case, no failure/skip, and the actual nonzero Catch assertion
summary. A synthetic report is not acceptance evidence.

Admission verifies exact 24-by-2 model/preset coverage, native family/provider
binding, one consumed cartridge, each required physical channel, epochs,
actual detector replay, positive kernel contribution, illumination clocks,
and source/artifact bytes before and after parsing. It emits schema
`fsim.sim-m03.countermeasure-receivers.v1` with 48 joined model receipts and
preserved native outcomes. This receipt must be joined by the full campaign
and report; standalone admission is not full M03 acceptance.

Pure validator controls run without SDK or native execution:

```powershell
python -m unittest discover -s tools/validation/weapon_quality -p test_countermeasures.py -v
```

Their synthetic fixtures establish fail-closed admission behavior only. They
provide no native receiver, runtime, installed-consumer, or Viewer evidence.
The native cartridge's motion and lifetime remain separately covered by
`test_countermeasure_core.cpp`.
