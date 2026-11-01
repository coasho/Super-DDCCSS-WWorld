# Bounded native flight recorder

Build `fsim-native-flight-probe` with the existing UCRT64 launcher. The recorder
creates one SDK World and drives its actual flight model. It implements no
flight, contact, guidance or acceptance physics.

```powershell
$env:TMP = $env:TEMP = $env:TORCH_EXTENSIONS_DIR = 'D:/ENV/tmp'
& ./tools/launch/fsim.ps1 --preset ucrt64-release build --target fsim-native-flight-probe
& ./build/ucrt64-release/bin/fsim-native-flight-probe.exe --model MODEL --output D:/ENV/tmp/flight.csv --height 3000 --forward-ms 40 --down-ms 0 --settle-s 2 --first-s 16 --second-s 15 --pitch-first 0.6 --pitch-second -0.8 --collective 0 --engines-running 1
```

`--hsa-speed SPEED` selects native HSA after settling instead of raw actuator
commands; its default `-1` preserves raw controls. Times must be nonnegative,
their sum is at most 600 simulated seconds, and wall time is bounded at 1200
seconds. Native terminal state ends recording. An unavailable property has an
empty CSV cell, never an invented zero. Exit zero means recording completed;
it does not mean the recorded physics passed any acceptance test.

The complete engineering workflow, status meanings, full static table checks,
native sampling and independent regression requirements are in
`docs/hangar-system.md`. Diagnostic excitation cannot replace a frozen test
scenario without explicit acceptance-policy authorization.

## Matched-state AAM pressure observation

The approved qbar fixture correction reads the first evaluated native force
sample, then still executes the complete original public step. It preserves
initial conditions, commands and the `1e-8` comparison bound. This changes the
observation boundary: the old end-step comparison is retained as a diagnostic,
not relabeled as passing. No aerodynamic coefficient or integration interval is
changed. Different pressure plants may legitimately have different final states.

```powershell
& ./tools/launch/fsim.ps1 --preset ucrt64-release build --target fsim-native-flight-probe fsim_missile_control_native_test
& D:/ENV/python/3.13/python.exe -B tools/validation/aerodynamics/aam_qbar_observation.py --preset ucrt64-release --output build/architecture-reviews/aam-qbar-resolution-20261101/replay-release
```

The generic roster tool runs all 24 AAMs with one rule, bounded at 60 seconds
per process. Each CSV retains every native force evaluation and the terminal
values. Each plant is also run without observation; all 104 named public-state
values and 20 final properties must be bitwise identical. Six normalized native
fin loads are compared at equal Mach, alpha, beta and actual fin positions.
Reports bind the executable, actually loaded JSBSim DLL, current source files
and generated model XML by SHA-256. This is bounded native sampling, not a
full-envelope validation or measurement calibration. The private observer is
model-scoped, thread-local and restored on scope exit. Its read-only use is a
trusted callback contract, not a public SDK permission mechanism.
