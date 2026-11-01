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
