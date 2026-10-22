# Resource binding checks

`check_rcs.py` compares each existing `aircraft/<type>/rcs.bin` with its actual
`<type>.glb`. It reads the pinned FSRCS 1 JSON header using the layout in
`src/simulation/mission_systems/rcs/Table.cpp` and the legacy native byte hash
in `tools/models/rcs/main.cpp`. SHA256 of both files is included in the result.

A matching original bake is accepted. If the original model differs, only a
matching native `verified` record is accepted, including model name, bands,
nonempty comparisons and the unchanged max/p99/p95 noise-relative criteria
recorded by `flightsim-rcs --verify`. The original bake is never relabelled as a
new computation. Stale, absent or failed provenance is rejected explicitly.
The tool invalidates an earlier passing output before starting another audit.

```powershell
.\fsim.cmd --preset ucrt64-headless python tools/validation/resources/check_rcs.py --aircraft-root aircraft --output build/rcs-binding.json
.\fsim.cmd --preset ucrt64-headless test -R resource_rcs_binding --no-tests=error
```

The synthetic tests exercise binding and malformed-data refusals. Their byte
strings and metrics are fixtures, not geometry or equipment calibration.
An always-accepting negative control fails 11 assertions across eight tests.
Two additional process-boundary regressions cover CLI output/exit status and
replacement of an older passing result when the current asset is invalid.
Before regeneration the real B-52H asset fails both this binding check and the
existing native electromagnetic comparison; those are separate kinds of
evidence, and neither failure is removed by changing a tolerance.

This gate establishes file correspondence with a bake or recorded verification.
It inspects header binding; native readers retain responsibility for the table
payload's validity.
It does not calculate RCS, certify a full angular envelope, validate real
equipment, or replace the canonical/native electromagnetic tests. The existing
five-elevation verification is still that bounded check. Full numerical data
are recomputed with the existing recipe when its native comparison fails.
