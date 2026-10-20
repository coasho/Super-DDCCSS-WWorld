# HC-53 external CMake fixture routing

## Scope and design

Date: 2026-10-20. Starting main HEAD:
`3c4685d8f28f5eeb126f68d9bd9f1aa57abfd183`; the tracked working tree was clean.
The user assigned the HC-53 correction, verification and commit, followed by
research-only preparation for the remaining SIM-M03 work.

Exclusive implementation paths are
`tools/validation/testing/scopes.toml`,
`tools/validation/testing/test_scope.py`, this record,
`docs/development.md`, `docs/status.md`,
`docs/sdk/weapon-fidelity-validation.md`, and the HC-53 entry in
`docs/architecture/validation/handoff-carryover-register.md`.
Only the root worker executes repository checks; research workers perform no
native execution or source writes. Temporary files remain under `D:/ENV/tmp`.

`tools/validation/missile/native/OpenVspIntersectionFixture.cmake` explicitly
belongs to an external OpenVSP geometry/API build. The SDK does not include it.
Its previous broad Python-reference scope was not recognized by the selector's
all-CMake invariant, which only admitted shared SDK configuration and known
repository test/package scripts.

The correction gives this one file an explicit external-fixture scope and
requires the repository's `missile_aero_reference_checker`. Selecting this file
alone requests no SDK build or render-enabled Release run. Its external native
OpenVSP checks remain an explicit campaign with upstream source/tool provenance.
The all-CMake invariant recognizes only this exact external input; other CMake
inputs retain their existing routing requirements. A mixed change containing
shared SDK CMake still escalates both SDK presets.

Regression coverage must exercise exact selection, the bounded exception,
missing mandatory reference guards before any runner starts, and mixed shared
configuration. Historical SIM-M03 test results remain dated evidence; HC-53
closure is recorded separately after current verification.

## Verification and closure

Before changing the rules, the expanded checker ran 105 test methods and
reported eight failing assertions/subcases, including the original all-CMake
failure, the missing exact external scope and missing-guard false passes.
The mixed SDK CMake control already passed and remains required.

The final rule and checker changes pass on both configured inventories:

| Preset | Focused CTest result | Internal cases |
| --- | --- | --- |
| `ucrt64-headless` | 2/2 passed in 12.21 s | Selector 105/105; aero safeguards 150/150 |
| `ucrt64-release` | 2/2 passed in 12.29 s | Selector 105/105; aero safeguards 150/150 |

Both commands use the existing PowerShell launcher with
`test --no-tests=error --timeout 120 -j 1 -R
"^(testing_scope_checker|missile_aero_reference_checker)$"`.
Raw RED/GREEN output and full internal counts are retained under
`build/architecture-reviews/hc-53/`.

The actual CLI plan for the exact fixture also succeeds: only the Headless
inventory is used, `build=false`, no escalation or errors, and the selected
tests are `missile_aero_reference_checker` plus the selector checker discovered
as a reader. The report is `fixture-plan.json` in that evidence directory.
No SDK configure/build, external OpenVSP build, solver campaign or simulation
model change was needed. Independent read-only review approved the bounded
exception and the missing-guard/mixed-input controls with no findings.

HC-53 is closed for test routing. The original SIM-M03 18/19 result remains
historical evidence; the two focused current passes establish this correction
without claiming that the earlier entire selection or full suite was rerun.
