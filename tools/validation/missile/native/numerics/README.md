# Imported numerical research fixtures

These are selected, hash-verified files from AeroNumericsLab deliveries `02`,
`completion` and `completion-revision`, integrated on main from `d5c8203`.
`PROVENANCE.json` records each original relative path and SHA256, together with
the integrated path/hash. The LAB was read only. `DELIVERY.md` is the unedited
author handoff; its claims remain distinct from independent project review.

The integration includes the required-schema/continuation correction, strict
VSPGEOM/VKEY readers, the active-wake containment check, its actual native ADB
negative controls, the modified VSPAERO R2 source patch, both retained R2 cases,
and all four final SU2 histories. Panel closure's exact kernel patch and all
336 native controls were already present under the parent directory; they are
reused rather than replaced with a second implementation.

The geometry-reader function bodies were extracted unchanged. Containment now
accepts an explicit basename and imports the project reader. Its ADB reader
also rejects nonfinite data and bounds file/record sizes; a new negative
regression demonstrated that the delivered reader did not reject NaN Gamma.
The orchestration suite mocks binary readers only for its synthetic process
fixtures; the separate containment suite uses actual native geometry and ADB.

The R2 patch applies to the released reference `VSP_Solver.C`, SHA256
`5a8c97d10bebba850091df5ac247ae24918dc244ae6ee4be9f2e325b8c39fc7d`.
It reproduces the delivered modified unit
`1bf4afa5d6d7c232aec38d50c45640f75940cc3d2752ce8e0e74eacc8acf106c`.
The delivered executable SHA256 is
`bf462933fd3f35c3ff321712faa59fef74717d25344260c067f4dd1625aa26d0`.
The existing wake-root changes in `VSP_Geom.C`, `VSP_Edge.C` and
`Vortex_Trail.C` are prerequisites. This is modified VSPAERO 7.2.2, not stock
OpenVSP. Retain the included OpenVSP license and modification attribution.

`run_numerics_vspaero.py` is the delivery's bounded native launcher retargeted
to these checked-in input fixtures. It retains the original two-thread command,
120-second timeout, fresh output directory, exact input/executable hashes and
output-presence checks. Supply an explicit executable and SHA256; it never
installs a tool or changes its source. Run alpha0 and alpha2 serially with
`--case`, `--exe`, `--exe-sha256` and `--run-dir`, then pass both fresh folders
to `review_numerics.py --vsp-cases <alpha0> <alpha2> --out <report>`.

Run from the repository root using the configured project interpreter:

```powershell
.\fsim.cmd --preset ucrt64-headless python tools/validation/missile/test_aero.py
.\fsim.cmd --preset ucrt64-headless python tools/validation/missile/test_aero_wake_containment.py
.\fsim.cmd --preset ucrt64-headless python tools/validation/missile/review_numerics.py --out build/numerics-review.json
```

Both complete CTest inventories register the suites and retained-data replay.
The latter requires NumPy, already required by the Hangar environment. The
standard-library aerodynamic reference suite keeps optional NumPy imports lazy.

A passing replay requires retaining the original coarse alpha-0 negative-drag
failure, passing the other coarse physical gates, and passing the local-grid
and R2 checks. Its `review_passed` describes reproducibility of the declared
delivery disposition. Every result retains `model_accepted=false`.

No table is written into a flight model. The two-grid sensitivity pair is not
a spatial convergence sequence; static inviscid loads supply no dynamic damping,
crossflow or viscous channels, and no equipment-specific calibration. Native
iteration convergence alone does not prove superiority over the current game
model. These candidates remain available for subsequent correction and
independent load-accuracy validation without hiding their failures.
