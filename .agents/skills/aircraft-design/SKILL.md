---
name: aircraft-design
description: Design, analyse and validate aircraft through Hangar source geometry, VLM tables, JSBSim generation and SDK runtime checks.
---

# Aircraft design with Hangar

Keep this file and `.claude/skills/aircraft-design/SKILL.md` byte-identical.
Use the [single Hangar guide](../../../docs/hangar-system.md) and `docs/status.md`.

Use `setup.ps1`, `build.ps1`, `run.ps1` and `stop.ps1`. Inspect geometry with
`run.ps1 -Mode hangar -Arguments @('MODEL', 'geometry')`; construct and verify
with the normal stages. Do not provision or invoke the retired SU2 workflow.

Author sourced geometry, airfoil/control definitions, mass, CG, inertia and
propulsion in the design. Keep runtime authority in the SDK. Generated XML is
an output, not a place to fit an unexplained correction. Preserve unrelated work
and public SDK compatibility. Check actual nonzero test counts and SDK loads.

The fixed-wing model contains both shared engineering correlations and optional
per-design calibration. Its shared vortex relation also records F16 calibration
influence. Report these honestly; neither full-angle tables nor a successful
performance fit proves equipment accuracy. The user requires aircraft/store
unification through the aircraft pipeline; historical store and rotor branches
must not be labeled completed universal VLM work. See current status before
claiming migration, fleet regeneration, clean-clone or physical acceptance.

Read authoritative upstream documentation and relevant research before changing
numerical methods. Keep one guide, normal entrypoint reproduction and bounded
temporary storage. Distinguish unfinished work from a delivered feature.
