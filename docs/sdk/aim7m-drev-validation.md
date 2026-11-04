# AIM-7M DREV-TM-1999-050 validation

Date: 2026-10-30. Source: public, unclassified DREV-TM-1999-050
(E. Y. Fournier, July 1999), a 1:7-scale AIM-7M trisonic wind-tunnel
investigation. This record is a validation baseline, not a claim that the
AIM-7M model is calibrated.

## Dataset and tooling

- Dataset: `tools/validation/missile/data/aim7m_drev_tm1999_050.json`
- Validator: `tools/validation/missile/aim7m_drev.py`
- Public source: https://archive.org/details/micro_IA40243311_1824
- Local PDF/OCR: `build/architecture-reviews/sim-m02-sources/aim7m-drev-tm-1999-050.{pdf,ocr.txt}`
- Tables admitted: CNalpha, CMalpha and XCP for the measured 1:7-scale model;
  drag is incomplete because the axial-force balance failed.
- Limits: lower Reynolds than free flight, sting interference 10-15% for drag,
  only normal-force/pitching-moment/XCP slopes, not full-scale AIM-7M.

## Run

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -Command "& './run.ps1' -Mode python -Arguments @('-B','tools/validation/missile/aim7m_drev.py','--out','build/architecture-reviews/sim-m02-sources/aim7m-drev-validation.json')"
```

The validator measures the generated AIM-7M static fixture at alpha -5, 0 and
+5 deg, computes the same linear slopes, and compares them with DREV Table C-1
to C-5 for 0 deg roll and 0 deg wing deflection. It does not fit parameters or
write production assets. The OCR does not print the slope unit; the values are
treated as per radian, which the magnitudes support, and the validator uses
radians.

Offline dataset integrity checks:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -Command "& './run.ps1' -Mode python -Arguments @('-B','-m','unittest','tools.validation.missile.test_aim7m_drev')"
```

The five checks cover source identity, table coverage, known C-2/C-5 values,
explicit null handling, and the catalog link. They passed 5/5 on 2026-10-30.

## Current SDK baseline

| Mach | DREV CNalpha | SDK CNalpha | Error | DREV CMalpha | SDK CMalpha | Error | DREV XCP | SDK XCP | Error |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.5 | 42.710 | 25.824 | -39.54% | -57.149 | -32.984 | -42.28% | 1.3381 | 1.2772 | -4.55% |
| 2.0 | 33.264 | 22.230 | -33.17% | -46.155 | -31.254 | -32.28% | 1.3875 | 1.4060 | +1.33% |
| 2.5 | 27.167 | 17.626 | -35.12% | -35.027 | -25.248 | -27.92% | 1.2893 | 1.4324 | +11.10% |
| 3.0 | 24.448 | 14.601 | -40.28% | -27.093 | -21.194 | -21.77% | 1.1082 | 1.4515 | +30.98% |
| 3.5 | 21.841 | 12.802 | -41.39% | -23.047 | -18.733 | -18.72% | 1.0552 | 1.4633 | +38.67% |

NRMSE over these five points:

- CNalpha: **38.03%**
- CMalpha: **29.78%**
- XCP: **22.81%**

## Reading

The generated AIM-7M model is significantly low in normal-force slope and
pitching-moment slope over Mach 1.5-3.5. Center of pressure is close at Mach
1.5-2.0 but drifts aft at higher Mach. This is a real model/reference gap, not
a measurement noise claim. The next correction step is to determine whether the
gap comes from the authored surface geometry, the shared VLM lift model, or the
reference normalization; the DREV tables provide the target and the validator
provides the before/after measurement.

## Correction triage: geometry

Figure B-4 of the report gives the model control fin in inches: root chord
2.616, tip chord 0.360, exposed semispan 2.305. At 1:7 these are 0.465 m,
0.064 m and 0.410 m. The authored AIM-7M control fin is larger (0.62 m root,
0.10 m tip, 0.3985 m exposed semispan), so a source-derived geometry is a
candidate correction.

A scratch shared-VLM comparison replacing only the control fin gives the
relative sensitivity below. These are shared-VLM coefficients, not the SDK
static fixture, so this is a triage check, not a replacement for the DREV
baseline.

| Mach | baseline cna | DREV-fin cna | baseline cma | DREV-fin cma |
|---:|---:|---:|---:|---:|
| 2.0 | 18.900 | 15.436 | -13.491 | -4.948 |
| 3.0 | 12.409 | 9.989 | -4.970 | +0.832 |

This is the opposite of the needed correction: the source-derived smaller fin
reduces normal-force slope further and makes the pitching moment less negative.
Geometry alone therefore cannot close the 33-41% CNalpha deficit. The next
correction must address the shared VLM body/fin carryover or lift method, not
shrink the authored TOML. The Figure B-5 tail-fin chain is not used yet because
its dimensions are less clear in the scan.

The scratch check is reproducible with:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -Command "& './run.ps1' -Mode python -Arguments @('-B','tools/validation/missile/aim7m_geometry_check.py')"
```

It writes `build/architecture-reviews/sim-m02-sources/aim7m-geometry-sensitivity.json`
and does not modify production geometry.

No parameter is changed by this record. The result is a baseline for a future
correction, and it does not close SIM-M02 or any full-envelope AIM-7M claim.
