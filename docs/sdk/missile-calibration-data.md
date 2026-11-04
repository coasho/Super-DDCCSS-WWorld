# Missile geometry and aerodynamic-calibration data status

Date: 2026-10-29. Baseline: main `004e1f7` plus the existing uncommitted
Hangar calibration work. This record covers only the inputs used to build the
store aerodynamic tables: length, diameter, span, mass and the authored
surface geometry. It does not claim aerodynamic-coefficient calibration.

## Evidence rule

A field is `published` only when a public source states it for the named
variant. A manufacturer page that gives mass, length and diameter is used for
those three fields only. A span that no source publishes stays `assumed` with
its secondary/photo origin and a widened uncertainty. No aerodynamic
coefficient is imported from a generic missile report.

## Current status

| Card | Geometry/mass status | Source class | Aerodynamic data |
| --- | --- | --- | --- |
| AIM-120C-7 | 3.66 m / 0.1778 m / 0.483 m / 161.5 kg | official USN/USAF | no named-variant coefficient table; VLM estimate |
| AIM-120D | same airframe as C-7 | official USN/USAF | no named-variant coefficient table; VLM estimate |
| AIM-7M | 3.64 m / 0.203 m / 1.0 m / 225 kg | official USAF | no named-variant coefficient table; NASA TP-1078 is Sparrow III type only |
| AIM-9M | 2.87 m / 0.127 m / 0.63 m / 85.5 kg | official USAF | no named-variant coefficient table; NACA Sidewinder reports are early marks |
| AIM-9X Block I | 3.018 m / 0.127 m / 0.45 m / 84 kg | official USN/NAVAIR | no named-variant coefficient table; VLM estimate |
| ASRAAM | 2.9 m / 0.166 m / 88 kg; span 0.45 m | official MBDA (span estimate) | no public coefficient table; VLM estimate |
| IRIS-T | 2.936 m / 0.127 m; mass 87.4 kg and span 0.447 m | official Saab length/diameter; mass/span estimate | no public coefficient table; VLM estimate |
| Magic 2 | 2.75 m / 0.157 m / 0.66 m / 90 kg | official ODIN | no public coefficient table; VLM estimate |
| Meteor | 3.7 m / 0.178 m / 190 kg | official MBDA | no public coefficient table; VLM estimate |
| MICA EM | 3.1 m / 0.16 m / 112 kg; span 0.48 m | official MBDA (span estimate) | no public coefficient table; VLM estimate |
| MICA IR | 3.1 m / 0.16 m / 112 kg; span 0.48 m | official MBDA (span estimate) | no public coefficient table; VLM estimate |
| PL-8 | 2.95 m / 0.16 m / 0.8 m / 115 kg | secondary | no public coefficient table; VLM estimate |
| PL-10 | 3.0 m / 0.16 m / 89 kg; span missing | secondary | no public coefficient table; VLM estimate |
| PL-12 | 3.93 m / 0.203 m / 0.67 m / 180 kg | secondary | no public coefficient table; VLM estimate |
| PL-15 | 4.0 m length per ODIN; diameter/span secondary | partial official | no public coefficient table; VLM estimate |
| R-27ER | 4.78 m / 0.26 m / 0.803 m / 350 kg | secondary | no public coefficient table; VLM estimate |
| R-27ET | 4.49 m / 0.26 m / 0.803 m / 348 kg | secondary | no public coefficient table; VLM estimate |
| R-27R | 4.08 m / 0.23 m / 0.772 m / 253 kg | secondary | no public coefficient table; VLM estimate |
| R-27T | 3.795 m / 0.23 m / 0.774 m / 253 kg | secondary | no public coefficient table; VLM estimate |
| R-37M | 4.06 m / 0.38 m / 0.72 m / 510 kg | secondary | no public coefficient table; VLM estimate |
| R-60M | 2.132 m / 0.12 m / 0.39 m / 45 kg | secondary | no public coefficient table; VLM estimate |
| R-73 | 2.93 m / 0.165 m / 0.51 m; mass 105 kg and warhead 8 kg | official Rosoboronexport mass/warhead; geometry secondary | no public coefficient table; VLM estimate |
| R-77M | 3.71 m / 0.2 m / 190 kg | secondary | no public coefficient table; VLM estimate |
| Super 530D | 3.8 m / 0.263 m / 0.62 m / 270 kg | secondary | no public coefficient table; VLM estimate |

## Changes made on 2026-10-29

- ASRAAM: mass, length and diameter moved from Wikipedia to MBDA's
  specification page; span kept as an explicit estimate.
- IRIS-T: length corrected to 2.936 m and diameter to the Saab specification;
  mass and span kept as explicit estimates.
- Meteor: length corrected to 3.7 m; mass, length and diameter moved to MBDA.
- MICA EM/IR: mass, length and diameter moved to MBDA; span kept as an
  explicit estimate.
- Magic 2: length corrected to 2.75 m and mass to 90 kg from ODIN; diameter
  and span confirmed.
- R-73: mass and warhead moved to Rosoboronexport (105 kg, 8 kg).

## Remaining gaps

No public named-variant wind-tunnel coefficient table was found for any of the
24 AAMs. The reports in the calibration plan (NASA CR-2835/2836, NASA
TP-1078, NASA TP-2157, NACA RM SL55C22, AIAA 3.62226) are generic or
nearby-configuration data. They can validate a method, not certify a named
variant. The current aerodynamic tables are therefore geometry-based VLM
estimates, not measured coefficients; they must remain labelled as such.

The next source work is the Russian and Chinese cards: R-27 family, R-60M,
R-37M, R-77M, PL-8/10/12/15. Their current geometry values are secondary and
should be replaced only by an exact manufacturer/government publication; until
then they remain estimates. Magic 2's warhead mass, Super 530D's geometry and
the R-27 family's exact variant dimensions also remain secondary.

## Stale aerodynamic tables

Changing a published length, diameter or mass changes the geometry input of
the shared VLM table. The cards' `hangar-vlm` references still point to the
tables produced from the previous geometry. ASRAAM, IRIS-T, Meteor, MICA,
Magic 2, R-73 and PL-15 therefore need a Hangar regeneration before their
runtime aerodynamic tables can be treated as matching these cards. Until that
regeneration runs, the existing tables are explicitly stale; they are not
new calibration evidence.
