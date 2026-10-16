# NTIA's propagation models

The U.S. reference implementations of two radio propagation models, by the
Institute for Telecommunication Sciences of the National Telecommunications
and Information Administration (NTIA/ITS). They are works of US Government
employees, not subject to copyright in the United States. NTIA grants the
rights to copy, modify and distribute them elsewhere (each folder's
`LICENSE.md`). The platform's radio path uses them (ADR-30, MS-2;
`src/simulation/mission_systems/rf`).

| Folder | Model | From | Commit |
| --- | --- | --- | --- |
| `p528` | Recommendation ITU-R P.528-5, its Annex 2 step-by-step method, with P.676's gaseous attenuation and P.835's reference atmosphere | https://github.com/NTIA/p528 | 74870ab0c9051f3db41c15055a6ef9395644cfc5 |
| `itm` | The Irregular Terrain Model (Longley-Rice) | https://github.com/NTIA/itm | 183ad95bd813a8be11009df396e1c631356864b2 |

Only `include/`, `src/`, the licence, the README and the error list of each are
copied, plus the ITM's own examples (`p2p.csv`, `pfls.csv`, `area.csv`), which
the tests reproduce (`tests/test_rf_path.cpp`).

## Changes (2026-10-06, flightsim)

As NTIA's licence asks, the modifications are stated here:

- Line endings changed to LF, and the ITM's `#include "..\include\..."` written with forward slashes.
- Every header gained `#pragma once`, and `DLLEXPORT` is defined empty: the
  platform compiles each model as one translation unit, inside a namespace
  of its own (`src/simulation/mission_systems/rf/NtiaP528.cpp`, `src/simulation/mission_systems/rf/NtiaItm.cpp`), with C++
  linkage and nothing exported.
- `p528/src/p676/RayTrace.cpp`: `GetLayerProperties` renamed
  `GetLayerPropertiesExact`, with its declaration added to
  `p528/include/p676.h`. The platform's `GetLayerProperties`
  (`src/simulation/mission_systems/rf/NtiaP528.cpp`) reads the layer's refractive index and specific
  attenuation from tables. They are tabulated per frequency from
  `GetLayerPropertiesExact`, four points to P.676's layer, so a P.528 path
  costs ~120-260 us instead of ~10 ms. The answers match the unmodified
  computation to 0.001 dB, and `ntia::tabulateLayers(false)` restores it
  (`tests/test_rf_path.cpp`).
- `p528/src/p528/TerminalGeometry.cpp`: `TerminalGeometry` renamed
  `TerminalGeometryExact`, with its declaration added to `p528/include/p528.h`.
  The platform's `TerminalGeometry` (`src/simulation/mission_systems/rf/NtiaP528.cpp`) reads the grazing
  ray's trace from the ground to the terminal - its arrival angle, gas loss,
  length and bending, functions of the frequency and the terminal's height alone
  - from a table per frequency, filled node by node from
  `TerminalGeometryExact`'s trace as heights are asked and interpolated cubically
  in the square root of the height. That trace was ~90 % of a P.528 path; the
  answers match the unmodified computation to 0.002 dB (`tests/test_rf_path.cpp`),
  and `ntia::tabulateLayers(false)` restores it.

No other line was changed.
