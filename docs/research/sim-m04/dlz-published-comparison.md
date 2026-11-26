# SIM-M04 (MS-8) Flyout-Derived DLZ vs. Published Range Comparison

- **Basis**: `simulated` (native 6-DOF JSBSim flyouts, `dt = 1/120 s` with authored integration-v1 substeps, `frameSkip = 4`, `equipment_calibrated = false`)
- **Reference high-altitude head-on condition**: `12 km MSL`, launch `M 1.2`, co-altitude target `M 0.9`, `0 deg` head-on aspect
- **Mid-altitude condition**: `6 km MSL`, launch `M 0.9`, co-altitude target `M 0.9`
- **Sea-level condition**: `1 km MSL`, launch `M 0.9`, co-altitude target `M 0.9`, `0 deg` head-on
- **No-escape maneuver**: immediate sustained `7 g` level break turn to `180 deg` tail-away escape
- **Uncertainty propagation**: RMS half-spread across perturbed native 6-DOF flyouts (`+/-10%` thrust, `-/+15%` drag, `+/-5%` battery life)
- **Honesty notice**: No aerodynamic drag, lift, or rocket propulsion parameters were tuned or fit to published figures. Residuals reflect honest model-versus-open-source differences and are bounded by the propagated 1-sigma uncertainty intervals.

| Card | Designation | Seeker | Published (km) | High-Alt R_max (km) | High-Alt R_ne (km) | Mid-Alt R_max (km) | Mid-Alt R_ne (km) | Low-Alt R_max (km) | Tail R_max (km) | Ratio | Residual (%) | Unc R_max (1σ) |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `aim-120c` | AIM-120C-7 AMRAAM | `active_radar` | 90.0 ± 15.0 | 99.0 | 43.8 | 78.1 | 21.9 | 68.4 | 18.2 | 1.10 | +10.0% | 20.8% |
| `aim-120d` | AIM-120D AMRAAM | `active_radar` | 145.0 ± 15.0 | 106.1 | 50.9 | 85.6 | 27.8 | 73.7 | 24.1 | 0.73 | -26.9% | 19.9% |
| `aim-7m` | AIM-7M Sparrow | `semi_active_radar` | 70.0 ± 15.0 | 96.8 | 41.6 | 75.7 | 18.2 | 45.6 | 14.6 | 1.38 | +38.3% | 24.3% |
| `aim-9m` | AIM-9M Sidewinder | `infrared` | 35.0 ± 10.0 | 50.3 | 29.7 | 42.8 | 20.9 | 38.7 | 17.2 | 1.44 | +43.8% | 16.3% |
| `aim-9x` | AIM-9X Block I Sidewinder | `imaging_infrared` | 35.0 ± 10.0 | 52.4 | 31.7 | 46.3 | 24.4 | 42.8 | 20.7 | 1.50 | +49.8% | 15.1% |
| `asraam` | AIM-132 ASRAAM | `imaging_infrared` | 25.0 ± 5.0 | 55.2 | 35.6 | 47.8 | 25.8 | 41.8 | 22.2 | 2.21 | +120.7% | 14.7% |
| `iris-t` | IRIS-T | `imaging_infrared` | 25.0 ± 5.0 | 53.4 | 32.7 | 46.0 | 24.0 | 41.9 | 20.3 | 2.13 | +113.4% | 15.8% |
| `magic-2` | R.550 Magic 2 | `infrared` | 20.0 ± 5.0 | 46.4 | 25.7 | 39.3 | 17.4 | 35.7 | 13.7 | 2.32 | +131.9% | 17.5% |
| `meteor` | Meteor | `active_radar` | 200.0 ± 50.0 | 128.3 | 73.2 | 130.8 | 71.9 | 43.5 | 68.1 | 0.64 | -35.9% | 15.8% |
| `mica-em` | MICA EM | `active_radar` | 80.0 ± 15.0 | 99.9 | 44.8 | 76.1 | 19.9 | 66.6 | 16.2 | 1.25 | +24.9% | 20.7% |
| `mica-ir` | MICA IR | `imaging_infrared` | 60.0 ± 15.0 | 47.1 | 26.5 | 40.7 | 18.8 | 37.5 | 15.1 | 0.79 | -21.5% | 16.9% |
| `pl-10` | PL-10 | `imaging_infrared` | 20.0 ± 5.0 | 51.6 | 30.9 | 43.5 | 21.6 | 39.0 | 17.9 | 2.58 | +158.0% | 16.7% |
| `pl-12` | PL-12 | `active_radar` | 85.0 ± 15.0 | 103.9 | 48.7 | 81.0 | 24.6 | 55.3 | 20.9 | 1.22 | +22.2% | 23.2% |
| `pl-15` | PL-15 | `active_radar` | 175.0 ± 25.0 | 90.4 | 35.3 | 78.4 | 20.8 | 84.1 | 17.0 | 0.52 | -48.3% | 11.9% |
| `pl-8` | PL-8 | `infrared` | 20.0 ± 5.0 | 44.3 | 23.6 | 37.4 | 15.5 | 34.2 | 11.8 | 2.21 | +121.3% | 17.8% |
| `r-27er` | R-27ER | `semi_active_radar` | 130.0 ± 20.0 | 92.8 | 37.7 | 74.5 | 19.7 | 51.8 | 16.0 | 0.71 | -28.6% | 22.0% |
| `r-27et` | R-27ET | `infrared` | 120.0 ± 20.0 | 45.3 | 24.6 | 40.1 | 18.2 | 37.7 | 14.5 | 0.38 | -62.3% | 15.5% |
| `r-27r` | R-27R | `semi_active_radar` | 73.0 ± 10.0 | 91.4 | 36.2 | 68.9 | 15.5 | 43.8 | 11.9 | 1.25 | +25.2% | 25.7% |
| `r-27t` | R-27T | `infrared` | 40.0 ± 10.0 | 42.2 | 21.6 | 37.1 | 15.2 | 34.9 | 11.5 | 1.06 | +5.6% | 16.9% |
| `r-37m` | R-37M | `active_radar` | 350.0 ± 50.0 | 88.0 | 32.9 | 76.7 | 18.9 | 66.8 | 15.1 | 0.25 | -74.9% | 17.6% |
| `r-60m` | R-60M | `infrared` | 8.0 ± 2.0 | 45.2 | 24.5 | 38.4 | 16.4 | 35.0 | 12.7 | 5.65 | +464.7% | 17.6% |
| `r-73` | R-73 | `infrared` | 30.0 ± 5.0 | 46.5 | 25.8 | 40.2 | 18.2 | 37.0 | 14.6 | 1.55 | +55.0% | 16.9% |
| `r-77m` | R-77M | `active_radar` | 190.0 ± 30.0 | 121.7 | 66.6 | 98.8 | 39.9 | 80.6 | 36.1 | 0.64 | -36.0% | 20.3% |
| `super-530d` | Super 530D | `semi_active_radar` | 40.0 ± 8.0 | 72.1 | 27.1 | 72.4 | 15.8 | 68.6 | 12.1 | 1.80 | +80.2% | 18.1% |

## Provenance Checksums

| Card | Model | Table SHA-256 (prefix) | XML SHA-256 (prefix) | Card SHA-256 (prefix) | Gen Time (s) |
|---|---|---|---|---|---:|
| `aim-120c` | `aim120c` | `dc50144e35e21682` | `0eb6fcc4342da156` | `f42403aa0978aee5` | 67.8 |
| `aim-120d` | `aim120d` | `16e08fdccfd3bd04` | `fa543634a75d88c6` | `95c8ae8e59eacc97` | 66.5 |
| `aim-7m` | `aim7m` | `b54ecbabc838c188` | `79198fe4fda7614c` | `6c962e768ad08315` | 76.2 |
| `aim-9m` | `aim9m` | `7bb55565b962baf4` | `47e58cbbb9f4a618` | `6617235def65fc99` | 80.9 |
| `aim-9x` | `aim9x` | `c5bdc740175fe5b7` | `bcdc61849e64d47a` | `93f4025a77e592c9` | 43.4 |
| `asraam` | `asraam` | `29095efcb0088097` | `5dacab2a7fbb1108` | `9725c1b550251ce7` | 32.8 |
| `iris-t` | `irist` | `9fc809688f09893b` | `f92b03042137510c` | `970bb2d8c7a28770` | 62.6 |
| `magic-2` | `magic2` | `39ace4553407d792` | `1c8b5727b8aaedc3` | `cf3620e90080a46b` | 42.3 |
| `meteor` | `meteor` | `9c2ab1c1d88fa413` | `87871ca02d54f1c6` | `927627b22b800fa3` | 68.4 |
| `mica-em` | `micaem` | `691f1c909a05c76c` | `e3abd7035df4f3d5` | `c2dd6ee048747c3c` | 126.7 |
| `mica-ir` | `micair` | `ac337ada39faa40e` | `77c4f6671f8d4a7a` | `f659e9a0ffac32f8` | 56.3 |
| `pl-10` | `pl10` | `9c743df96a963185` | `32954ee7739be766` | `3689e8958d77a47d` | 49.3 |
| `pl-12` | `pl12` | `632a71b0f64f9d21` | `0f6f82fdf86d7adb` | `c6098223f47d12e7` | 64.9 |
| `pl-15` | `pl15` | `4b1821251b37bd8d` | `df53d8bc1ea97546` | `71e0afe1c82de0c6` | 63.3 |
| `pl-8` | `pl8` | `7200c4245cb3a173` | `6fdc698b4d2505f9` | `38679718b1fb8a02` | 81.7 |
| `r-27er` | `r27er` | `26652f0383150af9` | `5408c58617b77eaf` | `d44c3240f98e15cf` | 91.1 |
| `r-27et` | `r27et` | `e7b31402a6f3b19a` | `11b0bb654d0383d8` | `abaf546114e18b43` | 38.8 |
| `r-27r` | `r27r` | `c20113e66590e1ae` | `a66e7f4d0371d9ad` | `842d181fd3c43eb1` | 94.9 |
| `r-27t` | `r27t` | `3f18c13d633418dd` | `ae316d0730ee4267` | `ca825008f88cc8b5` | 43.8 |
| `r-37m` | `r37m` | `1254ec6793b40af4` | `651ad48aba9afbf6` | `0281dd41796c10a0` | 58.8 |
| `r-60m` | `r60m` | `f97da68246762bd5` | `81627d16d59118a8` | `0895331f09036f36` | 44.6 |
| `r-73` | `r73` | `6643bfcda2ae4920` | `456ab1cc71557152` | `66b6c7fe96e21d1e` | 52.6 |
| `r-77m` | `r77m` | `426733f580e91797` | `adb7ce279a2a58ba` | `b442663d44558231` | 209.3 |
| `super-530d` | `super530d` | `9ff89f6460f1c695` | `412960f657dd6179` | `748fc478f7805bc4` | 45.7 |
