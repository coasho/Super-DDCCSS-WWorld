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
| `aim-120c` | AIM-120C-7 AMRAAM | `active_radar` | 90.0 ± 15.0 | 137.1 | 66.0 | 102.0 | 32.0 | 59.2 | 28.2 | 1.52 | +52.3% | 25.5% |
| `aim-120d` | AIM-120D AMRAAM | `active_radar` | 145.0 ± 15.0 | 192.7 | 100.4 | 146.8 | 48.8 | 116.0 | 45.0 | 1.33 | +32.9% | 21.6% |
| `aim-7m` | AIM-7M Sparrow | `semi_active_radar` | 70.0 ± 15.0 | 129.9 | 69.5 | 93.6 | 32.3 | 61.9 | 28.6 | 1.86 | +85.6% | 20.2% |
| `aim-9m` | AIM-9M Sidewinder | `infrared` | 35.0 ± 10.0 | 52.8 | 26.8 | 45.2 | 17.5 | 41.7 | 13.8 | 1.51 | +50.8% | 17.3% |
| `aim-9x` | AIM-9X Block I Sidewinder | `imaging_infrared` | 35.0 ± 10.0 | 58.4 | 29.8 | 51.7 | 21.3 | 48.3 | 17.6 | 1.67 | +66.8% | 17.4% |
| `asraam` | AIM-132 ASRAAM | `imaging_infrared` | 25.0 ± 5.0 | 65.1 | 33.8 | 54.6 | 21.3 | 49.2 | 17.6 | 2.60 | +160.4% | 18.2% |
| `iris-t` | IRIS-T | `imaging_infrared` | 25.0 ± 5.0 | 61.5 | 32.9 | 53.3 | 22.8 | 48.6 | 19.1 | 2.46 | +146.0% | 17.7% |
| `magic-2` | R.550 Magic 2 | `infrared` | 20.0 ± 5.0 | 43.0 | 19.7 | 37.2 | 12.4 | 35.0 | 8.7 | 2.15 | +114.9% | 18.4% |
| `meteor` | Meteor | `active_radar` | 200.0 ± 50.0 | 329.9 | 205.8 | 194.6 | 120.3 | 58.5 | 116.5 | 1.65 | +65.0% | 9.6% |
| `mica-em` | MICA EM | `active_radar` | 80.0 ± 15.0 | 116.5 | 56.1 | 87.8 | 28.9 | 56.8 | 25.2 | 1.46 | +45.7% | 21.8% |
| `mica-ir` | MICA IR | `imaging_infrared` | 60.0 ± 15.0 | 101.7 | 54.5 | 78.0 | 29.2 | 57.9 | 25.5 | 1.69 | +69.5% | 20.5% |
| `pl-10` | PL-10 | `imaging_infrared` | 20.0 ± 5.0 | 59.6 | 31.0 | 49.7 | 19.3 | 45.0 | 15.6 | 2.98 | +197.8% | 18.4% |
| `pl-12` | PL-12 | `active_radar` | 85.0 ± 15.0 | 144.5 | 68.1 | 108.4 | 34.0 | 61.3 | 30.3 | 1.70 | +70.0% | 21.9% |
| `pl-15` | PL-15 | `active_radar` | 175.0 ± 25.0 | 257.2 | 149.0 | 209.2 | 93.4 | 162.8 | 89.6 | 1.47 | +47.0% | 25.1% |
| `pl-8` | PL-8 | `infrared` | 20.0 ± 5.0 | 58.2 | 32.2 | 47.3 | 19.7 | 42.0 | 16.0 | 2.91 | +190.8% | 18.1% |
| `r-27er` | R-27ER | `semi_active_radar` | 130.0 ± 20.0 | 201.3 | 114.3 | 136.5 | 46.3 | 73.5 | 42.5 | 1.55 | +54.8% | 26.6% |
| `r-27et` | R-27ET | `infrared` | 120.0 ± 20.0 | 185.3 | 103.6 | 126.6 | 43.6 | 72.1 | 39.9 | 1.54 | +54.4% | 25.2% |
| `r-27r` | R-27R | `semi_active_radar` | 73.0 ± 10.0 | 137.6 | 77.1 | 97.6 | 36.1 | 62.0 | 32.3 | 1.88 | +88.5% | 20.5% |
| `r-27t` | R-27T | `infrared` | 40.0 ± 10.0 | 115.5 | 65.6 | 87.0 | 34.4 | 63.9 | 30.6 | 2.89 | +188.6% | 19.3% |
| `r-37m` | R-37M | `active_radar` | 350.0 ± 50.0 | 506.6 | 318.8 | 306.9 | 128.0 | 191.1 | 124.2 | 1.45 | +44.7% | 32.1% |
| `r-60m` | R-60M | `infrared` | 8.0 ± 2.0 | 25.4 | 14.8 | 22.6 | 11.4 | 21.4 | 7.8 | 3.18 | +217.6% | 15.6% |
| `r-73` | R-73 | `infrared` | 30.0 ± 5.0 | 60.2 | 28.9 | 51.1 | 17.8 | 47.1 | 14.1 | 2.01 | +100.5% | 18.4% |
| `r-77m` | R-77M | `active_radar` | 190.0 ± 30.0 | 199.9 | 91.7 | 160.4 | 45.3 | 136.9 | 41.5 | 1.05 | +5.2% | 35.4% |
| `super-530d` | Super 530D | `semi_active_radar` | 40.0 ± 8.0 | 100.9 | 56.3 | 76.0 | 29.9 | 64.6 | 26.2 | 2.52 | +152.1% | 19.8% |

## Provenance Checksums

| Card | Model | Table SHA-256 (prefix) | XML SHA-256 (prefix) | Card SHA-256 (prefix) | Gen Time (s) |
|---|---|---|---|---|---:|
| `aim-120c` | `aim120c` | `80de1f6b055b32de` | `2bae0905c02170a0` | `57652ecac0ec51ed` | 81.9 |
| `aim-120d` | `aim120d` | `0334bbab3570fd84` | `f3a5b1161aa490ef` | `051db8b63bbf29cf` | 115.6 |
| `aim-7m` | `aim7m` | `9a5a3a68162a4ad9` | `bb6f3a47f4b93118` | `b4c4b3b0f3aee9ad` | 90.3 |
| `aim-9m` | `aim9m` | `be5d5eccb8a73530` | `42b2247ab5d4ce57` | `398ac00268e5e2ae` | 93.7 |
| `aim-9x` | `aim9x` | `0893f14f9be3e52c` | `e5ba67c16cf318f2` | `7e91b4396a243303` | 58.3 |
| `asraam` | `asraam` | `71a956f4a9e13a56` | `3cb771d4bc22b9fd` | `fc17957dccad4525` | 48.3 |
| `iris-t` | `irist` | `359eff8c6691babd` | `f050bdda7f83ca6f` | `5e31d9c0c4144814` | 59.7 |
| `magic-2` | `magic2` | `777302ea1efa8e98` | `b045ab05af05220b` | `3f52d91016460e99` | 45.3 |
| `meteor` | `meteor` | `1e7a66ddca73b0c4` | `52467d3fa779325a` | `76abca0c36e06137` | 121.5 |
| `mica-em` | `micaem` | `3ce8ce07bf467c71` | `ca37a1456c1fc2c9` | `000d92bd923e9dbf` | 91.1 |
| `mica-ir` | `micair` | `46e5b631590a3664` | `3e9d195eafb2bfc9` | `03048681c3bfdea8` | 73.6 |
| `pl-10` | `pl10` | `ae2561ba0316ed5b` | `f56a4fa95e5bb3d9` | `931e91e8d5f581fe` | 48.0 |
| `pl-12` | `pl12` | `6b4dd60b5a66b39f` | `568b226e780dc6d7` | `e06e209c8029b2c3` | 89.2 |
| `pl-15` | `pl15` | `d98ce30c0b6860d7` | `360444d1556401c6` | `ec2336f0aae73be3` | 98.2 |
| `pl-8` | `pl8` | `a1eb67fdf17a2df7` | `afc79681113772d0` | `9b25119a27b77d6b` | 97.9 |
| `r-27er` | `r27er` | `cefd0a1add57b0a9` | `97841ea49e9c3078` | `36273478060ef5c1` | 141.3 |
| `r-27et` | `r27et` | `5713ad6d2a5c6938` | `04c51422e4020bdc` | `f0c16f40b208efd0` | 134.4 |
| `r-27r` | `r27r` | `2016696698b6381e` | `24bf75c3cdcb2229` | `244117d9b27f68d3` | 111.6 |
| `r-27t` | `r27t` | `fc9330931eda2ff2` | `aca1633b9501932d` | `45f5c386c8d69a88` | 94.0 |
| `r-37m` | `r37m` | `3e60e11ee9dd1fbc` | `088b7d6fced76224` | `262fbb6a3b8e209a` | 105.7 |
| `r-60m` | `r60m` | `c20aa80ff6cb56b6` | `1ed5c6eb86df6703` | `ba8f176255d1eeaf` | 26.3 |
| `r-73` | `r73` | `de5600e2529496ad` | `6f73fcb4e3d1164a` | `66fc1aa221d32261` | 75.9 |
| `r-77m` | `r77m` | `c3af0874b5c51516` | `2ee93d8c536e5b58` | `dd06717dc29a6403` | 460.7 |
| `super-530d` | `super530d` | `a31f59662e576e2c` | `a329de29f8fb6439` | `1e014a9b9ede9e29` | 45.3 |
