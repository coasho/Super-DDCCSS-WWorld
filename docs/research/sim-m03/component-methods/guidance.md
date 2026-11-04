# Per-model guidance theory and physical applicability

This source-bound package contains 24 distinct model projections. It preserves actual native geometry, mass/CG/inertia, receiver capability, actuator limits and engine inputs. It does not admit current numerical/native encounter acceptance, hardware calibration or superiority over DCS. Historical executed comparisons remain in the JSON under their original source/card epochs.

## Governing method

`a_PN=N*V_scale*(LOS_dot projected perpendicular own velocity); gravity compensation added before total maxG clipping`

`V_scale=positive measured closing when available; otherwise own speed. Range/closing availability is a sensor capability, not inferred target truth`

`F_required=m*a; LOSrate_saturation≈g0*sqrt(maxG^2-1)/(N*V_scale) for orthogonal gravity and steering`

## State, I/O and execution

Input: admitted measured LOS/rate and optional measured range/range-rate with their original sample epochs, own estimated pose/velocity, permitted launcher support and own age. Output: SI ECEF acceleration demand, phase/support/track status. Terminal-acquisition latch is reset through the versioned guidance reset interface. Prediction uses original measurement time once; no referee target state enters terminal PN.

## Assumptions and validity

- Finite receiver cadence/processing/scan aperture and gimbal dynamics precede guidance; tracked LOS and LOS rate are correlated estimates
- Noise, missed exposure and target maneuver affect LOS_dot. Shared N is an engineering assumption, not identified variant guidance law
- Gravity/loft/saturation coupling and capability-dependent V_scale invalidate one common closing-speed benchmark for all24

The declared analysis point is 500m/s at 6000m, zero incidence/sideslip, standard tropospheric air and launch/half/depleted propellant states. It is an offline projection of actual serialized equations. Positive local poles would be a defect indicator; negative local poles do not establish finite delayed nonlinear encounter stability. The source parameter basis and uncertainty are preserved; the +/-10% sweeps are sensitivity, not fitted confidence intervals.

## Primary references

- [Palumbo, Blauwkamp and Lloyd, Basic Principles of Homing Guidance, JHU APL Technical Digest 29(1), 2010](https://secwww.jhuapl.edu/techdigest/content/techdigest/pdf/V29-N01/29-01-Palumbo_Principles_Rev2018.pdf): PN LOS reconstruction, sensor availability and finite lateral-acceleration realization; not OEM missile gains

## Actual model deltas

| Model | Native mass kg | Jyy kg m2 | Mechanism | Pitch local max real pole 1/s |
|---|---:|---:|---|---:|
| aim-120c | 161.5 | 180.688 | fsim.guidance.measured_pn.v2 | -4.1654 |
| aim-120d | 161.5 | 180.689 | fsim.guidance.measured_pn.v2 | -4.16195 |
| aim-7m | 225 | 248.99 | fsim.guidance.measured_pn.v2 | -3.33602 |
| aim-9m | 85.5 | 58.8201 | fsim.guidance.measured_pn.v2 | -1.6915 |
| aim-9x | 84 | 63.9021 | fsim.guidance.measured_pn.v2 | -2.13191 |
| asraam | 88 | 61.8117 | fsim.guidance.measured_pn.v2 | -2.49357 |
| iris-t | 87.4 | 62.9245 | fsim.guidance.measured_pn.v2 | -1.79029 |
| magic-2 | 90 | 56.8461 | fsim.guidance.measured_pn.v2 | -1.8692 |
| meteor | 190 | 217.246 | fsim.guidance.measured_pn.v2 | -4.21734 |
| mica-em | 112 | 89.8958 | fsim.guidance.measured_pn.v2 | -2.23237 |
| mica-ir | 112 | 89.8958 | fsim.guidance.measured_pn.v2 | -2.3158 |
| pl-10 | 89 | 66.9008 | fsim.guidance.measured_pn.v2 | -2.5 |
| pl-12 | 180 | 232.194 | fsim.guidance.measured_pn.v2 | -3.27688 |
| pl-15 | 210 | 280.632 | fsim.guidance.measured_pn.v2 | -4.77315 |
| pl-8 | 115 | 83.5867 | fsim.guidance.measured_pn.v2 | -1.66724 |
| r-27er | 350 | 667.91 | fsim.guidance.measured_pn.v2 | -3.79635 |
| r-27et | 348 | 585.959 | fsim.guidance.measured_pn.v2 | -3.19681 |
| r-27r | 253 | 351.753 | fsim.guidance.measured_pn.v2 | -5.51097 |
| r-27t | 253 | 304.325 | fsim.guidance.measured_pn.v2 | -4.31654 |
| r-37m | 510 | 702.132 | fsim.guidance.measured_pn.v2 | -6.00102 |
| r-60m | 45 | 17.0837 | fsim.guidance.measured_pn.v2 | -2.91192 |
| r-73 | 105 | 75.2869 | fsim.guidance.measured_pn.v2 | -5.19525 |
| r-77m | 190 | 218.423 | fsim.guidance.measured_pn.v2 | -3.04367 |
| super-530d | 270 | 325.63 | fsim.guidance.measured_pn.v2 | -3.97841 |

### aim-120c

Parameter projection SHA-256: `fbf3c62b35fd77a9df5a230dc4132a924207042189a267a4034eaa6325131c9f`. Plant SHA-256: `550d9dc2489bc5356abea09b35f859ca9a9ab76a61e48a32e6f49af1763591d8`.

- radar measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass161.499989kg requires8478.74944N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=20deg, maxG=35, updates=1; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 47.25 / 52.5 / 57.75 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 7630.8745 / 8478.74944 / 9326.62439 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### aim-120d

Parameter projection SHA-256: `177b6ca4c80f31ad9ed7fd902e4c28cf2ee19ec4103225cc2c6f22f65a6c079c`. Plant SHA-256: `3ce42309096c29790d6d45eb6098b3e35b31b36b8eca4790cf187f95d636c7a5`.

- radar measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass161.499998kg requires8478.74992N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=20deg, maxG=35, updates=1; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 47.25 / 52.5 / 57.75 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 7630.87493 / 8478.74992 / 9326.62491 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### aim-7m

Parameter projection SHA-256: `55b90d42b0babe8ddb5227a2e9168b0a542647df9fbc89bd277cbf96a55b46ab`. Plant SHA-256: `75c07167eba4f3be6fa48df961b72c09c07119ed94b9da73244da936e1b6f160`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass225kg requires7875N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=20deg, maxG=35, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 7087.5 / 7875 / 8662.5 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### aim-9m

Parameter projection SHA-256: `51a1799852ff2f44a56736ea61f9252115338b35fc0a856b8d4c010a7de6ab1b`. Plant SHA-256: `2519f562e95ec92b6c69bd6a6f6267b28d269c61164fd0bfe777e13d6bd08f84`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass85.499989kg requires2992.49962N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=0deg, maxG=40, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 2693.24965 / 2992.49962 / 3291.74958 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.0227699112 / 0.0237699112 / 0.0247699112 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### aim-9x

Parameter projection SHA-256: `b233c0b68de448c6bcdc8423472f9035f2abe959396f71a546b3995ba5da7dbb`. Plant SHA-256: `aca7c078fce2c1bef2c78273b0497a00dd90b1edd5514be72fe8108e95625688`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass84.0000044kg requires2940.00016N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=0deg, maxG=40, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 2646.00014 / 2940.00016 / 3234.00017 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### asraam

Parameter projection SHA-256: `1e3cd5eef54af36643370af822f17b3bf94cfdd97ba7815340d6d9263b26db18`. Plant SHA-256: `a7c76134c340f7e36c0000fd1f2ecde32f6ae4c2a40b1a111caec03ba163c699`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass87.9999951kg requires3079.99983N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=0deg, maxG=40, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 2771.99985 / 3079.99983 / 3387.99981 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### iris-t

Parameter projection SHA-256: `04143ce3d884319bfcdb524f49fb80902cf10d63ba05f38f2f6705c2d289ea52`. Plant SHA-256: `a0f0853283b555c6db8fd9c94bc50b20dec29ca3727611a2200413381022aa04`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass87.4000149kg requires3059.00052N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=0deg, maxG=40, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 2753.10047 / 3059.00052 / 3364.90057 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### magic-2

Parameter projection SHA-256: `e633631307113c387fd43eb1e99431e9eebd8bb1e93020a7c2f4ccf6e1c94353`. Plant SHA-256: `dd3d2519d2b59d3d3a5b3e6cff82ac9aeaf665ba84999682cb5ab154af5f9bab`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass90.00002kg requires3150.0007N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=0deg, maxG=40, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 2835.00063 / 3150.0007 / 3465.00077 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.0227699112 / 0.0237699112 / 0.0247699112 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### meteor

Parameter projection SHA-256: `f37223aba2e7d5e557904e27b725c621e1e2ff7ae329997ac8c68fc21de86fe9`. Plant SHA-256: `02c922fa807029d53ce55ec354be2a763fbddf3f4fa87a5fcc5d5beadedb6718`.

- radar measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass190.000019kg requires9975.00101N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=20deg, maxG=35, updates=1; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 47.25 / 52.5 / 57.75 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 8977.5009 / 9975.00101 / 10972.5011 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### mica-em

Parameter projection SHA-256: `ff390680932078d2c2e655243cc2562e79870f5d00579013f25ec9707a46f824`. Plant SHA-256: `9e36c380bc4070d60a315639b68a48054afd2ab642d2f6fc84b1bea65a61640b`.

- radar measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass112.000007kg requires5880.00039N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=20deg, maxG=35, updates=1; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 47.25 / 52.5 / 57.75 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 5292.00035 / 5880.00039 / 6468.00043 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### mica-ir

Parameter projection SHA-256: `e6046050f4917cc0ffcbe6099a2f8242bfe22ee3740c723ccce6fdfe1f1a5fbd`. Plant SHA-256: `1821db51276f0afb37cb0da2295ceeaeb6d62d9729deef6f5bf7203e80f38c23`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass112.000007kg requires3920.00026N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=0deg, maxG=40, updates=1; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 3528.00023 / 3920.00026 / 4312.00029 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### pl-10

Parameter projection SHA-256: `e0c48a844510701246c677da88f5a7dd8d825dacb1d642b63c9e6fddab99b140`. Plant SHA-256: `c410ed7e34ed47054a887ac352349a28962c0b92f0b332e3d7d77699af084a90`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass89.0000166kg requires3115.00058N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=0deg, maxG=40, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 2803.50052 / 3115.00058 / 3426.50064 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### pl-12

Parameter projection SHA-256: `b15d02bd6b1258fee5eb7580d847ff675a6cb7b1d85e9886066aabe9683060bf`. Plant SHA-256: `e4c01fcbe3bbf27a19795c5f6bdc93158f6dbc2b8322da6510c9bcb94848e569`.

- radar measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass180.000022kg requires9450.00114N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=20deg, maxG=35, updates=1; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 47.25 / 52.5 / 57.75 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 8505.00103 / 9450.00114 / 10395.0013 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### pl-15

Parameter projection SHA-256: `be4dde56eef7d0cb371c6e7ec7ca76186dc3901f4c238e8a4ff32ad8a1589c81`. Plant SHA-256: `2195e1957c015ca4015c4c11a0c12fd2d91ff2623d15a9b3970d92b3ca3c594b`.

- radar measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass210.000004kg requires11025.0002N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=20deg, maxG=35, updates=1; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 47.25 / 52.5 / 57.75 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 9922.5002 / 11025.0002 / 12127.5002 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### pl-8

Parameter projection SHA-256: `730c253252738ad187ab2278ca67c3b74049732c10e450569638c33fd30869e2`. Plant SHA-256: `bdbca3627ca78563b6f9672487ecb0e54f14e21b1fbdf4c1408893d4295dbd38`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass115.000022kg requires4025.00077N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=0deg, maxG=40, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 3622.50069 / 4025.00077 / 4427.50085 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.0227699112 / 0.0237699112 / 0.0247699112 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### r-27er

Parameter projection SHA-256: `6be11b0dea5733a35b4ab97d15262bdd528191df256641186ad43fc946713842`. Plant SHA-256: `470f960a5e26febb47bc625e498f7883693bf270c7ebb78db6458d4ef1f45520`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass350.000001kg requires12250N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=20deg, maxG=35, updates=1; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 11025 / 12250 / 13475 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### r-27et

Parameter projection SHA-256: `e9c8c8f4d860f8867f3709ea7b77e4741f4a0a75e106cf2ab1b8fefce10bcec8`. Plant SHA-256: `3449c4f07b6a23fca3c7d0da91b02b59c5eaa29a27e383c767733b21f25f418d`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass348.000022kg requires12180.0008N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=0deg, maxG=40, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 10962.0007 / 12180.0008 / 13398.0008 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.0227699112 / 0.0237699112 / 0.0247699112 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### r-27r

Parameter projection SHA-256: `ff2dcc9cdfe95d038c1405424c9d99a416da1152896e48c161d722e241bb717f`. Plant SHA-256: `b427f8d0361a2ac66cc060c7e4ab20e55160d3cff00855690ab948555ac448cd`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass252.999989kg requires8854.99963N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=20deg, maxG=35, updates=1; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 7969.49967 / 8854.99963 / 9740.49959 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### r-27t

Parameter projection SHA-256: `7b49b26e3c8dd9e7d7b054068c324e9380f8c5680a59470e7e66be9e8dda7b68`. Plant SHA-256: `d91746d1a779f3862acec146456475e274563d57b21d9b6d51e2e34f901c0492`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass252.999989kg requires8854.99963N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=0deg, maxG=40, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 7969.49967 / 8854.99963 / 9740.49959 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.0227699112 / 0.0237699112 / 0.0247699112 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### r-37m

Parameter projection SHA-256: `145f53478e16f267cb51a98cd71b09ac433109f48d0db80913f5df30e647a33c`. Plant SHA-256: `a480640a3ada7b9437ff4b78418ea98d9813f51856f97f707908b8870dee154c`.

- radar measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass509.999997kg requires26774.9998N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=20deg, maxG=35, updates=1; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 47.25 / 52.5 / 57.75 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 24097.4998 / 26774.9998 / 29452.4998 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### r-60m

Parameter projection SHA-256: `9028b995f1940e9a78f50a39447701d5b09bd9c3af7ad98820b7a41733154535`. Plant SHA-256: `5773bc316905806dd04d1b4b85aa6199e1704e63f7f86d8904befe9e21f7077c`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass44.9999918kg requires1574.99971N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=0deg, maxG=40, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 1417.49974 / 1574.99971 / 1732.49969 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.0227699112 / 0.0237699112 / 0.0247699112 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### r-73

Parameter projection SHA-256: `fc17ca9aa7da39726476521b9c96c11516225361ffb9beb25b82d1145fc23d8f`. Plant SHA-256: `1f683d7303285ceed50d276ca09e904b59b51c13d9047f6f34c3bf862c278933`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass105.000016kg requires3675.00055N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=0deg, maxG=40, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 3307.5005 / 3675.00055 / 4042.50061 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.0227699112 / 0.0237699112 / 0.0247699112 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### r-77m

Parameter projection SHA-256: `a51a5dd91dacc55092705648a090910a7cfacb75adbef4beab297bb59e504ea7`. Plant SHA-256: `9d0e52c06857d82e55fe03c5cbb147ae31fff731f66ec41fc4e997e2ca03dac2`.

- radar measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass189.999983kg requires9974.99909N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=20deg, maxG=35, updates=1; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 47.25 / 52.5 / 57.75 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 8977.49918 / 9974.99909 / 10972.499 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

### super-530d

Parameter projection SHA-256: `beef5eab461a26e157cb837a0b62920596e4290d3d48d6029c254dd9d9702c59`. Plant SHA-256: `48195a0aa0d0193d56a976539a58882005de1a231fdb14b0e891160af2af9120`.

- bearing measurement binding with actual Ts=0.02s and processing=0.01s; no universal synthetic closing observation
- Actual launch mass270.000001kg requires9450.00003N for the declared PN example; physical trim authority and receiver delay are separate limits
- Loft=20deg, maxG=35, updates=0; exact per-role basis/uncertainty retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- navigation_gain: 31.5 / 35 / 38.5 m/s2 at -10% / nominal / +10%. Fixed same LOS rate/scale, unsaturated; sensitivity is not OEM uncertainty.
- available_scale_speed: 8505.00003 / 9450.00003 / 10395 N at -10% / nominal / +10%. Fixed launch mass and same LOS rate.
- processing_delay: 0.019 / 0.02 / 0.021 s at -10% / nominal / +10%. Heuristic mean lag, not actual capture epoch.

## Reproduction and proof separation

Run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-write-per-model-six-theory-20261103.py` with the recorded DENV environment. It reads explicit 24 catalogue/card/TOML/native XML paths and engine references, rechecks source hashes, and writes only the 13 assigned document files. Then run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-check-per-model-six-theory-20261103.py`. The adjacent `per-model-six-components.json` contains all 144 rows and full actual plant derivations. The generation script retains historical comparisons without wrapping them again.

The JSON deliberately sets current `passed`, numerical comparison and native acceptance false. A source-derived sensitivity or synthetic method benchmark cannot replace current model-native captures. No scenario, gain, threshold, provider, geometry or runtime code was changed for this projection.
