# Per-model allocator theory and physical applicability

This source-bound package contains 24 distinct model projections. It preserves actual native geometry, mass/CG/inertia, receiver capability, actuator limits and engine inputs. It does not admit current numerical/native encounter acceptance, hardware calibration or superiority over DCS. Historical executed comparisons remain in the JSON under their original source/card epochs.

## Governing method

`delta_fin=clip(G*delta_virtual,fin_stops); physical moment=M_nonfin+sum_i M_i(delta_fin_i)`

`local B_i=d(moment_i)/d(delta_i); angular effectiveness=diag(J)^-1*B*G, not fin count or provider name`

`Mixed-fin saturation/rate invalidates unconstrained inverse/channel equivalence; rank deficiency means genuine unavailable authority`

## State, I/O and execution

Input: three signed virtual channel angles, actual authored mixer and each physical fin travel/rate/failure. Output: finite fin demands through native FCS. Channel mixing is algebraic; subsequent native actuator state persists. Static matrix output is not an optimal saturated allocation. Passive fin angles are gyro-driven observations, not additional normalized controls.

## Assumptions and validity

- Authored deterministic mixer does not redistribute saturated fins as a constrained optimal allocator would
- No command or active authority is invented for passive rollerons. Rank/singular values apply only to declared state and angle segment
- High-angle surface interaction and finite-rate simultaneous demand can reduce authority below single-axis static estimates

The declared analysis point is 500m/s at 6000m, zero incidence/sideslip, standard tropospheric air and launch/half/depleted propellant states. It is an offline projection of actual serialized equations. Positive local poles would be a defect indicator; negative local poles do not establish finite delayed nonlinear encounter stability. The source parameter basis and uncertainty are preserved; the +/-10% sweeps are sensitivity, not fitted confidence intervals.

## Primary references

- [Harkegard, Efficient active set algorithms for solving constrained least squares problems in aircraft control allocation, 2002](https://rt.isy.liu.se/research/reports/2002/2426.pdf): Section2 equations1-4: effectiveness matrix and finite actuator bounds; comparison framework, not claim that authored mixer solves QP

## Actual model deltas

| Model | Native mass kg | Jyy kg m2 | Mechanism | Pitch local max real pole 1/s |
|---|---:|---:|---|---:|
| aim-120c | 161.5 | 180.688 | fsim.allocator.jsbsim_authored_fin_mixer | -4.1654 |
| aim-120d | 161.5 | 180.689 | fsim.allocator.jsbsim_authored_fin_mixer | -4.16195 |
| aim-7m | 225 | 248.99 | fsim.allocator.jsbsim_authored_fin_mixer | -3.33602 |
| aim-9m | 85.5 | 58.8201 | fsim.allocator.jsbsim_authored_fin_mixer | -1.6915 |
| aim-9x | 84 | 63.9021 | fsim.allocator.jsbsim_authored_fin_mixer | -2.13191 |
| asraam | 88 | 61.8117 | fsim.allocator.jsbsim_authored_fin_mixer | -2.49357 |
| iris-t | 87.4 | 62.9245 | fsim.allocator.jsbsim_authored_fin_mixer | -1.79029 |
| magic-2 | 90 | 56.8461 | fsim.allocator.jsbsim_authored_fin_mixer | -1.8692 |
| meteor | 190 | 217.246 | fsim.allocator.jsbsim_authored_fin_mixer | -4.21734 |
| mica-em | 112 | 89.8958 | fsim.allocator.jsbsim_authored_fin_mixer | -2.23237 |
| mica-ir | 112 | 89.8958 | fsim.allocator.jsbsim_authored_fin_mixer | -2.3158 |
| pl-10 | 89 | 66.9008 | fsim.allocator.jsbsim_authored_fin_mixer | -2.5 |
| pl-12 | 180 | 232.194 | fsim.allocator.jsbsim_authored_fin_mixer | -3.27688 |
| pl-15 | 210 | 280.632 | fsim.allocator.jsbsim_authored_fin_mixer | -4.77315 |
| pl-8 | 115 | 83.5867 | fsim.allocator.jsbsim_authored_fin_mixer | -1.66724 |
| r-27er | 350 | 667.91 | fsim.allocator.jsbsim_authored_fin_mixer | -3.79635 |
| r-27et | 348 | 585.959 | fsim.allocator.jsbsim_authored_fin_mixer | -3.19681 |
| r-27r | 253 | 351.753 | fsim.allocator.jsbsim_authored_fin_mixer | -5.51097 |
| r-27t | 253 | 304.325 | fsim.allocator.jsbsim_authored_fin_mixer | -4.31654 |
| r-37m | 510 | 702.132 | fsim.allocator.jsbsim_authored_fin_mixer | -6.00102 |
| r-60m | 45 | 17.0837 | fsim.allocator.jsbsim_authored_fin_mixer | -2.91192 |
| r-73 | 105 | 75.2869 | fsim.allocator.jsbsim_authored_fin_mixer | -5.19525 |
| r-77m | 190 | 218.423 | fsim.allocator.jsbsim_authored_fin_mixer | -3.04367 |
| super-530d | 270 | 325.63 | fsim.allocator.jsbsim_authored_fin_mixer | -3.97841 |

### aim-120c

Parameter projection SHA-256: `847ed2a9b72481601341b94ee75104af8cfbee7130615173fbb044f4e019e436`. Plant SHA-256: `550d9dc2489bc5356abea09b35f859ca9a9ab76a61e48a32e6f49af1763591d8`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[3271.4222449118183, 93.0205595332224, 93.01746834203188]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in aim-120c actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 83.7185036 / 93.0205595 / 102.322615 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 103.356177 / 93.0205595 / 84.564145 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### aim-120d

Parameter projection SHA-256: `6613da0d0118c4501b0a990bf5d1fcd58d271ac8edfd687c082bcbac6b956264`. Plant SHA-256: `3ce42309096c29790d6d45eb6098b3e35b31b36b8eca4790cf187f95d636c7a5`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[3271.4233753442336, 93.02036084662367, 93.01726966205685]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in aim-120d actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 83.7183248 / 93.0203608 / 102.322397 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 103.355956 / 93.0203608 / 84.5639644 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### aim-7m

Parameter projection SHA-256: `d09e01f46f47474f539ed4c414e367a41389726804c1d815c2e9a2a2a4591593`. Plant SHA-256: `75c07167eba4f3be6fa48df961b72c09c07119ed94b9da73244da936e1b6f160`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[8346.12008359023, 133.95064696885575, 133.82090042447223]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in aim-7m actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 120.555582 / 133.950647 / 147.345712 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 148.834052 / 133.950647 / 121.773315 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### aim-9m

Parameter projection SHA-256: `608daef6fabedd4ab0cba071f1c907038f29dc5a63225f6e16b9974df75528ed`. Plant SHA-256: `2519f562e95ec92b6c69bd6a6f6267b28d269c61164fd0bfe777e13d6bd08f84`.

- 8physical fins; 4passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=2, singular values=[183.76211094734762, 183.65605257544112, 0.0]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in aim-9m actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 165.3859 / 183.762111 / 202.138322 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 204.180123 / 183.762111 / 167.056464 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### aim-9x

Parameter projection SHA-256: `57d4ba926bde6b27b7f388d19d35c01713fc19fc07c852b91600e5b2597306f0`. Plant SHA-256: `aca7c078fce2c1bef2c78273b0497a00dd90b1edd5514be72fe8108e95625688`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[9789.187595021958, 200.90587313412118, 200.89338434805535]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in aim-9x actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 180.815286 / 200.905873 / 220.99646 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 223.228748 / 200.905873 / 182.641703 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### asraam

Parameter projection SHA-256: `2555458f20ee7e0e76b6de8587583a1f54730f3ad2c43e3b5d13acf8027674df`. Plant SHA-256: `a7c76134c340f7e36c0000fd1f2ecde32f6ae4c2a40b1a111caec03ba163c699`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[5958.201314172806, 197.07204322968224, 197.06591380922575]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in asraam actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 177.364839 / 197.072043 / 216.779248 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 218.968937 / 197.072043 / 179.156403 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### iris-t

Parameter projection SHA-256: `e41c61457a5b37e92755c33246fcf85005a653a6b2a39f5c6acebc2d2a38b7d8`. Plant SHA-256: `a0f0853283b555c6db8fd9c94bc50b20dec29ca3727611a2200413381022aa04`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[10957.962534429022, 236.20204419162494, 236.18411625669262]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in iris-t actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 212.58184 / 236.202044 / 259.822249 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 262.446716 / 236.202044 / 214.729131 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### magic-2

Parameter projection SHA-256: `e7c689b580572413b461f361f73873f4dd1dec2e49f9c864cefb661cfef7c352`. Plant SHA-256: `dd3d2519d2b59d3d3a5b3e6cff82ac9aeaf665ba84999682cb5ab154af5f9bab`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=2, singular values=[186.19008874402772, 186.16275675021996, 0.0]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in magic-2 actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 167.57108 / 186.190089 / 204.809098 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 206.877876 / 186.190089 / 169.263717 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### meteor

Parameter projection SHA-256: `29e47a7d9121f40dc400c8a6c519cc7f4b0c9e0568a13a853184c1ac746e85c2`. Plant SHA-256: `02c922fa807029d53ce55ec354be2a763fbddf3f4fa87a5fcc5d5beadedb6718`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[1614.2566220288545, 50.03785246452148, 50.03744353276858]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in meteor actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 45.0340672 / 50.0378525 / 55.0416377 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 55.5976138 / 50.0378525 / 45.4889568 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### mica-em

Parameter projection SHA-256: `722e351284247886ca63e09063b6acf7cd08933166d38f5047de6bafde117bf9`. Plant SHA-256: `9e36c380bc4070d60a315639b68a48054afd2ab642d2f6fc84b1bea65a61640b`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[6127.730105193734, 173.32694355125358, 173.3204664420541]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in mica-em actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 155.994249 / 173.326944 / 190.659638 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 192.585493 / 173.326944 / 157.569949 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### mica-ir

Parameter projection SHA-256: `a0556ee4215c56c65fcf1a1fde516b289b7957d4e7a4fea1624649b9e7536223`. Plant SHA-256: `1821db51276f0afb37cb0da2295ceeaeb6d62d9729deef6f5bf7203e80f38c23`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[6130.335656030178, 173.51193988070696, 173.5054537143187]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in mica-ir actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 156.160746 / 173.51194 / 190.863134 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 192.791044 / 173.51194 / 157.738127 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### pl-10

Parameter projection SHA-256: `0f1d2dab1a6f93acd9a74b7bb361c1f0e4ba5f9ea83dee75e50e8343e0528be8`. Plant SHA-256: `c410ed7e34ed47054a887ac352349a28962c0b92f0b332e3d7d77699af084a90`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[4170.600139510537, 136.06014231276532, 136.05704151386973]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in pl-10 actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 122.454128 / 136.060142 / 149.666157 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 151.177936 / 136.060142 / 123.691038 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### pl-12

Parameter projection SHA-256: `c01be530c24afee52ea948de75765515b56aaf6a8e5caac2103da9f96674fb83`. Plant SHA-256: `e4c01fcbe3bbf27a19795c5f6bdc93158f6dbc2b8322da6510c9bcb94848e569`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[5929.520437395298, 157.51313573105134, 157.5051509014725]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in pl-12 actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 141.761822 / 157.513136 / 173.264449 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 175.014595 / 157.513136 / 143.19376 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### pl-15

Parameter projection SHA-256: `28ff310d65b2a1cf5945861442aa142dcd30dd450c358fc56aa8f9e58404ee43`. Plant SHA-256: `2195e1957c015ca4015c4c11a0c12fd2d91ff2623d15a9b3970d92b3ca3c594b`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[839.9797752884708, 28.066364342929827, 28.0659989707908]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in pl-15 actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 25.2597279 / 28.0663643 / 30.8730008 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 31.1848493 / 28.0663643 / 25.5148767 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### pl-8

Parameter projection SHA-256: `0b4d22f19364f9f85dbfd19aca8a2fbcb36573ffb792484cd407775a0e3b17d8`. Plant SHA-256: `bdbca3627ca78563b6f9672487ecb0e54f14e21b1fbdf4c1408893d4295dbd38`.

- 8physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[8740.851148054013, 113.05487706645965, 112.94978320012942]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in pl-8 actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 101.749389 / 113.054877 / 124.360365 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 125.61653 / 113.054877 / 102.777161 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### r-27er

Parameter projection SHA-256: `a315f6e677c51379cf4ebf9c52c1fcc5cba5a64f7aa9a2f376657289d9620ff3`. Plant SHA-256: `470f960a5e26febb47bc625e498f7883693bf270c7ebb78db6458d4ef1f45520`.

- 8physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[3907.0470225118306, 98.16400567818731, 98.15732815867563]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in r-27er actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 88.3476051 / 98.1640057 / 107.980406 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 109.071117 / 98.1640057 / 89.2400052 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### r-27et

Parameter projection SHA-256: `2557e04779dd782d9b41c7907e74d686cb94dfc38a9b46814539770ba2814a00`. Plant SHA-256: `3449c4f07b6a23fca3c7d0da91b02b59c5eaa29a27e383c767733b21f25f418d`.

- 8physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[3898.61928118382, 118.59063819732226, 118.58288972780483]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in r-27et actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 106.731574 / 118.590638 / 130.449702 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 131.767376 / 118.590638 / 107.809671 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### r-27r

Parameter projection SHA-256: `660bdc968f995282cc12c6d871ccefb483c99dd2a1bbd5309c3df52bb49bc75a`. Plant SHA-256: `b427f8d0361a2ac66cc060c7e4ab20e55160d3cff00855690ab948555ac448cd`.

- 8physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[6246.629566622892, 138.594257463037, 138.581658514449]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in r-27r actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 124.734832 / 138.594257 / 152.453683 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 153.993619 / 138.594257 / 125.99478 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### r-27t

Parameter projection SHA-256: `785d12d0cef03ba11a222f62f9a95a3a4784b69d63591ecf748e09b036becb94`. Plant SHA-256: `d91746d1a779f3862acec146456475e274563d57b21d9b6d51e2e34f901c0492`.

- 8physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[6279.180688591802, 170.0544421835865, 170.04004722508066]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in r-27t actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 153.048998 / 170.054442 / 187.059886 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 188.94938 / 170.054442 / 154.594947 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### r-37m

Parameter projection SHA-256: `94ff3f1fdc29bc70cfbe8603aec47f2762da4bc860068bfb486ae2e65ca4b7c6`. Plant SHA-256: `a480640a3ada7b9437ff4b78418ea98d9813f51856f97f707908b8870dee154c`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[331.9675839058888, 21.33964668924245, 21.339565481830213]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in r-37m actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 19.205682 / 21.3396467 / 23.4736114 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 23.7107185 / 21.3396467 / 19.3996788 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### r-60m

Parameter projection SHA-256: `b04441c47382de36ea798d019595690429eda1eb92ec1367bf7dcc74299dd4d8`. Plant SHA-256: `5773bc316905806dd04d1b4b85aa6199e1704e63f7f86d8904befe9e21f7077c`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=2, singular values=[132.14365892203614, 132.1257137739618, 0.0]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in r-60m actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 118.929293 / 132.143659 / 145.358025 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 146.826288 / 132.143659 / 120.130599 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### r-73

Parameter projection SHA-256: `f412fdcc62a3a7ab82f4c964edd9030b0a5a58bafe8f3998e58ab84211a50e43`. Plant SHA-256: `1f683d7303285ceed50d276ca09e904b59b51c13d9047f6f34c3bf862c278933`.

- 8physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[2320.997277557536, 132.52643295975537, 132.50598566431825]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in r-73 actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 119.27379 / 132.526433 / 145.779076 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 147.251592 / 132.526433 / 120.478575 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### r-77m

Parameter projection SHA-256: `f4676b604f6526e6339009a7160e57aaeb566fc96e54ff2ed230648de511adac`. Plant SHA-256: `9d0e52c06857d82e55fe03c5cbb147ae31fff731f66ec41fc4e997e2ca03dac2`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[2584.9077453775762, 74.89375669610564, 74.89302526177876]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in r-77m actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 67.404381 / 74.8937567 / 82.3831324 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 83.2152852 / 74.8937567 / 68.0852334 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

### super-530d

Parameter projection SHA-256: `f14816489bb552452af9328c9f3c81d6b651804483f83d02df0b4e80c985d821`. Plant SHA-256: `48195a0aa0d0193d56a976539a58882005de1a231fdb14b0e891160af2af9120`.

- 4physical fins; 0passive rollerons; actualG and piecewisefinmomentJacobian retained
- Angular map rank=3, singular values=[1727.757096469997, 73.4910036230895, 73.49001815578342]; fixed point/rad units, not normalized command authority
- Geometry surface leading edges/chords/control span/pivot/limits retained in super-530d actual plant record; no same-fin-count equivalence claim

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- physical_moment_effectiveness: 66.1419033 / 73.4910036 / 80.840104 rad/s2 pervirtualrad at -10% / nominal / +10%. Same G and actual J.
- inertia_at_fixed_moment: 81.6566707 / 73.4910036 / 66.8100033 rad/s2 pervirtualrad at -10% / nominal / +10%. Not a mass-only identity; explicit J sensitivity.

## Reproduction and proof separation

Run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-write-per-model-six-theory-20261103.py` with the recorded DENV environment. It reads explicit 24 catalogue/card/TOML/native XML paths and engine references, rechecks source hashes, and writes only the 13 assigned document files. Then run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-check-per-model-six-theory-20261103.py`. The adjacent `per-model-six-components.json` contains all 144 rows and full actual plant derivations. The generation script retains historical comparisons without wrapping them again.

The JSON deliberately sets current `passed`, numerical comparison and native acceptance false. A source-derived sensitivity or synthetic method benchmark cannot replace current model-native captures. No scenario, gain, threshold, provider, geometry or runtime code was changed for this projection.
