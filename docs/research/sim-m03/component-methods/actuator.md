# Per-model actuator theory and physical applicability

This source-bound package contains 24 distinct model projections. It preserves actual native geometry, mass/CG/inertia, receiver capability, actuator limits and engine inputs. It does not admit current numerical/native encounter acceptance, hardware calibration or superiority over DCS. Historical executed comparisons remain in the JSON under their original source/card epochs.

## Governing method

`y_lag[k]=h/(2*tau+h)*(u[k]+u[k-1])+(2*tau-h)/(2*tau+h)*y_lag[k-1]`

`native downstream rate clamp |delta[k]-delta[k-1]|<=rate*h, then physical stops and malfunctions`

`passive wheel: Iw*wheel_dot=T_air-(Dwind+Dbearing)*wheel; hinge: Ih*angle_ddot=-Keff*angle-Dh*angle_dot-Iw*wheel*roll_rate`

`passive hinge/wheel states have no driven command/malfunction ports; aerodynamic and gyroscopic body reactions remain native`

## State, I/O and execution

Driven state: previous input, lag state and actual angle; native substep updates lag, malfunction, rate and stops in source order. Passive state: wheel speed, hinge angle/rate, initialized/reset separately by the physical rolleron model. TVC has independent finite angle/rate and requires actual thrust. Missing command/malfunction ports on passive fins are unavailable, not fabricated zero.

## Assumptions and validity

- Bilinear lag has finite sample and nonlinear limit ordering; matching a continuous time constant does not prove full discrete-loop equivalence
- Passive gyro closure is a sourced mechanism with assumed physical coefficients; no equipment wheel/hinge calibration
- TVC has separate angle/rate and thrust-dependent authority; depleted motor cannot be credited powered torque

The declared analysis point is 500m/s at 6000m, zero incidence/sideslip, standard tropospheric air and launch/half/depleted propellant states. It is an offline projection of actual serialized equations. Positive local poles would be a defect indicator; negative local poles do not establish finite delayed nonlinear encounter stability. The source parameter basis and uncertainty are preserved; the +/-10% sweeps are sensitivity, not fitted confidence intervals.

## Primary references

- [SciPy primary cont2discrete documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.cont2discrete.html): Bilinear transformation versus ZOH; actual native FGActuator source is authoritative
- [JSBSim FGActuator source](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/flight_control/FGActuator.cpp): Native lag recurrence, downstream rate/position limits and malfunction ordering; vendored source hash bound separately
- [NACA RM SL55C22, Rolleron roll stabilization of the Sidewinder missile, 1955](https://ntrs.nasa.gov/api/citations/20090023599/downloads/20090023599.pdf): Historic gyro-hinge stabilization principle and flight-test domain; no transfer of historical constants to modern models

## Actual model deltas

| Model | Native mass kg | Jyy kg m2 | Mechanism | Pitch local max real pole 1/s |
|---|---:|---:|---|---:|
| aim-120c | 161.5 | 180.688 | fsim.actuator.jsbsim_lag_rate_stops | -4.1654 |
| aim-120d | 161.5 | 180.689 | fsim.actuator.jsbsim_lag_rate_stops | -4.16195 |
| aim-7m | 225 | 248.99 | fsim.actuator.jsbsim_lag_rate_stops | -3.33602 |
| aim-9m | 85.5 | 58.8201 | fsim.actuator.jsbsim_lag_rate_stops | -1.6915 |
| aim-9x | 84 | 63.9021 | fsim.actuator.jsbsim_lag_rate_stops | -2.13191 |
| asraam | 88 | 61.8117 | fsim.actuator.jsbsim_lag_rate_stops | -2.49357 |
| iris-t | 87.4 | 62.9245 | fsim.actuator.jsbsim_lag_rate_stops | -1.79029 |
| magic-2 | 90 | 56.8461 | fsim.actuator.jsbsim_lag_rate_stops | -1.8692 |
| meteor | 190 | 217.246 | fsim.actuator.jsbsim_lag_rate_stops | -4.21734 |
| mica-em | 112 | 89.8958 | fsim.actuator.jsbsim_lag_rate_stops | -2.23237 |
| mica-ir | 112 | 89.8958 | fsim.actuator.jsbsim_lag_rate_stops | -2.3158 |
| pl-10 | 89 | 66.9008 | fsim.actuator.jsbsim_lag_rate_stops | -2.5 |
| pl-12 | 180 | 232.194 | fsim.actuator.jsbsim_lag_rate_stops | -3.27688 |
| pl-15 | 210 | 280.632 | fsim.actuator.jsbsim_lag_rate_stops | -4.77315 |
| pl-8 | 115 | 83.5867 | fsim.actuator.jsbsim_lag_rate_stops | -1.66724 |
| r-27er | 350 | 667.91 | fsim.actuator.jsbsim_lag_rate_stops | -3.79635 |
| r-27et | 348 | 585.959 | fsim.actuator.jsbsim_lag_rate_stops | -3.19681 |
| r-27r | 253 | 351.753 | fsim.actuator.jsbsim_lag_rate_stops | -5.51097 |
| r-27t | 253 | 304.325 | fsim.actuator.jsbsim_lag_rate_stops | -4.31654 |
| r-37m | 510 | 702.132 | fsim.actuator.jsbsim_lag_rate_stops | -6.00102 |
| r-60m | 45 | 17.0837 | fsim.actuator.jsbsim_lag_rate_stops | -2.91192 |
| r-73 | 105 | 75.2869 | fsim.actuator.jsbsim_lag_rate_stops | -5.19525 |
| r-77m | 190 | 218.423 | fsim.actuator.jsbsim_lag_rate_stops | -3.04367 |
| super-530d | 270 | 325.63 | fsim.actuator.jsbsim_lag_rate_stops | -3.97841 |

### aim-120c

Parameter projection SHA-256: `2cac31a21b5817e72b0569cae74897de0a6bdf40ff781b59683a8d60cbb8eba6`. Plant SHA-256: `550d9dc2489bc5356abea09b35f859ca9a9ab76a61e48a32e6f49af1763591d8`.

- 55native internal steps per 1/120s World tick; Tustin pole=0.996219281664 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### aim-120d

Parameter projection SHA-256: `08b4e87e40cd9b8794e92077ca46da32abbcbe9267e6c14949f98cd3f55b9c99`. Plant SHA-256: `3ce42309096c29790d6d45eb6098b3e35b31b36b8eca4790cf187f95d636c7a5`.

- 55native internal steps per 1/120s World tick; Tustin pole=0.996219281664 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### aim-7m

Parameter projection SHA-256: `2fbf03e3afd79f9c6b5257bb1d543d84ba7d8770d0662801956786dd3d6c71d6`. Plant SHA-256: `75c07167eba4f3be6fa48df961b72c09c07119ed94b9da73244da936e1b6f160`.

- 71native internal steps per 1/120s World tick; Tustin pole=0.99707002637 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### aim-9m

Parameter projection SHA-256: `c32f1af95d1a94960281a2ef1801e6dbf7d496ea34dc837631eb275b3d376320`. Plant SHA-256: `2519f562e95ec92b6c69bd6a6f6267b28d269c61164fd0bfe777e13d6bd08f84`.

- 106native internal steps per 1/120s World tick; Tustin pole=0.998036520715 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Passive gyro hinge and wheel dynamics are separate from the driven-fin contract; actual properties are retained

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.
- passive_wheel_inertia_at_steady_wheel_speed: -0.00794117815 / -0.00882353127 / -0.0097058844 s at -10% / nominal / +10%. Gyro hinge angle per imposed roll rate; drive/bearing set equilibrium wheel speed, stiffness fixed; no driven command.

### aim-9x

Parameter projection SHA-256: `61cd4fccdd8ca2fad2ea27ca686c2cc97ef245a42a6d959c5a920d308a1482ac`. Plant SHA-256: `aca7c078fce2c1bef2c78273b0497a00dd90b1edd5514be72fe8108e95625688`.

- 83native internal steps per 1/120s World tick; Tustin pole=0.997493106042 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### asraam

Parameter projection SHA-256: `7c6131bd0521b4a1548e7cc1324e45231489e3368998d3133d76348314ba94df`. Plant SHA-256: `a7c76134c340f7e36c0000fd1f2ecde32f6ae4c2a40b1a111caec03ba163c699`.

- 66native internal steps per 1/120s World tick; Tustin pole=0.996848408446 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### iris-t

Parameter projection SHA-256: `af007c83d454d6a42d28e756cb18a2c74026c69793faf0060558dc4ebc0c4572`. Plant SHA-256: `a0f0853283b555c6db8fd9c94bc50b20dec29ca3727611a2200413381022aa04`.

- 130native internal steps per 1/120s World tick; Tustin pole=0.998398718975 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### magic-2

Parameter projection SHA-256: `9047796a543a9a4665941e0cd896c5dcafeacf3f94d37a1bca7e27ac389e98f5`. Plant SHA-256: `dd3d2519d2b59d3d3a5b3e6cff82ac9aeaf665ba84999682cb5ab154af5f9bab`.

- 88native internal steps per 1/120s World tick; Tustin pole=0.997635374793 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### meteor

Parameter projection SHA-256: `b14f688ff5a2ccd76f3f0a04c93049ebeeee559a07a4a52fa6a34f348bc2a680`. Plant SHA-256: `02c922fa807029d53ce55ec354be2a763fbddf3f4fa87a5fcc5d5beadedb6718`.

- 49native internal steps per 1/120s World tick; Tustin pole=0.995757318625 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### mica-em

Parameter projection SHA-256: `6828eae79c8bd43db3f6d16a3a22c48468e6abd33883ba45d9015808019605c9`. Plant SHA-256: `9e36c380bc4070d60a315639b68a48054afd2ab642d2f6fc84b1bea65a61640b`.

- 104native internal steps per 1/120s World tick; Tustin pole=0.99799879928 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### mica-ir

Parameter projection SHA-256: `85132a7075b1b8a38873fdbb0ab5438b03d19674131fc33455607a05acb0c51e`. Plant SHA-256: `1821db51276f0afb37cb0da2295ceeaeb6d62d9729deef6f5bf7203e80f38c23`.

- 104native internal steps per 1/120s World tick; Tustin pole=0.99799879928 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### pl-10

Parameter projection SHA-256: `ebcdaa742b1533176c6fcc69099c3e9478a792df0a8d6a3f29104cf1c5df6a41`. Plant SHA-256: `c410ed7e34ed47054a887ac352349a28962c0b92f0b332e3d7d77699af084a90`.

- 95native internal steps per 1/120s World tick; Tustin pole=0.997809419496 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### pl-12

Parameter projection SHA-256: `c71b3099e3301a88e0dd5063ffaa2ef22799913a827cfb1f472f6d048996a561`. Plant SHA-256: `e4c01fcbe3bbf27a19795c5f6bdc93158f6dbc2b8322da6510c9bcb94848e569`.

- 57native internal steps per 1/120s World tick; Tustin pole=0.996351696461 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### pl-15

Parameter projection SHA-256: `73b92fb1d697a36252abde1e25f370483e562d3405bc930d853abb360458563e`. Plant SHA-256: `2195e1957c015ca4015c4c11a0c12fd2d91ff2623d15a9b3970d92b3ca3c594b`.

- 45native internal steps per 1/120s World tick; Tustin pole=0.995381062356 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### pl-8

Parameter projection SHA-256: `cc3189ac1a040bf9a354e48fd9d614c26e09e0fc3c4937ea9b8bd292fe12e371`. Plant SHA-256: `bdbca3627ca78563b6f9672487ecb0e54f14e21b1fbdf4c1408893d4295dbd38`.

- 101native internal steps per 1/120s World tick; Tustin pole=0.997939418916 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### r-27er

Parameter projection SHA-256: `04acee6a365c9021e6fd597ed12b1242853fae74766cda0544a639c9b5a3a3cf`. Plant SHA-256: `470f960a5e26febb47bc625e498f7883693bf270c7ebb78db6458d4ef1f45520`.

- 50native internal steps per 1/120s World tick; Tustin pole=0.995841995842 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### r-27et

Parameter projection SHA-256: `ca32950bea8b5f50b68326bdab88b947edd4ef4782c5e03b6b89f601b88419ad`. Plant SHA-256: `3449c4f07b6a23fca3c7d0da91b02b59c5eaa29a27e383c767733b21f25f418d`.

- 50native internal steps per 1/120s World tick; Tustin pole=0.995841995842 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### r-27r

Parameter projection SHA-256: `665dd98476039a4bc3da6ff12f53dc1c1855805a6c8ef08ce850c4b080178940`. Plant SHA-256: `b427f8d0361a2ac66cc060c7e4ab20e55160d3cff00855690ab948555ac448cd`.

- 55native internal steps per 1/120s World tick; Tustin pole=0.996219281664 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### r-27t

Parameter projection SHA-256: `925be4158d5482f8c36005f78470827f831f9dbef2687530917adb0012c7dfc5`. Plant SHA-256: `d91746d1a779f3862acec146456475e274563d57b21d9b6d51e2e34f901c0492`.

- 55native internal steps per 1/120s World tick; Tustin pole=0.996219281664 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### r-37m

Parameter projection SHA-256: `13ab7e7d43da56a1db99df81242e846b2474f9ac5c8740f1ee25ec4dbb699652`. Plant SHA-256: `a480640a3ada7b9437ff4b78418ea98d9813f51856f97f707908b8870dee154c`.

- 40native internal steps per 1/120s World tick; Tustin pole=0.994805194805 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### r-60m

Parameter projection SHA-256: `1429baff1db7736318f6543a09951361898afcfadaa50594109b1f31064b9649`. Plant SHA-256: `5773bc316905806dd04d1b4b85aa6199e1704e63f7f86d8904befe9e21f7077c`.

- 91native internal steps per 1/120s World tick; Tustin pole=0.997713240338 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### r-73

Parameter projection SHA-256: `6a4c9fb40242d33a40e2add453b266ca6d108b93724015f972c0604c8907d005`. Plant SHA-256: `1f683d7303285ceed50d276ca09e904b59b51c13d9047f6f34c3bf862c278933`.

- 68native internal steps per 1/120s World tick; Tustin pole=0.996940960538 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### r-77m

Parameter projection SHA-256: `22353a03bc6eeb59f304d0e40675073be851f79c609016a758e3fb4fc82f5d43`. Plant SHA-256: `9d0e52c06857d82e55fe03c5cbb147ae31fff731f66ec41fc4e997e2ca03dac2`.

- 68native internal steps per 1/120s World tick; Tustin pole=0.996940960538 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

### super-530d

Parameter projection SHA-256: `e33ba0eda572c2369ec4faf39ddeb69f936a0a265adf85b94b0dcf4ad0029a62`. Plant SHA-256: `48195a0aa0d0193d56a976539a58882005de1a231fdb14b0e891160af2af9120`.

- 41native internal steps per 1/120s World tick; Tustin pole=0.99493157628 differs from e^(-h/tau)
- Physical rate-only full-reversal bound 0.174532925s; mixed fin rates/stops subsequent to continuous lag remain essential
- Driven finite FGActuator fins; no passive gyro state invented

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- lag_at_fixed_time_tau_nominal: 0.46831164 / 0.441303401 / 0.416861195 rad at -10% / nominal / +10%. Unsaturated continuous reference; no rate clamp.
- physical_rate: 0.193925472 / 0.174532925 / 0.158666296 s at -10% / nominal / +10%. Full reversal rate-only lower bound.

## Reproduction and proof separation

Run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-write-per-model-six-theory-20261103.py` with the recorded DENV environment. It reads explicit 24 catalogue/card/TOML/native XML paths and engine references, rechecks source hashes, and writes only the 13 assigned document files. Then run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-check-per-model-six-theory-20261103.py`. The adjacent `per-model-six-components.json` contains all 144 rows and full actual plant derivations. The generation script retains historical comparisons without wrapping them again.

The JSON deliberately sets current `passed`, numerical comparison and native acceptance false. A source-derived sensitivity or synthetic method benchmark cannot replace current model-native captures. No scenario, gain, threshold, provider, geometry or runtime code was changed for this projection.
