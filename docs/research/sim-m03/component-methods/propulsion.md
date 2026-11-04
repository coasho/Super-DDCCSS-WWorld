# Per-model propulsion theory and physical applicability

This source-bound package contains 24 distinct model projections. It preserves actual native geometry, mass/CG/inertia, receiver capability, actuator limits and engine inputs. It does not admit current numerical/native encounter acceptance, hardware calibration or superiority over DCS. Historical executed comparisons remain in the JSON under their original source/card epochs.

## Governing method

`F_vac=Isp*g0*mdot_nominal; F_net=F_vac-A_exit*p_ambient`

`finite-feed opt-in: request=F_vac*dt/(Isp*g0); same native priority/equal-share feed, supply=sum min(share,pre_usable_tank); F_vac_actual=Isp*g0*supply/dt unless full supply preserves exact nominal force`

`DeltaV_ideal=Isp*g0*ln(m_launch/m_depleted), excluding drag/gravity; airbreathing sustainer is not equivalent to a solid rocket`

`ducted: mdot_air=rho*V*A_capture above lightoff; burntFuel=min(mdot_fuel,FARmax*mdot_air); F=(mdot_air+mdot_fuel)*V_exit-mdot_air*V; valve and inlet residence are finite states`

## State, I/O and execution

Input: native throttle/ignition/pulse state, ambient pressure, same-step eligible tank contents/priority and finite timestep. Output: native thrust/nozzle force, requested flow separately from supplied batch, native tank debit and starved state. Solid remaining-mass curve, commanded pulse clock, Meteor valve/inlet resident air and consumable fuel are distinct states. Opt-in feed closes supply before force and avoids a second debit; legacy absent policy remains unchanged.

## Assumptions and validity

- Authored phase/Isp/nozzle/grain values retain each parameter basis/uncertainty; no unpublished OEM burn curve is claimed
- Ideal deltaV ignores drag/gravity/pressure and guidance work; it is not range or lethality
- Meteor cycle equilibrium sensitivity below is not finite-inlet transient validation; dual-pulse ignition is a state machine, not a new Isp

The declared analysis point is 500m/s at 6000m, zero incidence/sideslip, standard tropospheric air and launch/half/depleted propellant states. It is an offline projection of actual serialized equations. Positive local poles would be a defect indicator; negative local poles do not establish finite delayed nonlinear encounter stability. The source parameter basis and uncertainty are preserved; the +/-10% sweeps are sensitivity, not fitted confidence intervals.

## Primary references

- [NASA Glenn, Specific Impulse](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/specific-impulse/): F=mdot*Ve+Ae*(pe-pa), Isp=F/(mdot*g0), total impulse integral
- [NASA Glenn, Ideal Rocket Equation](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-rocket-equation/): DeltaV=Isp*g0*ln(m0/mf), ideal unforced vacuum upper comparison, not native range
- [NASA Glenn, Ramjet/Scramjet thrust](https://www.grc.nasa.gov/www/k-12/airplane/ramth.html): Gross exhaust momentum minus captured-air ram drag; finite inlet/combustor engineering closure remains source-specific

## Actual model deltas

| Model | Native mass kg | Jyy kg m2 | Mechanism | Pitch local max real pole 1/s |
|---|---:|---:|---|---:|
| aim-120c | 161.5 | 180.688 | fsim.propulsion.jsbsim_generated_solid_rocket | -4.1654 |
| aim-120d | 161.5 | 180.689 | fsim.propulsion.jsbsim_generated_solid_rocket | -4.16195 |
| aim-7m | 225 | 248.99 | fsim.propulsion.jsbsim_generated_solid_rocket | -3.33602 |
| aim-9m | 85.5 | 58.8201 | fsim.propulsion.jsbsim_generated_solid_rocket | -1.6915 |
| aim-9x | 84 | 63.9021 | fsim.propulsion.jsbsim_generated_solid_rocket | -2.13191 |
| asraam | 88 | 61.8117 | fsim.propulsion.jsbsim_generated_solid_rocket | -2.49357 |
| iris-t | 87.4 | 62.9245 | fsim.propulsion.jsbsim_generated_solid_rocket | -1.79029 |
| magic-2 | 90 | 56.8461 | fsim.propulsion.jsbsim_generated_solid_rocket | -1.8692 |
| meteor | 190 | 217.246 | fsim.propulsion.jsbsim_ducted_rocket.v2 | -4.21734 |
| mica-em | 112 | 89.8958 | fsim.propulsion.jsbsim_generated_solid_rocket | -2.23237 |
| mica-ir | 112 | 89.8958 | fsim.propulsion.jsbsim_generated_solid_rocket | -2.3158 |
| pl-10 | 89 | 66.9008 | fsim.propulsion.jsbsim_generated_solid_rocket | -2.5 |
| pl-12 | 180 | 232.194 | fsim.propulsion.jsbsim_generated_solid_rocket | -3.27688 |
| pl-15 | 210 | 280.632 | fsim.propulsion.jsbsim_commanded_dual_pulse.v1 | -4.77315 |
| pl-8 | 115 | 83.5867 | fsim.propulsion.jsbsim_generated_solid_rocket | -1.66724 |
| r-27er | 350 | 667.91 | fsim.propulsion.jsbsim_generated_solid_rocket | -3.79635 |
| r-27et | 348 | 585.959 | fsim.propulsion.jsbsim_generated_solid_rocket | -3.19681 |
| r-27r | 253 | 351.753 | fsim.propulsion.jsbsim_generated_solid_rocket | -5.51097 |
| r-27t | 253 | 304.325 | fsim.propulsion.jsbsim_generated_solid_rocket | -4.31654 |
| r-37m | 510 | 702.132 | fsim.propulsion.jsbsim_commanded_dual_pulse.v1 | -6.00102 |
| r-60m | 45 | 17.0837 | fsim.propulsion.jsbsim_generated_solid_rocket | -2.91192 |
| r-73 | 105 | 75.2869 | fsim.propulsion.jsbsim_generated_solid_rocket | -5.19525 |
| r-77m | 190 | 218.423 | fsim.propulsion.jsbsim_generated_solid_rocket | -3.04367 |
| super-530d | 270 | 325.63 | fsim.propulsion.jsbsim_generated_solid_rocket | -3.97841 |

### aim-120c

Parameter projection SHA-256: `32f361bbc04352843ed7df5518064a53a1d31e41d7d12aa48bb3dffd34905206`. Plant SHA-256: `550d9dc2489bc5356abea09b35f859ca9a9ab76a61e48a32e6f49af1763591d8`.

- Native fuel=38.9156988kg/m0=161.499989kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 662.419774m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 596.177797 / 662.419774 / 728.661752 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 151.358799 / 136.222919 / 123.839017 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### aim-120d

Parameter projection SHA-256: `588c1ca182a0edd9b3f33776748e40a40cc96f98f96e91c4fa403715fe2f73b9`. Plant SHA-256: `3ce42309096c29790d6d45eb6098b3e35b31b36b8eca4790cf187f95d636c7a5`.

- Native fuel=41.8291771kg/m0=161.499998kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 720.2129m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 648.19161 / 720.2129 / 792.23419 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 151.358791 / 136.222912 / 123.83901 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### aim-7m

Parameter projection SHA-256: `f695b3617e42e21ca62af0e231d1b83a3d5d2d64b507f3f63c983e1f6a494d66`. Plant SHA-256: `75c07167eba4f3be6fa48df961b72c09c07119ed94b9da73244da936e1b6f160`.

- Native fuel=42.588146kg/m0=225kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 524.729335m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 472.256401 / 524.729335 / 577.202268 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 123.45679 / 111.111111 / 101.010101 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### aim-9m

Parameter projection SHA-256: `e6db75a7de0ccfd26b78128eb7c53539b19b441fe63c6c8a5702988f580d2975`. Plant SHA-256: `2519f562e95ec92b6c69bd6a6f6267b28d269c61164fd0bfe777e13d6bd08f84`.

- Native fuel=22.8915868kg/m0=85.499989kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 748.699535m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 673.829581 / 748.699535 / 823.569488 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 142.949986 / 128.654987 / 116.959079 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### aim-9x

Parameter projection SHA-256: `28098ac6af0ae3c79938e9528df15a2e1da047904ac57ed4fae621191c140530`. Plant SHA-256: `aca7c078fce2c1bef2c78273b0497a00dd90b1edd5514be72fe8108e95625688`.

- Native fuel=22.8915868kg/m0=84.0000044kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 764.437884m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 687.994095 / 764.437884 / 840.881672 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 145.502638 / 130.952374 / 119.047613 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### asraam

Parameter projection SHA-256: `1f3f3a9fee786fef6af62ac9e5647bbb892ab939a4abac1e422659f27ade659b`. Plant SHA-256: `a7c76134c340f7e36c0000fd1f2ecde32f6ae4c2a40b1a111caec03ba163c699`.

- Native fuel=30.7995932kg/m0=87.9999951kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 1034.99462m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 931.495159 / 1034.99462 / 1138.49408 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 315.656583 / 284.090925 / 258.264477 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### iris-t

Parameter projection SHA-256: `ceea6e64bdce20cb3449b0e0ec109f61cd023203cc98aefe1c4e13d9e1176524`. Plant SHA-256: `a0f0853283b555c6db8fd9c94bc50b20dec29ca3727611a2200413381022aa04`.

- Native fuel=24.9726413kg/m0=87.4000149kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 808.464548m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 727.618094 / 808.464548 / 889.311003 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 152.555275 / 137.299748 / 124.817953 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### magic-2

Parameter projection SHA-256: `d5488221a4fb5770bd32abe4d03ce88f7ec1c84f741bc85c607784d7074e040e`. Plant SHA-256: `dd3d2519d2b59d3d3a5b3e6cff82ac9aeaf665ba84999682cb5ab154af5f9bab`.

- Native fuel=20.8105367kg/m0=90.00002kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 631.797831m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 568.618048 / 631.797831 / 694.977614 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 308.641907 / 277.777716 / 252.525196 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### meteor

Parameter projection SHA-256: `a3fcb99980391f156b5f9dd3c56315e3b0badd1c2fdbbfe6cc6366e0b80c6edf`. Plant SHA-256: `02c922fa807029d53ce55ec354be2a763fbddf3f4fa87a5fcc5d5beadedb6718`.

- Native fuel=52.0263374kg/m0=190.000019kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_ducted_rocket.v2; separate finite valve/inlet resident mass/ram drag; no global solid-Isp equivalence
- Ideal comparison 278.716713m/s scope=Booster only; remaining airbreathing fuel still carried

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 250.845041 / 278.716713 / 306.588384 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 146.198816 / 131.578934 / 119.617213 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.
- atmospheric_density_ducted_cycle: 3090.45908 / 3305.48186 / 3381.43947 N at -10% / nominal / +10%. Steady declared cycle only; finite valve/inlet and fuel supply remain native states.

### mica-em

Parameter projection SHA-256: `aecb8bd5420b166e15c9fe48b5efd01861b776acb5d4c21756d0de9d886450ed`. Plant SHA-256: `9e36c380bc4070d60a315639b68a48054afd2ab642d2f6fc84b1bea65a61640b`.

- Native fuel=24.9726413kg/m0=112.000007kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 606.126455m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 545.513809 / 606.126455 / 666.7391 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 148.809514 / 133.928563 / 121.753239 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### mica-ir

Parameter projection SHA-256: `59a9c3769c9d064d59e4ad8608e5369770369efc2ecde284231f511e1d2e9d5f`. Plant SHA-256: `1821db51276f0afb37cb0da2295ceeaeb6d62d9729deef6f5bf7203e80f38c23`.

- Native fuel=24.9726413kg/m0=112.000007kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 606.126455m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 545.513809 / 606.126455 / 666.7391 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 148.809514 / 133.928563 / 121.753239 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### pl-10

Parameter projection SHA-256: `0904316fa78fc7aa7b700fb78d6ca1dea165b7807846c82070d0d51884617ae5`. Plant SHA-256: `c410ed7e34ed47054a887ac352349a28962c0b92f0b332e3d7d77699af084a90`.

- Native fuel=24.9726413kg/m0=89.0000166kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 791.247861m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 712.123075 / 791.247861 / 870.372648 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 149.812706 / 134.831435 / 122.574032 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### pl-12

Parameter projection SHA-256: `dccaac6b3db83c948865b6b6ee167bd1d707615cbc430b83ca829ed521f8eca0`. Plant SHA-256: `e4c01fcbe3bbf27a19795c5f6bdc93158f6dbc2b8322da6510c9bcb94848e569`.

- Native fuel=46.1993899kg/m0=180.000022kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 712.634495m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 641.371045 / 712.634495 / 783.897944 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 154.320969 / 138.888872 / 126.262611 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### pl-15

Parameter projection SHA-256: `02e86873f6b110cc680ccb469d5aa38026c8b7dad246f815211b3c18f2d3a1ca`. Plant SHA-256: `2195e1957c015ca4015c4c11a0c12fd2d91ff2623d15a9b3970d92b3ca3c594b`.

- Native fuel=61.604176kg/m0=210.000004kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_commanded_dual_pulse.v1; commanded hold and pulse clock remain explicit; no automatic second pulse without a native command
- Ideal comparison 834.251337m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 750.826203 / 834.251337 / 917.67647 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 132.27513 / 119.047617 / 108.225106 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### pl-8

Parameter projection SHA-256: `26e5b47239f83fd005b085006dbf1e1c4482d345919d888367cb30704be7b29c`. Plant SHA-256: `bdbca3627ca78563b6f9672487ecb0e54f14e21b1fbdf4c1408893d4295dbd38`.

- Native fuel=24.9726413kg/m0=115.000022kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 588.208007m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 529.387206 / 588.208007 / 647.028808 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 193.236678 / 173.91301 / 158.102737 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### r-27er

Parameter projection SHA-256: `8642507b9f21714b5992036b9bb662e728d32a8e31b4753232b10fca3686e5b6`. Plant SHA-256: `470f960a5e26febb47bc625e498f7883693bf270c7ebb78db6458d4ef1f45520`.

- Native fuel=67.1813028kg/m0=350.000001kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 532.966052m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 479.669447 / 532.966052 / 586.262657 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 111.111111 / 99.9999997 / 90.9090906 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### r-27et

Parameter projection SHA-256: `0edf10af95ba266d4c111280a3627a868b12a1bb85fa8d64387f48b4a3f67a1f`. Plant SHA-256: `3449c4f07b6a23fca3c7d0da91b02b59c5eaa29a27e383c767733b21f25f418d`.

- Native fuel=67.1813028kg/m0=348.000022kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 536.382258m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 482.744032 / 536.382258 / 590.020484 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 111.749674 / 100.574706 / 91.4315513 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### r-27r

Parameter projection SHA-256: `f068eaae80654e6f39d6be031d17a405ce92d42595855bdce0b26bee9fbd8d9f`. Plant SHA-256: `b427f8d0361a2ac66cc060c7e4ab20e55160d3cff00855690ab948555ac448cd`.

- Native fuel=43.187981kg/m0=252.999989kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 468.074017m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 421.266615 / 468.074017 / 514.881419 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 79.0513867 / 71.146248 / 64.6784073 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### r-27t

Parameter projection SHA-256: `0d5845389741ab78863ac4f56ca8a03aedf5bff36dfbe8c8e989244d1f32512d`. Plant SHA-256: `d91746d1a779f3862acec146456475e274563d57b21d9b6d51e2e34f901c0492`.

- Native fuel=43.187981kg/m0=252.999989kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 468.074017m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 421.266615 / 468.074017 / 514.881419 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 79.0513867 / 71.146248 / 64.6784073 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### r-37m

Parameter projection SHA-256: `e1c81a2d14b20ae8081e4b3f1b392bf687620bbe7003aedc2e56e3aa2ff64d68`. Plant SHA-256: `a480640a3ada7b9437ff4b78418ea98d9813f51856f97f707908b8870dee154c`.

- Native fuel=137.357853kg/m0=509.999997kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_commanded_dual_pulse.v1; commanded hold and pulse clock remain explicit; no automatic second pulse without a native command
- Ideal comparison 753.926223m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 678.533601 / 753.926223 / 829.318845 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 87.1459701 / 78.431373 / 71.3012482 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### r-60m

Parameter projection SHA-256: `c34c16ecec6e123213306adde466cd42aeb53ca116bed2981c1053b09a89a37c`. Plant SHA-256: `5773bc316905806dd04d1b4b85aa6199e1704e63f7f86d8904befe9e21f7077c`.

- Native fuel=9.98905653kg/m0=44.9999918kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 603.064407m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 542.757966 / 603.064407 / 663.370847 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 197.5309 / 177.77781 / 161.616191 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### r-73

Parameter projection SHA-256: `adc083f4850080f7553f81dd63efac2fd05b38b2271be3fa3cf064fbcd1f84c7`. Plant SHA-256: `1f683d7303285ceed50d276ca09e904b59b51c13d9047f6f34c3bf862c278933`.

- Native fuel=22.3937691kg/m0=105.000016kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 599.854887m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 539.869398 / 599.854887 / 659.840376 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 148.148126 / 133.333313 / 121.212103 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### r-77m

Parameter projection SHA-256: `d27774a6a1567e3580398a6c0402d174dc6eea801726078e9e6379eb4fb45cc1`. Plant SHA-256: `9d0e52c06857d82e55fe03c5cbb147ae31fff731f66ec41fc4e997e2ca03dac2`.

- Native fuel=61.604176kg/m0=189.999983kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 941.605416m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 847.444874 / 941.605416 / 1035.76596 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 146.198844 / 131.578959 / 119.617236 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

### super-530d

Parameter projection SHA-256: `87d2e886b99da089a0b86aa08334f06a40a31655ef1520a3fba5049dbfc5d333`. Plant SHA-256: `48195a0aa0d0193d56a976539a58882005de1a231fdb14b0e891160af2af9120`.

- Native fuel=44.7875337kg/m0=270.000001kg; engine feed IDs/nozzle/serialized thrust axis/full XML retained; not TOML thrust alone
- fsim.propulsion.jsbsim_generated_solid_rocket; native grain remaining-mass axis and buildup; finite feed before force
- Ideal comparison 453.570461m/s scope=Ideal constant-Isp vacuum, all native propellant; not a conservation proof or encounter range

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- Isp_fixedmass: 408.213415 / 453.570461 / 498.927507 m/s at -10% / nominal / +10%. Ideal only; authored fuel/curve identity not retuned.
- launch_mass_fixedpeakforce: 164.609053 / 148.148148 / 134.680134 m/s2 at -10% / nominal / +10%. Local sensitivity, not an alternative physical mass.

## Reproduction and proof separation

Run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-write-per-model-six-theory-20261103.py` with the recorded DENV environment. It reads explicit 24 catalogue/card/TOML/native XML paths and engine references, rechecks source hashes, and writes only the 13 assigned document files. Then run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-check-per-model-six-theory-20261103.py`. The adjacent `per-model-six-components.json` contains all 144 rows and full actual plant derivations. The generation script retains historical comparisons without wrapping them again.

The JSON deliberately sets current `passed`, numerical comparison and native acceptance false. A source-derived sensitivity or synthetic method benchmark cannot replace current model-native captures. No scenario, gain, threshold, provider, geometry or runtime code was changed for this projection.
