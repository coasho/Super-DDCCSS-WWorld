# Per-model autopilot theory and physical applicability

This source-bound package contains 24 distinct model projections. It preserves actual native geometry, mass/CG/inertia, receiver capability, actuator limits and engine inputs. It does not admit current numerical/native encounter acceptance, hardware calibration or superiority over DCS. Historical executed comparisons remain in the JSON under their original source/card epochs.

## Governing method

`F_eff=F_angle-F_control*M_angle/M_control; solve force/path and moment trim together at actual CG`

`x=[angle,body_rate,actual_virtual_surface]; A=[[F_angle/(mV),sign,F_control/(mV)],[M_angle/J,M_rate/J,M_control/J],[0,0,-1/tau]]`

`cascade K=[(J*sign/(Tr*Ti)+M_angle)/M_control,(J/Tr+M_rate)/M_control,0]; actuator-aware K instead assigns eig(A-BK)=[-1/tau,-1/Tr,-1/Ti]`

`operating-point controller minimizes ||J^-1*(M_pred(u)-M_target)||^2 under finite channel/fin lag, rate, stops and current nonlinear fin curves; local search is not global optimality`

## State, I/O and execution

Input: SI body demand/gravity, own incidence/rates/airspeed, actual native mass and diagonal inertia, body derivatives at current CG, measured aerodynamic force/moment and actual controls. Output: normalized aileron/elevator/rudder demands only. Cascade and pole-assigned laws read current state; R77M finite predictor advances channel input/lag/actual and each physical fin over actual native substeps. Actual FCS alone applies force. No hidden integral replaces missing feedback.

## Assumptions and validity

- Offline small-angle, frozen500m/s6000m diagonal-inertia analysis; not a time-varying native pole measurement
- Continuous local stability is necessary only; delay, mixed-fin rate/position saturation, nonlinear incidence, TVC and moving CG can invalidate it
- R77M local cascade Jacobian shown only as initialization; its actual finite nonlinear predictor/search needs separate current native equality test

The declared analysis point is 500m/s at 6000m, zero incidence/sideslip, standard tropospheric air and launch/half/depleted propellant states. It is an offline projection of actual serialized equations. Positive local poles would be a defect indicator; negative local poles do not establish finite delayed nonlinear encounter stability. The source parameter basis and uncertainty are preserved; the +/-10% sweeps are sensitivity, not fitted confidence intervals.

## Primary references

- [Palumbo, Blauwkamp and Lloyd, Basic Principles of Homing Guidance, JHU APL Technical Digest 29(1), 2010](https://secwww.jhuapl.edu/techdigest/content/techdigest/pdf/V29-N01/29-01-Palumbo_Principles_Rev2018.pdf): PN LOS reconstruction, sensor availability and finite lateral-acceleration realization; not OEM missile gains
- [SciPy primary place_poles implementation/documentation, Yang-Tits method](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.place_poles.html): eig(A-BK) comparison; controllability and finite frozen linearization prerequisites
- [Harkegard, Efficient active set algorithms for solving constrained least squares problems in aircraft control allocation, 2002](https://rt.isy.liu.se/research/reports/2002/2426.pdf): Section2 equations1-4: effectiveness matrix and finite actuator bounds; comparison framework, not claim that authored mixer solves QP

## Actual model deltas

| Model | Native mass kg | Jyy kg m2 | Mechanism | Pitch local max real pole 1/s |
|---|---:|---:|---|---:|
| aim-120c | 161.5 | 180.688 | fsim.control.load_feedback_cascade | -4.1654 |
| aim-120d | 161.5 | 180.689 | fsim.control.load_feedback_cascade | -4.16195 |
| aim-7m | 225 | 248.99 | fsim.control.load_feedback_cascade | -3.33602 |
| aim-9m | 85.5 | 58.8201 | fsim.control.load_feedback_cascade | -1.6915 |
| aim-9x | 84 | 63.9021 | fsim.control.load_feedback_cascade | -2.13191 |
| asraam | 88 | 61.8117 | fsim.control.load_feedback_cascade | -2.49357 |
| iris-t | 87.4 | 62.9245 | fsim.control.load_feedback_cascade | -1.79029 |
| magic-2 | 90 | 56.8461 | fsim.control.load_feedback_cascade | -1.8692 |
| meteor | 190 | 217.246 | fsim.control.load_feedback_cascade | -4.21734 |
| mica-em | 112 | 89.8958 | fsim.control.load_feedback_cascade | -2.23237 |
| mica-ir | 112 | 89.8958 | fsim.control.load_feedback_cascade | -2.3158 |
| pl-10 | 89 | 66.9008 | fsim.control.actuator_aware_cascade.v1 | -2.5 |
| pl-12 | 180 | 232.194 | fsim.control.load_feedback_cascade | -3.27688 |
| pl-15 | 210 | 280.632 | fsim.control.load_feedback_cascade | -4.77315 |
| pl-8 | 115 | 83.5867 | fsim.control.load_feedback_cascade | -1.66724 |
| r-27er | 350 | 667.91 | fsim.control.load_feedback_cascade | -3.79635 |
| r-27et | 348 | 585.959 | fsim.control.load_feedback_cascade | -3.19681 |
| r-27r | 253 | 351.753 | fsim.control.load_feedback_cascade | -5.51097 |
| r-27t | 253 | 304.325 | fsim.control.load_feedback_cascade | -4.31654 |
| r-37m | 510 | 702.132 | fsim.control.load_feedback_cascade | -6.00102 |
| r-60m | 45 | 17.0837 | fsim.control.load_feedback_cascade | -2.91192 |
| r-73 | 105 | 75.2869 | fsim.control.load_feedback_cascade | -5.19525 |
| r-77m | 190 | 218.423 | fsim.control.operating_point_bounded.v1 | -3.04367 |
| super-530d | 270 | 325.63 | fsim.control.load_feedback_cascade | -3.97841 |

### aim-120c

Parameter projection SHA-256: `a5c093cbc2a421275ad7bf0a8159c55d2be1357c10baed7c68b3fa8bbd1ce2f8`. Plant SHA-256: `550d9dc2489bc5356abea09b35f859ca9a9ab76a61e48a32e6f49af1763591d8`.

- Actual Jyy=180.688369kgm2, launch CG=1.82999981m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-4.16540374/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -6.14586402 / -4.16540374 / -3.58275717 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00687994541 / 0.00619195087 / 0.00562904625 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### aim-120d

Parameter projection SHA-256: `746c40997ebd875c5bfefc23611760738470640faad229cf5e94671adb78569f`. Plant SHA-256: `3ce42309096c29790d6d45eb6098b3e35b31b36b8eca4790cf187f95d636c7a5`.

- Actual Jyy=180.688655kgm2, launch CG=1.83000067m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-4.16195407/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -6.14536903 / -4.16195407 / -3.58002768 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00687994503 / 0.00619195052 / 0.00562904593 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### aim-7m

Parameter projection SHA-256: `5abb72636512691e89c958ffd9342999f3685edf321248dc7fef183c0fa06af9`. Plant SHA-256: `75c07167eba4f3be6fa48df961b72c09c07119ed94b9da73244da936e1b6f160`.

- Actual Jyy=248.989547kgm2, launch CG=1.82000014m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-3.33601589/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -4.66545905 / -3.33601589 / -2.58659426 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.0049382716 / 0.00444444444 / 0.00404040404 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### aim-9m

Parameter projection SHA-256: `a3f308ee6e3339784446f201bbfe1655aee1c6247483da6f2439b2f15ba7125b`. Plant SHA-256: `2519f562e95ec92b6c69bd6a6f6267b28d269c61164fd0bfe777e13d6bd08f84`.

- Actual Jyy=58.820131kgm2, launch CG=1.43500061m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-1.6915021/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -3.29662481 / -1.6915021 / -0.480505563 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.0129954533 / 0.0116959079 / 0.0106326436 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### aim-9x

Parameter projection SHA-256: `140d98489422cbc0da305809294bd7f0d1fa7e2bba46b3d1509ceed8cea67870`. Plant SHA-256: `aca7c078fce2c1bef2c78273b0497a00dd90b1edd5514be72fe8108e95625688`.

- Actual Jyy=63.9020936kgm2, launch CG=1.50900216m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-2.13190586/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -3.46934415 / -2.13190586 / -1.22955323 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.0132275125 / 0.0119047613 / 0.0108225103 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### asraam

Parameter projection SHA-256: `c3485570c9e4d48439e3acd77c10bf34374fcb71ee8c039f60f681f02d18c992`. Plant SHA-256: `a7c76134c340f7e36c0000fd1f2ecde32f6ae4c2a40b1a111caec03ba163c699`.

- Actual Jyy=61.8117323kgm2, launch CG=1.4499963m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-2.49356507/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -3.75534518 / -2.49356507 / -1.68536495 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.0126262633 / 0.011363637 / 0.0103305791 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### iris-t

Parameter projection SHA-256: `51f7f85131840407f8ada65abeef1f6d4bb5e2ad81d4ed9921d55f7f745f3928`. Plant SHA-256: `a0f0853283b555c6db8fd9c94bc50b20dec29ca3727611a2200413381022aa04`.

- Actual Jyy=62.9245488kgm2, launch CG=1.46800118m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-1.79028651/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -3.25852769 / -1.79028651 / -0.743024471 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.0127129396 / 0.0114416456 / 0.010401496 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### magic-2

Parameter projection SHA-256: `581b88980e782effce32c661bd52fc3b5a5bdf239cf899742b687e1b3eb1b937`. Plant SHA-256: `dd3d2519d2b59d3d3a5b3e6cff82ac9aeaf665ba84999682cb5ab154af5f9bab`.

- Actual Jyy=56.846082kgm2, launch CG=1.37499705m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-1.86919746/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -3.41478341 / -1.86919746 / -0.725749702 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.0123456763 / 0.0111111086 / 0.0101010079 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### meteor

Parameter projection SHA-256: `717b612acbccf6cd711dac5114e92918f668c6ac20987a18960de19e1492c07d`. Plant SHA-256: `02c922fa807029d53ce55ec354be2a763fbddf3f4fa87a5fcc5d5beadedb6718`.

- Actual Jyy=217.246472kgm2, launch CG=1.84999997m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-4.21734362/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -5.89628118 / -4.21734362 / -3.65088221 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00584795263 / 0.00526315736 / 0.00478468851 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### mica-em

Parameter projection SHA-256: `fdeaead3bbae25a4638984f3bc26c9ff465941ae5c8cdcf749c13787030e7bfc`. Plant SHA-256: `9e36c380bc4070d60a315639b68a48054afd2ab642d2f6fc84b1bea65a61640b`.

- Actual Jyy=89.8958041kgm2, launch CG=1.55000231m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-2.23236724/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -3.54133505 / -2.23236724 / -1.36239053 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00992063426 / 0.00892857084 / 0.00811688258 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### mica-ir

Parameter projection SHA-256: `8f5dc5e63b5498b2c8764e99a9b5d9e8a3e361259674b266e4121f7717d2ae42`. Plant SHA-256: `1821db51276f0afb37cb0da2295ceeaeb6d62d9729deef6f5bf7203e80f38c23`.

- Actual Jyy=89.8958041kgm2, launch CG=1.55000231m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-2.31580073/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -3.60380601 / -2.31580073 / -1.470706 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00992063426 / 0.00892857084 / 0.00811688258 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### pl-10

Parameter projection SHA-256: `26d6bbe3cbf65b82d7a5b357419a498e8e4fca587f53b710c882b71756de58b9`. Plant SHA-256: `c410ed7e34ed47054a887ac352349a28962c0b92f0b332e3d7d77699af084a90`.

- Actual Jyy=66.9007879kgm2, launch CG=1.50000292m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-2.5/s at declared point; not sampled closed-loop damping/native encounter pass
- Finite three-state pole assignment, not legacy cascade gain reuse

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -4.67448764 / -2.5 / -1.83017715 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.0124843922 / 0.011235953 / 0.0102145027 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### pl-12

Parameter projection SHA-256: `074cc7008127fa91d717cdf8b636a6dc85e65fba66bd20e7c694274d4b2d260b`. Plant SHA-256: `e4c01fcbe3bbf27a19795c5f6bdc93158f6dbc2b8322da6510c9bcb94848e569`.

- Actual Jyy=232.194177kgm2, launch CG=1.96499758m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-3.2768818/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -4.50183733 / -3.2768818 / -2.63146318 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00617283876 / 0.00555555488 / 0.00505050444 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### pl-15

Parameter projection SHA-256: `fff468c7c1ec93109293e462d97daeb3f9fafee280e9b2a799c317bee30d33d7`. Plant SHA-256: `2195e1957c015ca4015c4c11a0c12fd2d91ff2623d15a9b3970d92b3ca3c594b`.

- Actual Jyy=280.631967kgm2, launch CG=2.00000311m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-4.77315055/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -5.40477059 / -4.77315055 / -4.12163746 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00529100518 / 0.00476190467 / 0.00432900424 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### pl-8

Parameter projection SHA-256: `533606611479655ded4dfd7d32fc97a9a80100170428e013697c2cfdad73fad7`. Plant SHA-256: `bdbca3627ca78563b6f9672487ecb0e54f14e21b1fbdf4c1408893d4295dbd38`.

- Actual Jyy=83.5866962kgm2, launch CG=1.47499953m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-1.66724124/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -3.28736525 / -1.66724124 / -0.438308147 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.0096618339 / 0.00869565051 / 0.00790513683 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### r-27er

Parameter projection SHA-256: `62538d3fae74c41f0c2dd904c53fdb3ce2484a908f14168f5a22d5984512a555`. Plant SHA-256: `470f960a5e26febb47bc625e498f7883693bf270c7ebb78db6458d4ef1f45520`.

- Actual Jyy=667.910032kgm2, launch CG=2.38999796m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-3.79634629/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -3.39164983 / -3.79634629 / -4.21824933 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00317460316 / 0.00285714285 / 0.00259740259 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### r-27et

Parameter projection SHA-256: `c23092006787b76bba7c0b7eeca4eae4a076ae648fa5e743872e46cdb5f3cc21`. Plant SHA-256: `3449c4f07b6a23fca3c7d0da91b02b59c5eaa29a27e383c767733b21f25f418d`.

- Actual Jyy=585.959034kgm2, launch CG=2.24499951m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-3.19680868/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -2.90131836 / -3.19680868 / -3.49270283 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00319284782 / 0.00287356304 / 0.00261233004 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### r-27r

Parameter projection SHA-256: `47e1fc45269f0acab7941568ec1a33ea695976d352aabe9aed836035f1782ebf`. Plant SHA-256: `b427f8d0361a2ac66cc060c7e4ab20e55160d3cff00855690ab948555ac448cd`.

- Actual Jyy=351.752526kgm2, launch CG=2.04000077m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-5.51097331/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -4.66880671 / -5.51097331 / -6.79441535 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00439174371 / 0.00395256934 / 0.00359324485 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### r-27t

Parameter projection SHA-256: `39fcf8cb341558cd951d21b527ebfa5815abcd02ef0c8d6b8d5e1ef2068fb09e`. Plant SHA-256: `d91746d1a779f3862acec146456475e274563d57b21d9b6d51e2e34f901c0492`.

- Actual Jyy=304.325489kgm2, launch CG=1.89749848m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-4.31653559/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -3.83513934 / -4.31653559 / -4.83948409 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00439174371 / 0.00395256934 / 0.00359324485 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### r-37m

Parameter projection SHA-256: `188a0788950d5740013623a43d6b172585c9470c98ee1d4a7d71286e4ef4f0e7`. Plant SHA-256: `a480640a3ada7b9437ff4b78418ea98d9813f51856f97f707908b8870dee154c`.

- Actual Jyy=702.131806kgm2, launch CG=2.03000087m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-6.00101655/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -4.80052817 / -6.00101655 / -5.84816072 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00217864925 / 0.00196078433 / 0.00178253121 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### r-60m

Parameter projection SHA-256: `5227d388bc9184e33dc0ca6fc0f7bd3586e14e0db201c430ea5731dfbd708b81`. Plant SHA-256: `5773bc316905806dd04d1b4b85aa6199e1704e63f7f86d8904befe9e21f7077c`.

- Actual Jyy=17.0836945kgm2, launch CG=1.06599937m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-2.91192351/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -4.15861804 / -2.91192351 / -2.17047892 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.0246913625 / 0.0222222263 / 0.0202020239 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### r-73

Parameter projection SHA-256: `e0ef7fca18e3717364387f8ac41ec48732c0d2517100a7f70865a6164c9004bd`. Plant SHA-256: `1f683d7303285ceed50d276ca09e904b59b51c13d9047f6f34c3bf862c278933`.

- Actual Jyy=75.2868795kgm2, launch CG=1.46499914m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-5.19524699/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -4.46734214 / -5.19524699 / -6.15755417 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.010582009 / 0.0095238081 / 0.00865800736 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### r-77m

Parameter projection SHA-256: `145cdc2165c7e89dc44c7caeadbd5595c0e2df0d4f9debcc2d547f72866e5282`. Plant SHA-256: `9d0e52c06857d82e55fe03c5cbb147ae31fff731f66ec41fc4e997e2ca03dac2`.

- Actual Jyy=218.423058kgm2, launch CG=1.85500208m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-3.04366949/s at declared point; not sampled closed-loop damping/native encounter pass
- Bounded nonlinear operating-point search; linear initialization below does not prove solver prediction

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -4.1817711 / -3.04366949 / -2.42563501 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00584795375 / 0.00526315837 / 0.00478468943 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

### super-530d

Parameter projection SHA-256: `ceeb2f8d174fd1f6a9fa720226f8d3faf676012a04713137e17424dbf5a7a505`. Plant SHA-256: `48195a0aa0d0193d56a976539a58882005de1a231fdb14b0e891160af2af9120`.

- Actual Jyy=325.63016kgm2, launch CG=1.89999796m structural; half/depleted CG/J recomputed from native grain geometry
- Frozen local pitch maximum real pole=-3.97840769/s at declared point; not sampled closed-loop damping/native encounter pass
- Load-feedback cascade; finite actuator omitted from two-state gain design but included in the local three-state analysis

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- control_force_and_moment_effectiveness_fixed_K: -6.11082553 / -3.97840769 / -3.40909665 1/s at -10% / nominal / +10%. Same actual mass/J/frozen point; pure local continuous linearization.
- mass_at_fixed_force: 0.00411522632 / 0.00370370369 / 0.00336700336 1/kg at -10% / nominal / +10%. Acceleration per N; real burn also changes J/CG, captured separately.

## Reproduction and proof separation

Run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-write-per-model-six-theory-20261103.py` with the recorded DENV environment. It reads explicit 24 catalogue/card/TOML/native XML paths and engine references, rechecks source hashes, and writes only the 13 assigned document files. Then run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-check-per-model-six-theory-20261103.py`. The adjacent `per-model-six-components.json` contains all 144 rows and full actual plant derivations. The generation script retains historical comparisons without wrapping them again.

The JSON deliberately sets current `passed`, numerical comparison and native acceptance false. A source-derived sensitivity or synthetic method benchmark cannot replace current model-native captures. No scenario, gain, threshold, provider, geometry or runtime code was changed for this projection.
