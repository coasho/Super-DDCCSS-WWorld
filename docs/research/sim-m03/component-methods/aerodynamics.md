# Per-model aerodynamics theory and physical applicability

This source-bound package contains 24 distinct model projections. It preserves actual native geometry, mass/CG/inertia, receiver capability, actuator limits and engine inputs. It does not admit current numerical/native encounter acceptance, hardware calibration or superiority over DCS. Historical executed comparisons remain in the JSON under their original source/card epochs.

## Governing method

`VLM: sum_j A_ij*Gamma_j=-V_inf dot n_i; F=rho*Gamma*(V cross segment); section/compressibility/drag closures additional`

`F_angle=q*S*C_Fangle; M_angle=q*S*length*C_Mangle + (r_ref-r_CG) cross F_angle`

`pitch static margin=-Cm_alpha/Cz_alpha in this body-normal convention; check exact sign/axis before comparing other conventions`

`Native table evaluation/clamping, actual geometry and fin nonlinear increments remain distinct from local linear derivative projection`

## State, I/O and execution

Input: actual air-relative Mach, alpha/beta, body rates, each physical fin angle and qbar. Serialized generated functions output native force/moment; World converts their current derivatives and CG transport to SI controller inputs. Tables interpolate/clamp at represented bounds. VLM circulation solve and section closures are generation methods; they do not execute an alternative client flight model.

## Assumptions and validity

- VLM/section closure is not viscous/transonic/shock-separated/high-alpha CFD; serialized table boundary clamps are extrapolation policy
- Geometry and inertia are source-authored engineering approximations, not measured OEM plant identification
- Local alpha=beta0 stability/control derivatives do not establish maneuvering/high-angle/native encounter accuracy

The declared analysis point is 500m/s at 6000m, zero incidence/sideslip, standard tropospheric air and launch/half/depleted propellant states. It is an offline projection of actual serialized equations. Positive local poles would be a defect indicator; negative local poles do not establish finite delayed nonlinear encounter stability. The source parameter basis and uncertainty are preserved; the +/-10% sweeps are sensitivity, not fitted confidence intervals.

## Primary references

- [W. H. Mason, Applied Computational Aerodynamics, chapter6](https://archive.aoe.vt.edu/mason/Mason_f/CAtxtChap6.pdf): Sections6.2/6.4, linearized nonpenetration and Biot-Savart lattice; inviscid thin mean surface, not CFD or high-alpha validation
- [U.S. Standard Atmosphere1976, NASA-TM-X-74335 / NOAA-S/T-76-1562](https://ntrs.nasa.gov/citations/19770009539): Declared6000m tropospheric analysis point only; actual World atmosphere remains native authority

## Actual model deltas

| Model | Native mass kg | Jyy kg m2 | Mechanism | Pitch local max real pole 1/s |
|---|---:|---:|---|---:|
| aim-120c | 161.5 | 180.688 | fsim.aerodynamics.jsbsim_generated_store | -4.1654 |
| aim-120d | 161.5 | 180.689 | fsim.aerodynamics.jsbsim_generated_store | -4.16195 |
| aim-7m | 225 | 248.99 | fsim.aerodynamics.jsbsim_generated_store | -3.33602 |
| aim-9m | 85.5 | 58.8201 | fsim.aerodynamics.jsbsim_generated_store | -1.6915 |
| aim-9x | 84 | 63.9021 | fsim.aerodynamics.jsbsim_generated_store | -2.13191 |
| asraam | 88 | 61.8117 | fsim.aerodynamics.jsbsim_generated_store | -2.49357 |
| iris-t | 87.4 | 62.9245 | fsim.aerodynamics.jsbsim_generated_store | -1.79029 |
| magic-2 | 90 | 56.8461 | fsim.aerodynamics.jsbsim_generated_store | -1.8692 |
| meteor | 190 | 217.246 | fsim.aerodynamics.jsbsim_generated_store | -4.21734 |
| mica-em | 112 | 89.8958 | fsim.aerodynamics.jsbsim_generated_store | -2.23237 |
| mica-ir | 112 | 89.8958 | fsim.aerodynamics.jsbsim_generated_store | -2.3158 |
| pl-10 | 89 | 66.9008 | fsim.aerodynamics.jsbsim_generated_store | -2.5 |
| pl-12 | 180 | 232.194 | fsim.aerodynamics.jsbsim_generated_store | -3.27688 |
| pl-15 | 210 | 280.632 | fsim.aerodynamics.jsbsim_generated_store | -4.77315 |
| pl-8 | 115 | 83.5867 | fsim.aerodynamics.jsbsim_generated_store | -1.66724 |
| r-27er | 350 | 667.91 | fsim.aerodynamics.jsbsim_generated_store | -3.79635 |
| r-27et | 348 | 585.959 | fsim.aerodynamics.jsbsim_generated_store | -3.19681 |
| r-27r | 253 | 351.753 | fsim.aerodynamics.jsbsim_generated_store | -5.51097 |
| r-27t | 253 | 304.325 | fsim.aerodynamics.jsbsim_generated_store | -4.31654 |
| r-37m | 510 | 702.132 | fsim.aerodynamics.jsbsim_generated_store | -6.00102 |
| r-60m | 45 | 17.0837 | fsim.aerodynamics.jsbsim_generated_store | -2.91192 |
| r-73 | 105 | 75.2869 | fsim.aerodynamics.jsbsim_generated_store | -5.19525 |
| r-77m | 190 | 218.423 | fsim.aerodynamics.jsbsim_generated_store | -3.04367 |
| super-530d | 270 | 325.63 | fsim.aerodynamics.jsbsim_generated_store | -3.97841 |

### aim-120c

Parameter projection SHA-256: `b1ce2ed4f6657af11f893addae269a9b7cff37e704aa75ed78ae35bd8f90ae15`. Plant SHA-256: `550d9dc2489bc5356abea09b35f859ca9a9ab76a61e48a32e6f49af1763591d8`.

- Actual S=0.0248286665m2, b=0.1778m,c=0.1778m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-5.40825632,Cz_alpha=-8.20489381, margin=-0.659150069
- Actual CG shift across burn -0.332657895m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -93.6161465 / -104.017941 / -114.419735 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0366m: -2583.62114 / -1968.78153 / -1353.94193 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### aim-120d

Parameter projection SHA-256: `b9f8295da2403cb30f291f8f74a000979363b5bb93f6423111e04659a0368b8a`. Plant SHA-256: `3ce42309096c29790d6d45eb6098b3e35b31b36b8eca4790cf187f95d636c7a5`.

- Actual S=0.0248286665m2, b=0.1778m,c=0.1778m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-5.40825632,Cz_alpha=-8.13392067, margin=-0.664901533
- Actual CG shift across burn -0.349248101m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -92.8063524 / -103.118169 / -113.429986 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0366m: -2578.28824 / -1968.76706 / -1359.24588 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### aim-7m

Parameter projection SHA-256: `490a65cbd67842f8e3acb96749e7e190c664821dc5bc3bc43a8b8f40e05ef791`. Plant SHA-256: `75c07167eba4f3be6fa48df961b72c09c07119ed94b9da73244da936e1b6f160`.

- Actual S=0.0323654729m2, b=0.203m,c=0.203m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-22.4904463,Cz_alpha=-22.3392502, margin=-1.00676818
- Actual CG shift across burn -0.263321377m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -238.487121 / -264.98569 / -291.484259 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0364m: -14355.3625 / -12185.1297 / -10014.8969 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### aim-9m

Parameter projection SHA-256: `cf08cd90f93bf28f0c2bed071a5b8bad092cfdd3f0587ed1cf2d317a16a7d461`. Plant SHA-256: `2519f562e95ec92b6c69bd6a6f6267b28d269c61164fd0bfe777e13d6bd08f84`.

- Actual S=0.012667687m2, b=0.127m,c=0.127m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-89.3045972,Cz_alpha=-21.9962734, margin=-4.0599876
- Actual CG shift across burn -0.212978722m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -241.867364 / -268.741516 / -295.615667 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0287m: -12507.0064 / -11847.5551 / -11188.1038 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### aim-9x

Parameter projection SHA-256: `b293875ab3cd36752f9fc7bd856b073a9fa80e1fb1a2178db867d1ba6f19f3bd`. Plant SHA-256: `aca7c078fce2c1bef2c78273b0497a00dd90b1edd5514be72fe8108e95625688`.

- Actual S=0.012667687m2, b=0.127m,c=0.127m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-45.6620735,Cz_alpha=-9.29749484, margin=-4.91122333
- Actual CG shift across burn -0.245929003m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -104.05929 / -115.621434 / -127.183577 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.03018m: -6350.83931 / -6057.72509 / -5764.61086 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### asraam

Parameter projection SHA-256: `a013425df04dfb766e728d70efc68913c9bf19104bf18b05d9de276228accb7f`. Plant SHA-256: `a7c76134c340f7e36c0000fd1f2ecde32f6ae4c2a40b1a111caec03ba163c699`.

- Actual S=0.0216424318m2, b=0.166m,c=0.166m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-14.8008103,Cz_alpha=-6.98544789, margin=-2.11880619
- Actual CG shift across burn -0.395453819m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -127.501286 / -141.668096 / -155.834905 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.029m: -4746.42581 / -4384.88885 / -4023.35189 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### iris-t

Parameter projection SHA-256: `932ae45055ffbf808d6dbd941b984dafa086181c2c410dc8e93d6e36364bd426`. Plant SHA-256: `a0f0853283b555c6db8fd9c94bc50b20dec29ca3727611a2200413381022aa04`.

- Actual S=0.012667687m2, b=0.127m,c=0.127m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-64.64468,Cz_alpha=-11.4038027, margin=-5.66869507
- Actual CG shift across burn -0.220544631m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -122.668318 / -136.298131 / -149.927944 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.02936m: -8925.80391 / -8576.05412 / -8226.30434 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### magic-2

Parameter projection SHA-256: `d463627f9ed01808d20108097b12773f9509d0c58ec2c593ec771eaa368a4283`. Plant SHA-256: `dd3d2519d2b59d3d3a5b3e6cff82ac9aeaf665ba84999682cb5ab154af5f9bab`.

- Actual S=0.0193592793m2, b=0.157m,c=0.157m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-39.0924398,Cz_alpha=-16.2934578, margin=-2.39927216
- Actual CG shift across burn -0.234334816m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -260.109815 / -289.010905 / -317.911996 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0275m: -10513.3467 / -9798.04451 / -9082.74237 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### meteor

Parameter projection SHA-256: `6d69907f3f6b4a74aba7b5035c18007aafda37395f180ee332b2bc16cded82ba`. Plant SHA-256: `02c922fa807029d53ce55ec354be2a763fbddf3f4fa87a5fcc5d5beadedb6718`.

- Actual S=0.0248845554m2, b=0.178m,c=0.178m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-4.77367886,Cz_alpha=-6.34627177, margin=-0.752202084
- Actual CG shift across burn -0.293023112m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -61.6867382 / -68.5408202 / -75.3949023 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.037m: -2225.48502 / -1743.64301 / -1261.80099 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### mica-em

Parameter projection SHA-256: `0d427b1fe92742c210e386a083712b1a8ecb3532cf341b2a4c48d08ec90b1cd9`. Plant SHA-256: `9e36c380bc4070d60a315639b68a48054afd2ab642d2f6fc84b1bea65a61640b`.

- Actual S=0.020106193m2, b=0.16m,c=0.16m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-29.3139527,Cz_alpha=-8.10509406, margin=-3.61673195
- Actual CG shift across burn -0.262011624m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -107.985826 / -119.984251 / -131.982676 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.031m: -8192.95454 / -7776.36919 / -7359.78385 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### mica-ir

Parameter projection SHA-256: `40d6de3a769f7ddeb8d61db8ef62d640ed2317fc5e4fc46841cc79f4c98da4e2`. Plant SHA-256: `1821db51276f0afb37cb0da2295ceeaeb6d62d9729deef6f5bf7203e80f38c23`.

- Actual S=0.020106193m2, b=0.16m,c=0.16m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-27.214341,Cz_alpha=-8.23484276, margin=-3.30477968
- Actual CG shift across burn -0.262011624m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -109.714494 / -121.904994 / -134.095493 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.031m: -7642.63825 / -7219.38409 / -6796.12992 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### pl-10

Parameter projection SHA-256: `98480262fdc4247cedc6145b89b5bb7b7def81906471fe509d493fd95c0195fc`. Plant SHA-256: `c410ed7e34ed47054a887ac352349a28962c0b92f0b332e3d7d77699af084a90`.

- Actual S=0.020106193m2, b=0.16m,c=0.16m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-8.12013047,Cz_alpha=-6.11270458, margin=-1.32840224
- Actual CG shift across burn -0.33663006m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -102.487303 / -113.874781 / -125.26226 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.03m: -2458.12294 / -2154.07722 / -1850.03149 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### pl-12

Parameter projection SHA-256: `da0d8d7402b39574c7ae4a86573c6830fa712f7f9e0e7a56328b54ef46bd18d6`. Plant SHA-256: `e4c01fcbe3bbf27a19795c5f6bdc93158f6dbc2b8322da6510c9bcb94848e569`.

- Actual S=0.0323654729m2, b=0.203m,c=0.203m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-14.8626909,Cz_alpha=-10.7553367, margin=-1.38188988
- Actual CG shift across burn -0.423505465m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -143.525909 / -159.473233 / -175.420556 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0393m: -9180.66648 / -8052.5527 / -6924.43891 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### pl-15

Parameter projection SHA-256: `7f588467a4619715833588c5ef728323b6347cc3d6d2cb2c51d1588e640e34f1`. Plant SHA-256: `2195e1957c015ca4015c4c11a0c12fd2d91ff2623d15a9b3970d92b3ca3c594b`.

- Actual S=0.0323654729m2, b=0.203m,c=0.203m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=0.0437307935,Cz_alpha=-2.97705015, margin=0.0146893036
- Actual CG shift across burn -0.440057345m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -34.0522414 / -37.8358238 / -41.6194062 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.04m: -294.103202 / 23.7177247 / 341.538651 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### pl-8

Parameter projection SHA-256: `9d1776c2f8cf8c6d405a64444ff7a7d6e633458929fb12270c76e50eedee8f11`. Plant SHA-256: `bdbca3627ca78563b6f9672487ecb0e54f14e21b1fbdf4c1408893d4295dbd38`.

- Actual S=0.020106193m2, b=0.16m,c=0.16m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-66.3631626,Cz_alpha=-18.6604997, margin=-3.55634435
- Actual CG shift across burn -0.232473643m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -242.131945 / -269.035495 / -295.939044 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0295m: -18517.5258 / -17604.8227 / -16692.1196 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### r-27er

Parameter projection SHA-256: `a3ce11544fb6099035f2f77e2dc3c3df1e823be1fbdabc6346119873d1bf4bd4`. Plant SHA-256: `470f960a5e26febb47bc625e498f7883693bf270c7ebb78db6458d4ef1f45520`.

- Actual S=0.0530929158m2, b=0.26m,c=0.26m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=25.3303565,Cz_alpha=-10.3190195, margin=2.45472513
- Actual CG shift across burn -0.400541807m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -116.172641 / -129.080713 / -141.988784 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0478m: 26674.4355 / 28833.9558 / 30993.4761 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### r-27et

Parameter projection SHA-256: `fa30dfcaaf2dea925263d8606b7a63df6039dfb367a6c9b2da941f27fafbb48e`. Plant SHA-256: `3449c4f07b6a23fca3c7d0da91b02b59c5eaa29a27e383c767733b21f25f418d`.

- Actual S=0.0530929158m2, b=0.26m,c=0.26m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=30.4424322,Cz_alpha=-10.4363378, margin=2.91696503
- Actual CG shift across burn -0.368707153m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -118.168666 / -131.298517 / -144.428369 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0449m: 32601.6372 / 34653.2029 / 36704.7686 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### r-27r

Parameter projection SHA-256: `56adf6f8a9882a3ca13af82c7eceb1fa85c8be88733c191417aa2070cd5e474c`. Plant SHA-256: `b427f8d0361a2ac66cc060c7e4ab20e55160d3cff00855690ab948555ac448cd`.

- Actual S=0.0415475628m2, b=0.23m,c=0.23m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=4.80421308,Cz_alpha=-13.299041, margin=0.361245075
- Actual CG shift across burn -0.297860261m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -162.084795 / -180.094217 / -198.103638 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0408m: 1926.76431 / 3785.76878 / 5644.77324 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### r-27t

Parameter projection SHA-256: `c58db1e75f9c09aee4704e847fb4302e5298900de081a27873318f6ac8a3074e`. Plant SHA-256: `d91746d1a779f3862acec146456475e274563d57b21d9b6d51e2e34f901c0492`.

- Actual S=0.0415475628m2, b=0.23m,c=0.23m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=12.0984671,Cz_alpha=-13.5327219, margin=0.89401579
- Actual CG shift across burn -0.268525585m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -164.932829 / -183.258699 / -201.584569 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.03795m: 7774.02528 / 9533.55611 / 11293.0869 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### r-37m

Parameter projection SHA-256: `f385c50a5a9081b53f4e4067bcef2626125cfea6e8e0af5da3270419ddb9fdba`. Plant SHA-256: `a480640a3ada7b9437ff4b78418ea98d9813f51856f97f707908b8870dee154c`.

- Actual S=0.113411495m2, b=0.38m,c=0.38m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=2.25345557,Cz_alpha=-2.4999394, margin=0.901404078
- Actual CG shift across burn -0.471814775m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -41.2584855 / -45.8427617 / -50.4270379 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0406m: 7059.16895 / 8008.38917 / 8957.60939 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### r-60m

Parameter projection SHA-256: `7caec400449e496282011eec85b1a89c853b273f5ea0922ed3a326b593a37f1a`. Plant SHA-256: `5773bc316905806dd04d1b4b85aa6199e1704e63f7f86d8904befe9e21f7077c`.

- Actual S=0.0113097336m2, b=0.12m,c=0.12m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-8.17331172,Cz_alpha=-7.5699759, margin=-1.07970115
- Actual CG shift across burn -0.171437311m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -141.198907 / -156.887674 / -172.576442 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.02132m: -1065.238 / -914.719988 / -764.201981 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### r-73

Parameter projection SHA-256: `9839976502c65583a6350449975270550cadb756da36b61cbb6dee6b53069b52`. Plant SHA-256: `1f683d7303285ceed50d276ca09e904b59b51c13d9047f6f34c3bf862c278933`.

- Actual S=0.021382465m2, b=0.165m,c=0.165m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=3.70039574,Cz_alpha=-7.90910395, margin=0.467865357
- Actual CG shift across burn -0.245605085m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -119.534313 / -132.815903 / -146.097493 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0293m: 667.954853 / 1076.56304 / 1485.17123 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### r-77m

Parameter projection SHA-256: `afe703137aea1c1097fae1b31f5ba52021b3728fc386c7deb041f4d8c12076f3`. Plant SHA-256: `9d0e52c06857d82e55fe03c5cbb147ae31fff731f66ec41fc4e997e2ca03dac2`.

- Actual S=0.0314159265m2, b=0.2m,c=0.2m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-14.2455115,Cz_alpha=-4.6138229, margin=-3.08757224
- Actual CG shift across burn -0.430312178m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -56.6179488 / -62.908832 / -69.1997152 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.0371m: -7824.37014 / -7380.92582 / -6937.48151 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

### super-530d

Parameter projection SHA-256: `462d66f2a9ae7a3695c1cc9e3623caea7866df5112fde81e0aae910ba90f6b48`. Plant SHA-256: `48195a0aa0d0193d56a976539a58882005de1a231fdb14b0e891160af2af9120`.

- Actual S=0.0543252056m2, b=0.263m,c=0.263m; not a common reference area
- NativeMach/alpha tables determine derivative; at this point Cm_alpha=-3.75780236,Cz_alpha=-6.68310209, margin=-0.562284147
- Actual CG shift across burn -0.273933718m changesmoment independently of VLM geometry

Nonzero sensitivity (exact full fields/units/limits are in the matching JSON row):

- dynamic_pressure: -99.7958789 / -110.88431 / -121.972741 m/s2/rad at -10% / nominal / +10%. Same Cz_alpha/actual mass; physical q changes Mach/Re independently.
- structural_CG_x +/-0.038m: -5565.10024 / -4427.42722 / -3289.75419 Nm/rad. Frozen origin/table force; lever=origin-CG-aeroref.

## Reproduction and proof separation

Run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-write-per-model-six-theory-20261103.py` with the recorded DENV environment. It reads explicit 24 catalogue/card/TOML/native XML paths and engine references, rechecks source hashes, and writes only the 13 assigned document files. Then run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-check-per-model-six-theory-20261103.py`. The adjacent `per-model-six-components.json` contains all 144 rows and full actual plant derivations. The generation script retains historical comparisons without wrapping them again.

The JSON deliberately sets current `passed`, numerical comparison and native acceptance false. A source-derived sensitivity or synthetic method benchmark cannot replace current model-native captures. No scenario, gain, threshold, provider, geometry or runtime code was changed for this projection.
