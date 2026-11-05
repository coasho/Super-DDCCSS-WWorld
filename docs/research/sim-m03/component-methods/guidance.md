# Per-model measured guidance method and applicability

This package documents the actual shared `fsim.guidance.measured_pn.v2` software method for all 24 selected AAM configurations. It does not invent 24 independent guidance laws or claim calibrated hardware performance. The correction candidate has not been executed natively. Existing `passed`, numerical/native acceptance and calibration flags remain unadmitted.

## Exact shared law

- `h=unit(v_own_estimated), u=unit(up), g0=9.80665m/s2; P_h(a)=a-h*(h dot a). unit(0)=0 in the native numerical contract.`
- `For a tracking admitted measurement: l=track.direction, d=track.directionRate. If measurementTimeS is present, l=unit(l+d*(frame.timeS-track.sampleTimeS)); derivative d is not re-estimated by guidance.`
- `c=norm(v_own_estimated) when rangeRateMs is absent; otherwise c=max(0,-rangeRateMs), including zero/opening clamp with no own-speed fallback.`
- `Tracking steering is zero when rateAvailable=false, otherwise a_s=N*c*((l cross d) cross h)=N*c*(d*(l dot h)-l*(d dot h)). This is not generally N*c*P_h(d).`
- `With a positive measured c and measured range, timeToGo=range/c; absent/nonpositive measured closing leaves timeToGo unavailable, even when the bearing branch uses own speed.`
- `Permitted midcourse: p_est=e.position+e.velocity*(frame.timeS-e.sampleTimeS), r=p_est-own_position, R=norm(r), l_r=unit(r), v_rel=e.velocity-own_velocity, c_r=-v_rel dot l_r.`
- `For inertialMidcourse=true, no tracked measurement, R>0 and c_r>0: a_s=N*c_r*(((l_r cross v_rel)/R) cross h); timeToGo=R/c_r. Otherwise midcourse PN is zero.`
- `Only when the reported phase is Midcourse and inertialMidcourse=true: a_s+=sin(clamp(loftRad,0,0.7))*loftAccelerationG*g0*(u-h*(u dot h)); a Lost terminal latch suppresses loft, not permitted inertial estimate fallback.`
- `z=P_h(a_s)+g0*u; G=maxG*g0; command=z when norm(z)<=G, otherwise command=G*z/norm(z). Gravity is included before the total-vector bound.`
- `terminal_next=terminal_previous OR (engineering.version==3 AND tracking); phase=Homing if tracking, else Lost if terminal_next or retained track localId, else Midcourse if estimate present, else Seeking. Invalid calls return default Unguided and do not update the latch; resetGuidanceStateV1 clears it.`
- `supported=result.supported AND (seeker==semi_active_radar OR launcherUpdates); terminal measurements never use hidden target position/velocity. A launcher estimate is a separate permitted input, not a terminal range-rate substitute.`

The scalar Lagrange identity is the independent mathematical reference. Replacing the nested cross with a simple projection of LOS rate is valid only under additional geometric alignment conditions. The documented total limit includes gravity; it is a demand bound, not measured flight capability.

## State, inputs, domains and assumptions

- result.valid=true, finite own position/velocity/up, nonnegative finite own age, finite maxG>0 and finite navigationGain>2 are required. V3 own absolute frame time must be finite and nonnegative.
- For V3 tracking: a track, nonzero localId, finite nonnegative original track sampleTime<=frame.time, finite nonzero direction, finite directionRate even when rate unavailable, finite admitted optional range>=minimumRange and finite optional rangeRate are required.
- Normal PN interpretation assumes nonzero forward own velocity and admitted unit bearing/tangent rate; unit(0)=0 and unnormalised no-measurementTime inputs are numerical interface branches, not physical flight validity claims.
- The validity checks do not impose a new freshness window. Receiver delivery, scan completion, association, noise, covariance, rate availability and own-navigation synchronization remain upstream native responsibilities.
- The prediction branch is enabled by measurementTimeS presence and uses track.sampleTimeS; it does not use measurementTimeS value or deliveredTimeS as a replacement original epoch.
- Only the enabled inertialMidcourse branch validates its estimate as finite with 0<=sampleTime<=frame.time. Estimate presence determines phase even when inertialMidcourse=false, but that disabled branch does not validate/use the estimate for steering or loft.
- R=0 or nonclosing inertial estimate produces no PN; eligible Midcourse loft can remain. Loss of tracking is not a commanded zero-acceleration state because gravity and possibly permitted inertial fallback remain.
- G limits the demand, not realized aerodynamic/actuator authority. Plant mass/inertia, finite fins/TVC and directional autopilot realization are separate downstream mechanisms and are not proofs of delayed guidance stability.
- Shared N and game receiver/model inputs remain engineering assumptions with the original nonzero/unquantified uncertainty records. This is neither OEM identification, hardware calibration nor a claim of 24 distinct guidance laws.

The only persistent state in this provider is the terminal-acquisition latch. Track samples, receiver quality, covariance, original/receive times, own-navigation state and permitted launcher estimates remain input/producer responsibilities. Absolute sample time is not replaced with the output publication time. `timeToActiveS` is copied when available; it is not a new guidance prediction.

## Sensitivity and failure branches

- For unsaturated tracking at a fixed admitted l,d,h and capability scale c: da_s/dN=c*((l cross d) cross h), da_s/dc=N*((l cross d) cross h). rateAvailable=false makes both steering sensitivities zero.
- c=max(0,-rangeRate): derivative is -1 for strictly negative radial velocity and zero for strictly positive radial velocity; the zero boundary is nonsmooth and must be tested as its own branch.
- For z outside the total bound, command Jacobian versus z is G/norm(z)*(I-z*z^T/norm(z)^2); inside it is I. Pure radial rescaling in a saturated direction can leave command unchanged, so parameter perturbation is not automatically nonzero response.
- Epoch projection l(tau)=unit(l0+d*tau) has derivative (I-l*l^T)*d/norm(l0+d*tau). With tangent d at tau=0, the derivative of (l cross d) can vanish; do not invent first-order delay sensitivity or substitute Ts/2+processing for an actual capture epoch.
- Midcourse loft is zero for the actual zero-loft cards and after a terminal Lost latch. Changing launcherUpdates alone does not create inertialMidcourse or a measured terminal range/range-rate.

The JSON gives -10%, nominal and +10% algebraic responses for each model using its own archived LOS/rate, capability scale and configuration. It labels whether a nonzero command response actually occurs. These perturbations change no maintained card, gain, sensor, source, scene or trajectory. Original parameter uncertainty and basis records remain unchanged.

## Per-model operating projections

The archive is `build/architecture-reviews/sim-m03-current-20261104/full24-current-models/<model>/`. Its captures precede subsequent receiver/own-frame repairs, have `admission_eligible=false` and an empty producer source epoch. These are explicitly historical illustrations. Analysis-source/card hashes record what this authoring read; they do not establish those archived producers.

Counts and ranges below use actual archived tracking publication samples. Published navigation velocity, original track epoch and control demand have different epochs; no exact native guidance command residual is asserted from them. The algebraic example fixes local up to +z and combines archived LOS/rate with published navigation velocity. It is a disclosed mathematical projection, not the actual ECEF decision or a new trajectory. The prior 500m/s mass/authority/pole and heuristic Ts/2+processing examples are retained only in `historical_theory_projection`, explicitly superseded.

| Model | Receiver | maxG | Inertial / updates | Loft deg | Tracked / rate / radial | Published nav speed m/s | Measured closing m/s |
|---|---|---:|---|---:|---|---|---|
| aim-120c | rf_signal.v1 | 35 | 1 / 1 | 20 | 73 / 69 / 73 | 471.265..690.234 | 712.694..938.278 |
| aim-120d | rf_signal.v1 | 35 | 1 / 1 | 20 | 72 / 70 / 72 | 526.6..696.971 | 766.297..943.936 |
| aim-7m | rf_signal.v1 | 35 | 0 / 0 | 20 | 153 / 152 / 0 | 257.545..607.094 | unavailable |
| aim-9m | thermal_conscan_reticle.v1 | 40 | 0 / 0 | 0 | 9 / 3 / 0 | 462.289..697.229 | unavailable |
| aim-9x | thermal_image.v1 | 40 | 0 / 0 | 0 | 149 / 148 / 0 | 259.099..970.167 | unavailable |
| asraam | thermal_image.v1 | 40 | 1 / 0 | 0 | 125 / 124 / 0 | 270.752..1026.51 | unavailable |
| iris-t | thermal_image.v1 | 40 | 0 / 0 | 0 | 147 / 146 / 0 | 259.586..1003.87 | unavailable |
| magic-2 | thermal_conscan_reticle.v1 | 40 | 0 / 0 | 0 | 15 / 11 / 0 | 449.481..837.52 | unavailable |
| meteor | rf_signal.v1 | 35 | 1 / 1 | 20 | 80 / 76 / 80 | 431.251..565.888 | 673.526..815.528 |
| mica-em | rf_signal.v1 | 35 | 1 / 1 | 20 | 61 / 57 / 61 | 532.237..818.352 | 773.364..1068.89 |
| mica-ir | thermal_dual_image.v1 | 40 | 1 / 1 | 0 | 151 / 150 / 0 | 259.274..818.19 | unavailable |
| pl-10 | thermal_image.v1 | 40 | 1 / 0 | 0 | 149 / 148 / 0 | 259.281..973.668 | unavailable |
| pl-12 | rf_signal.v1 | 35 | 1 / 1 | 20 | 99 / 94 / 99 | 259.656..729.665 | 501.625..978.656 |
| pl-15 | rf_signal.v1 | 35 | 1 / 1 | 20 | 108 / 103 / 108 | 258.193..756.807 | 501.264..1007.6 |
| pl-8 | thermal_conscan_reticle.v1 | 40 | 0 / 0 | 0 | 9 / 5 / 0 | 541.379..796.66 | unavailable |
| r-27er | rf_signal.v1 | 35 | 1 / 1 | 20 | 155 / 154 / 0 | 256.709..604.727 | unavailable |
| r-27et | thermal_conscan_reticle.v1 | 40 | 0 / 0 | 0 | 41 / 35 / 0 | 256.762..598.782 | unavailable |
| r-27r | rf_signal.v1 | 35 | 1 / 1 | 20 | 160 / 159 / 0 | 254.469..634.103 | unavailable |
| r-27t | thermal_conscan_reticle.v1 | 40 | 0 / 0 | 0 | 18 / 13 / 0 | 254.481..598.926 | unavailable |
| r-37m | rf_signal.v1 | 35 | 1 / 1 | 20 | 155 / 154 / 155 | 255.004..652.94 | 500.877..898.313 |
| r-60m | thermal_conscan_reticle.v1 | 40 | 0 / 0 | 0 | 6 / 3 / 0 | 806.69..810.502 | unavailable |
| r-73 | thermal_conscan_reticle.v1 | 40 | 0 / 0 | 0 | 23 / 14 / 0 | 469.214..808.487 | unavailable |
| r-77m | rf_signal.v1 | 35 | 1 / 1 | 20 | 99 / 95 / 99 | 259.135..814.074 | 501.491..1064.77 |
| super-530d | rf_signal.v1 | 35 | 0 / 0 | 20 | 151 / 150 / 0 | 260.333..623.395 | unavailable |

### aim-120c

- Actual selected receiver=rf_signal.v1; measurement_family=radar; seeker=active_radar.
- Unchanged N=3.5, total maxG=35, loft=20deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=True.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 73 tracked packets with radial availability and 69 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `13.399999999999963s`, publication `13.466666666666629s`; disclosed capability-scale projection `780.064217m/s`. Original guidance parameter SHA `fbf3c62b35fd77a9df5a230dc4132a924207042189a267a4034eaa6325131c9f` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### aim-120d

- Actual selected receiver=rf_signal.v1; measurement_family=radar; seeker=active_radar.
- Unchanged N=3.5, total maxG=35, loft=20deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=True.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 72 tracked packets with radial availability and 70 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `13.333333333333297s`, publication `13.399999999999963s`; disclosed capability-scale projection `771.394211m/s`. Original guidance parameter SHA `177b6ca4c80f31ad9ed7fd902e4c28cf2ee19ec4103225cc2c6f22f65a6c079c` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### aim-7m

- Actual selected receiver=rf_signal.v1; measurement_family=bearing; seeker=semi_active_radar.
- Unchanged N=3.5, total maxG=35, loft=20deg, loftAccelerationG=2, inertialMidcourse=False, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 152 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.899999999999968s`, publication `11.966666666666635s`; disclosed capability-scale projection `261.154039m/s`. Original guidance parameter SHA `55b90d42b0babe8ddb5227a2e9168b0a542647df9fbc89bd277cbf96a55b46ab` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### aim-9m

- Actual selected receiver=thermal_conscan_reticle.v1; measurement_family=bearing; seeker=infrared.
- Unchanged N=3.5, total maxG=40, loft=0deg, loftAccelerationG=2, inertialMidcourse=False, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 3 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `13.160759944418359s`, publication `13.233333333333297s`; disclosed capability-scale projection `489.097632m/s`. Original guidance parameter SHA `51a1799852ff2f44a56736ea61f9252115338b35fc0a856b8d4c010a7de6ab1b` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### aim-9x

- Actual selected receiver=thermal_image.v1; measurement_family=bearing; seeker=imaging_infrared.
- Unchanged N=3.5, total maxG=40, loft=0deg, loftAccelerationG=2, inertialMidcourse=False, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 148 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.466666666666637s`, publication `11.533333333333303s`; disclosed capability-scale projection `267.674549m/s`. Original guidance parameter SHA `b233c0b68de448c6bcdc8423472f9035f2abe959396f71a546b3995ba5da7dbb` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### asraam

- Actual selected receiver=thermal_image.v1; measurement_family=bearing; seeker=imaging_infrared.
- Unchanged N=3.5, total maxG=40, loft=0deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 124 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.466666666666637s`, publication `11.533333333333303s`; disclosed capability-scale projection `289.63223m/s`. Original guidance parameter SHA `1e3cd5eef54af36643370af822f17b3bf94cfdd97ba7815340d6d9263b26db18` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### iris-t

- Actual selected receiver=thermal_image.v1; measurement_family=bearing; seeker=imaging_infrared.
- Unchanged N=3.5, total maxG=40, loft=0deg, loftAccelerationG=2, inertialMidcourse=False, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 146 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.466666666666637s`, publication `11.533333333333303s`; disclosed capability-scale projection `268.585944m/s`. Original guidance parameter SHA `04143ce3d884319bfcdb524f49fb80902cf10d63ba05f38f2f6705c2d289ea52` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### magic-2

- Actual selected receiver=thermal_conscan_reticle.v1; measurement_family=bearing; seeker=infrared.
- Unchanged N=3.5, total maxG=40, loft=0deg, loftAccelerationG=2, inertialMidcourse=False, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 11 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `13.062742253626361s`, publication `13.133333333333297s`; disclosed capability-scale projection `771.239341m/s`. Original guidance parameter SHA `e633631307113c387fd43eb1e99431e9eebd8bb1e93020a7c2f4ccf6e1c94353` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### meteor

- Actual selected receiver=rf_signal.v1; measurement_family=radar; seeker=active_radar.
- Unchanged N=3.5, total maxG=35, loft=20deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=True.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 80 tracked packets with radial availability and 76 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `13.166666666666631s`, publication `13.233333333333297s`; disclosed capability-scale projection `737.221144m/s`. Original guidance parameter SHA `f37223aba2e7d5e557904e27b725c621e1e2ff7ae329997ac8c68fc21de86fe9` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### mica-em

- Actual selected receiver=rf_signal.v1; measurement_family=radar; seeker=active_radar.
- Unchanged N=3.5, total maxG=35, loft=20deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=True.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 61 tracked packets with radial availability and 57 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `13.499999999999963s`, publication `13.566666666666629s`; disclosed capability-scale projection `787.660905m/s`. Original guidance parameter SHA `ff390680932078d2c2e655243cc2562e79870f5d00579013f25ec9707a46f824` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### mica-ir

- Actual selected receiver=thermal_dual_image.v1; measurement_family=bearing; seeker=imaging_infrared.
- Unchanged N=3.5, total maxG=40, loft=0deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=True.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 150 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.466666666666637s`, publication `11.533333333333303s`; disclosed capability-scale projection `268.006628m/s`. Original guidance parameter SHA `e6046050f4917cc0ffcbe6099a2f8242bfe22ee3740c723ccce6fdfe1f1a5fbd` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### pl-10

- Actual selected receiver=thermal_image.v1; measurement_family=bearing; seeker=imaging_infrared.
- Unchanged N=3.5, total maxG=40, loft=0deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 148 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.466666666666637s`, publication `11.533333333333303s`; disclosed capability-scale projection `268.029621m/s`. Original guidance parameter SHA `e0c48a844510701246c677da88f5a7dd8d825dacb1d642b63c9e6fddab99b140` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### pl-12

- Actual selected receiver=rf_signal.v1; measurement_family=radar; seeker=active_radar.
- Unchanged N=3.5, total maxG=35, loft=20deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=True.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 99 tracked packets with radial availability and 94 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `12.533333333333299s`, publication `12.599999999999966s`; disclosed capability-scale projection `659.787173m/s`. Original guidance parameter SHA `b15d02bd6b1258fee5eb7580d847ff675a6cb7b1d85e9886066aabe9683060bf` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### pl-15

- Actual selected receiver=rf_signal.v1; measurement_family=radar; seeker=active_radar.
- Unchanged N=3.5, total maxG=35, loft=20deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=True.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 108 tracked packets with radial availability and 103 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.633333333333303s`, publication `11.699999999999969s`; disclosed capability-scale projection `528.883613m/s`. Original guidance parameter SHA `be4dde56eef7d0cb371c6e7ec7ca76186dc3901f4c238e8a4ff32ad8a1589c81` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### pl-8

- Actual selected receiver=thermal_conscan_reticle.v1; measurement_family=bearing; seeker=infrared.
- Unchanged N=3.5, total maxG=40, loft=0deg, loftAccelerationG=2, inertialMidcourse=False, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 5 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `13.160759944418359s`, publication `13.233333333333297s`; disclosed capability-scale projection `578.626854m/s`. Original guidance parameter SHA `730c253252738ad187ab2278ca67c3b74049732c10e450569638c33fd30869e2` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### r-27er

- Actual selected receiver=rf_signal.v1; measurement_family=bearing; seeker=semi_active_radar.
- Unchanged N=3.5, total maxG=35, loft=20deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=True.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 154 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.899999999999968s`, publication `11.966666666666635s`; disclosed capability-scale projection `259.954697m/s`. Original guidance parameter SHA `6be11b0dea5733a35b4ab97d15262bdd528191df256641186ad43fc946713842` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### r-27et

- Actual selected receiver=thermal_conscan_reticle.v1; measurement_family=bearing; seeker=infrared.
- Unchanged N=3.5, total maxG=40, loft=0deg, loftAccelerationG=2, inertialMidcourse=False, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 35 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.524618490428796s`, publication `11.599999999999969s`; disclosed capability-scale projection `269.849712m/s`. Original guidance parameter SHA `e9c8c8f4d860f8867f3709ea7b77e4741f4a0a75e106cf2ab1b8fefce10bcec8` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### r-27r

- Actual selected receiver=rf_signal.v1; measurement_family=bearing; seeker=semi_active_radar.
- Unchanged N=3.5, total maxG=35, loft=20deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=True.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 159 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.899999999999968s`, publication `11.966666666666635s`; disclosed capability-scale projection `256.73336m/s`. Original guidance parameter SHA `ff2dcc9cdfe95d038c1405424c9d99a416da1152896e48c161d722e241bb717f` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### r-27t

- Actual selected receiver=thermal_conscan_reticle.v1; measurement_family=bearing; seeker=infrared.
- Unchanged N=3.5, total maxG=40, loft=0deg, loftAccelerationG=2, inertialMidcourse=False, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 13 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.524618490428796s`, publication `11.599999999999969s`; disclosed capability-scale projection `263.569982m/s`. Original guidance parameter SHA `7b49b26e3c8dd9e7d7b054068c324e9380f8c5680a59470e7e66be9e8dda7b68` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### r-37m

- Actual selected receiver=rf_signal.v1; measurement_family=radar; seeker=active_radar.
- Unchanged N=3.5, total maxG=35, loft=20deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=True.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 155 tracked packets with radial availability and 154 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.433333333333303s`, publication `11.49999999999997s`; disclosed capability-scale projection `503.442289m/s`. Original guidance parameter SHA `145f53478e16f267cb51a98cd71b09ac433109f48d0db80913f5df30e647a33c` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### r-60m

- Actual selected receiver=thermal_conscan_reticle.v1; measurement_family=bearing; seeker=infrared.
- Unchanged N=3.5, total maxG=40, loft=0deg, loftAccelerationG=2, inertialMidcourse=False, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 3 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `14.62348548392977s`, publication `14.699999999999958s`; disclosed capability-scale projection `808.593635m/s`. Original guidance parameter SHA `9028b995f1940e9a78f50a39447701d5b09bd9c3af7ad98820b7a41733154535` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### r-73

- Actual selected receiver=thermal_conscan_reticle.v1; measurement_family=bearing; seeker=infrared.
- Unchanged N=3.5, total maxG=40, loft=0deg, loftAccelerationG=2, inertialMidcourse=False, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 14 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `13.092901543100822s`, publication `13.166666666666631s`; disclosed capability-scale projection `487.644286m/s`. Original guidance parameter SHA `fc17ca9aa7da39726476521b9c96c11516225361ffb9beb25b82d1145fc23d8f` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### r-77m

- Actual selected receiver=rf_signal.v1; measurement_family=radar; seeker=active_radar.
- Unchanged N=3.5, total maxG=35, loft=20deg, loftAccelerationG=2, inertialMidcourse=True, launcherUpdates=True.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 99 tracked packets with radial availability and 95 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.666666666666636s`, publication `11.733333333333302s`; disclosed capability-scale projection `536.276042m/s`. Original guidance parameter SHA `a51a5dd91dacc55092705648a090910a7cfacb75adbef4beab297bb59e504ea7` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

### super-530d

- Actual selected receiver=rf_signal.v1; measurement_family=bearing; seeker=semi_active_radar.
- Unchanged N=3.5, total maxG=35, loft=20deg, loftAccelerationG=2, inertialMidcourse=False, launcherUpdates=False.
- Native scale uses the admitted optional radial measurement, not model name. This archive contains 0 tracked packets with radial availability and 150 rate-available packets.
- The archived per-model speed/radial/availability population below determines a distinct operating projection; it does not imply a distinct differential guidance law.
- Original track epoch `11.899999999999968s`, publication `11.966666666666635s`; disclosed capability-scale projection `265.174707m/s`. Original guidance parameter SHA `beef5eab461a26e157cb837a0b62920596e4290d3d48d6029c254dd9d9702c59` remains its recorded epoch.
- Applicable state/branch/domain and input uncertainty are the exact shared rules above plus this row's actual capability flags. A rate-less optical sample has no PN steering; an admitted radial measurement clamps opening speed instead of substituting own speed. See the row for its numerical domain, per-model sensitivity outputs and archived file/binary identity.

## Native analytical comparison and reproduction

The additive helper in `test_component_reference_native.cpp` runs within each of the existing 24 model rows and serializes `guidance_branches`: original synthetic inputs, latch precondition, expected/actual acceleration, phase, availability and arithmetic tolerance. The ordinary `component_reference_native` wrapper retains its exact two selected Catch cases. The focused `[component-guidance-reference]` case calls the same helper on all 24 unchanged loaded profiles. Planned branch comparisons total 328 (15 for each of 8 ARH configurations and 13 for each of 16 bearing configurations); these are branch comparisons, not a claimed Catch test/assertion count.

Comparisons cover oblique terminal Lagrange identity, no-rate with nonzero stored derivative, own-speed fallback, actual-ARH measured zero/opening clamp, original-epoch prediction, gravity-inclusive total clipping, seeking/loss/reset, rejected future sample not setting the latch, permitted/disabled midcourse, Lost-latch loft suppression and opening midcourse. The epoch-prediction direct-interface negative control deliberately sets the presence-only result marker to0.9s while retaining the authoritative track sample at0.75s, so predicting from the wrong epoch fails. This is a method contract control, not a claimed real receiver packet. Expected accelerations use scalar dot products rather than native cross helpers. The bound is `128*epsilon*max(1,maxG*g0,abs(expected_component))`, justified only for the disclosed finite arithmetic inputs; it changes no physical acceptance tolerance.

After root applies/reviews the candidate and holds the native slot, use the existing UCRT64 launcher and environment. The ordinary registered source-bound run is:

```powershell
$env:TMP="D:/ENV/tmp"; $env:TEMP="D:/ENV/tmp"; $env:TORCH_EXTENSIONS_DIR="D:/ENV/tmp"
D:/ENV/python/3.13/python.exe -B tools/validation/weapon_quality/component_reference_run.py --kind kinematics --executable build/ucrt64-release/tests/fsim_component_reference_native_test.exe --output build/ucrt64-release/validation/component_reference_runs/kinematics
```

This creates its fresh pre-execution epoch and existing `kinematics.json`, now containing the additive branch records. For an additional focused diagnostic, set `FSIM_COMPONENT_REFERENCE_EPOCH` to that actual freshly created run's `input-epoch.json` (use the exact path recorded by its execution receipt), then invoke the same executable with `[component-guidance-reference] --reporter junit --out <new-report.xml>` and capture unfiltered stdout. The focused diagnostic does not replace the ordinary wrapper, full-chain, installed or Viewer gates. Its recorded epoch must still match current inputs; an old path cannot admit a new run.

No native compile, execution, gate admission or matrix promotion was performed while authoring this candidate. Source-only changes and archive projections do not establish a passed method. Root must run the native comparison, inspect positive case/assertion counts and raw records, and rerun actual model-native encounters as required.

## Primary theory reference

- [Palumbo, Blauwkamp and Lloyd, Basic Principles of Homing Guidance, JHU APL Technical Digest 29(1), 2010](https://secwww.jhuapl.edu/techdigest/content/techdigest/pdf/V29-N01/29-01-Palumbo_Principles_Rev2018.pdf), introduction and PN/LOS reconstruction discussion: conceptual near-collision guidance, sensed LOS reconstruction and finite acceleration realization. This supports generic assumptions, not the simulator's model-specific gain or OEM performance.
- Exact branch/state behavior is defined by the maintained `MeasuredPnGuidanceV2.cpp`, its versioned reset interface and admitted `SeekerExecutionV2` value contract. The source-bound JSON separates current analysis bytes from archived producer identities.
