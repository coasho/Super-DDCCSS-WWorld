# Current directional autopilot theory for the 24 game plants

This package derives the selected 22 load-feedback cascade laws, PL-10 actuator-aware law and R-77M bounded operating-point law from current software. Inputs are existing game-plant PRE captures. No card, force, gain, actuator limit or outcome is changed. Numerical references are distinct from current native acceptance.

## Equations and physical ownership

- `m*(v_dot_body + omega cross v_body)=F_nongravity_body+m*g_body; J*omega_dot=M-omega cross(J*omega)`
- `pitch s=+1, theta=alpha, rate=q, transverse=z; yaw s=-1, theta=beta, rate=r, transverse=y`
- `theta_dot=s*rate+(F_theta+m*g_theta-(F_x+m*g_x)*theta)/(m*V) in the disclosed small-angle reduction`
- `R_F=F_measured_theta-F_angle*theta-F_control*delta_actual; R_M=M_measured_axis-M_angle*theta-M_rate*rate-M_control*delta_actual; G=(omega cross J*omega)_axis`
- `rate_ref=-s*(demand_theta+g_theta)/V+s*(demand_x+g_x)*theta_ref/V`
- `(F_angle-F_x+m*demand_x)*theta_ref+F_control*delta_trim+R_F=m*demand_theta; M_angle*theta_ref+M_rate*rate_ref+M_control*delta_trim+R_M-G=0`
- `D=F_angle-F_control*M_angle/M_control-F_x+m*demand_x-s*F_control*M_rate*(demand_x+g_x)/(M_control*V)`
- `theta_ref=[m*demand_theta-R_F+(F_control/M_control)*(M_rate*(-s*(demand_theta+g_theta)/V)+R_M-G)]/D`
- `if hypot(alpha_ref,beta_ref)>incidence_limit: retain angle direction and solve rate_ref=-s*[(F_eff-F_x-m*g_x)*theta_ref+R_F-(F_control/M_control)*(R_M-G)+m*g_theta]/[m*V-s*F_control*M_rate/M_control]`
- `x=[theta,rate,actual_virtual_surface]; A=[[(F_angle-F_x-m*g_x)/(m*V),s,F_control/(m*V)],[M_angle/J_axis,M_rate/J_axis,M_control/J_axis],[0,0,-1/tau]]; B=[0,0,1/tau]^T`
- `cascade K=[(J_axis*s/(T_rate*T_incidence)+M_angle)/M_control,(J_axis/T_rate+M_rate)/M_control,0]; closed_A=A-B*K only in frozen unsaturated residual-constant domain`
- `actuator-aware K assigns eig(A-B*K)=[-1/tau,-1/T_rate,-1/T_incidence] using CURRENT axial/gravity A, subject to its finite controllability determinant`
- `roll_surface=[-J_x*p/T_rate+(J_z-J_y)*q*r-L_damping*p-R_L]/L_control; normalize each signed channel with its actual positive/negative stop`
- `finite TVC: theta_nozzle_demand=clip(G_pitch*delta_virtual_pitch,+/-limit); analogous yaw; actual nozzle angle is native rate-limited and stopped, never an instantaneous substituted force`
- `native TVC derivatives: dFz/ddelta=-T*cos(theta_nozzle)*G_pitch; dFy/ddelta=T*cos(theta_nozzle)*cos(psi_nozzle)*G_yaw; moments use actual nozzle-to-CG arm; measured Fx is total nongravity force counted once`
- `operating-point search uses the directional cascade as a seed/target, advances captured channel-input/lag/rate/stops and physical-fin states, interpolates current finite moment curves, then minimizes sum_axis((M_pred-M_requested)/J_axis)^2 over normalized channel bounds; finite search is not global optimality`

The sign convention is x forward, y right, z down. Body acceleration obeys Newton/Euler equations; directional acceleration is the projection normal to velocity, so total axial force and axial gravity enter incidence kinematics. A positive thrust with zero aerodynamic derivatives can retain `D=-thrust`, provided its finite nozzle has actual moment authority. The old aerodynamic-only `F_eff` and `A00=F_angle/(mV)` are not the current measured-axial law, including in a coast with nonzero drag.

## State and domain

- SI body quantities and the same native PRE epoch; F_x is total nongravity axial force including propulsion, while transverse/moment residual ownership follows native aero/TVC composition.
- Small incidence, forward velocity cos(alpha)*cos(beta)>0, speed>=1m/s, positive mass/diagonal inertia, finite derivatives; the nonlinear native plant remains authority outside the frozen reduction.
- Current directional trim requires finite nonzero M_control and its own D determinant; aerodynamic F_eff=0 alone does not imply absence of powered directional authority.
- Finite stops, incidence projection, rate limits and actuator metadata agreement remain active. An eigenvalue calculation excludes those nonsmooth branches and is not a coupled stability/encounter proof.
- Every displayed numerical point is an explicitly supplied captured PRE input. No force, thrust, gravity, CG, inertia or motor age is inferred from a generic 500m/s operating point.
- The retained old 500m/s/6000m launch/half/depleted aerodynamic matrices are historical projections only; they omit current axial/gravity directional terms and cannot count as current poles or current acceptance.
- TVC combined local control derivatives apply only while the actual native nozzle follows its finite demand without rate/stop limiting; separate nozzle states/rate events invalidate a single unsaturated three-state pole interpretation.
- Signed controls have distinct force/moment polarity. Roll-inertial coupling and finite native FCS states are retained; no invented integral state, missing authority fallback or ideal-force execution is introduced.

Controller output is a normalized native channel request. Native FCS owns channel lag, signed stops, mixing, each physical fin's rate/position/malfunction, and nozzle rate/stops. Cascade has no invented integral state; the three-state A includes an unsaturated continuous virtual actuator only. The operating-point mechanism additionally retains previous channel INPUT, lag, actual position and every physical fin. It evaluates actual piecewise native moment curves with the correct body/current-CG transform.

## Executed mathematical references

The maintained Python tests independently solve the three coupled force/moment/incidence equations with NumPy, differentiate body-velocity Newton equations, compare the applicable 23 selected laws to captured native commands, compare PL-10 gains with SciPy `place_poles`, and compare the R-77M virtual lag to SciPy's bilinear transform. R-77M's captured command is independently evaluated against neutral/eight-corner feasible controls with the captured finite FCS and fin curves; this is not a claim of global optimizer superiority. Powered zero-aero authority and true singular refusal are separate controls.

Capture root: `build/architecture-reviews/sim-m03-current-20261104/baseline24`. Label: Frozen baseline24 initial native PRE illustrations; captured before later RK4/reticle repairs; not current native acceptance. Every JSON row records its actual input epoch and selected PRE input. No generator invocation upgrades it to current native acceptance. All 24 `passed` and `current_native_acceptance` fields remain false in this theory projection.

## Actual model differences

| Model | Selected law | PRE time s | Mass kg | Fx N | Pitch F_eff N/rad | Pitch D N/rad | Current directional local max pole 1/s | TVC |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| aim-120c | cascade | 11.391667 | 161.416 | 14995.88 | -4585.624 | -19514.47 | -6.388779 | none |
| aim-120d | cascade | 11.391667 | 161.416 | 15032.62 | -4549.162 | -19514.76 | -6.389065 | none |
| aim-7m | cascade | 11.858333 | 224.908 | 16994.16 | -18386.88 | -35380.84 | -7.435358 | none |
| aim-9m | cascade | 11.391667 | 85.4577 | 7498.703 | -7072.611 | -14546.57 | -3.388294 | none |
| aim-9x | cascade | 11.391667 | 83.9578 | 7521.792 | -1696.896 | -9194.164 | -3.867314 | finite |
| asraam | cascade | 11.391667 | 87.9044 | 17247.26 | -2238.333 | -19459.95 | -5.397577 | none |
| iris-t | cascade | 11.391667 | 87.3538 | 8223.622 | -1572.484 | -9770.593 | -3.712665 | finite |
| magic-2 | cascade | 11.391667 | 89.9041 | 17275.36 | -7926.697 | -25175.99 | -4.082099 | none |
| meteor | cascade | 11.391667 | 189.905 | 16988.28 | -2665.207 | -19574.67 | -5.632901 | none |
| mica-em | cascade | 11.391667 | 111.942 | 10205.09 | -2480.388 | -12639.01 | -4.370713 | finite |
| mica-ir | cascade | 11.391667 | 111.942 | 10204.21 | -2638.738 | -12810.1 | -4.504564 | finite |
| pl-10 | actuator-aware | 11.391667 | 88.9539 | 8105.001 | -2813.578 | -10892.48 | -2.5 | finite |
| pl-12 | cascade | 11.391667 | 179.905 | 17009.43 | -7741.294 | -24675.94 | -5.333763 | none |
| pl-15 | cascade | 11.391667 | 209.905 | 17036 | -2970.621 | -19918.71 | -5.715988 | none |
| pl-8 | cascade | 11.391667 | 114.923 | 13706.01 | -12474.14 | -26146.84 | -3.230482 | none |
| r-27er | cascade | 11.858333 | 349.872 | 23687.72 | -9392.053 | -33079.61 | -5.787979 | none |
| r-27et | cascade | 11.391667 | 347.872 | 23659.36 | -9046.407 | -32600.54 | -5.453555 | none |
| r-27r | cascade | 11.858333 | 252.934 | 11921.15 | -9457.342 | -21378.4 | -5.963917 | none |
| r-27t | cascade | 11.391667 | 252.934 | 11900.55 | -9146.453 | -20971.55 | -5.453255 | none |
| r-37m | cascade | 11.391667 | 509.848 | 26382.07 | -10197.65 | -36358.57 | -7.347737 | none |
| r-60m | cascade | 11.391667 | 44.9693 | 5441.484 | -2356.495 | -7784.943 | -4.890677 | none |
| r-73 | cascade | 11.391667 | 104.949 | 9488.392 | -4286.194 | -13743.77 | -6.657838 | finite |
| r-77m | operating-point (seed only) | 11.391667 | 189.904 | 17053.82 | -3854.043 | -20828.89 | -4.402445 | none |
| super-530d | cascade | 11.858333 | 269.854 | 27200.32 | -7623.568 | -34823.98 | -6.782152 | none |

Each pole is a frozen unsaturated mathematical illustration at that PRE input, not measured coupled damping. The R-77M entry is only its directional cascade initialization; it does not describe the bounded nonlinear search's closed-loop poles. Old 500m/s/6000m three-mass-state numbers remain explicitly historical in JSON and are not silently relabeled.

### aim-120c

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:aim120c; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=161.416088325kg, speed=251.173883994m/s, Jyy=180.583808296kgm2, total Fx=14995.8765149N.
Actual pitch D=-19514.4712724N/rad vs aerodynamic-only F_eff=-4585.62441704; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-18014.884 -> -21014.059 N/rad`; local maximum real pole `-6.1746897 -> -6.6380756 1/s`; trim surface `0.00069989382 -> 0.00013798361 rad`; normalized elevator `-0.26741893 -> -0.21830702`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-19514.471 -> -19514.471 N/rad`; local maximum real pole `-6.3887794 -> -6.3887794 1/s`; trim surface `0.0095092227 -> -0.0070578209 rad`; normalized elevator `-0.24177397 -> -0.24032307`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### aim-120d

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:aim120d; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=161.416097397kg, speed=251.17927083m/s, Jyy=180.590634853kgm2, total Fx=15032.6152179N.
Actual pitch D=-19514.7611529N/rad vs aerodynamic-only F_eff=-4549.16188448; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-18011.5 -> -21018.023 N/rad`; local maximum real pole `-6.1744358 -> -6.6391059 1/s`; trim surface `0.00070778377 -> 0.00014382913 rad`; normalized elevator `-0.26750196 -> -0.21826688`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-19514.761 -> -19514.761 N/rad`; local maximum real pole `-6.3890648 -> -6.3890648 1/s`; trim surface `0.0095167256 -> -0.0070517116 rad`; normalized elevator `-0.24178737 -> -0.24033412`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### aim-7m

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:aim7m; 4 controlled physical-fin rate entries.
Captured PRE t=11.858333333333302s: mass=224.908036763kg, speed=250.935091899m/s, Jyy=248.862922789kgm2, total Fx=16994.1636042N.
Actual pitch D=-35380.8438811N/rad vs aerodynamic-only F_eff=-18386.8800698; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-33681.428 -> -37080.26 N/rad`; local maximum real pole `-7.4655729 -> -7.4089638 1/s`; trim surface `0.0019479277 -> 0.0019595807 rad`; normalized elevator `-0.062434339 -> -0.062180468`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-35380.844 -> -35380.844 N/rad`; local maximum real pole `-7.4353579 -> -7.4353579 1/s`; trim surface `0.003373785 -> 0.00079241967 rad`; normalized elevator `-0.065778375 -> -0.059456432`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### aim-9m

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:aim9m; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=85.457686175kg, speed=251.105627966m/s, Jyy=58.7977840276kgm2, total Fx=7498.70253338N.
Actual pitch D=-14546.5718979N/rad vs aerodynamic-only F_eff=-7072.61056384; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-13796.702 -> -15296.442 N/rad`; local maximum real pole `-3.3523148 -> -3.4242279 1/s`; trim surface `-0.058002947 -> -0.052746258 rad`; normalized elevator `-0.13331877 -> -0.12678484`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-14546.572 -> -14546.572 N/rad`; local maximum real pole `-3.3882935 -> -3.3882935 1/s`; trim surface `-0.058576545 -> -0.052508486 rad`; normalized elevator `-0.13629277 -> -0.12463936`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### aim-9x

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:aim9x; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=83.9578069729kg, speed=251.134066728m/s, Jyy=63.8759011471kgm2, total Fx=7521.79155893N.
Actual pitch D=-9194.1643871N/rad vs aerodynamic-only F_eff=-1696.89566262; yaw is independently signed/recomputed.
Finite TVC selected with actual XML gain/arm/rate/stop metadata.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-8441.9852 -> -9946.3435 N/rad`; local maximum real pole `-3.8166976 -> -3.9180392 1/s`; trim surface `-0.015498285 -> -0.013181916 rad`; normalized elevator `-0.038625135 -> -0.032627249`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-9194.1644 -> -9194.1644 N/rad`; local maximum real pole `-3.867314 -> -3.867314 1/s`; trim surface `-0.0132384 -> -0.015069216 rad`; normalized elevator `-0.031892922 -> -0.038234603`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### asraam

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:asraam; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=87.9043759069kg, speed=252.512862104m/s, Jyy=61.7485954926kgm2, total Fx=17247.2593603N.
Actual pitch D=-19459.9487796N/rad vs aerodynamic-only F_eff=-2238.33270077; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-17735.223 -> -21184.675 N/rad`; local maximum real pole `-5.2318843 -> -5.5673837 1/s`; trim surface `-0.012542742 -> -0.010500892 rad`; normalized elevator `-0.040844365 -> -0.030511186`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-19459.949 -> -19459.949 N/rad`; local maximum real pole `-5.3975766 -> -5.3975766 1/s`; trim surface `-0.0099731435 -> -0.012624396 rad`; normalized elevator `-0.031317071 -> -0.038413056`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### iris-t

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:irist; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=87.3537896574kg, speed=251.195434966m/s, Jyy=62.9001279195kgm2, total Fx=8223.62213108N.
Actual pitch D=-9770.59299614N/rad vs aerodynamic-only F_eff=-1572.4844276; yaw is independently signed/recomputed.
Finite TVC selected with actual XML gain/arm/rate/stop metadata.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-8948.2308 -> -10592.955 N/rad`; local maximum real pole `-3.6631675 -> -3.7622229 1/s`; trim surface `-0.017115294 -> -0.014465617 rad`; normalized elevator `-0.038486568 -> -0.03272113`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-9770.593 -> -9770.593 N/rad`; local maximum real pole `-3.7126649 -> -3.7126649 1/s`; trim surface `-0.014791157 -> -0.016405322 rad`; normalized elevator `-0.031756145 -> -0.038310824`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### magic-2

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:magic2; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=89.9040539318kg, speed=252.463909977m/s, Jyy=56.7811722371kgm2, total Fx=17275.3634014N.
Actual pitch D=-25175.9869712N/rad vs aerodynamic-only F_eff=-7926.69734728; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-23448.451 -> -26903.523 N/rad`; local maximum real pole `-3.997432 -> -4.1666014 1/s`; trim surface `-0.032210475 -> -0.028593316 rad`; normalized elevator `-0.092993258 -> -0.087712194`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-25175.987 -> -25175.987 N/rad`; local maximum real pole `-4.0820986 -> -4.0820986 1/s`; trim surface `-0.031473667 -> -0.029299352 rad`; normalized elevator `-0.093978799 -> -0.087056505`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### meteor

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:meteor; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=189.904880471kg, speed=251.126890243m/s, Jyy=217.081636854kgm2, total Fx=16988.2815721N.
Actual pitch D=-19574.6727274N/rad vs aerodynamic-only F_eff=-2665.20654682; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-17875.845 -> -21273.501 N/rad`; local maximum real pole `-5.5054266 -> -5.7666727 1/s`; trim surface `-0.0015214579 -> -0.001235954 rad`; normalized elevator `-0.72688946 -> -0.58510438`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-19574.673 -> -19574.673 N/rad`; local maximum real pole `-5.6329011 -> -5.6329011 1/s`; trim surface `0.0087596847 -> -0.0096512273 rad`; normalized elevator `-0.69260553 -> -0.61485796`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### mica-em

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:micaem; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=111.942331669kg, speed=251.150192832m/s, Jyy=89.8424323589kgm2, total Fx=10205.0877886N.
Actual pitch D=-12639.0127477N/rad vs aerodynamic-only F_eff=-2480.38847244; yaw is independently signed/recomputed.
Finite TVC selected with actual XML gain/arm/rate/stop metadata.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-11618.504 -> -13659.522 N/rad`; local maximum real pole `-4.3067003 -> -4.435146 1/s`; trim surface `-0.014823419 -> -0.01262917 rad`; normalized elevator `-0.056757029 -> -0.047288596`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-12639.013 -> -12639.013 N/rad`; local maximum real pole `-4.3707128 -> -4.3707128 1/s`; trim surface `-0.011482998 -> -0.015400655 rad`; normalized elevator `-0.046864588 -> -0.055548172`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### mica-ir

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:micair; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=111.942331669kg, speed=251.14961415m/s, Jyy=89.8424323589kgm2, total Fx=10204.2103361N.
Actual pitch D=-12810.1039026N/rad vs aerodynamic-only F_eff=-2638.73828981; yaw is independently signed/recomputed.
Finite TVC selected with actual XML gain/arm/rate/stop metadata.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-11789.683 -> -13830.525 N/rad`; local maximum real pole `-4.4365981 -> -4.5730923 1/s`; trim surface `-0.0087013537 -> -0.007429448 rad`; normalized elevator `-0.031713749 -> -0.025465492`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-12810.104 -> -12810.104 N/rad`; local maximum real pole `-4.5045639 -> -4.5045639 1/s`; trim surface `-0.0059636713 -> -0.0096928915 rad`; normalized elevator `-0.023062676 -> -0.032659193`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### pl-10

Selected fsim.control.actuator_aware_cascade.v1; actual native model jsbsim:pl10; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=88.9539161081kg, speed=251.140661948m/s, Jyy=66.8621994681kgm2, total Fx=8105.00130501N.
Actual pitch D=-10892.484458N/rad vs aerodynamic-only F_eff=-2813.57839203; yaw is independently signed/recomputed.
Finite TVC selected with actual XML gain/arm/rate/stop metadata.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-10081.984 -> -11702.985 N/rad`; local maximum real pole `-2.5 -> -2.5 1/s`; trim surface `-0.0019600057 -> -0.001649797 rad`; normalized elevator `0.01067285 -> 0.015638377`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-10892.484 -> -10892.484 N/rad`; local maximum real pole `-2.5 -> -2.5 1/s`; trim surface `0.00051883875 -> -0.0036851593 rad`; normalized elevator `0.022052917 -> 0.0062699768`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### pl-12

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:pl12; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=179.904621382kg, speed=251.19213483m/s, Jyy=232.038539017kgm2, total Fx=17009.4269745N.
Actual pitch D=-24675.942477N/rad vs aerodynamic-only F_eff=-7741.29358416; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-22975 -> -26376.885 N/rad`; local maximum real pole `-5.2329137 -> -5.4371728 1/s`; trim surface `-0.01738301 -> -0.015477155 rad`; normalized elevator `-0.13442939 -> -0.1141217`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-24675.942 -> -24675.942 N/rad`; local maximum real pole `-5.333763 -> -5.333763 1/s`; trim surface `-0.013744388 -> -0.018508039 rad`; normalized elevator `-0.12459153 -> -0.12274443`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### pl-15

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:pl15; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=209.905031232kg, speed=251.024669067m/s, Jyy=280.50450003kgm2, total Fx=17036.0010367N.
Actual pitch D=-19918.7117153N/rad vs aerodynamic-only F_eff=-2970.62125221; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-18215.112 -> -21622.312 N/rad`; local maximum real pole `-5.588478 -> -5.8507982 1/s`; trim surface `0.011595752 -> 0.0097766223 rad`; normalized elevator `-0.78220031 -> -0.63252374`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-19918.712 -> -19918.712 N/rad`; local maximum real pole `-5.7159875 -> -5.7159875 1/s`; trim surface `0.022073646 -> 0.0012277332 rad`; normalized elevator `-0.7493771 -> -0.66134835`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### pl-8

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:pl8; 8 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=114.923141934kg, speed=251.514132816m/s, Jyy=83.5256758873kgm2, total Fx=13706.0125028N.
Actual pitch D=-26146.8424268N/rad vs aerodynamic-only F_eff=-12474.1398156; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-24776.241 -> -27517.444 N/rad`; local maximum real pole `-3.1888022 -> -3.2720475 1/s`; trim surface `-0.088035757 -> -0.079670252 rad`; normalized elevator `-0.22195706 -> -0.21418056`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-26146.842 -> -26146.842 N/rad`; local maximum real pole `-3.2304823 -> -3.2304823 1/s`; trim surface `-0.08855646 -> -0.079606073 rad`; normalized elevator `-0.2295533 -> -0.20830183`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### r-27er

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:r27er; 8 controlled physical-fin rate entries.
Captured PRE t=11.858333333333302s: mass=349.871980076kg, speed=250.834864724m/s, Jyy=667.532221437kgm2, total Fx=23687.7159396N.
Actual pitch D=-33079.6136778N/rad vs aerodynamic-only F_eff=-9392.05286101; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-30710.842 -> -35448.385 N/rad`; local maximum real pole `-5.7831572 -> -5.7928467 1/s`; trim surface `-0.0061059256 -> -0.0060199671 rad`; normalized elevator `-0.22603303 -> -0.22654234`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-33079.614 -> -33079.614 N/rad`; local maximum real pole `-5.787979 -> -5.787979 1/s`; trim surface `-0.0026700318 -> -0.0088333716 rad`; normalized elevator `-0.23981093 -> -0.21525636`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### r-27et

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:r27et; 8 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=347.872000598kg, speed=250.850497199m/s, Jyy=585.641436673kgm2, total Fx=23659.3579913N.
Actual pitch D=-32600.5379385N/rad vs aerodynamic-only F_eff=-9046.40707137; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-30234.602 -> -34966.474 N/rad`; local maximum real pole `-5.447246 -> -5.4598861 1/s`; trim surface `-0.03562973 -> -0.028593022 rad`; normalized elevator `0.13148467 -> 0.10197121`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-32600.538 -> -32600.538 N/rad`; local maximum real pole `-5.4535548 -> -5.4535548 1/s`; trim surface `-0.040807644 -> -0.024531995 rad`; normalized elevator `0.11300339 -> 0.11782811`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### r-27r

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:r27r; 8 controlled physical-fin rate entries.
Captured PRE t=11.858333333333302s: mass=252.934035122kg, speed=250.562420674m/s, Jyy=351.609695342kgm2, total Fx=11921.1488427N.
Actual pitch D=-21378.3967166N/rad vs aerodynamic-only F_eff=-9457.3419105; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-20186.282 -> -22570.512 N/rad`; local maximum real pole `-5.9611943 -> -5.9666715 1/s`; trim surface `-0.0027849494 -> -0.0028028749 rad`; normalized elevator `-0.15538543 -> -0.15525597`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-21378.397 -> -21378.397 N/rad`; local maximum real pole `-5.9639167 -> -5.9639167 1/s`; trim surface `-0.00035580757 -> -0.0047896337 rad`; normalized elevator `-0.16469897 -> -0.14764101`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### r-27t

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:r27t; 8 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=252.934035122kg, speed=250.573951974m/s, Jyy=304.208630953kgm2, total Fx=11900.5510898N.
Actual pitch D=-20971.5468413N/rad vs aerodynamic-only F_eff=-9146.45278165; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-19781.492 -> -22161.602 N/rad`; local maximum real pole `-5.4488271 -> -5.4576926 1/s`; trim surface `-0.029238004 -> -0.024751185 rad`; normalized elevator `0.1179385 -> 0.099036194`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-20971.547 -> -20971.547 N/rad`; local maximum real pole `-5.4532545 -> -5.4532545 1/s`; trim surface `-0.033835363 -> -0.021166139 rad`; normalized elevator `0.10853566 -> 0.1074727`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### r-37m

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:r37m; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=509.84844539kg, speed=250.633081049m/s, Jyy=701.868196095kgm2, total Fx=26382.0749539N.
Actual pitch D=-36358.5695078N/rad vs aerodynamic-only F_eff=-10197.6547239; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-33720.362 -> -38996.777 N/rad`; local maximum real pole `-6.7732551 -> -7.3129608 1/s`; trim surface `0.15015835 -> 0.12982756 rad`; normalized elevator `-1 -> -1`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-36358.57 -> -36358.57 N/rad`; local maximum real pole `-7.3477371 -> -7.3477371 1/s`; trim surface `0.16454149 -> 0.11856668 rad`; normalized elevator `-1 -> -1`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### r-60m

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:r60m; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=44.9692717204kg, speed=251.525250279m/s, Jyy=17.0711785975kgm2, total Fx=5441.48434841N.
Actual pitch D=-7784.94311874N/rad vs aerodynamic-only F_eff=-2356.49505581; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-7240.7947 -> -8329.0916 N/rad`; local maximum real pole `-4.7973603 -> -4.9851396 1/s`; trim surface `-0.032994959 -> -0.030219165 rad`; normalized elevator `-0.10640524 -> -0.095383114`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-7784.9431 -> -7784.9431 N/rad`; local maximum real pole `-4.8906766 -> -4.8906766 1/s`; trim surface `-0.031838674 -> -0.031241179 rad`; normalized elevator `-0.1025881 -> -0.098807855`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### r-73

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:r73; 8 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=104.948546914kg, speed=251.134019934m/s, Jyy=75.2411815962kgm2, total Fx=9488.39230083N.
Actual pitch D=-13743.7663777N/rad vs aerodynamic-only F_eff=-4286.19397845; yaw is independently signed/recomputed.
Finite TVC selected with actual XML gain/arm/rate/stop metadata.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-12794.927 -> -14692.606 N/rad`; local maximum real pole `-6.6590817 -> -6.6570133 1/s`; trim surface `-0.0012542699 -> -0.0014839349 rad`; normalized elevator `-0.023030967 -> -0.018350544`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-13743.766 -> -13743.766 N/rad`; local maximum real pole `-6.657838 -> -6.657838 1/s`; trim surface `0.0012355945 -> -0.0035146322 rad`; normalized elevator `-0.014887273 -> -0.025145308`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### r-77m

Selected fsim.control.operating_point_bounded.v1; actual native model jsbsim:r77m; 4 controlled physical-fin rate entries.
Captured PRE t=11.391666666666637s: mass=189.904322695kg, speed=251.137443548m/s, Jyy=218.324433818kgm2, total Fx=17053.8150864N.
Actual pitch D=-20828.8948193N/rad vs aerodynamic-only F_eff=-3854.04316645; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-19123.513 -> -22534.276 N/rad`; local maximum real pole `-4.3381103 -> -4.4672211 1/s`; trim surface `-0.05813398 -> -0.04925448 rad`; normalized elevator `-0.26100336 -> -0.21760526`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Directional cascade SEED only; the captured R77M finite search is not re-optimized by this perturbation.
- control_force_and_moment_effectiveness +/-10%: pitch D `-20828.895 -> -20828.895 N/rad`; local maximum real pole `-4.402445 -> -4.402445 1/s`; trim surface `-0.048812415 -> -0.057027519 rad`; normalized elevator `-0.23399996 -> -0.24041401`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Directional cascade SEED only; the captured R77M finite search is not re-optimized by this perturbation.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

### super-530d

Selected fsim.control.load_feedback_cascade; actual native model jsbsim:super530d; 4 controlled physical-fin rate entries.
Captured PRE t=11.858333333333302s: mass=269.854307282kg, speed=251.256477328m/s, Jyy=325.346303701kgm2, total Fx=27200.3228287N.
Actual pitch D=-34823.9831593N/rad vs aerodynamic-only F_eff=-7623.56769029; yaw is independently signed/recomputed.
No TVC selected; axial thrust/drag still enters the directional determinant and dynamics.

The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.

Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):

- measured_total_axial_force +/-10%: pitch D `-32103.951 -> -37544.015 N/rad`; local maximum real pole `-6.4840137 -> -7.1770359 1/s`; trim surface `-0.005346184 -> -0.005331899 rad`; normalized elevator `0.22073598 -> 0.22026819`. Other captured inputs are frozen.
  Only captured total Fx is perturbed; no prescribed nonzero output is assumed. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.
- control_force_and_moment_effectiveness +/-10%: pitch D `-34823.983 -> -34823.983 N/rad`; local maximum real pole `-6.7821521 -> -6.7821521 1/s`; trim surface `-0.0101765 -> -0.0013801067 rad`; normalized elevator `0.23282141 -> 0.21038942`. Other captured inputs are frozen.
  Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output. Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory.

Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.

## Primary references

- [Jackson, Overview of Missile Flight Control Systems, JHU/APL29(1),2010,pp9-24](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Jackson.pdf): Physical feedback/actuator/airframe/TVC separation; the displayed directional algebra is the project's disclosed reduction, not an OEM parameter claim.
- [SciPy place_poles primary documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.place_poles.html): Independent executed pole-assignment comparison for actuator-aware current A-BK, not cascade stability admission.
- [SciPy cont2discrete primary documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.cont2discrete.html): Independent bilinear-transform check of the captured finite operating-point virtual lag.

## Reproduction

These are captured-data integration math tests. First produce the complete actual24 population through the normal UCRT64 Release full-chain native gate. A clean checkout with no capture is expected to fail explicitly, not skip or use synthetic aliases. The tests use `FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT`; if absent, their only fallback is the normal preset output `build/ucrt64-release/validation/aam_full_chain`.

```powershell
$env:FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT = Join-Path (Get-Location) 'build/ucrt64-release/validation/aam_full_chain'
python -B tools/validation/weapon_quality/test_directional_control_theory.py -v
python -B tools/validation/weapon_quality/directional_control_theory.py --capture-root $env:FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT --capture-label 'Explicit fresh native PRE source epoch; acceptance remains separately owned' --output build/validation/directional-control-theory.json
# Add --write only in the authorized documentation window; it replaces only the 24 autopilot object spans.
```

To reproduce this document's exact supplied capture epoch, explicitly select it (an ignored capture directory is not a clean-checkout prerequisite). The capture label above determines whether these are historical illustrations:

```powershell
$env:FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT = Join-Path (Get-Location) 'build/architecture-reviews/sim-m03-current-20261104/baseline24'
python -B tools/validation/weapon_quality/test_directional_control_theory.py -v
python -B tools/validation/weapon_quality/directional_control_theory.py --capture-root $env:FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT --capture-label 'Frozen baseline24 initial native PRE illustrations; captured before later RK4/reticle repairs; not current native acceptance' --output build/validation/directional-control-theory-captured-epoch.json
```

The generator always requires an explicit capture root and never assigns native acceptance. Full24 encounters, the other component families and overall SIM-M03 acceptance remain separate gates.
