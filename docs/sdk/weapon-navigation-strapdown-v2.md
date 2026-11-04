# Weapon body-IMU strapdown navigation V2

`NavigationStrapdownV2` is an additive private navigation implementation. It
integrates measured body gyro and specific force into its own attitude,
velocity and position. It does not reset those estimates from later native
motion. `NavigationV1` and all public layouts remain unchanged. The V2
`estimate()` result uses the existing interleaved ECEF covariance layout
`[x,vx,y,vy,z,vz]`; the private attitude and full covariance diagnostics are
separate.

## Captured native seam and authority

`NavigationStrapdownInputV2` carries chronological captured ownship position,
velocity and total kinematic acceleration in ECEF, body-to-ECEF rotation
matrix **columns**, and body angular velocity relative ECEF resolved in body
axes. This is the convention of JSBSim `FGPropagate::GetPQR()`, as documented
in the maintained upstream `third_party/jsbsim/src/models/FGPropagate.h`.
`GetPQRi()` is inertial-relative and must not be passed to that field.

At release initialization only, the true native position, velocity and
orientation initialize noisy estimates. Later captured orientation, rates,
position, velocity and acceleration enter the sensor simulator. Native
position and velocity additionally enter independent noisy GNSS measurements
only when this exact weapon fit explicitly enables aiding and an exposure
is available. No target state enters this component. Native simulation
remains the flight authority; this estimator never changes physical motion.

For Earth rate vector `Omega=[0,0,earthRateRadS]`, true body-to-ECEF matrix
`C`, native ECEF velocity `v`, position `r` and acceleration `a`, sensor
physics produces

```
omega_measured = omega_body_relative_ECEF + C' Omega + b_g + n_g
f_measured = C' [a - gravitation(r) + 2 Omega x v
                + Omega x (Omega x r)] + b_a + n_a
```

Thus the gyro measures inertial angular rate, and the accelerometer measures
proper specific force rather than kinematic acceleration or a pre-resolved
ECEF acceleration. Body mounting axes are retained for biases and noise.
Each sensor has independent stable initial-bias, random-walk and white-error
streams. Sensor cadence is independent of callers, workers and report reads.

Native captures bound the physical scene reconstruction: position, velocity,
acceleration and rates interpolate linearly; orientation uses shortest-arc
quaternion SLERP. No higher-frequency native truth is synthesized. Availability
must hold at both endpoints before an intervening GNSS exposure can occur.

## Nominal strapdown equations

The Hamilton active quaternion `q=[w,x,y,z]` maps body vectors to ECEF.
With measured inertial gyro corrected by estimated gyro bias and the current
held white-error estimate, attitude advances as

```
q(t+h) = ExpQuaternion(-Omega*h) * q(t)
         * ExpQuaternion((omega_measured-b_g_est-w_g_est)*h)
```

Left multiplication removes Earth rotation; right multiplication integrates
body rotation. Quaternion normalization preserves the rotation representation,
not a truth attitude correction. Successive rotations retain their order.
The current finite gyro sample is held until the next IMU exposure.

ECEF translation integrates

```
r_dot = v
v_dot = C(q)*(f_measured-b_a_est-w_a_est) + gravitation(r)
        - 2 Omega x v - Omega x (Omega x r)
```

The model evaluates central plus J2 gravitational attraction, separately from
centrifugal acceleration. Defaults are `mu=3.986004418e14 m^3/s^2`, equatorial
radius `6378137 m`, `J2=1.08262982e-3` and
`earthRateRadS=7.292115e-5`. The J2 and Earth-rate conventions follow the pinned
JSBSim `FGInertial` implementation; the SI gravitational constant uses the
nominal WGS84 value rather than a rounded imperial conversion.

Position/velocity use RK4 with stage-specific quaternion, gravity, Coriolis
and centrifugal acceleration. Attitude is the exact two-sided exponential
for the held body gyro at each stage. The maximum integration substep is
explicit and independent of the sensor cadence. This is finite sampled
mechanization: changing rotation or specific force between unresolved captures
is not observable. Body noncommutativity executes through successive sampled
rotations; no claim of reconstructing unobserved sub-exposure coning or
sculling is made.

## Coupled error-state filter

The 15 physical navigation errors are ECEF position/velocity, **left ECEF
attitude error**, body accelerometer bias and body gyro bias. Six additional
held-white-error states retain each actual finite IMU sample's correlation:

```
delta = [dr(3), dv(3), dtheta_ECEF(3), db_a_body(3), db_g_body(3),
         dw_a_body(3), dw_g_body(3)]
C_true = ExpRotation(dtheta_ECEF) * C_est
```

For `S(x)y=x cross y`, corrected ECEF specific force `u=C*f_corrected`,
and analytic J2 gravitational Jacobian `G`, the nonzero error dynamics are

```
dr_dot     = dv
dv_dot     = (G-S(Omega)^2)*dr - 2*S(Omega)*dv
             - S(u)*dtheta - C*db_a - C*dw_a
dtheta_dot = -S(Omega)*dtheta - C*db_g - C*dw_g
```

Bias and white-error states are held between exposures. The transition matrix
integrates the coupled variational equation with the same RK4 stages as the
nominal model, then `P=Phi*P*Phi'`. This preserves position/velocity/attitude
and sensor-bias cross covariance instead of treating ECEF axes as independent
navigation filters. Symmetric roundoff averaging does not clip covariance
eigenvalues or substitute a diagonal covariance.

For physical IMU period `H`, each white measurement has independent variance
`noiseDensity^2/H`. At a new exposure its estimated white error becomes zero,
its covariance row/column is replaced by that independent variance, and its
old integrated correlations remain in the other 15 states. Bias random walk
receives one increment of variance `biasWalkDensity^2*H` at the exposure.
There is no second continuous process-noise term between those exposures.
This exact sampled-noise convention prevents artificial extra noise when a
native interval is subdivided or GNSS arrives inside an IMU exposure.

GNSS independently measures ECEF position and velocity with declared Gaussian
noise. The six-observable update computes gains for **all 21 states**, uses
Joseph covariance propagation, injects position/velocity/bias/held-error
corrections, and left-injects the attitude correction. Its covariance reset
uses the SO(3) left Jacobian of that correction, whose sign is independently
checked by differentiating rotation composition. GNSS outages retain the
last measurement epoch and unaided propagation; availability alone cannot
renew aiding. A later scheduled available exposure performs reacquisition.

## Explicit engineering defaults and uncertainty

These are uncalibrated model inputs, not published missile IMU specifications.
Intervals below are engineering screening ranges with no established
probabilistic coverage. Exact-fit GNSS applicability remains a separate
World integration decision; neither an active radar nor a datalink grants it.

| Quantity | Default | Screening interval | Unit |
|---|---:|---:|---|
| IMU period | 0.01 | 0.005 to 0.02 | s |
| GNSS period | 1 | 0.5 to 2 | s |
| Maximum integration substep | 0.005 | numerical convergence must be checked | s |
| Accelerometer white density | 0.03 | 0.015 to 0.06 | m/s^2 sqrt(s) |
| Gyro white density | 0.0001 | 0.00005 to 0.0002 | rad/s sqrt(s) |
| Initial accelerometer bias sigma | 0.01 | 0.005 to 0.02 | m/s^2 |
| Initial gyro bias sigma | 0.0001 | 0.00005 to 0.0002 | rad/s |
| Accelerometer bias-walk density | 0.0001 | 0.00005 to 0.0002 | m/s^2 /sqrt(s) |
| Gyro bias-walk density | 0.000001 | 0.0000005 to 0.000002 | rad/s /sqrt(s) |
| Release position sigma | 5 | 2.5 to 10 | m |
| Release velocity sigma | 0.5 | 0.25 to 1 | m/s |
| Release attitude sigma, each rotation-vector component | 0.001 | 0.0005 to 0.002 | rad |
| GNSS position sigma | 3 | 1.5 to 6 | m |
| GNSS velocity sigma | 0.1 | 0.05 to 0.2 | m/s |

Noise densities are the square roots of continuous covariance intensities;
their finite sample standard deviations are density divided by `sqrt(H)`.
The densities, bias strengths, initialization and aiding errors have nonzero
defaults. Zero values are permitted for isolated physical reference tests.
Bias walk is explicitly discrete at exposures rather than an independently
superimposed continuous process. Sensor scale factor, thermal behavior and
installation misalignment beyond initial attitude uncertainty are not fitted
equipment models; gravity and sensor-error parameter uncertainty must not be
mistaken for a validated hardware error budget.

## Lifetime, diagnostics and verification

Initialization occurs once. Reset closes the old estimate, clears exposure
history and restores the original independent streams. Repeated epochs draw
nothing. Invalid, reversed, overflowing or unrepresentable captures reject
transactionally, including random-stream state. Rotation input must be a
finite proper orthonormal frame. Finite operation and covariance symmetry
are checked before committing an advance. No true-position, velocity or
orientation fallback is permitted when estimation fails.

Focused native cases cover stationary/tilted Earth, rotating body force,
noncommuting rotations, independent sampled covariance, mid-exposure GNSS,
information-form GNSS posterior, attitude/velocity/bias coupling, actual
gyro-induced drift, observable attitude correction, outage/reacquisition,
stream/reset lifetime and malformed capture rejection. Their actual native
execution is a root-owned gate, not inferred from source presence.

The independent NumPy/SciPy reference is
[`navigation_strapdown_reference.py`](../research/sim-m03/navigation_strapdown_reference.py).
It differentiates rotation/force equations, checks the attitude reset
Jacobian, uses a matrix exponential for the fractional-exposure covariance
oracle, and samples a coupled 21-state error ensemble. It imports no simulator
and does not establish native integration or equipment calibration.

Method sources and the evidence distinction are recorded in the persistent
[research note](../research/sim-m03/strapdown-navigation-method.md).
