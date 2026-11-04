# Strapdown navigation method and source map

This note records the additive body-IMU implementation, not acceptance of a
particular installed build or fitted missile IMU. The implementation remains
SDK-owned; native motion is sensor physics, and later state corrections
require actual fitted GNSS exposures.

## Primary references checked

- [Oliver Woodman, An introduction to inertial navigation, Cambridge technical
  report 696 (2007)](https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf):
  body-fixed inertial sensors and attitude-dependent integration are the
  strapdown basis. Its discussion of gyro error driving translational drift
  motivates explicit attitude/bias coupling; its numerical hardware results
  are not imported as this simulator's performance.
- [Joan Sola, Quaternion kinematics for the error-state Kalman filter
  (2017)](https://arxiv.org/abs/1711.02508): rotation-vector errors, quaternion
  composition and covariance injection/reset provide the error-state method
  context. This implementation states its own Hamilton active and left-ECEF
  conventions and independently differentiates their reset Jacobian.
- [PX4 official EKF2 documentation](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf):
  body-fixed angular/velocity increments and attitude/gyro/accelerometer-bias
  estimation provide an open-source system comparison. PX4 is not linked,
  its implementation is not claimed as this implementation, and its aircraft
  tuning does not establish weapon calibration.
- The pinned project upstream files
  `third_party/jsbsim/src/models/FGPropagate.h` and `FGInertial.cpp` establish
  the native ECEF-relative `GetPQR()` convention, Earth rotation and central/J2
  attraction conventions. `GetPQRi()` remains inertial-relative. The integrator
  explicitly adds/removes Earth rate at the body sensor/ECEF mechanization
  boundary rather than treating those rate APIs as interchangeable.

## Source execution map

- `NavigationStrapdownV2::imu`: captured native orientation/rates and
  acceleration become biased/noisy body gyro and specific-force measurements.
- `integrate`: two-sided quaternion exponential and RK4 ECEF
  position/velocity with gravity, Coriolis and centrifugal terms.
- `gravityGradient` and `dynamics`: analytic J2 gradient and coupled
  position/velocity/left-attitude/body-bias/held-noise variational equations.
- `gnss`: independent noisy ownship measurements, six-observable/all-state
  Kalman gain, Joseph covariance, state/attitude injection and SO(3) reset.
- `advanceImpl`: fixed integer-index sensor clocks, physical capture
  interpolation, outage/reacquisition and sample noise replacement.
- `estimate`: unchanged V1 position/velocity/interleaved covariance output;
  full attitude/error diagnostics remain additive private interfaces.

The [method contract](../../sdk/weapon-navigation-strapdown-v2.md) records
equations, noise discretization, numerical steps, engineering defaults and
screening intervals. No target state is accepted. No recurring native
position/orientation correction is hidden in interpolation.

## Independent numerical reference

`navigation_strapdown_reference.py` uses SciPy rotations, central differences
and matrix exponentials rather than the production quaternion helpers, J2
Jacobian or RK4 transition. It checks the signs/cross-couplings of the 21-state
error dynamics and the left-attitude reset derivative, and supplies a frozen
fractional-exposure GNSS covariance used by a native behavioral test. Its
8192-realization Gaussian error ensemble checks expected covariance/NEES for
the declared sampled linearized process. It is a method oracle, not a native
execution or a hardware dataset.

Root executes the normal native target and records actual assertions,
subsequent World integration, installed consumer behavior and flight
convergence. Source presence, the reference script and a calibration-limit
description cannot substitute for those gates. The retained V1 component
continues its previous resolved-ECEF behavior for existing consumers.
