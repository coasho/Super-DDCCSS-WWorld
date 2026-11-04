# Weapon ownship navigation engineering model

`NavigationV1` models the weapon's own resolved inertial estimate and optional
explicitly fitted GNSS aiding. It supplies onboard return reports; it never
reads a target, modifies flight state, or substitutes for JSBSim integration.
The link's actual delivery time remains separate from the navigation epoch.

## Physical and numerical contract

The native runtime supplies chronological captured ownship states and ECEF
total kinematic acceleration. This acceleration has already been resolved from
the native body's motion into the Earth frame. The model adds sampled
accelerometer white error, a fixed initial acceleration bias and bias random
walk, then integrates its own position and velocity. True own position/velocity
are used only at release initialization and a fitted, available GNSS exposure.
An unaided weapon cannot reset its estimate from a later native position.

Each axis estimates `[position, velocity, acceleration bias, held white error]`.
The last state represents the error in the current finite IMU exposure; it is
replaced at the next exposure. Propagation uses
`p += v*h + (a_measured-b-w)*h^2/2`, `v += (a_measured-b-w)*h`.
Both estimated error terms are held between exposures. The exact transition is

```
F = [[1, h, -h^2/2, -h^2/2], [0, 1, -h, -h],
     [0, 0, 1, 0], [0, 0, 0, 1]]
```

Between exposures `P = F P F'` without adding an independent continuous-noise
increment. At each IMU exposure of period `H`, actual bias receives a draw with
variance `qb*H`; its estimated covariance receives that same increment. The
new acceleration white error has variance `qa/H`. Its estimate resets to zero
and its covariance row/column are replaced by an independent variance `qa/H`.
Previously accumulated position/velocity/bias correlations remain intact.

```
P(t+H) = R F(H) P(t) F(H)' R' + diag(0,0,qb*H,qa/H)
R = diag(1,1,1,0)
```

GNSS measures own position and velocity with independent declared errors. The
native linear Kalman update estimates both correlated acceleration errors and
uses Joseph covariance propagation across all four states. This retains the
current held-error correlation when GNSS arrives between IMU exposures; splitting
a decision interval cannot add independent error draws. Published six-state covariance follows the
accepted interleaved ECEF ordering. Cross-axis covariance is zero under the
declared independent resolved-error model, not unavailable or secretly inferred
from target truth. Unavailable aiding retains its original last measurement
epoch; reacquisition requires a new actual exposure.

The initial, accelerometer, bias and GNSS random streams are independently
derived from the release seed and stable component identifiers. IMU/GNSS clocks
use integer exposure indices relative to release. Repeated epoch reads draw
nothing. Invalid/rewound/overflowing input is rejected transactionally, including
its RNG state. A full reset restores the same seed and clears every history.

The native capture interval bounds interpolation of ownship scene values at an
exposure. The model uses linear scene interpolation and zero-order IMU holds;
it does not invent a higher-rate native trajectory. This is a resolved inertial
error model, not a full strapdown gyro/attitude/gravity-estimation system or a
fitted equipment error budget. These are explicit model-order limitations;
finite unaided drift, GNSS measurement/aiding, outages and covariance execute.

## Explicit policy and uncertainty

| Input | Engineering value | Screening interval | Unit |
| --- | --- | --- | --- |
| IMU exposure period | 0.01 | 0.005 to 0.02 | s |
| GNSS exposure period | 1 | 0.5 to 2 | s |
| Acceleration noise density | 0.03 | 0.015 to 0.06 | m/s^2 sqrt(s) |
| Initial acceleration bias sigma | 0.01 | 0.005 to 0.02 | m/s^2 |
| Bias random-walk density | 0.0001 | 0.00005 to 0.0002 | m/s^2/sqrt(s) |
| Release position sigma | 5 | 2.5 to 10 | m |
| Release velocity sigma | 0.5 | 0.25 to 1 | m/s |
| GNSS position sigma | 3 | 1.5 to 6 | m |
| GNSS velocity sigma | 0.1 | 0.05 to 0.2 | m/s |

These are assumed simulation policies with nonzero uncertainty. They are not
published AIM-120D, Meteor, IMU or GNSS performance. The fitted aiding boolean
is admitted separately by exact equipment evidence; a datalink or active seeker
does not imply GNSS hardware. Zero-error synthetic configurations are permitted
only as independent mathematical controls, not as current equipment accuracy.

## Method provenance and acceptance

Woodman, [An introduction to inertial navigation](https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf),
Cambridge technical report 696 (2007), sections 2, 4, 6 and 8, explains inertial
integration, acceleration errors, drift and external aiding. Its MEMS hardware
measurements are not reused as missile coefficients. The implementation reuses
the SDK's fixed-size native matrix algebra and deterministic stream generator;
it introduces no runtime Python dependency or alternative flight integrator.

Independent fixtures compare noiseless constant acceleration, exact sampled
impulse-weight sums, a SciPy matrix exponential of the augmented hold system,
and an information-form GNSS posterior against native propagation. A separate
Van Loan continuum limit is retained as a convergence reference, rather than
being mislabeled as exact covariance for a finite held measurement. Fractional
exposure and mid-exposure aiding cases compare the analytical posterior and
4,096 independent realized error histories against reported covariance. A
frozen independent-seed population compares aided and unaided drift; outage,
reacquisition, equal-epoch reads, malformed policy, overflow and reset cases
exercise native lifetime/error behavior. Current passing results belong in the
dated complete M03 receipt after actual execution.
