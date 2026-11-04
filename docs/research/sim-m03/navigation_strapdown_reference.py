"""Independent strapdown method oracles; no simulator imports or execution.

Run with D:/ENV Python and NumPy/SciPy. The closed sampled-noise oracle uses
SciPy matrix exponentials and scalar information updates, not production RK4.
The error-dynamics oracle differentiates independent rotation/force equations.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def gravity(position):
    x, y, z = position
    r = np.linalg.norm(position)
    mu, radius, j2 = 3.986004418e14, 6378137., 1.08262982e-3
    k = 1.5 * j2 * (radius / r) ** 2
    return -mu / r ** 3 * np.array([x * (1 + k * (1 - 5 * z*z/r**2)),
                                    y * (1 + k * (1 - 5 * z*z/r**2)),
                                    z * (1 + k * (3 - 5 * z*z/r**2))])


def sampled_noise_gnss():
    # p,v,held acceleration noise; 10 ms independent exposures, GNSS at 15 ms.
    f = np.array([[0., 1., 0.], [0., 0., -1.], [0., 0., 0.]])
    p = np.diag([0., 0., .04/.01])
    for start, end in [(0., .01), (.01, .015), (.015, .02), (.02, .025)]:
        phi = expm(f * (end-start))
        p = phi @ p @ phi.T
        if end == .015:
            h = np.array([[1., 0., 0.], [0., 1., 0.]])
            r = np.diag([.003**2, .001**2])
            k = np.linalg.solve(h @ p @ h.T + r, h @ p).T
            a = np.eye(3) - k @ h
            p = a @ p @ a.T + k @ r @ k.T
        if end in (.01, .02):
            p[2, :] = p[:, 2] = 0.
            p[2, 2] = .04/.01
    return p


def linearization_check():
    position = np.array([4510000., 1230000., 4360000.])
    velocity = np.array([300., -90., 140.])
    c = Rotation.from_rotvec([.2, -.4, .7]).as_matrix()
    omega = np.array([0., 0., 7.292115e-5])
    measured_rate, specific = np.array([.13, -.21, .17]), np.array([9., -3., 5.])
    o = skew(omega)

    def acceleration(r, v, rotation, force):
        return rotation @ force + gravity(r) - 2*o @ v - o @ o @ r

    nominal_acceleration = acceleration(position, velocity, c, specific)
    nominal_cdot = -o @ c + c @ skew(measured_rate)

    def derivative(error):
        dp, dv, theta, ba, bg, wa, wg = error.reshape(7, 3)
        ct = Rotation.from_rotvec(theta).as_matrix() @ c
        cdot = -o @ ct + ct @ skew(measured_rate-bg-wg)
        relative = ct @ c.T
        relative_dot = cdot @ c.T + ct @ nominal_cdot.T
        spin = relative_dot @ relative.T
        angular = np.array([spin[2, 1], spin[0, 2], spin[1, 0]])
        theta_norm = np.linalg.norm(theta)
        s = skew(theta)
        if theta_norm < 1e-4:
            j_inverse = np.eye(3) - .5*s + s@s/12
        else:
            j_inverse = np.eye(3) - .5*s + (1/theta_norm**2-(1+math.cos(theta_norm))/(2*theta_norm*math.sin(theta_norm)))*s@s
        result = np.zeros(21)
        result[:3] = dv
        result[3:6] = acceleration(position+dp, velocity+dv, ct, specific-ba-wa)-nominal_acceleration
        result[6:9] = j_inverse @ angular
        return result

    numeric = np.zeros((21, 21))
    for column in range(21):
        step = .5 if column < 3 else 1e-5
        delta = np.zeros(21)
        delta[column] = step
        numeric[:, column] = (derivative(delta)-derivative(-delta))/(2*step)
    analytic = np.zeros((21, 21))
    analytic[:3, 3:6] = np.eye(3)
    # Independent numerical gravitational Jacobian, separate from production's analytic J2 derivative.
    gradient = np.column_stack([(gravity(position+np.eye(3)[j])-gravity(position-np.eye(3)[j]))/2 for j in range(3)])
    analytic[3:6, :3] = gradient-o@o
    analytic[3:6, 3:6] = -2*o
    analytic[3:6, 6:9] = -skew(c@specific)
    analytic[3:6, 9:12] = analytic[3:6, 15:18] = -c
    analytic[6:9, 6:9] = -o
    analytic[6:9, 12:15] = analytic[6:9, 18:21] = -c
    return float(np.max(np.abs(numeric-analytic)))


def injection_reset_check():
    correction = np.array([.1, -.04, .03])
    r = np.linalg.norm(correction)
    s = skew(correction)
    analytic = np.eye(3)+(1-math.cos(r))/r**2*s+(r-math.sin(r))/r**3*s@s
    inverse = Rotation.from_rotvec(-correction)
    numeric = np.column_stack([((Rotation.from_rotvec(correction+np.eye(3)[j]*1e-6)*inverse).as_rotvec()
                                -(Rotation.from_rotvec(correction-np.eye(3)[j]*1e-6)*inverse).as_rotvec())/2e-6
                               for j in range(3)])
    return float(np.max(np.abs(analytic-numeric)))


def sampled_ensemble():
    """Independent coupled small-error Monte Carlo, not an equipment fit."""
    rng = np.random.default_rng(20261102)
    h, count = .01, 8192
    f = np.zeros((21, 21))
    f[:3, 3:6] = np.eye(3)
    f[3:6, 6:9] = -skew([5., 0., 0.])
    f[3:6, 9:12] = f[3:6, 15:18] = -np.eye(3)
    f[6:9, 12:15] = f[6:9, 18:21] = -np.eye(3)
    phi = expm(h*f)
    initial_variance = np.repeat([25., .25, .001**2, .01**2, .0001**2, .03**2/h, .0001**2/h], 3)
    p = np.diag(initial_variance)
    errors = rng.normal(size=(count, 21))*np.sqrt(initial_variance)
    for _ in range(20):
        errors = errors @ phi.T
        p = phi @ p @ phi.T
        for start, sigma in [(9, .0001*math.sqrt(h)), (12, .000001*math.sqrt(h))]:
            errors[:, start:start+3] += rng.normal(size=(count, 3))*sigma
            p[start:start+3, start:start+3] += np.eye(3)*sigma*sigma
        for start, sigma in [(15, .03/math.sqrt(h)), (18, .0001/math.sqrt(h))]:
            errors[:, start:start+3] = rng.normal(size=(count, 3))*sigma
            p[start:start+3, :] = p[:, start:start+3] = 0.
            p[start:start+3, start:start+3] = np.eye(3)*sigma*sigma
    mean_nees = np.einsum('ij,ij->', errors, np.linalg.solve(p, errors.T).T)/count
    variance_error = np.max(np.abs(np.mean(errors**2, axis=0)/np.diag(p)-1))
    return {'realizations': count, 'expected_nees': 21, 'mean_nees': float(mean_nees),
            'maximum_relative_diagonal_second_moment_error': float(variance_error),
            'minimum_covariance_eigenvalue': float(np.linalg.eigvalsh(p).min())}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    residual = linearization_check()
    reset_residual = injection_reset_check()
    p = sampled_noise_gnss()
    ensemble = sampled_ensemble()
    result = {'schema': 'fsim.strapdown-independent-reference.v1',
              'method': 'Independent NumPy/SciPy rotations, finite differences, matrix exponential and Joseph update; no production imports.',
              'error_dynamics_max_abs_residual': residual,
              'attitude_reset_max_abs_residual': reset_residual,
              'sampled_noise_mid_exposure_gnss_covariance_p_v_white': p.tolist(),
              'coupled_sampled_error_ensemble': ensemble,
              'passed': bool(residual < 2e-8 and reset_residual < 1e-9 and np.linalg.eigvalsh(p).min() >= 0
                             and abs(ensemble['mean_nees']/21-1) < .05
                             and ensemble['maximum_relative_diagonal_second_moment_error'] < .1
                             and ensemble['minimum_covariance_eigenvalue'] > 0)}
    text = json.dumps(result, indent=2, allow_nan=False) + '\n'
    if args.out:
        args.out.write_text(text, encoding='utf-8', newline='\n')
    print(text, end='')
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
