"""The trackers' cross-check (ADR-30, MS-5): Stone Soup (Dstl, MIT; pip install stonesoup) runs its own components on
scenarios a radar meets - a moving, rotated sensor measuring elevation, bearing, range and range rate, targets that
cross, false alarms - and this writes the measurements and its answers to tests/data/track/crosscheck.json, which
tests/test_track.cpp replays through the ported C++ and compares number by number.

    py -3.13 tools/track/crosscheck.py      (Stone Soup 1.9.1 wrote the committed fixture)
"""
import datetime
import json
import os

import numpy as np
from stonesoup.dataassociator.neighbour import GNNWith2DAssignment
from stonesoup.dataassociator.probability import JPDA
from stonesoup.deleter.time import UpdateTimeStepsDeleter
from stonesoup.hypothesiser.distance import DistanceHypothesiser
from stonesoup.hypothesiser.probability import PDAHypothesiser
from stonesoup.initiator.simple import MultiMeasurementInitiator
from stonesoup.measures import Mahalanobis
from stonesoup.models.measurement.nonlinear import CartesianToElevationBearingRangeRate
from stonesoup.models.transition.linear import CombinedLinearGaussianTransitionModel, ConstantVelocity
from stonesoup.predictor.kalman import ExtendedKalmanPredictor
from stonesoup.tracker.simple import MultiTargetMixtureTracker, MultiTargetTracker
from stonesoup.types.detection import Clutter, Detection
from stonesoup.types.hypothesis import SingleHypothesis
from stonesoup.types.state import GaussianState
from stonesoup.updater.kalman import ExtendedKalmanUpdater, UnscentedKalmanUpdater
import stonesoup

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
START = datetime.datetime(2026, 1, 1)
Q = (2.0, 2.0, 0.5)  # each axis' noise diffusion coefficient
NOISE = np.diag([0.002 ** 2, 0.002 ** 2, 30.0 ** 2, 2.0 ** 2])
ROTATION = np.array([[0.05], [-0.1], [0.7]])
SENSOR0 = np.array([1000.0, -2000.0, 500.0])
SENSOR_V = np.array([200.0, 30.0, -5.0])

transition = CombinedLinearGaussianTransitionModel([ConstantVelocity(q) for q in Q])
predictor = ExtendedKalmanPredictor(transition)


def model_at(t):
    return CartesianToElevationBearingRangeRate(
        ndim_state=6, mapping=(0, 2, 4), velocity_mapping=(1, 3, 5), noise_covar=NOISE, rotation_offset=ROTATION,
        translation_offset=(SENSOR0 + SENSOR_V * t).reshape(3, 1), velocity=SENSOR_V.reshape(3, 1))


def model_json(m):
    return {"rotation": [float(v) for v in np.asarray(m.rotation_matrix).ravel()],
            "translation": [float(v) for v in np.asarray(m.translation_offset).ravel()],
            "velocity": [float(v) for v in np.asarray(m.velocity).ravel()],
            "noise": [float(v) for v in np.diag(m.noise_covar)]}


def state_json(s):
    return {"x": [float(v) for v in np.asarray(s.state_vector, dtype=float).ravel()],
            "p": [float(v) for v in np.asarray(s.covar, dtype=float).ravel()]}


def truths(rng, starts, steps):
    """Each target's states, one a second, by the transition model (its noise drawn)."""
    paths = []
    for x0 in starts:
        x = np.array(x0, dtype=float).reshape(6, 1)
        path = []
        for k in range(steps):
            if k:
                x = transition.function(GaussianState(x, np.eye(6)), noise=transition.rvs(time_interval=datetime.timedelta(seconds=1),
                                                                                          random_state=rng),
                                        time_interval=datetime.timedelta(seconds=1))
                x = np.asarray(x, dtype=float)
            path.append(x.copy())
        paths.append(path)
    return paths


def measure(model, x, rng):
    z = model.function(GaussianState(x, np.eye(6)), noise=False)
    noise = rng.multivariate_normal(np.zeros(4), NOISE).reshape(4, 1)
    return np.asarray(z, dtype=float) + noise


def single(updater, name, rng):
    path = truths(rng, [[20e3, -150.0, 15e3, 40.0, 3e3, 2.0]], 30)[0]
    prior = GaussianState(path[0] + np.array([[300.0], [20.0], [-300.0], [-10.0], [100.0], [0.0]]),
                          np.diag([500.0 ** 2, 50.0 ** 2, 500.0 ** 2, 50.0 ** 2, 200.0 ** 2, 20.0 ** 2]), timestamp=START)
    state = prior
    steps = []
    for k in range(1, len(path)):
        t = START + datetime.timedelta(seconds=k)
        model = model_at(k)
        z = measure(model, path[k], rng)
        det = Detection(z, timestamp=t, measurement_model=model)
        prediction = predictor.predict(state, timestamp=t)
        state = updater.update(SingleHypothesis(prediction, det))
        steps.append({"t": float(k), "z": [float(v) for v in z.ravel()], "model": model_json(model), "posterior": state_json(state)})
    return {"name": name, "filter": "ukf" if isinstance(updater, UnscentedKalmanUpdater) else "ekf", "q": list(Q),
            "prior": dict(state_json(prior), t=0.0), "steps": steps}


def multi(name, kind, rng):
    starts = [[18e3, -120.0, 9e3, 60.0, 4e3, 0.0], [18e3, -120.0, 15e3, -60.0, 4.5e3, 0.0], [25e3, -200.0, 12e3, 0.0, 6e3, -5.0]]
    paths = truths(rng, starts, 40)
    updater = ExtendedKalmanUpdater()
    prior = GaussianState(np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]),
                          np.diag([0.0, 150.0 ** 2, 0.0, 150.0 ** 2, 0.0, 30.0 ** 2]))
    init_hyp = DistanceHypothesiser(predictor, updater, Mahalanobis(), missed_distance=3)
    initiator = MultiMeasurementInitiator(prior_state=prior, deleter=UpdateTimeStepsDeleter(2), data_associator=GNNWith2DAssignment(init_hyp),
                                          updater=updater, min_points=2)
    deleter = UpdateTimeStepsDeleter(3)
    clutter_rate = 2.0  # false alarms a scan, uniform over this box of measurement space
    box = [(-0.05, 0.25), (0.0, 0.6), (10e3, 40e3), (-500.0, 500.0)]
    volume = float(np.prod([hi - lo for lo, hi in box]))
    if kind == "gnn":
        tracker = MultiTargetTracker(initiator=initiator, deleter=deleter, detector=None,
                                     data_associator=GNNWith2DAssignment(DistanceHypothesiser(predictor, updater, Mahalanobis(), missed_distance=3)),
                                     updater=updater)
    else:
        tracker = MultiTargetMixtureTracker(initiator=initiator, deleter=deleter, detector=None,
                                            data_associator=JPDA(PDAHypothesiser(predictor, updater, clutter_spatial_density=1.0 / volume,
                                                                                 prob_detect=0.9, prob_gate=0.99)),
                                            updater=updater)
    steps = []
    for k in range(len(paths[0])):
        t = START + datetime.timedelta(seconds=k)
        model = model_at(k)
        dets = []
        for n, path in enumerate(paths):
            if rng.random() < 0.9:
                dets.append((Detection(measure(model, path[k], rng), timestamp=t, measurement_model=model), n + 1))
        for _ in range(rng.poisson(clutter_rate)):
            z = np.array([[rng.uniform(*b)] for b in box])
            dets.append((Clutter(z, timestamp=t, measurement_model=model), 0))
        # the tracker's step (MultiTargetTracker.__next__'s body, fed directly)
        detections = {d for d, _ in dets}
        tracker.detector_iter = iter([(t, detections)])
        tracker.__next__()
        steps.append({"t": float(k), "model": model_json(model),
                      "detections": [{"z": [float(v) for v in np.asarray(d.state_vector, dtype=float).ravel()], "truth": truth} for d, truth in dets],
                      "tracks": [state_json(tr.state) for tr in tracker.tracks]})
    return {"name": name, "kind": kind, "filter": "ekf", "q": list(Q), "prior": state_json(prior), "missed_distance": 3.0,
            "pd": 0.9, "pg": 0.99, "clutter_density": 1.0 / volume, "min_points": 2, "initiator_steps": 2, "deleter_steps": 3, "steps": steps}


def main():
    rng = np.random.default_rng(20261007)
    out = {"stonesoup": stonesoup.__version__,
           "scenarios": [single(ExtendedKalmanUpdater(), "ekf one target", rng), single(UnscentedKalmanUpdater(), "ukf one target", rng),
                         multi("gnn three crossing targets in clutter", "gnn", rng), multi("jpda three crossing targets in clutter", "jpda", rng)]}
    path = os.path.join(ROOT, "tests", "data", "track", "crosscheck.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(out, f, indent=None, separators=(",", ":"))
    print("wrote", path, os.path.getsize(path), "bytes")
    for s in out["scenarios"]:
        if "kind" in s:
            print(s["name"], "tracks a step:", [len(st["tracks"]) for st in s["steps"]])


if __name__ == "__main__":
    main()
