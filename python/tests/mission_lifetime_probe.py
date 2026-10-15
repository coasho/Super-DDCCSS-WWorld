"""Lifetime cases for test_mission.py that could end a process: each runs here, in a subprocess of its own, so that a
native failure fails its test instead of the test runner. Usage: mission_lifetime_probe.py <case>; the last line printed
is a JSON verdict, and the exit code is 0 only when every check held.

  close_reset   close() while another thread's reset runs without the GIL: close returns at once, the episode and its
                handles answer "closed" from then on, and the reset thread ends normally
  close_step    the same while a step runs
  overlap_reset a second reset refused as busy changes nothing of the first: its generation's key is the first one's
"""
import json
import os
import sys
import threading
import time

import fsim
from fsim import mission

HERE = os.path.dirname(os.path.abspath(__file__))
CONTRACT = os.path.join(os.path.dirname(os.path.dirname(HERE)), "tests", "fixtures", "mission_training", "contract")
WING = 9007199254740995


def config(extra):
    with open(os.path.join(CONTRACT, "config.two-ship-c2.json"), "rb") as f:
        document = json.loads(f.read().decode("utf-8"))
    document["scenario"]["world"]["capacity"] = extra + 8
    for index in range(extra):
        document["scenario"]["vehicles"].append({"name": "load-%d" % index, "type": "jsbsim:c172x",
                                                 "initial": {"lat_deg": 36.1, "lon_deg": -115.1, "alt_msl_m": 3000,
                                                             "heading_deg": 0, "airspeed_ms": 50}})
    return json.dumps(document)


def met(endpoint, thread):
    """Whether another thread's call was met holding the episode (an earlier handle answers busy meanwhile)."""
    until = time.monotonic() + 30
    while thread.is_alive() and time.monotonic() < until:
        try:
            endpoint.observation_json()
        except fsim.Error as e:
            if "busy" in str(e):
                return True
        time.sleep(0.001)
    return False


def closed(call):
    try:
        call()
    except fsim.Error as e:
        return "closed" in str(e)
    return False


def main(case):
    episode = mission.MissionEpisode(config(36 if case != "close_step" else 4))
    episode.reset(0)
    earlier = episode.endpoint(WING)
    referee = episode.referee()
    outcome = {}

    def work():
        try:
            if case == "close_step":
                for _ in range(200):
                    episode.step()
            else:
                episode.reset(11)
            outcome["first"] = "returned"
        except fsim.Error as e:
            outcome["first"] = "fsim.Error: %s" % e

    thread = threading.Thread(target=work)
    thread.start()
    verdict = {"case": case, "met_busy": met(earlier, thread)}
    if case == "overlap_reset":
        try:
            episode.reset(22)
            verdict["second"] = "returned"
        except fsim.Error as e:
            verdict["second"] = "busy" if "busy" in str(e) else str(e)
        thread.join(60)
        verdict["first"] = outcome.get("first")
        verdict["episode"] = json.loads(episode.endpoint(WING).observation_json())["episode"]
        ok = (verdict["met_busy"] and verdict["second"] == "busy" and verdict["first"] == "returned" and
              verdict["episode"] == {"env_id": "9007199254740993", "episode_generation": "2", "local_episode_index": "11"})
    else:
        start = time.monotonic()
        episode.close()
        verdict["close_s"] = round(time.monotonic() - start, 3)
        verdict["episode_closed"] = closed(lambda: episode.endpoint(WING))
        verdict["endpoint_closed"] = closed(earlier.observation_json)
        verdict["referee_closed"] = closed(referee.truth_json)
        thread.join(60)
        verdict["first"] = outcome.get("first")
        episode.close()  # (again: nothing)
        ok = (verdict["met_busy"] and not thread.is_alive() and verdict["episode_closed"] and verdict["endpoint_closed"] and
              verdict["referee_closed"] and verdict["first"] is not None and verdict["close_s"] < 5.0)
    verdict["ok"] = bool(ok)
    print(json.dumps(verdict), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
