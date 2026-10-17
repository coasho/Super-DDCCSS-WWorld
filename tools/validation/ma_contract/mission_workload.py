"""A fixed, recorded mission-batch workload (M7 core; docs/sdk/mission-training.md, section 18): initialization,
throughput, memory and episode reclamation of fsim.mission.MissionBatch, measured, plus a digest of every document the
workload produced - the same digest on every run is the evidence that the workload was the same.

    python tools/validation/ma_contract/mission_workload.py --python-path build/ucrt64-release/python [--out result.json]

The workload: `--slots` slots of the contract's two-ship configuration (distinct env_ids), NextStep autoreset after
`--episode-ticks` decision ticks, `--steps` batch steps; the wing submits an HSA at every fifth tick of its slot. Each
decision tick is `decision_steps` (30) World steps of 4 frames. Reclamation: the batch is built, run and destroyed
`--rebuilds` times; the process's working set and private commit after each are recorded (the working set moves with
the allocator's and the OS's trimming; the private commit is the reclamation measure). Windows only for memory (GetProcessMemoryInfo);
standard library only."""
import argparse
import ctypes
import gc
import hashlib
import json
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
WING = 9007199254740995


class _Counters(ctypes.Structure):
    _fields_ = [("cb", ctypes.c_ulong), ("PageFaultCount", ctypes.c_ulong), ("PeakWorkingSetSize", ctypes.c_size_t),
                ("WorkingSetSize", ctypes.c_size_t), ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
                ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t), ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t)]


def memory_mb():
    """(working set, peak working set, private commit) of this process in MB, or Nones off Windows."""
    if os.name != "nt":
        return None, None, None
    from ctypes import wintypes
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.GetCurrentProcess.restype = wintypes.HANDLE
    kernel32.K32GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(_Counters), wintypes.DWORD]
    kernel32.K32GetProcessMemoryInfo.restype = wintypes.BOOL
    c = _Counters()
    c.cb = ctypes.sizeof(c)
    if not kernel32.K32GetProcessMemoryInfo(kernel32.GetCurrentProcess(), ctypes.byref(c), c.cb):
        raise OSError(ctypes.get_last_error(), "GetProcessMemoryInfo failed")
    return round(c.WorkingSetSize / 2**20, 1), round(c.PeakWorkingSetSize / 2**20, 1), round(c.PagefileUsage / 2**20, 1)


def configs(slots, episode_ticks):
    with open(os.path.join(REPO, "tests", "training", "fixtures", "mission_training", "contract", "config.two-ship-c2.json"), "rb") as f:
        base = json.loads(f.read().decode("utf-8"))
    out = []
    for k in range(slots):
        c = json.loads(json.dumps(base))
        c["env_id"] = str(1000 + k)
        c["task"]["criteria"] = [{"id": "time", "kind": "tick_limit", "outcome": "truncated", "max_ticks": episode_ticks}]
        out.append(json.dumps(c))
    return out


def run(mission, texts, steps, workers):
    """One build, reset, `steps` steps and destruction. Its timings and the digest of what it produced."""
    digest = hashlib.sha256()
    t0 = time.perf_counter()
    batch = mission.MissionBatch(texts, autoreset="next_step", workers=workers)
    t1 = time.perf_counter()
    batch.reset()
    t2 = time.perf_counter()
    episodes, stepping = 0, 0.0
    for _ in range(steps):
        for slot in range(len(batch)):
            status = batch.status(slot)
            if status.episode_done or status.tick % 5:
                continue
            wing = batch.endpoint(slot, WING)
            snapshot = json.loads(wing.observation_json())
            wing.submit_json(json.dumps({"schema": "ma.actions.v1", "episode": snapshot["episode"], "service_id": str(WING), "tick": snapshot["tick"],
                                         "requests": [{"request_id": str(status.tick + 1), "kind": "flight", "operation": "new",
                                                       "parameters": {"mode": "hsa", "heading_rad": 0.1 * (slot + 1)}}]}))
        s0 = time.perf_counter()
        batch.step()
        stepping += time.perf_counter() - s0
        for slot in range(len(batch)):
            status = batch.status(slot)
            episodes += status.ended
            digest.update(batch.referee(slot).truth_json().encode())
            if status.ended:
                digest.update(batch.final_outcome_json(slot).encode())
    t3 = time.perf_counter()
    batch.close()
    t4 = time.perf_counter()
    return {"construct_s": round(t1 - t0, 4), "reset_s": round(t2 - t1, 4), "run_s": round(t3 - t2, 3), "step_s": round(stepping, 3),
            "close_s": round(t4 - t3, 4), "episodes_ended": episodes, "digest": digest.hexdigest()}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--python-path", required=True, help="the staged or installed Python package's parent directory")
    ap.add_argument("--slots", type=int, default=8)
    ap.add_argument("--steps", type=int, default=60)
    ap.add_argument("--episode-ticks", type=int, default=20)
    ap.add_argument("--workers", type=int, nargs="+", default=[1, 4])
    ap.add_argument("--rebuilds", type=int, default=4)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    sys.path.insert(0, os.path.abspath(a.python_path))
    import fsim
    from fsim import mission
    fsim.set_log_level(4)  # (errors alone: the workload's own output)
    texts = configs(a.slots, a.episode_ticks)
    result = {"fsim": os.path.dirname(os.path.realpath(fsim.__file__)), "slots": a.slots, "steps": a.steps, "episode_ticks": a.episode_ticks,
              "world_steps_per_tick": 30, "memory_before_mb": memory_mb()[0], "runs": {}}
    for workers in a.workers:
        r = run(mission, texts, a.steps, workers)
        r["decision_ticks_per_s"] = round(a.slots * a.steps / r["step_s"], 1)
        r["world_steps_per_s"] = round(a.slots * a.steps * 30 / r["step_s"], 1)
        result["runs"][str(workers)] = r
    rebuilds = []
    for _ in range(a.rebuilds):
        r = run(mission, texts, 10, max(a.workers))
        gc.collect()
        working, _, private = memory_mb()
        rebuilds.append({"working_set_mb": working, "private_mb": private, "digest": r["digest"]})
    result["reclamation"] = rebuilds
    result["memory_peak_mb"] = memory_mb()[1]
    digests = {r["digest"] for r in result["runs"].values()}
    result["same_digest_across_workers"] = len(digests) == 1
    result["same_digest_across_rebuilds"] = len({r["digest"] for r in rebuilds}) == 1
    private = [r["private_mb"] for r in rebuilds if r["private_mb"] is not None]
    # (bounded reclamation: the later half of the rebuilds holds no more than the earlier half did)
    half = len(private) // 2
    result["private_mb_max_by_half"] = [max(private[:half]), max(private[half:])] if half else None
    text = json.dumps(result, indent=1, sort_keys=True)
    if a.out:
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(text + "\n")
    print(text)
    return 0 if result["same_digest_across_workers"] and result["same_digest_across_rebuilds"] else 1


if __name__ == "__main__":
    sys.exit(main())
