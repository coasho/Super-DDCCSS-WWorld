"""Run the composition example (docs/sdk/ma-composition.md):

    fsim python -m ma_composition.run [--ticks N] [--trace trace.json]     (from examples/python)

One mission episode, its lead flight service's MA system - two policy components, a behaviour-tree executive and a
planner sharing one snapshot per tick, arbitrated deterministically - and a scripted C2. It runs the same local episode
twice in one episode object (a reset between) and once in a fresh object, and checks that all three traces are the
same: nothing of a generation survives into the next. Then it runs two generations of the same system under the SDK's
training adapter (fsim.training) over a batch, with "same_step" and with "next_step" autoreset, and checks each
generation's trace against the direct run of the same local episode. It uses the installed public `fsim` package
alone and publishes nothing; the trace says which package and library it ran on."""
import argparse
import json
import os
import sys

import fsim
from fsim import mission

from .batch import reference, under_training
from .scenario import config_text, make_system, run_episode


def library():
    """Where the loaded libfsim.dll came from (Windows), else None."""
    if os.name != "nt":
        return None
    import ctypes
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.GetModuleHandleW.restype = ctypes.c_void_p
    kernel.GetModuleHandleW.argtypes = [ctypes.c_wchar_p]
    kernel.GetModuleFileNameW.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_uint32]
    module = kernel.GetModuleHandleW("libfsim.dll")
    if not module:
        return None
    buffer = ctypes.create_unicode_buffer(32768)
    kernel.GetModuleFileNameW(module, buffer, 32768)
    return buffer.value


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ticks", type=int, default=10)
    ap.add_argument("--local", type=int, default=7, help="the local episode index")
    ap.add_argument("--trace", default=None, help="write the trace here (JSON)")
    a = ap.parse_args(argv)
    system = make_system()
    with mission.MissionEpisode(config_text()) as episode:
        first = run_episode(episode, system, a.local, a.ticks)
        again = run_episode(episode, system, a.local, a.ticks)  # (a reset: the next generation of the same episode)
    with mission.MissionEpisode(config_text()) as fresh:
        reference_run = run_episode(fresh, make_system(), a.local, a.ticks)
    same = first == again == reference_run
    direct = reference(a.ticks, a.local)
    training = {mode: under_training(a.ticks, a.local, mode) == direct for mode in ("same_step", "next_step")}
    out = {"fsim": os.path.dirname(os.path.realpath(fsim.__file__)), "library": library(), "same_after_reset": same,
           "same_under_training": training, "ticks": a.ticks, "trace": first}
    if a.trace:
        with open(a.trace, "w", encoding="utf-8", newline="\n") as f:
            json.dump(out, f, sort_keys=True, indent=1)
    decided = first["records"]["decisions"]
    print("composition: %d ticks, %d proposals, %d selected, %d not selected; %d receipts fed back; same after reset: %s"
          % (a.ticks, len(first["records"]["proposals"]), sum(1 for d in decided if d[4] is not None),
             sum(1 for d in decided if d[4] is None), len(first["records"]["receipts"]), same))
    print("under the training adapter, two generations each the same as run directly: %s" % training)
    print("package: %s" % out["fsim"])
    print("library: %s" % out["library"])
    return 0 if same and all(training.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
