"""Every aircraft design through hangar again, several at a time, and what came of it.

    python tools/hangar/rebuild_all.py [--jobs 6] [--out build/hangar-rebuild] [--only f16c c172 ...] [stage ...]

Each design is run as `python -m hangar <name> [stages]` in a process of its own (all stages when none is named), its
output kept in <out>/<name>.log. The summary, a line a design: how long it took, how it ended, the checks that failed,
and which of its files in the repository the run changed (`git status`): a tree whose generated files are what the tool
makes today shows none (docs/hangar.md, "Built again"). Stores have their own command (`python -m hangar stores`).

PYTHONPATH and FSIM_BIN as for hangar itself. The exit code is the number of designs that did not end cleanly.
"""
import argparse
import concurrent.futures
import glob
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def designs():
    """The aircraft designs (not the stores'): aircraft/<name>/<name>.toml whose [aircraft] has no store card."""
    out = []
    for path in sorted(glob.glob(os.path.join(ROOT, "aircraft", "*", "*.toml"))):
        name = os.path.basename(os.path.dirname(path))
        if os.path.basename(path) != name + ".toml":
            continue
        text = open(path, encoding="utf-8").read()
        if re.search(r"^\s*card\s*=", text, re.M) or re.search(r'^\s*category\s*=\s*"(missile|store)"', text, re.M):
            continue
        out.append(name)
    return out


def changed(name):
    r = subprocess.run(["git", "status", "--short", "--", "aircraft/" + name], cwd=ROOT, capture_output=True, text=True)
    return [line[3:].strip() for line in r.stdout.splitlines() if line.strip()]


def run(name, stages, out):
    t0 = time.time()
    log = os.path.join(out, name + ".log")
    with open(log, "w", encoding="utf-8") as f:
        code = subprocess.run([sys.executable, "-m", "hangar", name] + stages, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT).returncode
    text = open(log, encoding="utf-8", errors="replace").read()
    failed = [re.sub(r"\s+", " ", line.strip())[:110] for line in text.splitlines() if re.match(r"\s*FAIL\b", line)]
    crashed = "Traceback (most recent call last)" in text
    last = [line for line in text.splitlines() if line.strip()][-1][:110] if crashed else ""
    return dict(name=name, seconds=time.time() - t0, code=code, failed=failed, crashed=crashed, last=last)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--out", default=os.path.join(ROOT, "build", "hangar-rebuild"))
    ap.add_argument("--only", nargs="+", help="these designs (default: every aircraft)")
    ap.add_argument("stages", nargs="*", help="hangar's stages (default: all)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    names = a.only or designs()
    print("%d designs, %d at a time; logs in %s" % (len(names), a.jobs, a.out), flush=True)
    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.jobs) as pool:
        for r in pool.map(lambda n: run(n, a.stages, a.out), names):
            results.append(r)
            how = "stopped: " + r["last"] if r["crashed"] else "%d check(s) failed" % len(r["failed"]) if r["failed"] else "ok" if r["code"] == 0 else "exit %d" % r["code"]
            print("%-12s %5.0f s  %s" % (r["name"], r["seconds"], how), flush=True)
            for f in r["failed"]:
                print("             " + f, flush=True)
    # what the runs changed in the repository, asked once they are all done: git maps the files it compares, and a
    # design writing one of them meanwhile is refused (Windows: "Invalid argument")
    touched = 0
    for r in results:
        files = changed(r["name"])
        touched += bool(files)
        if files:
            print("%-12s changed: %s" % (r["name"], ", ".join(os.path.basename(c) for c in files)), flush=True)
    bad = [r for r in results if r["crashed"] or r["code"] != 0]
    print("%d of %d designs ended cleanly; %d with files changed" % (len(results) - len(bad), len(results), touched))
    return len(bad)


if __name__ == "__main__":
    sys.exit(main())
