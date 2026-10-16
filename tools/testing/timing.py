"""Where test time goes (docs/development.md, "Change-scoped testing").

    python tools/testing/timing.py record <ctest output log>... [--preset ucrt64-headless]
    python tools/testing/timing.py report [--preset ucrt64-headless] [--top 30]

`record` reads CTest's console output (`N/M Test #K: name ....   Passed   12.34 sec`, as `ctest` prints it and as
tools/testing/scope.py --log-dir keeps it) into build/<preset>/test-times.json: each test's latest outcome and seconds,
and - kept apart, so a failure, timeout or a test not run cannot overwrite it - its latest passing seconds, measured
under the run's own parallel load, which is what a later run of the same selection meets. tools/testing/scope.py
estimates from the passing seconds. `report` prints the time by label, the slowest tests, the
duration distribution and the full suite's floor: its longest test, or its total over the preset's jobs. CTest's own
CTestCostData.txt is a moving average that does not follow the tests driven through `cmake -P` scripts. Standard
library only."""
import argparse
import collections
import json
from pathlib import Path
import re
import subprocess
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
LINE = re.compile(r"Test +#\d+: (.+?) \.+(?:\*\*\*)?\s*(Passed|Failed|Not Run|Exception: \w+|Timeout|\w+)\s+([\d.]+) sec")


def store_path(preset):
    return REPO / "build" / preset / "test-times.json"


def read_log(path):
    raw = Path(path).read_bytes()
    return raw.decode("utf-16" if raw[:2] in (b"\xff\xfe", b"\xfe\xff") else "utf-8", errors="replace")


def parse(text):
    """{test name: (seconds, outcome)} from CTest's console output."""
    return {m.group(1).strip(): (float(m.group(3)), m.group(2)) for m in LINE.finditer(text)}


def load(preset, store=None):
    path = Path(store) if store else store_path(preset)
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}


def record(logs, preset, store=None):
    store = Path(store) if store else store_path(preset)
    times = load(preset, store)
    found = 0
    for log in logs:
        for name, (seconds, outcome) in parse(read_log(log)).items():
            entry = times.get(name, {})
            entry.update(outcome=outcome, last_seconds=seconds, log=str(Path(log).resolve()))
            if outcome == "Passed":
                entry["seconds"] = seconds  # (the passing duration; anything else leaves the earlier one)
            times[name] = entry
            found += 1
    store.parent.mkdir(parents=True, exist_ok=True)
    store.write_text(json.dumps(times, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    return found


def seconds(preset, store=None):
    """{test name: seconds} - the measured store's passing seconds, else CTest's cost data."""
    out = {}
    cost = REPO / "build" / preset / "Testing" / "Temporary" / "CTestCostData.txt"
    if cost.is_file():
        for line in cost.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.strip() == "---":
                break
            parts = line.rsplit(" ", 2)
            if len(parts) == 3:
                try:
                    out[parts[0]] = float(parts[2])
                except ValueError:
                    pass
    for name, entry in load(preset, store).items():
        # (an entry without last_seconds predates the separation: its seconds count only if it passed)
        if entry.get("seconds") is not None and ("last_seconds" in entry or entry.get("outcome") == "Passed"):
            out[name] = entry["seconds"]
    return out


def report(preset, top, jobs):
    import scope
    listing = subprocess.run([scope.CTEST, "--preset", preset, "--show-only=json-v1"], cwd=REPO,
                             capture_output=True, text=True)
    if listing.returncode != 0:
        print("no %s test inventory" % preset)
        return 2
    tests = []
    for t in json.loads(listing.stdout)["tests"]:
        props = {p["name"]: p["value"] for p in t.get("properties", [])}
        tests.append((t["name"], props.get("LABELS", [])))
    measured = seconds(preset)
    known = [(n, measured[n], labels) for n, labels in tests if n in measured]
    total = sum(s for _, s, _ in known)
    print("%s: %d tests, %d measured, %.0f s in all; floor of a full run at -j %d: %.0f s" % (
        preset, len(tests), len(known), total, jobs, max(max((s for _, s, _ in known), default=0), total / jobs)))
    by_label = collections.defaultdict(lambda: [0, 0.0])
    for _, s, labels in known:
        for label in labels or ["(none)"]:
            by_label[label][0] += 1
            by_label[label][1] += s
    print("\nby label (tests, seconds):")
    for label, (n, s) in sorted(by_label.items(), key=lambda kv: -kv[1][1]):
        print("  %-24s %4d %8.0f" % (label, n, s))
    print("\nslowest %d:" % top)
    for n, s, labels in sorted(known, key=lambda k: -k[1])[:top]:
        print("  %7.1f  %-72s %s" % (s, n[:72], ",".join(labels)))
    print("\nduration distribution:")
    for lo, hi in ((0, 1), (1, 5), (5, 30), (30, 120), (120, None)):
        sel = [s for _, s, _ in known if s >= lo and (hi is None or s < hi)]
        print("  %4s-%-5s s: %4d tests, %7.0f s" % (lo, hi if hi else "", len(sel), sum(sel)))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("action", choices=("record", "report"))
    ap.add_argument("logs", nargs="*")
    ap.add_argument("--preset", default="ucrt64-headless")
    ap.add_argument("--top", type=int, default=30)
    ap.add_argument("--jobs", type=int, default=16)
    a = ap.parse_args()
    if a.action == "record":
        if not a.logs:
            ap.error("record needs at least one CTest output log")
        print("%d test times recorded into %s" % (record(a.logs, a.preset), store_path(a.preset)))
        return 0
    return report(a.preset, a.top, a.jobs)


if __name__ == "__main__":
    sys.exit(main())
