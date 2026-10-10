"""fsim testbench: pick a scenario, set its options, run it and watch it - its checks, its vehicles' state and its
activities in this window, the aircraft in flightsim-viewer beside it.

    Testbench.cmd (double-click)        the window (a Microsoft Edge app window) and the viewer; or:
    fsim testbench                      the same, from a terminal
    fsim testbench --no-viewer          the window alone (start the viewer yourself: fsim viewer --world testbench)
    fsim testbench --run fa9.launch     start a scenario at once
    fsim testbench --list               print the scenarios and exit
    fsim testbench --headless fa9.launch[,fa10.recovery...]   run scenarios as fast as they go, no window; exit 1 on a failure
"""
import argparse
import os
import shutil
import subprocess
import sys
import time
import webbrowser

from . import registry, runner, server

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))  # (testbench/fsim_testbench)


def find_viewer(given=None):
    candidates = [given] if given else []
    if os.environ.get("FSIM_BIN"):
        candidates.append(os.path.join(os.environ["FSIM_BIN"], "flightsim-viewer.exe"))
    for d in ("build/ucrt64-release/bin", "build/ucrt64-release/dist/viewer/bin", "build/ucrt64-debug/bin"):
        candidates.append(os.path.join(ROOT, d, "flightsim-viewer.exe"))
    return next((c for c in candidates if c and os.path.isfile(c)), None)


def find_edge(given=None):
    candidates = [given] if given else []
    for base in (os.environ.get("ProgramFiles(x86)"), os.environ.get("ProgramFiles"), os.environ.get("LOCALAPPDATA")):
        if base:
            candidates.append(os.path.join(base, "Microsoft", "Edge", "Application", "msedge.exe"))
    candidates.append(shutil.which("msedge"))
    return next((c for c in candidates if c and os.path.isfile(c)), None)


class App:
    speeds = runner.SPEEDS

    def __init__(self, args):
        self.args = args
        self.viewer_path = None if args.no_viewer else find_viewer(args.viewer)
        self.viewer = None

    def viewer_available(self):
        return self.viewer_path is not None

    def viewer_running(self):
        return self.viewer is not None and self.viewer.poll() is None

    def start_viewer(self):
        if self.viewer_path is None or self.viewer_running():
            return
        cmd = [self.viewer_path, "--world", runner.WORLD_NAME, "--camera", "chase"] + self.args.viewer_args
        print("> " + subprocess.list2cmdline(cmd), flush=True)
        self.viewer = subprocess.Popen(cmd, cwd=os.path.dirname(self.viewer_path))

    def open_window(self, url):
        edge = find_edge(self.args.browser)
        if edge:
            # (the window's browser profile beside the program, with its logs - nothing in the user's profile)
            runtime = os.path.dirname(os.path.dirname(self.viewer_path)) if self.viewer_path else os.path.join(ROOT, "build")
            profile = os.path.join(runtime, "logs", "testbench-window")
            cmd = [edge, "--app=" + url, "--window-size=1280,900", "--user-data-dir=" + profile, "--no-first-run",
                   "--no-default-browser-check"]
            print("> " + subprocess.list2cmdline(cmd), flush=True)
            subprocess.Popen(cmd)
        else:
            print("Microsoft Edge not found: opening the default browser", flush=True)
            webbrowser.open(url)


def headless(ids, args):
    """Each scenario run as fast as it goes, its result printed; 1 if any failed."""
    r = runner.Runner(registry.REGISTRY, offline=args.offline)
    r.speed = 0.0
    r.signal_level = False  # (no viewer to draw a range-Doppler map: every radar at the detection level)
    r.start()
    failed = []
    given = dict(o.split("=", 1) for o in (args.option or []))
    for sid in ids:
        r.submit("run", sid, {k: v for k, v in given.items() if any(o.name == k for o in registry.REGISTRY[sid].options)})
        time.sleep(0.5)
        while True:
            with r.lock:
                snap = dict(r.snapshot)
            runs = snap.get("runs") or [{}]
            if runs[0].get("scenario") == sid and runs[0].get("status") in ("finished", "failed", "rejected", "error"):
                break
            time.sleep(0.2)
        run = runs[0]
        snap["status"] = run["status"]
        print("%-28s %s  (%s s simulated)" % (sid, snap["status"].upper(), snap.get("time")), flush=True)
        for m in run.get("motion", []):
            print("    moved: %-12s %6.2f km, ground speed up to %5.1f m/s, bank up to %3d deg" % (m["name"], m["path_km"], m["gs_max"], m["bank_max"]),
                  flush=True)
        for v in snap.get("vehicles", []):
            if v.get("radar"):
                print("    radar: %-12s %s" % (v["name"], v["radar"]), flush=True)
        for c in run.get("checks", []):
            print("    %s  %s%s" % (c["state"].upper().ljust(4), c["label"], ": " + c["detail"] if c["detail"] else ""), flush=True)
        if snap["status"] != "finished":
            failed.append(sid)
            for _, line in snap.get("log", [])[-12:]:
                print("    | " + line, flush=True)
    r.shutdown()
    print("%d of %d passed" % (len(ids) - len(failed), len(ids)), flush=True)
    return 1 if failed else 0


def main(argv=None):
    p = argparse.ArgumentParser(prog="fsim testbench", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=0, help="the UI's port on 127.0.0.1 (default: any free one)")
    p.add_argument("--no-window", action="store_true", help="do not open a window; print the address")
    p.add_argument("--no-viewer", action="store_true", help="do not start flightsim-viewer")
    p.add_argument("--viewer", help="flightsim-viewer.exe to start (default: the build's)")
    p.add_argument("--viewer-args", default="", help="more arguments for the viewer, e.g. \"--offline\"")
    p.add_argument("--browser", help="msedge.exe (default: found in Program Files)")
    p.add_argument("--run", help="a scenario to start at once")
    p.add_argument("--list", action="store_true", help="print the scenarios and exit")
    p.add_argument("--headless", help="run these scenarios (comma separated, or 'all', or a prefix like 'fa9.') without a window")
    p.add_argument("--keep-alive", action="store_true", help="keep serving after the window closes")
    p.add_argument("--option", action="append", metavar="NAME=VALUE",
                   help="headless: an option for every scenario that has it, e.g. --option aircraft=c17a (repeatable)")
    p.add_argument("--offline", action="store_true",
                   help="(the default) never fetch a map tile: the physics' terrain and the viewer's map from the local map set")
    p.add_argument("--online", action="store_true",
                   help="also download map tiles the local map set lacks, into it")
    args = p.parse_args(argv)
    args.viewer_args = args.viewer_args.split()
    args.offline = not args.online
    if args.online and "--online" not in args.viewer_args:
        args.viewer_args.append("--online")

    registry.discover()
    if args.list:
        group = None
        for s in registry.REGISTRY.values():
            if s.group != group:
                group = s.group
                print("\n" + group)
            print("  %-26s %s" % (s.id, s.title))
        return 0
    if args.headless:
        ids = []
        for part in args.headless.split(","):
            part = part.strip()
            if part == "all":
                ids += [k for k, s in registry.REGISTRY.items() if not s.interactive]
            elif part in registry.REGISTRY:
                ids.append(part)
            else:
                matched = [k for k, s in registry.REGISTRY.items() if k.startswith(part) and not s.interactive]
                if not matched:
                    print("no scenario %r (fsim testbench --list)" % part)
                    return 2
                ids += matched
        return headless(ids, args)

    app = App(args)
    r = runner.Runner(registry.REGISTRY, offline=args.offline)
    r.start()
    srv = server.serve(r, app, args.port)
    url = "http://127.0.0.1:%d/" % srv.server_address[1]
    print("fsim testbench: %s  (%d scenarios)" % (url, len(registry.REGISTRY)), flush=True)
    if args.run:
        r.submit("run", args.run, {})
    if not args.no_viewer:
        if app.viewer_path:
            app.start_viewer()
        else:
            print("flightsim-viewer.exe not found: build it (fsim build) or pass --viewer", flush=True)
    if not args.no_window:
        app.open_window(url)
    try:
        while True:
            time.sleep(1.0)
            idle = srv.last_poll is not None and time.monotonic() - srv.last_poll > 20.0
            if idle and not args.keep_alive and not args.no_window:
                print("the window closed: stopping", flush=True)
                break
    except KeyboardInterrupt:
        pass
    r.shutdown()
    srv.shutdown()
    if app.viewer_running():  # (the viewer it started goes with it)
        app.viewer.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
