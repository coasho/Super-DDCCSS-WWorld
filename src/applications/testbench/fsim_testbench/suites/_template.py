"""A template for a new system's suite (OMS, P2P, C2, ...): copy this file to suites/<system>.py (no leading underscore,
or it is not loaded) and write its scenarios. Each is a generator given a Test (`t`, fsim_testbench/runner.py):

    t.spawn(kind, north_m, east_m, agl_m=..., heading_deg=...)   a vehicle (north/east of the run's runway threshold, in its lane)
    t.spawn_on_runway(kind)                                       parked on 28R, facing down it
    t.world                                                       the fsim.World (published: the viewer shows it)
    t.watch(activity, "label")                                    shown in the Activities table
    t.metric("label", fn)                                         a live readout (sampled for the window)
    t.every(fn)                                                   fn() every 0.1 s simulated: records checks read
    t.action("label", fn)                                         a button while it runs
    t.check("what must hold", ok, "detail")                      a check, shown and judged
    t.log("text")
    yield t.wait(seconds) / t.until(fn, timeout_s, "label") / t.until_done(activity, timeout_s)
                                                                  (each resumes with True, or False on its timeout)

    import fsim
    from ..registry import aircraft, choice, number, scenario, suite

    suite("OMS", "Operations and mission systems: ...")

    @scenario("oms.example", suite="OMS", group="OMS-1 Something", title="A first OMS check", refs="OMS ADR-n",
              summary="What it shows, in a sentence or two.", options=[aircraft("c172", ("c172", "f16c"))])
    def example(t):
        v = t.spawn(t.opt("aircraft"), 0.0, 4000.0, agl_m=1000.0)
        a = t.watch(v.submit_hsa(heading_rad=0.0), "hsa north")
        ok = yield t.until(lambda: abs(v.state.euler_rad[2]) < 0.05, 120.0, "heading north")
        t.check("turned north", ok)

Run it in the window (fsim testbench), or without one: fsim testbench --headless oms. --offline keeps every map tile to
the cache.
"""
