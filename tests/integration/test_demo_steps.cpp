// The demo publisher's single steps (applications/demo/DemoSteps.h; ARCH-08 HC-28, HC-30): every request is one step
// the simulation takes while held, and no step of its own is mistaken for one.
//
//   schedules   DemoSteps against a model of the runner's thread (sim::SimRunner's loop as written: a step decided at
//               the loop's top - `!paused || latch` - and published when it completes; stop joins the thread, so a
//               step under way completes first). Its events are interleaved deterministically with the publisher's
//               passes: the review's delayed-work schedule, requests made while running, a backlog through a resume
//               and a pause, and 2000 seeded random schedules - in each, the requested steps the model took are the
//               requests made, one each, and DemoSteps counts exactly those
//   in-process  DemoSteps on the real runner (DemoSimulation): requests made while it runs, then a hold - exactly that
//               many agent steps after the hold's state; a backlog through a resume and a pause, likewise; and 30
//               holds and resumes - each joins the runner's thread and starts another - after 4 to warm up: the same
//               process handle count before and after (HC-32: a thread's sleep timer went unclosed with the thread)
//
// Usage: fsim_demo_steps_test <jsbsim root>

#include "applications/demo/DemoSimulation.h"
#include "applications/demo/DemoSteps.h"
#include "foundation/io/AssetResolver.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <thread>

#ifdef _WIN32
#    ifndef NOMINMAX
#        define NOMINMAX
#    endif
#    include <windows.h>
#endif

using namespace fsim;

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
        ++failures;
    }
}

/// sim::SimRunner's thread as its loop is written, its events taken one at a time by the schedule.
struct ModelRunner {
    bool paused = false, latch = false, thread = true; // (started running, as the publisher starts an unheld demo)
    bool underWay = false, underWayRequested = false;  // a step decided at the loop's top, not yet published
    int published = 0;                                 // snapshots since the publisher's last acquire
    std::uint64_t requestedSteps = 0, ownSteps = 0;    // what it took, by cause: the latch while paused, or running
    bool latchWhileRunning = false;                    // the latch ever set while the runner runs

    // DemoSteps' runner
    void setPaused(bool p) {
        paused = p;
        note();
    }
    void singleStep() {
        latch = true;
        note();
    }
    void stop() { // its thread joined: the step under way completes first
        if (underWay) complete();
        thread = false;
    }
    void start() { thread = true; }

    // the thread's events
    bool canDecide() const { return thread && !underWay; }
    void decide() { // the loop's top: `!paused_ || singleStep_.exchange(false)`
        bool step = !paused;
        if (paused) {
            step = latch;
            latch = false;
        }
        if (step) {
            underWay = true;
            underWayRequested = paused;
        }
    }
    void complete() {
        underWay = false;
        ++published;
        ++(underWayRequested ? requestedSteps : ownSteps);
    }
    // the publisher's acquire: whether anything was published since the last
    bool acquire() {
        const bool fresh = published > 0;
        published = 0;
        return fresh;
    }
    void note() {
        if (latch && !paused) latchWhileRunning = true;
    }
};

/// The viewer and the publisher around a model runner.
struct ModelDemo {
    ModelRunner runner;
    app::DemoSteps<ModelRunner> steps{runner, 0};
    std::uint64_t requested = 0;
    bool held = false;

    void pass() { steps.update(runner.acquire(), requested, held); }
    /// The thread's next event, if it has one: complete the step under way, else decide at the loop's top.
    void tick() {
        if (runner.underWay) runner.complete();
        else if (runner.canDecide()) runner.decide();
    }
    /// Held, as long as it takes: every request taken (bounded: it fails rather than spins).
    bool drain() {
        held = true;
        for (int i = 0; i < 100000; ++i) {
            pass();
            tick();
            if (steps.taken() == requested && !runner.underWay && !runner.latch) {
                pass();
                return steps.taken() == requested;
            }
        }
        return false;
    }
    void expect(const std::string& schedule, std::uint64_t ownSteps = UINT64_MAX) {
        check(runner.requestedSteps == requested, schedule + ": " + std::to_string(requested) + " requests, " +
                                                      std::to_string(runner.requestedSteps) + " requested steps taken");
        check(steps.taken() == requested, schedule + ": DemoSteps counted " + std::to_string(steps.taken()) + " of " +
                                              std::to_string(requested));
        check(steps.unexpected() == 0, schedule + ": a held runner published a step it was not handed");
        check(!runner.latchWhileRunning, schedule + ": the latch was set while the runner ran");
        if (ownSteps != UINT64_MAX)
            check(runner.ownSteps == ownSteps, schedule + ": " + std::to_string(runner.ownSteps) + " running steps, expected " +
                                                   std::to_string(ownSteps));
    }
};

void schedules() {
    {
        // The review's delayed-work schedule (HC-28): a running step is under way as the hold and two requests come,
        // and is slow - it completes only when the thread is joined. The hold's join waits for it; it is not a request.
        ModelDemo d;
        d.pass();
        d.runner.decide(); // a running step, under way
        d.held = true;
        d.requested = 2;
        for (int i = 0; i < 200; ++i) d.pass(); // (however long: no request is handed before the join)
        check(d.drain(), "delayed work: the requests were not all taken");
        d.expect("delayed work", 1);
    }
    {
        // Requests made while it runs (HC-30): none is handed while it runs, and all are taken at the next hold.
        ModelDemo d;
        for (int i = 0; i < 50; ++i) {
            d.pass();
            d.tick();
            if (i % 10 == 0) ++d.requested;
        }
        check(d.steps.taken() == 0 && d.runner.requestedSteps == 0, "while running: a request was taken before the hold");
        check(d.drain(), "while running: the requests were not all taken at the hold");
        d.expect("requests while running");
    }
    {
        // A backlog through a resume and a pause (HC-30): ten requests held, three taken, a resume - the step handed
        // over is finished first - then two more while it runs, and a hold: the rest are taken, one each.
        ModelDemo d;
        d.held = true;
        d.requested = 10;
        for (int i = 0; i < 10000 && d.steps.taken() < 3; ++i) {
            d.pass();
            d.tick();
        }
        d.held = false;
        for (int i = 0; i < 60; ++i) {
            d.pass();
            d.tick();
            if (i == 30) d.requested += 2;
        }
        check(d.runner.ownSteps > 0, "backlog: the resumed runner took no step of its own");
        check(d.drain(), "backlog: the requests were not all taken after the resume and pause");
        d.expect("backlog through a resume and a pause");
    }
    // Seeded random schedules: the viewer holds, resumes and asks at random; the thread and the publisher interleave at
    // random. Every request is one requested step, the runner's own steps are never counted, and the latch is never
    // left set while it runs.
    int schedulesRun = 0;
    for (std::uint32_t seed = 1; seed <= 2000; ++seed) {
        std::mt19937 random(seed);
        std::uniform_int_distribution<int> event(0, 99);
        ModelDemo d;
        d.held = (seed % 2) == 0;
        for (int i = 0; i < 400; ++i) {
            const int e = event(random);
            if (e < 3) d.held = !d.held;
            else if (e < 15) ++d.requested;
            else if (e < 55) d.tick();
            else d.pass();
            if (d.runner.latchWhileRunning || d.runner.requestedSteps > d.requested) break;
        }
        if (!d.drain()) check(false, "seed " + std::to_string(seed) + ": the requests were not all taken");
        const int before = failures;
        d.expect("seed " + std::to_string(seed));
        ++schedulesRun;
        if (failures > before) break;
    }
    std::printf("schedules: delayed work, requests while running, a backlog through a resume and a pause, and %d random "
                "schedules: %s\n", schedulesRun, failures ? "FAILED" : "every request one requested step");
}

/// The real runner, its passes as the publisher makes them.
struct RealDemo {
    std::unique_ptr<app::DemoSimulation> runner;
    std::unique_ptr<app::DemoSteps<app::DemoSimulation>> steps;
    std::uint64_t requested = 0;
    bool held = false;
    double newest = 0.0;
    int snapshots = 0;

    void pass() {
        const auto* fresh = runner->acquire();
        if (fresh) {
            newest = fresh->simTime;
            ++snapshots;
        }
        steps->update(fresh != nullptr, requested, held);
    }
    void passFor(double seconds) {
        const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < until) {
            pass();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    /// Held: the hold's pass (its join), then the pass taking what the old thread published - the hold's state.
    double hold() {
        held = true;
        pass();
        pass();
        return newest;
    }
    bool until(std::uint64_t taken, double seconds = 30.0) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (steps->taken() < taken && std::chrono::steady_clock::now() < deadline) {
            pass();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        pass();
        return steps->taken() >= taken; // (held, it goes on: `taken` is a floor, or all requested)
    }
};

int inProcess(const std::string& jsbsimRoot) {
    app::DemoOptions options;
    options.aircraft = "c172x";
    options.jsbsimRoot = jsbsimRoot;
    options.vehicles = 2;
    options.workers = 1;
    options.seed = 7;
    const double agentStep = options.dt * options.frameSkip;
    io::AssetResolver assets;
    RealDemo d;
    d.runner = app::DemoSimulation::create(options, assets, nullptr);
    if (!d.runner) {
        std::fprintf(stderr, "FAIL: the demo could not be made\n");
        return 1;
    }
    d.runner->start(); // running, as the publisher starts an unheld demo
    d.steps = std::make_unique<app::DemoSteps<app::DemoSimulation>>(*d.runner, 0);
    const auto stepsBetween = [&](double from, double to) { return (to - from) / agentStep; };

    // requests made while it runs: none taken until the hold, then exactly that many steps after the hold's state
    d.passFor(0.2);
    const double runningFrom = d.newest;
    d.requested += 10;
    d.passFor(0.2);
    check(d.newest > runningFrom, "in-process: the demo did not run");
    check(d.steps->taken() == 0, "in-process: a request was taken while the demo ran");
    const double heldAt = d.hold();
    check(d.until(10), "in-process: 10 requests made while running were not all taken after the hold");
    const double ten = stepsBetween(heldAt, d.newest);
    check(std::fabs(ten - 10.0) < 1e-6, "in-process: 10 requests made while running: " + std::to_string(ten) + " agent steps after the hold");
    d.passFor(0.3);
    check(std::fabs(stepsBetween(heldAt, d.newest) - 10.0) < 1e-6, "in-process: a step after the requests were taken");

    // a backlog through a resume and a pause: 12 held, a resume after 4, 3 more while it runs, a hold
    d.requested += 12;
    check(d.until(14), "in-process: the held backlog did not start");
    d.held = false;
    d.passFor(0.3);
    const std::uint64_t takenAtResume = d.steps->taken();
    d.requested += 3;
    d.passFor(0.2);
    check(d.steps->taken() == takenAtResume, "in-process: a request was taken while the demo ran");
    const double heldAgain = d.hold();
    const std::uint64_t pending = d.requested - d.steps->taken();
    check(d.until(d.requested), "in-process: the backlog was not all taken after the resume and pause");
    const double rest = stepsBetween(heldAgain, d.newest);
    check(std::fabs(rest - static_cast<double>(pending)) < 1e-6,
          "in-process: " + std::to_string(pending) + " pending requests: " + std::to_string(rest) + " agent steps after the hold");
    check(d.steps->unexpected() == 0, "in-process: a held runner published a step it was not handed");

    // repeated holds (HC-32): each joins the runner's thread and starts another, which sleeps between its steps
    const auto holdAndResume = [&] {
        d.held = false;
        d.passFor(0.03); // (running: the thread sleeps between its paced steps)
        d.hold();
        d.passFor(0.03); // (held: the new thread sleeps too)
    };
    for (int i = 0; i < 4; ++i) holdAndResume(); // (whatever the first holds open for good is not counted)
#ifdef _WIN32
    const auto handles = [] {
        DWORD count = 0;
        GetProcessHandleCount(GetCurrentProcess(), &count);
        return static_cast<long>(count);
    };
    const long before = handles();
#endif
    for (int i = 0; i < 30; ++i) holdAndResume();
#ifdef _WIN32
    const long after = handles();
    check(after == before, "in-process: process handles " + std::to_string(before) + " before 30 holds and resumes, " +
                               std::to_string(after) + " after");
    std::printf("in-process: 30 holds and resumes after 4: process handles %ld before, %ld after\n", before, after);
#endif
    check(d.steps->taken() == d.requested && d.steps->unexpected() == 0, "in-process: the holds took or published a step");
    d.runner->stop();
    std::printf("in-process: 10 requests while running, then %llu pending through a resume and a pause: exactly as many agent "
                "steps after each hold%s\n", static_cast<unsigned long long>(pending), failures ? " - NOT" : "");
    return failures ? 1 : 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <jsbsim root>\n", argv[0]);
        return 2;
    }
    schedules();
    if (failures) return 1;
    return inProcess(argv[1]);
}
