// The demo publisher (ARCH-08, HC-16): flightsim-demo flies exactly the demo the viewer ran in-process before, and
// takes the viewer's commands through the demo control block as the contract says. Driven by the viewer's own client,
// DemoLink, against the in-process DemoSimulation started from the same options.
//
//   parity     the two held, then stepped together one agent step at a time: after every step each vehicle's state,
//              as the viewer reads it from the world stream, is the in-process state to the bit
//   steps      single steps counted (HC-28): 20 requests made at once are 20 steps, the in-process demo's 20 to the
//              bit and no more; a resume drops the requests not yet taken, so a pause holds; a request after it is
//              still taken; and the publisher stops on request with thousands pending
//   lifecycle  starts that fail - the publisher exits before publishing - leave no process handle behind (HC-29),
//              and neither does a start and stop
//
// Usage: fsim_demo_publisher_test <flightsim-demo> <jsbsim root> [parity|steps|lifecycle]

#include "applications/demo/DemoSimulation.h"
#include "applications/viewer/DemoLink.h"
#include "foundation/io/AssetResolver.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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

/// The next snapshot `source` hands out, waiting for it up to `seconds`.
template <typename Source>
const sim::SnapshotBatch* next(Source& source, double seconds = 30.0) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < deadline) {
        if (const auto* batch = source.acquire()) return batch;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return nullptr;
}

/// Every snapshot the publisher hands out for `seconds`: how many, and the newest one's simulation time in `newest`
/// (unchanged when none came).
int drain(app::DemoLink& link, double seconds, double& newest) {
    int count = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < deadline) {
        if (const auto* batch = link.acquire()) {
            newest = batch->simTime;
            ++count;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return count;
}

bool same(const sim::SnapshotBatch& a, const sim::SnapshotBatch& b, std::size_t vehicles, const std::string& when) {
    bool ok = a.simTime == b.simTime;
    check(ok, when + ": simulation time " + std::to_string(a.simTime) + " in-process, " + std::to_string(b.simTime) + " published");
    for (std::size_t i = 0; i < vehicles; ++i) {
        const bool equal = std::memcmp(&a.states[i], &b.states[i], sizeof(sim::VehicleState)) == 0;
        check(equal, when + ": vehicle " + std::to_string(i) + "'s state differs");
        ok = ok && equal;
    }
    return ok;
}

void idle(double seconds) { std::this_thread::sleep_for(std::chrono::duration<double>(seconds)); }

/// The viewer's demo options, flat ground (no map set needed); the publisher gets them as the viewer passes them.
app::DemoOptions demoOptions(const std::string& jsbsimRoot) {
    app::DemoOptions options;
    options.aircraft = "c172x";
    options.jsbsimRoot = jsbsimRoot;
    options.vehicles = 4;
    options.workers = 2;
    options.seed = 7;
    return options;
}

std::vector<std::string> publisherArguments(const std::string& jsbsimRoot) {
    return {"--aircraft", "c172x", "--jsbsim-root", jsbsimRoot, "--vehicles", "4", "--workers", "2", "--seed", "7",
            "--elevation", "none"};
}

int parity(const std::string& program, const std::string& jsbsimRoot) {
    constexpr int kSteps = 240; // four simulated seconds at the demo's 60 Hz agent rate
    const auto options = demoOptions(jsbsimRoot);
    io::AssetResolver assets;
    auto here = app::DemoSimulation::create(options, assets, nullptr);
    if (!here) {
        std::fprintf(stderr, "FAIL: the in-process demo could not be made\n");
        return 1;
    }
    here->setPaused(true);
    here->start();
    auto there = app::DemoLink::launch(program, publisherArguments(jsbsimRoot), 1.0, true, 120.0);
    if (!there) {
        std::fprintf(stderr, "FAIL: the demo publisher did not publish its world\n");
        return 1;
    }
    check(there->vehicles() == options.vehicles, "the published world has the demo's vehicles");

    // each vehicle's CG's place from its model's origin, as the viewer hangs its picture from it
    for (std::size_t i = 0; i < options.vehicles; ++i) {
        double a[3] = {}, b[3] = {};
        here->cgFromOriginBodyM(i, a);
        there->cgFromOriginBodyM(i, b);
        check(std::memcmp(a, b, sizeof a) == 0, "vehicle " + std::to_string(i) + "'s CG offset");
    }

    // the first state, then step by step
    const auto* a = next(*here);
    const auto* b = next(*there);
    if (!a || !b) {
        std::fprintf(stderr, "FAIL: no first snapshot\n");
        return 1;
    }
    same(*a, *b, options.vehicles, "the first state");
    int stepped = 0;
    for (; stepped < kSteps; ++stepped) {
        here->singleStep();
        there->singleStep();
        a = next(*here);
        b = next(*there);
        if (!a || !b) {
            check(false, "step " + std::to_string(stepped + 1) + " published no snapshot");
            break;
        }
        if (!same(*a, *b, options.vehicles, "step " + std::to_string(stepped + 1))) break;
    }
    here->stop();
    check(there->stop(), "the publisher stops on request");
    std::printf("%d of %d steps compared: %s\n", stepped, kSteps, failures ? "differences" : "identical to the bit");
    return failures ? 1 : 0;
}

int steps(const std::string& program, const std::string& jsbsimRoot) {
    constexpr int kQueued = 20;
    const auto options = demoOptions(jsbsimRoot);
    io::AssetResolver assets;
    auto here = app::DemoSimulation::create(options, assets, nullptr);
    if (!here) {
        std::fprintf(stderr, "FAIL: the in-process demo could not be made\n");
        return 1;
    }
    here->setPaused(true);
    here->start();
    auto there = app::DemoLink::launch(program, publisherArguments(jsbsimRoot), 1.0, true, 120.0);
    if (!there) {
        std::fprintf(stderr, "FAIL: the demo publisher did not publish its world\n");
        return 1;
    }
    const auto* first = next(*here);
    if (!first || !next(*there)) {
        std::fprintf(stderr, "FAIL: no first snapshot\n");
        return 1;
    }

    // 20 requests at once to the publisher; the in-process demo stepped 20 times, one at a time
    for (int i = 0; i < kQueued; ++i) there->singleStep();
    sim::SnapshotBatch expected;
    for (int i = 0; i < kQueued; ++i) {
        here->singleStep();
        const auto* a = next(*here);
        if (!a) {
            std::fprintf(stderr, "FAIL: the in-process demo did not step\n");
            return 1;
        }
        expected = *a;
    }
    sim::SnapshotBatch published;
    int snapshots = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    while (std::chrono::steady_clock::now() < deadline && !(snapshots && published.simTime >= expected.simTime)) {
        if (const auto* b = there->acquire()) {
            published = *b;
            ++snapshots;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(snapshots > 0, "the queued requests took no step");
    if (snapshots > 0) same(expected, published, options.vehicles, std::to_string(kQueued) + " queued requests");
    double last = published.simTime;
    const int extra = drain(*there, 1.0, last);
    check(extra == 0, std::to_string(kQueued) + " queued requests took " + std::to_string(extra) + " step(s) more");
    std::printf("%d requests made at once: %d snapshot(s), the in-process demo's %d steps %s\n", kQueued, snapshots,
                kQueued, failures ? "NOT matched" : "to the bit, and no more");

    // a resume drops the requests not yet taken: the pause after it holds
    for (int i = 0; i < 1000; ++i) there->singleStep();
    double held = published.simTime;
    const int taken = drain(*there, 0.3, held);
    check(taken > 0, "the pending requests took no step while held");
    there->setPaused(false);
    double running = held;
    drain(*there, 0.3, running);
    check(running > held, "the resumed demo did not run");
    there->setPaused(true);
    double settled = running;
    drain(*there, 0.5, settled);
    double after = settled;
    const int afterPause = drain(*there, 1.0, after);
    check(afterPause == 0, "the pause did not hold with requests pending before the resume: " +
                               std::to_string(afterPause) + " step(s) more");
    std::printf("1000 requests, resumed after %d were taken: the pause after the resume holds%s\n", taken,
                afterPause ? " NOT" : "");

    // a request after that is still taken, once
    there->singleStep();
    double stepped = after;
    const auto* one = next(*there, 10.0);
    if (one) stepped = one->simTime;
    check(one && stepped > after, "a request after the pause was not taken");
    double later = stepped;
    const int more = drain(*there, 0.5, later);
    check(more == 0, "one request after the pause took " + std::to_string(more + 1) + " steps");

    // and the publisher stops on request with thousands pending
    for (int i = 0; i < 100000; ++i) there->singleStep();
    idle(0.1);
    const auto stopping = std::chrono::steady_clock::now();
    check(there->stop(), "the publisher stops on request with requests pending");
    const double stopSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - stopping).count();
    std::printf("stopped with 100000 requests pending in %.2f s\n", stopSeconds);
    here->stop();
    return failures ? 1 : 0;
}

#ifdef _WIN32
long processHandles() {
    DWORD count = 0;
    GetProcessHandleCount(GetCurrentProcess(), &count);
    return static_cast<long>(count);
}
#endif

int lifecycle(const std::string& program, const std::string& jsbsimRoot) {
    // the publisher refuses an unknown option and exits: the start fails, as when its world never comes
    const auto failedStart = [&] { return !app::DemoLink::launch(program, {"--no-such-option"}, 1.0, true, 30.0); };
    const auto startAndStop = [&] {
        auto link = app::DemoLink::launch(program, publisherArguments(jsbsimRoot), 1.0, true, 120.0);
        const bool ok = link && link->stop();
        link.reset();
        return ok;
    };
    // once each first: whatever the first of either opens for good is not counted
    check(failedStart(), "a publisher given an unknown option did not fail to start");
    check(startAndStop(), "the publisher did not start and stop");
#ifdef _WIN32
    const long before = processHandles();
#endif
    for (int i = 0; i < 3; ++i) check(failedStart(), "a publisher given an unknown option did not fail to start");
    check(startAndStop(), "the publisher did not start and stop");
#ifdef _WIN32
    const long after = processHandles();
    check(after == before, "process handles: " + std::to_string(before) + " before three failed starts and a start and stop, " +
                               std::to_string(after) + " after");
    std::printf("three failed starts and a start and stop: process handles %ld before, %ld after\n", before, after);
#else
    std::printf("three failed starts and a start and stop\n");
#endif
    return failures ? 1 : 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <flightsim-demo> <jsbsim root> [parity|steps|lifecycle]\n", argv[0]);
        return 2;
    }
    const std::string program = argv[1], jsbsimRoot = argv[2], mode = argc > 3 ? argv[3] : "parity";
    if (mode == "parity") return parity(program, jsbsimRoot);
    if (mode == "steps") return steps(program, jsbsimRoot);
    if (mode == "lifecycle") return lifecycle(program, jsbsimRoot);
    std::fprintf(stderr, "unknown mode %s\n", mode.c_str());
    return 2;
}
