// The demo publisher (ARCH-08, HC-16): flightsim-demo flies exactly the demo the viewer ran in-process before. The
// in-process DemoSimulation and the publisher - launched and driven by the viewer's own client, DemoLink, through the
// demo control block - start from the same options held paused, and are stepped together one agent step at a time.
// After every step each vehicle's state, as the viewer reads it from the world stream, is the in-process state to
// the bit. Then the publisher stops on request.
//
// Usage: fsim_demo_publisher_test <flightsim-demo> <jsbsim root>

#include "applications/demo/DemoSimulation.h"
#include "applications/viewer/DemoLink.h"
#include "foundation/io/AssetResolver.h"
#include "foundation/platform/Clock.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

using namespace fsim;

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
        ++failures;
    }
}

/// The next snapshot `source` hands out, waiting for it up to 30 s.
template <typename Source>
const sim::SnapshotBatch* next(Source& source) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (std::chrono::steady_clock::now() < deadline) {
        if (const auto* batch = source.acquire()) return batch;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return nullptr;
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

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <flightsim-demo> <jsbsim root>\n", argv[0]);
        return 2;
    }
    const std::string program = argv[1], jsbsimRoot = argv[2];
    constexpr int kSteps = 240; // four simulated seconds at the demo's 60 Hz agent rate

    // the viewer's demo options, flat ground (no map set needed); the publisher gets them as the viewer passes them
    app::DemoOptions options;
    options.aircraft = "c172x";
    options.jsbsimRoot = jsbsimRoot;
    options.vehicles = 4;
    options.workers = 2;
    options.seed = 7;
    io::AssetResolver assets;
    auto here = app::DemoSimulation::create(options, assets, nullptr);
    if (!here) {
        std::fprintf(stderr, "FAIL: the in-process demo could not be made\n");
        return 1;
    }
    here->setPaused(true);
    here->start();
    const std::vector<std::string> args{"--aircraft", options.aircraft, "--jsbsim-root", jsbsimRoot, "--vehicles", "4",
                                        "--workers", "2", "--seed", "7", "--elevation", "none"};
    auto there = app::DemoLink::launch(program, args, 1.0, true, 120.0);
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
