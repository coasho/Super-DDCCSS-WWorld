#pragma once

// The demo publisher's control block (ARCH-08, HC-16). `flightsim-demo` (applications/demo) runs the demo's simulation
// in a process of its own and publishes it on the read-only world stream (WorldLayout.h) like any world. The viewer
// that launched it drives the simulation's thread through this block, as it drove the in-process demo: pause, single
// step, time factor and quit. The world stream itself stays read-only; this is a separate, small named shared-memory
// segment beside it (demoControlName). The publisher writes one value back: its simulation throughput.
//
// Single steps are counted (HC-28): each increment of stepRequests is one agent step, taken while the simulation is
// held, one at a time and as fast as the simulation's pace allows - 20 requests made at once are 20 steps. A step
// starts once the held simulation has settled (a tenth of a second, at least one and a half paced periods), so a
// step it was taking as it was held is not counted as one. A resume drops the requests not yet taken: running
// supersedes them, and the next hold starts from none (a step already handed to the simulation may still be taken
// as that hold begins, before it settles). Quit abandons them. The viewer writes `paused` before it adds
// a request and both with release ordering; the publisher reads the count first, acquiring, so a request made after a
// pause is read with that pause.

#include <atomic>
#include <cstdint>
#include <string>

namespace fsim::ipc {

constexpr std::uint32_t kDemoControlMagic = 0x4C434446u; ///< "FDCL"
constexpr std::uint32_t kDemoControlVersion = 1;

/// Laid out in shared memory, created and initialised by the viewer before it launches the publisher (zero-filled on
/// creation). Every field is lock-free; the publisher reads the commands each time it looks for a new snapshot.
struct DemoControl {
    std::uint32_t magic = 0;
    std::uint32_t version = 0;
    std::atomic<std::uint32_t> paused{0};          ///< viewer -> publisher: hold the simulation
    std::atomic<std::uint32_t> quit{0};            ///< viewer -> publisher: stop and exit
    std::atomic<std::uint64_t> stepRequests{0};    ///< viewer -> publisher: one single step per increment, while held
    std::atomic<double> timeFactor{1.0};           ///< viewer -> publisher: simulated seconds per wall second
    std::atomic<double> throughput{0.0};           ///< publisher -> viewer: the simulation's vehicle steps per second
};

static_assert(std::atomic<std::uint64_t>::is_always_lock_free && std::atomic<double>::is_always_lock_free,
              "the demo control block's fields must be lock-free in shared memory");

/// The control block's segment name for a demo world.
inline std::string demoControlName(const std::string& world) { return world + ".control"; }

} // namespace fsim::ipc
