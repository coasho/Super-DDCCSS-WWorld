#pragma once

// The demo publisher's control block (ARCH-08, HC-16). `flightsim-demo` (applications/demo) runs the demo's simulation
// in a process of its own and publishes it on the read-only world stream (WorldLayout.h) like any world. The viewer
// that launched it drives the simulation's thread through this block, as it drove the in-process demo: pause, single
// step, time factor and quit. The world stream itself stays read-only; this is a separate, small named shared-memory
// segment beside it (demoControlName). The publisher writes two values back: the single-step requests it has taken,
// and its simulation throughput.
//
// Single steps are counted (HC-28, HC-30): each increment of stepRequests is one agent step, taken while the
// simulation is held - one at a time, as fast as the simulation's pace allows. None is lost: 20 requests made at once
// are 20 steps; a request made while the simulation runs waits for the next hold, as does one a resume left pending.
// Quit abandons the requests not yet taken. The steps are told apart from the simulation's own exactly, not by
// timing: as a hold begins the publisher waits for the simulation's thread to finish the step it may be taking, and
// it resumes only when the step it handed over is done. stepsTaken counts the requests taken, so a viewer knows when
// its steps are. The viewer writes `paused` before it adds a request, both with release ordering; the publisher reads
// the count first, acquiring, so a request made after a pause is read with that pause.

#include <atomic>
#include <cstdint>
#include <string>

namespace fsim::ipc {

constexpr std::uint32_t kDemoControlMagic = 0x4C434446u; ///< "FDCL"
constexpr std::uint32_t kDemoControlVersion = 2;          ///< (2: stepsTaken)

/// Laid out in shared memory, created and initialised by the viewer before it launches the publisher (zero-filled on
/// creation). Every field is lock-free; the publisher reads the commands each time it looks for a new snapshot.
struct DemoControl {
    std::uint32_t magic = 0;
    std::uint32_t version = 0;
    std::atomic<std::uint32_t> paused{0};          ///< viewer -> publisher: hold the simulation
    std::atomic<std::uint32_t> quit{0};            ///< viewer -> publisher: stop and exit
    std::atomic<std::uint64_t> stepRequests{0};    ///< viewer -> publisher: one single step per increment, taken while held
    std::atomic<double> timeFactor{1.0};           ///< viewer -> publisher: simulated seconds per wall second
    std::atomic<double> throughput{0.0};           ///< publisher -> viewer: the simulation's vehicle steps per second
    std::atomic<std::uint64_t> stepsTaken{0};      ///< publisher -> viewer: the requests taken, each a published step
};

static_assert(std::atomic<std::uint64_t>::is_always_lock_free && std::atomic<double>::is_always_lock_free,
              "the demo control block's fields must be lock-free in shared memory");

/// The control block's segment name for a demo world.
inline std::string demoControlName(const std::string& world) { return world + ".control"; }

} // namespace fsim::ipc
