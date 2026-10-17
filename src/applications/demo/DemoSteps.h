#pragma once

// The demo publisher's single steps (contracts/world_stream/DemoControl.h; ARCH-08 HC-28, HC-30): every request the
// viewer makes is one agent step, taken while the simulation is held, one at a time; none is lost or merged.
//
// The runner's single step is a latch - set any number of times before its thread takes it, it is one step - and a
// step it takes publishes a snapshot like any other. So the requested steps are told from the simulation's own
// exactly, by the runner's states rather than by timing:
// - a hold pauses the runner, then stops it - joining its thread, so the step it may have been taking is published -
//   and starts it again, held. What it published until then is the next pass's, and is not a requested step;
// - from then on a held runner steps only when handed one, and a step is handed only when the one before has been
//   published: every snapshot after is the handed step's;
// - a resume waits for the step it handed to be published, so the latch is never left set while the runner runs;
// - requests wait while the simulation runs, and through a resume, for the next hold.

#include <cstdint>

namespace fsim::app {

/// `Runner` holds the demo's simulation thread (DemoSimulation; in a test, a model of it): setPaused, singleStep (its
/// one-step latch), stop (its thread joined: no step is then under way) and start.
template <typename Runner>
class DemoSteps {
public:
    /// `requested`: the control block's count as the publisher starts - requests made before are not its.
    DemoSteps(Runner& runner, std::uint64_t requested) : runner_(runner), taken_(requested) {}

    /// One pass of the publisher's loop, after it has published what the runner published since the last pass:
    /// `fresh`, whether there was anything; `requested`, the control block's count (read before `held`); `held`,
    /// whether the viewer holds the simulation.
    void update(bool fresh, std::uint64_t requested, bool held) {
        if (fresh && quiet_) {
            if (handed_) {
                handed_ = false; // the step handed over: all a quiet hold does
                ++taken_;
            } else {
                ++unexpected_;
            }
        }
        if (held && !held_) {
            runner_.setPaused(true);
            runner_.stop(); // (the step it was taking, if any, is published when this returns)
            runner_.start();
            held_ = true;
            quiet_ = false;
            return;
        }
        if (held_ && !quiet_) quiet_ = true; // the old thread's last snapshot was this pass's `fresh`
        if (!held && held_ && !handed_) {
            runner_.setPaused(false);
            held_ = quiet_ = false;
            return;
        }
        if (held && quiet_ && !handed_ && taken_ < requested) {
            runner_.singleStep();
            handed_ = true;
        }
    }

    /// The requests taken: each a published step.
    std::uint64_t taken() const noexcept { return taken_; }
    /// Snapshots a quiet hold published with no step handed: none, if the runner is what it claims.
    std::uint64_t unexpected() const noexcept { return unexpected_; }

private:
    Runner& runner_;
    std::uint64_t taken_;
    std::uint64_t unexpected_ = 0;
    bool held_ = false;   ///< the runner is held (the viewer holds it, or a resume waits for a handed step)
    bool quiet_ = false;  ///< ... and what its thread published before the hold has been passed
    bool handed_ = false; ///< a step handed over and not yet published
};

} // namespace fsim::app
