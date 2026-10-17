#pragma once

#include <vsg/all.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace fsim::render {

/// The GPU's time between marks in a frame's commands (Vulkan timestamp queries), averaged over frames: what each
/// pass costs on the GPU, apart from the CPU's side of the frame. Marks are recorded in the order they are made; each
/// closes the section since the one before it, under its label (sections with the same label add up). The queries
/// go round kRing pools, a frame's read kRing - 1 frames later, so reading them never waits for the GPU.
class GpuTimer {
public:
    GpuTimer();
    ~GpuTimer();
    GpuTimer(const GpuTimer&) = delete;
    GpuTimer& operator=(const GpuTimer&) = delete;

    /// Recorded first in the frame, outside any render pass: the frame's pool made ready.
    vsg::ref_ptr<vsg::Command> reset();
    /// A mark, closing the section since the mark before it under `label` (the first mark's label is not used).
    vsg::ref_ptr<vsg::Command> mark(const std::string& label);

    /// After each frame is submitted: the oldest frame's times read (when the GPU has them) and its pool reused next.
    /// It hands them to addFrame, or tells addMissing that the frame's were not read.
    void frameDone(vsg::Device& device, float timestampPeriodNs);
    /// A frame's timestamps (one a mark, in the GPU's ticks of `tickMs` milliseconds), in the order the frames were
    /// recorded. A frame whose stamps do not run forward through its marks (or span a second or more) is not used:
    /// it counts as unread, as addMissing's frame does.
    void addFrame(const std::vector<std::uint64_t>& stamps, double tickMs);
    /// A frame whose timestamps were not read. The next frame's gap is then not known: the GPU's time between two
    /// frames is only measured between frames read one after the other.
    void addMissing();
    /// The sections' mean GPU milliseconds since the last report, in the order first marked. Then, on a line of its
    /// own ("gpu worst |"), the frame with the largest total of sections: its sections, and the GPU's gap before it -
    /// from the previous frame's last mark to its first, "n/a" when the previous frame was not read. Then, apart from
    /// that frame, the largest gap between any two frames read one after the other, and the frames not read. A gap is
    /// time outside the marked passes: the GPU idle, waiting to present, or doing other work; the timestamps do not
    /// say which. Then cleared (a gap across the report's boundary still counts in the next one).
    std::string report();
    bool empty() const noexcept { return marks_ == 0; }

    static constexpr std::size_t kRing = 4;

    struct State;

private:
    vsg::ref_ptr<State> state_;
    State& state();
    std::uint32_t marks_ = 0;
    std::vector<std::string> labels_; ///< per mark: the section it closes
    std::vector<std::string> order_;  ///< the labels as first met
    std::map<std::string, double> sumMs_;
    std::uint64_t frames_ = 0;
    std::map<std::string, double> worstMs_; ///< the frame with the largest total since the last report
    double worstTotal_ = 0.0;
    double worstGap_ = -1.0;  ///< the GPU's gap before that frame (ms; < 0: not known)
    double maxGap_ = -1.0;    ///< the largest gap between consecutive frames read (ms; < 0: none measured)
    std::uint64_t unread_ = 0; ///< frames not read since the last report
    std::uint64_t lastEnd_ = 0; ///< the frame read just before: its last mark (GPU ticks; 0: none, or not read)
};

} // namespace fsim::render
