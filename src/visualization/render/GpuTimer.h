#pragma once

#include <vsg/all.h>

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
    void frameDone(vsg::Device& device, float timestampPeriodNs);
    /// The sections' mean GPU milliseconds since the last report, in the order first marked; then, on a line of its
    /// own ("gpu worst |"), the slowest frame's sections, the GPU's gap before that frame (from the previous frame's
    /// last mark to its first: the GPU idle or busy with work outside these marks, another device's or a transfer's)
    /// and the largest such gap; then cleared.
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
    std::map<std::string, double> worstMs_; ///< the slowest frame's sections since the last report
    double worstTotal_ = 0.0, worstGap_ = 0.0, maxGap_ = 0.0;
    std::uint64_t lastEnd_ = 0;             ///< the previous frame's last mark (GPU ticks)
};

} // namespace fsim::render
