#include "render/GpuTimer.h"

#include <algorithm>
#include <cstdio>

namespace fsim::render {

struct GpuTimer::State : public vsg::Inherit<vsg::Object, GpuTimer::State> {
    std::array<vsg::ref_ptr<vsg::QueryPool>, kRing> pools;
    std::array<bool, kRing> used{};
    std::uint32_t count = 0;   ///< marks
    std::size_t current = 0;   ///< the pool this frame records into
};

namespace {

class Reset : public vsg::Inherit<vsg::Command, Reset> {
public:
    explicit Reset(vsg::ref_ptr<GpuTimer::State> s) : state(std::move(s)) {}
    vsg::ref_ptr<GpuTimer::State> state;
    void compile(vsg::Context& context) override {
        for (auto& p : state->pools) {
            if (!p) {
                p = vsg::QueryPool::create();
                p->queryType = VK_QUERY_TYPE_TIMESTAMP;
                p->queryCount = std::max(state->count, 1u);
            }
            p->compile(context);
        }
    }
    void record(vsg::CommandBuffer& commandBuffer) const override {
        const auto& p = state->pools[state->current];
        if (p && p->vk()) vkCmdResetQueryPool(commandBuffer, p->vk(), 0, p->queryCount);
        state->used[state->current] = true;
    }
};

class Mark : public vsg::Inherit<vsg::Command, Mark> {
public:
    Mark(vsg::ref_ptr<GpuTimer::State> s, std::uint32_t i) : state(std::move(s)), index(i) {}
    vsg::ref_ptr<GpuTimer::State> state;
    std::uint32_t index;
    void record(vsg::CommandBuffer& commandBuffer) const override {
        const auto& p = state->pools[state->current];
        if (p && p->vk() && index < p->queryCount) vkCmdWriteTimestamp(commandBuffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, p->vk(), index);
    }
};

} // namespace

GpuTimer::GpuTimer() = default;
GpuTimer::~GpuTimer() = default;

GpuTimer::State& GpuTimer::state() {
    if (!state_) state_ = State::create();
    return *state_;
}

vsg::ref_ptr<vsg::Command> GpuTimer::reset() {
    state();
    return Reset::create(state_);
}

vsg::ref_ptr<vsg::Command> GpuTimer::mark(const std::string& label) {
    state();
    labels_.push_back(label);
    if (marks_ > 0 && std::find(order_.begin(), order_.end(), label) == order_.end()) order_.push_back(label);
    state_->count = marks_ + 1;
    return Mark::create(state_, marks_++);
}

void GpuTimer::frameDone(vsg::Device& device, float timestampPeriodNs) {
    (void)device;
    if (!state_) return;
    // the pool recorded kRing - 1 frames ago is the next one to record into: read it first
    const std::size_t next = (state_->current + 1) % kRing;
    const auto& p = state_->pools[next];
    if (p && p->vk() && state_->used[next] && marks_ > 1) {
        std::vector<std::uint64_t> stamps(marks_);
        if (p->getResults(stamps, 0, VK_QUERY_RESULT_64_BIT) == VK_SUCCESS) {
            for (std::uint32_t i = 1; i < marks_; ++i) {
                const double ms = static_cast<double>(stamps[i] - stamps[i - 1]) * static_cast<double>(timestampPeriodNs) * 1e-6;
                if (stamps[i] >= stamps[i - 1] && ms < 1000.0) sumMs_[labels_[i]] += ms;
            }
            ++frames_;
        }
    }
    state_->current = next;
}

std::string GpuTimer::report() {
    std::string s = "gpu";
    char buf[96];
    double total = 0.0;
    for (const auto& label : order_) {
        const double ms = frames_ ? sumMs_[label] / static_cast<double>(frames_) : 0.0;
        total += ms;
        std::snprintf(buf, sizeof buf, " | %s %.3f", label.c_str(), ms);
        s += buf;
    }
    std::snprintf(buf, sizeof buf, " | total %.3f ms (%llu frames)", total, static_cast<unsigned long long>(frames_));
    s += buf;
    sumMs_.clear();
    frames_ = 0;
    return s;
}

} // namespace fsim::render
