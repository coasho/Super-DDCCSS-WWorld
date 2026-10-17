#pragma once

#include <vsg/all.h>

#include <cstdint>

namespace fsim::world {

/// A draw's shading rate (VK_KHR_fragment_shading_rate's per-draw rate, a dynamic state of its pipeline): `size` pixels
/// a fragment each way - the contrails' and the detonations' marches, soft volumes shaded coarser on a large window.
class ShadingRate : public vsg::Inherit<vsg::Command, ShadingRate> {
public:
    std::uint32_t size = 1;
    void record(vsg::CommandBuffer& commandBuffer) const override {
        if (!set_) set_ = reinterpret_cast<PFN_vkCmdSetFragmentShadingRateKHR>(vkGetDeviceProcAddr(commandBuffer.getDevice()->vk(), "vkCmdSetFragmentShadingRateKHR"));
        if (!set_) return;
        const VkExtent2D extent{size, size};
        const VkFragmentShadingRateCombinerOpKHR keep[2] = {VK_FRAGMENT_SHADING_RATE_COMBINER_OP_KEEP_KHR, VK_FRAGMENT_SHADING_RATE_COMBINER_OP_KEEP_KHR};
        set_(commandBuffer.vk(), &extent, keep);
    }

private:
    mutable PFN_vkCmdSetFragmentShadingRateKHR set_ = nullptr;
};

} // namespace fsim::world
