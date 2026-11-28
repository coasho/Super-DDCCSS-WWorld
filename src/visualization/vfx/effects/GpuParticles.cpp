#include "visualization/vfx/effects/GpuParticles.h"

#include "foundation/core/Log.h"
#include "visualization/scene/Clouds.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace fsim::world {

namespace {

/// Resets GPU-local particle memory to dead (-1.0f age) on initial frame or seek/clear(),
/// and resets the two VkDrawIndirectCommand instance counts inline every active frame.
class PrepareGpuParticleBuffers : public vsg::Inherit<vsg::Command, PrepareGpuParticleBuffers> {
public:
    vsg::ref_ptr<vsg::BufferInfo> poolInfo;
    vsg::ref_ptr<vsg::BufferInfo> indirectInfo;
    std::shared_ptr<std::atomic<bool>> resetFlag;
    VkDeviceSize indirectBytes = 48;

    void record(vsg::CommandBuffer& commandBuffer) const override {
        if (!poolInfo || !poolInfo->buffer || !indirectInfo || !indirectInfo->buffer) return;
        const VkBuffer poolVk = poolInfo->buffer->vk(commandBuffer.deviceID);
        const VkBuffer indVk = indirectInfo->buffer->vk(commandBuffer.deviceID);
        if (poolVk == VK_NULL_HANDLE || indVk == VK_NULL_HANDLE) return;

        // Wait for any previous frame's indirect draw or compute reads before updating buffers.
        VkMemoryBarrier preBarrier{};
        preBarrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        preBarrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_INDIRECT_COMMAND_READ_BIT;
        preBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        vkCmdPipelineBarrier(commandBuffer,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT,
                             VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &preBarrier, 0, nullptr, 0, nullptr);

        if (resetFlag && resetFlag->exchange(false, std::memory_order_acq_rel)) {
            // 0xBF800000 = IEEE-754 -1.0f -> marks every particle slot dead (posAge.w < 0.0).
            vkCmdFillBuffer(commandBuffer, poolVk, 0, VK_WHOLE_SIZE, 0xBF800000u);
            vkCmdFillBuffer(commandBuffer, indVk, 0, indirectBytes, 0u);
        }

        // Reset both VkDrawIndirectCommand structs while preserving persistent writeCursor at offset 32.
        const std::uint32_t resetCmds[8] = {
            6u, 0u, 0u, 0u,      // Alpha bin: vertexCount=6, instanceCount=0, firstVertex=0, firstInstance=0
            6u, 0u, 0u, 32768u,  // Additive bin: vertexCount=6, instanceCount=0, firstVertex=0, firstInstance=32768
        };
        vkCmdUpdateBuffer(commandBuffer, indVk, 0, sizeof resetCmds, resetCmds);

        VkMemoryBarrier postBarrier{};
        postBarrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        postBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        postBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             0, 1, &postBarrier, 0, nullptr, 0, nullptr);
    }
};

std::array<vsg::vec4, 6> extractEyeRelativeFrustum(const vsg::dmat4& projection, const vsg::dmat4& viewMatrix,
                                                   const vsg::dvec3& eyeEcef) noexcept {
    // Eye-relative view matrix shifts world origin to eyeEcef in double precision before building clip planes.
    const vsg::dmat4 eyeView = viewMatrix * vsg::translate(eyeEcef);
    const vsg::dmat4 m = projection * eyeView;
    const auto row = [&](std::size_t r) { return vsg::dvec4(m[0][r], m[1][r], m[2][r], m[3][r]); };
    const vsg::dvec4 r0 = row(0), r1 = row(1), r2 = row(2), r3 = row(3);
    // Vulkan reverse-Z clip space: -w <= x <= w, -w <= y <= w, 0 <= z <= w.
    const std::array<vsg::dvec4, 6> raw = {
        r3 + r0, // Left
        r3 - r0, // Right
        r3 + r1, // Bottom
        r3 - r1, // Top
        r2,      // Far (z >= 0)
        r3 - r2, // Near (z <= w)
    };
    std::array<vsg::vec4, 6> out{};
    for (std::size_t i = 0; i < 6; ++i) {
        const double len = std::sqrt(raw[i].x * raw[i].x + raw[i].y * raw[i].y + raw[i].z * raw[i].z);
        if (len > 1e-9) {
            out[i] = vsg::vec4(static_cast<float>(raw[i].x / len), static_cast<float>(raw[i].y / len),
                               static_cast<float>(raw[i].z / len), static_cast<float>(raw[i].w / len));
        } else {
            // Infinite reverse-Z far plane has zero normal and positive w: always passes.
            out[i] = vsg::vec4(0.0f, 0.0f, 0.0f, 1.0e9f);
        }
    }
    return out;
}

vsg::dvec3 orthoVec(const vsg::dvec3& n) noexcept {
    const vsg::dvec3 a = (std::abs(n.z) < 0.9) ? vsg::dvec3(0.0, 0.0, 1.0) : vsg::dvec3(1.0, 0.0, 0.0);
    return vsg::normalize(vsg::cross(n, a));
}

} // namespace

GpuParticles::GpuParticles(vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders)
    : shaders_(shaders) {
    (void)options;
    computeRoot_ = vsg::Group::create();
    drawRoot_ = vsg::Group::create();
    computeSwitch_ = vsg::Switch::create();
    drawSwitch_ = vsg::Switch::create();
    computeRoot_->addChild(computeSwitch_);
    drawRoot_->addChild(drawSwitch_);
    resetFlag_ = std::make_shared<std::atomic<bool>>(true);
    pendingSpawns_.reserve(kMaxSpawnsPerFrame);

    if (shaders_.empty()) {
        LOG_WARN("world") << "no GPU particle engine: shader directory missing";
        return;
    }
    valid_ = true;
}

bool GpuParticles::build(vsg::ref_ptr<vsg::Device> device, vsg::ref_ptr<vsg::ImageInfo> sceneDepth,
                         vsg::ref_ptr<vsg::ImageInfo> flowFront) {
    if (!valid_ || built_ || !device || !sceneDepth || shaders_.empty()) return built_;

    auto spawnComp = loadCloudShader(shaders_, "combat_particle_spawn.comp", VK_SHADER_STAGE_COMPUTE_BIT);
    auto simComp = loadCloudShader(shaders_, "combat_particle_sim.comp", VK_SHADER_STAGE_COMPUTE_BIT);
    auto partVert = loadCloudShader(shaders_, "combat_particle.vert", VK_SHADER_STAGE_VERTEX_BIT);
    auto partFrag = loadCloudShader(shaders_, "combat_particle.frag", VK_SHADER_STAGE_FRAGMENT_BIT);
    auto ribVert = loadCloudShader(shaders_, "combat_ribbon.vert", VK_SHADER_STAGE_VERTEX_BIT);
    auto ribFrag = loadCloudShader(shaders_, "combat_ribbon.frag", VK_SHADER_STAGE_FRAGMENT_BIT);

    vsg::ShaderStages spawnStages{spawnComp};
    vsg::ShaderStages simStages{simComp};
    vsg::ShaderStages partStages{partVert, partFrag};
    vsg::ShaderStages ribStages{ribVert, ribFrag};
    auto compiler = vsg::ShaderCompiler::create();
    if (!spawnComp || !simComp || !partVert || !partFrag || !ribVert || !ribFrag || !compiler->supported() ||
        !compiler->compile(spawnStages) || !compiler->compile(simStages) || !compiler->compile(partStages) ||
        !compiler->compile(ribStages)) {
        LOG_WARN("world") << "VIS-M13 GPU particle shaders failed to compile; combat VFX disabled";
        valid_ = false;
        return false;
    }

    // Host-visible dynamic staging & UBOs.
    spawnStaging_ = vsg::vec4Array::create(kMaxSpawnsPerFrame * 4u, vsg::vec4(0.0f, 0.0f, 0.0f, -1.0f));
    simUbo_ = vsg::vec4Array::create(12u, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    renderUbo_ = vsg::vec4Array::create(8u, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    ribbonStaging_ = vsg::vec4Array::create(kMaxRibbons * 4u, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    spawnStaging_->properties.dataVariance = vsg::DYNAMIC_DATA;
    simUbo_->properties.dataVariance = vsg::DYNAMIC_DATA;
    renderUbo_->properties.dataVariance = vsg::DYNAMIC_DATA;
    ribbonStaging_->properties.dataVariance = vsg::DYNAMIC_DATA;

    // Safe frame-0 warmup defaults (all frustum planes accept, Flow scissor disabled).
    simUbo_->set(0, vsg::vec4(0.0f, 0.0f, 0.0f, static_cast<float>(kMaxParticles)));
    simUbo_->set(2, vsg::vec4(0.0f, 0.0f, 1.0f, 9.80665f));
    for (std::uint32_t i = 0; i < 6; ++i) simUbo_->set(4u + i, vsg::vec4(0.0f, 0.0f, 0.0f, 1.0e9f));
    renderUbo_->set(0, vsg::vec4(0.0f, 0.0f, 1.0f, 1.0f));
    renderUbo_->set(3, vsg::vec4(0.0f, 0.0f, 1.0f, 0.0f));
    renderUbo_->set(6, vsg::vec4(0.0f, 0.0f, -1.0f, -1.0f));
    renderUbo_->set(7, vsg::vec4(2.0f, 0.0f, 0.0f, 0.0f));

    // Allocate GPU-local SSBOs & indirect command buffer up front so CollectResourceRequirements
    // and CompileTraversal always see bound valid VkBuffers across betweenPass and overScene.
    const VkDeviceSize poolBytes = static_cast<VkDeviceSize>(kMaxParticles) * 64u;
    const VkDeviceSize aliveBytes = static_cast<VkDeviceSize>(kMaxParticles) * 4u;
    const VkDeviceSize indirectBytes = 48u;
    auto poolBuf = vsg::createBufferAndMemory(
        device, poolBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_SHARING_MODE_EXCLUSIVE, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    auto aliveBuf = vsg::createBufferAndMemory(
        device, aliveBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_SHARING_MODE_EXCLUSIVE, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    auto indirectBuf = vsg::createBufferAndMemory(
        device, indirectBytes,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_SHARING_MODE_EXCLUSIVE, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (!poolBuf || !aliveBuf || !indirectBuf) {
        LOG_WARN("world") << "VIS-M13 failed to allocate device-local GPU particle buffers";
        valid_ = false;
        return false;
    }

    auto poolInfo = vsg::BufferInfo::create(poolBuf, 0, poolBytes);
    auto aliveInfo = vsg::BufferInfo::create(aliveBuf, 0, aliveBytes);
    auto indirectInfo = vsg::BufferInfo::create(indirectBuf, 0, indirectBytes);
    auto indirectAlphaInfo = vsg::BufferInfo::create(indirectBuf, 0, 16);
    auto indirectAddInfo = vsg::BufferInfo::create(indirectBuf, 16, 16);

    auto prep = PrepareGpuParticleBuffers::create();
    prep->poolInfo = poolInfo;
    prep->indirectInfo = indirectInfo;
    prep->resetFlag = resetFlag_;
    prep->indirectBytes = indirectBytes;

    // --- 1. Compute pass (betweenPass): Prepare -> Spawn -> Barrier -> Sim & Frustum Compact -> Barrier ---
    {
        const VkShaderStageFlags cs = VK_SHADER_STAGE_COMPUTE_BIT;
        auto spawnSetLayout = vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{
            {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {3, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, cs, nullptr}});
        auto spawnPipeLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{spawnSetLayout}, vsg::PushConstantRanges{});
        auto spawnSet = vsg::DescriptorSet::create(
            spawnSetLayout,
            vsg::Descriptors{
                vsg::DescriptorBuffer::create(vsg::BufferInfoList{poolInfo}, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                vsg::DescriptorBuffer::create(spawnStaging_, 1, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                vsg::DescriptorBuffer::create(vsg::BufferInfoList{indirectInfo}, 2, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                vsg::DescriptorBuffer::create(simUbo_, 3, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER)});

        auto spawnGroup = vsg::StateGroup::create();
        spawnGroup->add(vsg::BindComputePipeline::create(vsg::ComputePipeline::create(spawnPipeLayout, spawnComp)));
        spawnGroup->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_COMPUTE, spawnPipeLayout, 0, spawnSet));
        spawnDispatch_ = vsg::Dispatch::create(1, 1, 1);
        spawnGroup->addChild(spawnDispatch_);

        auto spawnBarrier = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0);
        spawnBarrier->memoryBarriers.push_back(vsg::MemoryBarrier::create(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT));

        auto spawnWrap = vsg::Group::create();
        spawnWrap->addChild(spawnGroup);
        spawnWrap->addChild(spawnBarrier);
        spawnSwitch_ = vsg::Switch::create();
        spawnSwitch_->addChild(true, spawnWrap); // ON until compiled()

        auto simSetLayout = vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{
            {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {3, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, cs, nullptr}});
        auto simPipeLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{simSetLayout}, vsg::PushConstantRanges{});
        auto simSet = vsg::DescriptorSet::create(
            simSetLayout,
            vsg::Descriptors{
                vsg::DescriptorBuffer::create(vsg::BufferInfoList{poolInfo}, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                vsg::DescriptorBuffer::create(vsg::BufferInfoList{aliveInfo}, 1, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                vsg::DescriptorBuffer::create(vsg::BufferInfoList{indirectInfo}, 2, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                vsg::DescriptorBuffer::create(simUbo_, 3, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER)});

        auto simGroup = vsg::StateGroup::create();
        simGroup->add(vsg::BindComputePipeline::create(vsg::ComputePipeline::create(simPipeLayout, simComp)));
        simGroup->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_COMPUTE, simPipeLayout, 0, simSet));
        simDispatch_ = vsg::Dispatch::create(kMaxParticles / 256u, 1, 1);
        simGroup->addChild(simDispatch_);

        auto doneBarrier = vsg::PipelineBarrier::create(
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT | VK_PIPELINE_STAGE_VERTEX_SHADER_BIT, 0);
        doneBarrier->memoryBarriers.push_back(vsg::MemoryBarrier::create(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_INDIRECT_COMMAND_READ_BIT | VK_ACCESS_SHADER_READ_BIT));

        auto pass = vsg::Group::create();
        pass->addChild(prep); // MUST compile first to allocate shared GPU-local BufferInfos!
        pass->addChild(spawnSwitch_);
        pass->addChild(simGroup);
        pass->addChild(doneBarrier);

        computeSwitch_->children.clear();
        computeSwitch_->addChild(true, pass); // ON until compiled()
    }

    // Common overScene translucent pipeline states (premultiplied alpha + reverse-Z depth in fragment shader).
    auto makeOverStates = []() {
        auto assembly = vsg::InputAssemblyState::create();
        assembly->topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        auto raster = vsg::RasterizationState::create();
        raster->cullMode = VK_CULL_MODE_NONE;
        auto depth = vsg::DepthStencilState::create();
        depth->depthTestEnable = VK_FALSE;
        depth->depthWriteEnable = VK_FALSE;
        auto blend = vsg::ColorBlendState::create();
        blend->attachments = {VkPipelineColorBlendAttachmentState{
            VK_TRUE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
            VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};
        return vsg::GraphicsPipelineStates{vsg::VertexInputState::create(), assembly, raster,
                                           vsg::MultisampleState::create(), blend, depth};
    };

    // --- 2. Indirect GPU particles graphics pass (overScene): alpha bin then additive bin ---
    {
        const VkShaderStageFlags vs = VK_SHADER_STAGE_VERTEX_BIT;
        const VkShaderStageFlags fs = VK_SHADER_STAGE_FRAGMENT_BIT;
        const VkShaderStageFlags both = vs | fs;
        auto setLayout = vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{
            {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, vs, nullptr},
            {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, vs, nullptr},
            {2, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, both, nullptr},
            {3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr},
            {4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr}});
        auto pipeLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout},
                                                      vsg::PushConstantRanges{{vs, 0, 128}});
        auto descSet = vsg::DescriptorSet::create(
            setLayout,
            vsg::Descriptors{
                vsg::DescriptorBuffer::create(vsg::BufferInfoList{poolInfo}, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                vsg::DescriptorBuffer::create(vsg::BufferInfoList{aliveInfo}, 1, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                vsg::DescriptorBuffer::create(renderUbo_, 2, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
                vsg::DescriptorImage::create(sceneDepth, 3, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                vsg::DescriptorImage::create(flowFront ? flowFront : sceneDepth, 4, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER)});

        auto sg = vsg::StateGroup::create();
        sg->add(vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(pipeLayout, partStages, makeOverStates())));
        sg->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipeLayout, 0, descSet));

        auto drawAlpha = vsg::DrawIndirect::create();
        drawAlpha->bufferInfo = indirectAlphaInfo;
        drawAlpha->drawCount = 1;
        drawAlpha->stride = 16;
        sg->addChild(drawAlpha);

        auto drawAdd = vsg::DrawIndirect::create();
        drawAdd->bufferInfo = indirectAddInfo;
        drawAdd->drawCount = 1;
        drawAdd->stride = 16;
        sg->addChild(drawAdd);

        drawTransform_ = vsg::MatrixTransform::create();
        drawTransform_->addChild(sg);
    }

    // --- 3. Direct exact-pose combat ribbons & cores (tracers, muzzle flashes, IR flare cores, shockwaves) ---
    {
        const VkShaderStageFlags vs = VK_SHADER_STAGE_VERTEX_BIT;
        const VkShaderStageFlags fs = VK_SHADER_STAGE_FRAGMENT_BIT;
        const VkShaderStageFlags both = vs | fs;
        auto setLayout = vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{
            {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, vs, nullptr},
            {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, both, nullptr},
            {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr},
            {3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr}});
        auto pipeLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout},
                                                      vsg::PushConstantRanges{{vs, 0, 128}});
        auto descSet = vsg::DescriptorSet::create(
            setLayout,
            vsg::Descriptors{
                vsg::DescriptorBuffer::create(ribbonStaging_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                vsg::DescriptorBuffer::create(renderUbo_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
                vsg::DescriptorImage::create(sceneDepth, 2, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                vsg::DescriptorImage::create(flowFront ? flowFront : sceneDepth, 3, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER)});

        auto sg = vsg::StateGroup::create();
        sg->add(vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(pipeLayout, ribStages, makeOverStates())));
        sg->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipeLayout, 0, descSet));
        ribbonDraw_ = vsg::Draw::create(6, 0, 0, 0);
        sg->addChild(ribbonDraw_);

        ribbonTransform_ = vsg::MatrixTransform::create();
        ribbonTransform_->addChild(sg);
        ribbonSwitch_ = vsg::Switch::create();
        ribbonSwitch_->addChild(true, ribbonTransform_); // ON until compiled()
    }

    auto overWrap = vsg::Group::create();
    overWrap->addChild(drawTransform_);
    overWrap->addChild(ribbonSwitch_);
    drawSwitch_->children.clear();
    drawSwitch_->addChild(true, overWrap); // ON until compiled()

    built_ = true;
    LOG_INFO("world") << "VIS-M13 GPU particle engine ready (" << kMaxParticles << " compute particles + "
                      << kMaxRibbons << " combat ribbons/cores)";
    return true;
}

void GpuParticles::compiled() {
    if (computeSwitch_) computeSwitch_->setAllChildren(false);
    if (spawnSwitch_) spawnSwitch_->setAllChildren(false);
    if (drawSwitch_) drawSwitch_->setAllChildren(false);
    if (ribbonSwitch_) ribbonSwitch_->setAllChildren(false);
    active_ = false;
}

void GpuParticles::clear() {
    pendingSpawns_.clear();
    seenEventSerials_.clear();
    decoyTrails_.clear();
    debrisEmbers_.clear();
    rotorStates_.clear();
    muzzleFlashes_.clear();
    shockwaves_.clear();
    hasOrigin_ = false;
    lastClock_ = -1e9;
    particleIdleUntil_ = -1e9;
    lastSpawnCount_ = 0;
    lastRibbonCount_ = 0;
    active_ = false;
    if (resetFlag_) resetFlag_->store(true, std::memory_order_release);
    if (ribbonDraw_) ribbonDraw_->instanceCount = 0;
    if (computeSwitch_) computeSwitch_->setAllChildren(false);
    if (spawnSwitch_) spawnSwitch_->setAllChildren(false);
    if (drawSwitch_) drawSwitch_->setAllChildren(false);
    if (ribbonSwitch_) ribbonSwitch_->setAllChildren(false);
}

float GpuParticles::nextRand() noexcept {
    // Deterministic Xorshift32 -> [0, 1).
    std::uint32_t x = rngState_;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    rngState_ = x ? x : 0x9E3779B9u;
    return static_cast<float>(rngState_ & 0x00FFFFFFu) / 16777216.0f;
}

vsg::dvec3 GpuParticles::randomUnitVec() noexcept {
    const double z = 2.0 * static_cast<double>(nextRand()) - 1.0;
    const double a = 6.283185307179586 * static_cast<double>(nextRand());
    const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    return vsg::dvec3(r * std::cos(a), r * std::sin(a), z);
}

void GpuParticles::queueParticle(const vsg::dvec3& posEcef, const vsg::dvec3& velEcef, float maxLifeS,
                                 const vsg::vec3& color, float baseRadiusM, float kind, float dragPerS, float extra) {
    if (pendingSpawns_.size() >= kMaxSpawnsPerFrame) return;
    const vsg::dvec3 rel = posEcef - originEcef_;
    GpuParticleCpu p{};
    p.posAge = vsg::vec4(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z), 0.0f);
    p.velLife = vsg::vec4(static_cast<float>(velEcef.x), static_cast<float>(velEcef.y), static_cast<float>(velEcef.z),
                          std::max(maxLifeS, 0.05f));
    p.colorSize = vsg::vec4(color.r, color.g, color.b, std::max(baseRadiusM, 0.03f));
    p.params = vsg::vec4(kind, std::max(dragPerS, 0.0f), nextRand(), extra);
    pendingSpawns_.push_back(p);
}

void GpuParticles::addDetonationShockwave(const vsg::dvec3& posEcef, const vsg::dvec3& velEcef, double tntKg, double simTime) {
    if (!built_ || !(tntKg > 0.0) || !std::isfinite(simTime)) return;
    // Deduplicate against simultaneous DetonationShockwave event from .vfx sidecar.
    for (const auto& existing : shockwaves_) {
        if (std::abs(existing.startTime - simTime) < 0.08 && vsg::length2(existing.posEcef - posEcef) < 2500.0) return;
    }
    const double cbrtW = std::cbrt(tntKg);
    ActiveShockwave sw;
    sw.posEcef = posEcef;
    sw.velEcef = velEcef * 0.25;
    sw.startTime = simTime;
    sw.duration = std::clamp(0.24 + 0.036 * cbrtW, 0.24, 0.48);
    sw.maxRadiusM = static_cast<float>(std::clamp(15.0 * cbrtW, 13.0, 88.0));
    sw.intensity = static_cast<float>(std::clamp(0.60 + 0.10 * cbrtW, 0.60, 1.05));
    sw.seed = nextRand();
    shockwaves_.push_back(sw);

    if (!hasOrigin_) {
        originEcef_ = posEcef;
        hasOrigin_ = true;
    }
    // High-speed supersonic spark burst + shock condensation mist ring!
    const int nSparks = std::clamp(static_cast<int>(18.0 + 4.0 * cbrtW), 18, 42);
    for (int i = 0; i < nSparks; ++i) {
        const vsg::dvec3 dir = randomUnitVec();
        const double spd = 48.0 + 110.0 * static_cast<double>(nextRand());
        queueParticle(posEcef + dir * 0.8, velEcef * 0.45 + dir * spd,
                      0.38f + 0.65f * nextRand(), vsg::vec3(1.0f, 0.72f, 0.22f),
                      0.042f + 0.032f * nextRand(), 1.0f, 1.9f + 0.9f * nextRand(), 3.6f);
    }
    const int nMist = 14;
    for (int i = 0; i < nMist; ++i) {
        const vsg::dvec3 dir = randomUnitVec();
        const double spd = 25.0 + 55.0 * static_cast<double>(nextRand());
        queueParticle(posEcef + dir * (0.25 * sw.maxRadiusM), velEcef * 0.2 + dir * spd,
                      static_cast<float>(sw.duration * (0.75 + 0.45 * nextRand())),
                      vsg::vec3(0.90f, 0.93f, 0.96f), 1.6f + 1.8f * nextRand(), 5.0f, 2.4f, 0.0f);
    }
    particleIdleUntil_ = std::max(particleIdleUntil_, simTime + 1.6);
}

void GpuParticles::addRotorEffects(Span<const sim::VehicleState> states, const std::vector<unsigned char>& alive,
                                   const VehicleVisuals& visuals, const ipc::TacticalOverlayFrameV1* tactical,
                                   double humidity, double simTime) {
    if (!built_ || !std::isfinite(simTime)) return;
    const double rh = std::isfinite(humidity) ? std::clamp(humidity, 0.0, 1.0) : 0.0;
    const std::size_t n = std::min(states.size(), alive.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (!alive[i] || states[i].crashed || !visuals.visible(i)) continue;
        const auto& shape = visuals.shape(i);
        if (shape.rotors.empty()) continue;

        const ipc::TacticalPropulsionExtWireV1* propExt = nullptr;
        if (tactical) {
            for (const auto& p : tactical->propulsion) {
                if (p.slot == i) {
                    propExt = &p;
                    break;
                }
            }
        }

        const sim::VehicleState& st = states[i];
        const vsg::dvec3 posEcef(st.positionEcef[0], st.positionEcef[1], st.positionEcef[2]);
        if (vsg::length2(posEcef) < 1e6) continue;
        const vsg::dvec3 zenith = vsg::normalize(posEcef);
        const double* R = st.rotationBodyToEcef;
        const auto bodyVecToEcef = [&](const vsg::dvec3& b) {
            return vsg::dvec3(R[0] * b.x + R[1] * b.y + R[2] * b.z,
                              R[3] * b.x + R[4] * b.y + R[5] * b.z,
                              R[6] * b.x + R[7] * b.y + R[8] * b.z);
        };
        const vsg::dvec3 velEcef = bodyVecToEcef(vsg::dvec3(st.velocityBodyMs[0], st.velocityBodyMs[1], st.velocityBodyMs[2]));
        const double agl = std::isfinite(st.altitudeAglM) ? st.altitudeAglM : 1000.0;
        const double tas = std::isfinite(st.airspeedTrueMs) ? st.airspeedTrueMs : 0.0;
        const bool overWater = propExt && ((propExt->flags & ipc::kTactPropOverWater) != 0u);

        for (std::size_t rIdx = 0; rIdx < shape.rotors.size(); ++rIdx) {
            const auto& r = shape.rotors[rIdx];
            if (!(r.radiusM > 0.25)) continue;
            double rpm = 0.0;
            if (r.engine >= 0 && r.engine < st.engineCount && r.engine < sim::VehicleState::kMaxEngines && std::abs(st.engineRpm[r.engine]) > 1.0)
                rpm = st.engineRpm[r.engine];
            else if (propExt && propExt->rotorRpm > 1.0f)
                rpm = static_cast<double>(propExt->rotorRpm);
            else if (st.engineCount > 0)
                rpm = st.engineRpm[0];
            rpm = std::abs(rpm);
            if (!std::isfinite(rpm) || rpm < 40.0) continue;

            const std::uint64_t key = (static_cast<std::uint64_t>(i) << 8u) | static_cast<std::uint64_t>(rIdx & 0xFFu);
            auto& rv = rotorStates_[key];
            if (simTime < rv.lastEmitTime - 0.05 || simTime > rv.lastEmitTime + 3.0) {
                rv.lastEmitTime = simTime - 0.035;
            }
            const double elapsed = simTime - rv.lastEmitTime;
            if (elapsed < 0.032) continue;
            rv.lastEmitTime = simTime;
            rv.phaseRad = std::fmod(rv.phaseRad + (rpm / 60.0) * 6.283185307179586 * std::min(elapsed, 0.1), 6.283185307179586);

            const vsg::dvec3 hubEcef = posEcef + bodyVecToEcef(r.hub);
            const vsg::dvec3 axisEcef = vsg::normalize(bodyVecToEcef(r.axis));
            const bool isLiftRotor = std::abs(r.axis.z) > 0.7;

            if (!hasOrigin_) {
                originEcef_ = hubEcef;
                hasOrigin_ = true;
            }

            // 1. Subtle ground-hugging rotor downwash dust / sea-spray veil near terrain or water (AGL < 2.2 R).
            // Uses soft translucent mist puffs (kind 5, extra=1) hugging the ground rather than opaque discrete clumps.
            if (isLiftRotor && agl > 0.2 && agl < 2.2 * r.radiusM && tas < 38.0) {
                const double gf = std::pow(std::clamp(1.0 - agl / (2.2 * r.radiusM), 0.0, 1.0), 1.45);
                const vsg::dvec3 groundCenter = hubEcef - zenith * std::max(agl - 0.20, 0.0);
                const vsg::dvec3 hU = orthoVec(zenith);
                const vsg::dvec3 hV = vsg::cross(zenith, hU);
                const vsg::vec3 ringCol = overWater ? vsg::vec3(0.84f, 0.90f, 0.94f) : vsg::vec3(0.56f, 0.49f, 0.39f);
                const int nRing = std::clamp(static_cast<int>(4.0 + 4.0 * gf), 4, 8);
                for (int k = 0; k < nRing; ++k) {
                    const double ang = 6.283185307179586 * (static_cast<double>(k) + static_cast<double>(nextRand())) / static_cast<double>(nRing);
                    const vsg::dvec3 radial = hU * std::cos(ang) + hV * std::sin(ang);
                    const double startR = r.radiusM * (0.55 + 0.55 * static_cast<double>(nextRand()));
                    const double outSpd = (5.5 + 11.0 * gf) * (0.80 + 0.40 * static_cast<double>(nextRand()));
                    const double liftSpd = (0.35 + 0.95 * gf) * (0.6 + 0.8 * static_cast<double>(nextRand()));
                    queueParticle(groundCenter + radial * startR + zenith * (0.15 + 0.35 * static_cast<double>(nextRand())),
                                  radial * outSpd + zenith * liftSpd,
                                  1.10f + 0.90f * nextRand(), ringCol,
                                  static_cast<float>(1.15 + 0.95 * gf + 0.55 * nextRand()), 5.0f, 1.8f, 1.0f);
                }
                particleIdleUntil_ = std::max(particleIdleUntil_, simTime + 2.2);
            }
            // Note: Do not emit sparse discrete blade-tip billboard puffs — at 250+ RPM discrete point spawns
            // separate by several metres and read as ugly floating white dots around the main and tail rotors.
            (void)rh;
            (void)velEcef;
            (void)axisEcef;
        }
    }
}

void GpuParticles::update(const ipc::CombatVfxFrameV1* vfxFrame, const ipc::DebrisFrameV1* debrisFrame,
                          const vsg::dmat4& viewMatrix, const vsg::dmat4& projection,
                          std::uint32_t screenWidth, std::uint32_t screenHeight,
                          const vsg::dvec3& eyeEcef, const vsg::dvec3& cameraRight, const vsg::dvec3& cameraUp,
                          const vsg::dvec3& sunDirEcef, float daylight, double simTime,
                          const DebrisVisuals::FlowInterleave* flowRef) {
    if (!built_) return;

    const double clock = std::isfinite(simTime) ? simTime : (vfxFrame ? vfxFrame->simTime : 0.0);
    // Reset cleanly on backward seek or large forward jump.
    if (lastClock_ > -1e8 && (clock < lastClock_ - 0.03 || clock > lastClock_ + 4.0)) {
        clear();
    }
    const double dt = (lastClock_ > -1e8) ? std::clamp(clock - lastClock_, 0.0, 0.10) : 0.0;
    lastClock_ = clock;

    // Maintain floating origin within 25 km of camera for < 1.5 mm GPU float precision worldwide.
    if (!hasOrigin_ || vsg::length(eyeEcef - originEcef_) > 25000.0) {
        if (hasOrigin_) {
            // Rebase any queued spawns from earlier this frame (e.g. addDetonationShockwave) onto new origin.
            const vsg::dvec3 shift = originEcef_ - eyeEcef;
            for (auto& sp : pendingSpawns_) {
                sp.posAge.x += static_cast<float>(shift.x);
                sp.posAge.y += static_cast<float>(shift.y);
                sp.posAge.z += static_cast<float>(shift.z);
            }
        }
        originEcef_ = eyeEcef;
        hasOrigin_ = true;
        if (resetFlag_) resetFlag_->store(true, std::memory_order_release);
    }

    std::vector<RibbonCpu> ribbons;
    ribbons.reserve(256);

    const vsg::dvec3 upLocal = vsg::length(eyeEcef) > 1.0 ? vsg::normalize(eyeEcef) : vsg::dvec3(0.0, 0.0, 1.0);
    const vsg::dvec3 sunN = vsg::length(sunDirEcef) > 1e-6 ? vsg::normalize(sunDirEcef) : vsg::dvec3(0.0, 0.0, 1.0);
    const vsg::dvec3 rightN = vsg::length(cameraRight) > 1e-6 ? vsg::normalize(cameraRight) : vsg::dvec3(1.0, 0.0, 0.0);
    const vsg::dvec3 upN = vsg::length(cameraUp) > 1e-6 ? vsg::normalize(cameraUp) : vsg::dvec3(0.0, 1.0, 0.0);

    // --- 1. Process authoritative discrete combat VFX events (deduplicated by serial) ---
    if (vfxFrame) {
        for (const auto& ev : vfxFrame->events) {
            if (!seenEventSerials_.insert(ev.serial).second) continue;
            if (seenEventSerials_.size() > 4096u) seenEventSerials_.clear();
            const vsg::dvec3 pos(ev.positionEcef[0], ev.positionEcef[1], ev.positionEcef[2]);
            const vsg::dvec3 vel(ev.velocityEcef[0], ev.velocityEcef[1], ev.velocityEcef[2]);
            vsg::dvec3 dir(ev.dirEcef[0], ev.dirEcef[1], ev.dirEcef[2]);
            dir = (vsg::length(dir) > 1e-4) ? vsg::normalize(dir) : upLocal;
            const auto kind = static_cast<ipc::CombatVfxEventKindV1>(ev.kind);

            if (kind == ipc::CombatVfxEventKindV1::MuzzleFlash) {
                // Keep / refresh glowing directional muzzle flash cone for this shooter.
                bool refreshed = false;
                for (auto& mf : muzzleFlashes_) {
                    if (mf.shooterId == ev.sourceVehicleId) {
                        mf.posEcef = pos;
                        mf.velEcef = vel;
                        mf.dirEcef = dir;
                        mf.startTime = clock;
                        mf.calibreM = std::max(ev.magnitude, 0.012f);
                        mf.seed = nextRand();
                        refreshed = true;
                        break;
                    }
                }
                if (!refreshed) {
                    muzzleFlashes_.push_back(ActiveMuzzleFlash{ev.sourceVehicleId, pos, vel, dir, clock, 0.075,
                                                               std::max(ev.magnitude, 0.012f), nextRand()});
                }
                // Brief gun propellant gas & tiny delicate muzzle sparks.
                const vsg::dvec3 uAx = orthoVec(dir);
                const vsg::dvec3 vAx = vsg::cross(dir, uAx);
                for (int i = 0; i < 4; ++i) {
                    const double ang = 6.2831853 * static_cast<double>(nextRand());
                    const double spread = 2.0 + 6.0 * static_cast<double>(nextRand());
                    const vsg::dvec3 pVel = vel + dir * (18.0 + 24.0 * static_cast<double>(nextRand())) +
                                            (uAx * std::cos(ang) + vAx * std::sin(ang)) * spread;
                    queueParticle(pos + dir * 0.35, pVel, 0.22f + 0.20f * nextRand(),
                                  vsg::vec3(0.58f, 0.56f, 0.53f), 0.24f + 0.16f * nextRand(), 3.0f, 6.0f, 1.25f);
                }
                for (int i = 0; i < 3; ++i) {
                    const double ang = 6.2831853 * static_cast<double>(nextRand());
                    const double spread = 4.0 + 11.0 * static_cast<double>(nextRand());
                    const vsg::dvec3 sVel = vel + dir * (35.0 + 55.0 * static_cast<double>(nextRand())) +
                                            (uAx * std::cos(ang) + vAx * std::sin(ang)) * spread;
                    queueParticle(pos + dir * 0.25, sVel, 0.06f + 0.07f * nextRand(),
                                  vsg::vec3(1.0f, 0.72f, 0.20f), 0.024f + 0.014f * nextRand(), 1.0f, 3.2f, 2.8f);
                }
                particleIdleUntil_ = std::max(particleIdleUntil_, clock + 0.6);
            } else if (kind == ipc::CombatVfxEventKindV1::BulletAirframeHit) {
                // Tight, realistic metallic ricochet sparks + compact HEI impact flash/smoke right on the struck skin!
                const bool isHe = ev.param > 1e-4f;
                const int nSparks = isHe ? 6 : 4;
                const vsg::dvec3 refl = -dir;
                for (int i = 0; i < nSparks; ++i) {
                    const vsg::dvec3 randDir = vsg::normalize(refl * 0.45 + randomUnitVec());
                    const double spd = 14.0 + 42.0 * static_cast<double>(nextRand());
                    queueParticle(pos, vel + randDir * spd, 0.12f + 0.16f * nextRand(),
                                  vsg::vec3(1.0f, 0.66f, 0.14f), 0.024f + 0.016f * nextRand(), 1.0f,
                                  2.8f + 1.2f * nextRand(), 2.8f);
                }
                {
                    const vsg::dvec3 d = randomUnitVec();
                    queueParticle(pos + d * 0.12, vel * 0.78 + d * (4.0 + 7.0 * static_cast<double>(nextRand())),
                                  0.26f + 0.22f * nextRand(),
                                  isHe ? vsg::vec3(0.26f, 0.25f, 0.24f) : vsg::vec3(0.50f, 0.51f, 0.53f),
                                  isHe ? (0.26f + 0.16f * nextRand()) : (0.18f + 0.10f * nextRand()),
                                  3.0f, 4.8f, isHe ? 0.85f : 0.40f);
                }
                particleIdleUntil_ = std::max(particleIdleUntil_, clock + 0.7);
            } else if (kind == ipc::CombatVfxEventKindV1::BulletSurfaceImpact) {
                const auto surf = static_cast<ipc::SurfaceKindV1>(ev.flags);
                const bool water = (surf == ipc::SurfaceKindV1::Water);
                const bool rock = (surf == ipc::SurfaceKindV1::Rock);
                const vsg::dvec3 uAx = orthoVec(dir);
                const vsg::dvec3 vAx = vsg::cross(dir, uAx);
                const vsg::vec3 col = water ? vsg::vec3(0.92f, 0.96f, 0.99f)
                                            : (rock ? vsg::vec3(0.62f, 0.61f, 0.58f) : vsg::vec3(0.50f, 0.41f, 0.31f));
                const int nPlumes = water ? 16 : 12;
                for (int i = 0; i < nPlumes; ++i) {
                    const double ang = 6.2831853 * static_cast<double>(nextRand());
                    const double lat = (water ? 6.0 : 9.0) * static_cast<double>(nextRand());
                    const double upSpd = (water ? 16.0 : 12.0) + (water ? 28.0 : 18.0) * static_cast<double>(nextRand());
                    const vsg::dvec3 v = dir * upSpd + (uAx * std::cos(ang) + vAx * std::sin(ang)) * lat;
                    queueParticle(pos + dir * 0.4, v, 0.85f + 0.95f * nextRand(), col,
                                  (water ? 0.55f : 0.70f) + 0.50f * nextRand(), 4.0f, water ? 1.8f : 2.2f, 0.0f);
                }
                if (!water) {
                    const int nSparks = rock ? 14 : 6;
                    for (int i = 0; i < nSparks; ++i) {
                        const vsg::dvec3 d = vsg::normalize(dir * 0.8 + randomUnitVec() * 0.6);
                        queueParticle(pos + dir * 0.2, d * (25.0 + 55.0 * static_cast<double>(nextRand())),
                                      0.35f + 0.45f * nextRand(), vsg::vec3(1.0f, 0.82f, 0.42f),
                                      0.08f + 0.04f * nextRand(), 1.0f, 1.5f, 5.5f);
                    }
                }
                particleIdleUntil_ = std::max(particleIdleUntil_, clock + 2.0);
            } else if (kind == ipc::CombatVfxEventKindV1::DetonationShockwave) {
                addDetonationShockwave(pos, vel, static_cast<double>(ev.magnitude), clock);
            } else if (kind == ipc::CombatVfxEventKindV1::CountermeasureEject) {
                const bool isFlare = (ev.flags == static_cast<std::uint8_t>(ipc::DecoyKindV1::Flare));
                const vsg::dvec3 uAx = orthoVec(dir);
                const vsg::dvec3 vAx = vsg::cross(dir, uAx);
                if (isFlare) {
                    // Compact pyrotechnic flare cartridge ignition spark pop!
                    for (int i = 0; i < 6; ++i) {
                        const double ang = 6.2831853 * static_cast<double>(nextRand());
                        const vsg::dvec3 v = vel + dir * (10.0 + 14.0 * static_cast<double>(nextRand())) +
                                             (uAx * std::cos(ang) + vAx * std::sin(ang)) * (4.0 + 9.0 * static_cast<double>(nextRand()));
                        queueParticle(pos, v, 0.16f + 0.18f * nextRand(), vsg::vec3(1.0f, 0.78f, 0.28f),
                                      0.032f + 0.020f * nextRand(), 1.0f, 2.8f, 2.6f);
                    }
                    particleIdleUntil_ = std::max(particleIdleUntil_, clock + 1.0);
                } else {
                    // Chaff cartridge burst: delicate sparkling metallic dipole flecks + translucent silver-grey cloud!
                    for (int i = 0; i < 24; ++i) {
                        const vsg::dvec3 d = randomUnitVec();
                        const double spd = 6.0 + 18.0 * static_cast<double>(nextRand());
                        queueParticle(pos + d * 0.4, vel * 0.50 + d * spd, 2.2f + 2.2f * nextRand(),
                                      vsg::vec3(0.84f, 0.89f, 0.95f), 0.055f + 0.035f * nextRand(), 6.0f, 3.6f, 0.0f);
                    }
                    for (int i = 0; i < 10; ++i) {
                        const vsg::dvec3 d = randomUnitVec();
                        queueParticle(pos + d * 0.8, vel * 0.35 + d * (3.5 + 8.5 * static_cast<double>(nextRand())),
                                      3.8f + 2.0f * nextRand(), vsg::vec3(0.76f, 0.81f, 0.87f),
                                      1.10f + 0.75f * nextRand(), 2.0f, 3.8f, 0.0f);
                    }
                    particleIdleUntil_ = std::max(particleIdleUntil_, clock + 6.0);
                }
            }
        }

        // --- 2. Live Ballistic Gun Projectiles -> Exact-Pose Sleek HDR Tracer Dashes ---
        for (const auto& p : vfxFrame->projectiles) {
            if (ribbons.size() >= kMaxRibbons) break;
            const vsg::dvec3 pos(p.positionEcef[0], p.positionEcef[1], p.positionEcef[2]);
            const vsg::dvec3 vel(p.velocityEcef[0], p.velocityEcef[1], p.velocityEcef[2]);
            const double spd = vsg::length(vel);
            if (spd < 30.0) continue;
            const vsg::dvec3 dir = vel / spd;
            const bool isTracer = (p.flags & ipc::kProjectileFlagTracer) != 0u;
            const bool isHe = (p.flags & ipc::kProjectileFlagHe) != 0u;
            const vsg::dvec3 rel = pos - eyeEcef;

            RibbonCpu r{};
            const float width = isTracer ? (0.12f + 2.0f * std::max(p.calibreM - 0.02f, 0.0f)) : 0.055f;
            const float length = isTracer ? static_cast<float>(std::clamp(spd * 0.0095, 7.0, 12.0))
                                          : static_cast<float>(std::clamp(spd * 0.0035, 2.8, 4.8));
            r.posRadius = vsg::vec4(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z), width);
            r.dirLength = vsg::vec4(static_cast<float>(dir.x), static_cast<float>(dir.y), static_cast<float>(dir.z), length);
            if (isTracer) {
                // Crisp distinct golden-amber tracer dashes with white-hot leading tip
                const vsg::vec3 tint = isHe ? vsg::vec3(1.0f, 0.56f, 0.08f) : vsg::vec3(1.0f, 0.72f, 0.20f);
                r.colorIntensity = vsg::vec4(tint.r, tint.g, tint.b, 1.25f);
            } else {
                r.colorIntensity = vsg::vec4(0.95f, 0.78f, 0.48f, 0.22f);
            }
            r.params = vsg::vec4(0.0f, std::clamp(p.ageS / std::max(p.maxAgeS, 0.1f), 0.0f, 1.0f),
                                 static_cast<float>(p.id & 0xFFu) / 255.0f, 0.0f);
            ribbons.push_back(r);
        }

        // --- 3. Live Countermeasures: IR Flares (HDR Core + Corkscrew Smoke) & Chaff Clouds ---
        std::unordered_set<std::uint32_t> aliveDecoyIds;
        aliveDecoyIds.reserve(vfxFrame->decoys.size() * 2u + 1u);
        for (const auto& d : vfxFrame->decoys) {
            aliveDecoyIds.insert(d.id);
            const vsg::dvec3 pos(d.positionEcef[0], d.positionEcef[1], d.positionEcef[2]);
            const vsg::dvec3 vel(d.velocityEcef[0], d.velocityEcef[1], d.velocityEcef[2]);
            auto& trail = decoyTrails_[d.id];

            if (d.kind == static_cast<std::uint8_t>(ipc::DecoyKindV1::Flare)) {
                // Blazing HDR pyrotechnic magnesium flare core + 6-point optical diffraction star!
                if (ribbons.size() < kMaxRibbons) {
                    const vsg::dvec3 rel = pos - eyeEcef;
                    const float normInt = std::clamp(std::sqrt(std::max(d.intensityOrSigma, 0.0f) / 6000.0f), 0.12f, 1.35f);
                    // Brief pyrotechnic cartridge ignition ramp over the first 0.06 s after leaving the dispenser bay.
                    const float ignite = std::clamp(0.35f + (d.ageS / 0.06f) * 0.65f, 0.35f, 1.0f);
                    RibbonCpu r{};
                    r.posRadius = vsg::vec4(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z),
                                            1.75f * ignite * (0.45f + 0.65f * normInt));
                    r.dirLength = vsg::vec4(0.0f, 0.0f, 1.0f, 1.75f);
                    r.colorIntensity = vsg::vec4(1.0f, 0.90f, 0.56f, 1.25f * normInt * ignite);
                    r.params = vsg::vec4(2.0f, std::clamp(d.ageS / std::max(d.lifeS, 0.1f), 0.0f, 1.0f),
                                         static_cast<float>(d.id & 0xFFu) / 255.0f, normInt);
                    ribbons.push_back(r);
                }

                // Continuous sub-stepped dense corkscrew white MgO smoke trail + delicate magnesium sparks!
                if (!trail.initialized) {
                    trail.lastPosEcef = pos;
                    trail.lastEmitTime = clock - 0.015;
                    trail.initialized = true;
                }
                const double elapsed = clock - trail.lastEmitTime;
                if (elapsed >= 0.008 && d.ageS < d.lifeS * 0.96f) {
                    const int steps = std::clamp(static_cast<int>(elapsed / 0.008), 1, 32);
                    const vsg::dvec3 uAx = orthoVec(upLocal);
                    const vsg::dvec3 vAx = vsg::cross(upLocal, uAx);
                    for (int s = 1; s <= steps; ++s) {
                        const double frac = static_cast<double>(s) / static_cast<double>(steps);
                        const vsg::dvec3 sp = trail.lastPosEcef + (pos - trail.lastPosEcef) * frac;
                        const double helixAng = static_cast<double>(d.id) * 1.9 + (clock - elapsed + elapsed * frac) * 14.0;
                        const vsg::dvec3 curlVel = (uAx * std::cos(helixAng) + vAx * std::sin(helixAng)) * 3.4;
                        queueParticle(sp, vel * 0.05 + curlVel, 2.8f + 1.4f * nextRand(),
                                      vsg::vec3(0.95f, 0.95f, 0.94f), 0.72f + 0.28f * nextRand(), 0.0f, 3.2f, 0.0f);
                        if ((s % 4) == 0) {
                            const vsg::dvec3 sparkDir = randomUnitVec();
                            queueParticle(sp, vel * 0.18 + sparkDir * (5.0 + 12.0 * static_cast<double>(nextRand())),
                                          0.14f + 0.16f * nextRand(), vsg::vec3(1.0f, 0.78f, 0.28f),
                                          0.026f + 0.016f * nextRand(), 1.0f, 2.6f, 2.6f);
                        }
                    }
                    trail.lastPosEcef = pos;
                    trail.lastEmitTime = clock;
                    particleIdleUntil_ = std::max(particleIdleUntil_, clock + 4.4);
                }
            } else {
                // Chaff dipole cloud: initial burst + sustained shimmering metallic flecks & translucent haze!
                if (!trail.initialized) {
                    trail.lastPosEcef = pos;
                    trail.lastEmitTime = clock;
                    trail.initialized = true;
                    for (int i = 0; i < 16; ++i) {
                        const vsg::dvec3 dDir = randomUnitVec();
                        queueParticle(pos + dDir * (static_cast<double>(d.bloomRadiusM) * 0.55),
                                      vel * 0.22 + dDir * (2.5 + 6.5 * static_cast<double>(nextRand())),
                                      2.0f + 2.0f * nextRand(), vsg::vec3(0.84f, 0.89f, 0.95f),
                                      0.050f + 0.032f * nextRand(), 6.0f, 3.4f, 0.0f);
                    }
                    for (int i = 0; i < 7; ++i) {
                        const vsg::dvec3 dDir = randomUnitVec();
                        queueParticle(pos + dDir * (static_cast<double>(d.bloomRadiusM) * 0.4),
                                      vel * 0.15 + dDir * (1.5 + 4.0 * static_cast<double>(nextRand())),
                                      3.2f + 1.8f * nextRand(), vsg::vec3(0.76f, 0.81f, 0.87f),
                                      std::max(1.05f, d.bloomRadiusM * 0.45f), 2.0f, 3.6f, 0.0f);
                    }
                    particleIdleUntil_ = std::max(particleIdleUntil_, clock + 5.0);
                } else if (d.ageS < 2.2f && clock - trail.lastEmitTime >= 0.08) {
                    for (int i = 0; i < 4; ++i) {
                        const vsg::dvec3 dDir = randomUnitVec();
                        queueParticle(pos + dDir * (static_cast<double>(d.bloomRadiusM) * 0.65),
                                      vel * 0.18 + dDir * (1.8 + 5.0 * static_cast<double>(nextRand())),
                                      1.4f + 1.4f * nextRand(), vsg::vec3(0.84f, 0.89f, 0.95f),
                                      0.048f + 0.030f * nextRand(), 6.0f, 3.4f, 0.0f);
                    }
                    trail.lastPosEcef = pos;
                    trail.lastEmitTime = clock;
                    particleIdleUntil_ = std::max(particleIdleUntil_, clock + 3.0);
                }
            }
        }
        for (auto it = decoyTrails_.begin(); it != decoyTrails_.end();) {
            if (aliveDecoyIds.find(it->first) == aliveDecoyIds.end()) it = decoyTrails_.erase(it);
            else ++it;
        }
    }

    // --- 4. Burning DebrisField Chunks -> Dense GPU Incandescent Ember & Slag Showers ---
    if (debrisFrame) {
        std::unordered_set<std::uint32_t> aliveDebrisIds;
        for (const auto& b : debrisFrame->bodies) {
            if ((b.flags & ipc::kDebrisFlagBurning) == 0u || b.temperatureK < 650.0f) continue;
            const auto arch = static_cast<ipc::DebrisArchetypeV1>(b.archetype);
            if (arch == ipc::DebrisArchetypeV1::SparkEmber || arch == ipc::DebrisArchetypeV1::SplashPlume) continue;
            aliveDebrisIds.insert(b.id);
            auto& st = debrisEmbers_[b.id];
            if (clock - st.lastEmitTime >= 0.040) {
                st.lastEmitTime = clock;
                const vsg::dvec3 pos(b.positionEcef[0], b.positionEcef[1], b.positionEcef[2]);
                const vsg::dvec3 vel(b.velocityEcef[0], b.velocityEcef[1], b.velocityEcef[2]);
                const float heat = std::clamp((b.temperatureK - 650.0f) / 1400.0f, 0.2f, 1.0f);
                const int count = (arch == ipc::DebrisArchetypeV1::AirframeHull || arch == ipc::DebrisArchetypeV1::WingFragment) ? 2 : 1;
                for (int k = 0; k < count; ++k) {
                    const vsg::dvec3 jitter = randomUnitVec();
                    const double rad = std::max({b.halfExtentsM[0], b.halfExtentsM[1], b.halfExtentsM[2], 0.3f});
                    queueParticle(pos + jitter * (rad * 0.40),
                                  vel * 0.65 + jitter * (5.0 + 14.0 * static_cast<double>(nextRand())),
                                  0.35f + 0.55f * nextRand(),
                                  vsg::vec3(1.0f, 0.48f + 0.32f * heat, 0.08f + 0.20f * heat),
                                  0.034f + 0.024f * nextRand(), 1.0f, 1.8f + 0.8f * nextRand(), 3.2f);
                }
                particleIdleUntil_ = std::max(particleIdleUntil_, clock + 1.4);
            }
        }
        for (auto it = debrisEmbers_.begin(); it != debrisEmbers_.end();) {
            if (aliveDebrisIds.find(it->first) == aliveDebrisIds.end()) it = debrisEmbers_.erase(it);
            else ++it;
        }
    }

    // --- 5. Active Muzzle Flash Cones & Expanding Supersonic Detonation Shockwaves ---
    for (auto it = muzzleFlashes_.begin(); it != muzzleFlashes_.end();) {
        const double age = clock - it->startTime;
        if (age < 0.0 || age >= it->duration) {
            it = muzzleFlashes_.erase(it);
            continue;
        }
        if (ribbons.size() < kMaxRibbons) {
            const vsg::dvec3 posNow = it->posEcef + it->velEcef * age;
            const vsg::dvec3 rel = posNow - eyeEcef;
            const float calScale = std::clamp(it->calibreM / 0.02f, 0.7f, 1.6f);
            RibbonCpu r{};
            r.posRadius = vsg::vec4(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z),
                                    0.52f * calScale);
            r.dirLength = vsg::vec4(static_cast<float>(it->dirEcef.x), static_cast<float>(it->dirEcef.y),
                                    static_cast<float>(it->dirEcef.z), 1.85f * calScale);
            r.colorIntensity = vsg::vec4(1.0f, 0.72f, 0.24f, 1.25f);
            r.params = vsg::vec4(1.0f, static_cast<float>(age / it->duration), it->seed, 1.0f);
            ribbons.push_back(r);
        }
        ++it;
    }

    for (auto it = shockwaves_.begin(); it != shockwaves_.end();) {
        const double age = clock - it->startTime;
        if (age < 0.0 || age >= it->duration) {
            it = shockwaves_.erase(it);
            continue;
        }
        if (ribbons.size() < kMaxRibbons) {
            const float ageNorm = static_cast<float>(age / it->duration);
            // Sedov-Taylor / Hopkinson-Cranz supersonic blast expansion: R(t) ~ t^0.52.
            const float curRad = it->maxRadiusM * std::pow(std::max(ageNorm, 0.005f), 0.52f);
            const vsg::dvec3 posNow = it->posEcef + it->velEcef * age;
            const vsg::dvec3 rel = posNow - eyeEcef;
            RibbonCpu r{};
            r.posRadius = vsg::vec4(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z), curRad);
            r.dirLength = vsg::vec4(0.0f, 0.0f, 1.0f, curRad);
            r.colorIntensity = vsg::vec4(0.92f, 0.95f, 1.0f, it->intensity);
            r.params = vsg::vec4(3.0f, ageNorm, it->seed, 1.0f);
            ribbons.push_back(r);
        }
        ++it;
    }

    // --- 6. Upload Staging Buffers & Toggle Switches (Zero-Cost When Completely Idle!) ---
    lastSpawnCount_ = static_cast<std::uint32_t>(pendingSpawns_.size());
    lastRibbonCount_ = static_cast<std::uint32_t>(ribbons.size());
    const bool particlesLive = (lastSpawnCount_ > 0u) || (clock <= particleIdleUntil_);
    const bool ribbonsLive = (lastRibbonCount_ > 0u);
    active_ = particlesLive || ribbonsLive;

    if (!active_) {
        if (computeSwitch_) computeSwitch_->setAllChildren(false);
        if (spawnSwitch_) spawnSwitch_->setAllChildren(false);
        if (drawSwitch_) drawSwitch_->setAllChildren(false);
        if (ribbonSwitch_) ribbonSwitch_->setAllChildren(false);
        if (ribbonDraw_) ribbonDraw_->instanceCount = 0;
        return;
    }

    // Update floating eye-relative transform on both draw subgraphs.
    const vsg::dmat4 eyeTranslate = vsg::translate(eyeEcef);
    if (drawTransform_) drawTransform_->matrix = eyeTranslate;
    if (ribbonTransform_) ribbonTransform_->matrix = eyeTranslate;

    const vsg::dvec3 originToEye = originEcef_ - eyeEcef;
    const float p22 = static_cast<float>(projection[2][2]);
    const float p32 = static_cast<float>(projection[3][2]);
    const float invW = 1.0f / static_cast<float>(std::max<std::uint32_t>(screenWidth, 1u));
    const float invH = 1.0f / static_cast<float>(std::max<std::uint32_t>(screenHeight, 1u));
    const bool flowActive = (flowRef != nullptr && flowRef->active);
    const float timeWrap = static_cast<float>(std::fmod(std::max(clock, 0.0), 600.0));

    // Render frame UBO (shared by indirect particles + direct ribbons).
    renderUbo_->set(0, vsg::vec4(static_cast<float>(sunN.x), static_cast<float>(sunN.y), static_cast<float>(sunN.z),
                                 std::clamp(daylight, 0.0f, 1.0f)));
    renderUbo_->set(1, vsg::vec4(static_cast<float>(rightN.x), static_cast<float>(rightN.y), static_cast<float>(rightN.z), 0.0f));
    renderUbo_->set(2, vsg::vec4(static_cast<float>(upN.x), static_cast<float>(upN.y), static_cast<float>(upN.z), 0.0f));
    renderUbo_->set(3, vsg::vec4(static_cast<float>(upLocal.x), static_cast<float>(upLocal.y), static_cast<float>(upLocal.z), timeWrap));
    renderUbo_->set(4, vsg::vec4(static_cast<float>(originToEye.x), static_cast<float>(originToEye.y), static_cast<float>(originToEye.z), 0.0f));
    renderUbo_->set(5, vsg::vec4(p22, p32, invW, invH));
    renderUbo_->set(6, flowActive ? flowRef->laid : vsg::vec4(0.0f, 0.0f, -1.0f, -1.0f));
    renderUbo_->set(7, vsg::vec4(flowActive ? std::max(flowRef->scale, 1.0f) : 2.0f, 0.0f, 0.0f, 0.0f));
    renderUbo_->dirty();

    // Compute simulation UBO & spawn staging buffer.
    if (particlesLive) {
        const auto frustum = extractEyeRelativeFrustum(projection, viewMatrix, eyeEcef);
        simUbo_->set(0, vsg::vec4(static_cast<float>(dt), timeWrap, static_cast<float>(lastSpawnCount_),
                                  static_cast<float>(kMaxParticles)));
        simUbo_->set(1, vsg::vec4(static_cast<float>(originToEye.x), static_cast<float>(originToEye.y),
                                  static_cast<float>(originToEye.z), std::clamp(daylight, 0.0f, 1.0f)));
        simUbo_->set(2, vsg::vec4(static_cast<float>(upLocal.x), static_cast<float>(upLocal.y), static_cast<float>(upLocal.z), 9.80665f));
        simUbo_->set(3, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
        for (std::uint32_t i = 0; i < 6; ++i) simUbo_->set(4u + i, frustum[i]);
        simUbo_->set(10, vsg::vec4(p22, p32, invW, invH));
        simUbo_->set(11, vsg::vec4(static_cast<float>(sunN.x), static_cast<float>(sunN.y), static_cast<float>(sunN.z), 0.0f));
        simUbo_->dirty();

        if (lastSpawnCount_ > 0u) {
            for (std::uint32_t i = 0; i < lastSpawnCount_; ++i) {
                const auto& sp = pendingSpawns_[i];
                spawnStaging_->set(i * 4u + 0u, sp.posAge);
                spawnStaging_->set(i * 4u + 1u, sp.velLife);
                spawnStaging_->set(i * 4u + 2u, sp.colorSize);
                spawnStaging_->set(i * 4u + 3u, sp.params);
            }
            spawnStaging_->dirty();
            if (spawnDispatch_) spawnDispatch_->groupCountX = (lastSpawnCount_ + 63u) / 64u;
            if (spawnSwitch_) spawnSwitch_->setAllChildren(true);
        } else {
            if (spawnSwitch_) spawnSwitch_->setAllChildren(false);
        }
        if (computeSwitch_) computeSwitch_->setAllChildren(true);
    } else {
        if (computeSwitch_) computeSwitch_->setAllChildren(false);
        if (spawnSwitch_) spawnSwitch_->setAllChildren(false);
    }

    // Direct combat ribbons staging buffer.
    if (ribbonsLive) {
        for (std::uint32_t i = 0; i < lastRibbonCount_; ++i) {
            const auto& r = ribbons[i];
            ribbonStaging_->set(i * 4u + 0u, r.posRadius);
            ribbonStaging_->set(i * 4u + 1u, r.dirLength);
            ribbonStaging_->set(i * 4u + 2u, r.colorIntensity);
            ribbonStaging_->set(i * 4u + 3u, r.params);
        }
        ribbonStaging_->dirty();
        if (ribbonDraw_) ribbonDraw_->instanceCount = lastRibbonCount_;
        if (ribbonSwitch_) ribbonSwitch_->setAllChildren(true);
    } else {
        if (ribbonDraw_) ribbonDraw_->instanceCount = 0;
        if (ribbonSwitch_) ribbonSwitch_->setAllChildren(false);
    }

    if (drawSwitch_) drawSwitch_->setAllChildren(true);
    pendingSpawns_.clear();
}

} // namespace fsim::world
