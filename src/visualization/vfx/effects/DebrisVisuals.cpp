#include "visualization/vfx/effects/DebrisVisuals.h"

#include "foundation/core/Log.h"
#include "visualization/scene/Clouds.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fsim::world {

namespace {

struct MeshBuilder {
    std::vector<vsg::vec3> positions;
    std::vector<vsg::vec3> normals;
    std::vector<vsg::vec2> meta; // x = fracture edge glow (0..1), y = surface tone (0..1)

    void addTri(const vsg::vec3& a, const vsg::vec3& b, const vsg::vec3& c, float glowA, float glowB, float glowC, float tone) {
        const vsg::vec3 e1 = b - a;
        const vsg::vec3 e2 = c - a;
        vsg::vec3 n = vsg::cross(e1, e2);
        const float len = vsg::length(n);
        if (len < 1e-6f) return;
        n /= len;
        positions.push_back(a);
        positions.push_back(b);
        positions.push_back(c);
        normals.push_back(n);
        normals.push_back(n);
        normals.push_back(n);
        meta.emplace_back(glowA, tone);
        meta.emplace_back(glowB, tone);
        meta.emplace_back(glowC, tone);
    }

    void addQuad(const vsg::vec3& a, const vsg::vec3& b, const vsg::vec3& c, const vsg::vec3& d, float gA, float gB, float gC, float gD,
                 float tone) {
        addTri(a, b, c, gA, gB, gC, tone);
        addTri(a, c, d, gA, gC, gD, tone);
    }

    vsg::ref_ptr<vsg::VertexDraw> build() const {
        const std::uint32_t count = static_cast<std::uint32_t>(positions.size());
        auto posArr = vsg::vec3Array::create(count);
        auto normArr = vsg::vec3Array::create(count);
        auto metaArr = vsg::vec2Array::create(count);
        for (std::uint32_t i = 0; i < count; ++i) {
            posArr->set(i, positions[i]);
            normArr->set(i, normals[i]);
            metaArr->set(i, meta[i]);
        }
        auto draw = vsg::VertexDraw::create();
        draw->assignArrays({posArr, normArr, metaArr});
        draw->vertexCount = count;
        draw->instanceCount = 0;
        draw->firstVertex = 0;
        draw->firstInstance = 0;
        return draw;
    }
};

// Archetype 0 (AirframeHull): torn hollow aerospace fuselage section with sheared jagged bulkhead edges
// and internal structural rib backing so tumbling hull segments never read as clean boxes.
vsg::ref_ptr<vsg::VertexDraw> buildAirframeHullMesh() {
    MeshBuilder m;
    // Longitudinal rings along X (-1..+1): jagged sheared rear (-1) and torn forward (+1) edges.
    static constexpr int kSeg = 7;
    static constexpr float kAng[kSeg] = {-2.25f, -1.55f, -0.80f, 0.0f, 0.82f, 1.58f, 2.30f};
    static constexpr float kJagBack[kSeg] = {-0.96f, -0.72f, -1.04f, -0.82f, -1.02f, -0.68f, -0.92f};
    static constexpr float kJagFront[kSeg] = {0.88f, 1.04f, 0.78f, 0.98f, 0.74f, 1.02f, 0.84f};
    static constexpr float kRadY[kSeg] = {0.82f, 0.98f, 0.92f, 0.88f, 0.94f, 1.00f, 0.80f};
    static constexpr float kRadZ[kSeg] = {0.84f, 0.92f, 1.02f, 0.96f, 1.00f, 0.90f, 0.82f};

    auto pt = [](float x, int i, float rScale) {
        const float a = kAng[i];
        return vsg::vec3(x, std::sin(a) * kRadY[i] * rScale, -std::cos(a) * kRadZ[i] * rScale);
    };
    for (int i = 0; i + 1 < kSeg; ++i) {
        const float midX0 = 0.5f * (kJagBack[i] + kJagFront[i]);
        const float midX1 = 0.5f * (kJagBack[i + 1] + kJagFront[i + 1]);
        const vsg::vec3 b0 = pt(kJagBack[i], i, 0.92f);
        const vsg::vec3 b1 = pt(kJagBack[i + 1], i + 1, 0.92f);
        const vsg::vec3 m0 = pt(midX0, i, 1.02f);
        const vsg::vec3 m1 = pt(midX1, i + 1, 1.02f);
        const vsg::vec3 f0 = pt(kJagFront[i], i, 0.88f);
        const vsg::vec3 f1 = pt(kJagFront[i + 1], i + 1, 0.88f);
        // Outer curved fuselage skin (high fracture glow at jagged ends, clean skin in middle)
        m.addQuad(b0, b1, m1, m0, 0.95f, 0.95f, 0.25f, 0.25f, 0.95f);
        m.addQuad(m0, m1, f1, f0, 0.25f, 0.25f, 0.95f, 0.95f, 0.90f);
    }
    // Torn internal floor / keel beam spanning across the hollow shell for structural silhouette.
    m.addQuad(vsg::vec3(-0.90f, -0.68f, 0.42f), vsg::vec3(-0.85f, 0.68f, 0.46f), vsg::vec3(0.84f, 0.62f, 0.36f),
              vsg::vec3(0.76f, -0.64f, 0.40f), 0.70f, 0.70f, 0.75f, 0.75f, 0.35f);
    // Internal crumpled bulkhead rib near centre
    m.addTri(vsg::vec3(-0.08f, -0.78f, 0.35f), vsg::vec3(0.06f, 0.0f, -0.82f), vsg::vec3(-0.02f, 0.76f, 0.38f), 0.55f, 0.45f, 0.55f, 0.30f);
    return m.build();
}

// Archetype 1 (WingFragment): swept tapered torn wing / stabilator panel with sheared jagged root spar.
vsg::ref_ptr<vsg::VertexDraw> buildWingFragmentMesh() {
    MeshBuilder m;
    // Leading edge sweeps aft from root (y=-1) to tip (y=+1); root edge is jaggedly sheared.
    const vsg::vec3 rLE(0.82f, -0.92f, -0.18f);
    const vsg::vec3 rMid(0.18f, -1.04f, -0.62f);
    const vsg::vec3 rTE(-0.88f, -0.82f, 0.05f);
    const vsg::vec3 rBot(0.10f, -0.96f, 0.58f);

    const vsg::vec3 mLE(0.54f, 0.04f, -0.10f);
    const vsg::vec3 mMid(-0.04f, 0.00f, -0.44f);
    const vsg::vec3 mTE(-0.82f, 0.08f, 0.04f);
    const vsg::vec3 mBot(-0.08f, 0.02f, 0.38f);

    const vsg::vec3 tLE(0.16f, 0.96f, -0.04f);
    const vsg::vec3 tMid(-0.28f, 1.00f, -0.22f);
    const vsg::vec3 tTE(-0.76f, 0.88f, 0.02f);
    const vsg::vec3 tBot(-0.26f, 0.94f, 0.20f);

    // Upper cambered wing skin
    m.addQuad(rLE, mLE, mMid, rMid, 0.95f, 0.25f, 0.20f, 0.95f, 1.00f);
    m.addQuad(rMid, mMid, mTE, rTE, 0.95f, 0.20f, 0.30f, 0.95f, 0.92f);
    m.addQuad(mLE, tLE, tMid, mMid, 0.25f, 0.70f, 0.65f, 0.20f, 1.00f);
    m.addQuad(mMid, tMid, tTE, mTE, 0.20f, 0.65f, 0.75f, 0.30f, 0.90f);
    // Lower wing skin
    m.addQuad(rLE, rBot, mBot, mLE, 0.95f, 0.95f, 0.20f, 0.25f, 0.72f);
    m.addQuad(rBot, rTE, mTE, mBot, 0.95f, 0.95f, 0.30f, 0.20f, 0.68f);
    m.addQuad(mLE, mBot, tBot, tLE, 0.25f, 0.20f, 0.65f, 0.70f, 0.72f);
    m.addQuad(mBot, mTE, tTE, tBot, 0.20f, 0.30f, 0.75f, 0.65f, 0.68f);
    // Sheared glowing root spar face
    m.addTri(rLE, rMid, rBot, 1.0f, 1.0f, 1.0f, 0.30f);
    m.addTri(rMid, rTE, rBot, 1.0f, 1.0f, 1.0f, 0.30f);
    // Torn tip cap
    m.addTri(tLE, tBot, tMid, 0.75f, 0.75f, 0.75f, 0.45f);
    m.addTri(tMid, tBot, tTE, 0.75f, 0.75f, 0.75f, 0.45f);
    return m.build();
}

// Archetype 2 (PanelShard): warped, bent jagged aircraft skin plate / cowling shard that flashes in sunlight.
vsg::ref_ptr<vsg::VertexDraw> buildPanelShardMesh() {
    MeshBuilder m;
    const vsg::vec3 p0(-0.96f, -0.78f, 0.32f);
    const vsg::vec3 p1(0.08f, -0.94f, -0.34f);
    const vsg::vec3 p2(0.98f, -0.46f, 0.22f);
    const vsg::vec3 p3(0.74f, 0.72f, -0.28f);
    const vsg::vec3 p4(-0.22f, 0.96f, 0.36f);
    const vsg::vec3 p5(-0.88f, 0.42f, -0.18f);
    const vsg::vec3 ctr(0.02f, -0.04f, -0.14f);

    m.addTri(ctr, p0, p1, 0.25f, 0.90f, 0.85f, 1.00f);
    m.addTri(ctr, p1, p2, 0.25f, 0.85f, 0.95f, 0.95f);
    m.addTri(ctr, p2, p3, 0.25f, 0.95f, 0.90f, 0.92f);
    m.addTri(ctr, p3, p4, 0.25f, 0.90f, 0.85f, 0.98f);
    m.addTri(ctr, p4, p5, 0.25f, 0.85f, 0.95f, 0.90f);
    m.addTri(ctr, p5, p0, 0.25f, 0.95f, 0.90f, 0.94f);
    // Bent structural stiffener flange on underside
    m.addTri(p1, p3, ctr + vsg::vec3(0.0f, 0.0f, 0.55f), 0.75f, 0.75f, 0.40f, 0.40f);
    return m.build();
}

// Archetype 3 (BurningChunk): crumpled irregular glowing structural core / turbine / frame wreckage chunk.
vsg::ref_ptr<vsg::VertexDraw> buildBurningChunkMesh() {
    MeshBuilder m;
    const vsg::vec3 v[8] = {
        vsg::vec3(0.98f, 0.12f, -0.24f),
        vsg::vec3(0.28f, 0.94f, 0.18f),
        vsg::vec3(-0.86f, 0.56f, -0.42f),
        vsg::vec3(-0.94f, -0.48f, 0.34f),
        vsg::vec3(-0.18f, -0.96f, -0.22f),
        vsg::vec3(0.68f, -0.62f, 0.46f),
        vsg::vec3(0.10f, 0.08f, -0.96f), // dorsal spike
        vsg::vec3(-0.14f, -0.10f, 0.92f) // ventral spike
    };
    const float g[8] = {1.0f, 0.85f, 0.95f, 1.0f, 0.80f, 0.92f, 0.75f, 0.88f};
    for (int i = 0; i < 6; ++i) {
        const int j = (i + 1) % 6;
        m.addTri(v[6], v[i], v[j], g[6], g[i], g[j], (i % 2 == 0) ? 0.55f : 0.35f);
        m.addTri(v[7], v[j], v[i], g[7], g[j], g[i], (i % 2 == 0) ? 0.32f : 0.48f);
    }
    return m.build();
}

float hashSeed(std::uint32_t id) noexcept {
    std::uint32_t x = id * 747796405u + 2891336453u;
    x = ((x >> ((x >> 28u) + 4u)) ^ x) * 277803737u;
    x = (x >> 22u) ^ x;
    return static_cast<float>(x & 0xFFFFu) / 65535.0f;
}

vsg::vec3 archetypeColor(ipc::DebrisArchetypeV1 arch, std::uint32_t id) noexcept {
    const float h = hashSeed(id);
    switch (arch) {
    case ipc::DebrisArchetypeV1::AirframeHull:
        return vsg::vec3(0.46f + 0.12f * h, 0.49f + 0.12f * h, 0.53f + 0.10f * h); // tactical fighter grey
    case ipc::DebrisArchetypeV1::WingFragment:
        return vsg::vec3(0.50f + 0.10f * h, 0.53f + 0.10f * h, 0.56f + 0.10f * h);
    case ipc::DebrisArchetypeV1::PanelShard:
        return vsg::vec3(0.58f + 0.18f * h, 0.60f + 0.16f * h, 0.62f + 0.16f * h); // bright sheared aluminum
    case ipc::DebrisArchetypeV1::BurningChunk:
        return vsg::vec3(0.24f + 0.08f * h, 0.23f + 0.08f * h, 0.25f + 0.08f * h); // dark charred alloy
    default:
        return vsg::vec3(0.45f, 0.47f, 0.50f);
    }
}

struct PendingQuad {
    vsg::vec4 posAndRadius;
    vsg::vec4 velAndTemp;
    vsg::vec4 colorAlpha;
    vsg::vec4 params;
    float distSq = 0.0f;
    bool additive = false;
};

} // namespace

DebrisVisuals::DebrisVisuals(vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders) {
    (void)options;
    solidRoot_ = vsg::Group::create();
    trailRoot_ = vsg::Group::create();
    solidSwitch_ = vsg::Switch::create();
    trailSwitch_ = vsg::Switch::create();
    solidRoot_->addChild(solidSwitch_);
    trailRoot_->addChild(trailSwitch_);

    if (shaders.empty()) {
        LOG_WARN("world") << "no debris renderer: shader directory missing";
        return;
    }

    auto solidVert = loadCloudShader(shaders, "debris.vert", VK_SHADER_STAGE_VERTEX_BIT);
    auto solidFrag = loadCloudShader(shaders, "debris.frag", VK_SHADER_STAGE_FRAGMENT_BIT);
    auto trailVert = loadCloudShader(shaders, "debris_trail.vert", VK_SHADER_STAGE_VERTEX_BIT);
    auto trailFrag = loadCloudShader(shaders, "debris_trail.frag", VK_SHADER_STAGE_FRAGMENT_BIT);

    vsg::ShaderStages solidStages{solidVert, solidFrag};
    vsg::ShaderStages trailStages{trailVert, trailFrag};
    auto compiler = vsg::ShaderCompiler::create();
    if (!solidVert || !solidFrag || !trailVert || !trailFrag || !compiler->supported() || !compiler->compile(solidStages) ||
        !compiler->compile(trailStages)) {
        LOG_WARN("world") << "debris shaders failed to compile; lightweight debris visuals disabled";
        return;
    }

    const VkShaderStageFlags both = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    vsg::DescriptorSetLayoutBindings bindings{{0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, both, nullptr},
                                              {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, both, nullptr}};
    auto setLayout = vsg::DescriptorSetLayout::create(bindings);
    auto layout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout}, vsg::PushConstantRanges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}});

    frameUbo_ = vsg::vec4Array::create(4, vsg::vec4(0.0f, 0.0f, 1.0f, 1.0f));
    solidInstances_ = vsg::vec4Array::create(kMaxSolidInstances * 4u, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    trailQuads_ = vsg::vec4Array::create(kMaxTrailQuads * 4u, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    frameUbo_->properties.dataVariance = vsg::DYNAMIC_DATA;
    solidInstances_->properties.dataVariance = vsg::DYNAMIC_DATA;
    trailQuads_->properties.dataVariance = vsg::DYNAMIC_DATA;

    // --- 1. Opaque sunlit + incandescent 3-D tumbling debris shards pipeline ---
    {
        vsg::Descriptors desc{vsg::DescriptorBuffer::create(solidInstances_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                              vsg::DescriptorBuffer::create(frameUbo_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER)};
        auto set = vsg::DescriptorSet::create(setLayout, desc);

        vsg::VertexInputState::Bindings vBindings{
            VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},
            VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},
            VkVertexInputBindingDescription{2, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX}};
        vsg::VertexInputState::Attributes vAttribs{
            VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
            VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0},
            VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32G32_SFLOAT, 0}};
        auto vertexInput = vsg::VertexInputState::create(vBindings, vAttribs);

        auto assembly = vsg::InputAssemblyState::create();
        assembly->topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        auto raster = vsg::RasterizationState::create();
        raster->cullMode = VK_CULL_MODE_NONE; // double-sided so hollow torn fuselage shells render cleanly inside & out
        auto depth = vsg::DepthStencilState::create();
        depth->depthTestEnable = VK_TRUE;
        depth->depthWriteEnable = VK_TRUE;
        depth->depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL; // VSG reverse-Z
        auto blend = vsg::ColorBlendState::create();
        blend->configureAttachments(false);

        vsg::GraphicsPipelineStates states{vertexInput, assembly, raster, vsg::MultisampleState::create(), blend, depth};
        auto sg = vsg::StateGroup::create();
        sg->add(vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(layout, solidStages, states)));
        sg->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, set));

        solidDraws_[0] = buildAirframeHullMesh();
        solidDraws_[1] = buildWingFragmentMesh();
        solidDraws_[2] = buildPanelShardMesh();
        solidDraws_[3] = buildBurningChunkMesh();
        for (const auto& draw : solidDraws_) sg->addChild(draw);

        solidTransform_ = vsg::MatrixTransform::create();
        solidTransform_->addChild(sg);
        solidSwitch_->addChild(false, solidTransform_);
    }

    // --- 2. Translucent sparks, burning debris smoke ribbons, flame cores & water splash plumes ---
    {
        vsg::Descriptors desc{vsg::DescriptorBuffer::create(trailQuads_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                              vsg::DescriptorBuffer::create(frameUbo_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER)};
        auto set = vsg::DescriptorSet::create(setLayout, desc);

        auto assembly = vsg::InputAssemblyState::create();
        assembly->topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        auto raster = vsg::RasterizationState::create();
        raster->cullMode = VK_CULL_MODE_NONE;
        auto depth = vsg::DepthStencilState::create();
        depth->depthTestEnable = VK_TRUE;
        depth->depthWriteEnable = VK_FALSE;
        depth->depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;
        auto blend = vsg::ColorBlendState::create();
        blend->attachments = {VkPipelineColorBlendAttachmentState{
            VK_TRUE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD, VK_BLEND_FACTOR_ONE,
            VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};

        vsg::GraphicsPipelineStates states{vsg::VertexInputState::create(), assembly, raster, vsg::MultisampleState::create(), blend, depth};
        auto sg = vsg::StateGroup::create();
        sg->add(vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(layout, trailStages, states)));
        sg->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, set));

        trailDraw_ = vsg::Draw::create(6, 0, 0, 0);
        sg->addChild(trailDraw_);

        trailTransform_ = vsg::MatrixTransform::create();
        trailTransform_->addChild(sg);
        trailSwitch_->addChild(false, trailTransform_);
    }

    valid_ = true;
}

void DebrisVisuals::clear() {
    smokeTrails_.clear();
    lastClock_ = -1e9;
    solidCount_ = 0;
    trailCount_ = 0;
    for (auto& d : solidDraws_)
        if (d) d->instanceCount = 0;
    if (trailDraw_) trailDraw_->instanceCount = 0;
    if (solidSwitch_) solidSwitch_->setAllChildren(false);
    if (trailSwitch_) trailSwitch_->setAllChildren(false);
}

void DebrisVisuals::update(const ipc::DebrisFrameV1& frame, const vsg::dvec3& eyeEcef, const vsg::dvec3& cameraRight,
                           const vsg::dvec3& cameraUp, const vsg::dvec3& sunDirEcef, float daylight, double simTime) {
    if (!valid_) return;

    const double clock = std::isfinite(simTime) ? simTime : frame.simTime;
    if (lastClock_ > -1e8 && (clock < lastClock_ - 0.05 || clock > lastClock_ + 10.0)) smokeTrails_.clear();
    lastClock_ = clock;

    // Eye-centered transform avoids ECEF float32 jitter anywhere on the globe.
    solidTransform_->matrix = vsg::translate(eyeEcef);
    trailTransform_->matrix = vsg::translate(eyeEcef);

    const vsg::dvec3 upLocal = vsg::length(eyeEcef) > 1.0 ? vsg::normalize(eyeEcef) : vsg::dvec3(0.0, 0.0, 1.0);
    const vsg::dvec3 sunN = vsg::length(sunDirEcef) > 1e-6 ? vsg::normalize(sunDirEcef) : vsg::dvec3(0.0, 0.0, 1.0);
    const vsg::dvec3 rightN = vsg::length(cameraRight) > 1e-6 ? vsg::normalize(cameraRight) : vsg::dvec3(1.0, 0.0, 0.0);
    const vsg::dvec3 upN = vsg::length(cameraUp) > 1e-6 ? vsg::normalize(cameraUp) : vsg::dvec3(0.0, 1.0, 0.0);
    const float timeWrap = static_cast<float>(std::fmod(std::max(clock, 0.0), 600.0));

    frameUbo_->set(0, vsg::vec4(static_cast<float>(sunN.x), static_cast<float>(sunN.y), static_cast<float>(sunN.z), std::clamp(daylight, 0.0f, 1.0f)));
    frameUbo_->set(1, vsg::vec4(static_cast<float>(rightN.x), static_cast<float>(rightN.y), static_cast<float>(rightN.z), 0.0f));
    frameUbo_->set(2, vsg::vec4(static_cast<float>(upN.x), static_cast<float>(upN.y), static_cast<float>(upN.z), 0.0f));
    frameUbo_->set(3, vsg::vec4(static_cast<float>(upLocal.x), static_cast<float>(upLocal.y), static_cast<float>(upLocal.z), timeWrap));
    frameUbo_->dirty();

    // Partition solid 3-D bodies into the 4 procedural mesh buckets and collect billboard quads.
    std::array<std::vector<const ipc::DebrisBodyWireV1*>, 4> buckets;
    std::vector<PendingQuad> quads;
    quads.reserve(512);

    static constexpr double kMaxRenderDistM = 35000.0;
    static constexpr double kSmokeIntervalS = 0.025;
    static constexpr double kSmokeStepDistM = 1.9;
    static constexpr double kSmokeLifetimeS = 6.5;
    static constexpr std::size_t kMaxSamplesPerBody = 44;

    for (const auto& b : frame.bodies) {
        const vsg::dvec3 pos(b.positionEcef[0], b.positionEcef[1], b.positionEcef[2]);
        const vsg::dvec3 rel = pos - eyeEcef;
        const double distSq = vsg::length2(rel);
        if (!std::isfinite(distSq) || distSq > kMaxRenderDistM * kMaxRenderDistM) continue;

        const auto arch = static_cast<ipc::DebrisArchetypeV1>(b.archetype);
        const float ageNorm = std::clamp(b.maxAgeS > 1e-4f ? b.ageS / b.maxAgeS : 0.0f, 0.0f, 1.0f);
        const float seed = hashSeed(b.id);
        const vsg::vec3 relF(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z));

        if (arch == ipc::DebrisArchetypeV1::SparkEmber) {
            const vsg::vec3 vel(static_cast<float>(b.velocityEcef[0]), static_cast<float>(b.velocityEcef[1]), static_cast<float>(b.velocityEcef[2]));
            const float speed = vsg::length(vel);
            const float radius = std::clamp(std::min(b.halfExtentsM[1], 0.14f), 0.055f, 0.15f);
            const float stretch = std::clamp(4.5f + speed * 0.08f, 4.5f, 18.0f);
            PendingQuad q;
            q.posAndRadius = vsg::vec4(relF.x, relF.y, relF.z, radius);
            q.velAndTemp = vsg::vec4(vel.x, vel.y, vel.z, std::max(b.temperatureK, 1500.0f));
            q.colorAlpha = vsg::vec4(1.0f, 0.72f, 0.28f, std::clamp(1.0f - ageNorm * 0.75f, 0.1f, 1.0f));
            q.params = vsg::vec4(1.0f, ageNorm, seed, stretch);
            q.distSq = static_cast<float>(distSq);
            q.additive = true;
            quads.push_back(q);
            continue;
        }

        if (arch == ipc::DebrisArchetypeV1::SplashPlume) {
            const float baseR = std::max({b.halfExtentsM[0], b.halfExtentsM[1], b.halfExtentsM[2], 1.8f});
            const float radius = baseR * (1.25f + 2.6f * std::sqrt(ageNorm));
            const float alpha = std::clamp((1.0f - ageNorm) * 0.78f, 0.0f, 0.85f);
            PendingQuad q;
            q.posAndRadius = vsg::vec4(relF.x, relF.y, relF.z, radius);
            q.velAndTemp = vsg::vec4(0.0f, 0.0f, 0.0f, 300.0f);
            q.colorAlpha = vsg::vec4(0.89f, 0.93f, 0.97f, alpha);
            q.params = vsg::vec4(2.0f, ageNorm, seed, 1.0f);
            q.distSq = static_cast<float>(distSq);
            q.additive = false;
            quads.push_back(q);
            continue;
        }

        // Solid 3-D tumbling chunk (AirframeHull=0, WingFragment=1, PanelShard=2, BurningChunk=3)
        const std::size_t bucketIdx = std::min<std::size_t>(static_cast<std::size_t>(b.archetype), 3u);
        buckets[bucketIdx].push_back(&b);

        const float extent = std::max({b.halfExtentsM[0], b.halfExtentsM[1], b.halfExtentsM[2], 0.25f});
        const bool burning = (b.flags & ipc::kDebrisFlagBurning) != 0u || b.temperatureK > 1100.0f;
        const bool smoking = (b.flags & ipc::kDebrisFlagSmoking) != 0u || b.smokeIntensity > 0.08f || burning;

        // Tight turbulent flame core clinging to actively burning chunks
        if (burning && b.temperatureK > 1000.0f) {
            const float flicker = 0.88f + 0.22f * std::sin(timeWrap * 25.0f + seed * 41.0f);
            const float flameR = std::clamp(extent * 0.95f * flicker, 0.35f, 2.0f);
            const float intensity = std::clamp((b.temperatureK - 950.0f) / 1100.0f, 0.15f, 1.0f);
            PendingQuad q;
            q.posAndRadius = vsg::vec4(relF.x, relF.y, relF.z, flameR);
            q.velAndTemp = vsg::vec4(0.0f, 0.0f, 0.0f, b.temperatureK);
            q.colorAlpha = vsg::vec4(1.0f, 0.38f, 0.06f, intensity * 0.72f);
            q.params = vsg::vec4(3.0f, ageNorm, seed + timeWrap * 0.5f, 1.0f);
            q.distSq = static_cast<float>(distSq);
            q.additive = true;
            quads.push_back(q);
        }

        // Lay dense, continuous velocity-stretched dark soot ribbons behind burning/smoking debris
        if (smoking && b.smokeIntensity > 0.06f) {
            auto& hist = smokeTrails_[b.id];
            const double movedM = hist.hasLastPos ? vsg::length(pos - hist.lastPosEcef) : kSmokeStepDistM;
            if (!hist.hasLastPos || movedM >= kSmokeStepDistM || (clock - hist.lastSampleTime) >= kSmokeIntervalS) {
                const int subSteps = hist.hasLastPos ? std::clamp(static_cast<int>(std::floor(movedM / kSmokeStepDistM)), 1, 10) : 1;
                const vsg::dvec3 startPos = hist.hasLastPos ? hist.lastPosEcef : pos;
                const double startT = hist.hasLastPos ? hist.lastSampleTime : clock;
                const float baseR = std::clamp(extent * 1.65f, 1.15f, 3.4f);
                const vsg::vec3 velF(static_cast<float>(b.velocityEcef[0]), static_cast<float>(b.velocityEcef[1]), static_cast<float>(b.velocityEcef[2]));
                const double stepDist = movedM / static_cast<double>(subSteps);
                const float stretch = std::clamp(static_cast<float>(stepDist / std::max(baseR * 0.75f, 0.5f)), 1.35f, 3.2f);
                for (int step = 1; step <= subSteps; ++step) {
                    const double frac = static_cast<double>(step) / static_cast<double>(subSteps);
                    SmokeSample s;
                    s.posEcef = startPos + (pos - startPos) * frac;
                    s.velEcef = velF;
                    s.birthTime = startT + (clock - startT) * frac;
                    s.baseRadius = baseR;
                    s.tempK = b.temperatureK;
                    s.opacity = std::clamp(b.smokeIntensity * 0.80f, 0.24f, 0.84f);
                    s.seed = hashSeed(b.id * 37u + static_cast<std::uint32_t>(hist.samples.size() * 23u + static_cast<std::size_t>(step) * 11u));
                    s.stretch = stretch;
                    hist.samples.push_back(s);
                    while (hist.samples.size() > kMaxSamplesPerBody) hist.samples.pop_front();
                }
                hist.lastPosEcef = pos;
                hist.lastSampleTime = clock;
                hist.hasLastPos = true;
            }
        }
    }

    // Write solid instance SSBO partitioned by archetype bucket
    std::uint32_t solidWritten = 0;
    for (std::size_t k = 0; k < 4; ++k) {
        const std::uint32_t first = solidWritten;
        for (const ipc::DebrisBodyWireV1* bp : buckets[k]) {
            if (solidWritten >= kMaxSolidInstances) break;
            const auto& b = *bp;
            const vsg::dvec3 rel(b.positionEcef[0] - eyeEcef.x, b.positionEcef[1] - eyeEcef.y, b.positionEcef[2] - eyeEcef.z);
            const auto arch = static_cast<ipc::DebrisArchetypeV1>(b.archetype);
            const vsg::vec3 col = archetypeColor(arch, b.id);
            const float scorch = std::clamp((b.temperatureK - 340.0f) / 900.0f + ((b.flags & ipc::kDebrisFlagBurning) ? 0.45f : 0.18f), 0.12f, 1.0f);

            const std::uint32_t base = solidWritten * 4u;
            solidInstances_->set(base + 0u, vsg::vec4(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z), b.temperatureK));
            solidInstances_->set(base + 1u, vsg::vec4(b.quatBodyToEcef[0], b.quatBodyToEcef[1], b.quatBodyToEcef[2], b.quatBodyToEcef[3]));
            solidInstances_->set(base + 2u, vsg::vec4(b.halfExtentsM[0], b.halfExtentsM[1], b.halfExtentsM[2], scorch));
            solidInstances_->set(base + 3u, vsg::vec4(col.x, col.y, col.z, static_cast<float>(k)));
            ++solidWritten;
        }
        solidDraws_[k]->firstInstance = first;
        solidDraws_[k]->instanceCount = solidWritten - first;
    }
    solidCount_ = solidWritten;
    if (solidCount_ > 0) solidInstances_->dirty();
    solidSwitch_->setAllChildren(solidCount_ > 0);

    // Harvest and age persistent smoke trails from all smoking/burning chunks
    for (auto it = smokeTrails_.begin(); it != smokeTrails_.end();) {
        auto& q = it->second.samples;
        while (!q.empty() && (clock - q.front().birthTime) > kSmokeLifetimeS) q.pop_front();
        if (q.empty()) {
            it = smokeTrails_.erase(it);
            continue;
        }
        for (const auto& s : q) {
            const double age = std::max(clock - s.birthTime, 0.0);
            const float ageNorm = static_cast<float>(std::clamp(age / kSmokeLifetimeS, 0.0, 1.0));
            // Gentle thermal buoyancy drift upward along local zenith
            const vsg::dvec3 driftPos = s.posEcef + upLocal * (age * 2.4);
            const vsg::dvec3 rel = driftPos - eyeEcef;
            const double distSq = vsg::length2(rel);
            if (!std::isfinite(distSq) || distSq > kMaxRenderDistM * kMaxRenderDistM) continue;
            const float radius = s.baseRadius * (1.0f + 3.2f * std::sqrt(ageNorm));
            // Rich dark charcoal hydrocarbon soot lightening gently to grey smoke as it disperses
            const float soot = std::clamp(0.045f + 0.11f * ageNorm, 0.04f, 0.22f);
            PendingQuad pq;
            pq.posAndRadius = vsg::vec4(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z), radius);
            pq.velAndTemp = vsg::vec4(s.velEcef.x, s.velEcef.y, s.velEcef.z, s.tempK);
            pq.colorAlpha = vsg::vec4(soot, soot * 0.98f, soot * 0.97f, s.opacity * (1.0f - 0.50f * ageNorm));
            pq.params = vsg::vec4(0.0f, ageNorm, s.seed, s.stretch);
            pq.distSq = static_cast<float>(distSq);
            pq.additive = false;
            quads.push_back(pq);
        }
        ++it;
    }

    // Sort opaque-ish translucent puffs (smoke, splash) back-to-front, with pure additive sparks/flames last.
    std::sort(quads.begin(), quads.end(), [](const PendingQuad& a, const PendingQuad& b) {
        if (a.additive != b.additive) return !a.additive;
        return a.distSq > b.distSq;
    });

    const std::uint32_t trailWritten = static_cast<std::uint32_t>(std::min<std::size_t>(quads.size(), kMaxTrailQuads));
    for (std::uint32_t i = 0; i < trailWritten; ++i) {
        const auto& q = quads[i];
        const std::uint32_t base = i * 4u;
        trailQuads_->set(base + 0u, q.posAndRadius);
        trailQuads_->set(base + 1u, q.velAndTemp);
        trailQuads_->set(base + 2u, q.colorAlpha);
        trailQuads_->set(base + 3u, q.params);
    }
    trailCount_ = trailWritten;
    trailDraw_->instanceCount = trailCount_;
    if (trailCount_ > 0) trailQuads_->dirty();
    trailSwitch_->setAllChildren(trailCount_ > 0);
}

} // namespace fsim::world
