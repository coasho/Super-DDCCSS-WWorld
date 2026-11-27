#include "visualization/vfx/effects/DebrisVisuals.h"

#include "foundation/core/Log.h"
#include "visualization/scene/Clouds.h"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <manifold/manifold.h>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace fsim::world {

namespace {

float hashSeed(std::uint32_t id) noexcept {
    std::uint32_t x = id * 747796405u + 2891336453u;
    x = ((x >> ((x >> 28u) + 4u)) ^ x) * 277803737u;
    x = (x >> 22u) ^ x;
    return static_cast<float>(x & 0xFFFFu) / 65535.0f;
}

// Build 6 distinct watertight fractured aerospace skin/rib shards using Google's elalish/manifold 3.4.1
// (Apache-2.0): starts from curved hollow aerospace fuselage/wing shells and carves genuine 3-D Voronoi
// cells via Manifold::TrimByPlane + organic tearing Warp, automatically distinguishing original painted
// exterior skin (low fracture glow) from newly sliced charred cross-section rims (high edge glow)!
vsg::ref_ptr<vsg::VertexDraw> buildManifoldVoronoiShard(int variant) {
    using manifold::Manifold;
    using manifold::vec3;

    // Construct an aerospace base blank: curved hollow fuselage shell or swept airfoil panel.
    Manifold blank;
    if (variant % 3 == 0) {
        // Curved thin-walled fuselage skin section (outer cylinder minus inner cylinder)
        Manifold outer = Manifold::Cylinder(2.2, 1.05, 0.92, 20, true).Rotate(0.0, 90.0, 0.0);
        Manifold inner = Manifold::Cylinder(2.6, 0.88, 0.76, 20, true).Rotate(0.0, 90.0, 0.0);
        blank = outer - inner;
    } else if (variant % 3 == 1) {
        // Tapered aerospace wing skin panel with internal spar stiffener rib
        Manifold skin = Manifold::Cube(vec3(2.2, 2.0, 0.22), true);
        Manifold rib = Manifold::Cube(vec3(2.2, 0.14, 0.48), true).Translate(vec3(0.0, -0.25, 0.12));
        blank = skin + rib;
    } else {
        // Curved aerodynamic cowling / fairing panel with beveled bulkhead flange
        Manifold shell = Manifold::Sphere(1.25, 16).Scale(vec3(1.15, 0.95, 0.45));
        Manifold hollow = Manifold::Sphere(1.06, 16).Scale(vec3(1.12, 0.92, 0.34)).Translate(vec3(0.0, 0.0, 0.10));
        blank = shell - hollow;
    }

    // Carve a genuine Voronoi polyhedral fracture cell out of the aerospace blank using TrimByPlane!
    struct PlaneCut {
        vec3 n;
        double d;
    };
    const double phase = static_cast<double>(variant) * 1.17 + 0.35;
    std::vector<PlaneCut> cuts;
    cuts.reserve(6);
    for (int k = 0; k < 5; ++k) {
        const double a = phase + static_cast<double>(k) * 1.256637; // ~72 deg Voronoi neighbors
        const double tilt = 0.25 * std::sin(phase * 1.9 + static_cast<double>(k) * 2.1);
        const double nx = std::cos(a) * std::cos(tilt);
        const double ny = std::sin(a) * std::cos(tilt);
        const double nz = std::sin(tilt);
        const double offset = -0.35 - 0.38 * std::abs(std::sin(phase + static_cast<double>(k) * 1.7));
        cuts.push_back({vec3(nx, ny, nz), offset});
    }
    Manifold shard = blank;
    for (const auto& c : cuts) {
        Manifold next = shard.TrimByPlane(c.n, c.d);
        if (!next.IsEmpty() && next.NumTri() >= 8) shard = std::move(next);
    }

    // Apply subtle metal buckling/tearing warp via Manifold::Warp so no shard looks machine-cut flat.
    const double sSeed = static_cast<double>(variant) * 3.71;
    shard = shard.Warp([sSeed](vec3& p) {
        const double bend = 0.09 * std::sin(p.x * 2.6 + sSeed) * std::cos(p.y * 2.2 - sSeed * 0.7);
        p.z += bend;
        p.x += 0.05 * std::sin(p.y * 3.8 + sSeed);
    });

    const manifold::MeshGL mesh = shard.GetMeshGL();
    const std::size_t nTri = mesh.NumTri();
    const std::uint32_t stride = mesh.numProp;

    // Normalize shard to fit [-1, 1] unit half-extents centered at its geometric centroid.
    vsg::vec3 bmin(1e9f, 1e9f, 1e9f), bmax(-1e9f, -1e9f, -1e9f);
    for (std::size_t i = 0; i < mesh.NumVert(); ++i) {
        const vsg::vec3 p(mesh.vertProperties[i * stride + 0], mesh.vertProperties[i * stride + 1], mesh.vertProperties[i * stride + 2]);
        bmin.x = std::min(bmin.x, p.x); bmin.y = std::min(bmin.y, p.y); bmin.z = std::min(bmin.z, p.z);
        bmax.x = std::max(bmax.x, p.x); bmax.y = std::max(bmax.y, p.y); bmax.z = std::max(bmax.z, p.z);
    }
    const vsg::vec3 ctr = (bmin + bmax) * 0.5f;
    const vsg::vec3 half = vsg::vec3(std::max((bmax.x - bmin.x) * 0.5f, 1e-4f),
                                     std::max((bmax.y - bmin.y) * 0.5f, 1e-4f),
                                     std::max((bmax.z - bmin.z) * 0.5f, 1e-4f));

    std::vector<vsg::vec3> positions;
    std::vector<vsg::vec3> normals;
    std::vector<vsg::vec2> meta;
    positions.reserve(nTri * 3);
    normals.reserve(nTri * 3);
    meta.reserve(nTri * 3);

    for (std::size_t t = 0; t < nTri; ++t) {
        const std::uint32_t i0 = mesh.triVerts[t * 3 + 0];
        const std::uint32_t i1 = mesh.triVerts[t * 3 + 1];
        const std::uint32_t i2 = mesh.triVerts[t * 3 + 2];
        const auto getP = [&](std::uint32_t idx) {
            return vsg::vec3((mesh.vertProperties[idx * stride + 0] - ctr.x) / half.x,
                             (mesh.vertProperties[idx * stride + 1] - ctr.y) / half.y,
                             (mesh.vertProperties[idx * stride + 2] - ctr.z) / half.z);
        };
        const vsg::vec3 a = getP(i0), b = getP(i1), c = getP(i2);
        vsg::vec3 n = vsg::cross(b - a, c - a);
        const float nLen = vsg::length(n);
        if (nLen < 1e-6f) continue;
        n /= nLen;

        // Detect newly sliced Voronoi fracture cross-section faces by alignment with TrimByPlane cut normals.
        bool cutFace = false;
        for (const auto& cut : cuts) {
            const float d = std::abs(n.x * static_cast<float>(cut.n.x) + n.y * static_cast<float>(cut.n.y) + n.z * static_cast<float>(cut.n.z));
            if (d > 0.90f) { cutFace = true; break; }
        }
        const float edgeGlow = cutFace ? 0.95f : 0.12f;
        const float tone = cutFace ? 0.32f : 0.95f;
        positions.push_back(a); positions.push_back(b); positions.push_back(c);
        normals.push_back(n); normals.push_back(n); normals.push_back(n);
        meta.emplace_back(edgeGlow, tone);
        meta.emplace_back(edgeGlow, tone);
        meta.emplace_back(edgeGlow, tone);
    }

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

vsg::vec3 archetypeColor(ipc::DebrisArchetypeV1 arch, std::uint32_t id) noexcept {
    const float h = hashSeed(id);
    switch (arch) {
    case ipc::DebrisArchetypeV1::AirframeHull:
        return vsg::vec3(0.42f + 0.10f * h, 0.45f + 0.10f * h, 0.49f + 0.08f * h); // tactical fighter grey
    case ipc::DebrisArchetypeV1::WingFragment:
        return vsg::vec3(0.46f + 0.10f * h, 0.49f + 0.10f * h, 0.53f + 0.08f * h);
    case ipc::DebrisArchetypeV1::PanelShard:
        return vsg::vec3(0.50f + 0.16f * h, 0.53f + 0.14f * h, 0.57f + 0.14f * h); // painted aluminum skin
    case ipc::DebrisArchetypeV1::BurningChunk:
        return vsg::vec3(0.20f + 0.07f * h, 0.19f + 0.07f * h, 0.21f + 0.07f * h); // dark charred frame
    default:
        return vsg::vec3(0.44f, 0.46f, 0.49f);
    }
}

vsg::dmat4 quatToMat4(const float q[4]) noexcept {
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double xx = x * x, yy = y * y, zz = z * z;
    const double xy = x * y, xz = x * z, yz = y * z;
    const double wx = w * x, wy = w * y, wz = w * z;
    // VSG dmat4 is column-major: column 0 = body +X in ECEF, column 1 = body +Y, column 2 = body +Z.
    return vsg::dmat4(
        1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz),       2.0 * (xz - wy),       0.0,
        2.0 * (xy - wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx),       0.0,
        2.0 * (xz + wy),       2.0 * (yz - wx),       1.0 - 2.0 * (xx + yy), 0.0,
        0.0,                   0.0,                   0.0,                   1.0);
}

const char* sectionNameForTag(ipc::DebrisPartTagV1 tag) noexcept {
    switch (tag) {
    case ipc::DebrisPartTagV1::SectionNose:      return "nose";
    case ipc::DebrisPartTagV1::SectionCentre:    return "centre";
    case ipc::DebrisPartTagV1::SectionLeftWing:  return "left-wing";
    case ipc::DebrisPartTagV1::SectionRightWing: return "right-wing";
    case ipc::DebrisPartTagV1::SectionTail:      return "tail";
    default:                                     return nullptr;
    }
}

struct PendingQuad {
    vsg::vec4 posAndRadius;
    vsg::vec4 velAndTemp;
    vsg::vec4 colorAlpha;
    vsg::vec4 params;
    float distSq = 0.0f;
    int sortOrder = 1; // 0 = surface decals (scorch/foam), 1 = back-to-front volumes (smoke/water/dust), 2 = additive (flames/sparks)
};

} // namespace

DebrisVisuals::DebrisVisuals(vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders)
    : shaders_(shaders) {
    (void)options;
    solidRoot_ = vsg::Group::create();
    trailRoot_ = vsg::Group::create();
    frontTrailRoot_ = vsg::Group::create();
    solidSwitch_ = vsg::Switch::create();
    sectionPoolSwitch_ = vsg::Switch::create();
    trailSwitch_ = vsg::Switch::create();
    frontTrailSwitch_ = vsg::Switch::create();
    solidRoot_->addChild(solidSwitch_);
    solidRoot_->addChild(sectionPoolSwitch_);
    trailRoot_->addChild(trailSwitch_);
    frontTrailRoot_->addChild(frontTrailSwitch_);

    if (shaders.empty()) {
        LOG_WARN("world") << "no debris renderer: shader directory missing";
        return;
    }

    auto solidVert = loadCloudShader(shaders, "debris.vert", VK_SHADER_STAGE_VERTEX_BIT);
    auto solidFrag = loadCloudShader(shaders, "debris.frag", VK_SHADER_STAGE_FRAGMENT_BIT);
    vsg::ShaderStages solidStages{solidVert, solidFrag};
    auto compiler = vsg::ShaderCompiler::create();
    if (!solidVert || !solidFrag || !compiler->supported() || !compiler->compile(solidStages)) {
        LOG_WARN("world") << "debris solid shaders failed to compile; lightweight debris visuals disabled";
        return;
    }

    const VkShaderStageFlags both = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    vsg::DescriptorSetLayoutBindings bindings{{0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, both, nullptr},
                                              {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, both, nullptr}};
    auto setLayout = vsg::DescriptorSetLayout::create(bindings);
    auto layout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout}, vsg::PushConstantRanges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}});

    frameUbo_ = vsg::vec4Array::create(7, vsg::vec4(0.0f, 0.0f, 1.0f, 1.0f));
    frameUbo_->set(5, vsg::vec4(0.0f, 0.0f, -1.0f, -1.0f));
    frameUbo_->set(6, vsg::vec4(2.0f, 0.0f, 0.0f, 0.0f));
    solidInstances_ = vsg::vec4Array::create(kMaxSolidInstances * 4u, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    trailQuads_ = vsg::vec4Array::create(kMaxTrailQuads * 4u, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    frameUbo_->properties.dataVariance = vsg::DYNAMIC_DATA;
    solidInstances_->properties.dataVariance = vsg::DYNAMIC_DATA;
    trailQuads_->properties.dataVariance = vsg::DYNAMIC_DATA;

    // --- 1. Opaque sunlit + incandescent manifold3d Voronoi aircraft shard pipeline ---
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
        raster->cullMode = VK_CULL_MODE_NONE;
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

        for (std::uint32_t k = 0; k < kVoronoiBuckets; ++k) {
            solidDraws_[k] = buildManifoldVoronoiShard(static_cast<int>(k));
            sg->addChild(solidDraws_[k]);
        }

        solidTransform_ = vsg::MatrixTransform::create();
        solidTransform_->addChild(sg);
        solidSwitch_->addChild(false, solidTransform_);
    }

    // --- 2. Real aircraft GLB section breakup pool (eye-relative MatrixTransform -> compiled GLB section subgraph) ---
    {
        sectionPoolTransform_ = vsg::MatrixTransform::create();
        auto emptyGroup = vsg::Group::create();
        for (std::uint32_t i = 0; i < kMaxSectionPool; ++i) {
            auto sw = vsg::Switch::create();
            auto xf = vsg::MatrixTransform::create();
            xf->addChild(emptyGroup);
            sw->addChild(false, xf);
            sectionPoolTransform_->addChild(sw);
            sectionSlots_[i] = {sw, xf, emptyGroup};
        }
        sectionPoolSwitch_->addChild(false, sectionPoolTransform_);
    }

    valid_ = true;
}

bool DebrisVisuals::buildTrails(vsg::ref_ptr<vsg::ImageInfo> sceneDepth, vsg::ref_ptr<vsg::ImageInfo> flowFront) {
    if (!valid_ || !sceneDepth || shaders_.empty()) return false;
    auto trailVert = loadCloudShader(shaders_, "debris_trail.vert", VK_SHADER_STAGE_VERTEX_BIT);
    auto trailFrag = loadCloudShader(shaders_, "debris_trail.frag", VK_SHADER_STAGE_FRAGMENT_BIT);
    vsg::ShaderStages trailStages{trailVert, trailFrag};
    auto compiler = vsg::ShaderCompiler::create();
    if (!trailVert || !trailFrag || !compiler->supported() || !compiler->compile(trailStages)) {
        LOG_WARN("world") << "debris soft-particle trail shaders failed to compile";
        return false;
    }

    const VkShaderStageFlags both = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    vsg::DescriptorSetLayoutBindings bindings{
        {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, both, nullptr},
        {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, both, nullptr},
        {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
        {3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}};
    auto setLayout = vsg::DescriptorSetLayout::create(bindings);
    auto layout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout}, vsg::PushConstantRanges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}});
    vsg::Descriptors desc{
        vsg::DescriptorBuffer::create(trailQuads_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
        vsg::DescriptorBuffer::create(frameUbo_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
        vsg::DescriptorImage::create(sceneDepth, 2, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
        vsg::DescriptorImage::create(flowFront ? flowFront : sceneDepth, 3, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER)};
    auto set = vsg::DescriptorSet::create(setLayout, desc);

    auto assembly = vsg::InputAssemblyState::create();
    assembly->topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    auto raster = vsg::RasterizationState::create();
    raster->cullMode = VK_CULL_MODE_NONE;
    // Runs in overScene (1-sample): reverse-Z depth testing & soft-particle fade happen cleanly against sceneDepthTex + flowFrontTex!
    auto depth = vsg::DepthStencilState::create();
    depth->depthTestEnable = VK_FALSE;
    depth->depthWriteEnable = VK_FALSE;
    auto blend = vsg::ColorBlendState::create();
    blend->attachments = {VkPipelineColorBlendAttachmentState{
        VK_TRUE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD, VK_BLEND_FACTOR_ONE,
        VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};

    vsg::GraphicsPipelineStates states{vsg::VertexInputState::create(), assembly, raster, vsg::MultisampleState::create(), blend, depth};
    auto bindPipe = vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(layout, trailStages, states));
    auto bindSet = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, set);

    // Back pass (drawn BEFORE FlowVolumes::compositeNode()): decals + quads behind active Flow cloud
    {
        auto sg = vsg::StateGroup::create();
        sg->add(bindPipe);
        sg->add(bindSet);
        trailDraw_ = vsg::Draw::create(6, 0, 0, 0);
        sg->addChild(trailDraw_);
        trailTransform_ = vsg::MatrixTransform::create();
        trailTransform_->addChild(sg);
        trailSwitch_->children.clear();
        trailSwitch_->addChild(true, trailTransform_); // kept ON until compiled() so VSG compiles the overScene pipeline
    }
    // Front pass (drawn AFTER FlowVolumes::compositeNode()): quads in front of active Flow cloud, sharing the same SSBO & pipeline!
    {
        auto sg = vsg::StateGroup::create();
        sg->add(bindPipe);
        sg->add(bindSet);
        frontTrailDraw_ = vsg::Draw::create(6, 0, 0, 0);
        sg->addChild(frontTrailDraw_);
        frontTrailTransform_ = vsg::MatrixTransform::create();
        frontTrailTransform_->addChild(sg);
        frontTrailSwitch_->children.clear();
        frontTrailSwitch_->addChild(true, frontTrailTransform_); // kept ON until compiled()
    }
    return true;
}

void DebrisVisuals::compiled() {
    if (trailSwitch_) trailSwitch_->setAllChildren(false);
    if (frontTrailSwitch_) frontTrailSwitch_->setAllChildren(false);
}

void DebrisVisuals::clear() {
    smokeTrails_.clear();
    lastClock_ = -1e9;
    solidCount_ = 0;
    trailCount_ = 0;
    for (auto& d : solidDraws_)
        if (d) d->instanceCount = 0;
    for (auto& s : sectionSlots_)
        if (s.sw) s.sw->setAllChildren(false);
    if (trailDraw_) trailDraw_->instanceCount = 0;
    if (frontTrailDraw_) frontTrailDraw_->instanceCount = 0;
    if (solidSwitch_) solidSwitch_->setAllChildren(false);
    if (sectionPoolSwitch_) sectionPoolSwitch_->setAllChildren(false);
    if (trailSwitch_) trailSwitch_->setAllChildren(false);
    if (frontTrailSwitch_) frontTrailSwitch_->setAllChildren(false);
}

void DebrisVisuals::update(const ipc::DebrisFrameV1& frame, const VehicleVisuals& visuals,
                           const std::unordered_map<std::uint32_t, std::size_t>& slotByVehicleId,
                           const vsg::dmat4& projection, std::uint32_t screenWidth, std::uint32_t screenHeight,
                           const vsg::dvec3& eyeEcef, const vsg::dvec3& cameraRight, const vsg::dvec3& cameraUp,
                           const vsg::dvec3& sunDirEcef, float daylight, double simTime,
                           const FlowInterleave* flowRef) {
    if (!valid_) return;

    const double clock = std::isfinite(simTime) ? simTime : frame.simTime;
    if (lastClock_ > -1e8 && (clock < lastClock_ - 0.03 || clock > lastClock_ + 4.0)) smokeTrails_.clear();
    lastClock_ = clock;

    solidTransform_->matrix = vsg::translate(eyeEcef);
    sectionPoolTransform_->matrix = vsg::translate(eyeEcef);
    if (trailTransform_) trailTransform_->matrix = vsg::translate(eyeEcef);
    if (frontTrailTransform_) frontTrailTransform_->matrix = vsg::translate(eyeEcef);

    const vsg::dvec3 upLocal = vsg::length(eyeEcef) > 1.0 ? vsg::normalize(eyeEcef) : vsg::dvec3(0.0, 0.0, 1.0);
    const vsg::dvec3 sunN = vsg::length(sunDirEcef) > 1e-6 ? vsg::normalize(sunDirEcef) : vsg::dvec3(0.0, 0.0, 1.0);
    const vsg::dvec3 rightN = vsg::length(cameraRight) > 1e-6 ? vsg::normalize(cameraRight) : vsg::dvec3(1.0, 0.0, 0.0);
    const vsg::dvec3 upN = vsg::length(cameraUp) > 1e-6 ? vsg::normalize(cameraUp) : vsg::dvec3(0.0, 1.0, 0.0);
    const float timeWrap = static_cast<float>(std::fmod(std::max(clock, 0.0), 600.0));

    // VSG projection matrix is column-major: projection[2][2] = P22, projection[3][2] = P32 (near plane in reverse-Z).
    const float p22 = static_cast<float>(projection[2][2]);
    const float p32 = static_cast<float>(projection[3][2]);
    const float invW = 1.0f / static_cast<float>(std::max<std::uint32_t>(screenWidth, 1u));
    const float invH = 1.0f / static_cast<float>(std::max<std::uint32_t>(screenHeight, 1u));

    const bool flowActive = (flowRef != nullptr && flowRef->active);
    const double flowCenterDistSq = flowActive ? vsg::length2(flowRef->centerEcef - eyeEcef) : -1.0;

    frameUbo_->set(0, vsg::vec4(static_cast<float>(sunN.x), static_cast<float>(sunN.y), static_cast<float>(sunN.z), std::clamp(daylight, 0.0f, 1.0f)));
    frameUbo_->set(1, vsg::vec4(static_cast<float>(rightN.x), static_cast<float>(rightN.y), static_cast<float>(rightN.z), 0.0f));
    frameUbo_->set(2, vsg::vec4(static_cast<float>(upN.x), static_cast<float>(upN.y), static_cast<float>(upN.z), 0.0f));
    frameUbo_->set(3, vsg::vec4(static_cast<float>(upLocal.x), static_cast<float>(upLocal.y), static_cast<float>(upLocal.z), timeWrap));
    frameUbo_->set(4, vsg::vec4(p22, p32, invW, invH));
    frameUbo_->set(5, flowActive ? flowRef->laid : vsg::vec4(0.0f, 0.0f, -1.0f, -1.0f));
    frameUbo_->set(6, vsg::vec4(flowActive ? std::max(flowRef->scale, 1.0f) : 2.0f, 0.0f, 0.0f, 0.0f));
    frameUbo_->dirty();

    std::array<std::vector<const ipc::DebrisBodyWireV1*>, kVoronoiBuckets> buckets;
    std::vector<PendingQuad> quads;
    quads.reserve(1024);
    std::uint32_t sectionsActive = 0;

    static constexpr double kMaxRenderDistM = 35000.0;
    static constexpr double kSmokeLifetimeS = 4.6;
    static constexpr std::size_t kMaxSamplesPerBody = 64;
    std::uint32_t smallShardSmokers = 0;

    for (const auto& b : frame.bodies) {
        const vsg::dvec3 pos(b.positionEcef[0], b.positionEcef[1], b.positionEcef[2]);
        const vsg::dvec3 rel = pos - eyeEcef;
        const double distSq = vsg::length2(rel);
        if (!std::isfinite(distSq) || distSq > kMaxRenderDistM * kMaxRenderDistM) continue;

        const auto arch = static_cast<ipc::DebrisArchetypeV1>(b.archetype);
        const auto partTag = ipc::debrisPartTagV1(b.flags);
        const float ageNorm = std::clamp(b.maxAgeS > 1e-4f ? b.ageS / b.maxAgeS : 0.0f, 0.0f, 1.0f);
        const float seed = hashSeed(b.id);
        const vsg::vec3 relF(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z));
        const vsg::vec3 velF(static_cast<float>(b.velocityEcef[0]), static_cast<float>(b.velocityEcef[1]), static_cast<float>(b.velocityEcef[2]));

        // --- 1. Incandescent Sparks ---
        if (arch == ipc::DebrisArchetypeV1::SparkEmber) {
            const float speed = vsg::length(velF);
            const float radius = std::clamp(std::min(b.halfExtentsM[1], 0.11f), 0.045f, 0.12f);
            const float stretch = std::clamp(5.0f + speed * 0.09f, 5.0f, 20.0f);
            PendingQuad q;
            q.posAndRadius = vsg::vec4(relF.x, relF.y, relF.z, radius);
            q.velAndTemp = vsg::vec4(velF.x, velF.y, velF.z, std::max(b.temperatureK, 1600.0f));
            q.colorAlpha = vsg::vec4(1.0f, 0.78f, 0.35f, std::clamp(1.0f - ageNorm * 0.8f, 0.1f, 1.0f));
            q.params = vsg::vec4(1.0f, ageNorm, seed, stretch);
            q.distSq = static_cast<float>(distSq);
            q.sortOrder = 2;
            quads.push_back(q);
            continue;
        }

        // --- 2. Splash / Dust / Ocean Foam Wake / Charred Skid Furrow ---
        if (arch == ipc::DebrisArchetypeV1::SplashPlume) {
            const float baseR = std::max({b.halfExtentsM[0], b.halfExtentsM[1], 1.6f});
            if (partTag == ipc::DebrisPartTagV1::ScorchMark) {
                PendingQuad q;
                q.posAndRadius = vsg::vec4(relF.x, relF.y, relF.z, b.halfExtentsM[1]);
                q.velAndTemp = vsg::vec4(velF.x, velF.y, velF.z, 300.0f);
                q.colorAlpha = vsg::vec4(0.055f, 0.048f, 0.042f, 0.80f * (1.0f - 0.45f * ageNorm));
                q.params = vsg::vec4(6.0f, ageNorm, seed, std::max(b.halfExtentsM[0] / std::max(b.halfExtentsM[1], 0.5f), 1.0f));
                q.distSq = static_cast<float>(distSq);
                q.sortOrder = 0;
                quads.push_back(q);
                continue;
            }
            if (partTag == ipc::DebrisPartTagV1::FoamWake) {
                // Expanding subtle circular ocean splash ring / lacy seafoam collar (never a bright opaque paint stripe!).
                const float widthBase = std::max(b.halfExtentsM[1], 1.5f);
                const float bloom = std::pow(std::min(ageNorm * 5.0f, 1.0f), 0.55f);
                const float radius = widthBase * (0.40f + 1.25f * bloom + 0.55f * std::sqrt(ageNorm));
                const float foamStrength = std::clamp(b.smokeIntensity, 0.18f, 0.65f);
                PendingQuad q;
                q.posAndRadius = vsg::vec4(relF.x, relF.y, relF.z, radius);
                q.velAndTemp = vsg::vec4(velF.x, velF.y, velF.z, 288.0f);
                q.colorAlpha = vsg::vec4(0.92f, 0.96f, 0.99f, foamStrength);
                q.params = vsg::vec4(4.0f, ageNorm, seed, 1.18f);
                q.distSq = static_cast<float>(distSq);
                q.sortOrder = 0;
                quads.push_back(q);
                continue;
            }
            if (partTag == ipc::DebrisPartTagV1::DustPlume) {
                const float radius = baseR * (0.95f + 1.35f * std::sqrt(ageNorm));
                PendingQuad q;
                q.posAndRadius = vsg::vec4(relF.x, relF.y, relF.z, radius);
                q.velAndTemp = vsg::vec4(velF.x, velF.y, velF.z, 300.0f);
                q.colorAlpha = vsg::vec4(0.36f, 0.31f, 0.24f, 0.36f); // soft seamless earth-dust apron around crater base
                q.params = vsg::vec4(5.0f, ageNorm, seed, 1.0f);
                q.distSq = static_cast<float>(distSq);
                q.sortOrder = 1;
                quads.push_back(q);
                continue;
            }
            // Grounded hydrodynamic ocean water curtain (locked upright to zenith, NEVER flips at apex!):
            // halfExtentsM[0] = horizontal width radius (m), halfExtentsM[2] = current vertical half-height (m).
            const float widthR = std::clamp(b.halfExtentsM[0] * (1.0f + 0.50f * ageNorm), 2.2f, 9.2f);
            const float halfH = std::clamp(b.halfExtentsM[2], 1.4f, 22.0f);
            const float aspect = std::clamp(halfH / std::max(widthR, 0.5f), 0.90f, 4.4f);
            PendingQuad q;
            q.posAndRadius = vsg::vec4(relF.x, relF.y, relF.z, widthR);
            q.velAndTemp = vsg::vec4(velF.x, velF.y, velF.z, 288.0f);
            q.colorAlpha = vsg::vec4(0.95f, 0.98f, 1.00f, 0.58f); // fuller frothy sunlit seawater plume!
            q.params = vsg::vec4(2.0f, ageNorm, seed, aspect);
            q.distSq = static_cast<float>(distSq);
            q.sortOrder = 1;
            quads.push_back(q);
            continue;
        }

        // --- 3. Solid 3-D Real Aircraft GLB Section OR manifold3d Voronoi Shard ---
        bool renderedAsRealSection = false;
        if (const char* secName = sectionNameForTag(partTag); secName && sectionsActive < kMaxSectionPool) {
            const auto slotIt = slotByVehicleId.find(b.ownerVehicleId);
            if (slotIt != slotByVehicleId.end()) {
                const auto sections = visuals.sectionVisualsOf(slotIt->second);
                for (const auto& sv : sections) {
                    if (sv.section == secName && sv.node) {
                        auto& slot = sectionSlots_[sectionsActive++];
                        slot.xf->matrix = vsg::translate(rel) * quatToMat4(b.quatBodyToEcef);
                        if (slot.boundChild != sv.node) {
                            slot.xf->children = {sv.node};
                            slot.boundChild = sv.node;
                        }
                        slot.sw->setAllChildren(true);
                        renderedAsRealSection = true;
                        break;
                    }
                }
            }
        }
        if (!renderedAsRealSection) {
            std::size_t bucketIdx = 0;
            const std::uint8_t tagNum = static_cast<std::uint8_t>(partTag);
            if (tagNum >= static_cast<std::uint8_t>(ipc::DebrisPartTagV1::VoronoiShard0) &&
                tagNum <= static_cast<std::uint8_t>(ipc::DebrisPartTagV1::VoronoiShard5)) {
                bucketIdx = static_cast<std::size_t>(tagNum - static_cast<std::uint8_t>(ipc::DebrisPartTagV1::VoronoiShard0));
            } else {
                bucketIdx = static_cast<std::size_t>(b.id % kVoronoiBuckets);
            }
            buckets[bucketIdx].push_back(&b);
        }

        // --- 4. Tight Flame Core + Continuous Pitch-Black Volumetric Smoke Ribbon ---
        const float extent = std::max({b.halfExtentsM[0], b.halfExtentsM[1], b.halfExtentsM[2], 0.25f});
        const bool burning = (b.flags & ipc::kDebrisFlagBurning) != 0u || b.temperatureK > 1150.0f;
        const bool smoking = (b.flags & ipc::kDebrisFlagSmoking) != 0u || b.smokeIntensity > 0.12f || burning;
        const bool waterChunk = (b.flags & ipc::kDebrisFlagWater) != 0u;

        // Tight blazing flame tongues ONLY on burning major airframe sections (zero boxy orange stickers on small shards!).
        if (renderedAsRealSection && burning && !waterChunk && b.temperatureK > 1150.0f && b.ageS < 1.8f) {
            const float flicker = 0.85f + 0.25f * std::sin(timeWrap * 28.0f + seed * 41.0f);
            const float flameR = std::clamp(extent * 0.55f * flicker, 0.45f, 1.45f);
            const float intensity = std::clamp((b.temperatureK - 1100.0f) / 900.0f, 0.20f, 1.0f) * (1.0f - b.ageS / 2.2f);
            PendingQuad q;
            q.posAndRadius = vsg::vec4(relF.x, relF.y, relF.z, flameR);
            q.velAndTemp = vsg::vec4(0.0f, 0.0f, 0.0f, b.temperatureK);
            q.colorAlpha = vsg::vec4(1.0f, 0.44f, 0.06f, intensity * 0.65f);
            q.params = vsg::vec4(3.0f, ageNorm, seed + timeWrap * 0.7f, 1.0f);
            q.distSq = static_cast<float>(distSq);
            q.sortOrder = 2;
            quads.push_back(q);
        }

        // Billowing turbulent volumetric black smoke plumes behind burning major sections + top 2 largest burning shards
        // (densely overlapping soft cloud puffs with 3D vortex curl drift -> continuous undulating smoke, ZERO polka dots, pipes, or bricks!).
        const bool allowSmoke = renderedAsRealSection || (burning && extent > 0.48f && smallShardSmokers < 2u);
        if (smoking && !waterChunk && b.smokeIntensity > 0.22f && allowSmoke) {
            if (!renderedAsRealSection) ++smallShardSmokers;
            auto& hist = smokeTrails_[b.id];
            const float baseR = renderedAsRealSection ? std::clamp(extent * 0.92f, 1.15f, 2.15f)
                                                      : std::clamp(extent * 0.80f, 0.58f, 0.95f);
            // Overlapping step spacing (~0.52 * baseR) keeps fast flying wing trails continuous while curling organically!
            // Stationary/slow burning wreckage emits only 1 curling rising puff every 0.16 s (never stacking 32 bricks!).
            const double stepTriggerM = renderedAsRealSection ? std::clamp(static_cast<double>(baseR) * 0.52, 0.72, 1.10)
                                                              : std::clamp(static_cast<double>(baseR) * 0.62, 0.52, 0.85);
            const double movedM = hist.hasLastPos ? vsg::length(pos - hist.lastPosEcef) : stepTriggerM;
            const double dtEmit = clock - hist.lastSampleTime;
            const bool emitByMove = hist.hasLastPos && movedM >= stepTriggerM;
            const bool emitByTime = !hist.hasLastPos || (movedM < 0.75 && dtEmit >= 0.16) || (dtEmit >= 0.08 && movedM >= 0.45);
            if (emitByMove || emitByTime) {
                const int subSteps = emitByMove ? std::clamp(static_cast<int>(std::ceil(movedM / stepTriggerM)), 1, 12) : 1;
                const vsg::dvec3 startPos = hist.hasLastPos ? hist.lastPosEcef : pos;
                const double startT = hist.hasLastPos ? hist.lastSampleTime : clock;
                // Local tangent basis perpendicular to upLocal for swirling lateral curl drift.
                const vsg::dvec3 refAxis = std::abs(upLocal.z) < 0.9 ? vsg::dvec3(0.0, 0.0, 1.0) : vsg::dvec3(1.0, 0.0, 0.0);
                const vsg::dvec3 tA = vsg::normalize(vsg::cross(upLocal, refAxis));
                const vsg::dvec3 tB = vsg::normalize(vsg::cross(upLocal, tA));
                for (int step = 1; step <= subSteps; ++step) {
                    const double frac = static_cast<double>(step) / static_cast<double>(subSteps);
                    const std::uint32_t hIdx = b.id * 97u + static_cast<std::uint32_t>(hist.samples.size() * 31u + static_cast<std::size_t>(step) * 17u) +
                                               static_cast<std::uint32_t>(clock * 60.0) * 13u;
                    const float r0 = hashSeed(hIdx);
                    const float r1 = hashSeed(hIdx ^ 0x517cc1b7u);
                    const float r2 = hashSeed(hIdx ^ 0x9e3779b9u);
                    const float swirlAng = r0 * 6.2831853f;
                    const float swirlSpd = (renderedAsRealSection ? 0.85f : 0.55f) + 1.35f * r1;
                    const float riseSpd = 2.4f + 2.0f * r2;
                    const vsg::dvec3 drift = tA * (std::cos(swirlAng) * swirlSpd) +
                                             tB * (std::sin(swirlAng) * swirlSpd) +
                                             upLocal * riseSpd;
                    // Gentle turbulent spawn offset breaks straight-line uniformity into natural churning vortices.
                    const vsg::dvec3 jitter = tA * ((r1 - 0.5) * baseR * 0.42) + tB * ((r2 - 0.5) * baseR * 0.42);
                    SmokeSample s;
                    s.posEcef = startPos + (pos - startPos) * frac + jitter;
                    s.velEcef = velF;
                    s.driftEcef = vsg::vec3(static_cast<float>(drift.x), static_cast<float>(drift.y), static_cast<float>(drift.z));
                    s.birthTime = startT + (clock - startT) * frac;
                    s.baseRadius = baseR * (0.78f + 0.50f * r1);
                    s.tempK = b.temperatureK;
                    // Layered volumetric opacity: dense overlapping dark billows near the wreck, thinning organically downwind!
                    s.opacity = std::clamp(b.smokeIntensity * (renderedAsRealSection ? 0.46f : 0.28f), 0.16f, 0.52f);
                    s.seed = r0;
                    s.spin = (r2 < 0.5f ? -1.0f : 1.0f) * (0.45f + 0.65f * r1);
                    hist.samples.push_back(s);
                    while (hist.samples.size() > kMaxSamplesPerBody) hist.samples.pop_front();
                }
                hist.lastPosEcef = pos;
                hist.lastSampleTime = clock;
                hist.hasLastPos = true;
            }
        }
    }

    // Turn off unused GLB section slots cleanly.
    for (std::uint32_t i = sectionsActive; i < kMaxSectionPool; ++i)
        if (sectionSlots_[i].sw) sectionSlots_[i].sw->setAllChildren(false);
    sectionPoolSwitch_->setAllChildren(sectionsActive > 0);

    // Upload manifold3d Voronoi shard instances.
    std::uint32_t solidWritten = 0;
    for (std::size_t k = 0; k < kVoronoiBuckets; ++k) {
        const std::uint32_t first = solidWritten;
        for (const ipc::DebrisBodyWireV1* bp : buckets[k]) {
            if (solidWritten >= kMaxSolidInstances) break;
            const auto& b = *bp;
            const vsg::dvec3 rel(b.positionEcef[0] - eyeEcef.x, b.positionEcef[1] - eyeEcef.y, b.positionEcef[2] - eyeEcef.z);
            const auto arch = static_cast<ipc::DebrisArchetypeV1>(b.archetype);
            const vsg::vec3 col = archetypeColor(arch, b.id);
            const float scorch = std::clamp((b.temperatureK - 360.0f) / 900.0f + ((b.flags & ipc::kDebrisFlagBurning) ? 0.55f : 0.25f), 0.18f, 1.0f);

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
    solidCount_ = solidWritten + sectionsActive;
    if (solidWritten > 0) solidInstances_->dirty();
    solidSwitch_->setAllChildren(solidWritten > 0);

    // Harvest billowing turbulent black smoke clouds (curling & expanding organically with age!).
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
            // Turbulent 3D vortex drift + gentle velocity carry-through decays exponentially as air arrests the puff.
            const double carry = 0.08 * (1.0 - std::exp(-age * 4.0));
            const vsg::dvec3 driftPos = s.posEcef +
                                        vsg::dvec3(s.driftEcef.x, s.driftEcef.y, s.driftEcef.z) * (age * (0.65 + 0.55 * ageNorm)) +
                                        vsg::dvec3(s.velEcef.x, s.velEcef.y, s.velEcef.z) * carry;
            const vsg::dvec3 rel = driftPos - eyeEcef;
            const double distSq = vsg::length2(rel);
            if (!std::isfinite(distSq) || distSq > kMaxRenderDistM * kMaxRenderDistM) continue;
            // Tight overlapping root right behind torn metal (0.70x), swelling organically into billowing clouds (2.95x)!
            const float radius = s.baseRadius * (0.70f + 2.25f * std::pow(ageNorm, 0.62f));
            // True deep charcoal/pitch-black hydrocarbon soot (gently lightening with atmospheric mixing as it ages).
            const float soot = std::clamp(0.014f + 0.045f * ageNorm, 0.014f, 0.068f);
            PendingQuad pq;
            pq.posAndRadius = vsg::vec4(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z), radius);
            pq.velAndTemp = vsg::vec4(s.velEcef.x, s.velEcef.y, s.velEcef.z, s.tempK);
            pq.colorAlpha = vsg::vec4(soot, soot * 1.02f, soot * 1.06f, s.opacity * (1.0f - 0.68f * ageNorm));
            pq.params = vsg::vec4(0.0f, ageNorm, s.seed, s.spin);
            pq.distSq = static_cast<float>(distSq);
            pq.sortOrder = 1;
            quads.push_back(pq);
        }
        ++it;
    }

    // Partition into Back pass (drawn BEFORE Flow composite: decals + volumes/sparks behind Flow center)
    // and Front pass (drawn AFTER Flow composite: volumes/sparks in front of Flow center, softly interleaved via flowFrontTex!).
    std::vector<PendingQuad> backQuads;
    std::vector<PendingQuad> frontQuads;
    backQuads.reserve(quads.size());
    if (flowActive && frontTrailDraw_) frontQuads.reserve(quads.size() / 2 + 16);
    const float flowDistSqF = static_cast<float>(flowCenterDistSq);
    for (auto& q : quads) {
        if (flowActive && frontTrailDraw_ && q.sortOrder > 0 && q.distSq < flowDistSqF) {
            q.colorAlpha.a = -std::abs(q.colorAlpha.a); // negative alpha sign signals front-pass volumetric interleaving
            frontQuads.push_back(q);
        } else {
            backQuads.push_back(q);
        }
    }
    const auto quadCmp = [](const PendingQuad& a, const PendingQuad& b) {
        if (a.sortOrder != b.sortOrder) return a.sortOrder < b.sortOrder;
        return a.distSq > b.distSq;
    };
    std::sort(backQuads.begin(), backQuads.end(), quadCmp);
    std::sort(frontQuads.begin(), frontQuads.end(), quadCmp);

    const std::uint32_t backWritten = static_cast<std::uint32_t>(std::min<std::size_t>(backQuads.size(), kMaxTrailQuads));
    const std::uint32_t frontCap = (kMaxTrailQuads > backWritten) ? (kMaxTrailQuads - backWritten) : 0u;
    const std::uint32_t frontWritten = static_cast<std::uint32_t>(std::min<std::size_t>(frontQuads.size(), frontCap));
    for (std::uint32_t i = 0; i < backWritten; ++i) {
        const auto& q = backQuads[i];
        const std::uint32_t base = i * 4u;
        trailQuads_->set(base + 0u, q.posAndRadius);
        trailQuads_->set(base + 1u, q.velAndTemp);
        trailQuads_->set(base + 2u, q.colorAlpha);
        trailQuads_->set(base + 3u, q.params);
    }
    for (std::uint32_t i = 0; i < frontWritten; ++i) {
        const auto& q = frontQuads[i];
        const std::uint32_t base = (backWritten + i) * 4u;
        trailQuads_->set(base + 0u, q.posAndRadius);
        trailQuads_->set(base + 1u, q.velAndTemp);
        trailQuads_->set(base + 2u, q.colorAlpha);
        trailQuads_->set(base + 3u, q.params);
    }
    trailCount_ = backWritten + frontWritten;
    if (trailDraw_) {
        trailDraw_->firstInstance = 0u;
        trailDraw_->instanceCount = backWritten;
    }
    if (frontTrailDraw_) {
        frontTrailDraw_->firstInstance = backWritten;
        frontTrailDraw_->instanceCount = frontWritten;
    }
    if (trailCount_ > 0) trailQuads_->dirty();
    if (trailSwitch_) trailSwitch_->setAllChildren(backWritten > 0);
    if (frontTrailSwitch_) frontTrailSwitch_->setAllChildren(frontWritten > 0);
}

} // namespace fsim::world
