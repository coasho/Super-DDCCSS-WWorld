#include "visualization/vfx/effects/TacticalVolumes.h"

#include "foundation/core/Log.h"
#include "visualization/scene/Clouds.h"
#include "visualization/scene/Frames.h"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace fsim::world {

namespace {

constexpr double kPi = 3.14159265358979323846;

vsg::dvec3 bodyToEcef(const sim::VehicleState& s, const vsg::dvec3& b) noexcept {
    const double* R = s.rotationBodyToEcef;
    return vsg::dvec3(
        R[0] * b.x + R[1] * b.y + R[2] * b.z,
        R[3] * b.x + R[4] * b.y + R[5] * b.z,
        R[6] * b.x + R[7] * b.y + R[8] * b.z);
}

void localEnu(const vsg::dvec3& pos, vsg::dvec3& east, vsg::dvec3& north, vsg::dvec3& up) noexcept {
    up = localUp(pos);
    east = vsg::cross(vsg::dvec3(0.0, 0.0, 1.0), up);
    if (vsg::length(east) < 1e-6) east = vsg::dvec3(1.0, 0.0, 0.0);
    east = vsg::normalize(east);
    north = vsg::cross(up, east);
}

void orthoBasis(const vsg::dvec3& dir, vsg::dvec3& u, vsg::dvec3& v) noexcept {
    const vsg::dvec3 ref = std::abs(dir.z) < 0.9 ? vsg::dvec3(0.0, 0.0, 1.0) : vsg::dvec3(1.0, 0.0, 0.0);
    u = vsg::normalize(vsg::cross(ref, dir));
    v = vsg::normalize(vsg::cross(dir, u));
}

vsg::vec3 radarSideColor(TacticalVolumes::Side side, bool stt, bool illum) noexcept {
    if (side == TacticalVolumes::Side::Red) {
        if (illum) return vsg::vec3(1.00f, 0.22f, 0.16f);
        if (stt)   return vsg::vec3(0.98f, 0.30f, 0.20f);
        return vsg::vec3(0.94f, 0.22f, 0.26f); // Clean tactical crimson
    }
    if (side == TacticalVolumes::Side::Blue) {
        if (illum) return vsg::vec3(0.20f, 0.84f, 1.00f);
        if (stt)   return vsg::vec3(0.16f, 0.74f, 1.00f);
        return vsg::vec3(0.12f, 0.62f, 0.98f); // Clean tactical sapphire-blue
    }
    return stt ? vsg::vec3(0.56f, 0.84f, 0.96f) : vsg::vec3(0.32f, 0.68f, 0.86f);
}

vsg::vec3 weatherDbzLinear(float dbz) noexcept {
    if (dbz >= 52.0f) return vsg::vec3(0.85f, 0.14f, 0.85f);
    if (dbz >= 42.0f) return vsg::vec3(0.94f, 0.22f, 0.16f);
    if (dbz >= 32.0f) return vsg::vec3(0.95f, 0.74f, 0.16f);
    if (dbz >= 20.0f) return vsg::vec3(0.22f, 0.80f, 0.34f);
    return vsg::vec3(0.24f, 0.68f, 0.82f);
}

} // namespace

TacticalVolumes::TacticalVolumes(vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders)
    : shaders_(shaders) {
    (void)options;
    root_ = vsg::Group::create();
    switch_ = vsg::Switch::create();
    root_->addChild(switch_);
    if (shaders_.empty()) {
        LOG_WARN("world") << "no tactical volume renderer: shader directory missing";
        return;
    }
    valid_ = true;
}

TacticalVolumes::Side TacticalVolumes::inferSide(const std::string& callsign) noexcept {
    if (callsign.empty()) return Side::Neutral;
    std::string lo;
    lo.reserve(callsign.size());
    for (char ch : callsign) lo.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    if (lo.find("red") != std::string::npos || lo.find("bandit") != std::string::npos ||
        lo.find("hostile") != std::string::npos || lo.find("opfor") != std::string::npos ||
        lo.find("enemy") != std::string::npos || lo.find("flanker") != std::string::npos ||
        lo.find("fulcrum") != std::string::npos || lo.find("aggressor") != std::string::npos ||
        lo.find("su27") != std::string::npos || lo.find("su30") != std::string::npos ||
        lo.find("su35") != std::string::npos || lo.find("mig29") != std::string::npos ||
        lo.find("j10") != std::string::npos || lo.find("j11") != std::string::npos ||
        lo.find("j16") != std::string::npos || lo.find("j20") != std::string::npos) {
        return Side::Red;
    }
    if (lo.find("blue") != std::string::npos || lo.find("viper") != std::string::npos ||
        lo.find("eagle") != std::string::npos || lo.find("raptor") != std::string::npos ||
        lo.find("hornet") != std::string::npos || lo.find("lightning") != std::string::npos ||
        lo.find("friendly") != std::string::npos || lo.find("allied") != std::string::npos ||
        lo.find("awacs") != std::string::npos || lo.find("hawk") != std::string::npos ||
        lo.find("f15") != std::string::npos || lo.find("f16") != std::string::npos ||
        lo.find("f18") != std::string::npos || lo.find("f22") != std::string::npos ||
        lo.find("f35") != std::string::npos) {
        return Side::Blue;
    }
    return Side::Neutral;
}

bool TacticalVolumes::build(vsg::ref_ptr<vsg::ImageInfo> sceneDepth) {
    if (!valid_ || built_ || !sceneDepth || shaders_.empty()) return built_;

    auto vert = loadCloudShader(shaders_, "tactical_volume.vert", VK_SHADER_STAGE_VERTEX_BIT);
    auto frag = loadCloudShader(shaders_, "tactical_volume.frag", VK_SHADER_STAGE_FRAGMENT_BIT);
    vsg::ShaderStages stages{vert, frag};
    auto compiler = vsg::ShaderCompiler::create();
    if (!vert || !frag || !compiler->supported() || !compiler->compile(stages)) {
        LOG_WARN("world") << "VIS-M14 tactical volume shaders failed to compile; 3-D volumes disabled";
        valid_ = false;
        return false;
    }

    vertexStaging_ = vsg::vec4Array::create(kMaxVertices * 4u, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    vertexStaging_->properties.dataVariance = vsg::DYNAMIC_DATA;
    frameUbo_ = vsg::vec4Array::create(4u, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    frameUbo_->properties.dataVariance = vsg::DYNAMIC_DATA;

    const VkShaderStageFlags vs = VK_SHADER_STAGE_VERTEX_BIT;
    const VkShaderStageFlags fs = VK_SHADER_STAGE_FRAGMENT_BIT;
    auto setLayout = vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{
        {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, vs, nullptr},
        {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, fs, nullptr},
        {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr}});
    auto pipeLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout},
                                                  vsg::PushConstantRanges{{vs, 0, 128}});
    auto descSet = vsg::DescriptorSet::create(
        setLayout,
        vsg::Descriptors{
            vsg::DescriptorBuffer::create(vertexStaging_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
            vsg::DescriptorBuffer::create(frameUbo_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
            vsg::DescriptorImage::create(sceneDepth, 2, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER)});

    auto assembly = vsg::InputAssemblyState::create();
    assembly->topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    auto raster = vsg::RasterizationState::create();
    raster->cullMode = VK_CULL_MODE_NONE;
    auto depth = vsg::DepthStencilState::create();
    depth->depthTestEnable = VK_FALSE; // Soft reverse-Z depth tested against sceneDepthTex in fragment shader
    depth->depthWriteEnable = VK_FALSE;
    auto blend = vsg::ColorBlendState::create();
    blend->attachments = {VkPipelineColorBlendAttachmentState{
        VK_TRUE, VK_BLEND_FACTOR_SRC_ALPHA, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
        VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};
    vsg::GraphicsPipelineStates states{vsg::VertexInputState::create(), assembly, raster,
                                       vsg::MultisampleState::create(), blend, depth};

    auto sg = vsg::StateGroup::create();
    sg->add(vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(pipeLayout, stages, states)));
    sg->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipeLayout, 0, descSet));
    draw_ = vsg::Draw::create(0, 1, 0, 0);
    sg->addChild(draw_);

    transform_ = vsg::MatrixTransform::create();
    transform_->addChild(sg);

    switch_->children.clear();
    switch_->addChild(true, transform_);
    built_ = true;
    return true;
}

void TacticalVolumes::compiled() {
    if (switch_) switch_->setAllChildren(false);
}

void TacticalVolumes::pushVert(const vsg::dvec3& worldEcef, float rNorm,
                               const vsg::dvec3& normalEcef, float uCoord,
                               const vsg::vec4& color, const vsg::vec4& params) noexcept {
    if (cursor_ >= kMaxVertices) return;
    const vsg::dvec3 rel = worldEcef - eyeEcef_;
    const std::uint32_t base = cursor_ * 4u;
    vertexStaging_->set(base + 0u, vsg::vec4(static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(rel.z), rNorm));
    vertexStaging_->set(base + 1u, vsg::vec4(static_cast<float>(normalEcef.x), static_cast<float>(normalEcef.y), static_cast<float>(normalEcef.z), uCoord));
    vertexStaging_->set(base + 2u, color);
    vertexStaging_->set(base + 3u, params);
    ++cursor_;
}

void TacticalVolumes::pushTri(const vsg::dvec3& p0, float r0, const vsg::dvec3& n0, float u0,
                              const vsg::dvec3& p1, float r1, const vsg::dvec3& n1, float u1,
                              const vsg::dvec3& p2, float r2, const vsg::dvec3& n2, float u2,
                              const vsg::vec4& color, const vsg::vec4& params) noexcept {
    if (cursor_ + 3u > kMaxVertices) return;
    pushVert(p0, r0, n0, u0, color, params);
    pushVert(p1, r1, n1, u1, color, params);
    pushVert(p2, r2, n2, u2, color, params);
}

void TacticalVolumes::pushQuad(const vsg::dvec3& p00, float r00, const vsg::dvec3& n00, float u00,
                               const vsg::dvec3& p10, float r10, const vsg::dvec3& n10, float u10,
                               const vsg::dvec3& p11, float r11, const vsg::dvec3& n11, float u11,
                               const vsg::dvec3& p01, float r01, const vsg::dvec3& n01, float u01,
                               const vsg::vec4& color, const vsg::vec4& params) noexcept {
    if (cursor_ + 6u > kMaxVertices) return;
    pushTri(p00, r00, n00, u00, p10, r10, n10, u10, p11, r11, n11, u11, color, params);
    pushTri(p00, r00, n00, u00, p11, r11, n11, u11, p01, r01, n01, u01, color, params);
}

void TacticalVolumes::pushBeamRibbon(const vsg::dvec3& a, const vsg::dvec3& b, double halfWidthM,
                                     const vsg::vec4& color, float style, float aux1, float aux2) noexcept {
    const vsg::dvec3 delta = b - a;
    const double len = vsg::length(delta);
    if (len < 1.0) return;
    const vsg::dvec3 dir = delta / len;
    const vsg::dvec3 mid = (a + b) * 0.5;
    const vsg::dvec3 toEye = eyeEcef_ - mid;
    vsg::dvec3 side = vsg::cross(toEye, dir);
    const double sn = vsg::length(side);
    if (sn < 1e-6) {
        vsg::dvec3 u, v;
        orthoBasis(dir, u, v);
        side = u;
    } else {
        side /= sn;
    }
    const double distEye = std::max(vsg::length(toEye), 10.0);
    const double w = std::max(halfWidthM, distEye * 0.0014);
    const vsg::dvec3 nrm = vsg::normalize(toEye);
    const vsg::vec4 params(style, aux1, aux2, static_cast<float>(len));
    pushQuad(a - side * w, 0.0f, nrm, -1.0f,
             a + side * w, 0.0f, nrm,  1.0f,
             b + side * w, 1.0f, nrm,  1.0f,
             b - side * w, 1.0f, nrm, -1.0f,
             color, params);
}

void TacticalVolumes::pushRingRibbon(const vsg::dvec3& center, const vsg::dvec3& axisU, const vsg::dvec3& axisV,
                                     double radiusU, double radiusV, double halfWidthM, int segments,
                                     const vsg::vec4& color, float dashFreq, float dashSpeed) noexcept {
    const int n = std::clamp(segments, 12, 64);
    const vsg::dvec3 normal = vsg::normalize(vsg::cross(axisU, axisV));
    const double distEye = std::max(vsg::length(eyeEcef_ - center), 10.0);
    const double w = std::max(halfWidthM, distEye * 0.0016);
    const vsg::vec4 params(6.0f, dashFreq, dashSpeed, static_cast<float>(radiusU));
    for (int i = 0; i < n; ++i) {
        const float r0 = static_cast<float>(i) / static_cast<float>(n);
        const float r1 = static_cast<float>(i + 1) / static_cast<float>(n);
        const double a0 = 2.0 * kPi * r0;
        const double a1 = 2.0 * kPi * r1;
        const vsg::dvec3 dir0 = axisU * std::cos(a0) + axisV * std::sin(a0);
        const vsg::dvec3 dir1 = axisU * std::cos(a1) + axisV * std::sin(a1);
        const vsg::dvec3 p0 = center + axisU * (radiusU * std::cos(a0)) + axisV * (radiusV * std::sin(a0));
        const vsg::dvec3 p1 = center + axisU * (radiusU * std::cos(a1)) + axisV * (radiusV * std::sin(a1));
        pushQuad(p0 - dir0 * w, r0, normal, -1.0f,
                 p0 + dir0 * w, r0, normal,  1.0f,
                 p1 + dir1 * w, r1, normal,  1.0f,
                 p1 - dir1 * w, r1, normal, -1.0f,
                 color, params);
    }
}

void TacticalVolumes::emitRadars(const ipc::TacticalOverlayFrameV1& frame, Span<const sim::VehicleState> states,
                                 const std::vector<unsigned char>& alive, const std::vector<Side>& sides, int selectedSlot) {
    if (selectedSlot < 0) return;
    for (const auto& r : frame.radars) {
        // Strictly focused-vehicle only: zero god-view clutter from unrelated aircraft.
        if (static_cast<int>(r.slot) != selectedSlot) continue;
        if (r.slot >= states.size() || r.slot >= alive.size() || !alive[r.slot]) continue;

        const sim::VehicleState& st = states[r.slot];
        const vsg::dvec3 origin(st.positionEcef[0], st.positionEcef[1], st.positionEcef[2]);
        const vsg::dvec3 bFwd = bodyToEcef(st, vsg::dvec3(1.0, 0.0, 0.0));
        const vsg::dvec3 bRight = bodyToEcef(st, vsg::dvec3(0.0, 1.0, 0.0));
        const vsg::dvec3 bUp = bodyToEcef(st, vsg::dvec3(0.0, 0.0, -1.0));

        const Side side = (r.slot < sides.size()) ? sides[r.slot] : Side::Neutral;
        const bool stt = (r.flags & ipc::kTactRadarSttLock) != 0u;
        const bool illum = (r.flags & ipc::kTactRadarIlluminating) != 0u;
        const bool roto = (r.flags & ipc::kTactRadarRotodome) != 0u;
        const bool jammed = (r.flags & ipc::kTactRadarJammed) != 0u;

        const double rawRange = static_cast<double>(r.detectionRangeM > 100.0f ? r.detectionRangeM : r.rangeScaleM);
        const bool hasBurn = jammed && r.burnThroughRangeM > 100.0f && r.burnThroughRangeM < static_cast<float>(rawRange) * 0.96f;
        const double nomRange = hasBurn
                                    ? std::clamp(std::max(static_cast<double>(r.burnThroughRangeM) * 2.0,
                                                          static_cast<double>(r.burnThroughRangeM) + 9500.0),
                                                 14000.0, std::min(rawRange, 42000.0))
                                    : std::clamp(rawRange, 6000.0, 42000.0);
        const float burnFrac = hasBurn
                                   ? std::clamp(r.burnThroughRangeM / static_cast<float>(nomRange), 0.12f, 0.92f)
                                   : 1.0f;

        const vsg::vec3 rgb = radarSideColor(side, stt, illum);
        const vsg::vec4 col(rgb.x, rgb.y, rgb.z, 1.0f);

        if (roto) {
            vsg::dvec3 e, n, u;
            localEnu(origin, e, n, u);
            const double beamAz = std::atan2(std::sin(static_cast<double>(r.beamAzimuthRad)), std::cos(static_cast<double>(r.beamAzimuthRad)));
            const float beamNorm = static_cast<float>(beamAz / kPi);
            const vsg::vec4 params(0.0f, burnFrac, beamNorm, static_cast<float>(nomRange));
            constexpr int kSegs = 40;
            constexpr int kRings = 4;

            // Subtle translucent cylindrical rim skirt giving AWACS coverage genuine 3-D volume depth
            const double skirtHalfH = std::clamp(nomRange * 0.015, 160.0, 520.0);
            const vsg::vec4 skirtParams(7.0f, 1.0f, 1.0f, static_cast<float>(nomRange));
            const vsg::vec4 skirtCol(rgb.x, rgb.y, rgb.z, 0.85f);
            for (int i = 0; i < kSegs; ++i) {
                const float t0 = static_cast<float>(i) / static_cast<float>(kSegs);
                const float t1 = static_cast<float>(i + 1) / static_cast<float>(kSegs);
                const double a0 = -kPi + 2.0 * kPi * t0;
                const double a1 = -kPi + 2.0 * kPi * t1;
                const vsg::dvec3 d0 = e * std::sin(a0) + n * std::cos(a0);
                const vsg::dvec3 d1 = e * std::sin(a1) + n * std::cos(a1);
                pushQuad(origin + d0 * nomRange - u * skirtHalfH, 0.5f, d0, -1.0f,
                         origin + d1 * nomRange - u * skirtHalfH, 0.5f, d1, -1.0f,
                         origin + d1 * nomRange + u * skirtHalfH, 0.5f, d1,  1.0f,
                         origin + d0 * nomRange + u * skirtHalfH, 0.5f, d0,  1.0f,
                         skirtCol, skirtParams);
            }

            for (int ri = 0; ri < kRings; ++ri) {
                const float r0 = static_cast<float>(ri) / static_cast<float>(kRings);
                const float r1 = static_cast<float>(ri + 1) / static_cast<float>(kRings);
                for (int i = 0; i < kSegs; ++i) {
                    const double a0 = -kPi + (2.0 * kPi * i) / kSegs;
                    const double a1 = -kPi + (2.0 * kPi * (i + 1)) / kSegs;
                    const float u0 = static_cast<float>(a0 / kPi);
                    const float u1 = static_cast<float>(a1 / kPi);
                    const vsg::dvec3 d0 = e * std::sin(a0) + n * std::cos(a0);
                    const vsg::dvec3 d1 = e * std::sin(a1) + n * std::cos(a1);
                    pushQuad(origin + d0 * (nomRange * r0), r0, u, u0,
                             origin + d1 * (nomRange * r0), r0, u, u1,
                             origin + d1 * (nomRange * r1), r1, u, u1,
                             origin + d0 * (nomRange * r1), r1, u, u0,
                             col, params);
                }
            }
        } else {
            const double azC = static_cast<double>(r.azimuthCentreRad);
            const double azH = std::clamp(static_cast<double>(r.azimuthHalfWidthRad), 0.08, 1.25);
            const double elC = static_cast<double>(r.elevationCentreRad);
            const double elVisual = std::clamp(static_cast<double>(r.elevationHalfWidthRad), 0.050, 0.125);
            const double activeEnd = hasBurn ? static_cast<double>(r.burnThroughRangeM) : nomRange;
            const float beamAzNorm = std::clamp(static_cast<float>((static_cast<double>(r.beamAzimuthRad) - azC) / std::max(azH, 0.05)), -1.0f, 1.0f);
            const vsg::vec4 params(0.0f, burnFrac, beamAzNorm, static_cast<float>(nomRange));

            auto dirAt = [&](double az, double el) {
                return bFwd * (std::cos(el) * std::cos(az)) + bRight * (std::cos(el) * std::sin(az)) + bUp * std::sin(el);
            };
            auto azRightAt = [&](double az) {
                return -bFwd * std::sin(az) + bRight * std::cos(az);
            };
            auto elUpAt = [&](double az, double el) {
                return (-bFwd * std::cos(az) - bRight * std::sin(az)) * std::sin(el) + bUp * std::cos(el);
            };

            // 1. Subtle Translucent 3-D Volumetric Walls, Sloped Ceiling/Floor & Curved Portal Pane
            //    Drawn before the mid-plane deck and border rails so crisp strokes stay razor-sharp.
            const vsg::vec4 sideWallParams(7.0f, burnFrac, 0.0f, static_cast<float>(nomRange));
            const vsg::vec4 roofWallParams(7.0f, burnFrac, 0.5f, static_cast<float>(nomRange));
            const vsg::vec4 sideWallCol(rgb.x, rgb.y, rgb.z, 0.92f);
            const vsg::vec4 roofWallCol(rgb.x, rgb.y, rgb.z, 0.78f);

            // 1a. Left and right elevation side walls (bottom rail -> top rail)
            constexpr int kWallRadSteps = 6;
            for (const double sAz : {-1.0, 1.0}) {
                const double az = azC + azH * sAz;
                const vsg::dvec3 dTop = dirAt(az, elC + elVisual);
                const vsg::dvec3 dBot = dirAt(az, elC - elVisual);
                const vsg::dvec3 nOut = azRightAt(az) * sAz;
                for (int ri = 0; ri < kWallRadSteps; ++ri) {
                    const float rf0 = static_cast<float>(ri) / static_cast<float>(kWallRadSteps);
                    const float rf1 = static_cast<float>(ri + 1) / static_cast<float>(kWallRadSteps);
                    pushQuad(origin + dBot * (nomRange * rf0), rf0, nOut, -1.0f,
                             origin + dBot * (nomRange * rf1), rf1, nOut, -1.0f,
                             origin + dTop * (nomRange * rf1), rf1, nOut,  1.0f,
                             origin + dTop * (nomRange * rf0), rf0, nOut,  1.0f,
                             sideWallCol, sideWallParams);
                }
            }

            // 1b. Light top sloped ceiling & shaded bottom sloped floor shells
            constexpr int kRoofAzSteps = 16;
            constexpr int kRoofRadSteps = 4;
            for (const double sEl : {-1.0, 1.0}) {
                const double el = elC + elVisual * sEl;
                for (int ri = 0; ri < kRoofRadSteps; ++ri) {
                    const float rf0 = static_cast<float>(ri) / static_cast<float>(kRoofRadSteps);
                    const float rf1 = static_cast<float>(ri + 1) / static_cast<float>(kRoofRadSteps);
                    for (int ai = 0; ai < kRoofAzSteps; ++ai) {
                        const float u0 = -1.0f + 2.0f * static_cast<float>(ai) / static_cast<float>(kRoofAzSteps);
                        const float u1 = -1.0f + 2.0f * static_cast<float>(ai + 1) / static_cast<float>(kRoofAzSteps);
                        const double a0 = azC + azH * u0;
                        const double a1 = azC + azH * u1;
                        const vsg::dvec3 d0 = dirAt(a0, el);
                        const vsg::dvec3 d1 = dirAt(a1, el);
                        const vsg::dvec3 n0 = elUpAt(a0, el) * sEl;
                        const vsg::dvec3 n1 = elUpAt(a1, el) * sEl;
                        pushQuad(origin + d0 * (nomRange * rf0), rf0, n0, u0,
                                 origin + d1 * (nomRange * rf0), rf0, n1, u1,
                                 origin + d1 * (nomRange * rf1), rf1, n1, u1,
                                 origin + d0 * (nomRange * rf1), rf1, n0, u0,
                                 roofWallCol, roofWallParams);
                    }
                }
            }

            // 1c. Delicate curved optical-glass portal curtain at activeEnd (ultra-clear center, beveled rim)
            constexpr int kPortalSegs = 16;
            const vsg::vec4 portalPaneCol = hasBurn ? vsg::vec4(1.00f, 0.80f, 0.20f, 0.90f)
                                                    : vsg::vec4(rgb.x, rgb.y, rgb.z, 0.78f);
            const vsg::vec4 portalPaneParams(7.0f, 1.0f, 2.0f, static_cast<float>(activeEnd));
            for (int i = 0; i < kPortalSegs; ++i) {
                const float t0 = static_cast<float>(i) / static_cast<float>(kPortalSegs);
                const float t1 = static_cast<float>(i + 1) / static_cast<float>(kPortalSegs);
                const double a0 = azC + azH * (-1.0 + 2.0 * static_cast<double>(t0));
                const double a1 = azC + azH * (-1.0 + 2.0 * static_cast<double>(t1));
                const vsg::dvec3 pBot0 = origin + dirAt(a0, elC - elVisual) * activeEnd;
                const vsg::dvec3 pBot1 = origin + dirAt(a1, elC - elVisual) * activeEnd;
                const vsg::dvec3 pTop1 = origin + dirAt(a1, elC + elVisual) * activeEnd;
                const vsg::dvec3 pTop0 = origin + dirAt(a0, elC + elVisual) * activeEnd;
                const vsg::dvec3 n0 = dirAt(a0, elC);
                const vsg::dvec3 n1 = dirAt(a1, elC);
                pushQuad(pBot0, t0, n0, -1.0f,
                         pBot1, t1, n1, -1.0f,
                         pTop1, t1, n1,  1.0f,
                         pTop0, t0, n0,  1.0f,
                         portalPaneCol, portalPaneParams);
            }

            // 2. Clean mid-plane tactical sector deck (primary top-down / orbit readability)
            constexpr int kAzSteps = 32;
            constexpr int kRadSteps = 6;
            for (int ri = 0; ri < kRadSteps; ++ri) {
                const float rf0 = static_cast<float>(ri) / static_cast<float>(kRadSteps);
                const float rf1 = static_cast<float>(ri + 1) / static_cast<float>(kRadSteps);
                for (int ai = 0; ai < kAzSteps; ++ai) {
                    const float u0 = -1.0f + 2.0f * static_cast<float>(ai) / static_cast<float>(kAzSteps);
                    const float u1 = -1.0f + 2.0f * static_cast<float>(ai + 1) / static_cast<float>(kAzSteps);
                    const double a0 = azC + azH * u0;
                    const double a1 = azC + azH * u1;
                    const vsg::dvec3 d0 = dirAt(a0, elC);
                    const vsg::dvec3 d1 = dirAt(a1, elC);
                    pushQuad(origin + d0 * (nomRange * rf0), rf0, bUp, u0,
                             origin + d1 * (nomRange * rf0), rf0, bUp, u1,
                             origin + d1 * (nomRange * rf1), rf1, bUp, u1,
                             origin + d0 * (nomRange * rf1), rf1, bUp, u0,
                             col, params);
                }
            }

            // 3. True 3-D Stereoscopic Elevation Corridor Rails & Portal Frames:
            // Ensures the radar volume remains unmistakably 3-D even at grazing Chase angles behind the tail
            // (prevents flat-plane compression into an illegible thin horizontal line on the horizon).
            const vsg::vec4 railActive(rgb.x, rgb.y, rgb.z, 0.88f);
            const vsg::vec4 railLower(rgb.x, rgb.y, rgb.z, 0.54f);
            const vsg::vec4 railBlind(0.68f, 0.75f, 0.86f, 0.56f);
            const vsg::vec4 portalCol = hasBurn ? vsg::vec4(1.00f, 0.80f, 0.18f, 0.96f) : vsg::vec4(rgb.x, rgb.y, rgb.z, 0.92f);

            // 4 corner elevation rails from nose out to activeEnd (and dashed beyond burn-through)
            for (const double sAz : {-1.0, 1.0}) {
                const double az = azC + azH * sAz;
                const vsg::dvec3 dTop = dirAt(az, elC + elVisual);
                const vsg::dvec3 dBot = dirAt(az, elC - elVisual);
                pushBeamRibbon(origin, origin + dTop * activeEnd, 10.0, railActive, 6.0f, 0.0f, 0.0f);
                pushBeamRibbon(origin, origin + dBot * activeEnd, 8.5,  railLower,  6.0f, 0.0f, 0.0f);
                if (hasBurn && nomRange > activeEnd + 600.0) {
                    pushBeamRibbon(origin + dTop * activeEnd, origin + dTop * nomRange, 8.5, railBlind, 6.0f, 14.0f, 0.0f);
                    pushBeamRibbon(origin + dBot * activeEnd, origin + dBot * nomRange, 7.5, railBlind, 6.0f, 14.0f, 0.0f);
                }
            }

            // Crisp 3-D curved elevation portal frame at activeEnd (Gold when jammed = Burn-Through Gate!)
            const double portalW = hasBurn ? 14.0 : 11.0;
            for (int i = 0; i < kPortalSegs; ++i) {
                const double u0 = -1.0 + 2.0 * static_cast<double>(i) / static_cast<double>(kPortalSegs);
                const double u1 = -1.0 + 2.0 * static_cast<double>(i + 1) / static_cast<double>(kPortalSegs);
                const double a0 = azC + azH * u0;
                const double a1 = azC + azH * u1;
                pushBeamRibbon(origin + dirAt(a0, elC + elVisual) * activeEnd,
                               origin + dirAt(a1, elC + elVisual) * activeEnd, portalW, portalCol, 6.0f, 0.0f, 0.0f);
                pushBeamRibbon(origin + dirAt(a0, elC - elVisual) * activeEnd,
                               origin + dirAt(a1, elC - elVisual) * activeEnd, portalW * 0.85,
                               vsg::vec4(portalCol.x, portalCol.y, portalCol.z, portalCol.w * 0.72f), 6.0f, 0.0f, 0.0f);
            }
            for (const double sAz : {-1.0, 1.0}) {
                const double az = azC + azH * sAz;
                pushBeamRibbon(origin + dirAt(az, elC - elVisual) * activeEnd,
                               origin + dirAt(az, elC + elVisual) * activeEnd, portalW, portalCol, 6.0f, 0.0f, 0.0f);
                if (hasBurn && nomRange > activeEnd + 600.0) {
                    pushBeamRibbon(origin + dirAt(az, elC - elVisual) * nomRange,
                                   origin + dirAt(az, elC + elVisual) * nomRange, 8.0, railBlind, 6.0f, 6.0f, 0.0f);
                }
            }
            if (hasBurn && nomRange > activeEnd + 600.0) {
                for (int i = 0; i < kPortalSegs; ++i) {
                    const double u0 = -1.0 + 2.0 * static_cast<double>(i) / static_cast<double>(kPortalSegs);
                    const double u1 = -1.0 + 2.0 * static_cast<double>(i + 1) / static_cast<double>(kPortalSegs);
                    const double a0 = azC + azH * u0;
                    const double a1 = azC + azH * u1;
                    pushBeamRibbon(origin + dirAt(a0, elC + elVisual) * nomRange,
                                   origin + dirAt(a1, elC + elVisual) * nomRange, 8.0, railBlind, 6.0f, 4.0f, 0.0f);
                }
            }
        }

        // Clean target track line for the focused aircraft's radar
        if ((r.flags & ipc::kTactRadarHasTarget) != 0u) {
            const vsg::dvec3 tgt(r.targetEcef[0], r.targetEcef[1], r.targetEcef[2]);
            const vsg::vec3 lockRgb = radarSideColor(side, true, illum);
            if (stt || illum) {
                pushBeamRibbon(origin, tgt, 14.0, vsg::vec4(lockRgb.x, lockRgb.y, lockRgb.z, 0.92f), 6.0f, 0.0f, 0.0f);
            } else {
                pushBeamRibbon(origin, tgt, 9.0, vsg::vec4(lockRgb.x, lockRgb.y, lockRgb.z, 0.68f), 6.0f, 24.0f, 0.0f);
            }
        }
    }
}

void TacticalVolumes::emitJammers(const ipc::TacticalOverlayFrameV1& frame, Span<const sim::VehicleState> states,
                                  const std::vector<unsigned char>& alive, const std::vector<Side>& sides, int selectedSlot) {
    if (selectedSlot < 0 || static_cast<std::size_t>(selectedSlot) >= states.size() ||
        static_cast<std::size_t>(selectedSlot) >= alive.size() || !alive[static_cast<std::size_t>(selectedSlot)]) {
        return;
    }

    for (const auto& j : frame.jammers) {
        // Strictly focused-vehicle own jammer only (never leak an enemy jammer cone when focused on another side!).
        if (static_cast<int>(j.slot) != selectedSlot) continue;

        const vsg::dvec3 origin(j.originEcef[0], j.originEcef[1], j.originEcef[2]);
        vsg::dvec3 dir(j.boresightEcef[0], j.boresightEcef[1], j.boresightEcef[2]);
        const double dn = vsg::length(dir);
        if (dn < 1e-6) continue;
        dir /= dn;

        const bool allRound = (j.flags & ipc::kTactJamAllRound) != 0u;
        const bool comms = (j.flags & ipc::kTactJamComms) != 0u;
        const Side side = (j.slot < sides.size()) ? sides[j.slot] : Side::Red;
        const float emph = 1.0f;

        const vsg::vec3 rgb = comms ? vsg::vec3(0.95f, 0.56f, 0.14f)
                              : (side == Side::Blue ? vsg::vec3(0.46f, 0.30f, 0.96f)
                                                    : vsg::vec3(0.68f, 0.24f, 0.96f));
        const vsg::vec4 col(rgb.x, rgb.y, rgb.z, emph);
        const double rng = std::clamp(static_cast<double>(j.rangeM), 14000.0, 32000.0);
        const vsg::vec4 params(1.0f, j.frequencyGhz, comms ? 1.0f : 0.0f, static_cast<float>(rng));

        if (allRound) {
            vsg::dvec3 e, n, u;
            localEnu(origin, e, n, u);
            const double rad = rng * 0.45;
            constexpr int kLat = 10;
            constexpr int kLon = 24;
            for (int li = 0; li < kLat; ++li) {
                const double p0 = -0.5 * kPi + kPi * li / kLat;
                const double p1 = -0.5 * kPi + kPi * (li + 1) / kLat;
                const float r0 = static_cast<float>(li) / static_cast<float>(kLat);
                const float r1 = static_cast<float>(li + 1) / static_cast<float>(kLat);
                for (int ci = 0; ci < kLon; ++ci) {
                    const double t0 = 2.0 * kPi * ci / kLon;
                    const double t1 = 2.0 * kPi * (ci + 1) / kLon;
                    const float u0 = -1.0f + 2.0f * static_cast<float>(ci) / static_cast<float>(kLon);
                    const float u1 = -1.0f + 2.0f * static_cast<float>(ci + 1) / static_cast<float>(kLon);
                    auto sDir = [&](double p, double th) {
                        return (e * std::cos(th) + n * std::sin(th)) * std::cos(p) + u * std::sin(p);
                    };
                    const vsg::dvec3 d00 = sDir(p0, t0), d10 = sDir(p0, t1), d11 = sDir(p1, t1), d01 = sDir(p1, t0);
                    pushQuad(origin + d00 * rad, r0, d00, u0,
                             origin + d10 * rad, r0, d10, u1,
                             origin + d11 * rad, r1, d11, u1,
                             origin + d01 * rad, r1, d01, u0,
                             col, params);
                }
            }
        } else {
            vsg::dvec3 u, v;
            orthoBasis(dir, u, v);
            const double halfAng = std::clamp(static_cast<double>(j.halfAngleRad), 0.08, 0.85);
            const double sinH = std::sin(halfAng);
            const double cosH = std::cos(halfAng);
            constexpr int kSlices = 28;
            constexpr int kRings = 6;
            for (int ri = 0; ri < kRings; ++ri) {
                const float rf0 = static_cast<float>(ri) / static_cast<float>(kRings);
                const float rf1 = static_cast<float>(ri + 1) / static_cast<float>(kRings);
                const double d0 = rng * rf0;
                const double d1 = rng * rf1;
                const double rad0 = d0 * std::tan(halfAng);
                const double rad1 = d1 * std::tan(halfAng);
                for (int si = 0; si < kSlices; ++si) {
                    const double a0 = 2.0 * kPi * si / kSlices;
                    const double a1 = 2.0 * kPi * (si + 1) / kSlices;
                    const float uc0 = -1.0f + 2.0f * static_cast<float>(si) / static_cast<float>(kSlices);
                    const float uc1 = -1.0f + 2.0f * static_cast<float>(si + 1) / static_cast<float>(kSlices);
                    const vsg::dvec3 rDir0 = u * std::cos(a0) + v * std::sin(a0);
                    const vsg::dvec3 rDir1 = u * std::cos(a1) + v * std::sin(a1);
                    const vsg::dvec3 n0 = vsg::normalize(rDir0 * cosH - dir * sinH);
                    const vsg::dvec3 n1 = vsg::normalize(rDir1 * cosH - dir * sinH);
                    pushQuad(origin + dir * d0 + rDir0 * rad0, rf0, n0, uc0,
                             origin + dir * d0 + rDir1 * rad0, rf0, n1, uc1,
                             origin + dir * d1 + rDir1 * rad1, rf1, n1, uc1,
                             origin + dir * d1 + rDir0 * rad1, rf1, n0, uc0,
                             col, params);
                }
            }
        }
    }
}

void TacticalVolumes::emitEsm(const ipc::TacticalOverlayFrameV1& frame, const std::vector<Side>& sides, int selectedSlot) {
    (void)sides;
    if (selectedSlot < 0) return;
    for (const auto& e : frame.esmRays) {
        // Strictly focused-vehicle ESM/RWR only.
        if (static_cast<int>(e.observerSlot) != selectedSlot) continue;

        const vsg::dvec3 origin(e.originEcef[0], e.originEcef[1], e.originEcef[2]);
        vsg::dvec3 dir(e.rayDirEcef[0], e.rayDirEcef[1], e.rayDirEcef[2]);
        const double dn = vsg::length(dir);
        if (dn < 1e-6) continue;
        dir /= dn;

        const bool warn = (e.flags & ipc::kTactEsmWarning) != 0u;
        const bool hasGeo = (e.flags & ipc::kTactEsmHasGeo) != 0u;
        const bool comint = (e.flags & ipc::kTactEsmComint) != 0u;

        const vsg::vec3 rgb = warn   ? vsg::vec3(1.00f, 0.30f, 0.22f)
                            : comint ? vsg::vec3(0.20f, 0.90f, 0.64f)
                                     : vsg::vec3(0.98f, 0.80f, 0.22f);
        const double len = hasGeo
                               ? std::clamp(vsg::length(vsg::dvec3(e.geoPosEcef[0], e.geoPosEcef[1], e.geoPosEcef[2]) - origin), 500.0, 65000.0)
                               : std::clamp(static_cast<double>(e.rayLengthM), 7000.0, 22000.0);

        // Clean narrow dashed bearing line (never a thick fuzzy wedge!)
        pushBeamRibbon(origin, origin + dir * len, 12.0, vsg::vec4(rgb.x, rgb.y, rgb.z, 0.88f), 6.0f, 20.0f, 0.0f);

        // Clean geometric 3-D CRLB uncertainty ellipsoid at converged geolocation fix
        if (hasGeo && e.crlbCep50M > 1.0f) {
            const vsg::dvec3 geo(e.geoPosEcef[0], e.geoPosEcef[1], e.geoPosEcef[2]);
            vsg::dvec3 east, north, up;
            localEnu(geo, east, north, up);
            const double rx = std::clamp(static_cast<double>(e.crlbCep50M), 60.0, 25000.0);
            const double rz = std::clamp(static_cast<double>(e.crlbVertM > 1.0f ? e.crlbVertM : e.crlbCep50M * 0.65f), 45.0, 18000.0);
            const vsg::vec4 ellParams(2.0f, 1.0f, 0.0f, static_cast<float>(rx));
            constexpr int kLat = 10;
            constexpr int kLon = 20;
            for (int li = 0; li < kLat; ++li) {
                const double p0 = -0.5 * kPi + kPi * li / kLat;
                const double p1 = -0.5 * kPi + kPi * (li + 1) / kLat;
                const float r0 = static_cast<float>(li) / static_cast<float>(kLat);
                const float r1 = static_cast<float>(li + 1) / static_cast<float>(kLat);
                for (int ci = 0; ci < kLon; ++ci) {
                    const double t0 = 2.0 * kPi * ci / kLon;
                    const double t1 = 2.0 * kPi * (ci + 1) / kLon;
                    const float uc0 = -1.0f + 2.0f * static_cast<float>(ci) / static_cast<float>(kLon);
                    const float uc1 = -1.0f + 2.0f * static_cast<float>(ci + 1) / static_cast<float>(kLon);
                    auto pt = [&](double p, double th, vsg::dvec3& nOut) {
                        const vsg::dvec3 h = east * std::cos(th) + north * std::sin(th);
                        nOut = vsg::normalize(h * (std::cos(p) / rx) + up * (std::sin(p) / rz));
                        return geo + h * (rx * std::cos(p)) + up * (rz * std::sin(p));
                    };
                    vsg::dvec3 n00, n10, n11, n01;
                    const vsg::dvec3 q00 = pt(p0, t0, n00), q10 = pt(p0, t1, n10), q11 = pt(p1, t1, n11), q01 = pt(p1, t0, n01);
                    pushQuad(q00, r0, n00, uc0,
                             q10, r0, n10, uc1,
                             q11, r1, n11, uc1,
                             q01, r1, n01, uc0,
                             vsg::vec4(rgb.x, rgb.y, rgb.z, 1.0f), ellParams);
                }
            }
        }
    }
}

void TacticalVolumes::emitWeapons(const ipc::TacticalOverlayFrameV1& frame, Span<const sim::VehicleState> states,
                                  const std::vector<unsigned char>& alive, int selectedSlot) {
    if (selectedSlot < 0) return;
    for (const auto& w : frame.weapons) {
        if (static_cast<int>(w.slot) != selectedSlot) continue;
        if (w.slot >= states.size() || w.slot >= alive.size() || !alive[w.slot]) continue;

        const sim::VehicleState& st = states[w.slot];
        const vsg::dvec3 origin(st.positionEcef[0], st.positionEcef[1], st.positionEcef[2]);
        const vsg::dvec3 bFwd = bodyToEcef(st, vsg::dvec3(1.0, 0.0, 0.0));
        vsg::dvec3 east, north, up;
        localEnu(origin, east, north, up);

        if ((w.flags & ipc::kTactWeaponHasDlz) != 0u && w.rMaxM > 100.0f) {
            vsg::dvec3 aimDir = bFwd;
            const vsg::dvec3 tgt(w.targetEcef[0], w.targetEcef[1], w.targetEcef[2]);
            if (vsg::length2(tgt - origin) > 100.0) aimDir = vsg::normalize(tgt - origin);
            vsg::dvec3 horiz = aimDir - up * vsg::dot(aimDir, up);
            if (vsg::length(horiz) < 1e-4) horiz = north;
            horiz = vsg::normalize(horiz);
            const vsg::dvec3 right = vsg::normalize(vsg::cross(horiz, up));

            const double halfAng = 0.22; // ~13 deg clean ruler fan
            const vsg::dvec3 deckOrigin = origin - up * 35.0; // Subtle vertical stagger cleanly separates DLZ from radar deck
            auto emitFanBand = [&](double rInner, double rOuter, const vsg::vec4& col, float zoneKind) {
                if (!(rOuter > rInner + 20.0)) return;
                // Two-tier architectural 3-D side walls along left/right fan edges:
                //   uCoord in [-1, 0]: cool vertical drop-shadow skirt hanging beneath deckOrigin
                //   uCoord in [ 0, 1]: upper vertical side wall rising cleanly toward the gate arches
                const double riseIn  = std::clamp(rInner * 0.026, 65.0, 520.0) * 0.78;
                const double riseOut = std::clamp(rOuter * 0.026, 75.0, 520.0) * 0.78;
                const double dropIn  = std::clamp(rInner * 0.020, 55.0, 380.0);
                const double dropOut = std::clamp(rOuter * 0.020, 65.0, 380.0);
                const vsg::vec4 skirtParams(7.0f, 1.0f, 1.5f, static_cast<float>(rOuter));
                const vsg::vec4 skirtCol(col.x, col.y, col.z, col.w * 0.92f);
                for (const double s : {-1.0, 1.0}) {
                    const double a = s * halfAng;
                    const vsg::dvec3 dEdge = horiz * std::cos(a) + right * std::sin(a);
                    const vsg::dvec3 nOut = (-horiz * std::sin(a) + right * std::cos(a)) * s;
                    const vsg::dvec3 pIn  = deckOrigin + dEdge * rInner;
                    const vsg::dvec3 pOut = deckOrigin + dEdge * rOuter;
                    // Lower under-deck drop-shadow skirt (uCoord = -1 .. 0)
                    pushQuad(pIn  - up * dropIn,  0.0f, nOut, -1.0f,
                             pOut - up * dropOut, 1.0f, nOut, -1.0f,
                             pOut,                1.0f, nOut,  0.0f,
                             pIn,                 0.0f, nOut,  0.0f,
                             skirtCol, skirtParams);
                    // Upper vertical side wall (uCoord = 0 .. +1)
                    pushQuad(pIn,                 0.0f, nOut,  0.0f,
                             pOut,                1.0f, nOut,  0.0f,
                             pOut + up * riseOut, 1.0f, nOut,  1.0f,
                             pIn  + up * riseIn,  0.0f, nOut,  1.0f,
                             skirtCol, skirtParams);
                }

                const vsg::vec4 params(3.0f, zoneKind, 0.0f, static_cast<float>(rOuter));
                constexpr int kSegs = 20;
                for (int i = 0; i < kSegs; ++i) {
                    const float u0 = -1.0f + 2.0f * static_cast<float>(i) / static_cast<float>(kSegs);
                    const float u1 = -1.0f + 2.0f * static_cast<float>(i + 1) / static_cast<float>(kSegs);
                    const double a0 = halfAng * u0;
                    const double a1 = halfAng * u1;
                    const vsg::dvec3 d0 = horiz * std::cos(a0) + right * std::sin(a0);
                    const vsg::dvec3 d1 = horiz * std::cos(a1) + right * std::sin(a1);
                    pushQuad(deckOrigin + d0 * rInner, 0.0f, up, u0,
                             deckOrigin + d1 * rInner, 0.0f, up, u1,
                             deckOrigin + d1 * rOuter, 1.0f, up, u1,
                             deckOrigin + d0 * rOuter, 1.0f, up, u0,
                             col, params);
                }
            };

            // Upright 3-D range-gate brackets (shaded curtain pane + under-deck shadow drop + posts + arch bar)
            // so Rmin / NEZ / Rmax show unmistakable vertical thickness in Orbit, 3/4, and Chase cameras!
            auto emitUprightRangeGate = [&](double distM, double gateHalfWScale, const vsg::vec4& gateCol, double widthM) {
                if (!(distM > 150.0)) return;
                const double gAng = halfAng * gateHalfWScale;
                const double gateH = std::clamp(distM * 0.026, 65.0, 520.0);
                const double dropH = std::clamp(distM * 0.020, 55.0, 380.0);
                constexpr int kArcSegs = 12;
                const vsg::vec4 curtainParams(7.0f, 1.0f, 1.5f, static_cast<float>(distM));
                const vsg::vec4 curtainCol(gateCol.x, gateCol.y, gateCol.z, gateCol.w * 0.92f);
                for (int i = 0; i < kArcSegs; ++i) {
                    const float t0 = static_cast<float>(i) / static_cast<float>(kArcSegs);
                    const float t1 = static_cast<float>(i + 1) / static_cast<float>(kArcSegs);
                    const double a0 = -gAng + 2.0 * gAng * static_cast<double>(t0);
                    const double a1 = -gAng + 2.0 * gAng * static_cast<double>(t1);
                    const vsg::dvec3 d0 = horiz * std::cos(a0) + right * std::sin(a0);
                    const vsg::dvec3 d1 = horiz * std::cos(a1) + right * std::sin(a1);
                    // Under-deck vertical shadow drop beneath the range gate
                    pushQuad(deckOrigin + d0 * distM - up * dropH, t0, d0, -1.0f,
                             deckOrigin + d1 * distM - up * dropH, t1, d1, -1.0f,
                             deckOrigin + d1 * distM,              t1, d1,  0.0f,
                             deckOrigin + d0 * distM,              t0, d0,  0.0f,
                             curtainCol, curtainParams);
                    // Upper arched range-gate curtain above the deck
                    pushQuad(deckOrigin + d0 * distM,              t0, d0,  0.0f,
                             deckOrigin + d1 * distM,              t1, d1,  0.0f,
                             deckOrigin + d1 * distM + up * gateH, t1, d1,  1.0f,
                             deckOrigin + d0 * distM + up * gateH, t0, d0,  1.0f,
                             curtainCol, curtainParams);
                }
                for (int i = 0; i < kArcSegs; ++i) {
                    const double a0 = -gAng + 2.0 * gAng * static_cast<double>(i) / static_cast<double>(kArcSegs);
                    const double a1 = -gAng + 2.0 * gAng * static_cast<double>(i + 1) / static_cast<double>(kArcSegs);
                    const vsg::dvec3 d0 = horiz * std::cos(a0) + right * std::sin(a0);
                    const vsg::dvec3 d1 = horiz * std::cos(a1) + right * std::sin(a1);
                    pushBeamRibbon(deckOrigin + d0 * distM + up * gateH,
                                   deckOrigin + d1 * distM + up * gateH, widthM, gateCol, 6.0f, 0.0f, 0.0f);
                }
                for (const double s : {-1.0, 1.0}) {
                    const vsg::dvec3 dEdge = horiz * std::cos(s * gAng) + right * std::sin(s * gAng);
                    pushBeamRibbon(deckOrigin + dEdge * distM - up * (dropH * 0.45),
                                   deckOrigin + dEdge * distM + up * gateH, widthM, gateCol, 6.0f, 0.0f, 0.0f);
                }
            };

            const double rMin = std::max(200.0, static_cast<double>(w.rMinM));
            const double rNe  = std::max(rMin + 200.0, static_cast<double>(w.rNeM));
            const double rMax = std::min(95000.0, std::max(rNe + 200.0, static_cast<double>(w.rMaxM)));
            const bool inNez = (w.flags & ipc::kTactWeaponInNez) != 0u;

            emitFanBand(rMin * 0.65, rMin, vsg::vec4(0.95f, 0.24f, 0.26f, 0.90f), 2.0f);
            emitFanBand(rMin, rNe, inNez ? vsg::vec4(0.14f, 0.92f, 0.50f, 1.0f) : vsg::vec4(0.18f, 0.84f, 0.50f, 0.88f), 1.0f);
            emitFanBand(rNe, rMax, vsg::vec4(0.94f, 0.74f, 0.24f, 0.80f), 0.0f);

            emitUprightRangeGate(rMin, 0.78, vsg::vec4(0.96f, 0.28f, 0.30f, 0.88f), 8.0);
            emitUprightRangeGate(rNe,  0.94, vsg::vec4(0.16f, 0.94f, 0.52f, 0.95f), 12.0);
            emitUprightRangeGate(rMax, 1.00, vsg::vec4(0.96f, 0.76f, 0.22f, 0.86f), 11.0);
        }

        if ((w.flags & ipc::kTactWeaponHasLcos) != 0u) {
            const vsg::dvec3 pip(w.lcosPipperEcef[0], w.lcosPipperEcef[1], w.lcosPipperEcef[2]);
            const vsg::vec4 lcosCol(0.32f, 0.96f, 0.56f, 0.86f);
            pushBeamRibbon(origin, pip, 4.0, lcosCol, 6.0f, 16.0f, 0.0f);
            vsg::dvec3 u, v;
            const vsg::dvec3 d = vsg::normalize(pip - origin);
            orthoBasis(d, u, v);
            // Compact gun aiming pipper ring (14 m radius — never obscures the enemy target in Chase cam!)
            pushRingRibbon(pip, u, v, 14.0, 14.0, 3.0, 24, lcosCol, 0.0f, 0.0f);
        }

        if ((w.flags & ipc::kTactWeaponHasCcip) != 0u) {
            const vsg::dvec3 imp(w.ccipImpactEcef[0], w.ccipImpactEcef[1], w.ccipImpactEcef[2]);
            const vsg::vec4 ccipCol(1.00f, 0.78f, 0.20f, 0.92f);
            vsg::dvec3 ie, in, iu;
            localEnu(imp, ie, in, iu);
            pushBeamRibbon(origin, imp, 8.0, ccipCol, 6.0f, 18.0f, 0.0f);
            pushRingRibbon(imp + iu * 4.0, ie, in, 65.0, 65.0, 7.5, 28, ccipCol, 0.0f, 0.0f);
        }
    }
}

void TacticalVolumes::emitLinks(const ipc::TacticalOverlayFrameV1& frame, const std::vector<Side>& sides, int selectedSlot) {
    if (selectedSlot < 0) return;
    const Side selSide = (static_cast<std::size_t>(selectedSlot) < sides.size()) ? sides[static_cast<std::size_t>(selectedSlot)] : Side::Neutral;
    for (const auto& l : frame.links) {
        const auto kind = static_cast<ipc::TacticalLinkKindV1>(l.kind);
        const Side fromSide = (l.fromSlot < sides.size()) ? sides[l.fromSlot] : Side::Neutral;
        const bool sameSideIncoming =
            ((kind == ipc::TacticalLinkKindV1::Link16Dms || kind == ipc::TacticalLinkKindV1::C2Consent ||
              kind == ipc::TacticalLinkKindV1::Mode5IffReply) &&
             static_cast<int>(l.toSlot) == selectedSlot &&
             (selSide == Side::Neutral || fromSide == selSide));
        const bool involvesSelected = (static_cast<int>(l.fromSlot) == selectedSlot) || sameSideIncoming;
        if (!involvesSelected) continue;

        const vsg::dvec3 from(l.fromEcef[0], l.fromEcef[1], l.fromEcef[2]);
        const vsg::dvec3 to(l.toEcef[0], l.toEcef[1], l.toEcef[2]);
        const float phase = std::clamp(l.ageS / std::max(0.1f, l.durationS), 0.0f, 1.0f);

        if (kind == ipc::TacticalLinkKindV1::Link16Dms || kind == ipc::TacticalLinkKindV1::C2Consent ||
            kind == ipc::TacticalLinkKindV1::Mode5IffReply) {
            const vsg::vec4 col = (kind == ipc::TacticalLinkKindV1::C2Consent)     ? vsg::vec4(1.00f, 0.80f, 0.22f, 0.92f)
                                : (kind == ipc::TacticalLinkKindV1::Mode5IffReply) ? vsg::vec4(0.36f, 0.88f, 0.98f, 0.86f)
                                                                                   : vsg::vec4(0.20f, 0.90f, 0.64f, 0.90f);
            const double span = vsg::length(to - from);
            if (span < 10.0) continue;
            const vsg::dvec3 upMid = localUp((from + to) * 0.5);
            const double arch = std::min(span * 0.07, 1600.0);
            constexpr int kArcSegs = 14;
            const double distEye = std::max(vsg::length(eyeEcef_ - (from + to) * 0.5), 10.0);
            const double halfW = std::max(14.0, distEye * 0.0015);
            const vsg::vec4 params(4.0f, phase, 0.0f, static_cast<float>(span));
            for (int i = 0; i < kArcSegs; ++i) {
                const float t0 = static_cast<float>(i) / static_cast<float>(kArcSegs);
                const float t1 = static_cast<float>(i + 1) / static_cast<float>(kArcSegs);
                const vsg::dvec3 p0 = from + (to - from) * t0 + upMid * (4.0 * t0 * (1.0 - t0) * arch);
                const vsg::dvec3 p1 = from + (to - from) * t1 + upMid * (4.0 * t1 * (1.0 - t1) * arch);
                const vsg::dvec3 dir = vsg::normalize(p1 - p0);
                const vsg::dvec3 toEye = eyeEcef_ - (p0 + p1) * 0.5;
                vsg::dvec3 side = vsg::cross(toEye, dir);
                if (vsg::length(side) < 1e-6) side = upMid;
                else side = vsg::normalize(side);
                const vsg::dvec3 nrm = vsg::normalize(toEye);
                pushQuad(p0 - side * halfW, t0, nrm, -1.0f,
                         p0 + side * halfW, t0, nrm,  1.0f,
                         p1 + side * halfW, t1, nrm,  1.0f,
                         p1 - side * halfW, t1, nrm, -1.0f,
                         col, params);
            }
        } else if (kind == ipc::TacticalLinkKindV1::Mode5IffInterrogate) {
            vsg::dvec3 e, n, u;
            localEnu(from, e, n, u);
            const double rad = std::max(500.0, static_cast<double>(l.param) * phase);
            const float a = std::clamp(1.0f - phase, 0.18f, 0.85f);
            const vsg::vec4 col = (l.flags & ipc::kTactLinkJammed) ? vsg::vec4(1.00f, 0.42f, 0.20f, a)
                                                                   : vsg::vec4(0.32f, 0.86f, 0.98f, a);
            pushRingRibbon(from, e, n, rad, rad, 28.0, 40, col, 0.0f, 0.0f);
        } else if (kind == ipc::TacticalLinkKindV1::PntGnssDrift) {
            vsg::dvec3 e, n, u;
            localEnu(from, e, n, u);
            const bool jammed = (l.flags & ipc::kTactLinkJammed) != 0u;
            const bool locked = (l.flags & ipc::kTactLinkGnssLocked) != 0u;
            const double rad = std::max(static_cast<double>(l.param), 45.0);
            const vsg::vec4 col = jammed ? vsg::vec4(0.98f, 0.26f, 0.22f, 0.92f)
                                : locked ? vsg::vec4(0.22f, 0.88f, 0.48f, 0.78f)
                                         : vsg::vec4(0.42f, 0.78f, 0.98f, 0.78f);
            pushRingRibbon(from, e, n, rad, rad, jammed ? 16.0 : 9.0, 32, col, jammed ? 0.0f : 16.0f, 0.0f);
        }
    }
}

void TacticalVolumes::emitSwaths(const ipc::TacticalOverlayFrameV1& frame, int selectedSlot) {
    if (selectedSlot < 0) return;
    for (const auto& s : frame.swaths) {
        if (static_cast<int>(s.slot) != selectedSlot) continue;
        const vsg::dvec3 center(s.centerEcef[0], s.centerEcef[1], s.centerEcef[2]);
        const vsg::dvec3 axE(s.axisEastEcef[0], s.axisEastEcef[1], s.axisEastEcef[2]);
        const vsg::dvec3 axN(s.axisNorthEcef[0], s.axisNorthEcef[1], s.axisNorthEcef[2]);
        const vsg::dvec3 up = localUp(center);
        const double hw = static_cast<double>(s.halfWidthM);
        const double hl = static_cast<double>(s.halfLengthM);
        const bool isWx = (s.kind == static_cast<std::uint8_t>(ipc::TacticalSwathKindV1::WeatherCell));
        const vsg::vec3 rgb = isWx ? weatherDbzLinear(s.metricDb) : vsg::vec3(0.20f, 0.84f, 0.94f);
        const vsg::vec4 col(rgb.x, rgb.y, rgb.z, 0.90f);
        const vsg::vec4 params(5.0f, isWx ? 1.0f : 0.0f, 0.0f, static_cast<float>(std::max(hw, hl)));
        const vsg::dvec3 cLift = center + up * 15.0;
        const vsg::dvec3 c00 = cLift - axE * hw - axN * hl;
        const vsg::dvec3 c10 = cLift + axE * hw - axN * hl;
        const vsg::dvec3 c11 = cLift + axE * hw + axN * hl;
        const vsg::dvec3 c01 = cLift - axE * hw + axN * hl;
        // Subtle 3-D shadow skirt around SAR / weather footprint
        const double skirtH = isWx ? 360.0 : 55.0;
        const vsg::vec4 skirtParams(7.0f, 1.0f, 1.0f, static_cast<float>(std::max(hw, hl)));
        const vsg::vec4 skirtCol(rgb.x, rgb.y, rgb.z, 0.72f);
        const vsg::dvec3 corners[4] = {c00, c10, c11, c01};
        const vsg::dvec3 normals[4] = {-axN, axE, axN, -axE};
        for (int e = 0; e < 4; ++e) {
            const vsg::dvec3& p0 = corners[e];
            const vsg::dvec3& p1 = corners[(e + 1) & 3];
            pushQuad(p0, 0.0f, normals[e], -1.0f,
                     p1, 1.0f, normals[e], -1.0f,
                     p1 + up * skirtH, 1.0f, normals[e], 1.0f,
                     p0 + up * skirtH, 0.0f, normals[e], 1.0f,
                     skirtCol, skirtParams);
        }
        pushQuad(c00, 0.0f, up, -1.0f,
                 c10, 0.0f, up,  1.0f,
                 c11, 1.0f, up,  1.0f,
                 c01, 1.0f, up, -1.0f,
                 col, params);
    }
}

void TacticalVolumes::update(std::uint32_t overlayMask,
                             const ipc::TacticalOverlayFrameV1* frame,
                             Span<const sim::VehicleState> states,
                             const std::vector<unsigned char>& alive,
                             const std::vector<Side>& sides,
                             int selectedSlot,
                             const vsg::dmat4& projection,
                             const vsg::dvec3& eyeEcef,
                             std::uint32_t screenWidth,
                             std::uint32_t screenHeight,
                             float daylight,
                             double simTime,
                             double wallTime,
                             const vsg::dvec3& sunDirEcef) {
    lastVertexCount_ = 0u;
    if (!built_ || !frame || overlayMask == 0u || selectedSlot < 0) {
        if (switch_) switch_->setAllChildren(false);
        if (draw_) draw_->vertexCount = 0u;
        return;
    }

    eyeEcef_ = eyeEcef;
    cursor_ = 0u;

    if (overlayMask & kMaskSwaths)  emitSwaths(*frame, selectedSlot);
    if (overlayMask & kMaskRadar)   emitRadars(*frame, states, alive, sides, selectedSlot);
    if (overlayMask & kMaskJammer)  emitJammers(*frame, states, alive, sides, selectedSlot);
    if (overlayMask & kMaskEsm)     emitEsm(*frame, sides, selectedSlot);
    if (overlayMask & kMaskWeapons) emitWeapons(*frame, states, alive, selectedSlot);
    if (overlayMask & kMaskLinks)   emitLinks(*frame, sides, selectedSlot);

    lastVertexCount_ = cursor_;
    if (cursor_ == 0u) {
        if (switch_) switch_->setAllChildren(false);
        if (draw_) draw_->vertexCount = 0u;
        return;
    }

    if (transform_) transform_->matrix = vsg::translate(eyeEcef);
    const float p22 = static_cast<float>(projection[2][2]);
    const float p32 = static_cast<float>(projection[3][2]);
    const float invW = 1.0f / static_cast<float>(std::max<std::uint32_t>(screenWidth, 1u));
    const float invH = 1.0f / static_cast<float>(std::max<std::uint32_t>(screenHeight, 1u));
    vsg::dvec3 eyeEast(1.0, 0.0, 0.0), eyeNorth(0.0, 1.0, 0.0), eyeUp(0.0, 0.0, 1.0);
    if (vsg::length2(eyeEcef) > 1.0) localEnu(eyeEcef, eyeEast, eyeNorth, eyeUp);

    // Sculptural 3/4 directional key light aligned with scene sun azimuth (clamped to ~33 deg elevation
    // so opposite vertical side walls always get unmistakable bright-side vs. dark-side contrast).
    vsg::dvec3 keyLight = vsg::normalize((-eyeEast * 0.60 - eyeNorth * 0.58) + eyeUp * 0.55);
    const double sunLen = vsg::length(sunDirEcef);
    if (sunLen > 0.1) {
        const vsg::dvec3 sUnit = sunDirEcef / sunLen;
        const double sunElev = vsg::dot(sUnit, eyeUp);
        const vsg::dvec3 sHoriz = sUnit - eyeUp * sunElev;
        const double hLen = vsg::length(sHoriz);
        if (sunElev > -0.05 && hLen > 0.12) {
            keyLight = vsg::normalize((sHoriz / hLen) * 0.84 + eyeUp * 0.54);
        }
    }

    frameUbo_->set(0, vsg::vec4(p22, p32, invW, invH));
    frameUbo_->set(1, vsg::vec4(static_cast<float>(std::fmod(std::max(simTime, 0.0), 600.0)),
                                static_cast<float>(std::fmod(std::max(wallTime, 0.0), 600.0)),
                                std::clamp(daylight, 0.1f, 1.0f), 0.0f));
    frameUbo_->set(2, vsg::vec4(static_cast<float>(eyeUp.x), static_cast<float>(eyeUp.y), static_cast<float>(eyeUp.z), 0.0f));
    frameUbo_->set(3, vsg::vec4(static_cast<float>(keyLight.x), static_cast<float>(keyLight.y), static_cast<float>(keyLight.z), 0.0f));
    frameUbo_->dirty();
    vertexStaging_->dirty();

    if (draw_) draw_->vertexCount = cursor_;
    if (switch_) switch_->setAllChildren(true);
}

} // namespace fsim::world
