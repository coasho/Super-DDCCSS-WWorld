#include "world/FlowVolumes.h"

#include "core/Log.h"
#include "world/FlowFires.h"
#include "world/Frames.h"
#include "world/ShadingRate.h"
#include "world/Sky.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

namespace fsim::world {

namespace ff = flowfires;

namespace {

constexpr double kBottomRadius = 6360000.0; // (the clouds' atmosphere, as world::Detonations')
constexpr double kExposure = 10.0;
constexpr std::uint32_t kRows = 8;          // the frame's rows before the black body's table (assets/clouds/shaders/flow.glsl)
constexpr std::uint32_t kPagesAt = kRows + SunlitBlackbody::kRows;
constexpr std::uint32_t kAtlas = FlowVolumes::kBricks * FlowVolumes::kBrick; // texels a side

/// IEEE half from a float (round to nearest; what the fields hold is never near its range's ends)
std::uint16_t toHalf(float f) {
    std::uint32_t x;
    std::memcpy(&x, &f, 4);
    const std::uint32_t sign = (x >> 16) & 0x8000u;
    const int exponent = static_cast<int>((x >> 23) & 0xffu) - 127 + 15;
    std::uint32_t mantissa = x & 0x7fffffu;
    if (exponent <= 0) return static_cast<std::uint16_t>(sign); // (under 6e-5: none)
    if (exponent >= 31) return static_cast<std::uint16_t>(sign | 0x7bffu);
    mantissa += 0x1000u; // (round)
    if (mantissa & 0x800000u) return static_cast<std::uint16_t>(sign | (static_cast<std::uint32_t>(exponent + 1) << 10));
    return static_cast<std::uint16_t>(sign | (static_cast<std::uint32_t>(exponent) << 10) | (mantissa >> 13));
}

/// a leaf's place as a key (21 bits an axis: +-1M leaves, 500 km at 0.5 m)
std::uint64_t leafKey(int x, int y, int z) {
    const auto u = [](int v) { return static_cast<std::uint64_t>(static_cast<std::uint32_t>(v + (1 << 20)) & 0x1fffffu); };
    return u(x) | (u(y) << 21) | (u(z) << 42);
}
/// the shader's hash of a leaf's place (flow.glsl's flowHash)
std::uint32_t leafHash(int x, int y, int z) {
    return (static_cast<std::uint32_t>(x) * 73856093u) ^ (static_cast<std::uint32_t>(y) * 19349663u) ^ (static_cast<std::uint32_t>(z) * 83492791u);
}

} // namespace

FlowVolumes::FlowVolumes(vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders, const AtmosphereTables& tables,
                         bool coarseShading) {
    (void)options;
    root_ = vsg::Group::create();
    switch_ = vsg::Switch::create();
    root_->addChild(switch_);
    const CloudNoise noise = shaders.empty() ? CloudNoise{} : loadCloudNoise(shaders.parent_path());
    if (shaders.empty() || !tables.valid() || !noise.valid()) {
        LOG_WARN("world") << "no Flow volumes drawn: the clouds' shaders, noises or the atmosphere's tables are missing";
        return;
    }
    FlowSolver::Settings settings;
    settings.cellM = kCellM;
    settings.combustion = ff::combustion();
    if (!solver_.start(settings)) {
        LOG_INFO("world") << "NVIDIA Flow not running (not built with it, or nvflow.dll, nvflowext.dll not beside the viewer)";
        return;
    }
    const std::string cap = "#define BLACKBODY_AT " + std::to_string(kRows) + "\n#define BLACKBODY_ROWS " + std::to_string(SunlitBlackbody::kRows) +
                            "\n#define PAGES_AT " + std::to_string(kPagesAt) + "\n#define FLOW_BRICKS " + std::to_string(kBricks) +
                            "\n#define FLOW_BRICK " + std::to_string(kBrick) + "\n#define FLOW_HASH " + std::to_string(kHash) + "\n";
    auto vertex = loadCloudShader(shaders, "flow.vert", VK_SHADER_STAGE_VERTEX_BIT, cap);
    auto fragment = loadCloudShader(shaders, "flow.frag", VK_SHADER_STAGE_FRAGMENT_BIT, cap);
    vsg::ShaderStages stages{vertex, fragment};
    auto compiler = vsg::ShaderCompiler::create();
    if (!vertex || !fragment || !compiler->supported() || !compiler->compile(stages)) {
        LOG_WARN("world") << "the Flow volume's shaders did not build; none drawn";
        return;
    }
    const VkShaderStageFlags both = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    vsg::DescriptorSetLayoutBindings bindings{{0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, both, nullptr},
                                              {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, both, nullptr},
                                              {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
                                              {3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
                                              {5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}};
    for (std::uint32_t b : {6u, 7u, 8u, 20u}) bindings.push_back({b, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, both, nullptr});
    auto setLayout = vsg::DescriptorSetLayout::create(bindings);
    auto layout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout}, vsg::PushConstantRanges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}});
    rows_ = vsg::vec4Array::create(kPagesAt + 2 * kPages, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    blackbody_.write(*rows_, kRows);
    frame_ = vsg::vec4Array::create(4, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    hash_ = vsg::uivec4Array::create(kHash, vsg::uivec4(0, 0, 0, 0));
    atlas_ = vsg::usvec2Array3D::create(kAtlas, kAtlas, kAtlas, vsg::usvec2(0, 0), vsg::Data::Properties{VK_FORMAT_R16G16_SFLOAT});
    for (vsg::Data* d : {static_cast<vsg::Data*>(rows_.get()), static_cast<vsg::Data*>(frame_.get()), static_cast<vsg::Data*>(hash_.get()),
                         static_cast<vsg::Data*>(atlas_.get())})
        d->properties.dataVariance = vsg::DYNAMIC_DATA;
    auto sampler = vsg::Sampler::create();
    sampler->minFilter = sampler->magFilter = VK_FILTER_LINEAR;
    sampler->mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    sampler->addressModeU = sampler->addressModeV = sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler->maxLod = 0.0f;
    vsg::Descriptors descriptors{vsg::DescriptorBuffer::create(rows_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                                 vsg::DescriptorBuffer::create(frame_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
                                 vsg::DescriptorImage::create(sampler, atlas_, 2, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                                 vsg::DescriptorBuffer::create(hash_, 5, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                                 vsg::DescriptorImage::create(noise.shape, 3, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER)};
    for (const auto& d : tables.descriptors()) descriptors.push_back(d);
    auto set = vsg::DescriptorSet::create(setLayout, descriptors);

    // premultiplied over, depth tested and not written; each page's box by its far faces (as a detonation's), the
    // pages far to near
    auto raster = vsg::RasterizationState::create();
    raster->cullMode = VK_CULL_MODE_FRONT_BIT;
    auto assembly = vsg::InputAssemblyState::create();
    assembly->topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
    auto depth = vsg::DepthStencilState::create();
    depth->depthWriteEnable = VK_FALSE;
    auto blend = vsg::ColorBlendState::create();
    blend->attachments = {VkPipelineColorBlendAttachmentState{
        VK_TRUE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD, VK_BLEND_FACTOR_ONE,
        VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};
    vsg::GraphicsPipelineStates states{vsg::VertexInputState::create(), assembly, raster, vsg::MultisampleState::create(), blend, depth};
    if (coarseShading) {
        states.push_back(vsg::DynamicState::create(VK_DYNAMIC_STATE_FRAGMENT_SHADING_RATE_KHR));
        shadingRate_ = ShadingRate::create();
    }
    auto group = vsg::StateGroup::create();
    group->add(vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(layout, stages, states)));
    group->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, set));
    draw_ = vsg::Draw::create(14, 0, 0, 0);
    if (shadingRate_) group->addChild(shadingRate_);
    group->addChild(draw_);
    at_ = vsg::MatrixTransform::create();
    at_->addChild(group);
    switch_->addChild(true, at_);
    LOG_INFO("world") << "NVIDIA Flow: fire and smoke solved at " << kCellM << " m, up to " << kBricks * kBricks * kBricks << " leaves drawn";
}

void FlowVolumes::setAircraft(std::size_t slot, double wingspanM, double massKg) {
    if (slot >= aircraft_.size()) aircraft_.resize(slot + 1, vsg::dvec2(0.0, 0.0));
    aircraft_[slot] = vsg::dvec2(std::max(wingspanM, 0.0), std::max(massKg, 0.0));
}

double FlowVolumes::nearFieldThinning() { return -std::log(1.0 - ff::kSmokeDilution); }

void FlowVolumes::setScreen(std::uint32_t width, std::uint32_t height) {
    screenHeight_ = std::max<std::uint32_t>(height, 1);
    coarse_ = shadingRate_ && static_cast<double>(width) * static_cast<double>(height) >= 2.0e6 ? 2u : 1u;
    if (shadingRate_) static_cast<ShadingRate*>(shadingRate_.get())->size = coarse_;
}

void FlowVolumes::setVisible(bool on) {
    if (on == visible_) return;
    visible_ = on;
    switch_->setAllChildren(on);
    if (!on) clear();
}

void FlowVolumes::clear() {
    bursts_.clear();
    leaks_.clear();
    haveOrigin_ = false;
    lastTime_ = -1.0;
    cleared_ = true;
    if (draw_) draw_->instanceCount = 0;
}

vsg::dvec3 FlowVolumes::windLocal() const {
    // blowing towards from + 180 degrees
    const double from = vsg::radians(air_.windFromDeg);
    return vsg::dvec3(-std::sin(from), -std::cos(from), 0.0) * air_.windMs;
}

vsg::dvec3 FlowVolumes::toAir(const vsg::dvec3& ecef, double t) const {
    const vsg::dvec3 d = ecef - origin_;
    return vsg::dvec3(vsg::dot(d, east_), vsg::dot(d, north_), vsg::dot(d, up_)) - windLocal() * (t - originTime_);
}

vsg::dvec3 FlowVolumes::velocityToAir(const vsg::dvec3& v) const {
    return vsg::dvec3(vsg::dot(v, east_), vsg::dot(v, north_), vsg::dot(v, up_)) - windLocal();
}

void FlowVolumes::origin(const vsg::dvec3& at, double t, const vsg::EllipsoidModel& ellipsoid) {
    if (haveOrigin_) return;
    const vsg::dvec3 lla = ellipsoid.convertECEFToLatLongAltitude(at);
    const double lat = vsg::radians(lla.x), lon = vsg::radians(lla.y); // (vsg's are degrees)
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    origin_ = at;
    originTime_ = t;
    east_ = vsg::dvec3(-so, co, 0.0);
    north_ = vsg::dvec3(-sl * co, -sl * so, cl);
    up_ = vsg::dvec3(cl * co, cl * so, sl);
    haveOrigin_ = true;
}

void FlowVolumes::add(const ipc::EngagementEvent& event, const vsg::EllipsoidModel& ellipsoid) {
    if (!available()) return;
    const bool fire = event.kind == static_cast<std::uint8_t>(ipc::EngagementKind::Fire);
    if (event.kind != static_cast<std::uint8_t>(ipc::EngagementKind::Detonation) && !fire) return;
    const vsg::dvec3 at(event.positionEcef[0], event.positionEcef[1], event.positionEcef[2]);
    const vsg::dvec3 velocity(event.velocityEcef[0], event.velocityEcef[1], event.velocityEcef[2]);
    origin(at, event.simTime, ellipsoid);
    const double altitude = ellipsoid.convertECEFToLatLongAltitude(at).z;
    const double pressure = Detonations::pressureAt(air_, altitude);
    const vsg::dvec3 p = toAir(at, event.simTime), v = velocityToAir(velocity);
    if (fire) {
        // what leaks and burns on, from its component (written again as it changes; 0: it is out)
        const std::string component(event.name, ::strnlen(event.name, sizeof event.name));
        Leak* leak = nullptr;
        for (Leak& l : leaks_)
            if (l.slot == event.slot && l.component == component) leak = &l;
        if (!leak && event.rateKgS > 0.0) {
            leaks_.push_back(Leak{event.slot, component, 0.0, event.simTime, nextLuid_});
            nextLuid_ += 2;
            leak = &leaks_.back();
        }
        if (leak) {
            leak->rateKgS = std::max(event.rateKgS, 0.0);
            LOG_INFO("world") << "Flow: " << component << " on slot " << event.slot << " leaks " << leak->rateKgS << " kg/s from t=" << event.simTime;
        }
    }
    if (!(event.amountKg > 0.0)) return;
    Burst b;
    b.detonation = !fire;
    b.time = event.simTime;
    b.amountKg = event.amountKg;
    b.pressurePa = pressure;
    b.at[0] = p.x, b.at[1] = p.y, b.at[2] = p.z;
    vsg::dvec3 motion = v; // a spill: the hole's path
    if (b.detonation) {
        // its products leave with the warhead's momentum, shared with the air the fireball takes in
        const double R = 0.5 * ff::detonationDiameter(event.amountKg, pressure);
        const double T = std::max(216.65, air_.seaLevelK - 0.0065 * altitude);
        const double air = pressure / (287.05 * T) * 4.0 / 3.0 * 3.141592653589793 * R * R * R;
        motion = v * (event.amountKg / (event.amountKg + air));
    }
    b.motion[0] = motion.x, b.motion[1] = motion.y, b.motion[2] = motion.z;
    b.luid = nextLuid_;
    nextLuid_ += 2;
    bursts_.push_back(b);
    LOG_INFO("world") << "Flow: " << (fire ? "a fire's spill" : "a detonation") << " at t=" << event.simTime << " s, " << event.amountKg
                      << " kg" << (fire ? "" : " TNT") << " at " << altitude << " m";
}

void FlowVolumes::update(double now, Span<const sim::VehicleState> states, const std::vector<unsigned char>& alive, const VehicleVisuals& visuals,
                         const vsg::dmat4& projection, const vsg::dvec3& eye, const vsg::EllipsoidModel& ellipsoid, const vsg::dvec3& sunEcef) {
    if (!available() || !visible_) return;
    const auto t0 = std::chrono::steady_clock::now();
    // (a replay's seek back: start over)
    if (lastTime_ >= 0.0 && now < lastTime_ - 0.25) clear();
    // the time to step: since the last frame (a second at most: a longer gap - a seek forward - is not caught up)
    const double gap = lastTime_ < 0.0 ? 1.0 / 60.0 : std::clamp(now - lastTime_, 0.0, 1.0);
    const double start = now - gap;
    lastTime_ = now;

    if (gap > 0.0 && (haveOrigin_ || cleared_)) {
        // the leaks' holes now, and how they move (through the air)
        struct Hole {
            const Leak* leak;
            vsg::dvec3 at, velocity;
            double pressure;
        };
        std::vector<Hole> holes;
        // and the burning aircraft's wakes: each its middle, velocity, axes (the solver's frame), its air
        struct Wake {
            std::size_t slot;
            vsg::dvec3 at, velocity, right, up;
            double pressure, airK;
        };
        std::vector<Wake> wakes;
        for (const Leak& l : leaks_) {
            if (!(l.rateKgS > 0.0) || now < l.since || l.slot >= states.size() || l.slot >= alive.size() || !alive[l.slot]) continue;
            const VehicleVisuals::Shape& shape = visuals.shape(l.slot);
            if (!shape.valid) continue;
            for (const auto& c : shape.components) {
                if (c.name != l.component) continue;
                const sim::VehicleState& st = states[l.slot];
                const vsg::dvec3 v = bodyAxisEcef(st, 0) * st.velocityBodyMs[0] + bodyAxisEcef(st, 1) * st.velocityBodyMs[1] +
                                     bodyAxisEcef(st, 2) * st.velocityBodyMs[2];
                holes.push_back(Hole{&l, toAir(bodyToEcef(st) * c.at, now), velocityToAir(v), Detonations::pressureAt(air_, st.altitudeMslM)});
                bool laid = false;
                for (const Wake& w : wakes) laid = laid || w.slot == l.slot;
                if (!laid && l.slot < aircraft_.size() && aircraft_[l.slot].x > 0.0) {
                    const auto axis = [&](const vsg::dvec3& e) { return vsg::dvec3(vsg::dot(e, east_), vsg::dot(e, north_), vsg::dot(e, up_)); };
                    wakes.push_back(Wake{l.slot, toAir(positionEcef(st), now), velocityToAir(v), axis(bodyAxisEcef(st, 1)), -axis(bodyAxisEcef(st, 2)),
                                         Detonations::pressureAt(air_, st.altitudeMslM),
                                         std::max(216.65, air_.seaLevelK - 0.0065 * st.altitudeMslM)});
                }
                break;
            }
        }
        // stepped in calls of two of the solver's steps at most (Flow's maxStepsPerSimulate), each with its sources
        // where they were then: a slow frame (a screenshot saved, a hitch) caught up, not left behind
        const double call = 2.0 / solver_.settings().stepsPerSecond;
        const int calls = std::max(1, static_cast<int>(std::ceil(gap / call - 1e-9)));
        for (int k = 0; k < calls; ++k) {
            const double t = start + gap * (k + 1) / calls, dt = gap / calls;
            sources_.clear();
            bursts_.erase(std::remove_if(bursts_.begin(), bursts_.end(),
                                         [&](const Burst& b) {
                                             const double age = t - b.time;
                                             return b.detonation ? !ff::detonation(b.amountKg, b.at, b.motion, b.pressurePa, age, dt, b.luid, sources_)
                                                                 : !ff::spill(b.amountKg, b.at, b.motion, b.pressurePa, age, dt, b.luid, sources_);
                                         }),
                          bursts_.end());
            for (const Wake& w : wakes) {
                const vsg::dvec3 a = w.at - w.velocity * (now - t);
                const double at[3] = {a.x, a.y, a.z}, velocity[3] = {w.velocity.x, w.velocity.y, w.velocity.z};
                const double right[3] = {w.right.x, w.right.y, w.right.z}, up[3] = {w.up.x, w.up.y, w.up.z};
                ff::wake(aircraft_[w.slot].y, aircraft_[w.slot].x, at, velocity, right, up, w.pressure, w.airK, dt,
                         (1u << 20) + w.slot, sources_);
            }
            for (const Hole& h : holes) {
                const vsg::dvec3 a = h.at - h.velocity * (now - t);
                const double at[3] = {a.x, a.y, a.z}, velocity[3] = {h.velocity.x, h.velocity.y, h.velocity.z};
                ff::leak(h.leak->rateKgS, at, velocity, h.pressure, dt, h.leak->luid, sources_);
            }
            solver_.step(dt, sources_, cleared_);
            cleared_ = false;
        }
        cost_.solverGpuMs = solver_.gpuMs();
    }
    const auto p0 = std::chrono::steady_clock::now();
    if (haveOrigin_) pack(toAir(eye, now));
    cost_.packMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - p0).count();

    {
        // the frame, as a detonation's: the eye, a pixel's angle, the altitude correction, the sun, the night light
        const vsg::dvec3 lla = ellipsoid.convertECEFToLatLongAltitude(eye);
        const vsg::dvec3 surface = ellipsoid.convertLatLongAltitudeToECEF(vsg::dvec3(lla.x, lla.y, 0.0));
        const double a2 = ellipsoid.radiusEquator() * ellipsoid.radiusEquator(), b2 = ellipsoid.radiusPolar() * ellipsoid.radiusPolar();
        const vsg::dvec3 normal = vsg::normalize(vsg::dvec3(surface.x / a2, surface.y / a2, surface.z / b2));
        const vsg::dvec3 correction = -(surface - normal * kBottomRadius);
        const vsg::dvec3 sun = vsg::normalize(sunEcef);
        const double pixel = 1.0 / std::max(1.0, 0.5 * std::abs(projection[1][1]) * static_cast<double>(screenHeight_));
        const Daylight d = daylightAt(sunEcef, normal);
        const double sunUp = vsg::dot(sun, vsg::normalize(eye));
        const double night = (1.0 - std::clamp((sunUp + 0.10) / 0.25, 0.0, 1.0)) * d.ambient;
        const auto F = [](double v) { return static_cast<float>(v); };
        frame_->set(0, vsg::vec4(F(eye.x), F(eye.y), F(eye.z), F(pixel * coarse_)));
        frame_->set(1, vsg::vec4(F(correction.x), F(correction.y), F(correction.z), F(kBottomRadius)));
        frame_->set(2, vsg::vec4(F(sun.x), F(sun.y), F(sun.z), F(std::fmod(std::max(now, 0.0), 3600.0))));
        frame_->set(3, vsg::vec4(F(d.ambientColour.x * night), F(d.ambientColour.y * night), F(d.ambientColour.z * night), F(kExposure)));
        frame_->dirty();
    }
    if (haveOrigin_) {
        // the air's frame where it has drifted to, from the eye; its axes; the index's zero; the soot's constants
        const auto F = [](double v) { return static_cast<float>(v); };
        const vsg::dvec3 w = windLocal() * (now - originTime_);
        const vsg::dvec3 from = origin_ + east_ * w.x + north_ * w.y + up_ * w.z - eye;
        const FlowSolver::Fields& f = solver_.fields();
        rows_->set(0, vsg::vec4(F(from.x), F(from.y), F(from.z), F(f.voxel)));
        rows_->set(1, vsg::vec4(F(east_.x), F(east_.y), F(east_.z), 0.0f));
        rows_->set(2, vsg::vec4(F(north_.x), F(north_.y), F(north_.z), 0.0f));
        rows_->set(3, vsg::vec4(F(up_.x), F(up_.y), F(up_.z), 0.0f));
        rows_->set(4, vsg::vec4(F(f.zero[0]), F(f.zero[1]), F(f.zero[2]), 0.0f));
        rows_->set(5, vsg::vec4(1.0f, F(kSootAlbedo), F(ff::kFlameK - ff::kAmbientK), F(ff::kAmbientK)));
        // the flame's flicker below the cells (kFlicker of its temperature rms, its mean that of CCPS's emissive
        // power), its eddies kFlickerM across, the time they evolve by
        rows_->set(6, vsg::vec4(F(kFlicker), F(SunlitBlackbody::meanOfFlicker(ff::kFlameK, kFlicker) / ff::kFlameK), F(kFlickerM),
                                F(std::fmod(std::max(now, 0.0), 600.0))));
        rows_->dirty();
    }
    at_->matrix = vsg::translate(eye);
    cost_.cpuMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void FlowVolumes::pack(const vsg::dvec3& eyeLocal) {
    const FlowSolver::Fields& f = solver_.fields();
    cost_.leavesRead = f.leaves.size();
    cost_.readbackBytes = f.bytes;
    if (f.leaves.empty() || !(f.voxel > 0.0)) {
        draw_->instanceCount = 0;
        cost_.leaves = cost_.pages = 0;
        return;
    }
    // the leaves drawn: all, or the nearest that fit
    const std::size_t most = static_cast<std::size_t>(kBricks) * kBricks * kBricks;
    std::vector<const FlowSolver::Leaf*> drawn;
    drawn.reserve(f.leaves.size());
    for (const auto& leaf : f.leaves) drawn.push_back(&leaf);
    const auto middle = [&](const FlowSolver::Leaf* l) {
        return vsg::dvec3(f.zero[0] + (l->origin[0] + 4) * f.voxel, f.zero[1] + (l->origin[1] + 4) * f.voxel, f.zero[2] + (l->origin[2] + 4) * f.voxel);
    };
    if (drawn.size() > most) {
        std::nth_element(drawn.begin(), drawn.begin() + static_cast<std::ptrdiff_t>(most), drawn.end(),
                         [&](const FlowSolver::Leaf* a, const FlowSolver::Leaf* b) { return vsg::length2(middle(a) - eyeLocal) < vsg::length2(middle(b) - eyeLocal); });
        drawn.resize(most);
    }
    cost_.leaves = drawn.size();
    // their bricks, and the hash the shader finds them by
    slots_.clear();
    std::fill(hash_->begin(), hash_->end(), vsg::uivec4(0, 0, 0, 0));
    for (std::uint32_t s = 0; s < drawn.size(); ++s) {
        const int* o = drawn[s]->origin;
        const int L[3] = {o[0] >> 3, o[1] >> 3, o[2] >> 3};
        slots_[leafKey(L[0], L[1], L[2])] = s;
        std::uint32_t h = leafHash(L[0], L[1], L[2]) & (kHash - 1);
        while ((*hash_)[h].w != 0) h = (h + 1) & (kHash - 1);
        (*hash_)[h] = vsg::uivec4(static_cast<std::uint32_t>(L[0]), static_cast<std::uint32_t>(L[1]), static_cast<std::uint32_t>(L[2]), s + 1);
    }
    hash_->dirty();
    vsg::usvec2* atlas = atlas_->data();
    const auto texel = [&](std::uint32_t x, std::uint32_t y, std::uint32_t z) { return (static_cast<std::size_t>(z) * kAtlas + y) * kAtlas + x; };
    // (the bricks shared among threads: each its own, the map only read)
    const auto fill = [&](std::uint32_t first, std::uint32_t last) {
    for (std::uint32_t s = first; s < last; ++s) {
        const FlowSolver::Leaf& leaf = *drawn[s];
        const int L[3] = {leaf.origin[0] >> 3, leaf.origin[1] >> 3, leaf.origin[2] >> 3};
        // the leaf and its neighbours after it (the brick's last texels: the next leaf's first voxels)
        const FlowSolver::Leaf* next[8] = {&leaf, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
        for (int k = 1; k < 8; ++k) {
            const auto it = slots_.find(leafKey(L[0] + (k & 1), L[1] + ((k >> 1) & 1), L[2] + ((k >> 2) & 1)));
            next[k] = it == slots_.end() ? nullptr : drawn[it->second];
        }
        const std::uint32_t bx = (s % kBricks) * kBrick, by = ((s / kBricks) % kBricks) * kBrick, bz = (s / (kBricks * kBricks)) * kBrick;
        for (std::uint32_t k = 0; k < kBrick; ++k)
            for (std::uint32_t j = 0; j < kBrick; ++j)
                for (std::uint32_t i = 0; i < kBrick; ++i) {
                    const int n = (i >= 8 ? 1 : 0) | (j >= 8 ? 2 : 0) | (k >= 8 ? 4 : 0);
                    const FlowSolver::Leaf* from = next[n];
                    vsg::usvec2 v(0, 0);
                    if (from) {
                        const std::uint32_t at = ((i & 7u) << 6) | ((j & 7u) << 3) | (k & 7u); // (x slowest, z fastest)
                        v = vsg::usvec2(toHalf(from->smoke[at]), toHalf(from->temperature[at]));
                    }
                    atlas[texel(bx + i, by + j, bz + k)] = v;
                }
    }
    };
    const auto count = static_cast<std::uint32_t>(drawn.size());
    const std::uint32_t threads = std::clamp<std::uint32_t>(std::thread::hardware_concurrency() / 2, 1, 8);
    if (count < 256 || threads == 1) fill(0, count);
    else {
        std::vector<std::thread> pool;
        for (std::uint32_t t = 1; t < threads; ++t) pool.emplace_back(fill, count * t / threads, count * (t + 1) / threads);
        fill(0, count / threads);
        for (auto& t : pool) t.join();
    }
    atlas_->dirty();
    // the pages (8^3 leaves): each the box round its leaves, drawn far to near
    struct Page {
        int lo[3], hi[3];
        double distance;
    };
    std::unordered_map<std::uint64_t, Page> pages;
    for (const FlowSolver::Leaf* l : drawn) {
        const int L[3] = {l->origin[0] >> 3, l->origin[1] >> 3, l->origin[2] >> 3};
        auto [it, fresh] = pages.try_emplace(leafKey(L[0] >> 3, L[1] >> 3, L[2] >> 3), Page{{L[0], L[1], L[2]}, {L[0], L[1], L[2]}, 0.0});
        if (!fresh)
            for (int a = 0; a < 3; ++a) it->second.lo[a] = std::min(it->second.lo[a], L[a]), it->second.hi[a] = std::max(it->second.hi[a], L[a]);
    }
    std::vector<Page> order;
    order.reserve(pages.size());
    const double leafM = 8.0 * f.voxel;
    for (auto& [key, page] : pages) {
        (void)key;
        // (by the page's own cell's middle, not its box's: equal cells of a grid, far to near, are drawn in order)
        vsg::dvec3 c;
        for (std::size_t a = 0; a < 3; ++a) c[a] = f.zero[a] + ((page.lo[a] >> 3) * 8 + 4) * leafM;
        page.distance = vsg::length(c - eyeLocal);
        order.push_back(page);
    }
    std::sort(order.begin(), order.end(), [](const Page& a, const Page& b) { return a.distance > b.distance; });
    if (order.size() > kPages) order.erase(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(order.size() - kPages));
    const auto F = [](double v) { return static_cast<float>(v); };
    for (std::uint32_t k = 0; k < order.size(); ++k) {
        const Page& p = order[k];
        vsg::dvec3 lo, size;
        for (std::size_t a = 0; a < 3; ++a) lo[a] = f.zero[a] + p.lo[a] * leafM, size[a] = (p.hi[a] - p.lo[a] + 1) * leafM;
        rows_->set(kPagesAt + 2 * k, vsg::vec4(F(lo.x), F(lo.y), F(lo.z), 1.0f));
        rows_->set(kPagesAt + 2 * k + 1, vsg::vec4(F(size.x), F(size.y), F(size.z), 0.0f));
    }
    cost_.pages = order.size();
    draw_->instanceCount = static_cast<std::uint32_t>(order.size());
}

} // namespace fsim::world
