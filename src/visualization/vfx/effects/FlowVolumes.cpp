#include "visualization/vfx/effects/FlowVolumes.h"

#include "foundation/core/Log.h"
#include "visualization/vfx/flow/FlowFires.h"
#include "visualization/scene/Frames.h"
#include "visualization/scene/ShadingRate.h"
#include "visualization/scene/Sky.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <future>
#include <thread>

namespace fsim::world {

namespace ff = flowfires;

namespace {

constexpr double kBottomRadius = 6360000.0; // (the clouds' atmosphere)
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

double msSince(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

} // namespace

/// The packing's hands: a few threads that stay, each given a share of a range (and the caller one), all waited for.
class FlowVolumes::Pool {
public:
    explicit Pool(unsigned threads) {
        for (unsigned t = 0; t < threads; ++t) threads_.emplace_back([this, t] { run(t); });
    }
    ~Pool() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            quit_ = true;
        }
        start_.notify_all();
        for (auto& t : threads_) t.join();
    }
    /// fn(first, last) over [0, count), in as many shares as there are hands and the caller.
    void each(std::uint32_t count, const std::function<void(std::uint32_t, std::uint32_t)>& fn) {
        const auto shares = static_cast<std::uint32_t>(threads_.size()) + 1u;
        if (count < 256 || shares == 1) {
            fn(0, count);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            fn_ = &fn, count_ = count, left_ = static_cast<unsigned>(threads_.size());
            ++round_;
        }
        start_.notify_all();
        fn(0, count / shares);
        std::unique_lock<std::mutex> lock(mutex_);
        done_.wait(lock, [this] { return left_ == 0; });
    }

private:
    void run(unsigned t) {
        std::uint64_t seen = 0;
        std::unique_lock<std::mutex> lock(mutex_);
        for (;;) {
            start_.wait(lock, [&] { return quit_ || round_ != seen; });
            if (quit_) return;
            seen = round_;
            const auto shares = static_cast<std::uint32_t>(threads_.size()) + 1u;
            const std::uint32_t first = count_ * (t + 1) / shares, last = count_ * (t + 2) / shares;
            const auto* fn = fn_;
            lock.unlock();
            (*fn)(first, last);
            lock.lock();
            if (--left_ == 0) done_.notify_one();
        }
    }
    std::vector<std::thread> threads_;
    std::mutex mutex_;
    std::condition_variable start_, done_;
    const std::function<void(std::uint32_t, std::uint32_t)>* fn_ = nullptr;
    std::uint32_t count_ = 0;
    unsigned left_ = 0;
    std::uint64_t round_ = 0;
    bool quit_ = false;
};

FlowVolumes::~FlowVolumes() {
    if (worker_.joinable()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            quit_ = true;
        }
        wake_.notify_all();
        worker_.join();
    }
    // (the texture's data was a buffer of ours: the scene may outlive this, and must not be left pointing at it)
    if (atlas_) atlas_->assign(0, 0, 0, nullptr, atlas_->properties);
}

void FlowVolumes::setSynchronous(bool on) {
    std::lock_guard<std::mutex> lock(mutex_);
    synchronous_ = on;
}

void FlowVolumes::work(std::shared_ptr<void> started, FlowSolver::Settings settings) {
    // the solver is this thread's from its making to its end: Flow's device and context are used from here alone
    FlowSolver solver;
    const bool ok = solver.start(settings);
    static_cast<std::promise<bool>*>(started.get())->set_value(ok);
    if (!ok) return;
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
        wake_.wait(lock, [this] { return quit_ || !jobs_.empty(); });
        if (quit_) break;
        Job job = std::move(jobs_.front());
        jobs_.pop_front();
        // (a step asked before a clear() is not the grid's any more; of several waiting - a slow frame caught up -
        // only the last one's fields are drawn)
        const bool wanted = job.epoch == epoch_;
        const bool last = jobs_.empty() || jobs_.front().epoch != job.epoch;
        const bool wait = synchronous_;
        busy_ = true;
        lock.unlock();
        if (wanted) {
            const auto t0 = std::chrono::steady_clock::now();
            solver.step(job.dt, job.sources, job.clear, wait);
            if (last) {
                const auto p0 = std::chrono::steady_clock::now();
                pack(solver.fields(), vsg::dvec3(job.eye[0], job.eye[1], job.eye[2]), *back_);
                back_->epoch = job.epoch;
                back_->solverGpuMs = solver.gpuMs();
                back_->packMs = msSince(p0);
                back_->workerMs = msSince(t0);
            }
        }
        lock.lock();
        if (wanted && last) {
            std::swap(back_, ready_);
            fresh_ = true;
        }
        busy_ = false;
        if (jobs_.empty()) idle_.notify_all();
    }
}

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
    settings_.cellM = kCellM;
    settings_.combustion = ff::combustion();
    for (Packed& b : buffers_) {
        b.atlas.assign(static_cast<std::size_t>(kAtlas) * kAtlas * kAtlas, vsg::usvec2(0, 0));
        b.hash.assign(kHash, vsg::uivec4(0, 0, 0, 0));
    }
    front_ = &buffers_[0], ready_ = &buffers_[1], back_ = &buffers_[2];
    {
        // the solver's thread, and Flow made in it; its answer waited for here, once
        auto started = std::make_shared<std::promise<bool>>();
        std::future<bool> answer = started->get_future();
        worker_ = std::thread(&FlowVolumes::work, this, std::static_pointer_cast<void>(started), settings_);
        running_ = answer.get();
    }
    if (!running_) {
        worker_.join();
        LOG_INFO("world") << "NVIDIA Flow not running (nvflow.dll, nvflowext.dll not beside the viewer, or no device for it)";
        return;
    }
    pool_ = std::make_unique<Pool>(std::clamp<unsigned>(std::thread::hardware_concurrency() / 2, 1, 8) - 1);
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
    {
        // the texture's data is the front buffer's own memory (not a copy: 12 MB a step is then copied once, to the GPU)
        vsg::Data::Properties properties{VK_FORMAT_R16G16_SFLOAT};
        properties.dataVariance = vsg::DYNAMIC_DATA;
        properties.allocatorType = vsg::ALLOCATOR_TYPE_NO_DELETE;
        atlas_ = vsg::usvec2Array3D::create(kAtlas, kAtlas, kAtlas, front_->atlas.data(), properties);
    }
    for (vsg::Data* d : {static_cast<vsg::Data*>(rows_.get()), static_cast<vsg::Data*>(frame_.get()), static_cast<vsg::Data*>(hash_.get())})
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
    stepIndex_ = -1;
    quietSince_ = -1.0;
    cleared_ = true;
    {
        // what the solver was asked and has packed since is of the grid before
        std::lock_guard<std::mutex> lock(mutex_);
        ++epoch_;
        jobs_.clear();
    }
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
    vsg::dvec3 at(event.positionEcef[0], event.positionEcef[1], event.positionEcef[2]);
    const vsg::dvec3 velocity(event.velocityEcef[0], event.velocityEcef[1], event.velocityEcef[2]);
    // a ground strike's spill (its Fire event has the ground's normal): its cloud of fuel stands on the surface, not
    // half in it - the solver has no ground, and what it puts below the surface is lost to the fireball
    if (const vsg::dvec3 normal(event.normalEcef[0], event.normalEcef[1], event.normalEcef[2]); fire && event.amountKg > 0.0 && vsg::length(normal) > 0.5)
        at += vsg::normalize(normal) * std::cbrt(event.amountKg); // (the spill's cloud is M^(1/3) m round: world/FlowFires.h)
    origin(at, event.simTime, ellipsoid);
    const double altitude = ellipsoid.convertECEFToLatLongAltitude(at).z;
    const double pressure = flowfires::pressureAt(air_, altitude);
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
    lastTime_ = now;

    // the leaks that burn now: each its component on its aircraft
    struct Burning {
        const Leak* leak;
        const sim::VehicleState* state;
        vsg::dvec3 at; ///< the component (body axes)
    };
    std::vector<Burning> burning;
    for (const Leak& l : leaks_) {
        if (!(l.rateKgS > 0.0) || now < l.since || l.slot >= states.size() || l.slot >= alive.size() || !alive[l.slot]) continue;
        const VehicleVisuals::Shape& shape = visuals.shape(l.slot);
        if (!shape.valid) continue;
        for (const auto& c : shape.components)
            if (c.name == l.component) {
                burning.push_back(Burning{&l, &states[l.slot], c.at});
                break;
            }
    }
    // (a leak burning on after the solver was left to rest: its frame made again, where the aircraft is now)
    if (!haveOrigin_ && !burning.empty()) origin(positionEcef(*burning.front().state), now, ellipsoid);

    if (haveOrigin_ || cleared_) {
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
        if (haveOrigin_)
            for (const Burning& fire : burning) {
                const Leak& l = *fire.leak;
                const sim::VehicleState& st = *fire.state;
                const vsg::dvec3 v = bodyAxisEcef(st, 0) * st.velocityBodyMs[0] + bodyAxisEcef(st, 1) * st.velocityBodyMs[1] +
                                     bodyAxisEcef(st, 2) * st.velocityBodyMs[2];
                holes.push_back(Hole{&l, toAir(bodyToEcef(st) * fire.at, now), velocityToAir(v), flowfires::pressureAt(air_, st.altitudeMslM)});
                bool laid = false;
                for (const Wake& w : wakes) laid = laid || w.slot == l.slot;
                if (!laid && l.slot < aircraft_.size() && aircraft_[l.slot].x > 0.0) {
                    const auto axis = [&](const vsg::dvec3& e) { return vsg::dvec3(vsg::dot(e, east_), vsg::dot(e, north_), vsg::dot(e, up_)); };
                    wakes.push_back(Wake{l.slot, toAir(positionEcef(st), now), velocityToAir(v), axis(bodyAxisEcef(st, 1)), -axis(bodyAxisEcef(st, 2)),
                                         flowfires::pressureAt(air_, st.altitudeMslM),
                                         std::max(216.65, air_.seaLevelK - 0.0065 * st.altitudeMslM)});
                }
            }
        // The solver's steps that have fallen due, each given with its sources where they are at its time (the
        // calibration's way: one call a step, world/FlowFires.h). The steps stand on a grid of the world's time, so a
        // replay's fire is stepped at the same times every run. After a slow frame (a screenshot saved, a hitch) the
        // steps missed are given together, a second of them at most (a longer gap - a seek forward - is not caught up).
        const double step = 1.0 / settings_.stepsPerSecond;
        const auto due = static_cast<std::int64_t>(std::floor(now / step + 1e-9));
        if (stepIndex_ < 0) stepIndex_ = due - 1;
        stepIndex_ = std::max(stepIndex_, due - static_cast<std::int64_t>(settings_.stepsPerSecond));
        const vsg::dvec3 eyeLocal = haveOrigin_ ? toAir(eye, now) : vsg::dvec3(0.0, 0.0, 0.0);
        std::vector<Job> given;
        while (stepIndex_ < due) {
            ++stepIndex_;
            const double t = static_cast<double>(stepIndex_) * step;
            sources_.clear();
            bursts_.erase(std::remove_if(bursts_.begin(), bursts_.end(),
                                         [&](const Burst& b) {
                                             const double age = t - b.time;
                                             return b.detonation ? !ff::detonation(b.amountKg, b.at, b.motion, b.pressurePa, age, step, b.luid, sources_)
                                                                 : !ff::spill(b.amountKg, b.at, b.motion, b.pressurePa, age, step, b.luid, sources_);
                                         }),
                          bursts_.end());
            for (const Wake& w : wakes) {
                const vsg::dvec3 a = w.at - w.velocity * (now - t);
                const double at[3] = {a.x, a.y, a.z}, velocity[3] = {w.velocity.x, w.velocity.y, w.velocity.z};
                const double right[3] = {w.right.x, w.right.y, w.right.z}, up[3] = {w.up.x, w.up.y, w.up.z};
                ff::wake(aircraft_[w.slot].y, aircraft_[w.slot].x, at, velocity, right, up, w.pressure, w.airK, step,
                         (1u << 20) + w.slot, sources_);
            }
            for (const Hole& h : holes) {
                const vsg::dvec3 a = h.at - h.velocity * (now - t);
                const double at[3] = {a.x, a.y, a.z}, velocity[3] = {h.velocity.x, h.velocity.y, h.velocity.z};
                ff::leak(h.leak->rateKgS, at, velocity, h.pressure, step, h.leak->luid, sources_);
            }
            Job job;
            job.dt = step;
            job.sources = sources_;
            job.clear = cleared_;
            job.epoch = epoch_;
            job.eye[0] = eyeLocal.x, job.eye[1] = eyeLocal.y, job.eye[2] = eyeLocal.z;
            given.push_back(std::move(job));
            cleared_ = false;
        }
        if (!given.empty()) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                for (Job& job : given) jobs_.push_back(std::move(job));
            }
            wake_.notify_one();
        }
    }
    else stepIndex_ = -1; // (at rest: the next fire's steps start at its own time, none owed for the time between)

    // the newest fields the solver's thread has packed: its buffer the texture's from here on, the one the texture
    // had its to pack into after the next
    bool taken = false;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (synchronous_) idle_.wait(lock, [this] { return jobs_.empty() && !busy_; });
        if (fresh_) {
            std::swap(front_, ready_);
            fresh_ = false;
            taken = true;
        }
        cost_.waiting = jobs_.size();
    }
    const bool drawn = haveOrigin_ && front_->epoch == epoch_;
    if (taken && drawn) {
        vsg::Data::Properties properties = atlas_->properties;
        atlas_->assign(kAtlas, kAtlas, kAtlas, front_->atlas.data(), properties); // (and dirty: uploaded this frame)
        std::copy(front_->hash.begin(), front_->hash.end(), hash_->begin());
        hash_->dirty();
        cost_.solverGpuMs = front_->solverGpuMs, cost_.workerMs = front_->workerMs, cost_.packMs = front_->packMs;
        cost_.leaves = front_->leaves, cost_.leavesRead = front_->leavesRead, cost_.readbackBytes = front_->bytes;
    }
    if (drawn && front_->leaves > 0 && front_->voxel > 0.0) {
        // the pages (8^3 leaves): each the box round its leaves, drawn far to near from where the eye is now
        const Packed& f = *front_;
        const vsg::dvec3 eyeLocal = toAir(eye, now);
        const double leafM = 8.0 * f.voxel;
        std::vector<std::pair<double, const Page*>> order;
        order.reserve(f.pages.size());
        for (const Page& page : f.pages) {
            // (by the page's own cell's middle, not its box's: equal cells of a grid, far to near, are drawn in order)
            vsg::dvec3 c;
            for (std::size_t a = 0; a < 3; ++a) c[a] = f.zero[a] + ((page.lo[a] >> 3) * 8 + 4) * leafM;
            order.emplace_back(vsg::length(c - eyeLocal), &page);
        }
        std::sort(order.begin(), order.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
        if (order.size() > kPages) order.erase(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(order.size() - kPages));
        const auto F = [](double v) { return static_cast<float>(v); };
        for (std::uint32_t k = 0; k < order.size(); ++k) {
            const Page& p = *order[k].second;
            vsg::dvec3 lo, size;
            for (std::size_t a = 0; a < 3; ++a) lo[a] = f.zero[a] + p.lo[a] * leafM, size[a] = (p.hi[a] - p.lo[a] + 1) * leafM;
            rows_->set(kPagesAt + 2 * k, vsg::vec4(F(lo.x), F(lo.y), F(lo.z), 1.0f));
            rows_->set(kPagesAt + 2 * k + 1, vsg::vec4(F(size.x), F(size.y), F(size.z), 0.0f));
        }
        cost_.pages = order.size();
        draw_->instanceCount = static_cast<std::uint32_t>(order.size());
    }
    else {
        draw_->instanceCount = 0;
        cost_.pages = 0;
        if (!drawn) cost_.leaves = cost_.leavesRead = 0;
    }
    // Nothing burning and nothing left of what did (its smoke thinned away, the solver's blocks freed): the solver is
    // left to rest - no step asked of it, its frame given up, made again round the next fire - until there is one.
    if (haveOrigin_ && bursts_.empty() && burning.empty() && drawn && front_->leavesRead == 0 && cost_.waiting == 0) {
        if (quietSince_ < 0.0) quietSince_ = now;
        else if (now - quietSince_ > kRestAfterS) {
            haveOrigin_ = false;
            cleared_ = true; // (one more step, emptying the grid of what moves in it unseen)
            quietSince_ = -1.0;
        }
    }
    else quietSince_ = -1.0;

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
        const Packed& f = *front_;
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
    cost_.cpuMs = msSince(t0);
}

void FlowVolumes::pack(const FlowSolver::Fields& f, const vsg::dvec3& eyeLocal, Packed& out) {
    out.leavesRead = f.leaves.size();
    out.bytes = f.bytes;
    out.voxel = f.voxel;
    std::copy(f.zero, f.zero + 3, out.zero);
    out.pages.clear();
    out.leaves = 0;
    if (f.leaves.empty() || !(f.voxel > 0.0)) return;
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
    out.leaves = drawn.size();
    // their bricks, and the hash the shader finds them by
    slots_.clear();
    std::fill(out.hash.begin(), out.hash.end(), vsg::uivec4(0, 0, 0, 0));
    for (std::uint32_t s = 0; s < drawn.size(); ++s) {
        const int* o = drawn[s]->origin;
        const int L[3] = {o[0] >> 3, o[1] >> 3, o[2] >> 3};
        slots_[leafKey(L[0], L[1], L[2])] = s;
        std::uint32_t h = leafHash(L[0], L[1], L[2]) & (kHash - 1);
        while (out.hash[h].w != 0) h = (h + 1) & (kHash - 1);
        out.hash[h] = vsg::uivec4(static_cast<std::uint32_t>(L[0]), static_cast<std::uint32_t>(L[1]), static_cast<std::uint32_t>(L[2]), s + 1);
    }
    vsg::usvec2* atlas = out.atlas.data();
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
    pool_->each(static_cast<std::uint32_t>(drawn.size()), fill);
    // the pages (8^3 leaves): each the box round its leaves
    std::unordered_map<std::uint64_t, Page> pages;
    for (const FlowSolver::Leaf* l : drawn) {
        const int L[3] = {l->origin[0] >> 3, l->origin[1] >> 3, l->origin[2] >> 3};
        auto [it, fresh] = pages.try_emplace(leafKey(L[0] >> 3, L[1] >> 3, L[2] >> 3), Page{{L[0], L[1], L[2]}, {L[0], L[1], L[2]}});
        if (!fresh)
            for (int a = 0; a < 3; ++a) it->second.lo[a] = std::min(it->second.lo[a], L[a]), it->second.hi[a] = std::max(it->second.hi[a], L[a]);
    }
    out.pages.reserve(pages.size());
    for (const auto& [key, page] : pages) {
        (void)key;
        out.pages.push_back(page);
    }
    // (in an order of their own, whatever the map's: equal distances then sort alike every run)
    std::sort(out.pages.begin(), out.pages.end(), [](const Page& a, const Page& b) {
        return std::lexicographical_compare(a.lo, a.lo + 3, b.lo, b.lo + 3);
    });
}

} // namespace fsim::world
