#include "visualization/vfx/effects/FlowVolumes.h"

#include "foundation/core/Log.h"
#include "foundation/platform/Threads.h"
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
constexpr std::uint32_t kAtlas = FlowVolumes::kBricks * FlowVolumes::kBrick;            // density/temp texels a side
constexpr std::uint32_t kLightAtlas = FlowVolumes::kBricks * FlowVolumes::kLightBrick;  // precomputed light-grid texels a side
constexpr std::uint32_t kMaxBricks = FlowVolumes::kBricks * FlowVolumes::kBricks * FlowVolumes::kBricks;

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
            solver.step(job.dt, job.sources, job.clear, wait, job.ground.enabled ? &job.ground : nullptr);
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
    lightRoot_ = vsg::Group::create();
    marchRoot_ = vsg::Group::create();
    compositeRoot_ = vsg::Group::create();
    const CloudNoise noise = shaders.empty() ? CloudNoise{} : loadCloudNoise(shaders.parent_path());
    if (shaders.empty() || !tables.valid() || !noise.valid()) {
        LOG_WARN("world") << "no Flow volumes drawn: the clouds' shaders, noises or the atmosphere's tables are missing";
        return;
    }
    shaderDir_ = shaders;
    tables_ = tables;
    noiseShape_ = noise.shape;
    settings_.cellM = kCellM;
    settings_.combustion = ff::combustion();
    for (Packed& b : buffers_) {
        b.atlas.assign(static_cast<std::size_t>(kAtlas) * kAtlas * kAtlas, vsg::usvec2(0, 0));
        b.hash.assign(kHash, vsg::uivec4(0, 0, 0, 0));
        b.brickCoords.reserve(kMaxBricks);
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
                            "\n#define FLOW_BRICK " + std::to_string(kBrick) + "\n#define FLOW_LIGHT_BRICK " + std::to_string(kLightBrick) +
                            "\n#define FLOW_HASH " + std::to_string(kHash) + "\n";
    auto vertex = loadCloudShader(shaders, "flow.vert", VK_SHADER_STAGE_VERTEX_BIT, cap);
    auto fragment = loadCloudShader(shaders, "flow.frag", VK_SHADER_STAGE_FRAGMENT_BIT, cap);
    auto lightShader = loadCloudShader(shaders, "flow_light.comp", VK_SHADER_STAGE_COMPUTE_BIT, cap);
    vsg::ShaderStages stages{vertex, fragment};
    vsg::ShaderStages lightStages{lightShader};
    auto compiler = vsg::ShaderCompiler::create();
    if (!vertex || !fragment || !lightShader || !compiler->supported() || !compiler->compile(stages) || !compiler->compile(lightStages)) {
        LOG_WARN("world") << "the Flow volume's shaders did not build; none drawn";
        return;
    }
    rows_ = vsg::vec4Array::create(kPagesAt + 2 * kPages, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    // Valid orthonormal default axes & voxel for frame-0 pipeline warmup before any fire occurs.
    rows_->set(0, vsg::vec4(0.0f, 0.0f, 0.0f, static_cast<float>(kCellM)));
    rows_->set(1, vsg::vec4(1.0f, 0.0f, 0.0f, 0.0f));
    rows_->set(2, vsg::vec4(0.0f, 1.0f, 0.0f, 0.0f));
    rows_->set(3, vsg::vec4(0.0f, 0.0f, 1.0f, 0.0f));
    rows_->set(5, vsg::vec4(1.0f, static_cast<float>(kSootAlbedo), static_cast<float>(ff::kFlameK - ff::kAmbientK), static_cast<float>(ff::kAmbientK)));
    blackbody_.write(*rows_, kRows);
    frame_ = vsg::vec4Array::create(4, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    frame_->set(2, vsg::vec4(0.0f, 0.0f, 1.0f, 0.0f));
    hash_ = vsg::uivec4Array::create(kHash, vsg::uivec4(0, 0, 0, 0));
    brickCoords_ = vsg::ivec4Array::create(kMaxBricks, vsg::ivec4(0, 0, 0, 0));
    {
        // the texture's data is the front buffer's own memory (not a copy: 12 MB a step is then copied once, to the GPU)
        vsg::Data::Properties properties{VK_FORMAT_R16G16_SFLOAT};
        properties.dataVariance = vsg::DYNAMIC_DATA;
        properties.allocatorType = vsg::ALLOCATOR_TYPE_NO_DELETE;
        atlas_ = vsg::usvec2Array3D::create(kAtlas, kAtlas, kAtlas, front_->atlas.data(), properties);
    }
    for (vsg::Data* d : {static_cast<vsg::Data*>(rows_.get()), static_cast<vsg::Data*>(frame_.get()),
                         static_cast<vsg::Data*>(hash_.get()), static_cast<vsg::Data*>(brickCoords_.get())})
        d->properties.dataVariance = vsg::DYNAMIC_DATA;
    atlasSampler_ = vsg::Sampler::create();
    atlasSampler_->minFilter = atlasSampler_->magFilter = VK_FILTER_LINEAR;
    atlasSampler_->mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    atlasSampler_->addressModeU = atlasSampler_->addressModeV = atlasSampler_->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    atlasSampler_->maxLod = 0.0f;

    // Precomputed 3D sun & sky transmittance grid (80^3 RG16F, one 5x5x5 brick per active Flow leaf; VIS-M12).
    {
        auto lightImage = vsg::Image::create();
        lightImage->imageType = VK_IMAGE_TYPE_3D;
        lightImage->format = VK_FORMAT_R16G16_SFLOAT;
        lightImage->extent = VkExtent3D{kLightAtlas, kLightAtlas, kLightAtlas};
        lightImage->mipLevels = 1;
        lightImage->arrayLayers = 1;
        lightImage->samples = VK_SAMPLE_COUNT_1_BIT;
        lightImage->tiling = VK_IMAGE_TILING_OPTIMAL;
        lightImage->usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        auto lightView = vsg::ImageView::create(lightImage, VK_IMAGE_ASPECT_COLOR_BIT);
        lightView->viewType = VK_IMAGE_VIEW_TYPE_3D;
        auto lightSampler = vsg::Sampler::create();
        lightSampler->minFilter = lightSampler->magFilter = VK_FILTER_LINEAR;
        lightSampler->mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        lightSampler->addressModeU = lightSampler->addressModeV = lightSampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        lightSampler->maxLod = 0.0f;
        lightSampleInfo_ = vsg::ImageInfo::create(lightSampler, lightView, VK_IMAGE_LAYOUT_GENERAL);
        auto lightStorageInfo = vsg::ImageInfo::create(vsg::ref_ptr<vsg::Sampler>{}, lightView, VK_IMAGE_LAYOUT_GENERAL);

        const VkShaderStageFlags cs = VK_SHADER_STAGE_COMPUTE_BIT;
        auto lightSetLayout = vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{
            {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, cs, nullptr},
            {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, cs, nullptr},
            {3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, cs, nullptr},
            {4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {10, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, cs, nullptr}});
        auto lightPipelineLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{lightSetLayout}, vsg::PushConstantRanges{});
        auto lightSet = vsg::DescriptorSet::create(
            lightSetLayout,
            vsg::Descriptors{vsg::DescriptorBuffer::create(rows_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                             vsg::DescriptorBuffer::create(frame_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
                             vsg::DescriptorImage::create(atlasSampler_, atlas_, 2, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                             vsg::DescriptorImage::create(noiseShape_, 3, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                             vsg::DescriptorBuffer::create(brickCoords_, 4, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                             vsg::DescriptorBuffer::create(hash_, 5, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                             vsg::DescriptorImage::create(lightStorageInfo, 10, 0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE)});
        auto pass = vsg::Group::create();
        auto begin = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                                                  VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0);
        begin->imageMemoryBarriers.push_back(vsg::ImageMemoryBarrier::create(
            VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
            VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, lightImage, VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}));
        pass->addChild(begin);
        auto computeGroup = vsg::StateGroup::create();
        computeGroup->add(vsg::BindComputePipeline::create(vsg::ComputePipeline::create(lightPipelineLayout, lightShader)));
        computeGroup->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_COMPUTE, lightPipelineLayout, 0, lightSet));
        lightDispatch_ = vsg::Dispatch::create(1, 1, 1);
        computeGroup->addChild(lightDispatch_);
        pass->addChild(computeGroup);
        auto done = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0);
        done->memoryBarriers.push_back(vsg::MemoryBarrier::create(VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT));
        pass->addChild(done);
        lightSwitch_ = vsg::Switch::create();
        lightSwitch_->addChild(true, pass); // kept ON until compiled() so VSG compiles the compute pipeline
        lightRoot_->addChild(lightSwitch_);
    }

    const VkShaderStageFlags both = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    vsg::DescriptorSetLayoutBindings bindings{{0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, both, nullptr},
                                              {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, both, nullptr},
                                              {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
                                              {3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
                                              {4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
                                              {5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}};
    for (std::uint32_t b : {6u, 7u, 8u, 20u}) bindings.push_back({b, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, both, nullptr});
    auto setLayout = vsg::DescriptorSetLayout::create(bindings);
    auto layout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout}, vsg::PushConstantRanges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}});
    vsg::Descriptors descriptors{vsg::DescriptorBuffer::create(rows_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                                 vsg::DescriptorBuffer::create(frame_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
                                 vsg::DescriptorImage::create(atlasSampler_, atlas_, 2, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                                 vsg::DescriptorImage::create(noiseShape_, 3, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                                 vsg::DescriptorImage::create(lightSampleInfo_, 4, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                                 vsg::DescriptorBuffer::create(hash_, 5, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER)};
    for (const auto& d : tables.descriptors()) descriptors.push_back(d);
    auto set = vsg::DescriptorSet::create(setLayout, descriptors);

    // premultiplied over, depth tested and not written; each page's box by its far faces (as a detonation's), the
    // pages far to near (fallback when buildOffscreen is not used)
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

bool FlowVolumes::buildOffscreen(vsg::ref_ptr<vsg::ImageInfo> sceneDepth, std::uint32_t width, std::uint32_t height) {
    if (!available() || offscreen_ || !lightSampleInfo_ || !sceneDepth || shaderDir_.empty()) return offscreen_;
    const std::string cap = "#define BLACKBODY_AT " + std::to_string(kRows) + "\n#define BLACKBODY_ROWS " + std::to_string(SunlitBlackbody::kRows) +
                            "\n#define PAGES_AT " + std::to_string(kPagesAt) + "\n#define FLOW_BRICKS " + std::to_string(kBricks) +
                            "\n#define FLOW_BRICK " + std::to_string(kBrick) + "\n#define FLOW_LIGHT_BRICK " + std::to_string(kLightBrick) +
                            "\n#define FLOW_HASH " + std::to_string(kHash) + "\n";
    auto marchShader = loadCloudShader(shaderDir_, "flow_march.comp", VK_SHADER_STAGE_COMPUTE_BIT, cap);
    auto compVert = loadCloudShader(shaderDir_, "flow_composite.vert", VK_SHADER_STAGE_VERTEX_BIT);
    auto compFrag = loadCloudShader(shaderDir_, "flow_composite.frag", VK_SHADER_STAGE_FRAGMENT_BIT);
    auto compiler = vsg::ShaderCompiler::create();
    vsg::ShaderStages marchStages{marchShader};
    vsg::ShaderStages compStages{compVert, compFrag};
    if (!marchShader || !compVert || !compFrag || !compiler->supported() || !compiler->compile(marchStages) || !compiler->compile(compStages)) {
        LOG_WARN("world") << "NVIDIA Flow offscreen half-res shaders did not build; falling back to direct page draw";
        return false;
    }

    unsigned sw = 0, sh = 0;
    platform::screenSize(sw, sh);
    const std::uint32_t maxW = std::max({width, sw, 1u});
    const std::uint32_t maxH = std::max({height, sh, 1u});
    allocHalfW_ = std::max((maxW + 1u) / 2u, std::min(maxW, 2048u));
    allocHalfH_ = std::max((maxH + 1u) / 2u, std::min(maxH, 1152u));

    const auto makeImage = [&](VkFormat format) {
        auto img = vsg::Image::create();
        img->imageType = VK_IMAGE_TYPE_2D;
        img->format = format;
        img->extent = VkExtent3D{allocHalfW_, allocHalfH_, 1};
        img->mipLevels = 1;
        img->arrayLayers = 1;
        img->samples = VK_SAMPLE_COUNT_1_BIT;
        img->tiling = VK_IMAGE_TILING_OPTIMAL;
        img->usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        return img;
    };
    auto halfColor = makeImage(VK_FORMAT_R16G16B16A16_SFLOAT);
    auto halfFront = makeImage(VK_FORMAT_R32_SFLOAT);
    auto halfColorView = vsg::ImageView::create(halfColor, VK_IMAGE_ASPECT_COLOR_BIT);
    auto halfFrontView = vsg::ImageView::create(halfFront, VK_IMAGE_ASPECT_COLOR_BIT);
    auto nearest = vsg::Sampler::create();
    nearest->minFilter = nearest->magFilter = VK_FILTER_NEAREST;
    nearest->mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    nearest->addressModeU = nearest->addressModeV = nearest->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    nearest->maxLod = 0.0f;

    marchParams_ = vsg::vec4Array::create(11, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    marchParams_->properties.dataVariance = vsg::DYNAMIC_DATA;
    compositeParams_ = vsg::vec4Array::create(8, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    compositeParams_->properties.dataVariance = vsg::DYNAMIC_DATA;
    compositeParams_->set(6, vsg::vec4(0.0f, 0.0f, -1.0f, -1.0f)); // safe discard until active

    // Half-res front-to-back compute raymarch pass (betweenPass).
    {
        const VkShaderStageFlags cs = VK_SHADER_STAGE_COMPUTE_BIT;
        vsg::DescriptorSetLayoutBindings bindings{
            {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, cs, nullptr},
            {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, cs, nullptr},
            {3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, cs, nullptr},
            {4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, cs, nullptr},
            {5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {11, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, cs, nullptr},
            {12, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, cs, nullptr},
            {13, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, cs, nullptr},
            {14, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, cs, nullptr}};
        for (std::uint32_t b : {6u, 7u, 8u, 20u}) bindings.push_back({b, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, cs, nullptr});
        auto marchSetLayout = vsg::DescriptorSetLayout::create(bindings);
        auto marchLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{marchSetLayout}, vsg::PushConstantRanges{});
        vsg::Descriptors desc{
            vsg::DescriptorBuffer::create(rows_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
            vsg::DescriptorBuffer::create(frame_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
            vsg::DescriptorImage::create(atlasSampler_, atlas_, 2, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
            vsg::DescriptorImage::create(noiseShape_, 3, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
            vsg::DescriptorImage::create(lightSampleInfo_, 4, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
            vsg::DescriptorBuffer::create(hash_, 5, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
            vsg::DescriptorBuffer::create(marchParams_, 11, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
            vsg::DescriptorImage::create(sceneDepth, 12, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
            vsg::DescriptorImage::create(vsg::ImageInfo::create(vsg::ref_ptr<vsg::Sampler>{}, halfColorView, VK_IMAGE_LAYOUT_GENERAL), 13, 0,
                                         VK_DESCRIPTOR_TYPE_STORAGE_IMAGE),
            vsg::DescriptorImage::create(vsg::ImageInfo::create(vsg::ref_ptr<vsg::Sampler>{}, halfFrontView, VK_IMAGE_LAYOUT_GENERAL), 14, 0,
                                         VK_DESCRIPTOR_TYPE_STORAGE_IMAGE)};
        for (const auto& d : tables_.descriptors()) desc.push_back(d);
        auto marchSet = vsg::DescriptorSet::create(marchSetLayout, desc);

        auto pass = vsg::Group::create();
        auto begin = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT,
                                                  VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0);
        for (const auto& img : {halfColor, halfFront})
            begin->imageMemoryBarriers.push_back(vsg::ImageMemoryBarrier::create(
                VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, img, VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}));
        pass->addChild(begin);

        auto group = vsg::StateGroup::create();
        group->add(vsg::BindComputePipeline::create(vsg::ComputePipeline::create(marchLayout, marchShader)));
        group->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_COMPUTE, marchLayout, 0, marchSet));
        marchDispatch_ = vsg::Dispatch::create(1, 1, 1);
        group->addChild(marchDispatch_);
        pass->addChild(group);

        auto done = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0);
        done->memoryBarriers.push_back(vsg::MemoryBarrier::create(VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT));
        pass->addChild(done);

        marchSwitch_ = vsg::Switch::create();
        marchSwitch_->addChild(true, pass); // kept ON until compiled()
        marchRoot_->addChild(marchSwitch_);
    }

    // Full-resolution depth-aware bilateral composite draw (overScene).
    {
        const VkShaderStageFlags fs = VK_SHADER_STAGE_FRAGMENT_BIT;
        auto compSetLayout = vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{
            {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, fs, nullptr},
            {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr},
            {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr},
            {3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr}});
        auto compLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{compSetLayout},
                                                      vsg::PushConstantRanges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}});
        auto compSet = vsg::DescriptorSet::create(
            compSetLayout,
            vsg::Descriptors{vsg::DescriptorBuffer::create(compositeParams_, 0, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
                             vsg::DescriptorImage::create(sceneDepth, 1, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                             vsg::DescriptorImage::create(vsg::ImageInfo::create(nearest, halfColorView, VK_IMAGE_LAYOUT_GENERAL), 2, 0,
                                                          VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                             vsg::DescriptorImage::create(vsg::ImageInfo::create(nearest, halfFrontView, VK_IMAGE_LAYOUT_GENERAL), 3, 0,
                                                          VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER)});
        auto raster = vsg::RasterizationState::create();
        raster->cullMode = VK_CULL_MODE_NONE;
        auto depth = vsg::DepthStencilState::create();
        depth->depthTestEnable = VK_FALSE;
        depth->depthWriteEnable = VK_FALSE;
        auto blend = vsg::ColorBlendState::create();
        blend->attachments = {VkPipelineColorBlendAttachmentState{
            VK_TRUE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD, VK_BLEND_FACTOR_ONE,
            VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};
        vsg::GraphicsPipelineStates states{vsg::VertexInputState::create(), vsg::InputAssemblyState::create(), raster,
                                           vsg::MultisampleState::create(), blend, depth};
        auto group = vsg::StateGroup::create();
        group->add(vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(compLayout, compStages, states)));
        group->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, compLayout, 0, compSet));
        group->addChild(vsg::Draw::create(3, 1, 0, 0));
        compositeSwitch_ = vsg::Switch::create();
        compositeSwitch_->addChild(true, group); // kept ON until compiled()
        compositeRoot_->addChild(compositeSwitch_);
    }

    offscreen_ = true;
    setScreen(width, height);
    LOG_INFO("world") << "NVIDIA Flow: offscreen half-res front-to-back raymarch + bilateral composite ready (" << allocHalfW_ << "x" << allocHalfH_ << ")";
    return true;
}

void FlowVolumes::compiled() {
    if (offscreen_ && switch_) switch_->setAllChildren(false);
    if (lightSwitch_) lightSwitch_->setAllChildren(false);
    if (marchSwitch_) marchSwitch_->setAllChildren(false);
    if (compositeSwitch_) compositeSwitch_->setAllChildren(false);
}

void FlowVolumes::setAircraft(std::size_t slot, double wingspanM, double massKg) {
    if (slot >= aircraft_.size()) aircraft_.resize(slot + 1, vsg::dvec2(0.0, 0.0));
    aircraft_[slot] = vsg::dvec2(std::max(wingspanM, 0.0), std::max(massKg, 0.0));
}

double FlowVolumes::nearFieldThinning() { return -std::log(1.0 - ff::kSmokeDilution); }

void FlowVolumes::setScreen(std::uint32_t width, std::uint32_t height) {
    screenWidth_ = std::max<std::uint32_t>(width, 1);
    screenHeight_ = std::max<std::uint32_t>(height, 1);
    coarse_ = shadingRate_ && static_cast<double>(width) * static_cast<double>(height) >= 2.0e6 ? 2u : 1u;
    if (shadingRate_) static_cast<ShadingRate*>(shadingRate_.get())->size = coarse_;
}

void FlowVolumes::setVisible(bool on) {
    if (on == visible_) return;
    visible_ = on;
    if (switch_) switch_->setAllChildren(on && !offscreen_);
    if (!on) clear();
}

void FlowVolumes::clear() {
    bursts_.clear();
    leaks_.clear();
    pools_.clear();
    groundPlane_ = {};
    haveOrigin_ = false;
    lastTime_ = -1.0;
    stepIndex_ = -1;
    quietSince_ = -1.0;
    cleared_ = true;
    lightValid_ = false;
    active_ = false;
    {
        // what the solver was asked and has packed since is of the grid before
        std::lock_guard<std::mutex> lock(mutex_);
        ++epoch_;
        jobs_.clear();
    }
    if (draw_) draw_->instanceCount = 0;
    if (lightSwitch_) lightSwitch_->setAllChildren(false);
    if (marchSwitch_) marchSwitch_->setAllChildren(false);
    if (compositeSwitch_) compositeSwitch_->setAllChildren(false);
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
    const vsg::dvec3 surfaceEcef = at;
    const vsg::dvec3 velocity(event.velocityEcef[0], event.velocityEcef[1], event.velocityEcef[2]);
    // a ground strike's spill (its Fire event has the ground's normal): its cloud of fuel stands on the surface,
    // bounded below by the solid ground collider (B-53), and leaves a burning surface pool slick.
    const vsg::dvec3 normal(event.normalEcef[0], event.normalEcef[1], event.normalEcef[2]);
    const bool groundStrike = fire && event.amountKg > 0.0 && vsg::length(normal) > 0.5;
    if (groundStrike)
        at += vsg::normalize(normal) * std::cbrt(event.amountKg); // (the spill's cloud is M^(1/3) m round: world/FlowFires.h)
    origin(at, event.simTime, ellipsoid);
    const double altitude = ellipsoid.convertECEFToLatLongAltitude(at).z;
    const double pressure = flowfires::pressureAt(air_, altitude);
    const vsg::dvec3 p = toAir(at, event.simTime), v = velocityToAir(velocity);
    if (groundStrike) {
        const vsg::dvec3 surf = toAir(surfaceEcef, event.simTime);
        groundPlane_.enabled = true;
        groundPlane_.z = surf.z;
        groundPlane_.centerX = surf.x;
        groundPlane_.centerY = surf.y;
        groundPlane_.halfExtent = 250.0;
        PoolFire pool;
        pool.spilledKg = event.amountKg;
        // SFPE JP-8 surface pool burning rate (~0.055 kg/(m^2*s) over slick area, capped at 4 kg/s).
        const double poolR = ff::poolRadius(event.amountKg);
        const double sfpeRate = 0.055 * 3.141592653589793 * poolR * poolR;
        pool.rateKgS = std::clamp(event.rateKgS > 0.0 ? event.rateKgS : sfpeRate, 0.4, 4.0);
        pool.since = event.simTime;
        pool.pressurePa = pressure;
        pool.at[0] = surf.x, pool.at[1] = surf.y, pool.at[2] = surf.z;
        pool.luid = nextLuid_;
        nextLuid_ += 2;
        pools_.push_back(pool);
    }
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
                         const vsg::dmat4& projection, const vsg::dvec3& eye, const vsg::EllipsoidModel& ellipsoid, const vsg::dvec3& sunEcef,
                         const vsg::dmat4* view) {
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
            const vsg::dvec3 wind = windLocal();
            pools_.erase(std::remove_if(pools_.begin(), pools_.end(),
                                        [&](const PoolFire& pool) {
                                            const double age = t - pool.since;
                                            const double onset = 0.55 * ff::ccpsDuration(pool.spilledKg);
                                            const double life = std::clamp(onset + 14.0 + 0.006 * pool.spilledKg, 14.0, 26.0);
                                            if (age < onset) return false;
                                            if (age > life) return true;
                                            const double rampIn = std::clamp((age - onset) / 1.5, 0.0, 1.0);
                                            const double fadeOut = age > 0.65 * life ? std::max(0.0, 1.0 - (age - 0.65 * life) / (0.35 * life)) : 1.0;
                                            const double at[3] = {pool.at[0] - wind.x * age, pool.at[1] - wind.y * age, pool.at[2]};
                                            ff::poolFire(pool.spilledKg, pool.rateKgS * rampIn * fadeOut, at, pool.pressurePa, age - onset, pool.luid, sources_);
                                            return false;
                                        }),
                         pools_.end());
            Job job;
            job.dt = step;
            job.sources = sources_;
            job.ground = groundPlane_;
            if (job.ground.enabled) {
                const double dtGround = t - originTime_;
                job.ground.centerX -= wind.x * dtGround;
                job.ground.centerY -= wind.y * dtGround;
            }
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
    bool lightDirty = false;
    if (taken && drawn) {
        vsg::Data::Properties properties = atlas_->properties;
        atlas_->assign(kAtlas, kAtlas, kAtlas, front_->atlas.data(), properties); // (and dirty: uploaded this frame)
        std::copy(front_->hash.begin(), front_->hash.end(), hash_->begin());
        hash_->dirty();
        if (brickCoords_ && !front_->brickCoords.empty()) {
            const std::size_t nb = std::min<std::size_t>(front_->brickCoords.size(), brickCoords_->size());
            std::copy(front_->brickCoords.begin(), front_->brickCoords.begin() + static_cast<std::ptrdiff_t>(nb), brickCoords_->begin());
            brickCoords_->dirty();
        }
        lightDirty = true;
        cost_.solverGpuMs = front_->solverGpuMs, cost_.workerMs = front_->workerMs, cost_.packMs = front_->packMs;
        cost_.leaves = front_->leaves, cost_.leavesRead = front_->leavesRead, cost_.readbackBytes = front_->bytes;
    }

    const vsg::dvec3 sun = vsg::normalize(sunEcef);
    if (vsg::length2(sun - lastLightSun_) > 1e-8) lightDirty = true;

    const bool useOffscreen = offscreen_ && view != nullptr;
    double marchScale = static_cast<double>(coarse_);
    if (useOffscreen) {
        marchScale = static_cast<double>(screenWidth_) * static_cast<double>(screenHeight_) >= 2.0e6 ? 2.0 : 1.0;
        const double minScaleX = static_cast<double>(screenWidth_) / std::max<std::uint32_t>(allocHalfW_, 1);
        const double minScaleY = static_cast<double>(screenHeight_) / std::max<std::uint32_t>(allocHalfH_, 1);
        marchScale = std::max({marchScale, minScaleX, minScaleY});
    }

    {
        // the frame, as a detonation's: the eye, a pixel's angle, the altitude correction, the sun, the night light
        const vsg::dvec3 lla = ellipsoid.convertECEFToLatLongAltitude(eye);
        const vsg::dvec3 surface = ellipsoid.convertLatLongAltitudeToECEF(vsg::dvec3(lla.x, lla.y, 0.0));
        const double a2 = ellipsoid.radiusEquator() * ellipsoid.radiusEquator(), b2 = ellipsoid.radiusPolar() * ellipsoid.radiusPolar();
        const vsg::dvec3 normal = vsg::normalize(vsg::dvec3(surface.x / a2, surface.y / a2, surface.z / b2));
        const vsg::dvec3 correction = -(surface - normal * kBottomRadius);
        const double pixel = 1.0 / std::max(1.0, 0.5 * std::abs(projection[1][1]) * static_cast<double>(screenHeight_));
        const Daylight d = daylightAt(sunEcef, normal);
        const double sunUp = vsg::dot(sun, vsg::normalize(eye));
        const double night = (1.0 - std::clamp((sunUp + 0.10) / 0.25, 0.0, 1.0)) * d.ambient;
        const auto F = [](double v) { return static_cast<float>(v); };
        frame_->set(0, vsg::vec4(F(eye.x), F(eye.y), F(eye.z), F(pixel * marchScale)));
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
    }

    if (drawn && front_->leaves > 0 && front_->voxel > 0.0) {
        const Packed& f = *front_;
        const vsg::dvec3 eyeLocal = toAir(eye, now);
        const double leafM = 8.0 * f.voxel;
        std::vector<std::pair<double, const Page*>> order;
        order.reserve(f.pages.size());
        for (const Page& page : f.pages) {
            vsg::dvec3 c;
            for (std::size_t a = 0; a < 3; ++a) c[a] = f.zero[a] + ((page.lo[a] >> 3) * 8 + 4) * leafM;
            order.emplace_back(vsg::length(c - eyeLocal), &page);
        }
        if (useOffscreen) {
            // Front-to-back for single-pass compute raymarching with cross-page early ray termination (B-60).
            std::sort(order.begin(), order.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
            if (order.size() > kPages) order.resize(kPages);
        } else {
            // Back-to-front for legacy per-page graphics compositing.
            std::sort(order.begin(), order.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
            if (order.size() > kPages) order.erase(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(order.size() - kPages));
        }
        const auto F = [](double v) { return static_cast<float>(v); };
        for (std::uint32_t k = 0; k < order.size(); ++k) {
            const Page& p = *order[k].second;
            vsg::dvec3 lo, size;
            for (std::size_t a = 0; a < 3; ++a) lo[a] = f.zero[a] + p.lo[a] * leafM, size[a] = (p.hi[a] - p.lo[a] + 1) * leafM;
            rows_->set(kPagesAt + 2 * k, vsg::vec4(F(lo.x), F(lo.y), F(lo.z), 1.0f));
            rows_->set(kPagesAt + 2 * k + 1, vsg::vec4(F(size.x), F(size.y), F(size.z), 0.0f));
        }
        rows_->dirty();
        cost_.pages = order.size();
        draw_->instanceCount = useOffscreen ? 0u : static_cast<std::uint32_t>(order.size());

        // Refresh the precomputed 3D light grid only when solver data or sun direction changed.
        if (lightSwitch_ && lightDispatch_) {
            if (!lightValid_ || lightDirty) {
                lightDispatch_->groupCountX = static_cast<std::uint32_t>(f.leaves);
                lightDispatch_->groupCountY = 1;
                lightDispatch_->groupCountZ = 1;
                lightSwitch_->setAllChildren(true);
                lightValid_ = true;
                lastLightSun_ = sun;
            } else {
                lightSwitch_->setAllChildren(false);
            }
        }

        if (useOffscreen) {
            const vsg::dvec3 w = windLocal() * (now - originTime_);
            const vsg::dvec3 localOrigin = origin_ + east_ * w.x + north_ * w.y + up_ * w.z;
            vsg::dmat4 localToEcef;
            localToEcef(0, 0) = east_.x;  localToEcef(0, 1) = east_.y;  localToEcef(0, 2) = east_.z;  localToEcef(0, 3) = 0.0;
            localToEcef(1, 0) = north_.x; localToEcef(1, 1) = north_.y; localToEcef(1, 2) = north_.z; localToEcef(1, 3) = 0.0;
            localToEcef(2, 0) = up_.x;    localToEcef(2, 1) = up_.y;    localToEcef(2, 2) = up_.z;    localToEcef(2, 3) = 0.0;
            localToEcef(3, 0) = localOrigin.x; localToEcef(3, 1) = localOrigin.y; localToEcef(3, 2) = localOrigin.z; localToEcef(3, 3) = 1.0;
            const vsg::dmat4 toClip = projection * (*view) * localToEcef;
            double sLeft = 1e30, sRight = -1e30, sBottom = 1e30, sTop = -1e30;
            bool behind = false;
            for (std::uint32_t k = 0; k < order.size() && !behind; ++k) {
                const Page& p = *order[k].second;
                const double x0 = f.zero[0] + p.lo[0] * leafM, x1 = f.zero[0] + (p.hi[0] + 1) * leafM;
                const double y0 = f.zero[1] + p.lo[1] * leafM, y1 = f.zero[1] + (p.hi[1] + 1) * leafM;
                const double z0 = f.zero[2] + p.lo[2] * leafM, z1 = f.zero[2] + (p.hi[2] + 1) * leafM;
                for (int c = 0; c < 8; ++c) {
                    const vsg::dvec4 q = toClip * vsg::dvec4((c & 1) ? x1 : x0, (c & 2) ? y1 : y0, (c & 4) ? z1 : z0, 1.0);
                    if (q.w <= 0.0) {
                        behind = true;
                        break;
                    }
                    sLeft = std::min(sLeft, q.x / q.w);
                    sRight = std::max(sRight, q.x / q.w);
                    sBottom = std::min(sBottom, q.y / q.w);
                    sTop = std::max(sTop, q.y / q.w);
                }
            }
            const double ww = static_cast<double>(screenWidth_), wh = static_cast<double>(screenHeight_);
            const double uLeft = behind ? 0.0 : std::clamp((sLeft * 0.5 + 0.5) * ww, 0.0, ww);
            const double uRight = behind ? ww : std::clamp((sRight * 0.5 + 0.5) * ww, 0.0, ww);
            const double uBottom = behind ? 0.0 : std::clamp((sBottom * 0.5 + 0.5) * wh, 0.0, wh);
            const double uTop = behind ? wh : std::clamp((sTop * 0.5 + 0.5) * wh, 0.0, wh);
            const int halfW = static_cast<int>(std::min<std::uint32_t>(allocHalfW_, static_cast<std::uint32_t>(std::ceil(ww / marchScale))));
            const int halfH = static_cast<int>(std::min<std::uint32_t>(allocHalfH_, static_cast<std::uint32_t>(std::ceil(wh / marchScale))));
            const int tx0 = static_cast<int>(std::floor(uLeft / marchScale));
            const int ty0 = static_cast<int>(std::floor(uBottom / marchScale));
            const int tx1 = static_cast<int>(std::ceil(uRight / marchScale));
            const int ty1 = static_cast<int>(std::ceil(uTop / marchScale));
            const int rx0 = std::clamp(tx0 - 2, 0, halfW), ry0 = std::clamp(ty0 - 2, 0, halfH);
            const int rx1 = std::clamp(tx1 + 2, 0, halfW), ry1 = std::clamp(ty1 + 2, 0, halfH);
            const bool onScreen = uRight > uLeft && uTop > uBottom && rx1 > rx0 && ry1 > ry0;
            if (onScreen) {
                const vsg::dmat4 invProj = vsg::inverse(projection);
                const vsg::dmat4 viewToEcef = vsg::inverse(*view);
                for (std::size_t col = 0; col < 4; ++col) {
                    const vsg::vec4 ip(F(invProj[col][0]), F(invProj[col][1]), F(invProj[col][2]), F(invProj[col][3]));
                    marchParams_->set(static_cast<std::uint32_t>(col), ip);
                    compositeParams_->set(static_cast<std::uint32_t>(col), ip);
                    marchParams_->set(static_cast<std::uint32_t>(4 + col),
                                      vsg::vec4(F(viewToEcef[col][0]), F(viewToEcef[col][1]), F(viewToEcef[col][2]), F(viewToEcef[col][3])));
                }
                const vsg::vec4 screenVec(F(ww), F(wh), F(halfW), F(halfH));
                const vsg::vec4 regionVec(F(rx0), F(ry0), F(rx1), F(ry1));
                const vsg::vec4 countsVec(static_cast<float>(order.size()), F(marchScale), 0.0f, 0.0f);
                marchParams_->set(8, screenVec);
                marchParams_->set(9, regionVec);
                marchParams_->set(10, countsVec);
                marchParams_->dirty();

                const float lx0 = static_cast<float>(std::max(0.0, std::floor((tx0 - 1) * marchScale)));
                const float ly0 = static_cast<float>(std::max(0.0, std::floor((ty0 - 1) * marchScale)));
                const float lx1 = static_cast<float>(std::min(ww, std::ceil((tx1 + 1) * marchScale)));
                const float ly1 = static_cast<float>(std::min(wh, std::ceil((ty1 + 1) * marchScale)));
                compositeParams_->set(4, screenVec);
                compositeParams_->set(5, regionVec);
                compositeParams_->set(6, vsg::vec4(lx0, ly0, lx1, ly1));
                compositeParams_->set(7, countsVec);
                compositeParams_->dirty();

                marchDispatch_->groupCountX = static_cast<std::uint32_t>(rx1 - rx0 + 7) / 8;
                marchDispatch_->groupCountY = static_cast<std::uint32_t>(ry1 - ry0 + 7) / 8;
                marchDispatch_->groupCountZ = 1;
                marchSwitch_->setAllChildren(true);
                compositeSwitch_->setAllChildren(true);
                active_ = true;
                warmupPending_ = false;
            } else {
                marchSwitch_->setAllChildren(false);
                compositeSwitch_->setAllChildren(false);
                active_ = false;
            }
        }
    }
    else {
        if (haveOrigin_) rows_->dirty();
        draw_->instanceCount = 0;
        cost_.pages = 0;
        if (!drawn) cost_.leaves = cost_.leavesRead = 0;
        if (warmupPending_) {
            // Frame-0 1-workgroup shader cache warmup + initial image layout transition (B-62).
            if (lightSwitch_ && lightDispatch_) {
                lightDispatch_->groupCountX = 1;
                lightDispatch_->groupCountY = 1;
                lightDispatch_->groupCountZ = 1;
                lightSwitch_->setAllChildren(true);
            }
            if (offscreen_ && marchSwitch_ && compositeSwitch_ && marchDispatch_) {
                marchParams_->set(8, vsg::vec4(static_cast<float>(screenWidth_), static_cast<float>(screenHeight_),
                                               static_cast<float>(allocHalfW_), static_cast<float>(allocHalfH_)));
                marchParams_->set(9, vsg::vec4(0.0f, 0.0f, 1.0f, 1.0f));
                marchParams_->set(10, vsg::vec4(0.0f, 2.0f, 0.0f, 0.0f));
                marchParams_->dirty();
                compositeParams_->set(6, vsg::vec4(0.0f, 0.0f, -1.0f, -1.0f));
                compositeParams_->dirty();
                marchDispatch_->groupCountX = 1;
                marchDispatch_->groupCountY = 1;
                marchDispatch_->groupCountZ = 1;
                marchSwitch_->setAllChildren(true);
                compositeSwitch_->setAllChildren(true);
            }
            warmupPending_ = false;
            active_ = false;
        } else {
            if (lightSwitch_) lightSwitch_->setAllChildren(false);
            if (marchSwitch_) marchSwitch_->setAllChildren(false);
            if (compositeSwitch_) compositeSwitch_->setAllChildren(false);
            active_ = false;
        }
    }
    // Nothing burning and nothing left of what did (its smoke thinned away, the solver's blocks freed): the solver is
    // left to rest - no step asked of it, its frame given up, made again round the next fire - until there is one.
    if (haveOrigin_ && bursts_.empty() && burning.empty() && pools_.empty() && drawn && front_->leavesRead == 0 && cost_.waiting == 0) {
        if (quietSince_ < 0.0) quietSince_ = now;
        else if (now - quietSince_ > kRestAfterS) {
            haveOrigin_ = false;
            groundPlane_ = {};
            cleared_ = true; // (one more step, emptying the grid of what moves in it unseen)
            quietSince_ = -1.0;
        }
    }
    else quietSince_ = -1.0;

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
    out.brickCoords.resize(drawn.size());
    // their bricks, and the hash the shader finds them by
    slots_.clear();
    std::fill(out.hash.begin(), out.hash.end(), vsg::uivec4(0, 0, 0, 0));
    for (std::uint32_t s = 0; s < drawn.size(); ++s) {
        const int* o = drawn[s]->origin;
        const int L[3] = {o[0] >> 3, o[1] >> 3, o[2] >> 3};
        out.brickCoords[s] = vsg::ivec4(L[0], L[1], L[2], 0);
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
