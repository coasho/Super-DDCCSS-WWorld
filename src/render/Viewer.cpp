#include "render/Viewer.h"

#include <vsgImGui/RenderImGui.h>

#include <algorithm>
#include <chrono>
#include <thread>

#include <chrono>
#include <thread>

#include "foundation/core/Log.h"
#include "foundation/platform/Clock.h"
#include "foundation/platform/Paths.h"
#include "foundation/platform/Threads.h"

#include <vsgXchange/all.h>

namespace fsim::render {

void Viewer::quiesce(double graceSeconds) {
    if (!viewer_) return;
    viewer_->close(); // the window goes now, not after the scene has been freed

    // Tell the pager's read threads to stop taking work. numActiveRequests
    // counts queued requests rather than reads in flight, so it does not fall
    // to zero here - while the camera moves the queue refills as fast as it
    // drains, and after cancelling it simply stays where it was. It is not a
    // signal to wait on.
    for (const auto& task : viewer_->recordAndSubmitTasks)
        if (task && task->databasePager && task->databasePager->status) task->databasePager->status->set(false);

    // A short grace instead. A tile is written into the on-disk cache as it
    // arrives, and leaving mid-write would leave a truncated file that every
    // later run fails to decode. That write is one pass over a buffer that has
    // already been downloaded - microseconds - so a brief pause covers it.
    // What can take seconds is a read still waiting on the network, and a
    // thread blocked there has not started writing anything, so abandoning it
    // costs nothing.
    std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.0, graceSeconds)));
}

bool Viewer::create(const ViewerSettings& settings) {
    settings_ = settings;

    platform::enableHighDpiAwareness(); // client rect in physical pixels == swapchain extent

    options_ = vsg::Options::create();
    options_->add(vsgXchange::all::create()); // glTF/KTX/PNG/JPEG readers + http tile fetching
    const std::filesystem::path tileCache =
        settings_.tileCacheDir.empty() ? platform::mapsDir() : settings_.tileCacheDir;
    options_->fileCache = vsg::Path(tileCache.string());
    options_->sharedObjects = vsg::SharedObjects::create();

    auto traits = vsg::WindowTraits::create();
    traits->windowTitle = settings.title;
    traits->width = settings.width;
    traits->height = settings.height;
    traits->fullscreen = settings.fullscreen;
    traits->debugLayer = settings.debugLayer;
    traits->samples = settings.samples;
    // Depth precision over the 2 m .. 200 km range of a full-Earth scene (design 12.3).
    traits->depthFormat = VK_FORMAT_D32_SFLOAT;
    // The depth resolved to one sample as the frame's pass ends (Vulkan 1.2's depth resolve), for the pass over the
    // scene (setScene's overScene) to test against
    traits->vulkanVersion = VK_API_VERSION_1_2;
    traits->depthImageUsage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    // and the swapchain's images copied from (the scene's colour for that pass; screenshots)
    traits->swapchainPreferences.imageUsage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    // Vsync (FIFO) paces the loop for free; a frame cap needs a non-blocking present.
    if (settings_.maxFps > 0.0) traits->swapchainPreferences.presentMode = VK_PRESENT_MODE_MAILBOX_KHR;

    viewer_ = vsg::Viewer::create();
    window_ = vsg::Window::create(traits);
    if (!window_) {
        LOG_ERROR("render") << "could not create a Vulkan window (is a Vulkan 1.1 driver installed?)";
        return false;
    }
    viewer_->addWindow(window_);

    const auto& physical = window_->getOrCreatePhysicalDevice();
    // a draw's own shading rate, coarser than its pixels where what it draws is soft (the contrails' march at large
    // window sizes): asked for before the device is made (on first use), when the device has it
    if (physical->supportsDeviceExtension(VK_KHR_FRAGMENT_SHADING_RATE_EXTENSION_NAME)) {
        const auto rate = physical->getFeatures<VkPhysicalDeviceFragmentShadingRateFeaturesKHR,
                                                VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FRAGMENT_SHADING_RATE_FEATURES_KHR>();
        if (rate.pipelineFragmentShadingRate) {
            traits->deviceExtensionNames.push_back(VK_KHR_FRAGMENT_SHADING_RATE_EXTENSION_NAME);
            if (!traits->deviceFeatures) traits->deviceFeatures = vsg::DeviceFeatures::create();
            traits->deviceFeatures->get<VkPhysicalDeviceFragmentShadingRateFeaturesKHR,
                                        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FRAGMENT_SHADING_RATE_FEATURES_KHR>()
                .pipelineFragmentShadingRate = VK_TRUE;
            coarseShading_ = true;
        }
    }
    LOG_INFO("render") << "Vulkan device: " << physical->getProperties().deviceName << ", " << settings.width << "x"
                       << settings.height << (settings.debugLayer ? ", validation on" : "");
    return true;
}

namespace {

// The scene's colour copied out of the swapchain image the frame draws into, between the frame's pass and the pass
// over it: the image left as a colour attachment for that pass, the copy as a texture. Not wanted, the image is only
// handed on to that pass and the copy keeps what it had (made once, so it is always a texture to bind).
class CopySceneColour : public vsg::Inherit<vsg::Command, CopySceneColour> {
public:
    CopySceneColour(vsg::ref_ptr<vsg::Window> w, vsg::ref_ptr<vsg::Image> t, const bool* w2) :
        window(w), target(std::move(t)), wanted(w2) {}
    vsg::observer_ptr<vsg::Window> window;
    vsg::ref_ptr<vsg::Image> target;
    const bool* wanted;
    mutable bool made = false;

    void compile(vsg::Context& context) override { target->compile(context); }
    void record(vsg::CommandBuffer& commandBuffer) const override {
        auto w = window.ref_ptr();
        if (!w) return;
        const std::size_t index = w->imageIndex();
        if (index >= w->numFrames()) return;
        const VkImage source = w->imageView(index)->image->vk(commandBuffer.deviceID);
        const VkImage copy = target->vk(commandBuffer.deviceID);
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        const auto barrier = [&](VkImage image, VkAccessFlags from, VkAccessFlags to, VkImageLayout a, VkImageLayout b) {
            VkImageMemoryBarrier m{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER, nullptr, from, to, a, b, VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED,
                                   image, range};
            return m;
        };
        if (made && !*wanted) {
            const VkImageMemoryBarrier handOn = barrier(source, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                                                        VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                                                        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
            vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0, 0,
                                 nullptr, 0, nullptr, 1, &handOn);
            return;
        }
        made = true;
        const VkImageMemoryBarrier before[2] = {
            barrier(source, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL),
            barrier(copy, VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)};
        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                             VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 2, before);
        const VkExtent2D extent = w->extent2D();
        VkImageCopy region{};
        region.srcSubresource = VkImageSubresourceLayers{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        region.dstSubresource = region.srcSubresource;
        region.extent = VkExtent3D{std::min(extent.width, target->extent.width), std::min(extent.height, target->extent.height), 1};
        vkCmdCopyImage(commandBuffer, source, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, copy, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
        const VkImageMemoryBarrier after[2] = {
            barrier(source, VK_ACCESS_TRANSFER_READ_BIT, VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL),
            barrier(copy, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)};
        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 2, after);
    }
};

} // namespace

vsg::ref_ptr<vsg::ImageInfo> Viewer::sceneColour() {
    if (sceneColour_ || !window_) return sceneColour_;
    VkExtent2D extent = window_->extent2D();
    unsigned sw = 0, sh = 0;
    platform::screenSize(sw, sh);
    sceneCopy_ = vsg::Image::create();
    sceneCopy_->imageType = VK_IMAGE_TYPE_2D;
    sceneCopy_->format = window_->surfaceFormat().format;
    sceneCopy_->extent = VkExtent3D{std::max(extent.width, sw), std::max(extent.height, sh), 1};
    sceneCopy_->mipLevels = 1;
    sceneCopy_->arrayLayers = 1;
    sceneCopy_->samples = VK_SAMPLE_COUNT_1_BIT;
    sceneCopy_->tiling = VK_IMAGE_TILING_OPTIMAL;
    sceneCopy_->usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    auto sampler = vsg::Sampler::create();
    sampler->minFilter = sampler->magFilter = VK_FILTER_LINEAR;
    sampler->addressModeU = sampler->addressModeV = sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sceneColour_ = vsg::ImageInfo::create(sampler, vsg::ImageView::create(sceneCopy_, VK_IMAGE_ASPECT_COLOR_BIT), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    return sceneColour_;
}

vsg::ref_ptr<vsg::RenderPass> Viewer::overPass() {
    if (overPass_ || !window_) return overPass_;
    // the swapchain image and the resolved depth as the frame's pass left them, nothing cleared (the depth stored
    // too, though nothing reads it after: NVIDIA's driver lost the device on a frame that loaded it, drew nothing in
    // the pass and let it go - a frame with no gas and no panels)
    auto device = window_->getOrCreateDevice();
    const vsg::AttachmentDescription colour{0, window_->surfaceFormat().format, VK_SAMPLE_COUNT_1_BIT, VK_ATTACHMENT_LOAD_OP_LOAD,
                                            VK_ATTACHMENT_STORE_OP_STORE, VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_DONT_CARE,
                                            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR};
    const vsg::AttachmentDescription depth{0, window_->depthFormat(), VK_SAMPLE_COUNT_1_BIT, VK_ATTACHMENT_LOAD_OP_LOAD,
                                           VK_ATTACHMENT_STORE_OP_STORE, VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_DONT_CARE,
                                           VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    vsg::SubpassDescription subpass;
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachments.push_back(vsg::AttachmentReference{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL});
    subpass.depthStencilAttachments.push_back(vsg::AttachmentReference{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL});
    vsg::SubpassDependency in{};
    in.srcSubpass = VK_SUBPASS_EXTERNAL;
    in.dstSubpass = 0;
    in.srcStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    in.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    in.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    in.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                       VK_ACCESS_SHADER_READ_BIT;
    overPass_ = vsg::RenderPass::create(device, vsg::RenderPass::Attachments{colour, depth}, vsg::RenderPass::Subpasses{subpass},
                                        vsg::RenderPass::Dependencies{in});
    return overPass_;
}

vsg::ref_ptr<vsgImGui::RenderImGui> Viewer::createImGui() {
    if (!window_) return {};
    // as vsgImGui's own window constructor sets it up, for the pass over the scene (its pipeline is made for the
    // render pass it is given: the frame's is multisampled, this one not)
    auto device = window_->getOrCreateDevice();
    auto physical = device->getPhysicalDevice();
    uint32_t queueFamily = 0;
    std::tie(queueFamily, std::ignore) = physical->getQueueFamily(window_->traits()->queueFlags, window_->getSurface());
    VkSurfaceCapabilitiesKHR capabilities;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(*physical, *(window_->getSurface()), &capabilities);
    uint32_t imageCount = std::max(3u, capabilities.minImageCount);
    if (capabilities.maxImageCount > 0) imageCount = std::min(imageCount, capabilities.maxImageCount);
    return vsgImGui::RenderImGui::create(device, queueFamily, overPass(), capabilities.minImageCount, imageCount, window_->extent2D());
}

void Viewer::selectOverFramebuffer() {
    if (!overGraph_ || !window_) return;
    const std::size_t frames = window_->numFrames();
    // (held, not just their addresses: a swapchain made anew could put its views where the old ones were)
    std::vector<vsg::ref_ptr<vsg::ImageView>> views;
    for (std::size_t i = 0; i < frames; ++i) views.push_back(window_->imageView(i));
    auto depth = window_->getDepthImageView();
    views.push_back(depth);
    const VkExtent2D extent2D = window_->extent2D();
    if (views != overViews_ || extent2D.width != overExtent_.width || extent2D.height != overExtent_.height) {
        // (made anew with the swapchain: at the start and when the window is resized)
        overExtent_ = extent2D;
        overFramebuffers_.clear();
        const VkExtent2D extent = window_->extent2D();
        for (std::size_t i = 0; i < frames; ++i)
            overFramebuffers_.push_back(vsg::Framebuffer::create(overPass(), vsg::ImageViews{window_->imageView(i), depth}, extent.width, extent.height, 1));
        overViews_ = views;
    }
    const std::size_t index = window_->imageIndex();
    if (index < overFramebuffers_.size()) overGraph_->framebuffer = overFramebuffers_[index];
    overGraph_->renderArea = VkRect2D{VkOffset2D{0, 0}, window_->extent2D()};
}

bool Viewer::setScene(vsg::ref_ptr<vsg::Node> scene, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid,
                      vsg::ref_ptr<vsg::Node> imguiOverlay, vsg::ref_ptr<vsg::Node> beforePass, vsg::ref_ptr<vsg::Node> overScene) {
    if (!viewer_ || !window_) return false;

    root_ = vsg::Group::create();
    root_->addChild(scene);

    // Initial camera: a few hundred km above the scene's centre, looking down.
    lookAt_ = vsg::LookAt::create(vsg::dvec3(0.0, -3.0e7, 0.0), vsg::dvec3(0.0, 0.0, 0.0), vsg::dvec3(0.0, 0.0, 1.0));
    const double aspect = static_cast<double>(window_->extent2D().width) / static_cast<double>(window_->extent2D().height);
    vsg::ref_ptr<vsg::ProjectionMatrix> projection;
    if (ellipsoid) {
        // Far = distance to the horizon (+ mountains) from the eye's height above
        // the ellipsoid; near = far * ratio. VSG renders reverse depth into a
        // 32-bit float buffer, where the depth's precision is relative to the
        // distance whatever the ratio, so the ratio can be tiny: at 1e-5 the
        // near plane sat 3-4 km in front of a 10 km-high eye and clipped the
        // nearby terrain into flat grey slabs, and 1e-6 (0.36 m near the
        // ground) cut into a 9 cm quadrotor followed from 11 cm. 1e-8 keeps
        // it at millimetres near the ground and under 1 cm at 10 km.
        projection = vsg::EllipsoidPerspective::create(lookAt_, ellipsoid, settings_.fieldOfViewDeg, aspect, 1.0e-8, 9000.0);
    } else {
        projection = vsg::Perspective::create(settings_.fieldOfViewDeg, aspect, 1.0, 1.0e6);
    }
    camera_ = vsg::Camera::create(projection, lookAt_, vsg::ViewportState::create(window_->extent2D()));

    auto commandGraph = vsg::CommandGraph::create(window_);
    auto renderGraph = vsg::RenderGraph::create(window_);
    if (prepassScene_ && prepassDepth_) commandGraph->addChild(depthPrepass(projection));
    if (beforePass) commandGraph->addChild(beforePass); // (compute work the scene draws from)
    commandGraph->addChild(renderGraph);

    auto view = vsg::View::create(camera_, root_);
    if (settings_.headlight) view->addChild(vsg::createHeadlight());
    renderGraph->addChild(view);
    if (overScene) {
        // the scene's colour copied, then the pass over it
        sceneColour();
        commandGraph->addChild(CopySceneColour::create(window_, sceneCopy_, &sceneCopied_));
        overGraph_ = vsg::RenderGraph::create();
        overGraph_->renderPass = overPass();
        window_->getOrCreateSwapchain(); // (its images now: the compile below makes the pass's pipelines for its framebuffers)
        selectOverFramebuffer();
        overGraph_->addChild(vsg::View::create(camera_, overScene));
        if (imguiOverlay) overGraph_->addChild(imguiOverlay); // drawn last
        commandGraph->addChild(overGraph_);
    } else if (imguiOverlay) {
        renderGraph->addChild(imguiOverlay); // same render pass, drawn last
    }

    viewer_->assignRecordAndSubmitTaskAndPresentation({commandGraph});
    viewer_->addEventHandler(vsg::CloseHandler::create(viewer_));

    const auto result = viewer_->compile();
    if (!result) {
        LOG_ERROR("render") << "vsg::Viewer::compile failed";
        return false;
    }
    lastFrame_ = vsg::clock::now();
    return true;
}

void Viewer::setDepthPrepass(vsg::ref_ptr<vsg::Node> subgraph) {
    prepassScene_ = std::move(subgraph);
    if (!prepassScene_ || !window_ || prepassDepth_) return;
    // its own framebuffer's images (a later resize leaves them as they are: read where the window's pixels fall, it is
    // only coarser or finer), a colour attachment the pipelines draw into and nothing reads, and the depth kept for the
    // effects to read - as big as the screen, not the window as it starts (maximised later, it would be read coarser
    // than drawn)
    auto device = window_->getOrCreateDevice();
    VkExtent2D extent = window_->extent2D();
    unsigned sw = 0, sh = 0;
    platform::screenSize(sw, sh);
    extent.width = std::max(extent.width, sw);
    extent.height = std::max(extent.height, sh);
    const auto image = [&](VkFormat format, VkImageUsageFlags usage, VkImageAspectFlags aspect) {
        auto i = vsg::Image::create();
        i->imageType = VK_IMAGE_TYPE_2D;
        i->format = format;
        i->extent = VkExtent3D{extent.width, extent.height, 1};
        i->mipLevels = 1;
        i->arrayLayers = 1;
        i->samples = VK_SAMPLE_COUNT_1_BIT;
        i->tiling = VK_IMAGE_TILING_OPTIMAL;
        i->usage = usage;
        return vsg::createImageView(device, i, aspect);
    };
    prepassColourView_ = image(VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
    prepassDepthView_ = image(VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_IMAGE_ASPECT_DEPTH_BIT);
    prepassExtent_ = extent;
    auto sampler = vsg::Sampler::create();
    sampler->minFilter = sampler->magFilter = VK_FILTER_NEAREST;
    sampler->addressModeU = sampler->addressModeV = sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler->maxLod = 0.0f;
    prepassDepth_ = vsg::ImageInfo::create(sampler, prepassDepthView_, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL);
}

vsg::ref_ptr<vsg::Node> Viewer::depthPrepass(vsg::ref_ptr<vsg::ProjectionMatrix> projection) {
    // the pass drawing into the images setDepthPrepass made
    auto device = window_->getOrCreateDevice();
    const VkExtent2D extent = prepassExtent_;
    auto colour = prepassColourView_, depth = prepassDepthView_;

    const vsg::AttachmentDescription colourAttachment{0, VK_FORMAT_R8G8B8A8_UNORM, VK_SAMPLE_COUNT_1_BIT, VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                                                      VK_ATTACHMENT_STORE_OP_DONT_CARE, VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                                                      VK_ATTACHMENT_STORE_OP_DONT_CARE, VK_IMAGE_LAYOUT_UNDEFINED,
                                                      VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    const vsg::AttachmentDescription depthAttachment{0, VK_FORMAT_D32_SFLOAT, VK_SAMPLE_COUNT_1_BIT, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                     VK_ATTACHMENT_STORE_OP_STORE, VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                                                     VK_ATTACHMENT_STORE_OP_DONT_CARE, VK_IMAGE_LAYOUT_UNDEFINED,
                                                     VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};
    vsg::SubpassDescription subpass;
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachments.push_back(vsg::AttachmentReference{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, 0});
    subpass.depthStencilAttachments.push_back(vsg::AttachmentReference{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL, 0});
    // the last frame's readers done before it is cleared; the depth written before this frame's read it
    const vsg::SubpassDependency in{VK_SUBPASS_EXTERNAL, 0, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                                    VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
                                    VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, 0, 0};
    const vsg::SubpassDependency out{0, VK_SUBPASS_EXTERNAL, VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                                     VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT, 0, 0};
    auto renderPass = vsg::RenderPass::create(device, vsg::RenderPass::Attachments{colourAttachment, depthAttachment},
                                              vsg::RenderPass::Subpasses{subpass}, vsg::RenderPass::Dependencies{in, out});
    auto graph = vsg::RenderGraph::create();
    graph->framebuffer = vsg::Framebuffer::create(renderPass, vsg::ImageViews{colour, depth}, extent.width, extent.height, 1);
    graph->renderArea = VkRect2D{{0, 0}, extent};
    VkClearValue clearColour{}, clearDepth{};
    clearDepth.depthStencil = {0.0f, 0}; // reverse depth: nothing nearer than the far plane
    graph->clearValues = {clearColour, clearDepth};
    auto view = vsg::View::create(vsg::Camera::create(projection, lookAt_, vsg::ViewportState::create(extent)), prepassScene_);
    view->mask = vsg::MASK_ALL & ~kNotInPrepass;
    graph->addChild(view);

    prepassSwitch_ = vsg::Switch::create();
    prepassSwitch_->addChild(true, graph); // (on while the viewer compiles: views switched off are not compiled for)
    return prepassSwitch_;
}

bool Viewer::compile(vsg::ref_ptr<vsg::Object> object) {
    if (!viewer_ || !viewer_->compileManager || !object) return false;
    auto result = viewer_->compileManager->compile(object);
    if (result) vsg::updateViewer(*viewer_, result);
    return static_cast<bool>(result);
}

void Viewer::addEventHandler(vsg::ref_ptr<vsg::Visitor> handler) {
    if (viewer_) viewer_->addEventHandler(handler);
}

bool Viewer::frame() {
    if (!viewer_) return false;
    using seconds = std::chrono::duration<double>;
    auto t0 = vsg::clock::now();
    timing_.app = frameEnd_.time_since_epoch().count() ? seconds(t0 - frameEnd_).count() : 0.0;

    if (!viewer_->advanceToNextFrame()) return false;
    selectOverFramebuffer(); // (the image this frame draws into, now known)
    auto t1 = vsg::clock::now();
    timing_.advance = seconds(t1 - t0).count();

    viewer_->handleEvents();
    auto t2 = vsg::clock::now();
    timing_.events = seconds(t2 - t1).count();
    viewer_->update();
    auto t3 = vsg::clock::now();
    timing_.update = seconds(t3 - t2).count();
    viewer_->recordAndSubmit();
    auto t4 = vsg::clock::now();
    timing_.record = seconds(t4 - t3).count();
    viewer_->present();
    auto t5 = vsg::clock::now();
    timing_.present = seconds(t5 - t4).count();

    // Optional frame cap (a bare sleep_for() would be rounded up to the 15.6 ms
    // scheduler period and turn "60 fps" into 40; see platform::sleepUntil).
    // Only meaningful with a non-blocking present mode: with FIFO the cap and
    // the vsync beat against each other.
    if (settings_.maxFps > 0.0) {
        const auto target = lastFrame_ + std::chrono::duration_cast<vsg::clock::duration>(seconds(1.0 / settings_.maxFps));
        platform::sleepUntil(target);
    }
    const auto t6 = vsg::clock::now();
    timing_.sleep = seconds(t6 - t5).count();
    frameEnd_ = t6;

    const auto now = t6;
    frameSeconds_ = std::chrono::duration<double>(now - lastFrame_).count();
    lastFrame_ = now;
    fpsAccumulator_ += frameSeconds_;
    if (++fpsFrames_ >= 30) {
        fps_ = fpsAccumulator_ > 0.0 ? fpsFrames_ / fpsAccumulator_ : 0.0;
        fpsAccumulator_ = 0.0;
        fpsFrames_ = 0;
    }
    return true;
}

} // namespace fsim::render
