#pragma once

#include <vsg/all.h>

#include <cstdint>
#include <filesystem>
#include <string>

namespace vsgImGui {
class RenderImGui;
}

namespace fsim::render {

struct ViewerSettings {
    std::string title = "flightsim";
    std::uint32_t width = 1600;
    std::uint32_t height = 900;
    bool fullscreen = true; // On-screen views always use the primary display fullscreen.
    bool debugLayer = false;   ///< VK_LAYER_KHRONOS_validation
    VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_4_BIT;
    double fieldOfViewDeg = 30.0;
    bool headlight = true;     ///< off when the scene brings its own lights (sun)
    double maxFps = 0.0;       ///< optional frame-rate cap (0 = none: vsync paces the loop, which already costs ~3% of a core)
    /// Where the map tiles are. Empty = the local map set beside the program
    /// (platform::mapsDir()).
    std::filesystem::path tileCacheDir;
};

/// One window, one vsg::Viewer, one camera, one command graph: scene first,
/// ImGui overlay last in the same render pass (design 8, ADR-4). Owns nothing
/// about what is drawn; `world` supplies the scene, `ui` the overlay.
class Viewer {
public:
    Viewer() = default;

    /// Create the window and Vulkan device. Returns false (and logs) on failure.
    bool create(const ViewerSettings& settings);

    /// Build the command graph for `scene` (+ optional ImGui overlay node), add
    /// event handlers and compile everything. Call once after create().
    /// `beforePass` is recorded each frame before the render pass: compute work
    /// whose results the scene reads (it brings its own barriers).
    /// `overScene` is drawn in a second pass over the frame once the scene is
    /// drawn, with the scene's camera and its depth (resolved to one sample),
    /// the scene's colour copied into sceneColour() between them: what draws
    /// the scene behind it bent - the hot gas round the aircraft. The ImGui
    /// overlay is drawn there too, last.
    bool setScene(vsg::ref_ptr<vsg::Node> scene, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid,
                  vsg::ref_ptr<vsg::Node> imguiOverlay = {},
                  vsg::ref_ptr<vsg::Node> beforePass = {},
                  vsg::ref_ptr<vsg::Node> overScene = {},
                  vsg::ref_ptr<vsg::Node> betweenPass = {});
    /// The frame's colour as the scene left it, for `overScene` to read: the
    /// screen's size (the window's part used, from the top left), the
    /// swapchain's format, linear filtering; made on first call.
    vsg::ref_ptr<vsg::ImageInfo> sceneColour();
    /// Whether this frame copies the scene's colour (on by default): off, the
    /// copy keeps the last one made - for frames where nothing reads it.
    void setSceneCopied(bool on) { sceneCopied_ = on; }
    /// The resolved full-resolution scene depth (D32_SFLOAT, reverse Z) copied after the main render pass for
    /// `betweenPass` volumetric raymarching and `overScene` depth-aware bilateral upsampling; made on first call.
    vsg::ref_ptr<vsg::ImageInfo> sceneDepth();
    /// Whether this frame copies the resolved scene depth (off by default: 0 ms when no offscreen volumetric pass runs).
    void setSceneDepthCopied(bool on) { sceneDepthCopied_ = on; }
    /// Whether draws may be shaded at a coarser rate than their pixels (VK_KHR_fragment_shading_rate, the
    /// pipeline's own rate: what the contrails use at large window sizes); known once open() has made the window.
    bool coarseShading() const { return coarseShading_; }
    /// The render pass over the scene (one sample, nothing cleared): what is drawn there is made for it.
    vsg::ref_ptr<vsg::RenderPass> overPass();
    /// An ImGui overlay made for that pass (setScene's imguiOverlay when there is an overScene).
    vsg::ref_ptr<vsgImGui::RenderImGui> createImGui();

    /// Also draw `subgraph` (the vehicles) with the scene's camera before the
    /// frame's render pass, into a depth image of its own at the window's size
    /// - what an effect marching rays through the air round them reads to stop
    /// where they are solid. Call before setScene(): its depth image is made
    /// here (depthPrepass().first), its pass with the scene; the pass is drawn
    /// only while depthPrepass().second is on.
    void setDepthPrepass(vsg::ref_ptr<vsg::Node> subgraph);
    /// The traversal mask of what the pass leaves out (gas, not solid: the
    /// exhausts' ray-marched flames, which cost as much again drawn there):
    /// a child shown with only this bit is drawn by every other view.
    static constexpr vsg::Mask kNotInPrepass = vsg::Mask(1) << 62;
    /// Nodes masked with this bit are excluded from the offscreen PiP inset
    /// camera (e.g. main-viewport screen-space cloud/flow passes).
    static constexpr vsg::Mask kNotInPip = vsg::Mask(1) << 61;
    /// That depth (reverse depth, 0 = nothing; in DEPTH_STENCIL_READ_ONLY
    /// layout) and the switch drawing it (on to begin with); null before
    /// setScene() or without setDepthPrepass().
    std::pair<vsg::ref_ptr<vsg::ImageInfo>, vsg::ref_ptr<vsg::Switch>> depthPrepass() const { return {prepassDepth_, prepassSwitch_}; }

    /// Enable an optional offscreen Vulkan Picture-in-Picture (PiP) render
    /// target (`width x height` RGBA8 + D32_SFLOAT) compiled alongside the
    /// main scene and gated by a vsg::Switch (0.0 ms GPU cost when disabled).
    /// Call before setScene().
    void enablePip(std::uint32_t width = 480, std::uint32_t height = 270);
    vsg::ref_ptr<vsg::Camera> pipCamera() const { return pipCamera_; }
    vsg::ref_ptr<vsg::LookAt> pipLookAt() const { return pipLookAt_; }
    vsg::ref_ptr<vsg::Switch> pipSwitch() const { return pipSwitch_; }
    vsg::ref_ptr<vsg::ImageInfo> pipImageInfo() const { return pipImageInfo_; }
    vsg::ref_ptr<vsg::DescriptorSet> pipDescriptorSet() const { return pipDescriptorSet_; }
    VkExtent2D pipExtent() const noexcept { return pipExtent_; }
    void setPipEnabled(bool on);

    /// Compile a subgraph added to the scene after setScene() (a vehicle model
    /// loaded when a new type appears). Returns false when nothing was compiled.
    bool compile(vsg::ref_ptr<vsg::Object> object);

    /// Save the frame presented last as PNG (blocks until the GPU is idle).
    bool screenshot(const std::string& path);

    /// Add an event handler (key handler, trackball, ImGui event forwarder).
    /// ImGui's forwarder must be added before any other handler so it can
    /// consume events first (design 9.6).
    void addEventHandler(vsg::ref_ptr<vsg::Visitor> handler);

    /// Run one frame: events, update, record, submit, present. Returns false
    /// once the window has been closed.
    bool frame();

    bool active() const { return viewer_ && viewer_->active(); }
    void close() { if (viewer_) viewer_->close(); }

    /// Close the window, stop the tile pager taking new work, and pause long
    /// enough for a cache write already under way to finish. Call before
    /// leaving without unwinding the scene graph.
    void quiesce(double graceSeconds = 0.15);

    vsg::ref_ptr<vsg::Viewer> viewer() const { return viewer_; }
    vsg::ref_ptr<vsg::Window> window() const { return window_; }
    vsg::ref_ptr<vsg::Camera> camera() const { return camera_; }
    vsg::ref_ptr<vsg::LookAt> lookAt() const { return lookAt_; }
    vsg::ref_ptr<vsg::Options> options() const { return options_; }

    /// Last frame's wall-clock duration and rolling frames-per-second.
    double frameSeconds() const { return frameSeconds_; }
    double fps() const { return fps_; }

    /// Where the last frame's time went (seconds), for --stats and the debug panel.
    struct FrameTiming {
        double app = 0.0;      ///< caller's work between frames (poll, interpolate, labels)
        double advance = 0.0;  ///< advanceToNextFrame: waits for the previous frame's fences / swapchain image
        double events = 0.0;
        double update = 0.0;   ///< scene update (pager merges, animations)
        double record = 0.0;   ///< command recording + submit
        double present = 0.0;
        double sleep = 0.0;    ///< frame cap
    };
    const FrameTiming& timing() const { return timing_; }

private:
    vsg::ref_ptr<vsg::Node> depthPrepass(vsg::ref_ptr<vsg::ProjectionMatrix> projection);
    vsg::ref_ptr<vsg::Node> buildPipPass(vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid);
    void selectOverFramebuffer();

    ViewerSettings settings_;
    vsg::ref_ptr<vsg::Options> options_;
    vsg::ref_ptr<vsg::Viewer> viewer_;
    vsg::ref_ptr<vsg::Window> window_;
    vsg::ref_ptr<vsg::Camera> camera_;
    vsg::ref_ptr<vsg::LookAt> lookAt_;
    vsg::ref_ptr<vsg::Group> root_;
    vsg::ref_ptr<vsg::Node> prepassScene_;
    vsg::ref_ptr<vsg::ImageInfo> prepassDepth_;
    vsg::ref_ptr<vsg::ImageView> prepassColourView_, prepassDepthView_;
    VkExtent2D prepassExtent_{};
    vsg::ref_ptr<vsg::Switch> prepassSwitch_;
    // Offscreen Vulkan Picture-in-Picture (PiP) inset pass (VIS-M14, 0.0 ms when pipSwitch_ is OFF).
    bool pipRequested_ = false;
    VkExtent2D pipExtent_{480, 270};
    vsg::ref_ptr<vsg::ImageView> pipColourView_, pipDepthView_;
    vsg::ref_ptr<vsg::ImageInfo> pipImageInfo_;
    vsg::ref_ptr<vsg::DescriptorSet> pipDescriptorSet_;
    vsg::ref_ptr<vsg::Camera> pipCamera_;
    vsg::ref_ptr<vsg::LookAt> pipLookAt_;
    vsg::ref_ptr<vsg::Switch> pipSwitch_;
    // the pass over the scene: its render pass (the swapchain image and the resolved depth, both loaded), a
    // framebuffer per swapchain image (remade when the swapchain is), the copy of the scene's colour
    vsg::ref_ptr<vsg::Image> sceneCopy_;
    vsg::ref_ptr<vsg::ImageInfo> sceneColour_;
    bool sceneCopied_ = true;
    vsg::ref_ptr<vsg::Image> sceneDepthCopy_;
    vsg::ref_ptr<vsg::ImageInfo> sceneDepth_;
    bool sceneDepthCopied_ = false;
    bool coarseShading_ = false;
    vsg::ref_ptr<vsg::RenderPass> overPass_;
    vsg::ref_ptr<vsg::RenderGraph> overGraph_;
    std::vector<vsg::ref_ptr<vsg::Framebuffer>> overFramebuffers_;
    std::vector<vsg::ref_ptr<vsg::ImageView>> overViews_; ///< what they were made for: the swapchain's views, then the depth's
    VkExtent2D overExtent_{};

    double frameSeconds_ = 0.0;
    FrameTiming timing_;
    vsg::clock::time_point frameEnd_{};
    double fps_ = 0.0;
    double fpsAccumulator_ = 0.0;
    int fpsFrames_ = 0;
    vsg::clock::time_point lastFrame_{};
};

} // namespace fsim::render
