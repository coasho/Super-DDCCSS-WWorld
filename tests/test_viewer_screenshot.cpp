// A screenshot taken after frame() must contain the frame just presented,
// not the previous swapchain image. This uses a real fullscreen Vulkan window.
#include "render/Viewer.h"

#include <vsgXchange/all.h>

#include <array>
#include <cstdio>
#include <filesystem>
#include <string>

#define CHECK(cond)                                                                    \
    do {                                                                               \
        if (!(cond)) {                                                                 \
            std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #cond); \
            return 1;                                                                  \
        }                                                                              \
    } while (0)

int main(int argc, char** argv) {
    CHECK(argc == 2); // CTest supplies a directory in the build tree.
    const std::filesystem::path output = argv[1];
    std::filesystem::create_directories(output);
    fsim::render::ViewerSettings settings;
    settings.title = "flightsim screenshot regression";
    settings.width = 3840;
    settings.height = 2160;
    settings.fullscreen = true; // uses the monitor's native physical extent
    settings.headlight = false;
    settings.tileCacheDir = output;
    fsim::render::Viewer viewer;
    try {
        if (!viewer.create(settings)) {
            std::fprintf(stderr, "skipping: no Vulkan fullscreen window\n");
            return 77;
        }
    } catch (const vsg::Exception& e) {
        std::fprintf(stderr, "skipping: %s (%d)\n", e.message.c_str(), e.result);
        return 77;
    }

    VkClearAttachment attachment{};
    attachment.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    attachment.colorAttachment = 0;
    attachment.clearValue.color = VkClearColorValue{{1.0f, 0.0f, 0.0f, 1.0f}};
    VkClearRect rectangle{};
    rectangle.rect.extent = viewer.window()->extent2D();
    rectangle.layerCount = 1;
    auto clear = vsg::ClearAttachments::create(vsg::ClearAttachments::Attachments{attachment},
                                              vsg::ClearAttachments::Rects{rectangle});
    CHECK(viewer.setScene(clear, {}));
    // Fill the swapchain history with red before asking for blue: an old image is then deterministically wrong.
    for (unsigned warmup = 0; warmup < 3; ++warmup) {
        clear->rects[0].rect.extent = viewer.window()->extent2D();
        CHECK(viewer.frame());
    }
    const auto reader = vsg::Options::create(vsgXchange::all::create());
    const std::array<std::array<unsigned char, 3>, 2> expected{{{0, 0, 255}, {0, 255, 0}}};
    for (unsigned frame = 0; frame < expected.size(); ++frame) {
        for (unsigned channel = 0; channel < 3; ++channel)
            clear->attachments[0].clearValue.color.float32[channel] = expected[frame][channel] / 255.0f;
        clear->rects[0].rect.extent = viewer.window()->extent2D();
        CHECK(viewer.frame());
        const VkExtent2D extent = viewer.window()->extent2D();
        const auto path = output / ("frame-" + std::to_string(frame) + ".png");
        CHECK(viewer.screenshot(path.string()));
        const auto image = vsg::read_cast<vsg::ubvec4Array2D>(vsg::Path(path.string()), reader);
        CHECK(image);
        CHECK(image->width() == extent.width && image->height() == extent.height);
        const auto pixel = (*image)(extent.width / 2, extent.height / 2);
        std::fprintf(stderr, "frame %u, %ux%u: screenshot RGB %u,%u,%u; expected %u,%u,%u\n", frame,
                     image->width(), image->height(), static_cast<unsigned>(pixel.r), static_cast<unsigned>(pixel.g),
                     static_cast<unsigned>(pixel.b), static_cast<unsigned>(expected[frame][0]),
                     static_cast<unsigned>(expected[frame][1]), static_cast<unsigned>(expected[frame][2]));
        for (unsigned channel = 0; channel < 3; ++channel)
            CHECK(pixel[channel] == expected[frame][channel]);
    }
    viewer.close();
    std::printf("fullscreen screenshots contain the latest presented frame\n");
    return 0;
}
