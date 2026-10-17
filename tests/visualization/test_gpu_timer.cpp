// The GPU timer's report (render::GpuTimer, no Vulkan): what frames' timestamps give, and that the GPU's gap between
// two frames is measured only between frames read one after the other (HC-49). Three marks: the frame's start, then
// two sections, "one" and "two"; timestamps in ticks of 1 ms.
#include "visualization/render/GpuTimer.h"

#include <catch2/catch_test_macros.hpp>

#include <string>

namespace {

void marks(fsim::render::GpuTimer& timer) {
    timer.mark("start");
    timer.mark("one");
    timer.mark("two");
}

std::string meanLine(const std::string& report) { return report.substr(0, report.find('\n')); }

std::string worstLine(const std::string& report) {
    const auto at = report.find("\ngpu worst");
    return at == std::string::npos ? std::string() : report.substr(at + 1);
}

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

} // namespace

TEST_CASE("the GPU timer never measures a gap across a frame it did not read", "[render][gpu-timer]") {
    // (the review's probe: A read, B not read, C read. C's gap was given as 7 ms - B's own passes in it - for an
    // actual gap of 1 ms; it is not known)
    fsim::render::GpuTimer timer;
    marks(timer);
    timer.addFrame({100, 101, 105}, 1.0);
    timer.addMissing();
    timer.addFrame({112, 114, 118}, 1.0);
    const std::string report = timer.report();
    const std::string worst = worstLine(report);
    CHECK(has(worst, "| total 6.000 ms |")); // C's
    CHECK(has(worst, "| gap before it n/a |"));
    CHECK(has(worst, "| largest gap n/a |"));
    CHECK(has(worst, "| unread 1"));
    CHECK(has(meanLine(report), "(2 frames)"));
}

TEST_CASE("the GPU timer measures the gap between frames read one after the other", "[render][gpu-timer]") {
    // (the same timestamps, all three frames read: the actual gaps, 1 ms each)
    fsim::render::GpuTimer timer;
    marks(timer);
    timer.addFrame({100, 101, 105}, 1.0);
    timer.addFrame({106, 107, 111}, 1.0);
    timer.addFrame({112, 114, 118}, 1.0);
    const std::string report = timer.report();
    CHECK(meanLine(report) == "gpu | one 1.333 | two 4.000 | total 5.333 ms (3 frames)");
    const std::string worst = worstLine(report);
    CHECK(worst == "gpu worst | one 2.000 | two 4.000 | total 6.000 ms | gap before it 1.000 ms | largest gap 1.000 ms | unread 0");
}

TEST_CASE("the GPU timer keeps a gap across its report's boundary and starts the window afresh", "[render][gpu-timer]") {
    fsim::render::GpuTimer timer;
    marks(timer);
    timer.addFrame({100, 101, 105}, 1.0);
    const std::string first = timer.report();
    CHECK(has(worstLine(first), "| gap before it n/a | largest gap n/a | unread 0")); // (no frame before it)
    timer.addFrame({108, 109, 112}, 1.0);
    const std::string second = timer.report();
    CHECK(has(meanLine(second), "(1 frames)"));
    CHECK(has(worstLine(second), "| total 4.000 ms | gap before it 3.000 ms | largest gap 3.000 ms | unread 0"));
}

TEST_CASE("the GPU timer does not use a frame whose timestamps run backwards, nor measure the next gap",
          "[render][gpu-timer]") {
    fsim::render::GpuTimer timer;
    marks(timer);
    timer.addFrame({100, 101, 105}, 1.0);
    timer.addFrame({106, 104, 111}, 1.0); // (not forward: not used, counted unread)
    timer.addFrame({112, 113, 114}, 1.0);
    const std::string report = timer.report();
    CHECK(has(meanLine(report), "(2 frames)"));
    const std::string worst = worstLine(report);
    CHECK(has(worst, "| total 5.000 ms | gap before it n/a | largest gap n/a | unread 1"));
}

TEST_CASE("the GPU timer reports a window in which no frame was read", "[render][gpu-timer]") {
    fsim::render::GpuTimer timer;
    marks(timer);
    timer.addMissing();
    const std::string report = timer.report();
    CHECK(has(meanLine(report), "(0 frames)"));
    CHECK(has(worstLine(report), "| total 0.000 ms | gap before it n/a | largest gap n/a | unread 1"));
    // (a report with nothing read and nothing missed has no worst line)
    CHECK(worstLine(timer.report()).empty());
}
