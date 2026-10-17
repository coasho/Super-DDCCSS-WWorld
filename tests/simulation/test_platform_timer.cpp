// The platform's precise sleep (foundation/platform/Clock.h; ARCH-08 HC-32): its high-resolution timer is the sleeping
// thread's own and goes with the thread. A process that starts and ends threads that sleep - the demo publisher does
// at every hold - keeps a steady handle count, and the sleep still lasts until its deadline.

#include "foundation/platform/Clock.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <thread>

#ifdef _WIN32
#    ifndef NOMINMAX
#        define NOMINMAX
#    endif
#    include <windows.h>
#endif

using namespace fsim;

TEST_CASE("sleepUntil's timer goes with its thread: threads that slept leave no handle behind", "[platform]") {
    const auto threadThatSleeps = [] {
        std::thread t([] {
            platform::sleepUntil(platform::Clock::now() + std::chrono::milliseconds(1));
            platform::sleepUntil(platform::Clock::now() + std::chrono::milliseconds(1));
        });
        t.join();
    };
    for (int i = 0; i < 4; ++i) threadThatSleeps(); // (whatever the first threads open for good is not counted)
#ifdef _WIN32
    const auto handles = [] {
        DWORD count = 0;
        GetProcessHandleCount(GetCurrentProcess(), &count);
        return static_cast<long>(count);
    };
    const long before = handles();
#endif
    for (int i = 0; i < 30; ++i) threadThatSleeps();
#ifdef _WIN32
    const long after = handles();
    INFO("process handles: " << before << " before 30 threads that slept, " << after << " after");
    REQUIRE(after == before);
#endif
    const auto deadline = platform::Clock::now() + std::chrono::milliseconds(5);
    platform::sleepUntil(deadline);
    REQUIRE(platform::Clock::now() >= deadline);
}
