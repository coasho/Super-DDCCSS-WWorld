#include "parallel.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <thread>
#include <vector>

namespace {

std::size_t cases = 0;
std::uint64_t checks = 0;
std::size_t failures = 0;

std::uint64_t sample(std::size_t index) {
    const double x = static_cast<double>(index);
    const double value = (x + 0.25) * (x - 0.5);
    static_assert(sizeof(value) == sizeof(std::uint64_t));
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

void expect(bool condition, const char* message, std::size_t n, unsigned requested, unsigned hardware) {
    ++checks;
    if (!condition) {
        ++failures;
        std::cerr << "n=" << n << " requested=" << requested << " hardware=" << hardware << ": " << message << '\n';
    }
}

void exercise(std::size_t n, unsigned requested, unsigned hardware) {
    ++cases;
    std::vector<std::atomic<unsigned>> visits(n);
    std::vector<std::atomic<std::uint64_t>> output(n);
    for (std::size_t i = 0; i < n; ++i) {
        visits[i].store(0);
        output[i].store(0);
    }
    unsigned launched = 0;
    std::atomic<unsigned> started{0}, finished{0}, active{0};
    auto launch = [&](auto work) {
        ++launched;
        return std::thread([&, work]() {
            ++started;
            ++active;
            work();
            --active;
            ++finished;
        });
    };
    meshkit::detail::parallelWithLauncher(n, requested, hardware, [&](std::size_t i) {
        visits[i].fetch_add(1, std::memory_order_relaxed);
        output[i].store(sample(i), std::memory_order_relaxed);
    }, launch);

    // Independent expected dispatch: the actual production launcher loop must
    // create no worker that cannot own one of the unchanged 4096-point chunks.
    const std::size_t available = std::min(32u, requested ? requested : std::max(1u, hardware));
    const std::size_t chunks = n == 0 ? 0 : 1 + (n - 1) / 4096;
    const std::size_t workers = std::min(available, chunks);
    const std::size_t expectedLaunches = workers == 0 ? 0 : workers - 1;
    expect(launched == expectedLaunches, "created excess or insufficient real threads", n, requested, hardware);
    expect(started.load() == launched && finished.load() == launched && active.load() == 0,
           "actual worker threads were not all joined", n, requested, hardware);
    for (std::size_t i = 0; i < n; ++i) {
        expect(visits[i].load() == 1, "index was not evaluated exactly once", n, requested, hardware);
        expect(output[i].load() == sample(i), "per-index output differs from serial bytes", n, requested, hardware);
    }
}

} // namespace

int main() {
    try {
        for (const std::size_t n : std::array<std::size_t, 6>{0, 1, 4095, 4096, 4097, 8193})
            for (const unsigned requested : std::array<unsigned, 5>{0, 1, 2, 32, 64})
                exercise(n, requested, 16);
        exercise(0, 0, 0);
        exercise(4097, 0, 0);
        exercise(4096 * 40 + 1, 64, 16); // Retain the existing caller limit of 32.
    } catch (const std::exception& error) {
        std::cerr << "parallel regression exception: " << error.what() << '\n';
        return 2;
    }
    std::cout << "parallel regression: cases=" << cases << " checks=" << checks << " failures=" << failures << '\n';
    return cases != 0 && checks != 0 && failures == 0 ? 0 : 1;
}
