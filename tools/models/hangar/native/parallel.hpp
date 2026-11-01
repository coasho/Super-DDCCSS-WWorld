#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <thread>
#include <utility>
#include <vector>

namespace meshkit::detail {

inline constexpr std::size_t kParallelChunk = 4096;

struct ThreadLauncher {
    template <typename Work>
    std::thread operator()(Work work) const {
        return std::thread(std::move(work));
    }
};

/// Internal launcher seam lets the standalone regression count actual threads.
template <typename F, typename Launch>
void parallelWithLauncher(std::size_t n, unsigned threads, unsigned hardwareThreads, F fn, Launch launch) {
    if (n == 0) return;
    threads = std::min(32u, threads ? threads : std::max(1u, hardwareThreads));
    threads = static_cast<unsigned>(std::min<std::size_t>(threads, 1 + (n - 1) / kParallelChunk));
    std::atomic<std::size_t> next{0};
    auto work = [&]() {
        for (;;) {
            const std::size_t start = next.fetch_add(kParallelChunk);
            if (start >= n) break;
            const std::size_t end = std::min(n, start + kParallelChunk);
            for (std::size_t i = start; i < end; ++i) fn(i);
        }
    };
    std::vector<std::thread> pool;
    for (unsigned t = 1; t < threads; ++t) pool.emplace_back(launch(work));
    work();
    for (auto& thread : pool) thread.join();
}

template <typename F>
void parallel(std::size_t n, unsigned threads, F fn) {
    const unsigned hardware = threads ? 0u : std::thread::hardware_concurrency();
    parallelWithLauncher(n, threads, hardware, std::move(fn), ThreadLauncher{});
}

} // namespace meshkit::detail
