#include "foundation/core/Log.h"

#include "foundation/platform/Clock.h"

#include <atomic>
#include <cstdio>
#include <mutex>

namespace fsim::log {

namespace {

std::atomic<int> g_level{static_cast<int>(Level::Info)};
std::mutex g_sinkMutex;

void defaultSink(const Record& r) {
    std::fprintf(stderr, "[%s] %.*s: %.*s\n", levelName(r.level),
                 static_cast<int>(r.category.size()), r.category.data(),
                 static_cast<int>(r.message.size()), r.message.data());
}

Sink& sink() {
    static Sink s = defaultSink;
    return s;
}

} // namespace

const char* levelName(Level level) noexcept {
    switch (level) {
    case Level::Trace: return "trace";
    case Level::Debug: return "debug";
    case Level::Info: return "info";
    case Level::Warn: return "warn";
    case Level::Error: return "error";
    case Level::Off: return "off";
    }
    return "?";
}

void setSink(Sink s) {
    std::lock_guard<std::mutex> lock(g_sinkMutex);
    sink() = s ? std::move(s) : Sink(defaultSink);
}

void setLevel(Level level) noexcept { g_level.store(static_cast<int>(level), std::memory_order_relaxed); }

Level level() noexcept { return static_cast<Level>(g_level.load(std::memory_order_relaxed)); }

bool enabled(Level l) noexcept { return static_cast<int>(l) >= g_level.load(std::memory_order_relaxed); }

void write(Level l, std::string_view category, std::string_view message) {
    if (!enabled(l)) return;
    const Record record{l, category, message, platform::Clock::nanoseconds()};
    // Sink callbacks are serialized on the caller's thread. Logging remains
    // outside the allocation-free steady-state path.
    std::lock_guard<std::mutex> lock(g_sinkMutex);
    sink()(record);
}

} // namespace fsim::log
