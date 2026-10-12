#pragma once

// Private C ABI helpers. Internal linkage in each adapter preserves the original
// helper symbols; the error string itself remains the shared LastError store.

#include "fsim/fsim_c.h"
#include "core/Log.h"
#include "sdk/LastError.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <stdexcept>
#include <string>

namespace {

int fail(int code, const std::string& message) noexcept {
    fsim::sdk::lastError() = message;
    LOG_ERROR("sdk") << message;
    return code;
}

int guard(const char* what, const std::function<int()>& fn) noexcept {
    try {
        const int r = fn();
        if (r == FSIM_OK) fsim::sdk::lastError().clear();
        return r;
    } catch (const std::invalid_argument& e) {
        return fail(FSIM_INVALID_ARGUMENT, std::string(what) + ": " + e.what());
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string(what) + ": " + e.what());
    } catch (...) {
        return fail(FSIM_ERROR, std::string(what) + ": unknown error");
    }
}

/// `local` into the caller's struct, up to the size the caller gives (its struct_size first); false if it gives none.
template <typename T>
bool copyOut(const T& local, T* out) noexcept {
    if (!out || out->struct_size < sizeof(uint32_t)) return false;
    const uint32_t size = out->struct_size;
    std::memcpy(reinterpret_cast<char*>(out) + sizeof(uint32_t), reinterpret_cast<const char*>(&local) + sizeof(uint32_t),
                std::min<std::size_t>(size, sizeof(T)) - sizeof(uint32_t));
    return true;
}

} // namespace
