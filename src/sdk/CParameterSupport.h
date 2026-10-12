#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace {

std::vector<std::pair<std::string, double>> params(const char* const* names, const double* values, uint32_t count) {
    std::vector<std::pair<std::string, double>> out;
    if (!names || !values) return out;
    for (uint32_t i = 0; i < count; ++i) out.emplace_back(names[i] ? names[i] : "", values[i]);
    return out;
}

} // namespace
