#include "io/AircraftFile.h"

#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace fsim::io {

namespace {

double toKg(const std::string& unit, double v) { return unit == "KG" ? v : v * 0.45359237; } // (LBS the default)
double toM(const std::string& unit, double v) { return unit == "M" ? v : v * 0.3048; }       // (FT the default)

} // namespace

std::optional<AircraftMetrics> readAircraftMetrics(const std::filesystem::path& xml) {
    std::ifstream in(xml, std::ios::binary);
    if (!in) return std::nullopt;
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string s = ss.str();
    AircraftMetrics m;
    bool any = false;
    std::smatch match;
    static const std::regex span(R"re(<wingspan(?:\s+unit\s*=\s*"(\w+)")?\s*>\s*([-+0-9.eE]+)\s*</wingspan>)re");
    if (std::regex_search(s, match, span)) {
        m.wingspanM = toM(match[1].str(), std::stod(match[2].str()));
        any = true;
    }
    static const std::regex empty(R"re(<emptywt(?:\s+unit\s*=\s*"(\w+)")?\s*>\s*([-+0-9.eE]+)\s*</emptywt>)re");
    if (std::regex_search(s, match, empty)) {
        m.emptyMassKg = toKg(match[1].str(), std::stod(match[2].str()));
        any = true;
    }
    static const std::regex point(R"re(<pointmass[\s\S]*?<weight(?:\s+unit\s*=\s*"(\w+)")?\s*>\s*([-+0-9.eE]+)\s*</weight>)re");
    for (auto it = std::sregex_iterator(s.begin(), s.end(), point); it != std::sregex_iterator(); ++it)
        m.pointMassKg += toKg((*it)[1].str(), std::stod((*it)[2].str()));
    if (!any) return std::nullopt;
    return m;
}

} // namespace fsim::io
