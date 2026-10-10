#include "Model.h"

#include "core/Json.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iterator>
#include <vector>

namespace fsim::rcs {
namespace {

using core::Json;
using Mat = std::array<double, 16>; // column-major, as glTF

Mat identity() { return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; }

Mat multiply(const Mat& a, const Mat& b) {
    Mat r{};
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += a[static_cast<std::size_t>(k * 4 + row)] * b[static_cast<std::size_t>(c * 4 + k)];
            r[static_cast<std::size_t>(c * 4 + row)] = s;
        }
    return r;
}

Mat local(const Json& node) {
    if (const Json* m = node.find("matrix")) {
        Mat r{};
        for (std::size_t i = 0; i < 16; ++i) r[i] = m->asArray()[i].asNumber();
        return r;
    }
    double t[3] = {0, 0, 0}, q[4] = {0, 0, 0, 1}, s[3] = {1, 1, 1};
    if (const Json* v = node.find("translation")) for (std::size_t i = 0; i < 3; ++i) t[i] = v->asArray()[i].asNumber();
    if (const Json* v = node.find("rotation")) for (std::size_t i = 0; i < 4; ++i) q[i] = v->asArray()[i].asNumber();
    if (const Json* v = node.find("scale")) for (std::size_t i = 0; i < 3; ++i) s[i] = v->asArray()[i].asNumber();
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w),
                         2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
                         2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)}; // columns
    return {r[0] * s[0], r[1] * s[0], r[2] * s[0], 0, r[3] * s[1], r[4] * s[1], r[5] * s[1], 0,
            r[6] * s[2], r[7] * s[2], r[8] * s[2], 0, t[0], t[1], t[2], 1};
}

// (a station's pylon and launcher: shown only with a store there - the clean aircraft's signature is without them; a
// weapon bay's doors: its cover, fsim:cover:<bay>, is the bay shut - the doors' gaps are the viewer's, for them to turn)
bool excludedNode(const std::string& name) {
    return name.rfind("fsim:afterburner", 0) == 0 || name.rfind("fsim:station:", 0) == 0 || name.rfind("fsim:bay:", 0) == 0;
}

// (a breakup section's cap over its break - inside the airframe, no surface of it - and the exhaust's flame mesh)
bool excludedMaterial(const std::string& name) { return name == "afterburner" || name == "afterburner core" || name == "break"; }

/// The engine's kinds of surface (Mesh::kinds) by the hangar's materials.
std::uint8_t kindOf(const std::string& material) {
    if (material == "dark") return 1;
    if (material == "nozzle") return 2;
    if (material == "liner" || material == "turbine") return 3;
    return 0;
}

/// A gear joint's turn about its local x at gear position 0 (retracted): fsim:gear:<deg>[:<g0>:<g1>], as the viewer
/// (VehicleVisuals::Joint) turns it by deg * clamp((g - g0) / (g1 - g0), 0, 1).
Mat gearTurn(const std::string& name) {
    if (name.rfind("fsim:gear:", 0) != 0) return identity();
    std::vector<double> v;
    std::string rest = name.substr(10);
    for (std::size_t b = 0; b <= rest.size();) {
        const auto c = rest.find(':', b);
        v.push_back(std::atof(rest.substr(b, c == std::string::npos ? std::string::npos : c - b).c_str()));
        if (c == std::string::npos) break;
        b = c + 1;
    }
    const double g0 = v.size() >= 3 ? v[1] : 1.0, g1 = v.size() >= 3 ? v[2] : 0.0;
    if (v.empty() || g0 == g1) return identity();
    const double t = std::clamp((0.0 - g0) / (g1 - g0), 0.0, 1.0), a = v[0] * 3.14159265358979323846 / 180.0 * t;
    const double c = std::cos(a), s = std::sin(a);
    return {1, 0, 0, 0, 0, c, s, 0, 0, -s, c, 0, 0, 0, 0, 1};
}

} // namespace

bool readModel(const std::filesystem::path& file, Mesh& out, ModelReport* report, std::string* error) {
    auto failed = [&](const std::string& why) {
        if (error) *error = file.string() + ": " + why;
        return false;
    };
    std::ifstream in(file, std::ios::binary);
    if (!in) return failed("cannot open");
    std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    auto u32 = [&](std::size_t at) {
        std::uint32_t v;
        std::memcpy(&v, bytes.data() + at, 4);
        return v;
    };
    if (bytes.size() < 20 || u32(0) != 0x46546C67u || u32(16) != 0x4E4F534Au) return failed("not a glTF binary");
    const std::uint32_t jsonLength = u32(12);
    const std::size_t binStart = 20 + jsonLength + 8;
    if (binStart > bytes.size()) return failed("no binary chunk");
    Json doc;
    try {
        doc = Json::parse(std::string_view(bytes.data() + 20, jsonLength), file.string());
    } catch (const std::exception& e) {
        return failed(e.what());
    }
    const char* bin = bytes.data() + binStart;
    const auto& accessors = doc.child("accessors").asArray();
    const auto& views = doc.child("bufferViews").asArray();
    const auto& meshes = doc.child("meshes").asArray();
    const auto& nodes = doc.child("nodes").asArray();
    const Json& materials = doc.child("materials");

    // an accessor's elements as doubles (floats) or indices
    auto read = [&](std::size_t index, std::vector<double>& values, int& width) {
        const Json& a = accessors[index];
        const Json& v = views[static_cast<std::size_t>(a.number("bufferView", 0))];
        const std::string type = a.string("type", "SCALAR");
        width = type == "VEC3" ? 3 : type == "VEC2" ? 2 : type == "VEC4" ? 4 : 1;
        const auto count = static_cast<std::size_t>(a.number("count", 0));
        const auto component = static_cast<int>(a.number("componentType", 5126));
        const std::size_t size = component == 5126 || component == 5125 ? 4 : component == 5123 ? 2 : 1;
        const auto stride = static_cast<std::size_t>(v.number("byteStride", static_cast<double>(size * static_cast<std::size_t>(width))));
        const char* base = bin + static_cast<std::size_t>(v.number("byteOffset", 0)) + static_cast<std::size_t>(a.number("byteOffset", 0));
        values.resize(count * static_cast<std::size_t>(width));
        for (std::size_t i = 0; i < count; ++i)
            for (int c = 0; c < width; ++c) {
                const char* p = base + i * stride + static_cast<std::size_t>(c) * size;
                double x;
                if (component == 5126) { float f; std::memcpy(&f, p, 4); x = f; }
                else if (component == 5125) { std::uint32_t u; std::memcpy(&u, p, 4); x = u; }
                else if (component == 5123) { std::uint16_t u; std::memcpy(&u, p, 2); x = u; }
                else { std::uint8_t u; std::memcpy(&u, p, 1); x = u; }
                values[i * static_cast<std::size_t>(width) + static_cast<std::size_t>(c)] = x;
            }
    };

    ModelReport r;
    std::function<void(std::size_t, const Mat&)> visit = [&](std::size_t n, const Mat& parent) {
        const Json& node = nodes[n];
        if (excludedNode(node.string("name", ""))) {
            ++r.skippedNodes;
            return;
        }
        const Mat m = multiply(multiply(parent, local(node)), gearTurn(node.string("name", "")));
        if (const Json* mi = node.find("mesh")) {
            ++r.meshes;
            r.parts.push_back(ModelPart{node.string("name", ""), out.indices.size() / 3, 0});
            for (const Json& prim : meshes[static_cast<std::size_t>(mi->asNumber())].child("primitives").asArray()) {
                const Json& attrs = prim.child("attributes");
                if (!attrs.find("POSITION") || !prim.find("indices")) continue;
                std::vector<double> pos, nor, idx;
                int wp, wn = 3, wi;
                read(static_cast<std::size_t>(attrs.child("POSITION").asNumber()), pos, wp);
                if (attrs.find("NORMAL")) read(static_cast<std::size_t>(attrs.child("NORMAL").asNumber()), nor, wn);
                read(static_cast<std::size_t>(prim.child("indices").asNumber()), idx, wi);
                const std::string material = prim.find("material") && materials.isArray()
                                                 ? materials.asArray()[static_cast<std::size_t>(prim.child("material").asNumber())].string("name", "")
                                                 : std::string();
                const bool skip = excludedMaterial(material);
                if (skip) {
                    r.skippedTriangles += idx.size() / 3;
                    continue;
                }
                const auto first = static_cast<std::uint32_t>(out.positions.size() / 3);
                const std::size_t nv = pos.size() / 3;
                for (std::size_t i = 0; i < nv; ++i) {
                    double g[3], gn[3] = {0, 0, 0};
                    for (int row = 0; row < 3; ++row) {
                        g[row] = m[static_cast<std::size_t>(12 + row)];
                        for (int c = 0; c < 3; ++c) {
                            g[row] += m[static_cast<std::size_t>(c * 4 + row)] * pos[3 * i + static_cast<std::size_t>(c)];
                            if (!nor.empty()) gn[row] += m[static_cast<std::size_t>(c * 4 + row)] * nor[3 * i + static_cast<std::size_t>(c)];
                        }
                    }
                    const double len = std::sqrt(gn[0] * gn[0] + gn[1] * gn[1] + gn[2] * gn[2]);
                    if (len > 0) for (double& c : gn) c /= len;
                    // glTF (x left, y up, z forward) to the body axes (x forward, y right, z down)
                    const double b[3] = {g[2], -g[0], -g[1]}, bn[3] = {gn[2], -gn[0], -gn[1]};
                    for (int c = 0; c < 3; ++c) {
                        out.positions.push_back(static_cast<float>(b[c]));
                        out.normals.push_back(static_cast<float>(bn[c]));
                    }
                }
                for (double i : idx) out.indices.push_back(first + static_cast<std::uint32_t>(i));
                out.kinds.insert(out.kinds.end(), idx.size() / 3, kindOf(material));
                r.triangles += idx.size() / 3;
                r.parts.back().triangles += idx.size() / 3;
            }
        }
        if (const Json* children = node.find("children"))
            for (const Json& c : children->asArray()) visit(static_cast<std::size_t>(c.asNumber()), m);
    };
    const Json& scenes = doc.child("scenes");
    const auto scene = static_cast<std::size_t>(doc.number("scene", 0));
    if (!scenes.isArray() || scene >= scenes.asArray().size()) return failed("no scene");
    for (const Json& root : scenes.asArray()[scene].child("nodes").asArray()) visit(static_cast<std::size_t>(root.asNumber()), identity());
    if (report) *report = r;
    if (out.indices.empty()) return failed("no triangles");
    return true;
}

} // namespace fsim::rcs
