#include "Shapes.h"

#include <cmath>

namespace fsim::rcs::shapes {
namespace {

constexpr double kPi = 3.14159265358979323846;

void vertex(Mesh& m, double x, double y, double z, double nx, double ny, double nz) {
    m.positions.insert(m.positions.end(), {float(x), float(y), float(z)});
    m.normals.insert(m.normals.end(), {float(nx), float(ny), float(nz)});
}

/// A flat quad (corners in order), its normal the faces'.
void quad(Mesh& m, const double p[4][3]) {
    const double u[3] = {p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]};
    const double v[3] = {p[3][0] - p[0][0], p[3][1] - p[0][1], p[3][2] - p[0][2]};
    double n[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
    const double l = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    for (double& c : n) c /= l;
    const auto first = static_cast<std::uint32_t>(m.positions.size() / 3);
    for (int i = 0; i < 4; ++i) vertex(m, p[i][0], p[i][1], p[i][2], n[0], n[1], n[2]);
    m.indices.insert(m.indices.end(), {first, first + 1, first + 2, first, first + 2, first + 3});
}

/// Rotate a mesh so that `from` (unit) points along +x.
void alignToX(Mesh& m, const double from[3]) {
    // Rodrigues: the axis from x cross... rotate `from` onto x
    const double x[3] = {1, 0, 0};
    double k[3] = {from[1] * x[2] - from[2] * x[1], from[2] * x[0] - from[0] * x[2], from[0] * x[1] - from[1] * x[0]};
    const double s = std::sqrt(k[0] * k[0] + k[1] * k[1] + k[2] * k[2]), c = from[0];
    if (s < 1e-12) return;
    for (double& v : k) v /= s;
    auto rotate = [&](float* p) {
        const double v[3] = {p[0], p[1], p[2]};
        const double kv = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
        const double kxv[3] = {k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0]};
        for (int i = 0; i < 3; ++i) p[i] = float(v[i] * c + kxv[i] * s + k[i] * kv * (1 - c));
    };
    for (std::size_t i = 0; i < m.positions.size(); i += 3) rotate(&m.positions[i]), rotate(&m.normals[i]);
}

} // namespace

Mesh sphere(double a, int rings) {
    Mesh m;
    const int segments = 2 * rings;
    for (int i = 0; i <= rings; ++i) {
        const double th = kPi * i / rings;
        for (int j = 0; j <= segments; ++j) {
            const double ph = 2 * kPi * j / segments;
            const double n[3] = {std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)};
            vertex(m, a * n[0], a * n[1], a * n[2], n[0], n[1], n[2]);
        }
    }
    for (int i = 0; i < rings; ++i)
        for (int j = 0; j < segments; ++j) {
            const auto p = static_cast<std::uint32_t>(i * (segments + 1) + j), q = p + static_cast<std::uint32_t>(segments + 1);
            m.indices.insert(m.indices.end(), {p, q, p + 1, p + 1, q, q + 1});
        }
    return m;
}

Mesh plate(double a, int cells) {
    Mesh m;
    const double h = a / 2, d = a / cells;
    for (int i = 0; i < cells; ++i)
        for (int j = 0; j < cells; ++j) {
            const double y0 = -h + i * d, z0 = -h + j * d;
            const double p[4][3] = {{0, y0, z0}, {0, y0 + d, z0}, {0, y0 + d, z0 + d}, {0, y0, z0 + d}};
            quad(m, p);
        }
    return m;
}

Mesh dihedral(double a) {
    Mesh m;
    // faces through the seam (the y axis), at +-45 deg from -x: the opening towards +x
    const double c = a / std::sqrt(2.0), h = a / 2;
    const double top[4][3] = {{0, -h, 0}, {0, h, 0}, {c, h, -c}, {c, -h, -c}};
    const double bottom[4][3] = {{0, -h, 0}, {c, -h, c}, {c, h, c}, {0, h, 0}};
    quad(m, top);
    quad(m, bottom);
    return m;
}

Mesh trihedral(double a) {
    Mesh m;
    const double xy[4][3] = {{0, 0, 0}, {a, 0, 0}, {a, a, 0}, {0, a, 0}};
    const double yz[4][3] = {{0, 0, 0}, {0, a, 0}, {0, a, a}, {0, 0, a}};
    const double zx[4][3] = {{0, 0, 0}, {0, 0, a}, {a, 0, a}, {a, 0, 0}};
    quad(m, xy);
    quad(m, yz);
    quad(m, zx);
    // its symmetry axis (1,1,1) turned onto +x, the corner at the origin
    const double axis[3] = {1 / std::sqrt(3.0), 1 / std::sqrt(3.0), 1 / std::sqrt(3.0)};
    alignToX(m, axis);
    return m;
}

Mesh cylinder(double a, double l, int sides) {
    Mesh m;
    const double h = l / 2;
    for (int j = 0; j <= sides; ++j) {
        const double ph = 2 * kPi * j / sides, nx = std::cos(ph), nz = std::sin(ph);
        vertex(m, a * nx, -h, a * nz, nx, 0, nz);
        vertex(m, a * nx, h, a * nz, nx, 0, nz);
    }
    for (int j = 0; j < sides; ++j) {
        const auto p = static_cast<std::uint32_t>(2 * j);
        m.indices.insert(m.indices.end(), {p, p + 2, p + 1, p + 1, p + 2, p + 3});
    }
    for (int end = 0; end < 2; ++end) { // the caps (flat)
        const double y = end ? h : -h, ny = end ? 1 : -1;
        const auto centre = static_cast<std::uint32_t>(m.positions.size() / 3);
        vertex(m, 0, y, 0, 0, ny, 0);
        for (int j = 0; j <= sides; ++j) {
            const double ph = 2 * kPi * j / sides;
            vertex(m, a * std::cos(ph), y, a * std::sin(ph), 0, ny, 0);
        }
        for (int j = 0; j < sides; ++j) m.indices.insert(m.indices.end(), {centre, centre + 1 + static_cast<std::uint32_t>(j), centre + 2 + static_cast<std::uint32_t>(j)});
    }
    return m;
}

} // namespace fsim::rcs::shapes
