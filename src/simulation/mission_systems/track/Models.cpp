#include "simulation/mission_systems/track/Models.h"

namespace fsim::ms::track {

Covar ConstantVelocity::matrix(double dt) const noexcept {
    Covar f = identity<6>();
    for (std::size_t k = 0; k < 3; ++k) f(2 * k, 2 * k + 1) = dt;
    return f;
}

Covar ConstantVelocity::covar(double dt) const noexcept {
    const double t = std::fabs(dt);
    Covar c;
    for (std::size_t k = 0; k < 3; ++k) {
        const std::size_t p = 2 * k, v = p + 1;
        c(p, p) = t * t * t / 3.0 * q[k];
        c(p, v) = c(v, p) = t * t / 2.0 * q[k];
        c(v, v) = t * q[k];
    }
    return c;
}

Meas RadarMeasurement::function(const State& x) const noexcept {
    const double pos[3] = {x.a[0] - translation[0], x.a[2] - translation[1], x.a[4] - translation[2]};
    const double vel[3] = {x.a[1] - velocity[0], x.a[3] - velocity[1], x.a[5] - velocity[2]};
    double rot[3];
    for (std::size_t i = 0; i < 3; ++i) rot[i] = rotation(i, 0) * pos[0] + rotation(i, 1) * pos[1] + rotation(i, 2) * pos[2];
    const double rho = std::sqrt(rot[0] * rot[0] + rot[1] * rot[1] + rot[2] * rot[2]);
    const double norm = std::sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]);
    Meas z;
    z.a[0] = std::asin(rot[2] / rho);
    z.a[1] = std::atan2(rot[1], rot[0]);
    z.a[2] = rho;
    z.a[3] = (pos[0] * vel[0] + pos[1] * vel[1] + pos[2] * vel[2]) / norm;
    return z;
}

Mat<4, 6> RadarMeasurement::jacobian(const State& s) const noexcept {
    const double p0[3] = {s.a[0] - translation[0], s.a[2] - translation[1], s.a[4] - translation[2]};
    const double v0[3] = {s.a[1] - velocity[0], s.a[3] - velocity[1], s.a[5] - velocity[2]};
    double p[3], v[3];
    for (std::size_t i = 0; i < 3; ++i) {
        p[i] = rotation(i, 0) * p0[0] + rotation(i, 1) * p0[1] + rotation(i, 2) * p0[2];
        v[i] = rotation(i, 0) * v0[0] + rotation(i, 1) * v0[1] + rotation(i, 2) * v0[2];
    }
    const double x = p[0], y = p[1], z = p[2], vx = v[0], vy = v[1], vz = v[2];
    const double x2 = x * x, y2 = y * y, z2 = z * z, x2y2 = x2 + y2, r2 = x2y2 + z2, r = std::sqrt(r2);
    const double sqrtx2y2 = std::sqrt(x2y2), r32 = r2 * r, sqrtx2y2r2 = sqrtx2y2 * r2;
    // with respect to the rotated position and velocity (the columns of the sensor frame's x, y, z)
    double jp[4][3] = {}, jv[4][3] = {};
    jp[0][0] = -(x * z) / sqrtx2y2r2, jp[0][1] = -(y * z) / sqrtx2y2r2, jp[0][2] = sqrtx2y2 / r2;
    jp[1][0] = -y / x2y2, jp[1][1] = x / x2y2;
    jp[2][0] = jv[3][0] = x / r, jp[2][1] = jv[3][1] = y / r, jp[2][2] = jv[3][2] = z / r;
    const double vxx = vx * x, vyy = vy * y, vzz = vz * z;
    jp[3][0] = (-x * (vyy + vzz) + vx * (y2 + z2)) / r32;
    jp[3][1] = (vy * (x2 + z2) - y * (vxx + vzz)) / r32;
    jp[3][2] = (vz * x2y2 - (vxx + vyy) * z) / r32;
    // then with respect to the world's: post-multiplied by the rotation
    Mat<4, 6> jac;
    for (std::size_t m = 0; m < 4; ++m)
        for (std::size_t c = 0; c < 3; ++c) {
            double dp = 0.0, dv = 0.0;
            for (std::size_t k = 0; k < 3; ++k) dp += jp[m][k] * rotation(k, c), dv += jv[m][k] * rotation(k, c);
            jac(m, 2 * c) = dp, jac(m, 2 * c + 1) = dv;
        }
    return jac;
}

State RadarMeasurement::inverse(const Meas& m) const noexcept {
    const double theta = m.a[0], phi = m.a[1], rho = m.a[2], rate = m.a[3];
    // sphere2cart(rho, phi, theta): x = rho cos(theta) cos(phi), y = rho cos(theta) sin(phi), z = rho sin(theta)
    const double p[3] = {rho * std::cos(theta) * std::cos(phi), rho * std::cos(theta) * std::sin(phi), rho * std::sin(theta)};
    const double v[3] = {rate * std::cos(theta) * std::cos(phi), rate * std::cos(theta) * std::sin(phi), rate * std::sin(theta)};
    Mat<3, 3> inv;
    if (!track::inverse(rotation, inv)) inv = transpose(rotation);
    State x;
    for (std::size_t i = 0; i < 3; ++i) {
        x.a[2 * i] = inv(i, 0) * p[0] + inv(i, 1) * p[1] + inv(i, 2) * p[2] + translation[i];
        x.a[2 * i + 1] = inv(i, 0) * v[0] + inv(i, 1) * v[1] + inv(i, 2) * v[2] + velocity[i];
    }
    return x;
}

Meas RadarMeasurement::difference(const Meas& z, const Meas& prediction) noexcept {
    Meas d = z - prediction;
    d.a[0] = modElevation(d.a[0]);
    d.a[1] = modBearing(d.a[1]);
    return d;
}

} // namespace fsim::ms::track
