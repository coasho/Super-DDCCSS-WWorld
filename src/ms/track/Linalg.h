#pragma once
// Small fixed-size matrices for the trackers (ADR-30, MS-5): a state of six,
// a measurement of four. Row-major, double, no allocation.

#include <array>
#include <cmath>
#include <cstddef>

namespace fsim::ms::track {

template <std::size_t R, std::size_t C>
struct Mat {
    std::array<double, R * C> a{};
    double& operator()(std::size_t r, std::size_t c) noexcept { return a[r * C + c]; }
    double operator()(std::size_t r, std::size_t c) const noexcept { return a[r * C + c]; }
    static constexpr std::size_t rows = R, cols = C;
};

template <std::size_t N>
using Vec = Mat<N, 1>;

template <std::size_t R, std::size_t K, std::size_t C>
Mat<R, C> operator*(const Mat<R, K>& x, const Mat<K, C>& y) noexcept {
    Mat<R, C> out;
    for (std::size_t r = 0; r < R; ++r)
        for (std::size_t k = 0; k < K; ++k) {
            const double v = x(r, k);
            for (std::size_t c = 0; c < C; ++c) out(r, c) += v * y(k, c);
        }
    return out;
}

template <std::size_t R, std::size_t C>
Mat<R, C> operator+(Mat<R, C> x, const Mat<R, C>& y) noexcept {
    for (std::size_t i = 0; i < R * C; ++i) x.a[i] += y.a[i];
    return x;
}

template <std::size_t R, std::size_t C>
Mat<R, C> operator-(Mat<R, C> x, const Mat<R, C>& y) noexcept {
    for (std::size_t i = 0; i < R * C; ++i) x.a[i] -= y.a[i];
    return x;
}

template <std::size_t R, std::size_t C>
Mat<R, C> operator*(double s, Mat<R, C> x) noexcept {
    for (double& v : x.a) v *= s;
    return x;
}

template <std::size_t R, std::size_t C>
Mat<C, R> transpose(const Mat<R, C>& x) noexcept {
    Mat<C, R> out;
    for (std::size_t r = 0; r < R; ++r)
        for (std::size_t c = 0; c < C; ++c) out(c, r) = x(r, c);
    return out;
}

template <std::size_t N>
Mat<N, N> identity() noexcept {
    Mat<N, N> out;
    for (std::size_t i = 0; i < N; ++i) out(i, i) = 1.0;
    return out;
}

/// The inverse by Gauss-Jordan with partial pivoting; false (and `out` untouched) if singular.
template <std::size_t N>
bool inverse(const Mat<N, N>& m, Mat<N, N>& out) noexcept {
    Mat<N, N> a = m, inv = identity<N>();
    for (std::size_t c = 0; c < N; ++c) {
        std::size_t p = c;
        for (std::size_t r = c + 1; r < N; ++r)
            if (std::fabs(a(r, c)) > std::fabs(a(p, c))) p = r;
        if (!(std::fabs(a(p, c)) > 0.0)) return false;
        if (p != c)
            for (std::size_t k = 0; k < N; ++k) std::swap(a(p, k), a(c, k)), std::swap(inv(p, k), inv(c, k));
        const double d = 1.0 / a(c, c);
        for (std::size_t k = 0; k < N; ++k) a(c, k) *= d, inv(c, k) *= d;
        for (std::size_t r = 0; r < N; ++r) {
            if (r == c) continue;
            const double f = a(r, c);
            if (f == 0.0) continue;
            for (std::size_t k = 0; k < N; ++k) a(r, k) -= f * a(c, k), inv(r, k) -= f * inv(c, k);
        }
    }
    out = inv;
    return true;
}

/// The lower Cholesky factor of a symmetric positive-definite matrix; false if it is not.
template <std::size_t N>
bool cholesky(const Mat<N, N>& m, Mat<N, N>& out) noexcept {
    Mat<N, N> l;
    for (std::size_t j = 0; j < N; ++j) {
        double s = m(j, j);
        for (std::size_t k = 0; k < j; ++k) s -= l(j, k) * l(j, k);
        if (!(s > 0.0)) return false;
        l(j, j) = std::sqrt(s);
        for (std::size_t i = j + 1; i < N; ++i) {
            double t = m(i, j);
            for (std::size_t k = 0; k < j; ++k) t -= l(i, k) * l(j, k);
            l(i, j) = t / l(j, j);
        }
    }
    out = l;
    return true;
}

template <std::size_t N>
double determinant(const Mat<N, N>& m) noexcept {
    Mat<N, N> a = m;
    double det = 1.0;
    for (std::size_t c = 0; c < N; ++c) {
        std::size_t p = c;
        for (std::size_t r = c + 1; r < N; ++r)
            if (std::fabs(a(r, c)) > std::fabs(a(p, c))) p = r;
        if (a(p, c) == 0.0) return 0.0;
        if (p != c) {
            for (std::size_t k = 0; k < N; ++k) std::swap(a(p, k), a(c, k));
            det = -det;
        }
        det *= a(c, c);
        for (std::size_t r = c + 1; r < N; ++r) {
            const double f = a(r, c) / a(c, c);
            for (std::size_t k = c; k < N; ++k) a(r, k) -= f * a(c, k);
        }
    }
    return det;
}

template <std::size_t N>
double trace(const Mat<N, N>& m) noexcept {
    double t = 0.0;
    for (std::size_t i = 0; i < N; ++i) t += m(i, i);
    return t;
}

constexpr double kPi = 3.14159265358979323846;

/// Stone Soup's mod_bearing: into [-pi, pi).
inline double modBearing(double x) noexcept {
    double m = std::fmod(x + kPi, 2.0 * kPi);
    if (m < 0.0) m += 2.0 * kPi;
    return m - kPi;
}

/// Stone Soup's mod_elevation: into [-pi/2, pi/2] (folded, not wrapped).
inline double modElevation(double x) noexcept {
    double m = std::fmod(x, 2.0 * kPi);
    if (m < 0.0) m += 2.0 * kPi;
    const double n = std::floor(m / (kPi / 2.0));
    if (n == 1.0 || n == 2.0) return kPi - m;
    if (n == 3.0) return m - 2.0 * kPi;
    if (n == 4.0) return 0.0;
    return m;
}

} // namespace fsim::ms::track
