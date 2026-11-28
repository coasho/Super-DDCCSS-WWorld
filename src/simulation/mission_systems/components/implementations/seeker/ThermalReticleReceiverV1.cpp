#include "ThermalReticleReceiverV1.h"
#include "MovingRadiatorInverseV2.h"
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
#include "simulation/mission_systems/components/support/MovingConicMomentsV4.h"
#include <algorithm>
#include <atomic>
#include <cfenv>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <limits>
#include <mutex>
#include <numeric>
#include <chrono>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <cstring>
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
#include <xmmintrin.h>
#endif
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
#include <immintrin.h>
#endif

namespace fsim::ms::components {
namespace {
inline bool hasAvx2FmaV1() noexcept {
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
    static const bool supported = (__builtin_cpu_supports("avx2") && __builtin_cpu_supports("fma"));
    return supported;
#else
    return false;
#endif
}
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
__attribute__((target("avx2,fma")))
void reticleFmadd1Avx2(double* __restrict dst, const double* __restrict a, double c0, unsigned n) noexcept {
    const __m256d vc0 = _mm256_set1_pd(c0);
    unsigned j = 0;
    for (; j + 16u <= n; j += 16u) {
        _mm256_storeu_pd(dst + j,      _mm256_fmadd_pd(vc0, _mm256_loadu_pd(a + j),      _mm256_loadu_pd(dst + j)));
        _mm256_storeu_pd(dst + j + 4,  _mm256_fmadd_pd(vc0, _mm256_loadu_pd(a + j + 4),  _mm256_loadu_pd(dst + j + 4)));
        _mm256_storeu_pd(dst + j + 8,  _mm256_fmadd_pd(vc0, _mm256_loadu_pd(a + j + 8),  _mm256_loadu_pd(dst + j + 8)));
        _mm256_storeu_pd(dst + j + 12, _mm256_fmadd_pd(vc0, _mm256_loadu_pd(a + j + 12), _mm256_loadu_pd(dst + j + 12)));
    }
    for (; j + 4u <= n; j += 4u)
        _mm256_storeu_pd(dst + j, _mm256_fmadd_pd(vc0, _mm256_loadu_pd(a + j), _mm256_loadu_pd(dst + j)));
    for (; j < n; ++j) dst[j] += c0 * a[j];
}
__attribute__((target("avx2,fma")))
void reticleFmadd2Avx2(double* __restrict dst, const double* __restrict a, double c0,
                       const double* __restrict b, double c1, unsigned n) noexcept {
    const __m256d vc0 = _mm256_set1_pd(c0), vc1 = _mm256_set1_pd(c1);
    unsigned j = 0;
    for (; j + 8u <= n; j += 8u) {
        __m256d r0 = _mm256_fmadd_pd(vc0, _mm256_loadu_pd(a + j),     _mm256_loadu_pd(dst + j));
        __m256d r1 = _mm256_fmadd_pd(vc0, _mm256_loadu_pd(a + j + 4), _mm256_loadu_pd(dst + j + 4));
        _mm256_storeu_pd(dst + j,     _mm256_fmadd_pd(vc1, _mm256_loadu_pd(b + j),     r0));
        _mm256_storeu_pd(dst + j + 4, _mm256_fmadd_pd(vc1, _mm256_loadu_pd(b + j + 4), r1));
    }
    for (; j + 4u <= n; j += 4u) {
        __m256d r0 = _mm256_fmadd_pd(vc0, _mm256_loadu_pd(a + j), _mm256_loadu_pd(dst + j));
        _mm256_storeu_pd(dst + j, _mm256_fmadd_pd(vc1, _mm256_loadu_pd(b + j), r0));
    }
    for (; j < n; ++j) dst[j] += c0 * a[j] + c1 * b[j];
}
__attribute__((target("avx2,fma")))
void reticleFmadd3Avx2(double* __restrict dst, const double* __restrict a, double c0,
                       const double* __restrict b, double c1,
                       const double* __restrict c, double c2, unsigned n) noexcept {
    const __m256d vc0 = _mm256_set1_pd(c0), vc1 = _mm256_set1_pd(c1), vc2 = _mm256_set1_pd(c2);
    unsigned j = 0;
    for (; j + 8u <= n; j += 8u) {
        __m256d r0 = _mm256_fmadd_pd(vc0, _mm256_loadu_pd(a + j),     _mm256_loadu_pd(dst + j));
        __m256d r1 = _mm256_fmadd_pd(vc0, _mm256_loadu_pd(a + j + 4), _mm256_loadu_pd(dst + j + 4));
        r0 = _mm256_fmadd_pd(vc1, _mm256_loadu_pd(b + j),     r0);
        r1 = _mm256_fmadd_pd(vc1, _mm256_loadu_pd(b + j + 4), r1);
        _mm256_storeu_pd(dst + j,     _mm256_fmadd_pd(vc2, _mm256_loadu_pd(c + j),     r0));
        _mm256_storeu_pd(dst + j + 4, _mm256_fmadd_pd(vc2, _mm256_loadu_pd(c + j + 4), r1));
    }
    for (; j + 4u <= n; j += 4u) {
        __m256d r0 = _mm256_fmadd_pd(vc0, _mm256_loadu_pd(a + j), _mm256_loadu_pd(dst + j));
        r0 = _mm256_fmadd_pd(vc1, _mm256_loadu_pd(b + j), r0);
        _mm256_storeu_pd(dst + j, _mm256_fmadd_pd(vc2, _mm256_loadu_pd(c + j), r0));
    }
    for (; j < n; ++j) dst[j] += c0 * a[j] + c1 * b[j] + c2 * c[j];
}
__attribute__((target("avx2,fma")))
void reticleAddDiffAvx2(double* __restrict dst, const double* __restrict hi,
                        const double* __restrict lo, unsigned n) noexcept {
    unsigned j = 0;
    for (; j + 8u <= n; j += 8u) {
        _mm256_storeu_pd(dst + j,     _mm256_add_pd(_mm256_loadu_pd(dst + j),     _mm256_sub_pd(_mm256_loadu_pd(hi + j),     _mm256_loadu_pd(lo + j))));
        _mm256_storeu_pd(dst + j + 4, _mm256_add_pd(_mm256_loadu_pd(dst + j + 4), _mm256_sub_pd(_mm256_loadu_pd(hi + j + 4), _mm256_loadu_pd(lo + j + 4))));
    }
    for (; j + 4u <= n; j += 4u)
        _mm256_storeu_pd(dst + j, _mm256_add_pd(_mm256_loadu_pd(dst + j), _mm256_sub_pd(_mm256_loadu_pd(hi + j), _mm256_loadu_pd(lo + j))));
    for (; j < n; ++j) dst[j] += hi[j] - lo[j];
}
__attribute__((target("avx2,fma")))
void reticleFmaddDiffAvx2(double* __restrict dst, const double* __restrict hi,
                          const double* __restrict lo, double w, unsigned n) noexcept {
    const __m256d vw = _mm256_set1_pd(w);
    unsigned j = 0;
    for (; j + 8u <= n; j += 8u) {
        __m256d d0 = _mm256_sub_pd(_mm256_loadu_pd(hi + j),     _mm256_loadu_pd(lo + j));
        __m256d d1 = _mm256_sub_pd(_mm256_loadu_pd(hi + j + 4), _mm256_loadu_pd(lo + j + 4));
        _mm256_storeu_pd(dst + j,     _mm256_fmadd_pd(vw, d0, _mm256_loadu_pd(dst + j)));
        _mm256_storeu_pd(dst + j + 4, _mm256_fmadd_pd(vw, d1, _mm256_loadu_pd(dst + j + 4)));
    }
    for (; j + 4u <= n; j += 4u)
        _mm256_storeu_pd(dst + j, _mm256_fmadd_pd(vw, _mm256_sub_pd(_mm256_loadu_pd(hi + j), _mm256_loadu_pd(lo + j)), _mm256_loadu_pd(dst + j)));
    for (; j < n; ++j) dst[j] += w * (hi[j] - lo[j]);
}
__attribute__((target("avx2,fma")))
double reticleDotAvx2(const double* __restrict a, const double* __restrict b, unsigned n) noexcept {
    __m256d s0 = _mm256_setzero_pd(), s1 = _mm256_setzero_pd();
    __m256d s2 = _mm256_setzero_pd(), s3 = _mm256_setzero_pd();
    unsigned j = 0;
    for (; j + 16u <= n; j += 16u) {
        s0 = _mm256_fmadd_pd(_mm256_loadu_pd(a + j),      _mm256_loadu_pd(b + j),      s0);
        s1 = _mm256_fmadd_pd(_mm256_loadu_pd(a + j + 4),  _mm256_loadu_pd(b + j + 4),  s1);
        s2 = _mm256_fmadd_pd(_mm256_loadu_pd(a + j + 8),  _mm256_loadu_pd(b + j + 8),  s2);
        s3 = _mm256_fmadd_pd(_mm256_loadu_pd(a + j + 12), _mm256_loadu_pd(b + j + 12), s3);
    }
    for (; j + 4u <= n; j += 4u)
        s0 = _mm256_fmadd_pd(_mm256_loadu_pd(a + j), _mm256_loadu_pd(b + j), s0);
    __m256d s = _mm256_add_pd(_mm256_add_pd(s0, s1), _mm256_add_pd(s2, s3));
    __m128d lo = _mm256_castpd256_pd128(s), hi = _mm256_extractf128_pd(s, 1);
    __m128d sum2 = _mm_add_pd(lo, hi);
    double out = _mm_cvtsd_f64(_mm_add_sd(sum2, _mm_unpackhi_pd(sum2, sum2)));
    for (; j < n; ++j) out += a[j] * b[j];
    return out;
}
__attribute__((target("avx2,fma")))
void reticleDot4Avx2(const double* __restrict d,
                     const double* __restrict b0, const double* __restrict b1,
                     const double* __restrict b2, const double* __restrict b3,
                     unsigned n, double out[4]) noexcept {
    __m256d a0 = _mm256_setzero_pd(), a1 = _mm256_setzero_pd();
    __m256d a2 = _mm256_setzero_pd(), a3 = _mm256_setzero_pd();
    unsigned j = 0;
    for (; j + 4u <= n; j += 4u) {
        const __m256d vd = _mm256_loadu_pd(d + j);
        a0 = _mm256_fmadd_pd(vd, _mm256_loadu_pd(b0 + j), a0);
        a1 = _mm256_fmadd_pd(vd, _mm256_loadu_pd(b1 + j), a1);
        a2 = _mm256_fmadd_pd(vd, _mm256_loadu_pd(b2 + j), a2);
        a3 = _mm256_fmadd_pd(vd, _mm256_loadu_pd(b3 + j), a3);
    }
    const __m256d acc[4] = {a0, a1, a2, a3};
    const double* bp[4] = {b0, b1, b2, b3};
    for (int k = 0; k < 4; ++k) {
        __m128d lo = _mm256_castpd256_pd128(acc[k]), hi = _mm256_extractf128_pd(acc[k], 1);
        __m128d sum2 = _mm_add_pd(lo, hi);
        double v = _mm_cvtsd_f64(_mm_add_sd(sum2, _mm_unpackhi_pd(sum2, sum2)));
        for (unsigned r = j; r < n; ++r) v += d[r] * bp[k][r];
        out[k] = v;
    }
}
__attribute__((target("avx2,fma")))
void reticleNormDotAvx2(const double* __restrict d, const double* __restrict w, unsigned n,
                        double& outNorm2, double& outCorr) noexcept {
    __m256d n0 = _mm256_setzero_pd(), n1 = _mm256_setzero_pd();
    __m256d c0 = _mm256_setzero_pd(), c1 = _mm256_setzero_pd();
    unsigned j = 0;
    for (; j + 8u <= n; j += 8u) {
        const __m256d w0 = _mm256_loadu_pd(w + j),     w1 = _mm256_loadu_pd(w + j + 4);
        const __m256d d0 = _mm256_loadu_pd(d + j),     d1 = _mm256_loadu_pd(d + j + 4);
        n0 = _mm256_fmadd_pd(w0, w0, n0);
        n1 = _mm256_fmadd_pd(w1, w1, n1);
        c0 = _mm256_fmadd_pd(d0, w0, c0);
        c1 = _mm256_fmadd_pd(d1, w1, c1);
    }
    for (; j + 4u <= n; j += 4u) {
        const __m256d w0 = _mm256_loadu_pd(w + j), d0 = _mm256_loadu_pd(d + j);
        n0 = _mm256_fmadd_pd(w0, w0, n0);
        c0 = _mm256_fmadd_pd(d0, w0, c0);
    }
    __m256d ns = _mm256_add_pd(n0, n1), cs = _mm256_add_pd(c0, c1);
    __m128d n2 = _mm_add_pd(_mm256_castpd256_pd128(ns), _mm256_extractf128_pd(ns, 1));
    __m128d c2 = _mm_add_pd(_mm256_castpd256_pd128(cs), _mm256_extractf128_pd(cs, 1));
    double norm2 = _mm_cvtsd_f64(_mm_add_sd(n2, _mm_unpackhi_pd(n2, n2)));
    double corr  = _mm_cvtsd_f64(_mm_add_sd(c2, _mm_unpackhi_pd(c2, c2)));
    for (; j < n; ++j) { norm2 += w[j] * w[j]; corr += d[j] * w[j]; }
    outNorm2 = norm2;
    outCorr = corr;
}
__attribute__((target("avx2,fma")))
double reticleResidualNorm2Avx2(const double* __restrict d, const double* __restrict w,
                                double amp, unsigned n) noexcept {
    const __m256d vamp = _mm256_set1_pd(amp);
    __m256d s0 = _mm256_setzero_pd(), s1 = _mm256_setzero_pd();
    unsigned j = 0;
    for (; j + 8u <= n; j += 8u) {
        const __m256d r0 = _mm256_fnmadd_pd(vamp, _mm256_loadu_pd(w + j),     _mm256_loadu_pd(d + j));
        const __m256d r1 = _mm256_fnmadd_pd(vamp, _mm256_loadu_pd(w + j + 4), _mm256_loadu_pd(d + j + 4));
        s0 = _mm256_fmadd_pd(r0, r0, s0);
        s1 = _mm256_fmadd_pd(r1, r1, s1);
    }
    for (; j + 4u <= n; j += 4u) {
        const __m256d r0 = _mm256_fnmadd_pd(vamp, _mm256_loadu_pd(w + j), _mm256_loadu_pd(d + j));
        s0 = _mm256_fmadd_pd(r0, r0, s0);
    }
    __m256d s = _mm256_add_pd(s0, s1);
    __m128d sum2 = _mm_add_pd(_mm256_castpd256_pd128(s), _mm256_extractf128_pd(s, 1));
    double out = _mm_cvtsd_f64(_mm_add_sd(sum2, _mm_unpackhi_pd(sum2, sum2)));
    for (; j < n; ++j) { const double r = d[j] - amp * w[j]; out += r * r; }
    return out;
}
#endif
inline void reticleFmadd1(double* dst, const double* a, double c0, unsigned n) noexcept {
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
    if (hasAvx2FmaV1()) { reticleFmadd1Avx2(dst, a, c0, n); return; }
#endif
    for (unsigned j = 0; j < n; ++j) dst[j] += c0 * a[j];
}
inline void reticleFmadd2(double* dst, const double* a, double c0,
                          const double* b, double c1, unsigned n) noexcept {
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
    if (hasAvx2FmaV1()) { reticleFmadd2Avx2(dst, a, c0, b, c1, n); return; }
#endif
    for (unsigned j = 0; j < n; ++j) dst[j] += c0 * a[j] + c1 * b[j];
}
inline void reticleFmadd3(double* dst, const double* a, double c0,
                          const double* b, double c1,
                          const double* c, double c2, unsigned n) noexcept {
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
    if (hasAvx2FmaV1()) { reticleFmadd3Avx2(dst, a, c0, b, c1, c, c2, n); return; }
#endif
    for (unsigned j = 0; j < n; ++j) dst[j] += c0 * a[j] + c1 * b[j] + c2 * c[j];
}
inline void reticleAddDiff(double* dst, const double* hi, const double* lo, unsigned n) noexcept {
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
    if (hasAvx2FmaV1()) { reticleAddDiffAvx2(dst, hi, lo, n); return; }
#endif
    for (unsigned j = 0; j < n; ++j) dst[j] += hi[j] - lo[j];
}
inline void reticleFmaddDiff(double* dst, const double* hi, const double* lo, double w, unsigned n) noexcept {
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
    if (hasAvx2FmaV1()) { reticleFmaddDiffAvx2(dst, hi, lo, w, n); return; }
#endif
    for (unsigned j = 0; j < n; ++j) dst[j] += w * (hi[j] - lo[j]);
}
inline double reticleDot(const double* a, const double* b, unsigned n) noexcept {
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
    if (hasAvx2FmaV1()) return reticleDotAvx2(a, b, n);
#endif
    double s = 0.;
    for (unsigned j = 0; j < n; ++j) s += a[j] * b[j];
    return s;
}
inline void reticleDot4(const double* d, const double* b0, const double* b1,
                        const double* b2, const double* b3, unsigned n, double out[4]) noexcept {
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
    if (hasAvx2FmaV1()) { reticleDot4Avx2(d, b0, b1, b2, b3, n, out); return; }
#endif
    double s0 = 0., s1 = 0., s2 = 0., s3 = 0.;
    for (unsigned j = 0; j < n; ++j) {
        const double dv = d[j];
        s0 += dv * b0[j]; s1 += dv * b1[j]; s2 += dv * b2[j]; s3 += dv * b3[j];
    }
    out[0] = s0; out[1] = s1; out[2] = s2; out[3] = s3;
}
inline void reticleNormDot(const double* d, const double* w, unsigned n,
                           double& outNorm2, double& outCorr) noexcept {
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
    if (hasAvx2FmaV1()) { reticleNormDotAvx2(d, w, n, outNorm2, outCorr); return; }
#endif
    double norm2 = 0., corr = 0.;
    for (unsigned j = 0; j < n; ++j) { norm2 += w[j] * w[j]; corr += d[j] * w[j]; }
    outNorm2 = norm2;
    outCorr = corr;
}
inline double reticleResidualNorm2(const double* d, const double* w, double amp, unsigned n) noexcept {
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
    if (hasAvx2FmaV1()) return reticleResidualNorm2Avx2(d, w, amp, n);
#endif
    double s = 0.;
    for (unsigned j = 0; j < n; ++j) { const double r = d[j] - amp * w[j]; s += r * r; }
    return s;
}
thread_local bool gReticleNestedPoolBlocked=false;
thread_local const std::atomic<unsigned>* gReticleActiveJobs=nullptr;
thread_local unsigned gReticleWorkerBudget=0u;
// Join-before-return worker launcher: avoids Windows DLL_PROCESS_DETACH loader-lock
// deadlocks that occur when static/thread_local worker threads join during DLL unload.
// Inside World's parallel VehiclePool workers, ReticleNestedPoolGuardV1 blocks nested
// dispatch so this runs synchronously with zero thread creation overhead.
struct DirectReticleLauncherV1 {
    template<class Fn>
    void run(unsigned workers,const Fn& fn) const {
        if(workers<=1u){fn(0u);return;}
        std::vector<std::thread> pool;
        pool.reserve(workers-1u);
        for(unsigned id=1u;id<workers;++id)pool.emplace_back([&fn,id]{fn(id);});
        fn(0u);
        for(auto& t:pool)if(t.joinable())t.join();
    }
};
DirectReticleLauncherV1& threadLocalReticlePool() noexcept {
    static DirectReticleLauncherV1 launcher;
    return launcher;
}
using namespace measured;
using V=weapon::V3;
struct UnitGroundClockV2 {
    using Clock=std::chrono::steady_clock;
    ThermalGroundWorkV2* work;bool original;Clock::time_point began{};
    UnitGroundClockV2(ThermalGroundWorkV2* report,bool native):work(report),original(native){if(work)began=Clock::now();}
    ~UnitGroundClockV2(){if(work){const double us=std::chrono::duration<double,std::micro>(Clock::now()-began).count();if(original)work->originalMicros+=us;else work->totalMicros+=us;}}
};

constexpr double pi=3.14159265358979323846;
double inner(const std::vector<double>& a,const std::vector<double>& b) {
    return std::inner_product(a.begin(),a.end(),b.begin(),0.);
}
double omega(double x,double y,double pitch) {
    const auto f=[](double a,double b){return std::atan(a*b/std::sqrt(1+a*a+b*b));};
    const double x0=std::tan(x-pitch/2),x1=std::tan(x+pitch/2),y0=std::tan(y-pitch/2),y1=std::tan(y+pitch/2);
    return f(x1,y1)-f(x0,y1)-f(x1,y0)+f(x0,y0);
}
std::vector<double> filtered(std::vector<double> values,double alpha) {
    double previous=0;
    for(auto& value:values){previous+=alpha*(value-previous);value=previous;}
    return values;
}
using P4=std::array<double,4>;
// Box-constrained, column-scaled augmented least squares. Pivoted Householder
// QR avoids squaring the conditioning of nearly dependent optical columns.
// All3^N active faces are finite numerical proposals, not accepted detections.
// The4D specialization retains the original operation and face order.
template<std::size_t Dimension>
bool dampedBoxStep(const std::array<std::vector<double>,Dimension>& jacobian,const std::vector<double>& residual,
    const std::array<double,Dimension>& low,const std::array<double,Dimension>& high,double damping,std::array<double,Dimension>& result) {
    static_assert(Dimension==4||Dimension==5);
    constexpr unsigned faceCount=Dimension==4?81:243;
    const auto count=residual.size(),rows=count+Dimension;
    if(!(damping>0)||!std::isfinite(damping))return false;
    std::array<double,Dimension> norms{};for(unsigned column=0;column<Dimension;++column) {
        if(jacobian[column].size()!=count)return false;
        for(const auto value:jacobian[column])norms[column]=std::hypot(norms[column],value);
        if(!std::isfinite(norms[column]))return false;
        if(norms[column]==0)norms[column]=1;
    }
    std::vector<std::array<double,Dimension>> design(rows);std::vector<double> target(rows);
    const double regularizer=std::sqrt(damping);
    for(std::size_t row=0;row<count;++row) {
        target[row]=-residual[row];if(!std::isfinite(target[row]))return false;
        for(unsigned column=0;column<Dimension;++column)design[row][column]=jacobian[column][row]/norms[column];
    }
    for(unsigned column=0;column<Dimension;++column)design[count+column][column]=regularizer;
    std::array<double,Dimension> scaledLow{},scaledHigh{};
    for(unsigned axis=0;axis<Dimension;++axis) {
        scaledLow[axis]=low[axis]*norms[axis];scaledHigh[axis]=high[axis]*norms[axis];
        if(!std::isfinite(scaledLow[axis])||!std::isfinite(scaledHigh[axis])||scaledLow[axis]>scaledHigh[axis])return false;
    }
    // Single initial Householder QR reduces the (count + D) x D least-squares
    // system to an exact orthogonal D x D upper-triangular system R * x = cTop
    // without squaring the condition number: ||A*x - b||^2 = ||R*x - cTop||^2 + const.
    std::array<std::array<double,Dimension>,Dimension> R{};
    std::array<double,Dimension> cTop{};
    {
        auto workA=design;
        auto workB=target;
        std::vector<double> reflector(rows,0.);
        for(unsigned col=0;col<Dimension;++col) {
            double colNorm=0;
            for(std::size_t row=col;row<rows;++row)colNorm=std::hypot(colNorm,workA[row][col]);
            if(!(colNorm>0)||!std::isfinite(colNorm))return false;
            const double diagonal=-std::copysign(colNorm,workA[col][col]);
            for(std::size_t row=col;row<rows;++row)reflector[row]=workA[row][col];
            reflector[col]-=diagonal;
            double norm2=0;for(std::size_t row=col;row<rows;++row)norm2+=reflector[row]*reflector[row];
            if(!(norm2>0)||!std::isfinite(norm2))return false;
            const double beta=2/norm2;
            for(unsigned dst=col+1;dst<Dimension;++dst) {
                double d=0;for(std::size_t row=col;row<rows;++row)d+=reflector[row]*workA[row][dst];
                for(std::size_t row=col;row<rows;++row)workA[row][dst]-=beta*reflector[row]*d;
            }
            double db=0;for(std::size_t row=col;row<rows;++row)db+=reflector[row]*workB[row];
            for(std::size_t row=col;row<rows;++row)workB[row]-=beta*reflector[row]*db;
            workA[col][col]=diagonal;
            for(unsigned dst=col;dst<Dimension;++dst)R[col][dst]=workA[col][dst];
            cTop[col]=workB[col];
        }
    }
    double bestCost=std::numeric_limits<double>::infinity();std::array<double,Dimension> selected{};
    for(unsigned face=0;face<faceCount;++face) {
        unsigned encoded=face,freeCount=0;std::array<unsigned,Dimension> free{};std::array<double,Dimension> step{};
        std::array<double,Dimension> rhs=cTop;
        for(unsigned axis=0;axis<Dimension;++axis) {
            const auto state=encoded%3;encoded/=3;
            if(state==0)free[freeCount++]=axis;
            else {
                step[axis]=state==1?scaledLow[axis]:scaledHigh[axis];
                for(unsigned row=0;row<=axis;++row)rhs[row]-=R[row][axis]*step[axis];
            }
        }
        std::array<std::array<double,Dimension>,Dimension> matrix{};
        for(unsigned row=0;row<Dimension;++row)for(unsigned column=0;column<freeCount;++column)
            matrix[row][column]=R[row][free[column]];
        bool valid=true;
        if(face>0) {
            for(unsigned column=0;column<freeCount&&valid;++column) {
                unsigned pivot=column;double largest=-1;
                for(unsigned candidate=column;candidate<freeCount;++candidate) {
                    double sum2=0.,maxAbs=0.;
                    for(unsigned row=column;row<Dimension;++row) {
                        const double v=std::abs(matrix[row][candidate]);
                        if(v>maxAbs)maxAbs=v;
                        sum2+=v*v;
                    }
                    double length=0.;
                    if(maxAbs>0.) {
                        if(maxAbs>1e-150&&maxAbs<1e150)length=std::sqrt(sum2);
                        else {
                            double scaled=0.;
                            for(unsigned row=column;row<Dimension;++row){const double t=matrix[row][candidate]/maxAbs;scaled+=t*t;}
                            length=maxAbs*std::sqrt(scaled);
                        }
                    }
                    if(length>largest){largest=length;pivot=candidate;}
                }
                if(!(largest>0)||!std::isfinite(largest)){valid=false;break;}
                if(pivot!=column) {
                    std::swap(free[pivot],free[column]);
                    for(unsigned row=0;row<Dimension;++row)std::swap(matrix[row][pivot],matrix[row][column]);
                }
                const double diagonal=-std::copysign(largest,matrix[column][column]);
                std::array<double,Dimension> reflector{};
                for(unsigned row=column;row<Dimension;++row)reflector[row]=matrix[row][column];
                reflector[column]-=diagonal;
                double norm2=0;for(unsigned row=column;row<Dimension;++row)norm2+=reflector[row]*reflector[row];
                if(!(norm2>0)||!std::isfinite(norm2)){valid=false;break;}
                const double beta=2/norm2;
                for(unsigned destination=column;destination<freeCount;++destination) {
                    double d=0;for(unsigned row=column;row<Dimension;++row)d+=reflector[row]*matrix[row][destination];
                    for(unsigned row=column;row<Dimension;++row)matrix[row][destination]-=beta*reflector[row]*d;
                }
                double d=0;for(unsigned row=column;row<Dimension;++row)d+=reflector[row]*rhs[row];
                for(unsigned row=column;row<Dimension;++row)rhs[row]-=beta*reflector[row]*d;
                matrix[column][column]=diagonal;
            }
        }
        if(!valid)continue;
        std::array<double,Dimension> solved{};
        for(unsigned reverse=freeCount;reverse>0;--reverse) {
            const auto row=reverse-1;double value=rhs[row];
            for(unsigned column=row+1;column<freeCount;++column)value-=matrix[row][column]*solved[column];
            solved[row]=value/matrix[row][row];if(!std::isfinite(solved[row])){valid=false;break;}
        }
        if(!valid)continue;
        for(unsigned column=0;column<freeCount;++column)step[free[column]]=solved[column];
        for(unsigned axis=0;axis<Dimension;++axis) {
            const double tolerance=256*std::numeric_limits<double>::epsilon()*
                std::max({1.,std::abs(scaledLow[axis]),std::abs(scaledHigh[axis])});
            if(step[axis]<scaledLow[axis]-tolerance||step[axis]>scaledHigh[axis]+tolerance){valid=false;break;}
            step[axis]=std::clamp(step[axis],scaledLow[axis],scaledHigh[axis]);
        }
        if(!valid)continue;
        // The feasible unconstrained minimizer is also the unique box
        // minimizer of this strictly convex regularized least-squares problem.
        if(face==0){for(unsigned axis=0;axis<Dimension;++axis)result[axis]=step[axis]/norms[axis];return true;}
        double cost=0;
        for(unsigned row=0;row<Dimension;++row) {
            double error=-cTop[row];for(unsigned column=row;column<Dimension;++column)error+=R[row][column]*step[column];
            cost+=error*error;
        }
        if(std::isfinite(cost)&&cost<bestCost){bestCost=cost;selected=step;}
    }
    if(!std::isfinite(bestCost))return false;
    for(unsigned axis=0;axis<Dimension;++axis)result[axis]=selected[axis]/norms[axis];
    return true;
}
}

ReticleNestedPoolGuardV1::ReticleNestedPoolGuardV1(bool block) noexcept
    : previous(gReticleNestedPoolBlocked), previousActiveJobs(gReticleActiveJobs), previousBudget(gReticleWorkerBudget) {
    gReticleNestedPoolBlocked=block;
    gReticleActiveJobs=nullptr;
    gReticleWorkerBudget=0u;
}
ReticleNestedPoolGuardV1::ReticleNestedPoolGuardV1(const std::atomic<unsigned>* activeJobs,unsigned workerBudget) noexcept
    : previous(gReticleNestedPoolBlocked), previousActiveJobs(gReticleActiveJobs), previousBudget(gReticleWorkerBudget) {
    gReticleNestedPoolBlocked=(activeJobs!=nullptr);
    gReticleActiveJobs=activeJobs;
    gReticleWorkerBudget=workerBudget;
}
ReticleNestedPoolGuardV1::~ReticleNestedPoolGuardV1() noexcept {
    gReticleNestedPoolBlocked=previous;
    gReticleActiveJobs=previousActiveJobs;
    gReticleWorkerBudget=previousBudget;
}
bool ReticleNestedPoolGuardV1::blocked() noexcept {
    return gReticleNestedPoolBlocked && gReticleActiveJobs==nullptr;
}
unsigned ReticleNestedPoolGuardV1::maxWorkers(unsigned cap) noexcept {
    if(cap<=1u)return cap;
    const unsigned hw=std::max(1u,std::thread::hardware_concurrency());
    if(gReticleActiveJobs!=nullptr) {
        const unsigned active=std::max(1u,gReticleActiveJobs->load(std::memory_order_relaxed));
        const unsigned budget=std::max(hw,gReticleWorkerBudget);
        return std::min(cap,std::max(1u,budget/active));
    }
    if(gReticleNestedPoolBlocked)return 1u;
    return std::min(cap,hw);
}

double ThermalReticleReceiverV1::maskV1(const ThermalReticleConfigV1& c,double x,double y,double phase) noexcept {
    x+=c.scanRadiusRad*std::cos(phase);y+=c.scanRadiusRad*std::sin(phase);
    if(!std::isfinite(x)||!std::isfinite(y)||x*x+y*y>c.stopRadiusRad*c.stopRadiusRad)return 0;
    return std::cos(c.periods*std::atan2(y,x))>=0?1.:0.;
}
double ThermalReticleReceiverV1::angularSpeedV1(double x,double y,double radius,double phase,double frequency) noexcept {
    const double p=x*std::cos(phase)+y*std::sin(phase),denominator=x*x+y*y+2*radius*p+radius*radius;
    return denominator>0?2*pi*frequency*radius*(radius+p)/denominator:std::numeric_limits<double>::quiet_NaN();
}
std::optional<double> ThermalReticleReceiverV1::physicalBackgroundAmplitudeV1(double skyAtZero,double skyProjection,double unconstrained) noexcept {
    if(!std::isfinite(skyAtZero)||!std::isfinite(skyProjection)||!std::isfinite(unconstrained))return std::nullopt;
    double low=-std::numeric_limits<double>::infinity(),high=std::numeric_limits<double>::infinity();
    for(const double coefficient:{-skyProjection,1-skyProjection}) {
        if(coefficient==0){if(skyAtZero<0)return std::nullopt;continue;}
        const double boundary=-skyAtZero/coefficient;
        if(coefficient>0)low=std::max(low,boundary);else high=std::min(high,boundary);
    }
    if(low>high)return std::nullopt;
    double admitted=std::clamp(unconstrained,low,high);
    for(unsigned iteration=0;iteration<8;++iteration) {
        if(!std::isfinite(admitted))return std::nullopt;
        const double sky=skyAtZero-skyProjection*admitted,ground=sky+admitted;
        if(std::isfinite(sky)&&std::isfinite(ground)&&sky>=0&&ground>=0)return admitted;
        const double inside=std::isfinite(low)&&std::isfinite(high)?low/2+high/2:
            std::isfinite(low)?std::numeric_limits<double>::infinity():-std::numeric_limits<double>::infinity();
        const double next=std::nextafter(admitted,inside);
        if(next==admitted||next<low||next>high)return std::nullopt;
        admitted=next;
    }
    return std::nullopt;
}
std::optional<double> ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(const ThermalReticleConfigV1& config,
    const std::vector<std::array<double,5>>& modes) noexcept {
    if(!validConfig(config)||modes.empty())return std::nullopt;
    const auto& p=config.detector;
    const double halfX=.5*p.width*p.pixelPitchRad,halfY=.5*p.height*p.pixelPitchRad;
    const auto ray=[](const std::array<double,5>& q){return unit(V{1,std::tan(q[0]),std::tan(q[1])});};
    const auto selected=ray(modes.front());
    double lowX=-halfX,highX=halfX,lowY=-halfY,highY=halfY,envelope=0;
    const auto clearance=[&](V direction) {
        double distance=std::atan(config.stopRadiusRad+config.scanRadiusRad)-
            std::atan2(std::hypot(direction.y,direction.z),direction.x);
        for(const double sign:{-1.,1.}) {
            distance=std::min(distance,std::asin(std::clamp(std::sin(halfX)*direction.x+sign*std::cos(halfX)*direction.y,-1.,1.)));
            distance=std::min(distance,std::asin(std::clamp(std::sin(halfY)*direction.x+sign*std::cos(halfY)*direction.z,-1.,1.)));
        }
        return distance;
    };
    for(const auto& q:modes) {
        if(!std::all_of(q.begin(),q.end(),[](double value){return std::isfinite(value);})||q[4]<=0||
            q[2]<0||q[3]<0||q[2]>=halfX||q[3]>=halfY||
            q[0]-.5*q[2]<=-halfX||q[0]+.5*q[2]>=halfX||q[1]-.5*q[3]<=-halfY||q[1]+.5*q[3]>=halfY)return std::nullopt;
        const auto direction=ray(q);
        if(!(p.detectionSigma*std::sqrt(q[4])<clearance(direction)))return std::nullopt;
        lowX=std::max(lowX,q[0]-.5*q[2]);highX=std::min(highX,q[0]+.5*q[2]);
        lowY=std::max(lowY,q[1]-.5*q[3]);highY=std::min(highY,q[1]+.5*q[3]);
        if(lowX>highX||lowY>highY)return std::nullopt;
        // Outward rounding preserves the selected mode and local error even
        // when a subpixel reflected centre is only a few ulps away.
        const double displacement=std::nextafter(std::atan2(norm(cross(selected,direction)),dot(selected,direction)),
            std::numeric_limits<double>::infinity());
        const double spread=std::nextafter(std::sqrt(q[4]),std::numeric_limits<double>::infinity());
        const double total=std::nextafter(spread+displacement,std::numeric_limits<double>::infinity());
        envelope=std::max(envelope,std::nextafter(total*total,std::numeric_limits<double>::infinity()));
    }
    if(!std::isfinite(envelope)||!(p.detectionSigma*std::sqrt(envelope)<clearance(selected)))return std::nullopt;
    return envelope;
}
bool ThermalReticleReceiverV1::validConfig(const ThermalReticleConfigV1& c) noexcept {
    const auto& p=c.detector;
    return c.version==1&&ThermalImageReceiverV1::validConfig(p)&&p.width<=128&&p.height<=128&&
        c.periods>=3&&c.periods<=64&&c.samplesPerScan>=8*c.periods&&c.samplesPerScan<=512&&
        c.spatialSamples>=2&&c.spatialSamples<=8&&std::isfinite(c.scanRadiusRad)&&c.scanRadiusRad>0&&
        std::isfinite(c.stopRadiusRad)&&c.stopRadiusRad>c.scanRadiusRad&&c.stopRadiusRad<=.5*std::min(p.width,p.height)*p.pixelPitchRad&&
        std::isfinite(c.frequencyHz)&&c.frequencyHz>0&&c.frequencyHz<=2000&&
        p.integrationTimeS*c.frequencyHz*c.samplesPerScan<=1&&
        static_cast<std::size_t>(c.samplesPerScan)*p.exposureSamples*p.width*p.height<=4194304;
}
void ThermalReticleReceiverV1::project(std::vector<double>& values) const {
    if(!precomputed_)return;
    for(const auto& column:precomputed_->nuisance) {
        const double coefficient=inner(values,column);
        for(std::size_t i=0;i<values.size();++i)values[i]-=coefficient*column[i];
    }
}
namespace {
// L1-resident cubic Hermite table for P(z) = z * Phi(z) + phi(z) on [-8, 8]
// with h = 1/128 (2049 nodes = 32 KB). Max error <= h^4 / 384 * phi(0) < 4e-12.
struct GaussianPrimitiveHermiteTableV1 {
    static constexpr double kLo = -8.0;
    static constexpr double kHi = 8.0;
    static constexpr double kInvH = 128.0;
    static constexpr double kH = 1.0 / kInvH;
    static constexpr unsigned kNodes = static_cast<unsigned>((kHi - kLo) * kInvH) + 1u;
    struct Node { double p = 0., d = 0.; };
    std::array<Node, kNodes> nodes{};
    GaussianPrimitiveHermiteTableV1() {
        const double invSqrt2 = 1.0 / std::sqrt(2.0);
        const double invSqrt2Pi = 1.0 / std::sqrt(2.0 * pi);
        for (unsigned i = 0; i < kNodes; ++i) {
            const double z = kLo + static_cast<double>(i) * kH;
            const double phiCdf = 0.5 * std::erfc(-z * invSqrt2);
            const double pdf = std::exp(-0.5 * z * z) * invSqrt2Pi;
            nodes[i].p = z * phiCdf + pdf;
            nodes[i].d = phiCdf * kH;
        }
    }
    double eval(double z) const noexcept {
        if (z <= kLo) return 0.0;
        if (z >= kHi) return z;
        const double u = (z - kLo) * kInvH;
        const unsigned i = static_cast<unsigned>(u);
        const double t = u - static_cast<double>(i);
        const double p0 = nodes[i].p, d0 = nodes[i].d;
        const double p1 = nodes[i + 1u].p, d1 = nodes[i + 1u].d;
        const double a2 = 3.0 * (p1 - p0) - 2.0 * d0 - d1;
        const double a3 = 2.0 * (p0 - p1) + d0 + d1;
        return ((a3 * t + a2) * t + d0) * t + p0;
    }
};
const GaussianPrimitiveHermiteTableV1& gaussianPrimitiveTableV1() noexcept {
    static const GaussianPrimitiveHermiteTableV1 table;
    return table;
}
inline double gaussianIntervalFastV1(double low,double high,double pixelLow,double pixelHigh,double sigma) noexcept {
    const double invSigma=1.0/sigma;
    const auto& tbl=gaussianPrimitiveTableV1();
    const double hiVal=sigma*(tbl.eval((pixelHigh-low)*invSigma)-tbl.eval((pixelHigh-high)*invSigma));
    const double loVal=sigma*(tbl.eval((pixelLow-low)*invSigma)-tbl.eval((pixelLow-high)*invSigma));
    return std::clamp(hiVal-loVal,0.,std::min(high-low,pixelHigh-pixelLow));
}
} // namespace

std::vector<double> ThermalReticleReceiverV1::spot(double x,double y,double sigma,double width,double height,bool removeNuisance) const {
    const auto& c=config_;const auto& p=c.detector;
    const unsigned count=c.samplesPerScan;
    std::vector<double> result(count,0);
    if(!precomputed_)return result;
    const auto& pre=*precomputed_;
    const double invRoot2Sigma=1./(std::sqrt(2.)*sigma);
    const double interiorMargin=9.5*sigma;
    const auto primitive=[&](double edge,double centre) noexcept {return .5*std::erfc((centre-edge)*invRoot2Sigma);};
    const auto probability=[&](double low,double high,double centre,double extent) noexcept {
        if(extent==0)return primitive(high,centre)-primitive(low,centre);
        const double left=centre-.5*extent,right=centre+.5*extent;
        // Beyond 9.5*sigma inside [left, right], 1 - 2*Q(9.5) == 1.0 in binary64,
        // so returning (high-low)/extent avoids 4*erfc+4*exp AND ensures exact
        // bit-equality across interior pixels for 1D/2D prefix-sum collapse.
        if(low>=left+interiorMargin&&high<=right-interiorMargin)return (high-low)/extent;
        return gaussianIntervalFastV1(left,right,low,high,sigma)/extent;
    };
    const double pitch=p.pixelPitchRad,halfH=.5*p.height,halfW=.5*p.width,tail=9*sigma;
    const double yLoBound=y-.5*height-tail,yHiBound=y+.5*height+tail;
    const double xLoBound=x-.5*width-tail,xHiBound=x+.5*width+tail;
    // Explicit nine-sigma numerical support, matching the retained optical
    // renderer's tail bound; lost FOV energy is never renormalized.
    std::array<double,128> xProbabilities{};
    unsigned colLo=p.width,colHi=0;
    const bool useProjected=removeNuisance&&!pre.projWeightedWeights.empty();
    const double* wwBase=useProjected?pre.projWeightedWeights.data():pre.weightedWeights.data();
    double* resPtr=result.data();
    std::array<double,128> yProbabilities{};
    unsigned rowLo=p.height,rowHi=0;
    for(unsigned row=0;row<p.height;++row) {
        const double lowY=(row-halfH)*pitch,highY=lowY+pitch;
        if(lowY>yHiBound||highY<yLoBound)continue;
        const double wy=probability(lowY,highY,y,height);
        if(wy==0.)continue;
        yProbabilities[row]=wy;
        if(row<rowLo)rowLo=row;
        rowHi=row+1u;
    }
    if(rowLo<rowHi) {
        for(unsigned col=0;col<p.width;++col) {
            const double lowX=(col-halfW)*pitch,highX=lowX+pitch;
            if(lowX>xHiBound||highX<xLoBound)continue;
            const double wx=probability(lowX,highX,x,width);
            if(wx==0.)continue;
            xProbabilities[col]=wx;
            if(col<colLo)colLo=col;
            colHi=col+1u;
        }
    }
    if(rowLo<rowHi&&colLo<colHi) {
        if(useProjected&&!pre.projPrefix2D.empty()&&!pre.projRowPrefix.empty()) {
            const std::size_t pCols=static_cast<std::size_t>(p.width)+1u;
            const double* p2d=pre.projPrefix2D.data();
            const double* rp=pre.projRowPrefix.data();
            // Collapse contiguous equal-probability row and column runs (exact ==,
            // e.g. wide-rectangle interiors where wx == pitch/width, wy == pitch/height)
            // into O(count) 2D / 1D prefix-sum queries instead of O(W*H*count).
            for(unsigned row=rowLo;row<rowHi;) {
                const double wy=yProbabilities[row];
                if(wy==0.){++row;continue;}
                unsigned rEnd=row+1u;
                while(rEnd<rowHi&&yProbabilities[rEnd]==wy)++rEnd;
                for(unsigned col=colLo;col<colHi;) {
                    const double wx=xProbabilities[col];
                    if(wx==0.){++col;continue;}
                    unsigned cEnd=col+1u;
                    while(cEnd<colHi&&xProbabilities[cEnd]==wx)++cEnd;
                    const double weight=wy*wx;
                    if(rEnd-row>=2u&&cEnd-col>=2u) {
                        const double* a00=p2d+(static_cast<std::size_t>(row)*pCols+col)*count;
                        const double* a01=p2d+(static_cast<std::size_t>(row)*pCols+cEnd)*count;
                        const double* a10=p2d+(static_cast<std::size_t>(rEnd)*pCols+col)*count;
                        const double* a11=p2d+(static_cast<std::size_t>(rEnd)*pCols+cEnd)*count;
                        for(unsigned j=0;j<count;++j)resPtr[j]+=weight*((a11[j]-a01[j])-(a10[j]-a00[j]));
                    } else if(cEnd-col>=2u) {
                        for(unsigned r=row;r<rEnd;++r) {
                            const double* s0=rp+(static_cast<std::size_t>(r)*pCols+col)*count;
                            const double* s1=rp+(static_cast<std::size_t>(r)*pCols+cEnd)*count;
                            for(unsigned j=0;j<count;++j)resPtr[j]+=weight*(s1[j]-s0[j]);
                        }
                    } else if(rEnd-row>=2u&&!pre.projColPrefix.empty()) {
                        const double* cp=pre.projColPrefix.data();
                        for(unsigned cIdx=col;cIdx<cEnd;++cIdx) {
                            const double* s0=cp+(static_cast<std::size_t>(row)*p.width+cIdx)*count;
                            const double* s1=cp+(static_cast<std::size_t>(rEnd)*p.width+cIdx)*count;
                            for(unsigned j=0;j<count;++j)resPtr[j]+=weight*(s1[j]-s0[j]);
                        }
                    } else {
                        for(unsigned r=row;r<rEnd;++r) {
                            const std::size_t rowOffset=static_cast<std::size_t>(r)*p.width;
                            for(unsigned cIdx=col;cIdx<cEnd;++cIdx) {
                                const double* wRow=wwBase+(rowOffset+cIdx)*count;
                                for(unsigned j=0;j<count;++j)resPtr[j]+=weight*wRow[j];
                            }
                        }
                    }
                    col=cEnd;
                }
                row=rEnd;
            }
        } else {
            for(unsigned row=rowLo;row<rowHi;++row) {
                const double wy=yProbabilities[row];
                if(wy==0.)continue;
                const std::size_t rowOffset=static_cast<std::size_t>(row)*p.width;
                for(unsigned col=colLo;col<colHi;++col) {
                    const double wx=xProbabilities[col];
                    if(wx==0.)continue;
                    const double weight=wy*wx;
                    const double* wRow=wwBase+(rowOffset+col)*count;
                    for(unsigned j=0;j<count;++j)resPtr[j]+=weight*wRow[j];
                }
            }
        }
    }
    if(!useProjected) {
        const double alpha=-std::expm1(-1/(c.frequencyHz*count*p.pixelResponseTimeS));
        result=filtered(std::move(result),alpha);if(removeNuisance)project(result);
    }
    return result;
}
std::array<std::vector<double>,4> ThermalReticleReceiverV1::spotJacobianV1(double x,double y,double width,double height) const {
    const auto& c=config_;const auto& p=c.detector;
    const unsigned count=c.samplesPerScan;
    const auto& pre=*precomputed_;
    // Derivatives are with respect to centreX, centreY, width^2, height^2.
    // Squared extent has a finite nonzero Gaussian second-derivative limit
    // at a point source; a width derivative would falsely stick at zero.
    struct Axis {double probability,centre,squaredExtent;};
    const double sigma=p.psfSigmaRad,invSigma=1./sigma,invRoot2Sigma=1./(std::sqrt(2.)*sigma),invRoot2Pi=1./std::sqrt(2*pi);
    const double tinyExtent=sigma*std::pow(std::numeric_limits<double>::epsilon(),.25);
    const auto axis=[&](double low,double high,double centre,double extent) {
        const auto density=[&](double z) noexcept {return std::exp(-.5*z*z)*invRoot2Pi;};
        const auto at=[&](double location) noexcept {
            return .5*std::erfc((location-high)*invRoot2Sigma)-.5*std::erfc((location-low)*invRoot2Sigma);
        };
        const double a=(low-centre)*invSigma,b=(high-centre)*invSigma;
        const double pointCentre=(density(a)-density(b))*invSigma;
        const double pointSquared=(a*density(a)-b*density(b))*(invSigma*invSigma/24.);
        if(extent==0)return Axis{at(centre),pointCentre,pointSquared};
        const double left=centre-.5*extent,right=centre+.5*extent;
        if(extent>=tinyExtent&&low>=left+9.5*sigma&&high<=right-9.5*sigma) {
            const double span=high-low;
            return Axis{span/extent,0.,-span/(2*extent*extent*extent)};
        }
        const double integral=gaussianIntervalFastV1(left,right,low,high,sigma);
        const double probability=integral/extent;
        if(extent<tinyExtent)return Axis{probability,pointCentre,pointSquared};
        const double pl=at(left),pr=at(right);
        return Axis{probability,(pr-pl)/extent,(.5*(pr+pl)*extent-integral)/(2*extent*extent*extent)};
    };
    std::array<Axis,128> xs{},ys{};
    const double pitch=p.pixelPitchRad,halfW=.5*p.width,halfH=.5*p.height,tail=10*sigma;
    const double xLoBound=x-.5*width-tail,xHiBound=x+.5*width+tail;
    const double yLoBound=y-.5*height-tail,yHiBound=y+.5*height+tail;
    std::array<unsigned,128> activeCols{};unsigned activeColCount=0;
    for(unsigned col=0;col<p.width;++col) {
        const double lowX=(col-halfW)*pitch,highX=lowX+pitch;
        if(lowX>xHiBound||highX<xLoBound)continue;
        xs[col]=axis(lowX,highX,x,width);
        if(xs[col].probability!=0||xs[col].centre!=0||xs[col].squaredExtent!=0)activeCols[activeColCount++]=col;
    }
    std::array<unsigned,128> activeRows{};unsigned activeRowCount=0;
    for(unsigned row=0;row<p.height;++row) {
        const double lowY=(row-halfH)*pitch,highY=lowY+pitch;
        if(lowY>yHiBound||highY<yLoBound)continue;
        ys[row]=axis(lowY,highY,y,height);
        if(ys[row].probability!=0||ys[row].centre!=0||ys[row].squaredExtent!=0)activeRows[activeRowCount++]=row;
    }
    std::array<std::vector<double>,4> derivative;
    for(auto& values:derivative)values.assign(count,0);
    double* d0=derivative[0].data();double* d1=derivative[1].data();
    double* d2=derivative[2].data();double* d3=derivative[3].data();
    const bool useProjected=!pre.projWeightedWeights.empty();
    const double* wwBase=useProjected?pre.projWeightedWeights.data():pre.weightedWeights.data();
    // Separable contraction along columns per active row using 1D/2D prefix sums
    // on equal-weight interior runs: reduces wide-rectangle Jacobian work from
    // O(W*H*count) to O((W_edge + H_edge)*count) with zero approximation.
    const bool hasRowPref=useProjected&&!pre.projRowPrefix.empty();
    const std::size_t pCols=static_cast<std::size_t>(p.width)+1u;
    const double* rpBase=hasRowPref?pre.projRowPrefix.data():nullptr;
    std::array<double,512> rowProb{},rowXCentre{},rowXSq{};
    for(unsigned ri=0;ri<activeRowCount;++ri) {
        const unsigned row=activeRows[ri];
        const auto& by=ys[row];
        std::fill_n(rowProb.data(),count,0.);
        std::fill_n(rowXCentre.data(),count,0.);
        std::fill_n(rowXSq.data(),count,0.);
        const std::size_t rowOffset=static_cast<std::size_t>(row)*p.width;
        const bool needProb=(by.centre!=0||by.squaredExtent!=0);
        const bool needX=(by.probability!=0);
        for(unsigned ci=0;ci<activeColCount;) {
            const unsigned col=activeCols[ci];
            const auto& ax=xs[col];
            if(hasRowPref&&ax.centre==0.) {
                unsigned ciEnd=ci+1u;
                while(ciEnd<activeColCount&&activeCols[ciEnd]==col+(ciEnd-ci)&&
                      xs[activeCols[ciEnd]].centre==0.&&
                      xs[activeCols[ciEnd]].probability==ax.probability&&
                      xs[activeCols[ciEnd]].squaredExtent==ax.squaredExtent)++ciEnd;
                if(ciEnd-ci>=2u) {
                    const unsigned cEnd=col+(ciEnd-ci);
                    const double* s0=rpBase+(static_cast<std::size_t>(row)*pCols+col)*count;
                    const double* s1=rpBase+(static_cast<std::size_t>(row)*pCols+cEnd)*count;
                    if(needProb&&ax.probability!=0.) {
                        const double ap=ax.probability;
                        for(unsigned j=0;j<count;++j)rowProb[j]+=ap*(s1[j]-s0[j]);
                    }
                    if(needX&&ax.squaredExtent!=0.) {
                        const double as=ax.squaredExtent;
                        for(unsigned j=0;j<count;++j)rowXSq[j]+=as*(s1[j]-s0[j]);
                    }
                    ci=ciEnd;
                    continue;
                }
            }
            const double* wRow=wwBase+(rowOffset+col)*count;
            if(needProb&&ax.probability!=0) {
                const double ap=ax.probability;
                for(unsigned j=0;j<count;++j)rowProb[j]+=ap*wRow[j];
            }
            if(needX) {
                if(ax.centre!=0) {
                    const double ac=ax.centre;
                    for(unsigned j=0;j<count;++j)rowXCentre[j]+=ac*wRow[j];
                }
                if(ax.squaredExtent!=0) {
                    const double as=ax.squaredExtent;
                    for(unsigned j=0;j<count;++j)rowXSq[j]+=as*wRow[j];
                }
            }
            ++ci;
        }
        if(needX) {
            const double bp=by.probability;
            for(unsigned j=0;j<count;++j){d0[j]+=bp*rowXCentre[j];d2[j]+=bp*rowXSq[j];}
        }
        if(by.centre!=0) {
            const double bc=by.centre;
            for(unsigned j=0;j<count;++j)d1[j]+=bc*rowProb[j];
        }
        if(by.squaredExtent!=0) {
            const double bs=by.squaredExtent;
            for(unsigned j=0;j<count;++j)d3[j]+=bs*rowProb[j];
        }
    }
    if(!useProjected) {
        const double alpha=-std::expm1(-1/(c.frequencyHz*count*p.pixelResponseTimeS));
        for(auto& values:derivative){values=filtered(std::move(values),alpha);project(values);}
    }
    return derivative;
}
namespace {
bool reticlePrecomputeKeyEqual(const ThermalReticleConfigV1& a,const ThermalReticleConfigV1& b) noexcept {
    auto eqD=[](double x,double y) noexcept {
        std::uint64_t bx=0,by=0;std::memcpy(&bx,&x,sizeof(bx));std::memcpy(&by,&y,sizeof(by));return bx==by;
    };
    return a.version==b.version&&a.periods==b.periods&&a.samplesPerScan==b.samplesPerScan&&a.spatialSamples==b.spatialSamples&&
        eqD(a.scanRadiusRad,b.scanRadiusRad)&&eqD(a.stopRadiusRad,b.stopRadiusRad)&&eqD(a.frequencyHz,b.frequencyHz)&&
        a.detector.width==b.detector.width&&a.detector.height==b.detector.height&&a.detector.exposureSamples==b.detector.exposureSamples&&
        eqD(a.detector.pixelPitchRad,b.detector.pixelPitchRad)&&eqD(a.detector.psfSigmaRad,b.detector.psfSigmaRad)&&
        eqD(a.detector.integrationTimeS,b.detector.integrationTimeS)&&eqD(a.detector.pixelResponseTimeS,b.detector.pixelResponseTimeS);
}
}
bool ThermalReticleReceiverV1::configure(const ThermalReticleConfigV1& c,std::string& error) {
    if(!validConfig(c)){error="invalid finite conical-scan reticle policy";return false;}
    struct CacheEntry {ThermalReticleConfigV1 key;std::shared_ptr<const PrecomputedV1> value;};
    static std::mutex cacheMutex;
    static std::vector<CacheEntry> cache;
    {
        std::lock_guard<std::mutex> lock(cacheMutex);
        for(const auto& item:cache) {
            if(reticlePrecomputeKeyEqual(item.key,c)) {
                ThermalReticleReceiverV1 next;
                next.config_=c;
                next.precomputed_=item.value;
                *this=std::move(next);
                return true;
            }
        }
    }
    // Transactional configuration: no rejected policy mutates an active receiver.
    auto built=std::make_shared<PrecomputedV1>();
    const auto& p=c.detector;const auto pixels=static_cast<std::size_t>(p.width)*p.height;
    const double pixelArea=p.pixelPitchRad*p.pixelPitchRad;
    built->weights.assign(static_cast<std::size_t>(c.samplesPerScan)*pixels,0);
    built->timeWeights.resize(built->weights.size()*p.exposureSamples);
    built->solidAngles.resize(pixels);built->flat.assign(c.samplesPerScan,0);
    built->weightedWeights.assign(built->weights.size(),0);
    const unsigned symCols=(p.width+1u)/2u;
    built->symWeightedWeights.assign(static_cast<std::size_t>(p.height)*symCols*c.samplesPerScan,0);
    built->rowFullWave.assign(static_cast<std::size_t>(p.height)*c.samplesPerScan,0);
    for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
        const auto index=static_cast<std::size_t>(row)*p.width+col;
        const double x=(col+.5-.5*p.width)*p.pixelPitchRad,y=(row+.5-.5*p.height)*p.pixelPitchRad;
        built->solidAngles[index]=omega(x,y,p.pixelPitchRad);
        const double omegaScale=built->solidAngles[index]/pixelArea;
        for(unsigned j=0;j<c.samplesPerScan;++j)for(unsigned k=0;k<p.exposureSamples;++k) {
            const double phase=2*pi*((j+.5)/c.samplesPerScan+c.frequencyHz*p.integrationTimeS*((k+.5)/p.exposureSamples-.5));
            double transmission=0;
            for(unsigned sy=0;sy<c.spatialSamples;++sy)for(unsigned sx=0;sx<c.spatialSamples;++sx)
                transmission+=maskV1(c,x+((sx+.5)/c.spatialSamples-.5)*p.pixelPitchRad,
                    y+((sy+.5)/c.spatialSamples-.5)*p.pixelPitchRad,phase);
            transmission/=static_cast<double>(c.spatialSamples)*c.spatialSamples;
            built->timeWeights[(static_cast<std::size_t>(j)*p.exposureSamples+k)*pixels+index]=transmission;
            built->weights[index*c.samplesPerScan+j]+=transmission/p.exposureSamples;
        }
        double* ww=built->weightedWeights.data()+index*c.samplesPerScan;
        const unsigned symCol=(col<symCols)?col:(p.width-1u-col);
        double* sww=built->symWeightedWeights.data()+(static_cast<std::size_t>(row)*symCols+symCol)*c.samplesPerScan;
        double* rw=built->rowFullWave.data()+static_cast<std::size_t>(row)*c.samplesPerScan;
        for(unsigned j=0;j<c.samplesPerScan;++j) {
            const double w=built->weights[index*c.samplesPerScan+j];
            built->flat[j]+=built->solidAngles[index]*w;
            const double weighted=omegaScale*w;
            ww[j]=weighted;
            sww[j]+=weighted;
            rw[j]+=weighted;
        }
    }
    built->rowFullWavePrefix.assign(static_cast<std::size_t>(p.height+1)*c.samplesPerScan,0);
    for(unsigned row=0;row<p.height;++row) {
        const double* src=built->rowFullWave.data()+static_cast<std::size_t>(row)*c.samplesPerScan;
        const double* prev=built->rowFullWavePrefix.data()+static_cast<std::size_t>(row)*c.samplesPerScan;
        double* dst=built->rowFullWavePrefix.data()+static_cast<std::size_t>(row+1)*c.samplesPerScan;
        for(unsigned j=0;j<c.samplesPerScan;++j)dst[j]=prev[j]+src[j];
    }
    const std::size_t pCols=static_cast<std::size_t>(p.width)+1u;
    built->rowColPrefix.assign(static_cast<std::size_t>(p.height)*pCols*c.samplesPerScan,0);
    for(unsigned m=0;m<5;++m)built->rowMomentWave[m].assign(static_cast<std::size_t>(p.height)*c.samplesPerScan,0);
    const double invHalfW=2.0/static_cast<double>(p.width);
    for(unsigned row=0;row<p.height;++row) {
        const std::size_t rowOff=static_cast<std::size_t>(row)*p.width;
        const std::size_t prefRowOff=static_cast<std::size_t>(row)*pCols;
        double* m0=built->rowMomentWave[0].data()+static_cast<std::size_t>(row)*c.samplesPerScan;
        double* m1=built->rowMomentWave[1].data()+static_cast<std::size_t>(row)*c.samplesPerScan;
        double* m2=built->rowMomentWave[2].data()+static_cast<std::size_t>(row)*c.samplesPerScan;
        double* m3=built->rowMomentWave[3].data()+static_cast<std::size_t>(row)*c.samplesPerScan;
        double* m4=built->rowMomentWave[4].data()+static_cast<std::size_t>(row)*c.samplesPerScan;
        for(unsigned col=0;col<p.width;++col) {
            const double* src=built->weightedWeights.data()+(rowOff+col)*c.samplesPerScan;
            const double* prev=built->rowColPrefix.data()+(prefRowOff+col)*c.samplesPerScan;
            double* dst=built->rowColPrefix.data()+(prefRowOff+col+1u)*c.samplesPerScan;
            const double u=(static_cast<double>(col)+0.5-0.5*static_cast<double>(p.width))*invHalfW;
            const double u2=u*u,u3=u2*u,u4=u2*u2;
            for(unsigned j=0;j<c.samplesPerScan;++j) {
                const double v=src[j];
                dst[j]=prev[j]+v;
                m0[j]+=v;m1[j]+=u*v;m2[j]+=u2*v;m3[j]+=u3*v;m4[j]+=u4*v;
            }
        }
    }
    const double alpha=-std::expm1(-1/(c.frequencyHz*c.samplesPerScan*p.pixelResponseTimeS));
    const auto addNuisance=[&](std::vector<double> values) {
        for(const auto& column:built->nuisance) {
            const double coefficient=inner(values,column);
            for(std::size_t i=0;i<values.size();++i)values[i]-=coefficient*column[i];
        }
        const double length=std::sqrt(inner(values,values));
        if(length>128*std::numeric_limits<double>::epsilon()*std::sqrt(static_cast<double>(values.size()))) {
            for(auto& v:values)v/=length;
            built->nuisance.push_back(std::move(values));
        }
    };
    addNuisance(std::vector<double>(c.samplesPerScan,1));
    std::vector<double> transient(c.samplesPerScan);
    for(unsigned j=0;j<c.samplesPerScan;++j)transient[j]=std::pow(1-alpha,j+1);
    addNuisance(std::move(transient));addNuisance(filtered(built->flat,alpha));
    for(unsigned harmonic=1;harmonic<=2;++harmonic)for(unsigned quadrature=0;quadrature<2;++quadrature) {
        std::vector<double> basis(c.samplesPerScan);
        for(unsigned j=0;j<c.samplesPerScan;++j) {
            const double phase=2*pi*(j+.5)/c.samplesPerScan*harmonic;
            basis[j]=quadrature?std::sin(phase):std::cos(phase);
        }
        addNuisance(filtered(std::move(basis),alpha));
    }
    // Pre-apply the linear filter+nuisance-projection operator to every pixel's
    // weightedWeights once per shared configuration so spot evaluation in fit()
    // only performs a single bounded contraction with zero per-eval filtering.
    built->projWeightedWeights.resize(built->weightedWeights.size());
    std::vector<double> colTmp(c.samplesPerScan);
    for(std::size_t idx=0;idx<pixels;++idx) {
        const double* src=built->weightedWeights.data()+idx*c.samplesPerScan;
        double prev=0;
        for(unsigned j=0;j<c.samplesPerScan;++j){prev+=alpha*(src[j]-prev);colTmp[j]=prev;}
        for(const auto& column:built->nuisance) {
            const double coeff=inner(colTmp,column);
            for(unsigned j=0;j<c.samplesPerScan;++j)colTmp[j]-=coeff*column[j];
        }
        std::copy(colTmp.begin(),colTmp.end(),built->projWeightedWeights.data()+idx*c.samplesPerScan);
    }
    // Pre-apply the linear filter+nuisance-projection operator to rowMomentWave
    // and precompute scalar projections onto uniform/transient so fitHorizon()
    // null search runs with zero heap allocations, zero filtering and zero projection.
    {
        auto unifVec=filtered(built->flat,alpha);for(auto& v:unifVec)v/=pixelArea;
        std::vector<double> tranVec(c.samplesPerScan);
        for(unsigned j=0;j<c.samplesPerScan;++j)tranVec[j]=std::pow(1-alpha,j+1);
        for(unsigned m=0;m<5;++m) {
            built->projRowMomentWave[m].assign(static_cast<std::size_t>(p.height)*c.samplesPerScan,0.);
            built->rowMomentUniformDot[m].assign(p.height,0.);
            built->rowMomentTransientDot[m].assign(p.height,0.);
            for(unsigned row=0;row<p.height;++row) {
                const double* src=built->rowMomentWave[m].data()+static_cast<std::size_t>(row)*c.samplesPerScan;
                double prev=0.;
                for(unsigned j=0;j<c.samplesPerScan;++j){prev+=alpha*(src[j]-prev);colTmp[j]=prev;}
                built->rowMomentUniformDot[m][row]=inner(colTmp,unifVec);
                built->rowMomentTransientDot[m][row]=inner(colTmp,tranVec);
                for(const auto& column:built->nuisance) {
                    const double coeff=inner(colTmp,column);
                    for(unsigned j=0;j<c.samplesPerScan;++j)colTmp[j]-=coeff*column[j];
                }
                std::copy(colTmp.begin(),colTmp.end(),
                    built->projRowMomentWave[m].data()+static_cast<std::size_t>(row)*c.samplesPerScan);
            }
        }
        built->projRowFullWavePrefix.assign(static_cast<std::size_t>(p.height+1u)*c.samplesPerScan,0.);
        built->rowFullUniformDotPrefix.assign(p.height+1u,0.);
        built->rowFullTransientDotPrefix.assign(p.height+1u,0.);
        for(unsigned row=0;row<p.height;++row) {
            const double* src=built->projRowMomentWave[0].data()+static_cast<std::size_t>(row)*c.samplesPerScan;
            const double* prev=built->projRowFullWavePrefix.data()+static_cast<std::size_t>(row)*c.samplesPerScan;
            double* dst=built->projRowFullWavePrefix.data()+static_cast<std::size_t>(row+1u)*c.samplesPerScan;
            for(unsigned j=0;j<c.samplesPerScan;++j)dst[j]=prev[j]+src[j];
            built->rowFullUniformDotPrefix[row+1u]=built->rowFullUniformDotPrefix[row]+built->rowMomentUniformDot[0][row];
            built->rowFullTransientDotPrefix[row+1u]=built->rowFullTransientDotPrefix[row]+built->rowMomentTransientDot[0][row];
        }
        const std::size_t symTotal=static_cast<std::size_t>(p.height)*symCols;
        built->projSymWeightedWeights.assign(symTotal*c.samplesPerScan,0.);
        built->symUniformDot.assign(symTotal,0.);
        built->symTransientDot.assign(symTotal,0.);
        for(std::size_t sIdx=0;sIdx<symTotal;++sIdx) {
            const double* src=built->symWeightedWeights.data()+sIdx*c.samplesPerScan;
            double prev=0.;
            for(unsigned j=0;j<c.samplesPerScan;++j){prev+=alpha*(src[j]-prev);colTmp[j]=prev;}
            built->symUniformDot[sIdx]=inner(colTmp,unifVec);
            built->symTransientDot[sIdx]=inner(colTmp,tranVec);
            for(const auto& column:built->nuisance) {
                const double coeff=inner(colTmp,column);
                for(unsigned j=0;j<c.samplesPerScan;++j)colTmp[j]-=coeff*column[j];
            }
            std::copy(colTmp.begin(),colTmp.end(),built->projSymWeightedWeights.data()+sIdx*c.samplesPerScan);
        }
    }
    // Precompute 1D row/col and 2D prefix sums over projWeightedWeights so wide
    // rectangles in spot(), spotJacobianV1(), prepareRowVec() and prepareColVec()
    // contract constant interior runs in O(samplesPerScan) instead of O(W*H*samplesPerScan).
    built->projRowPrefix.assign(static_cast<std::size_t>(p.height)*pCols*c.samplesPerScan,0);
    built->projColPrefix.assign(static_cast<std::size_t>(p.height+1u)*p.width*c.samplesPerScan,0);
    built->projPrefix2D.assign(static_cast<std::size_t>(p.height+1u)*pCols*c.samplesPerScan,0);
    for(unsigned row=0;row<p.height;++row) {
        const std::size_t rowOff=static_cast<std::size_t>(row)*p.width;
        const std::size_t prefRowOff=static_cast<std::size_t>(row)*pCols;
        for(unsigned col=0;col<p.width;++col) {
            const double* src=built->projWeightedWeights.data()+(rowOff+col)*c.samplesPerScan;
            const double* rpPrev=built->projRowPrefix.data()+(prefRowOff+col)*c.samplesPerScan;
            double* rpDst=built->projRowPrefix.data()+(prefRowOff+col+1u)*c.samplesPerScan;
            const double* cpPrev=built->projColPrefix.data()+(static_cast<std::size_t>(row)*p.width+col)*c.samplesPerScan;
            double* cpDst=built->projColPrefix.data()+(static_cast<std::size_t>(row+1u)*p.width+col)*c.samplesPerScan;
            const double* p2Up=built->projPrefix2D.data()+(static_cast<std::size_t>(row)*pCols+col+1u)*c.samplesPerScan;
            double* p2Dst=built->projPrefix2D.data()+(static_cast<std::size_t>(row+1u)*pCols+col+1u)*c.samplesPerScan;
            for(unsigned j=0;j<c.samplesPerScan;++j) {
                const double v=src[j];
                const double rp=rpPrev[j]+v;
                rpDst[j]=rp;
                cpDst[j]=cpPrev[j]+v;
                p2Dst[j]=p2Up[j]+rp;
            }
        }
    }
    ThermalReticleReceiverV1 next;
    next.config_=c;
    next.precomputed_=built;
    built->templates.reserve(pixels*c.samplesPerScan);
    built->templateInvNorm2.resize(pixels,0.);
    const double minTmplNorm2=128*std::numeric_limits<double>::epsilon();
    for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
        const auto values=next.spot((col+.5-.5*p.width)*p.pixelPitchRad,(row+.5-.5*p.height)*p.pixelPitchRad,p.psfSigmaRad);
        const double n2=inner(values,values);
        built->templateInvNorm2[static_cast<std::size_t>(row)*p.width+col]=(n2>minTmplNorm2)?(1.0/n2):0.;
        built->templates.insert(built->templates.end(),values.begin(),values.end());
    }
    // Linearity of optics, electronics and nuisance projection permits exact
    // unions of disjoint source cells. This bank only proposes inverse-fit
    // starts; every selected rectangle is re-evaluated by spot() below.
    // Include the same nine-sigma off-field support as the native renderer.
    const auto spacing=std::max(1u,static_cast<unsigned>(std::ceil(2*p.psfSigmaRad/p.pixelPitchRad)));
    built->supportPitch=spacing*p.pixelPitchRad;
    built->supportMargin=static_cast<unsigned>(std::ceil(9*p.psfSigmaRad/built->supportPitch));
    built->supportColumns=static_cast<unsigned>(std::ceil(static_cast<double>(p.width)/spacing))+2*built->supportMargin;
    built->supportRows=static_cast<unsigned>(std::ceil(static_cast<double>(p.height)/spacing))+2*built->supportMargin;
    const auto stride=static_cast<std::size_t>(built->supportColumns+1)*c.samplesPerScan;
    built->supportPrefix.assign(static_cast<std::size_t>(built->supportRows+1)*stride,0.);
    for(unsigned row=0;row<built->supportRows;++row)for(unsigned col=0;col<built->supportColumns;++col) {
        const auto values=next.spot((col+.5-built->supportMargin)*built->supportPitch-.5*p.width*p.pixelPitchRad,
            (row+.5-built->supportMargin)*built->supportPitch-.5*p.height*p.pixelPitchRad,
            p.psfSigmaRad,built->supportPitch,built->supportPitch);
        const auto here=static_cast<std::size_t>(row+1)*stride+static_cast<std::size_t>(col+1)*c.samplesPerScan;
        for(unsigned j=0;j<c.samplesPerScan;++j)
            built->supportPrefix[here+j]=values[j]+built->supportPrefix[here-stride+j]+
                built->supportPrefix[here-c.samplesPerScan+j]-built->supportPrefix[here-stride-c.samplesPerScan+j];
    }
    std::shared_ptr<const PrecomputedV1> frozen=std::move(built);
    {
        std::lock_guard<std::mutex> lock(cacheMutex);
        bool exists=false;
        for(const auto& item:cache)if(reticlePrecomputeKeyEqual(item.key,c)){frozen=item.value;exists=true;break;}
        if(!exists&&cache.size()<64)cache.push_back({c,frozen});
    }
    next.precomputed_=std::move(frozen);
    *this=std::move(next);return true;
}
void ThermalReticleReceiverV1::reset() noexcept {
    startedS_=lastTimeS_=lastDetectorSampleTimeS_=-1;detectorCharge_=expectedCharge_=0;seeded_=false;cached_={};
}
double ThermalReticleReceiverV1::normal() {
    const auto uniform=[&]{return (static_cast<double>(rng_()>>11)+.5)/9007199254740992.;};
    return std::sqrt(-2*std::log(uniform()))*std::cos(2*pi*uniform());
}
std::optional<std::array<double,3>> ThermalReticleReceiverV1::captureWindowV1(double readTimeS) const noexcept {
    if(startedS_<0||!std::isfinite(readTimeS)||readTimeS<startedS_)return std::nullopt;
    double scans=std::floor((readTimeS-startedS_)*config_.frequencyHz);
    if(!std::isfinite(scans))return std::nullopt;
    if(scans<1)return std::nullopt;
    double end=startedS_+scans/config_.frequencyHz;
    if(end>readTimeS){scans-=1;end=startedS_+scans/config_.frequencyHz;}
    if(scans<1||!std::isfinite(end)||end>readTimeS)return std::nullopt;
    const double start=scans==1?startedS_:startedS_+(scans-1)/config_.frequencyHz;
    if(!(start>=startedS_&&end>start))return std::nullopt;
    return std::array<double,3>{start,.5*(start+end),end};
}

ThermalReticleResultV1 ThermalReticleReceiverV1::expose(const ThermalSceneFrameV1& scene,V pointing,std::uint64_t seed) {
    return exposeWithMeasuredPriorV2(scene,pointing,seed,nullptr);
}

ThermalReticleResultV1 ThermalReticleReceiverV1::exposeWithMeasuredPriorV2(
    const ThermalSceneFrameV1& scene,V pointing,std::uint64_t seed,const ThermalReticleMeasuredPriorV2* prior) {
    return exposeWithMeasuredPriorV2(scene,pointing,seed,prior,nullptr);
}

namespace {
// Cutoff-independent full-frame geometry prepared once per fitHorizon() / expose()
// call so candidate cutoffs reuse row/column Gaussian intervals, tangents and 1D
// Gauss quadrature nodes for X = U(-pitch/2, pitch/2) + sigma * Z.
struct UnitGroundPrepV1 {
    ThermalImageConfigV1 p;
    V forward{},nativeUp{},right{},vertical{};
    double pitch=0,sigma=0,margin=0,pixelArea=0,halfX=0,halfY=0;
    double left=0,rightEdge=0,bottom=0,top=0;
    double a=0,b=0,c=0;
    bool valid=false,monoOk=false,symmetricX=false;
    double g2Offset=0;
    double g3Offset=0,g3W0=0,g3W1=0;
    struct Row {double low=0,high=0,sourceLow=0,sourceHigh=0,full=0,constant=0,minCy=0,maxCy=0,minY2=0,maxY2=0;};
    struct Col {double px0=0,px1=0,mid=0,sx0=0,sx1=0,ix=0;std::array<double,4> xBounds{};bool uncut=false;};
    std::array<Row,128> rows{};
    std::array<Col,128> cols{};
    // Cutoff-independent column/row cosine bounds & x-quadrature node geometry
    // lazily populated on the first evaluateRawWaveform() call (eliminates
    // 100k+ tan/sqrt/div calls and repeated scratch allocations inside fitHorizon).
    struct NodeGeom {double tx=0,tx2=0,n=0,fBot=0,fTop=0;};
    mutable bool waveBoundsReady=false;
    mutable bool waveAllUncut=false,waveAllFullX=false;
    mutable unsigned waveColStride=0;
    mutable std::array<double,128> rowGlobalMinCos{},rowGlobalMaxCos{},colU{};
    mutable std::vector<double> colRowMinCos,colRowMaxCos;
    mutable std::vector<std::array<NodeGeom,5>> colNodes;
    mutable std::vector<double> scratchMixedVals;

    void ensureWaveBounds() const {
        if(waveBoundsReady)return;
        waveBoundsReady=true;
        const auto bounds=[&](const std::array<double,4>& x,const Row& row) noexcept {
            const double n0=a+x[0]+row.minCy,n1=a+x[1]+row.maxCy;
            const double d0=std::sqrt(1+x[2]+row.minY2),d1=std::sqrt(1+x[3]+row.maxY2);
            return std::array<double,2>{std::min({n0/d0,n0/d1,n1/d0,n1/d1}),std::max({n0/d0,n0/d1,n1/d0,n1/d1})};
        };
        std::array<double,4> allXBounds{
            std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),0.};
        waveAllUncut=true;waveAllFullX=true;
        const double invHalfW=2.0/static_cast<double>(p.width);
        for(unsigned col=0;col<p.width;++col) {
            colU[col]=(static_cast<double>(col)+0.5-0.5*static_cast<double>(p.width))*invHalfW;
            const auto& ci=cols[col];
            if(!(ci.sx1>ci.sx0)||!ci.uncut){waveAllUncut=false;return;}
            if(std::abs(ci.ix-pitch)>1e-12*pitch)waveAllFullX=false;
            allXBounds[0]=std::min(allXBounds[0],ci.xBounds[0]);
            allXBounds[1]=std::max(allXBounds[1],ci.xBounds[1]);
            allXBounds[2]=std::min(allXBounds[2],ci.xBounds[2]);
            allXBounds[3]=std::max(allXBounds[3],ci.xBounds[3]);
        }
        for(unsigned row=0;row<p.height;++row) {
            const auto& info=rows[row];
            if(!(info.sourceHigh>info.sourceLow)){rowGlobalMinCos[row]=2.;rowGlobalMaxCos[row]=2.;continue;}
            const auto gr=bounds(allXBounds,info);
            rowGlobalMinCos[row]=gr[0];rowGlobalMaxCos[row]=gr[1];
        }
        const unsigned symCols=(p.width+1u)/2u;
        waveColStride=symmetricX?symCols:p.width;
        colRowMinCos.resize(static_cast<std::size_t>(waveColStride)*p.height);
        colRowMaxCos.resize(static_cast<std::size_t>(waveColStride)*p.height);
        colNodes.resize(waveColStride);
        const double tyBot=std::tan(bottom),tyTop=std::tan(top);
        const double tyBot2=tyBot*tyBot,tyTop2=tyTop*tyTop;
        const double cTyBot=c*tyBot,cTyTop=c*tyTop;
        const std::array<double,5> offsets{0.,-g2Offset,g2Offset,-g3Offset,g3Offset};
        for(unsigned col=0;col<waveColStride;++col) {
            const auto& ci=cols[col];
            const std::size_t base=static_cast<std::size_t>(col)*p.height;
            for(unsigned row=0;row<p.height;++row) {
                const auto& info=rows[row];
                if(!(info.sourceHigh>info.sourceLow)){colRowMinCos[base+row]=2.;colRowMaxCos[base+row]=2.;continue;}
                const auto cr=bounds(ci.xBounds,info);
                colRowMinCos[base+row]=cr[0];colRowMaxCos[base+row]=cr[1];
            }
            for(unsigned k=0;k<5;++k) {
                const double tx=std::tan(ci.mid+offsets[k]);
                const double tx2=tx*tx;
                const double n=a+b*tx;
                colNodes[col][k]=NodeGeom{tx,tx2,n,
                    (n+cTyBot)/std::sqrt(1.0+tx2+tyBot2),
                    (n+cTyTop)/std::sqrt(1.0+tx2+tyTop2)};
            }
        }
        scratchMixedVals.reserve(static_cast<std::size_t>(p.height)*waveColStride);
    }

    static double gaussianPrimitiveFast(double z) noexcept {
        return gaussianPrimitiveTableV1().eval(z);
    }

    static UnitGroundPrepV1 build(const ThermalImageConfigV1& cfg,V pointing,V up) {
        UnitGroundPrepV1 g;g.p=cfg;
        if(!ThermalImageReceiverV1::validConfig(cfg)||!finite(pointing)||norm(pointing)<=0||
           !finite(up)||std::abs(norm(up)-1)>1e-6)return g;
        g.forward=unit(pointing);g.nativeUp=unit(up);
        g.right=unit(cross(g.forward,g.nativeUp));
        if(norm(g.right)<.5)g.right=unit(cross(g.forward,std::abs(g.forward.z)<.9?V{0,0,1}:V{0,1,0}));
        g.vertical=unit(cross(g.right,g.forward));
        g.pitch=cfg.pixelPitchRad;g.sigma=cfg.psfSigmaRad;g.margin=9*g.sigma;g.pixelArea=g.pitch*g.pitch;
        g.halfX=.5*cfg.width*g.pitch;g.halfY=.5*cfg.height*g.pitch;
        g.left=std::max(-1.55,-g.halfX-g.margin);g.rightEdge=std::min(1.55,g.halfX+g.margin);
        g.bottom=std::max(-1.55,-g.halfY-g.margin);g.top=std::min(1.55,g.halfY+g.margin);
        g.a=dot(g.forward,g.nativeUp);g.b=dot(g.right,g.nativeUp);g.c=dot(g.vertical,g.nativeUp);
        const double maxTx=std::max(std::abs(std::tan(g.left)),std::abs(std::tan(g.rightEdge)));
        const double maxTy=std::max(std::abs(std::tan(g.bottom)),std::abs(std::tan(g.top)));
        const double maxN=std::nextafter(std::abs(g.a)+std::nextafter(std::abs(g.b)*maxTx,std::numeric_limits<double>::infinity()),std::numeric_limits<double>::infinity());
        g.monoOk=(g.rightEdge>g.left)&&(g.top>g.bottom)&&
            (std::nextafter(g.c,-std::numeric_limits<double>::infinity())>
             std::nextafter(maxN*maxTy,std::numeric_limits<double>::infinity()));
        g.symmetricX=(std::abs(g.b)<=1e-15);
        const double p2=g.pitch*g.pitch,s2=g.sigma*g.sigma;
        const double m2=p2/12.0+s2;
        const double m4=p2*p2/80.0+0.5*p2*s2+3.0*s2*s2;
        g.g2Offset=std::sqrt(m2);
        g.g3Offset=std::sqrt(m4/m2);
        g.g3W1=(m2*m2)/(2.0*m4);
        g.g3W0=1.0-2.0*g.g3W1;
        const auto minSquare=[](double low,double high){return low<=0&&high>=0?0.:std::min(low*low,high*high);};
        for(unsigned row=0;row<cfg.height;++row) {
            auto& info=g.rows[row];
            info.low=(row-.5*cfg.height)*g.pitch;info.high=info.low+g.pitch;
            info.sourceLow=std::max(g.bottom,info.low-g.margin);info.sourceHigh=std::min(g.top,info.high+g.margin);
            if(!(info.sourceHigh>info.sourceLow))continue;
            info.full=ThermalImageReceiverV1::gaussianIntervalV1(info.sourceLow,info.sourceHigh,info.low,info.high,g.sigma);
            info.constant=g.sigma*(gaussianPrimitiveFast((info.high-info.sourceLow)/g.sigma)-gaussianPrimitiveFast((info.low-info.sourceLow)/g.sigma));
            const double ty0=std::tan(info.sourceLow),ty1=std::tan(info.sourceHigh);
            info.minCy=std::min(g.c*ty0,g.c*ty1);info.maxCy=std::max(g.c*ty0,g.c*ty1);
            info.minY2=minSquare(ty0,ty1);info.maxY2=std::max(ty0*ty0,ty1*ty1);
        }
        for(unsigned col=0;col<cfg.width;++col) {
            auto& ci=g.cols[col];
            ci.px0=(col-.5*cfg.width)*g.pitch;ci.px1=ci.px0+g.pitch;ci.mid=.5*(ci.px0+ci.px1);
            ci.sx0=std::max(g.left,ci.px0-g.margin);ci.sx1=std::min(g.rightEdge,ci.px1+g.margin);
            ci.uncut=(ci.px0-g.margin>-1.55&&ci.px1+g.margin<1.55);
            if(ci.sx1>ci.sx0) {
                ci.ix=ThermalImageReceiverV1::gaussianIntervalV1(ci.sx0,ci.sx1,ci.px0,ci.px1,g.sigma);
                const double tx0=std::tan(ci.sx0),tx1=std::tan(ci.sx1);
                ci.xBounds={std::min(g.b*tx0,g.b*tx1),std::max(g.b*tx0,g.b*tx1),minSquare(tx0,tx1),std::max(tx0*tx0,tx1*tx1)};
            }
        }
        g.valid=true;
        return g;
    }

    std::optional<std::vector<double>> evaluateFullFrame(double cutoff,bool allowUnitFallback=true) const {
        if(!valid||!std::isfinite(cutoff)||std::abs(cutoff)>1)return std::nullopt;
        const auto fallback=[&]()->std::optional<std::vector<double>> {
            return allowUnitFallback?ThermalReticleReceiverV1::unitGroundFieldV1(p,forward,nativeUp,cutoff):std::nullopt;
        };
        if(!monoOk)return fallback();
        const std::size_t pixels=static_cast<std::size_t>(p.width)*p.height;
        std::vector<double> answer(pixels,0.);
        const auto bounds=[&](const std::array<double,4>& x,const Row& row) noexcept {
            const double n0=a+x[0]+row.minCy,n1=a+x[1]+row.maxCy;
            const double d0=std::sqrt(1+x[2]+row.minY2),d1=std::sqrt(1+x[3]+row.maxY2);
            return std::array<double,2>{std::min({n0/d0,n0/d1,n1/d0,n1/d1}),std::max({n0/d0,n0/d1,n1/d0,n1/d1})};
        };
        const double invSigma=1.0/sigma,invPitch=1.0/pitch;
        const bool flatHorizon=symmetricX&&cutoff==0.0;
        using Values=std::array<double,128>;
        std::array<bool,128> rowActive{};
        const unsigned colLimit=flatHorizon?1u:(symmetricX?((p.width+1u)/2u):p.width);
        for(unsigned col=0;col<colLimit;++col) {
            const auto& ci=cols[col];
            if(!(ci.sx1>ci.sx0))continue;
            if(!ci.uncut)return fallback();
            unsigned activeCount=0,activeLo=p.height,activeHi=0;
            for(unsigned row=0;row<p.height;++row) {
                rowActive[row]=false;
                const auto& info=rows[row];
                if(!(info.sourceHigh>info.sourceLow))continue;
                const auto range=bounds(ci.xBounds,info);
                if(range[0]>=cutoff)continue;
                if(range[1]<cutoff) {
                    const double v=ci.ix*info.full/pixelArea;
                    if(flatHorizon) {
                        for(unsigned dst=0;dst<p.width;++dst)answer[static_cast<std::size_t>(row)*p.width+dst]=cols[dst].ix*info.full/pixelArea;
                    } else {
                        answer[static_cast<std::size_t>(row)*p.width+col]=v;
                        if(symmetricX)answer[static_cast<std::size_t>(row)*p.width+(p.width-1u-col)]=v;
                    }
                    continue;
                }
                rowActive[row]=true;++activeCount;
                if(row<activeLo)activeLo=row;
                activeHi=row;
            }
            if(!activeCount)continue;
            const double tyBot=std::tan(bottom),tyTop=std::tan(top);
            const auto at=[&](double x) noexcept {
                Values values{};
                const double tx=std::tan(x),n=a+b*tx,tx2=tx*tx;
                // Because monoOk guarantees f(y) = (n + c*tan(y))/sqrt(1 + tx^2 + tan(y)^2)
                // is strictly increasing on [bottom, top], ground is a single interval [bottom, yRoot].
                const double fBot=(n+c*tyBot)/std::sqrt(1.0+tx2+tyBot*tyBot);
                if(fBot>=cutoff)return values;
                const double fTop=(n+c*tyTop)/std::sqrt(1.0+tx2+tyTop*tyTop);
                if(fTop<cutoff) {
                    for(unsigned row=activeLo;row<=activeHi;++row)if(rowActive[row])values[row]=rows[row].full;
                    return values;
                }
                double yRoot=top;
                bool foundRoot=false;
                if(cutoff==0.0) {
                    const double y=std::atan(-n/c);
                    if(y>bottom&&y<top){yRoot=y;foundRoot=true;}
                } else {
                    const double aa=c*c-cutoff*cutoff,bb=2*n*c,cc=n*n-cutoff*cutoff*(1+tx2);
                    const auto checkRoot=[&](double t) noexcept {
                        const double num=n+c*t;
                        if((cutoff>0.0&&num<0.0)||(cutoff<0.0&&num>0.0))return;
                        const double y=std::atan(t);
                        if(y>bottom&&y<top){yRoot=y;foundRoot=true;}
                    };
                    if(std::abs(aa)<1e-14){if(std::abs(bb)>1e-14)checkRoot(-cc/bb);}
                    else {
                        const double disc=bb*bb-4*aa*cc;
                        if(disc>=0){const double sq=std::sqrt(disc);checkRoot((-bb-sq)/(2*aa));if(!foundRoot)checkRoot((-bb+sq)/(2*aa));}
                    }
                }
                if(!foundRoot) {
                    for(unsigned row=activeLo;row<=activeHi;++row)if(rowActive[row])values[row]=rows[row].full;
                    return values;
                }
                std::array<double,129> primHi{};
                const bool hiInterior=(yRoot>rows[activeLo].sourceLow&&yRoot<rows[activeHi].sourceHigh);
                if(hiInterior) {
                    for(unsigned e=activeLo;e<=activeHi+1;++e) {
                        const double edge=(e==activeHi+1)?rows[activeHi].high:rows[e].low;
                        primHi[e]=gaussianPrimitiveFast((edge-yRoot)*invSigma);
                    }
                }
                for(unsigned row=activeLo;row<=activeHi;++row) {
                    if(!rowActive[row])continue;
                    const auto& info=rows[row];
                    if(yRoot<=info.sourceLow)continue;
                    if(yRoot>=info.sourceHigh){values[row]=info.full;continue;}
                    const double d=hiInterior?(primHi[row+1]-primHi[row]):
                        (gaussianPrimitiveFast((info.high-yRoot)*invSigma)-gaussianPrimitiveFast((info.low-yRoot)*invSigma));
                    values[row]=std::clamp(info.constant-sigma*d,0.,std::min(yRoot-info.sourceLow,pitch));
                }
                return values;
            };
            Values fine{};
            if(flatHorizon) {
                const Values s1=at(ci.mid);
                for(unsigned row=activeLo;row<=activeHi;++row) {
                    if(!rowActive[row])continue;
                    fine[row]=s1[row]*invPitch;
                }
            } else {
                // Adaptive 1D Gauss check for X = U + sigma*Z:
                // compare 1-pt vs 2-pt (3 evals) first; only evaluate 3-pt (5 evals) when curvature > 1e-8.
                const Values s3c=at(ci.mid);
                const Values s2m=at(ci.mid-g2Offset),s2p=at(ci.mid+g2Offset);
                bool needG3=false;
                for(unsigned row=activeLo;row<=activeHi;++row) {
                    if(!rowActive[row])continue;
                    const double g1=s3c[row]*invPitch;
                    const double g2=0.5*(s2m[row]+s2p[row])*invPitch;
                    fine[row]=g2;
                    if(std::abs(g2-g1)>1e-8)needG3=true;
                }
                if(needG3) {
                    const Values s3m=at(ci.mid-g3Offset),s3p=at(ci.mid+g3Offset);
                    bool converged=true;
                    for(unsigned row=activeLo;row<=activeHi;++row) {
                        if(!rowActive[row])continue;
                        const double g2=fine[row];
                        const double g3=(g3W0*s3c[row]+g3W1*(s3m[row]+s3p[row]))*invPitch;
                        fine[row]=g3;
                        if(std::abs(g3-g2)>1e-9)converged=false;
                    }
                    if(!converged)return fallback();
                }
            }
            for(unsigned row=activeLo;row<=activeHi;++row) {
                if(!rowActive[row])continue;
                if(!std::isfinite(fine[row])||fine[row]<0||fine[row]>1+1e-9)
                    return fallback();
                const double v=std::clamp(fine[row],0.,1.);
                if(flatHorizon) {
                    for(unsigned dst=0;dst<p.width;++dst) {
                        if(!cols[dst].uncut)return fallback();
                        answer[static_cast<std::size_t>(row)*p.width+dst]=v;
                    }
                } else {
                    answer[static_cast<std::size_t>(row)*p.width+col]=v;
                    if(symmetricX)answer[static_cast<std::size_t>(row)*p.width+(p.width-1u-col)]=v;
                }
            }
        }
        return answer;
    }

    std::optional<std::vector<double>> evaluateBackground(double skyRad,double groundRad,double cutoff) const {
        if(!valid||!monoOk||!std::isfinite(skyRad)||!std::isfinite(groundRad)||skyRad<0||groundRad<0)return std::nullopt;
        const std::size_t pixels=static_cast<std::size_t>(p.width)*p.height;
        if(skyRad==groundRad) {
            std::vector<double> bg(pixels,0.);
            for(unsigned row=0;row<p.height;++row) {
                const double rf=rows[row].full/pixelArea;
                for(unsigned col=0;col<p.width;++col)
                    bg[static_cast<std::size_t>(row)*p.width+col]=skyRad*cols[col].ix*rf;
            }
            return bg;
        }
        auto ug=evaluateFullFrame(cutoff);
        if(!ug||ug->size()!=pixels)return std::nullopt;
        const double maxRad=std::max(skyRad,groundRad);
        const double deltaRad=groundRad-skyRad;
        for(unsigned row=0;row<p.height;++row) {
            const double rf=rows[row].full/pixelArea;
            for(unsigned col=0;col<p.width;++col) {
                const std::size_t idx=static_cast<std::size_t>(row)*p.width+col;
                const double fullMass=cols[col].ix*rf;
                (*ug)[idx]=std::clamp(skyRad*fullMass+deltaRad*(*ug)[idx],0.0,maxRad*fullMass);
            }
        }
        return ug;
    }

    // Direct full-frame raw-waveform evaluation without allocating a 4096-pixel
    // intermediate field, avoiding 64KB column-stride cache aliasing by computing
    // mixed rows into a compact row-major scratch buffer and contracting smooth
    // rows via L2-resident x-moment basis (verified exact to <= 1e-10) with
    // exact row-major prefix-summed fallback for steeply tilted horizons.
    bool evaluateRawWaveform(double cutoff,const double* wwPtr,const double* swwPtr,
        const double* rwPrefixPtr,const double* rowColPrefixPtr,
        const std::array<std::vector<double>,5>& rowMomentWave,
        unsigned count,double alpha,std::vector<double>& outRawWave) const {
        if(!valid||!monoOk||!std::isfinite(cutoff)||std::abs(cutoff)>1)return false;
        ensureWaveBounds();
        if(!waveAllUncut)return false;
        outRawWave.assign(count,0.);
        std::array<std::uint8_t,128> globalRowState{}; // 0=mixed, 1=all-sky, 2=all-ground
        std::array<unsigned,128> mixedRows{};
        std::array<unsigned,128> rowToMixed{};
        unsigned mixedCount=0;
        for(unsigned row=0;row<p.height;++row) {
            const auto& info=rows[row];
            if(!(info.sourceHigh>info.sourceLow)||rowGlobalMinCos[row]>=cutoff){globalRowState[row]=1;continue;}
            if(rowGlobalMaxCos[row]<cutoff&&waveAllFullX&&std::abs(info.full-pitch)<=1e-12*pitch){globalRowState[row]=2;continue;}
            rowToMixed[row]=mixedCount;
            mixedRows[mixedCount++]=row;
        }
        // Accumulate contiguous all-ground runs in O(count) via prefix sums.
        for(unsigned row=0;row<p.height;) {
            if(globalRowState[row]!=2){++row;continue;}
            unsigned endRow=row+1;
            while(endRow<p.height&&globalRowState[endRow]==2)++endRow;
            const double* p0=rwPrefixPtr+static_cast<std::size_t>(row)*count;
            const double* p1=rwPrefixPtr+static_cast<std::size_t>(endRow)*count;
            for(unsigned j=0;j<count;++j)outRawWave[j]+=p1[j]-p0[j];
            row=endRow;
        }
        if(mixedCount>0) {
            const double invSigma=1.0/sigma,invPitch=1.0/pitch;
            const bool flatHorizon=symmetricX&&cutoff==0.0;
            const unsigned symCols=(p.width+1u)/2u;
            const unsigned colLimit=flatHorizon?1u:(symmetricX?symCols:p.width);
            scratchMixedVals.assign(static_cast<std::size_t>(mixedCount)*colLimit,0.);
            double* mixedPtr=scratchMixedVals.data();
            std::array<std::uint8_t,128> rowActive{};
            std::array<double,128> s3c{},s2m{},s2p{},s3m{},s3p{},fine{};
            std::array<double,129> primHi{};
            const double aa=c*c-cutoff*cutoff;
            const bool aaTiny=std::abs(aa)<1e-14;
            const double inv2Aa=aaTiny?0.:(1.0/(2.0*aa));
            const double cut2=cutoff*cutoff;
            const auto solveYRoot=[&](const NodeGeom& ng,double& outRoot) noexcept -> int {
                // Returns 0 = all sky, 2 = all ground, 1 = root in (bottom, top) stored in outRoot.
                if(ng.fBot>=cutoff)return 0;
                if(ng.fTop<cutoff)return 2;
                const double n=ng.n;
                if(cutoff==0.0) {
                    const double y=std::atan(-n/c);
                    if(y>bottom&&y<top){outRoot=y;return 1;}
                    return 2;
                }
                const double bb=2.0*n*c,cc=n*n-cut2*(1.0+ng.tx2);
                double yRoot=top;bool foundRoot=false;
                const auto checkRoot=[&](double t) noexcept {
                    const double num=n+c*t;
                    if((cutoff>0.0&&num<0.0)||(cutoff<0.0&&num>0.0))return;
                    const double y=std::atan(t);
                    if(y>bottom&&y<top){yRoot=y;foundRoot=true;}
                };
                if(aaTiny){if(std::abs(bb)>1e-14)checkRoot(-cc/bb);}
                else {
                    const double disc=bb*bb-4.0*aa*cc;
                    if(disc>=0.){const double sq=std::sqrt(disc);checkRoot((-bb-sq)*inv2Aa);if(!foundRoot)checkRoot((-bb+sq)*inv2Aa);}
                }
                if(!foundRoot)return 2;
                outRoot=yRoot;
                return 1;
            };
            const auto fillFromRoot=[&](int kind,double yRoot,unsigned activeLo,unsigned activeHi,double* outVals) noexcept {
                if(kind==0) {
                    for(unsigned row=activeLo;row<=activeHi;++row)if(rowActive[row])outVals[row]=0.;
                    return;
                }
                if(kind==2) {
                    for(unsigned row=activeLo;row<=activeHi;++row)if(rowActive[row])outVals[row]=rows[row].full;
                    return;
                }
                const bool hiInterior=(yRoot>rows[activeLo].sourceLow&&yRoot<rows[activeHi].sourceHigh);
                if(hiInterior) {
                    for(unsigned e=activeLo;e<=activeHi+1u;++e) {
                        const double edge=(e==activeHi+1u)?rows[activeHi].high:rows[e].low;
                        primHi[e]=gaussianPrimitiveFast((edge-yRoot)*invSigma);
                    }
                }
                for(unsigned row=activeLo;row<=activeHi;++row) {
                    if(!rowActive[row])continue;
                    const auto& info=rows[row];
                    if(yRoot<=info.sourceLow){outVals[row]=0.;continue;}
                    if(yRoot>=info.sourceHigh){outVals[row]=info.full;continue;}
                    const double d=hiInterior?(primHi[row+1u]-primHi[row]):
                        (gaussianPrimitiveFast((info.high-yRoot)*invSigma)-gaussianPrimitiveFast((info.low-yRoot)*invSigma));
                    outVals[row]=std::clamp(info.constant-sigma*d,0.,std::min(yRoot-info.sourceLow,pitch));
                }
            };
            const auto evalNode=[&](const NodeGeom& ng,unsigned activeLo,unsigned activeHi,double* outVals) noexcept {
                double yRoot=0.;
                const int kind=solveYRoot(ng,yRoot);
                fillFromRoot(kind,yRoot,activeLo,activeHi,outVals);
            };
            const auto evalColumn=[&](unsigned col)->bool {
                const auto& ci=cols[col];
                const std::size_t crBase=static_cast<std::size_t>(col)*p.height;
                unsigned activeCount=0,activeLo=p.height,activeHi=0;
                for(unsigned mIdx=0;mIdx<mixedCount;++mIdx) {
                    const unsigned row=mixedRows[mIdx];
                    rowActive[row]=0u;
                    if(colRowMinCos[crBase+row]>=cutoff)continue;
                    if(colRowMaxCos[crBase+row]<cutoff) {
                        mixedPtr[static_cast<std::size_t>(mIdx)*colLimit+col]=ci.ix*rows[row].full/pixelArea;
                        continue;
                    }
                    rowActive[row]=1u;++activeCount;
                    if(row<activeLo)activeLo=row;
                    activeHi=row;
                }
                if(!activeCount)return true;
                const auto& nodes=colNodes[col];
                if(flatHorizon) {
                    evalNode(nodes[0],activeLo,activeHi,s3c.data());
                    for(unsigned row=activeLo;row<=activeHi;++row)if(rowActive[row])fine[row]=s3c[row]*invPitch;
                } else {
                    double r0=0.,r1=0.,r2=0.;
                    const int k0=solveYRoot(nodes[0],r0);
                    const int k1=solveYRoot(nodes[1],r1);
                    const int k2=solveYRoot(nodes[2],r2);
                    fillFromRoot(k0,r0,activeLo,activeHi,s3c.data());
                    // Second-order quadrature error is <= 0.25 * (dr/sigma)^2;
                    // when |dr| <= 1e-5*sigma and |r1+r2-2*r0| <= 1e-9*sigma,
                    // 1-point matches 2-point to < 3e-11 (30x tighter than 1e-9).
                    const double rootTol=1e-5*sigma,curvTol=1e-9*sigma;
                    if(k0==k1&&k0==k2&&(k0!=1||(std::abs(r1-r0)<=rootTol&&std::abs(r2-r0)<=rootTol&&std::abs((r1+r2)-2.0*r0)<=curvTol))) {
                        for(unsigned row=activeLo;row<=activeHi;++row)if(rowActive[row])fine[row]=s3c[row]*invPitch;
                    } else {
                        fillFromRoot(k1,r1,activeLo,activeHi,s2m.data());
                        fillFromRoot(k2,r2,activeLo,activeHi,s2p.data());
                        bool needG3=false;
                        for(unsigned row=activeLo;row<=activeHi;++row) {
                            if(!rowActive[row])continue;
                            const double g1=s3c[row]*invPitch,g2=0.5*(s2m[row]+s2p[row])*invPitch;
                            fine[row]=g2;
                            if(std::abs(g2-g1)>1e-8)needG3=true;
                        }
                        if(needG3) {
                            evalNode(nodes[3],activeLo,activeHi,s3m.data());
                            evalNode(nodes[4],activeLo,activeHi,s3p.data());
                            for(unsigned row=activeLo;row<=activeHi;++row) {
                                if(!rowActive[row])continue;
                                const double g2=fine[row];
                                const double g3=(g3W0*s3c[row]+g3W1*(s3m[row]+s3p[row]))*invPitch;
                                if(std::abs(g3-g2)>1e-9)return false;
                                fine[row]=g3;
                            }
                        }
                    }
                }
                for(unsigned row=activeLo;row<=activeHi;++row) {
                    if(!rowActive[row])continue;
                    if(!std::isfinite(fine[row])||fine[row]<0||fine[row]>1+1e-9)return false;
                    mixedPtr[static_cast<std::size_t>(rowToMixed[row])*colLimit+col]=std::clamp(fine[row],0.,1.);
                }
                return true;
            };
            // Verified 5-column symmetric fast path: when b == 0 (nearly every
            // pose), evaluate 5 probe columns first (3 fit + 2 interior verification
            // nodes). If every mixed row's even quadratic in z = u^2 matches both
            // interior check columns to <= 1e-10 (and has uniform active state),
            // contract directly via L2-resident x-moments without touching the
            // remaining 27 columns. Falls back to full column evaluation otherwise.
            bool fast5Done=false;
            if(symmetricX&&!flatHorizon&&colLimit>=8u&&waveAllFullX) {
                const unsigned i0=0u,i1=colLimit/4u,i2=colLimit/2u,i3=(3u*colLimit)/4u,i4=colLimit-1u;
                const unsigned probes[5]={i0,i1,i2,i3,i4};
                bool probesOk=true;
                for(unsigned pk=0;pk<5;++pk)if(!evalColumn(probes[pk])){probesOk=false;break;}
                if(!probesOk)return false;
                const double z0=colU[i0]*colU[i0],z1=colU[i1]*colU[i1],z2=colU[i2]*colU[i2],z3=colU[i3]*colU[i3],z4=colU[i4]*colU[i4];
                std::array<std::array<double,3>,128> rowCoefs{};
                bool allPolyOk=true;
                for(unsigned mIdx=0;mIdx<mixedCount;++mIdx) {
                    const double* rv=mixedPtr+static_cast<std::size_t>(mIdx)*colLimit;
                    const double d02=(rv[i2]-rv[i0])/(z2-z0);
                    const double d24=(rv[i4]-rv[i2])/(z4-z2);
                    const double c4=(d24-d02)/(z4-z0);
                    const double c2=d02-c4*(z0+z2);
                    const double c0=rv[i0]-z0*(c2+c4*z0);
                    if(std::abs(rv[i1]-(c0+z1*(c2+c4*z1)))>1e-10||
                       std::abs(rv[i3]-(c0+z3*(c2+c4*z3)))>1e-10) {
                        allPolyOk=false;break;
                    }
                    rowCoefs[mIdx]={c0,c2,c4};
                }
                if(allPolyOk) {
                    for(unsigned mIdx=0;mIdx<mixedCount;++mIdx) {
                        const unsigned row=mixedRows[mIdx];
                        const double c0=rowCoefs[mIdx][0],c2=rowCoefs[mIdx][1],c4=rowCoefs[mIdx][2];
                        const std::size_t rOff=static_cast<std::size_t>(row)*count;
                        const double* m0=rowMomentWave[0].data()+rOff;
                        const double* m2=rowMomentWave[2].data()+rOff;
                        const double* m4=rowMomentWave[4].data()+rOff;
                        if(c2==0.&&c4==0.) {
                            if(c0!=0.)for(unsigned j=0;j<count;++j)outRawWave[j]+=c0*m0[j];
                        } else {
                            for(unsigned j=0;j<count;++j)outRawWave[j]+=c0*m0[j]+c2*m2[j]+c4*m4[j];
                        }
                    }
                    fast5Done=true;
                } else {
                    for(unsigned col=0;col<colLimit;++col) {
                        if(col==i0||col==i1||col==i2||col==i3||col==i4)continue;
                        if(!evalColumn(col))return false;
                    }
                }
            } else {
                for(unsigned col=0;col<colLimit;++col)if(!evalColumn(col))return false;
            }
            // Row-major contraction: use L2-resident x-moment basis when verified
            // exact to <= 1e-10 across all columns, else exact contiguous row-major scan.
            const std::size_t pCols=static_cast<std::size_t>(p.width)+1u;
            for(unsigned mIdx=0;!fast5Done&&mIdx<mixedCount;++mIdx) {
                const unsigned row=mixedRows[mIdx];
                const double* rv=mixedPtr+static_cast<std::size_t>(mIdx)*colLimit;
                if(flatHorizon) {
                    const double v=rv[0];
                    if(v==0.)continue;
                    const double* rw0=rwPrefixPtr+static_cast<std::size_t>(row)*count;
                    const double* rw1=rwPrefixPtr+static_cast<std::size_t>(row+1u)*count;
                    for(unsigned j=0;j<count;++j)outRawWave[j]+=v*(rw1[j]-rw0[j]);
                    continue;
                }
                if(symmetricX) {
                    if(colLimit>=3u) {
                        const unsigned i0=0u,i1=colLimit/2u,i2=colLimit-1u;
                        const double z0=colU[i0]*colU[i0],z1=colU[i1]*colU[i1],z2=colU[i2]*colU[i2];
                        const double d01=(rv[i1]-rv[i0])/(z1-z0);
                        const double d12=(rv[i2]-rv[i1])/(z2-z1);
                        const double c4=(d12-d01)/(z2-z0);
                        const double c2=d01-c4*(z0+z1);
                        const double c0=rv[i0]-z0*(c2+c4*z0);
                        bool polyOk=true;
                        for(unsigned col=0;col<colLimit;++col) {
                            const double z=colU[col]*colU[col];
                            if(std::abs(rv[col]-(c0+z*(c2+c4*z)))>1e-10){polyOk=false;break;}
                        }
                        if(polyOk) {
                            const std::size_t rOff=static_cast<std::size_t>(row)*count;
                            const double* m0=rowMomentWave[0].data()+rOff;
                            const double* m2=rowMomentWave[2].data()+rOff;
                            const double* m4=rowMomentWave[4].data()+rOff;
                            if(c2==0.&&c4==0.) {
                                if(c0!=0.)for(unsigned j=0;j<count;++j)outRawWave[j]+=c0*m0[j];
                            } else {
                                for(unsigned j=0;j<count;++j)outRawWave[j]+=c0*m0[j]+c2*m2[j]+c4*m4[j];
                            }
                            continue;
                        }
                    }
                    const std::size_t symRowBase=static_cast<std::size_t>(row)*symCols;
                    for(unsigned col=0;col<colLimit;++col) {
                        const double v=rv[col];
                        if(v==0.)continue;
                        const double* sw=swwPtr+(symRowBase+col)*count;
                        for(unsigned j=0;j<count;++j)outRawWave[j]+=v*sw[j];
                    }
                } else {
                    if(colLimit>=4u) {
                        const unsigned i0=0u,i1=colLimit/3u,i2=(2u*colLimit)/3u,i3=colLimit-1u;
                        const double u0=colU[i0],u1=colU[i1],u2=colU[i2],u3=colU[i3];
                        const double y0=rv[i0],y1=rv[i1],y2=rv[i2],y3=rv[i3];
                        const double d01=(y1-y0)/(u1-u0),d12=(y2-y1)/(u2-u1),d23=(y3-y2)/(u3-u2);
                        const double e012=(d12-d01)/(u2-u0),e123=(d23-d12)/(u3-u1);
                        const double c3=(e123-e012)/(u3-u0);
                        const double c2=e012-c3*(u0+u1+u2);
                        const double c1=d01-e012*(u0+u1)+c3*(u0*u1+u0*u2+u1*u2);
                        const double c0=y0-u0*(c1+u0*(c2+u0*c3));
                        bool polyOk=true;
                        for(unsigned col=0;col<colLimit;++col) {
                            const double u=colU[col];
                            if(std::abs(rv[col]-(c0+u*(c1+u*(c2+u*c3))))>1e-10){polyOk=false;break;}
                        }
                        if(polyOk) {
                            const std::size_t rOff=static_cast<std::size_t>(row)*count;
                            const double* m0=rowMomentWave[0].data()+rOff;
                            const double* m1=rowMomentWave[1].data()+rOff;
                            const double* m2=rowMomentWave[2].data()+rOff;
                            const double* m3=rowMomentWave[3].data()+rOff;
                            if(c1==0.&&c2==0.&&c3==0.) {
                                if(c0!=0.)for(unsigned j=0;j<count;++j)outRawWave[j]+=c0*m0[j];
                            } else {
                                for(unsigned j=0;j<count;++j)outRawWave[j]+=c0*m0[j]+c1*m1[j]+c2*m2[j]+c3*m3[j];
                            }
                            continue;
                        }
                    }
                    const std::size_t rowBase=static_cast<std::size_t>(row)*p.width;
                    const std::size_t prefRowBase=static_cast<std::size_t>(row)*pCols;
                    for(unsigned col=0;col<colLimit;) {
                        const double v=rv[col];
                        if(v==0.){++col;continue;}
                        unsigned cEnd=col+1u;
                        while(cEnd<colLimit&&rv[cEnd]==v)++cEnd;
                        if(cEnd-col>=2u&&rowColPrefixPtr) {
                            const double* s0=rowColPrefixPtr+(prefRowBase+col)*count;
                            const double* s1=rowColPrefixPtr+(prefRowBase+cEnd)*count;
                            if(v==1.) {
                                for(unsigned j=0;j<count;++j)outRawWave[j]+=s1[j]-s0[j];
                            } else {
                                for(unsigned j=0;j<count;++j)outRawWave[j]+=v*(s1[j]-s0[j]);
                            }
                        } else {
                            for(unsigned cIdx=col;cIdx<cEnd;++cIdx) {
                                const double* ww=wwPtr+(rowBase+cIdx)*count;
                                for(unsigned j=0;j<count;++j)outRawWave[j]+=v*ww[j];
                            }
                        }
                        col=cEnd;
                    }
                }
            }
        }
        double prev=0;
        for(unsigned j=0;j<count;++j){prev+=alpha*(outRawWave[j]-prev);outRawWave[j]=prev;}
        return true;
    }

    // Zero-allocation projected-wave + scalar uniform/transient dot evaluation
    // for fitHorizon() null-hypothesis search: uses linearity of filtering and
    // nuisance projection to eliminate heap allocations, sequential IIR filtering,
    // 7-column nuisance projection, and rawWave inner products per cutoff.
    bool evaluateProjectedNullV1(double cutoff,
        const double* projRwPrefixPtr,const std::array<std::vector<double>,5>& projRowMomentWave,
        const double* projSymPtr,const double* symUnifPtr,const double* symTranPtr,
        const double* rwUnifPref,const double* rwTranPref,
        const std::array<std::vector<double>,5>& rmUnif,const std::array<std::vector<double>,5>& rmTran,
        unsigned count,double* outWave,double& outDotUniform,double& outDotTransient) const {
        if(!valid||!monoOk||!std::isfinite(cutoff)||std::abs(cutoff)>1)return false;
        ensureWaveBounds();
        if(!waveAllUncut||!waveAllFullX||!symmetricX)return false;
        const bool flatHorizon=(cutoff==0.0);
        const unsigned symCols=(p.width+1u)/2u;
        const unsigned colLimit=flatHorizon?1u:symCols;
        if(!flatHorizon&&colLimit<8u)return false;
        std::fill_n(outWave,count,0.);
        outDotUniform=0.;outDotTransient=0.;
        std::array<std::uint8_t,128> globalRowState;
        std::array<unsigned,128> mixedRows;
        unsigned mixedCount=0;
        for(unsigned row=0;row<p.height;++row) {
            const auto& info=rows[row];
            if(!(info.sourceHigh>info.sourceLow)||rowGlobalMinCos[row]>=cutoff){globalRowState[row]=1;continue;}
            if(rowGlobalMaxCos[row]<cutoff&&std::abs(info.full-pitch)<=1e-12*pitch){globalRowState[row]=2;continue;}
            globalRowState[row]=0;
            mixedRows[mixedCount++]=row;
        }
        for(unsigned row=0;row<p.height;) {
            if(globalRowState[row]!=2){++row;continue;}
            unsigned endRow=row+1;
            while(endRow<p.height&&globalRowState[endRow]==2)++endRow;
            const double* p0=projRwPrefixPtr+static_cast<std::size_t>(row)*count;
            const double* p1=projRwPrefixPtr+static_cast<std::size_t>(endRow)*count;
            reticleAddDiff(outWave,p1,p0,count);
            outDotUniform+=rwUnifPref[endRow]-rwUnifPref[row];
            outDotTransient+=rwTranPref[endRow]-rwTranPref[row];
            row=endRow;
        }
        if(mixedCount==0)return true;
        const double invSigma=1.0/sigma,invPitch=1.0/pitch;
        const unsigned probeCols[5]={0u,colLimit/4u,colLimit/2u,(3u*colLimit)/4u,colLimit-1u};
        const unsigned nProbes=flatHorizon?1u:5u;
        std::array<std::array<double,5>,128> probeVals;
        std::array<std::uint8_t,128> rowActive;
        std::array<double,128> s3c,s2m,s2p,s3m,s3p,fine;
        std::array<double,129> primHi;
        const double aa=c*c-cutoff*cutoff;
        const bool aaTiny=std::abs(aa)<1e-14;
        const double inv2Aa=aaTiny?0.:(1.0/(2.0*aa));
        const double cut2=cutoff*cutoff;
        const auto solveYRoot=[&](const NodeGeom& ng,double& outRoot) noexcept -> int {
            if(ng.fBot>=cutoff)return 0;
            if(ng.fTop<cutoff)return 2;
            const double n=ng.n;
            if(cutoff==0.0) {
                const double y=std::atan(-n/c);
                if(y>bottom&&y<top){outRoot=y;return 1;}
                return 2;
            }
            const double bb=2.0*n*c,cc=n*n-cut2*(1.0+ng.tx2);
            double yRoot=top;bool foundRoot=false;
            const auto checkRoot=[&](double t) noexcept {
                const double num=n+c*t;
                if((cutoff>0.0&&num<0.0)||(cutoff<0.0&&num>0.0))return;
                const double y=std::atan(t);
                if(y>bottom&&y<top){yRoot=y;foundRoot=true;}
            };
            if(aaTiny){if(std::abs(bb)>1e-14)checkRoot(-cc/bb);}
            else {
                const double disc=bb*bb-4.0*aa*cc;
                if(disc>=0.){const double sq=std::sqrt(disc);checkRoot((-bb-sq)*inv2Aa);if(!foundRoot)checkRoot((-bb+sq)*inv2Aa);}
            }
            if(!foundRoot)return 2;
            outRoot=yRoot;
            return 1;
        };
        const auto fillFromRoot=[&](int kind,double yRoot,unsigned activeLo,unsigned activeHi,double* outVals) noexcept {
            if(kind==0){for(unsigned row=activeLo;row<=activeHi;++row)if(rowActive[row])outVals[row]=0.;return;}
            if(kind==2){for(unsigned row=activeLo;row<=activeHi;++row)if(rowActive[row])outVals[row]=rows[row].full;return;}
            const bool hiInterior=(yRoot>rows[activeLo].sourceLow&&yRoot<rows[activeHi].sourceHigh);
            if(hiInterior) {
                for(unsigned e=activeLo;e<=activeHi+1u;++e) {
                    const double edge=(e==activeHi+1u)?rows[activeHi].high:rows[e].low;
                    primHi[e]=gaussianPrimitiveFast((edge-yRoot)*invSigma);
                }
            }
            for(unsigned row=activeLo;row<=activeHi;++row) {
                if(!rowActive[row])continue;
                const auto& info=rows[row];
                if(yRoot<=info.sourceLow){outVals[row]=0.;continue;}
                if(yRoot>=info.sourceHigh){outVals[row]=info.full;continue;}
                const double d=hiInterior?(primHi[row+1u]-primHi[row]):
                    (gaussianPrimitiveFast((info.high-yRoot)*invSigma)-gaussianPrimitiveFast((info.low-yRoot)*invSigma));
                outVals[row]=std::clamp(info.constant-sigma*d,0.,std::min(yRoot-info.sourceLow,pitch));
            }
        };
        const auto evalColSubset=[&](unsigned col,const unsigned* subRows,unsigned subCount,double* outColVals) noexcept -> bool {
            const auto& ci=cols[col];
            const std::size_t crBase=static_cast<std::size_t>(col)*p.height;
            rowActive.fill(0u);
            unsigned activeCount=0,activeLo=p.height,activeHi=0;
            for(unsigned sIdx=0;sIdx<subCount;++sIdx) {
                const unsigned row=subRows[sIdx];
                if(colRowMinCos[crBase+row]>=cutoff){outColVals[sIdx]=0.;continue;}
                if(colRowMaxCos[crBase+row]<cutoff){outColVals[sIdx]=ci.ix*rows[row].full/pixelArea;continue;}
                rowActive[row]=1u;++activeCount;
                if(row<activeLo)activeLo=row;
                activeHi=row;
            }
            if(!activeCount)return true;
            const auto& nodes=colNodes[col];
            if(flatHorizon) {
                double r0=0.;const int k0=solveYRoot(nodes[0],r0);
                fillFromRoot(k0,r0,activeLo,activeHi,s3c.data());
                for(unsigned row=activeLo;row<=activeHi;++row)if(rowActive[row])fine[row]=s3c[row]*invPitch;
            } else {
                double r0=0.,r1=0.,r2=0.;
                const int k0=solveYRoot(nodes[0],r0);
                const int k1=solveYRoot(nodes[1],r1);
                const int k2=solveYRoot(nodes[2],r2);
                fillFromRoot(k0,r0,activeLo,activeHi,s3c.data());
                const double rootTol=1e-5*sigma,curvTol=1e-9*sigma;
                if(k0==k1&&k0==k2&&(k0!=1||(std::abs(r1-r0)<=rootTol&&std::abs(r2-r0)<=rootTol&&std::abs((r1+r2)-2.0*r0)<=curvTol))) {
                    for(unsigned row=activeLo;row<=activeHi;++row)if(rowActive[row])fine[row]=s3c[row]*invPitch;
                } else {
                    fillFromRoot(k1,r1,activeLo,activeHi,s2m.data());
                    fillFromRoot(k2,r2,activeLo,activeHi,s2p.data());
                    bool needG3=false;
                    for(unsigned row=activeLo;row<=activeHi;++row) {
                        if(!rowActive[row])continue;
                        const double g1=s3c[row]*invPitch,g2=0.5*(s2m[row]+s2p[row])*invPitch;
                        fine[row]=g2;
                        if(std::abs(g2-g1)>1e-8)needG3=true;
                    }
                    if(needG3) {
                        double r3=0.,r4=0.;
                        const int k3=solveYRoot(nodes[3],r3);
                        const int k4=solveYRoot(nodes[4],r4);
                        fillFromRoot(k3,r3,activeLo,activeHi,s3m.data());
                        fillFromRoot(k4,r4,activeLo,activeHi,s3p.data());
                        for(unsigned row=activeLo;row<=activeHi;++row) {
                            if(!rowActive[row])continue;
                            const double g2=fine[row];
                            const double g3=(g3W0*s3c[row]+g3W1*(s3m[row]+s3p[row]))*invPitch;
                            if(std::abs(g3-g2)>1e-9)return false;
                            fine[row]=g3;
                        }
                    }
                }
            }
            for(unsigned sIdx=0;sIdx<subCount;++sIdx) {
                const unsigned row=subRows[sIdx];
                if(!rowActive[row])continue;
                if(!std::isfinite(fine[row])||fine[row]<0||fine[row]>1+1e-9)return false;
                outColVals[sIdx]=std::clamp(fine[row],0.,1.);
            }
            return true;
        };
        std::array<double,128> colTmpVals;
        for(unsigned pk=0;pk<nProbes;++pk) {
            if(!evalColSubset(probeCols[pk],mixedRows.data(),mixedCount,colTmpVals.data()))return false;
            for(unsigned mIdx=0;mIdx<mixedCount;++mIdx)probeVals[mIdx][pk]=colTmpVals[mIdx];
        }
        if(flatHorizon) {
            for(unsigned mIdx=0;mIdx<mixedCount;++mIdx) {
                const double v=probeVals[mIdx][0];
                if(v==0.)continue;
                const unsigned row=mixedRows[mIdx];
                const double* m0=projRowMomentWave[0].data()+static_cast<std::size_t>(row)*count;
                reticleFmadd1(outWave,m0,v,count);
                outDotUniform+=v*rmUnif[0][row];
                outDotTransient+=v*rmTran[0][row];
            }
            return true;
        }
        const double z0=colU[probeCols[0]]*colU[probeCols[0]];
        const double z1=colU[probeCols[1]]*colU[probeCols[1]];
        const double z2=colU[probeCols[2]]*colU[probeCols[2]];
        const double z3=colU[probeCols[3]]*colU[probeCols[3]];
        const double z4=colU[probeCols[4]]*colU[probeCols[4]];
        const double invZ20=1.0/(z2-z0),invZ42=1.0/(z4-z2),invZ40=1.0/(z4-z0);
        // Per-row 5-probe polynomial classification: contract every smooth row
        // immediately via L2-resident x-moments, and evaluate remaining columns
        // ONLY for the few non-quadratic horizon-crossing rows directly into
        // projSymWeightedWeights (0 allocations, 0 IIR filtering, 0 projection).
        std::array<unsigned,128> remRows;
        std::array<unsigned,128> remOrigMIdx;
        unsigned remCount=0;
        for(unsigned mIdx=0;mIdx<mixedCount;++mIdx) {
            const auto& pv=probeVals[mIdx];
            const double d02=(pv[2]-pv[0])*invZ20;
            const double d24=(pv[4]-pv[2])*invZ42;
            const double c4=(d24-d02)*invZ40;
            const double c2=d02-c4*(z0+z2);
            const double c0=pv[0]-z0*(c2+c4*z0);
            if(std::abs(pv[1]-(c0+z1*(c2+c4*z1)))<=1e-10&&std::abs(pv[3]-(c0+z3*(c2+c4*z3)))<=1e-10) {
                const unsigned row=mixedRows[mIdx];
                const std::size_t rOff=static_cast<std::size_t>(row)*count;
                const double* m0=projRowMomentWave[0].data()+rOff;
                if(std::abs(c2)<=1e-14&&std::abs(c4)<=1e-14) {
                    if(c0!=0.) {
                        reticleFmadd1(outWave,m0,c0,count);
                        outDotUniform+=c0*rmUnif[0][row];
                        outDotTransient+=c0*rmTran[0][row];
                    }
                } else if(std::abs(c4)<=1e-14) {
                    const double* m2=projRowMomentWave[2].data()+rOff;
                    reticleFmadd2(outWave,m0,c0,m2,c2,count);
                    outDotUniform+=c0*rmUnif[0][row]+c2*rmUnif[2][row];
                    outDotTransient+=c0*rmTran[0][row]+c2*rmTran[2][row];
                } else {
                    const double* m2=projRowMomentWave[2].data()+rOff;
                    const double* m4=projRowMomentWave[4].data()+rOff;
                    reticleFmadd3(outWave,m0,c0,m2,c2,m4,c4,count);
                    outDotUniform+=c0*rmUnif[0][row]+c2*rmUnif[2][row]+c4*rmUnif[4][row];
                    outDotTransient+=c0*rmTran[0][row]+c2*rmTran[2][row]+c4*rmTran[4][row];
                }
            } else {
                remOrigMIdx[remCount]=mIdx;
                remRows[remCount++]=mixedRows[mIdx];
            }
        }
        if(remCount>0) {
            // Buffer remaining rows column-by-column, then contract row-major
            // so projSymPtr is streamed sequentially through L1/L2 prefetch.
            if(scratchMixedVals.size()<static_cast<std::size_t>(remCount)*colLimit)
                scratchMixedVals.resize(static_cast<std::size_t>(remCount)*colLimit);
            double* remMat=scratchMixedVals.data();
            for(unsigned pk=0;pk<5u;++pk) {
                const unsigned col=probeCols[pk];
                for(unsigned rIdx=0;rIdx<remCount;++rIdx)
                    remMat[static_cast<std::size_t>(rIdx)*colLimit+col]=probeVals[remOrigMIdx[rIdx]][pk];
            }
            for(unsigned col=0;col<colLimit;++col) {
                if(col==probeCols[0]||col==probeCols[1]||col==probeCols[2]||col==probeCols[3]||col==probeCols[4])continue;
                if(!evalColSubset(col,remRows.data(),remCount,colTmpVals.data()))return false;
                for(unsigned rIdx=0;rIdx<remCount;++rIdx)
                    remMat[static_cast<std::size_t>(rIdx)*colLimit+col]=colTmpVals[rIdx];
            }
            for(unsigned rIdx=0;rIdx<remCount;++rIdx) {
                const unsigned row=remRows[rIdx];
                const double* rv=remMat+static_cast<std::size_t>(rIdx)*colLimit;
                const std::size_t symRowBase=static_cast<std::size_t>(row)*symCols;
                for(unsigned col=0;col<colLimit;++col) {
                    const double v=rv[col];
                    if(v==0.)continue;
                    const std::size_t sIdx=symRowBase+col;
                    reticleFmadd1(outWave,projSymPtr+sIdx*count,v,count);
                    outDotUniform+=v*symUnifPtr[sIdx];
                    outDotTransient+=v*symTranPtr[sIdx];
                }
            }
        }
        return true;
    }
};
} // namespace

std::optional<std::vector<double>> ThermalReticleReceiverV1::fastBackgroundRadianceV1(
    const ThermalImageConfigV1& cfg,V pointing,V up,
    double skyRadianceWm2Sr,double groundRadianceWm2Sr,double groundHorizonCosine) {
    const auto prep=UnitGroundPrepV1::build(cfg,pointing,up);
    if(!prep.valid||!prep.monoOk)return std::nullopt;
    return prep.evaluateBackground(skyRadianceWm2Sr,groundRadianceWm2Sr,groundHorizonCosine);
}

ThermalReticleResultV1 ThermalReticleReceiverV1::exposeWithMeasuredPriorV2(
    const ThermalSceneFrameV1& scene,V pointing,std::uint64_t seed,const ThermalReticleMeasuredPriorV2* prior,
    ThermalReticleMovingDiagnosticV2* movingDiagnostic) {
    if(movingDiagnostic)*movingDiagnostic={};
    const auto& c=config_;const auto& p=c.detector;
    if(!validConfig(c)||!precomputed_||!ThermalImageReceiverV1::validScene(scene,p)||scene.timeS<lastTimeS_||!finite(pointing)||norm(pointing)<=0)return {};
    const auto& pre=*precomputed_;
    if(scene.timeS==lastTimeS_)return cached_;
    const auto beforeRng=rng_;const bool beforeSeeded=seeded_;
    const double beforeStarted=startedS_,beforeDetectorTime=lastDetectorSampleTimeS_,beforeCharge=detectorCharge_,beforeExpected=expectedCharge_;
    const auto refuse=[&]() {
        rng_=beforeRng;seeded_=beforeSeeded;startedS_=beforeStarted;lastDetectorSampleTimeS_=beforeDetectorTime;
        detectorCharge_=beforeCharge;expectedCharge_=beforeExpected;return ThermalReticleResultV1{};
    };
    if(!seeded_){rng_.seed(seed^0x52455449434c4531ull);seeded_=true;startedS_=lastDetectorSampleTimeS_=scene.timeS;}
    ThermalReticleResultV1 result;result.valid=true;result.timeS=scene.timeS;result.pointing=unit(pointing);result.horizonDiagnostic.ownUp=scene.up;
    const auto window=captureWindowV1(scene.timeS);
    if(!window){cached_=result;lastTimeS_=scene.timeS;return result;}
    const double start=(*window)[0],end=(*window)[2];
    if(cached_.complete&&start==cached_.exposureStartTimeS){lastTimeS_=scene.timeS;return cached_;}
    result.complete=true;result.exposureStartTimeS=start;result.exposureEndTimeS=end;result.measurementTimeS=.5*(start+end);
    const auto pixels=static_cast<std::size_t>(p.width)*p.height;
    // Equivalent-area engineering electronics. This is not a claim of4096
    // physical read amplifiers or an OEM scalar detector specification.
    const double area=static_cast<double>(pixels),fullWell=area*p.fullWellElectrons;
    const double readNoise=std::sqrt(area)*p.readNoiseElectrons;
    const double dark=area*p.darkCurrentElectronsS*p.integrationTimeS;
    const double adc=fullWell/(std::ldexp(1.,static_cast<int>(p.adcBits))-1);
    result.fullWellElectrons=fullWell;result.readNoiseElectrons=readNoise;result.darkElectrons=dark;result.adcStepElectrons=adc;
    const double gain=p.apertureAreaM2*p.opticalTransmission*p.integrationTimeS*p.quantumEfficiency*p.effectiveWavelengthM/(6.62607015e-34*299792458.);
    const double alpha=-std::expm1(-1/(c.frequencyHz*c.samplesPerScan*p.pixelResponseTimeS));
    auto optical=p;optical.exposureSamples=1;optical.integrationTimeS=p.integrationTimeS/p.exposureSamples;
    std::vector<double> background;
    const auto bgPrep=UnitGroundPrepV1::build(optical,pointing,scene.up);
    if(bgPrep.valid&&bgPrep.monoOk) {
        if(auto fastBg=bgPrep.evaluateBackground(scene.skyRadianceWm2Sr,scene.groundRadianceWm2Sr,scene.groundHorizonCosine))
            background=std::move(*fastBg);
    }
    const std::vector<double>* initialBg=background.empty()?nullptr:&background;
    std::vector<double> phasePhotons(c.samplesPerScan,0.);
    // Conservative FOV visibility filter across the entire scan window [start, end]:
    // patches that stay strictly outside the 9-sigma optical field contribute exact 0
    // to every sub-exposure. Eliminates 512 redundant full-frame renders when only
    // out-of-FOV moving targets (e.g. inside 25 deg launch cone, outside 2.75 deg reticle) exist.
    std::vector<std::size_t> activePatchIndices;
    if(!scene.patches.empty()&&bgPrep.valid) {
        const double fovCornerRad=std::atan(std::hypot(std::tan(std::min(1.55,bgPrep.halfX+bgPrep.margin)),
            std::tan(std::min(1.55,bgPrep.halfY+bgPrep.margin))));
        const double tMid=.5*(start+end);
        const double dtMid=scene.timeS-tMid;
        const double dtHalf=.5*(end-start)+p.integrationTimeS;
        for(std::size_t idx=0;idx<scene.patches.size();++idx) {
            const auto& patch=scene.patches[idx];
            const V midDir=unit(subtract(patch.direction,scale(patch.directionRate,dtMid)));
            const double angDist=std::acos(std::clamp(dot(midDir,bgPrep.forward),-1.,1.));
            const double halfPatch=.5*std::hypot(patch.widthRad,patch.heightRad);
            const double maxMove=norm(patch.directionRate)*dtHalf;
            if(angDist<=fovCornerRad+halfPatch+maxMove+1e-3)activePatchIndices.push_back(idx);
        }
    } else {
        activePatchIndices.resize(scene.patches.size());
        std::iota(activePatchIndices.begin(),activePatchIndices.end(),0);
    }
    const bool stationaryScene=std::all_of(activePatchIndices.begin(),activePatchIndices.end(),[&](std::size_t idx) {
        const auto& patch=scene.patches[idx];
        return patch.directionRate.x==0.&&patch.directionRate.y==0.&&patch.directionRate.z==0.;
    });
    const auto contractStationaryField=[&](const double* radField,std::vector<double>& outPhotons,double addOffset)->bool {
        for(std::size_t i=0;i<pixels;++i)if(!std::isfinite(radField[i]))return false;
        const unsigned count=c.samplesPerScan;
        std::vector<double> acc(count,0.);
        const double* wwBase=pre.weightedWeights.data();
        const double* rwPrefix=pre.rowFullWavePrefix.data();
        const double scale=gain*(p.pixelPitchRad*p.pixelPitchRad);
        std::array<double,128> rowUniformVal{};
        std::array<std::uint8_t,128> rowUniform{};
        for(unsigned row=0;row<p.height;++row) {
            const double* rowPtr=radField+static_cast<std::size_t>(row)*p.width;
            const double v0=std::max(0.,rowPtr[0]);
            bool uniform=true;
            for(unsigned col=1;col<p.width;++col) {
                if(std::max(0.,rowPtr[col])!=v0){uniform=false;break;}
            }
            if(uniform){rowUniform[row]=1u;rowUniformVal[row]=v0;}
        }
        for(unsigned row=0;row<p.height;) {
            if(rowUniform[row]) {
                const double rv=rowUniformVal[row];
                unsigned endRow=row+1u;
                while(endRow<p.height&&rowUniform[endRow]&&rowUniformVal[endRow]==rv)++endRow;
                if(rv!=0.) {
                    const double* p0=rwPrefix+static_cast<std::size_t>(row)*count;
                    const double* p1=rwPrefix+static_cast<std::size_t>(endRow)*count;
                    reticleFmaddDiff(acc.data(),p1,p0,rv,count);
                }
                row=endRow;
            } else {
                const std::size_t rowBase=static_cast<std::size_t>(row)*p.width;
                for(unsigned col=0;col<p.width;++col) {
                    const double rv=std::max(0.,radField[rowBase+col]);
                    if(rv==0.)continue;
                    const double* ww=wwBase+(rowBase+col)*count;
                    reticleFmadd1(acc.data(),ww,rv,count);
                }
                ++row;
            }
        }
        outPhotons.resize(count);
        for(unsigned j=0;j<count;++j) {
            const double photons=acc[j]*scale+addOffset;
            if(!std::isfinite(photons)||photons<0||photons>std::ldexp(1.,52))return false;
            outPhotons[j]=photons;
        }
        return true;
    };
    if(stationaryScene) {
        // When no visible patch moves during the scan (including empty FOV), every
        // sub-exposure has the exact same optical radiance field. Render it
        // once and contract through precomputed sub-exposure-averaged weightedWeights.
        const double* rad=nullptr;
        std::optional<ThermalIncidentFieldV1> field;
        if(activePatchIndices.empty()&&initialBg) {
            rad=initialBg->data();
        } else {
            auto exposure0=scene;
            if(activePatchIndices.size()<scene.patches.size()) {
                exposure0.patches.clear();exposure0.refereeAttributionIndices.clear();
                for(const auto idx:activePatchIndices) {
                    exposure0.patches.push_back(scene.patches[idx]);
                    exposure0.refereeAttributionIndices.push_back(scene.refereeAttributionIndices[idx]);
                }
            }
            const double middle0=start+(.5)/(c.frequencyHz*c.samplesPerScan);
            const double sample0=middle0+p.integrationTimeS*((.5)/p.exposureSamples-.5);
            exposure0.timeS=sample0+.5*optical.integrationTimeS;
            field=ThermalImageReceiverV1::incidentFieldImplV1(exposure0,pointing,optical,initialBg);
            if(!field)return refuse();
            rad=field->radiance.data();
        }
        if(!contractStationaryField(rad,phasePhotons,dark))return refuse();
    } else {
        // Compact scene containing only patches that can enter the reticle FOV.
        auto compactScene=scene;
        if(activePatchIndices.size()<scene.patches.size()) {
            compactScene.patches.clear();compactScene.refereeAttributionIndices.clear();
            for(const auto idx:activePatchIndices) {
                compactScene.patches.push_back(scene.patches[idx]);
                compactScene.refereeAttributionIndices.push_back(scene.refereeAttributionIndices[idx]);
            }
        }
        // Fast exact patch-delta path when background is known and every visible
        // patch run stays strictly on one side of the horizon (e.g. aircraft in sky):
        // computes base background photons ONCE and evaluates only the ~12x12 pixel
        // 9-sigma bounding box per sub-exposure with zero per-phase heap allocations.
        bool fastDeltaReady=false;
        bool bgAllNonNeg=false;
        std::vector<double> baseBgPhotons;
        std::vector<std::size_t> patchOrder;
        if(initialBg&&bgPrep.valid) {
            fastDeltaReady=contractStationaryField(initialBg->data(),baseBgPhotons,0.);
            if(fastDeltaReady) {
                bgAllNonNeg=true;
                const double* bgCheck=initialBg->data();
                for(std::size_t i=0;i<pixels;++i) {
                    if(!std::isfinite(bgCheck[i])||bgCheck[i]<0.){bgAllNonNeg=false;break;}
                }
                patchOrder.resize(compactScene.patches.size());
                std::iota(patchOrder.begin(),patchOrder.end(),0);
                if(patchOrder.size()>1u) {
                    std::sort(patchOrder.begin(),patchOrder.end(),[&](auto aIdx,auto bIdx){
                        const auto key=[](const auto& pt){return std::tuple{pt.rangeM,pt.direction.x,pt.direction.y,pt.direction.z,
                            pt.directionRate.x,pt.directionRate.y,pt.directionRate.z,
                            pt.widthRad,pt.heightRad,pt.radianceWm2Sr,pt.transmission,pt.pathRadianceWm2Sr};};
                        return key(compactScene.patches[aIdx])<key(compactScene.patches[bIdx]);
                    });
                }
            }
        }
        struct FastDeltaRect {double left,right,bottom,top;};
        struct FastDeltaVisible {std::size_t source;FastDeltaRect region;};
        struct FastDeltaScratch {
            std::vector<double> deltaBuf;
            std::vector<std::size_t> touched;
            std::vector<std::optional<FastDeltaRect>> projected;
            std::vector<double> yEdges,xEdges;
            std::vector<std::size_t> active;
            std::vector<FastDeltaVisible> visible,horizontal;
            void ensureMulti(std::size_t nPixels,std::size_t nPatches) {
                if(deltaBuf.size()!=nPixels)deltaBuf.assign(nPixels,0.);
                if(touched.capacity()<256u)touched.reserve(256u);
                if(projected.size()!=nPatches)projected.resize(nPatches);
            }
            void clearTouched() noexcept {
                for(const std::size_t idx:touched)deltaBuf[idx]=0.;
                touched.clear();
            }
        };
        const auto tryFastDeltaPhase=[&](unsigned j,FastDeltaScratch& sc,double& outPhotons)->bool {
            if(!fastDeltaReady)return false;
            const double* bgPtr=initialBg->data();
            const double* sol=pre.solidAngles.data();
            const double* tw=pre.timeWeights.data();
            const double halfW=.5*p.width,halfH=.5*p.height,pitch=p.pixelPitchRad,margin=9*p.psfSigmaRad;
            const double invPitch2=1./(pitch*pitch);
            const double gainPerSample=gain/p.exposureSamples;
            const double middle=start+(j+.5)/(c.frequencyHz*c.samplesPerScan);
            const double before=0.5*optical.integrationTimeS;
            const auto minSq=[](double lo,double hi) noexcept {return lo<=0&&hi>=0?0.:std::min(lo*lo,hi*hi);};
            const bool singlePatch=(compactScene.patches.size()==1u);
            if(!singlePatch)sc.ensureMulti(pixels,compactScene.patches.size());
            std::array<double,128> wx{},wy{};
            double deltaPhotons=0.;
            const auto evalSingleRun=[&](const FastDeltaVisible& run,const double* w) noexcept -> bool {
                const auto& part=run.region;const auto& patch=compactScene.patches[run.source];
                double bgRad=scene.skyRadianceWm2Sr;
                if(scene.skyRadianceWm2Sr!=scene.groundRadianceWm2Sr) {
                    const double tx0=std::tan(part.left),tx1=std::tan(part.right);
                    const double ty0=std::tan(part.bottom),ty1=std::tan(part.top);
                    const double n0=bgPrep.a+std::min(bgPrep.b*tx0,bgPrep.b*tx1)+std::min(bgPrep.c*ty0,bgPrep.c*ty1);
                    const double n1=bgPrep.a+std::max(bgPrep.b*tx0,bgPrep.b*tx1)+std::max(bgPrep.c*ty0,bgPrep.c*ty1);
                    const double d0=std::sqrt(1+minSq(tx0,tx1)+minSq(ty0,ty1));
                    const double d1=std::sqrt(1+std::max(tx0*tx0,tx1*tx1)+std::max(ty0*ty0,ty1*ty1));
                    const double minimum=std::min({n0/d0,n0/d1,n1/d0,n1/d1});
                    const double maximum=std::max({n0/d0,n0/d1,n1/d0,n1/d1});
                    if(minimum>=scene.groundHorizonCosine)bgRad=scene.skyRadianceWm2Sr;
                    else if(maximum<scene.groundHorizonCosine)bgRad=scene.groundRadianceWm2Sr;
                    else return false;
                }
                const double lightAtSensor=patch.transmission*patch.radianceWm2Sr+(1-patch.transmission)*patch.pathRadianceWm2Sr;
                const double netRad=(lightAtSensor-bgRad)*invPitch2;
                if(netRad==0.)return true;
                if(!std::isfinite(netRad))return false;
                const int firstX=std::max(0,static_cast<int>(std::floor((part.left-margin)/pitch+halfW)));
                const int lastX=std::min(static_cast<int>(p.width)-1,static_cast<int>(std::floor((part.right+margin)/pitch+halfW)));
                const int firstY=std::max(0,static_cast<int>(std::floor((part.bottom-margin)/pitch+halfH)));
                const int lastY=std::min(static_cast<int>(p.height)-1,static_cast<int>(std::floor((part.top+margin)/pitch+halfH)));
                if(firstX>lastX||firstY>lastY)return true;
                for(int x=firstX;x<=lastX;++x)wx[static_cast<unsigned>(x)]=gaussianIntervalFastV1(part.left,part.right,(x-halfW)*pitch,(x+1-halfW)*pitch,p.psfSigmaRad);
                for(int y=firstY;y<=lastY;++y)wy[static_cast<unsigned>(y)]=gaussianIntervalFastV1(part.bottom,part.top,(y-halfH)*pitch,(y+1-halfH)*pitch,p.psfSigmaRad);
                if(bgAllNonNeg&&netRad>0.) {
                    for(int y=firstY;y<=lastY;++y) {
                        const double wyScale=wy[static_cast<unsigned>(y)]*netRad*gainPerSample;
                        if(wyScale==0.)continue;
                        const std::size_t rowBase=static_cast<std::size_t>(y)*p.width;
                        const double* solRow=sol+rowBase;
                        const double* wRow=w+rowBase;
                        double rowSum=0.;
                        for(int x=firstX;x<=lastX;++x)rowSum+=wx[static_cast<unsigned>(x)]*solRow[x]*wRow[x];
                        deltaPhotons+=wyScale*rowSum;
                    }
                } else {
                    for(int y=firstY;y<=lastY;++y) {
                        const double wyv=wy[static_cast<unsigned>(y)]*netRad;
                        if(wyv==0.)continue;
                        const std::size_t rowBase=static_cast<std::size_t>(y)*p.width;
                        for(int x=firstX;x<=lastX;++x) {
                            const double wi=w[rowBase+static_cast<unsigned>(x)];
                            if(wi==0.)continue;
                            const double dv=wyv*wx[static_cast<unsigned>(x)];
                            const double bgv=bgPtr[rowBase+static_cast<unsigned>(x)];
                            const double newRad=bgv+dv;
                            if(!std::isfinite(newRad))return false;
                            deltaPhotons+=(std::max(0.,newRad)-std::max(0.,bgv))*sol[rowBase+static_cast<unsigned>(x)]*wi*gainPerSample;
                        }
                    }
                }
                return true;
            };
            for(unsigned k=0;k<p.exposureSamples;++k) {
                const double sample=middle+p.integrationTimeS*((k+.5)/p.exposureSamples-.5);
                const double expTime=sample+.5*optical.integrationTimeS;
                const double dtShift=scene.timeS-expTime;
                const auto offset=(static_cast<std::size_t>(j)*p.exposureSamples+k)*pixels;
                const double* w=tw+offset;
                if(singlePatch) {
                    const auto& patch=compactScene.patches[0];
                    const V dir0=unit(subtract(patch.direction,scale(patch.directionRate,dtShift)));
                    const V dir=unit(subtract(dir0,scale(patch.directionRate,before)));
                    const double fwd=dot(dir,bgPrep.forward);if(fwd<=0)continue;
                    const double ax=std::atan2(dot(dir,bgPrep.right),fwd),ay=std::atan2(dot(dir,bgPrep.vertical),fwd);
                    const FastDeltaRect rect{std::max({-1.55,-halfW*pitch-margin,ax-.5*patch.widthRad}),
                        std::min({1.55,halfW*pitch+margin,ax+.5*patch.widthRad}),
                        std::max({-1.55,-halfH*pitch-margin,ay-.5*patch.heightRad}),
                        std::min({1.55,halfH*pitch+margin,ay+.5*patch.heightRad})};
                    if(rect.left>=rect.right||rect.bottom>=rect.top)continue;
                    if(!evalSingleRun(FastDeltaVisible{0u,rect},w))return false;
                    continue;
                }
                sc.yEdges.clear();
                for(auto& pr:sc.projected)pr.reset();
                for(const auto idx:patchOrder) {
                    const auto& patch=compactScene.patches[idx];
                    const V dir0=unit(subtract(patch.direction,scale(patch.directionRate,dtShift)));
                    const V dir=unit(subtract(dir0,scale(patch.directionRate,before)));
                    const double fwd=dot(dir,bgPrep.forward);if(fwd<=0)continue;
                    const double ax=std::atan2(dot(dir,bgPrep.right),fwd),ay=std::atan2(dot(dir,bgPrep.vertical),fwd);
                    const FastDeltaRect rect{std::max({-1.55,-halfW*pitch-margin,ax-.5*patch.widthRad}),
                        std::min({1.55,halfW*pitch+margin,ax+.5*patch.widthRad}),
                        std::max({-1.55,-halfH*pitch-margin,ay-.5*patch.heightRad}),
                        std::min({1.55,halfH*pitch+margin,ay+.5*patch.heightRad})};
                    if(rect.left>=rect.right||rect.bottom>=rect.top)continue;
                    sc.projected[idx]=rect;sc.yEdges.push_back(rect.bottom);sc.yEdges.push_back(rect.top);
                }
                if(sc.yEdges.empty())continue;
                std::sort(sc.yEdges.begin(),sc.yEdges.end());sc.yEdges.erase(std::unique(sc.yEdges.begin(),sc.yEdges.end()),sc.yEdges.end());
                sc.visible.clear();
                for(std::size_t row=1;row<sc.yEdges.size();++row) {
                    const double bottom=sc.yEdges[row-1],top=sc.yEdges[row],midY=.5*(bottom+top);
                    sc.active.clear();sc.xEdges.clear();
                    for(const auto idx:patchOrder)if(sc.projected[idx]&&sc.projected[idx]->bottom<midY&&sc.projected[idx]->top>midY) {
                        sc.active.push_back(idx);sc.xEdges.push_back(sc.projected[idx]->left);sc.xEdges.push_back(sc.projected[idx]->right);
                    }
                    std::sort(sc.xEdges.begin(),sc.xEdges.end());sc.xEdges.erase(std::unique(sc.xEdges.begin(),sc.xEdges.end()),sc.xEdges.end());
                    sc.horizontal.clear();
                    for(std::size_t col=1;col<sc.xEdges.size();++col) {
                        const double left=sc.xEdges[col-1],right=sc.xEdges[col],midX=.5*(left+right);
                        std::optional<std::size_t> owner;
                        for(const auto idx:sc.active)if(sc.projected[idx]->left<midX&&sc.projected[idx]->right>midX){owner=idx;break;}
                        if(!owner)continue;
                        if(!sc.horizontal.empty()&&sc.horizontal.back().source==*owner&&sc.horizontal.back().region.right==left)
                            sc.horizontal.back().region.right=right;
                        else sc.horizontal.push_back({*owner,{left,right,bottom,top}});
                    }
                    for(const auto& run:sc.horizontal) {
                        bool merged=false;
                        for(auto& prev:sc.visible) {
                            if(prev.source==run.source&&prev.region.left==run.region.left&&
                               prev.region.right==run.region.right&&prev.region.top==bottom) {
                                prev.region.top=top;merged=true;break;
                            }
                        }
                        if(!merged)sc.visible.push_back(run);
                    }
                }
                if(sc.visible.size()==1u) {
                    if(!evalSingleRun(sc.visible[0],w))return false;
                    continue;
                }
                sc.touched.clear();
                for(const auto& run:sc.visible) {
                    const auto& part=run.region;const auto& patch=compactScene.patches[run.source];
                    double bgRad=scene.skyRadianceWm2Sr;
                    if(scene.skyRadianceWm2Sr!=scene.groundRadianceWm2Sr) {
                        const double tx0=std::tan(part.left),tx1=std::tan(part.right);
                        const double ty0=std::tan(part.bottom),ty1=std::tan(part.top);
                        const double n0=bgPrep.a+std::min(bgPrep.b*tx0,bgPrep.b*tx1)+std::min(bgPrep.c*ty0,bgPrep.c*ty1);
                        const double n1=bgPrep.a+std::max(bgPrep.b*tx0,bgPrep.b*tx1)+std::max(bgPrep.c*ty0,bgPrep.c*ty1);
                        const double d0=std::sqrt(1+minSq(tx0,tx1)+minSq(ty0,ty1));
                        const double d1=std::sqrt(1+std::max(tx0*tx0,tx1*tx1)+std::max(ty0*ty0,ty1*ty1));
                        const double minimum=std::min({n0/d0,n0/d1,n1/d0,n1/d1});
                        const double maximum=std::max({n0/d0,n0/d1,n1/d0,n1/d1});
                        if(minimum>=scene.groundHorizonCosine)bgRad=scene.skyRadianceWm2Sr;
                        else if(maximum<scene.groundHorizonCosine)bgRad=scene.groundRadianceWm2Sr;
                        else {sc.clearTouched();return false;}
                    }
                    const double lightAtSensor=patch.transmission*patch.radianceWm2Sr+(1-patch.transmission)*patch.pathRadianceWm2Sr;
                    const double netRad=(lightAtSensor-bgRad)*invPitch2;
                    if(netRad==0.)continue;
                    const int firstX=std::max(0,static_cast<int>(std::floor((part.left-margin)/pitch+halfW)));
                    const int lastX=std::min(static_cast<int>(p.width)-1,static_cast<int>(std::floor((part.right+margin)/pitch+halfW)));
                    const int firstY=std::max(0,static_cast<int>(std::floor((part.bottom-margin)/pitch+halfH)));
                    const int lastY=std::min(static_cast<int>(p.height)-1,static_cast<int>(std::floor((part.top+margin)/pitch+halfH)));
                    for(int x=firstX;x<=lastX;++x)wx[static_cast<unsigned>(x)]=gaussianIntervalFastV1(part.left,part.right,(x-halfW)*pitch,(x+1-halfW)*pitch,p.psfSigmaRad);
                    for(int y=firstY;y<=lastY;++y)wy[static_cast<unsigned>(y)]=gaussianIntervalFastV1(part.bottom,part.top,(y-halfH)*pitch,(y+1-halfH)*pitch,p.psfSigmaRad);
                    for(int y=firstY;y<=lastY;++y) {
                        const double wyv=wy[static_cast<unsigned>(y)]*netRad;
                        if(wyv==0.)continue;
                        const std::size_t rowBase=static_cast<std::size_t>(y)*p.width;
                        for(int x=firstX;x<=lastX;++x) {
                            const double dVal=wyv*wx[static_cast<unsigned>(x)];
                            if(dVal==0.)continue;
                            const std::size_t idx=rowBase+static_cast<unsigned>(x);
                            if(sc.deltaBuf[idx]==0.)sc.touched.push_back(idx);
                            sc.deltaBuf[idx]+=dVal;
                        }
                    }
                }
                for(const std::size_t idx:sc.touched) {
                    const double dv=sc.deltaBuf[idx];
                    sc.deltaBuf[idx]=0.;
                    const double wi=w[idx];
                    if(wi==0.)continue;
                    const double newRad=bgPtr[idx]+dv;
                    if(!std::isfinite(newRad)){sc.clearTouched();return false;}
                    deltaPhotons+=(std::max(0.,newRad)-std::max(0.,bgPtr[idx]))*sol[idx]*wi*gainPerSample;
                }
                sc.touched.clear();
            }
            const double total=baseBgPhotons[j]+deltaPhotons+dark;
            if(!std::isfinite(total)||total<0||total>std::ldexp(1.,52))return false;
            outPhotons=total;
            return true;
        };
        const auto computePhasePhotons=[&](unsigned j,FastDeltaScratch& sc,const std::vector<double>* bg,std::vector<double>* outBg,double& outPhotons)->bool {
            if(tryFastDeltaPhase(j,sc,outPhotons))return true;
            double photons=0;
            const double middle=start+(j+.5)/(c.frequencyHz*c.samplesPerScan);
            for(unsigned k=0;k<p.exposureSamples;++k) {
                const double sample=middle+p.integrationTimeS*((k+.5)/p.exposureSamples-.5);
                auto exposure=compactScene;exposure.timeS=sample+.5*optical.integrationTimeS;
                for(auto& patch:exposure.patches)patch.direction=unit(subtract(patch.direction,scale(patch.directionRate,scene.timeS-exposure.timeS)));
                const std::vector<double>* useBg=(bg&&!bg->empty())?bg:(outBg&&!outBg->empty()?outBg:nullptr);
                const auto field=ThermalImageReceiverV1::incidentFieldImplV1(exposure,pointing,optical,useBg);
                if(!field)return false;
                if(!useBg&&outBg)*outBg=field->backgroundRadiance;
                const auto offset=(static_cast<std::size_t>(j)*p.exposureSamples+k)*pixels;
                for(std::size_t i=0;i<pixels;++i) {
                    if(!std::isfinite(field->radiance[i]))return false;
                    photons+=std::max(0.,field->radiance[i])*pre.solidAngles[i]*pre.timeWeights[offset+i]*gain/p.exposureSamples;
                }
            }
            photons+=dark;
            if(!std::isfinite(photons)||photons<0||photons>std::ldexp(1.,52))return false;
            outPhotons=photons;
            return true;
        };
        FastDeltaScratch mainScratch;
        if(!computePhasePhotons(0u,mainScratch,initialBg,&background,phasePhotons[0]))return refuse();
        if(c.samplesPerScan>1u) {
            const unsigned hw=gReticleNestedPoolBlocked
                ?1u:std::min(16u,std::max(1u,std::thread::hardware_concurrency()));
            std::fenv_t callerEnv{};
            if(!scene.patches.empty()&&hw>=2u&&std::fegetenv(&callerEnv)==0) {
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
                const unsigned callerCsr=_mm_getcsr();
#endif
#if defined(__GNUC__) && (defined(__i386__) || defined(__x86_64__))
                unsigned short callerCw=0;__asm__ __volatile__("fnstcw %0":"=m"(callerCw));
#endif
                const unsigned workers=std::min(hw,c.samplesPerScan-1u);
                std::atomic<unsigned> nextPhase{1u};
                std::atomic<bool> failed{false};
                std::array<std::exception_ptr,16> errs{};
                const auto runForward=[&](unsigned worker) noexcept {
                    try {
                        if(worker!=0u) {
                            if(std::fesetenv(&callerEnv)!=0){failed.store(true,std::memory_order_relaxed);return;}
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
                            _mm_setcsr(callerCsr);
#endif
#if defined(__GNUC__) && (defined(__i386__) || defined(__x86_64__))
                            __asm__ __volatile__("fldcw %0"::"m"(callerCw));
#endif
                        }
                        FastDeltaScratch workerScratch;
                        while(!failed.load(std::memory_order_relaxed)) {
                            const unsigned j=nextPhase.fetch_add(1u,std::memory_order_relaxed);
                            if(j>=c.samplesPerScan)break;
                            if(!computePhasePhotons(j,workerScratch,&background,nullptr,phasePhotons[j])) {
                                failed.store(true,std::memory_order_relaxed);break;
                            }
                        }
                    } catch(...) {errs[worker]=std::current_exception();failed.store(true,std::memory_order_relaxed);}
                };
                threadLocalReticlePool().run(workers,runForward);
                for(const auto& e:errs)if(e)std::rethrow_exception(e);
                if(failed.load(std::memory_order_relaxed))return refuse();
            } else {
                for(unsigned j=1u;j<c.samplesPerScan;++j) {
                    if(!computePhasePhotons(j,mainScratch,&background,nullptr,phasePhotons[j]))return refuse();
                }
            }
        }
    }
    double maximumMeasured=0;
    for(unsigned j=0;j<c.samplesPerScan;++j) {
        const double photons=phasePhotons[j];
        const double middle=start+(j+.5)/(c.frequencyHz*c.samplesPerScan);
        result.incidentElectrons.push_back(photons);
        // Exact first-order ZOH update for the actual elapsed detector clock,
        // including the unobserved interval between requested readout windows.
        // Its unknown initial transient is a DSP nuisance, never a bearing.
        const double elapsed=middle-lastDetectorSampleTimeS_;
        if(!(elapsed>0))return refuse();
        const double response=-std::expm1(-elapsed/p.pixelResponseTimeS);
        expectedCharge_+=response*(photons-expectedCharge_);result.expectedElectrons.push_back(expectedCharge_);
        double shot=0;
        // mean==0 is a deterministic zero, not a violation of the library's
        // strict positive-mean constructor precondition. Full well belongs
        // ONLY to the post-response readout below, never to incident charge.
        if(photons>0){std::poisson_distribution<std::uint64_t> draw(photons);shot=static_cast<double>(draw(rng_));}
        detectorCharge_+=response*(shot-detectorCharge_);lastDetectorSampleTimeS_=middle;
        const double raw=detectorCharge_+normal()*readNoise;
        result.saturated=result.saturated||raw<=0||raw>=fullWell;
        const double measured=std::round(std::clamp(raw,0.,fullWell)/adc)*adc;
        result.measuredElectrons.push_back(measured);result.phases.push_back(2*pi*(j+.5)/c.samplesPerScan);
        maximumMeasured=std::max(maximumMeasured,measured);
    }
    // An explicit conservative Gaussian high-count screening covariance:
    // stable first-order convolution has l2 gain<=1. Estimate the input upper
    // scale from MEASURED ADC charge and finite response/noise, never photons.
    const double inputUpper=(maximumMeasured+p.detectionSigma*(std::sqrt(maximumMeasured)+readNoise)+adc)/alpha;
    result.varianceBoundElectrons2=inputUpper+readNoise*readNoise+adc*adc/12;
    if(!result.saturated)inferMeasuredWithPriorV2(result,pointing,scene.up,prior,movingDiagnostic);
    if(norm(result.horizonDiagnostic.ownUp)<=0)result.horizonDiagnostic.ownUp=scene.up;
    cached_=result;lastTimeS_=scene.timeS;return result;
}

void ThermalReticleReceiverV1::inferStaticHorizonForCalibrationV2(ThermalReticleResultV1& result,V pointing,V up,
    const ThermalReticleMeasuredPriorV2* prior) const {
    result.horizonDiagnostic={};
    result.features.clear();result.bearingResolved=false;result.preciseBearing=false;
    result.fitSnr=0;result.fitResidual=0;result.compactFitSnr=0;result.compactFitResidual=0;
    result.fittedCentreXRad=0;result.fittedCentreYRad=0;result.fittedWidthRad=0;result.fittedHeightRad=0;
    result.ambiguousAlternativeKind=0;result.alternativeRadianceAvailable=false;result.alternativeShape={};
    result.alternativeAmplitude=0;result.alternativeCostDifference=0;result.alternativeSky=0;result.alternativeSource=0;
    if(!result.valid||!result.complete||result.saturated||!precomputed_||precomputed_->weights.empty()||
       result.measuredElectrons.size()!=config_.samplesPerScan||!std::isfinite(result.varianceBoundElectrons2)||
       result.varianceBoundElectrons2<=0||!finite(pointing)||norm(pointing)<=0||!finite(up)||std::abs(norm(up)-1)>1e-6||
       !std::all_of(result.measuredElectrons.begin(),result.measuredElectrons.end(),[](double v){return std::isfinite(v)&&v>=0;}))return;
    fitHorizonWithMeasuredPriorV2(result,pointing,up,nullptr,false,prior);
}

void ThermalReticleReceiverV1::inferMeasuredV1(ThermalReticleResultV1& result,V pointing,V up) const {
    inferMeasuredWithPriorV2(result,pointing,up,nullptr);
}

void ThermalReticleReceiverV1::inferMeasuredWithPriorV2(ThermalReticleResultV1& result,V pointing,V up,
    const ThermalReticleMeasuredPriorV2* prior) const {
    inferMeasuredWithPriorV2(result,pointing,up,prior,nullptr);
}

void ThermalReticleReceiverV1::inferMeasuredWithPriorV2(ThermalReticleResultV1& result,V pointing,V up,
    const ThermalReticleMeasuredPriorV2* prior,ThermalReticleMovingDiagnosticV2* movingDiagnostic) const {
    if(movingDiagnostic)*movingDiagnostic={};
    result.horizonDiagnostic={};
    result.features.clear();result.bearingResolved=false;result.preciseBearing=false;
    result.fitSnr=0;result.fitResidual=0;result.compactFitSnr=0;result.compactFitResidual=0;
    result.fittedCentreXRad=0;result.fittedCentreYRad=0;result.fittedWidthRad=0;result.fittedHeightRad=0;
    result.ambiguousAlternativeKind=0;result.alternativeRadianceAvailable=false;result.alternativeShape={};
    result.alternativeAmplitude=0;result.alternativeCostDifference=0;result.alternativeSky=0;result.alternativeSource=0;
    if(!result.valid||!result.complete||result.saturated||!precomputed_||precomputed_->weights.empty()||
       result.measuredElectrons.size()!=config_.samplesPerScan||!std::isfinite(result.varianceBoundElectrons2)||
       result.varianceBoundElectrons2<=0||!finite(pointing)||norm(pointing)<=0||!finite(up)||std::abs(norm(up)-1)>1e-6||
       !std::all_of(result.measuredElectrons.begin(),result.measuredElectrons.end(),[](double v){return std::isfinite(v)&&v>=0;}))return;
    fit(result,pointing,up);
    if(!result.features.empty()) {
        auto hypothesis=result;hypothesis.features.clear();hypothesis.bearingResolved=false;hypothesis.preciseBearing=false;
        bool backgroundExplained=false;
        fitHorizonWithMeasuredPriorV2(hypothesis,pointing,up,&backgroundExplained,true,prior);
        result.horizonDiagnostic=hypothesis.horizonDiagnostic;
        if(backgroundExplained||hypothesis.horizonDiagnostic.refusal==21)result=std::move(hypothesis);
    } else if(result.features.empty())fitHorizonWithMeasuredPriorV2(result,pointing,up,nullptr,false,prior);
    // The stationary family and ALL its thresholds/diagnostics above remain
    // unchanged. A separately versioned measured-only family handles source
    // motion during an otherwise complete scalar exposure.
    if(result.features.empty()&&result.horizonDiagnostic.refusal!=2&&result.horizonDiagnostic.refusal!=5&&
       result.horizonDiagnostic.refusal!=21)inferMovingMeasuredWithPriorV2(result,pointing,up,prior,movingDiagnostic);
}

void ThermalReticleReceiverV1::inferMovingMeasuredV2(ThermalReticleResultV1& result,V pointing,V up,
    ThermalReticleMovingDiagnosticV2* diagnostic) const {
    inferMovingMeasuredWithPriorV2(result,pointing,up,nullptr,diagnostic);
}

void ThermalReticleReceiverV1::inferMovingMeasuredWithPriorV2(ThermalReticleResultV1& result,V pointing,V up,
    const ThermalReticleMeasuredPriorV2* prior,ThermalReticleMovingDiagnosticV2* diagnostic) const {
    if(!precomputed_)return;
    const auto& pre=*precomputed_;
    const moving_radiator_v2::Optics optics{config_,pre.weights,pre.timeWeights,pre.solidAngles,pre.flat,pre.nuisance,
        [this](const std::array<double,4>& q){return spot(q[0],q[1],config_.detector.psfSigmaRad,q[2],q[3],false);}};
    moving_radiator_v2::infer(optics,result,pointing,up,diagnostic,prior);
}

std::optional<std::array<std::vector<double>,2>> ThermalReticleReceiverV1::movingBasisForCalibrationV2(
    const ThermalReticleResultV1& clock,V pointing,V up,const std::array<double,7>& shape,unsigned branch) const {
    if(!precomputed_)return std::nullopt;
    const auto& pre=*precomputed_;
    const moving_radiator_v2::Optics optics{config_,pre.weights,pre.timeWeights,pre.solidAngles,pre.flat,pre.nuisance,
        [this](const std::array<double,4>& q){return spot(q[0],q[1],config_.detector.psfSigmaRad,q[2],q[3],false);}};
    return moving_radiator_v2::calibrateBasis(optics,clock,pointing,up,shape,branch);
}

std::optional<std::array<std::vector<double>,2>> ThermalReticleReceiverV1::movingBasisForCalibrationV4(
    const ThermalReticleResultV1& clock,V pointing,V up,const std::array<double,7>& shape,unsigned branch) const {
    if(!precomputed_)return std::nullopt;
    const auto& pre=*precomputed_;
    const moving_radiator_v2::Optics optics{config_,pre.weights,pre.timeWeights,pre.solidAngles,pre.flat,pre.nuisance,
        [this](const std::array<double,4>& q){return spot(q[0],q[1],config_.detector.psfSigmaRad,q[2],q[3],false);}};
    return moving_radiator_v2::calibrateBasisV4(optics,clock,pointing,up,shape,branch);
}

std::optional<std::array<std::vector<double>,2>> ThermalReticleReceiverV1::movingBasisForCalibrationV5(
    const ThermalReticleResultV1& clock,V pointing,V up,const std::array<double,7>& shape,unsigned branch) const {
    if(!precomputed_)return std::nullopt;
    const auto& pre=*precomputed_;
    const moving_radiator_v2::Optics optics{config_,pre.weights,pre.timeWeights,pre.solidAngles,pre.flat,pre.nuisance,
        [this](const std::array<double,4>& q){return spot(q[0],q[1],config_.detector.psfSigmaRad,q[2],q[3],false);}};
    return moving_radiator_v2::calibrateBasisV5(optics,clock,pointing,up,shape,branch);
}

std::optional<std::array<double,2>> ThermalReticleReceiverV1::rawSkyFitV1(
    const ThermalReticleResultV1& result,const std::vector<double>& rawTemplate) const {
    const auto& c=config_;const auto& p=c.detector;const auto count=c.samplesPerScan;
    if(!precomputed_||rawTemplate.size()!=count||result.measuredElectrons.size()!=count)return std::nullopt;
    const double alpha=-std::expm1(-1/(c.frequencyHz*count*p.pixelResponseTimeS));
    auto uniform=filtered(precomputed_->flat,alpha);for(auto& value:uniform)value/=p.pixelPitchRad*p.pixelPitchRad;
    std::vector<double> transient(count),remaining=result.measuredElectrons;
    double dark=0;const double input=static_cast<double>(p.width)*p.height*p.darkCurrentElectronsS*p.integrationTimeS;
    for(unsigned j=0;j<count;++j){transient[j]=std::pow(1-alpha,j+1);dark+=alpha*(input-dark);remaining[j]-=dark;}
    const double uu=inner(uniform,uniform),tt=inner(transient,transient),ut=inner(uniform,transient),det=uu*tt-ut*ut;
    if(!(uu>0)||!std::isfinite(uu)||!std::isfinite(tt))return std::nullopt;
    if(tt>0&&!(det>128*std::numeric_limits<double>::epsilon()*uu*tt))return std::nullopt;
    const double sky=tt>0?(inner(remaining,uniform)*tt-inner(remaining,transient)*ut)/det:inner(remaining,uniform)/uu;
    const double projection=tt>0?(inner(rawTemplate,uniform)*tt-inner(rawTemplate,transient)*ut)/det:inner(rawTemplate,uniform)/uu;
    if(!std::isfinite(sky)||!std::isfinite(projection))return std::nullopt;
    return std::array<double,2>{{sky,projection}};
}

std::optional<double> ThermalReticleReceiverV1::physicalSourceAmplitudeV1(const ThermalReticleResultV1& result,const P4& q,double amplitude) const {
    const auto& p=config_.detector;
    const auto budget=rawSkyFitV1(result,spot(q[0],q[1],p.psfSigmaRad,q[2],q[3],false));
    if(!budget)return std::nullopt;
    const double cells=q[2]*q[3]/(p.pixelPitchRad*p.pixelPitchRad);
    if(cells>0) {
        const double divided=amplitude/cells;
        const auto physical=physicalBackgroundAmplitudeV1((*budget)[0],(*budget)[1]*cells,divided);
        if(!physical)return std::nullopt;
        if(*physical==divided)return amplitude;
        const double value=*physical*cells;
        if(!std::isfinite(value))return std::nullopt;
        return value;
    }
    double low=0,high=std::numeric_limits<double>::infinity();
    if((*budget)[1]>0)high=(*budget)[0]/(*budget)[1];
    else if((*budget)[1]<0)low=std::max(0.,(*budget)[0]/(*budget)[1]);
    else if((*budget)[0]<0)return std::nullopt;
    if(low>high)return std::nullopt;
    const double value=std::clamp(amplitude,low,high);
    if(!std::isfinite(value)||(*budget)[0]-(*budget)[1]*value<0)return std::nullopt;
    return value;
}

std::optional<double> ThermalReticleReceiverV1::localAngularVarianceV1(const P4& q,double amplitude,double variance,double lattice) const {
    const auto& c=config_;const auto& p=c.detector;const auto count=c.samplesPerScan;
    const double h=p.pixelPitchRad/1024;
    auto xp=spot(q[0]+h,q[1],p.psfSigmaRad,q[2],q[3]),xm=spot(q[0]-h,q[1],p.psfSigmaRad,q[2],q[3]);
    auto yp=spot(q[0],q[1]+h,p.psfSigmaRad,q[2],q[3]),ym=spot(q[0],q[1]-h,p.psfSigmaRad,q[2],q[3]);
    const auto centre=spot(q[0],q[1],p.psfSigmaRad,q[2],q[3]);const double norm2=inner(centre,centre);
    for(unsigned j=0;j<count;++j){xp[j]=(xp[j]-xm[j])/(2*h);yp[j]=(yp[j]-ym[j])/(2*h);}
    // Eliminate fitted support widths as nuisance parameters as well as the
    // unknown amplitude. An extent is a waveform fit, never a source oracle.
    std::vector<std::vector<double>> shape;
    for(unsigned coordinate=0;coordinate<2;++coordinate) {
        if((coordinate?q[3]:q[2])<=h)continue;
        auto a=spot(q[0],q[1],p.psfSigmaRad,q[2]+(coordinate?0:h),q[3]+(coordinate?h:0));
        const auto b=spot(q[0],q[1],p.psfSigmaRad,q[2]-(coordinate?0:h),q[3]-(coordinate?h:0));
        for(unsigned j=0;j<count;++j)a[j]=(a[j]-b[j])/(2*h);
        const double amplitudeProjection=inner(a,centre)/norm2;
        for(unsigned j=0;j<count;++j)a[j]-=amplitudeProjection*centre[j];
        for(const auto& previous:shape){const double coefficient=inner(a,previous);for(unsigned j=0;j<count;++j)a[j]-=coefficient*previous[j];}
        const double length=std::sqrt(inner(a,a));
        if(length>128*std::numeric_limits<double>::epsilon()) {for(auto& value:a)value/=length;shape.push_back(std::move(a));}
    }
    for(const auto& column:shape) {
        const double a=inner(xp,column),b=inner(yp,column);
        for(unsigned j=0;j<count;++j){xp[j]-=a*column[j];yp[j]-=b*column[j];}
    }
    const double xx=inner(xp,xp)-std::pow(inner(xp,centre),2)/norm2;
    const double yy=inner(yp,yp)-std::pow(inner(yp,centre),2)/norm2;
    const double xy=inner(xp,yp)-inner(xp,centre)*inner(yp,centre)/norm2;
    const double determinant=xx*yy-xy*xy,trace=xx+yy;
    if(!std::isfinite(determinant)||determinant<=128*std::numeric_limits<double>::epsilon()*trace*trace||xx<=0||yy<=0)return std::nullopt;
    const double tx=std::tan(q[0]),ty=std::tan(q[1]),n2=1+tx*tx+ty*ty;
    const double gx=(1+tx*tx)*(1+tx*tx)*(1+ty*ty)/(n2*n2);
    const double gy=(1+ty*ty)*(1+ty*ty)*(1+tx*tx)/(n2*n2);
    const double gxy=-tx*ty*(1+tx*tx)*(1+ty*ty)/(n2*n2);
    const double angularMetric=.5*(gx+gy+std::hypot(gx-gy,2*gxy));
    const double value=(variance*(trace+std::hypot(xx-yy,2*xy))/(2*determinant*amplitude*amplitude)+lattice*lattice/12)*angularMetric;
    if(!std::isfinite(value)||value<=0)return std::nullopt;
    return value;
}

void ThermalReticleReceiverV1::fit(ThermalReticleResultV1& result,V pointing,V up) const {
    if(!precomputed_)return;
    const auto& pre=*precomputed_;
    const auto& c=config_;const auto& p=c.detector;const auto count=c.samplesPerScan;
    auto data=result.measuredElectrons;project(data);
    const double dataNorm=inner(data,data),variance=result.varianceBoundElectrons2;
    if(!std::isfinite(dataNorm)||!std::isfinite(variance)||variance<=0)return;
    double best=0,bestX=0,bestY=0,bestWidth=0,bestHeight=0,bestAmplitude=0,bestNorm=0;
    struct Seed {double score;P4 shape;};std::vector<Seed> supportSeeds;
    const double* dataPtr=data.data();
    const auto consider=[&](const double* values,double x,double y,double width,double height) {
        double norm2=0,correlation=0;
        reticleNormDot(dataPtr,values,count,norm2,correlation);
        if(norm2<=128*std::numeric_limits<double>::epsilon())return;
        const double explained=correlation*correlation/norm2;
        if(explained>best){best=explained;bestX=x;bestY=y;bestWidth=width;bestHeight=height;bestAmplitude=correlation/norm2;bestNorm=norm2;}
    };
    const double* tmplBase=pre.templates.data();
    const double* invN2Base=pre.templateInvNorm2.data();
    for(unsigned row=0;row<p.height;++row) {
        const std::size_t rowBase=static_cast<std::size_t>(row)*p.width;
        const double yCoord=(row+.5-.5*p.height)*p.pixelPitchRad;
        unsigned col=0;
        for(;col+4u<=p.width;col+=4u) {
            const double i0=invN2Base[rowBase+col],i1=invN2Base[rowBase+col+1u];
            const double i2=invN2Base[rowBase+col+2u],i3=invN2Base[rowBase+col+3u];
            if(i0==0.&&i1==0.&&i2==0.&&i3==0.)continue;
            double corrs[4];
            reticleDot4(dataPtr,
                tmplBase+(rowBase+col)*count,tmplBase+(rowBase+col+1u)*count,
                tmplBase+(rowBase+col+2u)*count,tmplBase+(rowBase+col+3u)*count,
                count,corrs);
            const double invs[4]={i0,i1,i2,i3};
            for(unsigned k=0;k<4u;++k) {
                if(invs[k]==0.)continue;
                const double explained=corrs[k]*corrs[k]*invs[k];
                if(explained>best) {
                    best=explained;
                    bestX=((col+k)+.5-.5*p.width)*p.pixelPitchRad;bestY=yCoord;
                    bestWidth=0;bestHeight=0;bestAmplitude=corrs[k]*invs[k];bestNorm=1.0/invs[k];
                }
            }
        }
        for(;col<p.width;++col) {
            const auto index=rowBase+col;
            const double invN2=invN2Base[index];
            if(invN2==0.)continue;
            const double correlation=reticleDot(dataPtr,tmplBase+index*count,count);
            const double explained=correlation*correlation*invN2;
            if(explained>best) {
                best=explained;
                bestX=(col+.5-.5*p.width)*p.pixelPitchRad;bestY=yCoord;
                bestWidth=0;bestHeight=0;bestAmplitude=correlation*invN2;bestNorm=1.0/invN2;
            }
        }
    }
    const double degrees=static_cast<double>(count)-static_cast<double>(pre.nuisance.size())-5,tail=.5*p.detectionSigma*p.detectionSigma;
    const double residualBound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    const double subdivisions=std::ldexp(1.,18);
    const double logTrials=2*std::log(p.width*subdivisions+1)+2*std::log(p.height*subdivisions+1);
    const double threshold=std::sqrt(p.detectionSigma*p.detectionSigma+2*(std::log(2.)+logTrials));
    // Exact Cauchy-Schwarz detection screen: no compact template can ever
    // explain more energy than dataNorm. When dataNorm/variance < threshold^2
    // (noise-only empty sky), fitSnr < threshold is guaranteed and no feature
    // can be emitted; skip the expensive (w,h) sweep and 18-level refinement.
    if(dataNorm/variance<threshold*threshold) {
        if(best>0&&std::isfinite(best)) {
            result.fittedCentreXRad=bestX;result.fittedCentreYRad=bestY;
            result.fittedWidthRad=bestWidth;result.fittedHeightRad=bestHeight;
            result.fitSnr=std::sqrt(best/variance);result.fitResidual=std::max(0.,dataNorm-best)/variance;
            result.compactFitSnr=result.fitSnr;result.compactFitResidual=result.fitResidual;
        }
        return;
    }
    // Separable 1D contraction cache on pre-filtered, pre-projected weights:
    // eliminates filtered()+project() from every evaluation AND restricts the
    // contracted window strictly to the active [rowLo..rowHi) / [colLo..colHi)
    // band visited by the current refinement coordinate.
    const double fitInvRoot2Sigma=1./(std::sqrt(2.)*p.psfSigmaRad);
    const double fitPitch=p.pixelPitchRad,fitHalfW=.5*p.width,fitHalfH=.5*p.height,fitTail=9*p.psfSigmaRad,fitInterior=9.5*p.psfSigmaRad;
    const auto axisProb=[&](double low,double high,double centre,double extent) noexcept {
        if(extent==0)return .5*std::erfc((centre-high)*fitInvRoot2Sigma)-.5*std::erfc((centre-low)*fitInvRoot2Sigma);
        const double left=centre-.5*extent,right=centre+.5*extent;
        if(low>=left+fitInterior&&high<=right-fitInterior)return (high-low)/extent;
        return gaussianIntervalFastV1(left,right,low,high,p.psfSigmaRad)/extent;
    };
    std::vector<double> rowVec(static_cast<std::size_t>(p.height)*count,0.);
    std::vector<double> colVec(static_cast<std::size_t>(p.width)*count,0.);
    bool hasRowVec=false,hasColVec=false;
    double cachedRowX=0,cachedRowW=0,cachedColY=0,cachedColH=0;
    unsigned cachedRowLo=0,cachedRowHi=0,cachedColLo=0,cachedColHi=0;
    std::vector<double> spotBuf(count,0.);
    const double* pwwBase=pre.projWeightedWeights.data();
    const double* prpBase=pre.projRowPrefix.empty()?nullptr:pre.projRowPrefix.data();
    const double* pcpBase=pre.projColPrefix.empty()?nullptr:pre.projColPrefix.data();
    const std::size_t fitPCols=static_cast<std::size_t>(p.width)+1u;
    const auto prepareRowVec=[&](double x,double width,unsigned reqRowLo,unsigned reqRowHi) {
        if(hasRowVec&&x==cachedRowX&&width==cachedRowW&&reqRowLo>=cachedRowLo&&reqRowHi<=cachedRowHi)return;
        std::array<double,128> wx{};
        unsigned colLo=p.width,colHi=0;
        const double xLo=x-.5*width-fitTail,xHi=x+.5*width+fitTail;
        for(unsigned col=0;col<p.width;++col) {
            const double lowX=(col-fitHalfW)*fitPitch,highX=lowX+fitPitch;
            if(lowX>xHi||highX<xLo)continue;
            const double w=axisProb(lowX,highX,x,width);
            if(w==0.)continue;
            wx[col]=w;if(col<colLo)colLo=col;colHi=col+1u;
        }
        const unsigned rLo=std::min(reqRowLo,p.height),rHi=std::min(reqRowHi,p.height);
        for(unsigned row=rLo;row<rHi;++row) {
            double* dst=rowVec.data()+static_cast<std::size_t>(row)*count;
            std::fill(dst,dst+count,0.);
            if(colLo<colHi) {
                const std::size_t rowOff=static_cast<std::size_t>(row)*p.width;
                const std::size_t prefRowOff=static_cast<std::size_t>(row)*fitPCols;
                for(unsigned col=colLo;col<colHi;) {
                    const double w=wx[col];
                    if(w==0.){++col;continue;}
                    unsigned cEnd=col+1u;
                    while(cEnd<colHi&&wx[cEnd]==w)++cEnd;
                    if(cEnd-col>=2u&&prpBase) {
                        const double* s0=prpBase+(prefRowOff+col)*count;
                        const double* s1=prpBase+(prefRowOff+cEnd)*count;
                        reticleFmaddDiff(dst,s1,s0,w,count);
                    } else {
                        for(unsigned cIdx=col;cIdx<cEnd;++cIdx) {
                            const double* src=pwwBase+(rowOff+cIdx)*count;
                            reticleFmadd1(dst,src,w,count);
                        }
                    }
                    col=cEnd;
                }
            }
        }
        cachedRowX=x;cachedRowW=width;cachedRowLo=rLo;cachedRowHi=rHi;hasRowVec=true;
    };
    const auto prepareColVec=[&](double y,double height,unsigned reqColLo,unsigned reqColHi) {
        if(hasColVec&&y==cachedColY&&height==cachedColH&&reqColLo>=cachedColLo&&reqColHi<=cachedColHi)return;
        std::array<double,128> wy{};
        unsigned rowLo=p.height,rowHi=0;
        const double yLo=y-.5*height-fitTail,yHi=y+.5*height+fitTail;
        for(unsigned row=0;row<p.height;++row) {
            const double lowY=(row-fitHalfH)*fitPitch,highY=lowY+fitPitch;
            if(lowY>yHi||highY<yLo)continue;
            const double w=axisProb(lowY,highY,y,height);
            if(w==0.)continue;
            wy[row]=w;if(row<rowLo)rowLo=row;rowHi=row+1u;
        }
        const unsigned cLo=std::min(reqColLo,p.width),cHi=std::min(reqColHi,p.width);
        for(unsigned col=cLo;col<cHi;++col) {
            double* dst=colVec.data()+static_cast<std::size_t>(col)*count;
            std::fill(dst,dst+count,0.);
        }
        if(rowLo<rowHi&&cLo<cHi) {
            for(unsigned row=rowLo;row<rowHi;) {
                const double w=wy[row];
                if(w==0.){++row;continue;}
                unsigned rEnd=row+1u;
                while(rEnd<rowHi&&wy[rEnd]==w)++rEnd;
                if(rEnd-row>=2u&&pcpBase) {
                    const std::size_t off0=static_cast<std::size_t>(row)*p.width;
                    const std::size_t off1=static_cast<std::size_t>(rEnd)*p.width;
                    for(unsigned col=cLo;col<cHi;++col) {
                        double* dst=colVec.data()+static_cast<std::size_t>(col)*count;
                        const double* s0=pcpBase+(off0+col)*count;
                        const double* s1=pcpBase+(off1+col)*count;
                        reticleFmaddDiff(dst,s1,s0,w,count);
                    }
                } else {
                    for(unsigned rIdx=row;rIdx<rEnd;++rIdx) {
                        const std::size_t rowOff=static_cast<std::size_t>(rIdx)*p.width;
                        for(unsigned col=cLo;col<cHi;++col) {
                            double* dst=colVec.data()+static_cast<std::size_t>(col)*count;
                            const double* src=pwwBase+(rowOff+col)*count;
                            reticleFmadd1(dst,src,w,count);
                        }
                    }
                }
                row=rEnd;
            }
        }
        cachedColY=y;cachedColH=height;cachedColLo=cLo;cachedColHi=cHi;hasColVec=true;
    };
    const auto evalSpotFromRowVec=[&](double y,double height) {
        std::fill(spotBuf.begin(),spotBuf.end(),0.);
        const double yLo=y-.5*height-fitTail,yHi=y+.5*height+fitTail;
        for(unsigned row=cachedRowLo;row<cachedRowHi;++row) {
            const double lowY=(row-fitHalfH)*fitPitch,highY=lowY+fitPitch;
            if(lowY>yHi||highY<yLo)continue;
            const double w=axisProb(lowY,highY,y,height);
            if(w==0.)continue;
            const double* src=rowVec.data()+static_cast<std::size_t>(row)*count;
            reticleFmadd1(spotBuf.data(),src,w,count);
        }
    };
    const auto evalSpotFromColVec=[&](double x,double width) {
        std::fill(spotBuf.begin(),spotBuf.end(),0.);
        const double xLo=x-.5*width-fitTail,xHi=x+.5*width+fitTail;
        for(unsigned col=cachedColLo;col<cachedColHi;++col) {
            const double lowX=(col-fitHalfW)*fitPitch,highX=lowX+fitPitch;
            if(lowX>xHi||highX<xLo)continue;
            const double w=axisProb(lowX,highX,x,width);
            if(w==0.)continue;
            const double* src=colVec.data()+static_cast<std::size_t>(col)*count;
            reticleFmadd1(spotBuf.data(),src,w,count);
        }
    };
    // A real unresolved compact radiator has finite unknown support. Fit its
    // rectangle-convolved PSF from the scalar waveform rather than copying
    // the scene's extent or rejecting every high-SNR non-point radiator.
    const double initialX=bestX,initialY=bestY;
    for(const double w:{0.,1.,2.,4.,8.,16.,32.}) {
        const double width=w*p.pixelPitchRad;
        prepareRowVec(initialX,width,0u,p.height);
        for(const double h:{0.,1.,2.,4.,8.,16.,32.}) {
            const double height=h*p.pixelPitchRad;
            evalSpotFromRowVec(initialY,height);
            consider(spotBuf.data(),initialX,initialY,width,height);
        }
    }
    double step=p.pixelPitchRad;
    const auto refine=[&](double initialStep) {
        step=initialStep;
        for(unsigned level=0;level<18;++level) {
            step*=.5;
            for(unsigned sweep=0;sweep<3;++sweep) {
                bool moved=false;
                for(unsigned coordinate=0;coordinate<4;++coordinate) {
                    const std::array<double,4> centre{bestX,bestY,bestWidth,bestHeight};
                    const double beforeBest=best;
                    const bool varyX=(coordinate==0||coordinate==2);
                    if(varyX) {
                        const double reach=.5*(centre[2]+step)+step+fitTail;
                        const unsigned cLo=static_cast<unsigned>(std::max(0,static_cast<int>(std::floor((centre[0]-reach)/fitPitch+fitHalfW))));
                        const unsigned cHi=static_cast<unsigned>(std::min(static_cast<int>(p.width),static_cast<int>(std::floor((centre[0]+reach)/fitPitch+fitHalfW))+1));
                        prepareColVec(centre[1],centre[3],cLo,cHi);
                    } else {
                        const double reach=.5*(centre[3]+step)+step+fitTail;
                        const unsigned rLo=static_cast<unsigned>(std::max(0,static_cast<int>(std::floor((centre[1]-reach)/fitPitch+fitHalfH))));
                        const unsigned rHi=static_cast<unsigned>(std::min(static_cast<int>(p.height),static_cast<int>(std::floor((centre[1]+reach)/fitPitch+fitHalfH))+1));
                        prepareRowVec(centre[0],centre[2],rLo,rHi);
                    }
                    for(const double sign:{-1.,1.}) {
                        auto q=centre;q[coordinate]+=sign*step;
                        if(std::abs(q[0])>.5*p.width*p.pixelPitchRad||std::abs(q[1])>.5*p.height*p.pixelPitchRad||
                           q[2]<0||q[3]<0||q[2]>p.width*p.pixelPitchRad||q[3]>p.height*p.pixelPitchRad)continue;
                        if(varyX)evalSpotFromColVec(q[0],q[2]);
                        else evalSpotFromRowVec(q[1],q[3]);
                        consider(spotBuf.data(),q[0],q[1],q[2],q[3]);
                    }
                    if(best>beforeBest)moved=true;
                }
                if(!moved)break;
            }
        }
    };
    refine(p.pixelPitchRad);
    const P4 initialShape{{bestX,bestY,bestWidth,bestHeight}};
    const auto initialPhysical=physicalSourceAmplitudeV1(result,initialShape,bestAmplitude);
    const double initialPhysicalScore=initialPhysical?
        2*(*initialPhysical)*bestAmplitude*bestNorm-(*initialPhysical)*(*initialPhysical)*bestNorm:0;
    const bool physicalRecovery=std::sqrt(std::max(0.,best)/variance)>=threshold&&
        (!(initialPhysicalScore>0)||!std::isfinite(initialPhysicalScore)||
         std::max(0.,dataNorm-initialPhysicalScore)/variance>residualBound);
    if(std::max(0.,dataNorm-best)/variance>residualBound||physicalRecovery) {
        // A wide source may have its largest point-template correlation at a
        // remote reticle alias. Search finite support AND centre jointly before
        // repeating the unchanged exact local refinement. A large correlation
        // can never substitute for the final waveform goodness-of-fit gate.
        double proposed=best;
        std::array<double,4> proposal{bestX,bestY,bestWidth,bestHeight};
        const auto stride=static_cast<std::size_t>(pre.supportColumns+1)*count;
        const auto pRows=pre.supportRows+1u,pCols=pre.supportColumns+1u;
        // Precompute exact data-independent 1/norm2 for every valid support rectangle
        // once per shared reticle configuration: reduces every subsequent support
        // search from O(rects * samplesPerScan) to O(rects) (~0.4 ms vs ~40 ms).
        std::call_once(pre.supportNormOnce,[&]() {
            const auto spacing=static_cast<unsigned>(std::llround(pre.supportPitch/p.pixelPitchRad));
            const auto sizes=[&](unsigned dimension) {
                std::vector<unsigned> supportedSizes{1u,2u,3u,4u,6u,8u,12u,16u,24u,32u,48u,64u,96u,128u};
                const auto maximum=dimension/spacing;
                if(maximum&&std::find(supportedSizes.begin(),supportedSizes.end(),maximum)==supportedSizes.end())supportedSizes.push_back(maximum);
                std::sort(supportedSizes.begin(),supportedSizes.end());return supportedSizes;
            };
            const auto widths=sizes(p.width),heights=sizes(p.height);
            const double maxAbsX=.5*p.width*p.pixelPitchRad,maxAbsY=.5*p.height*p.pixelPitchRad;
            std::vector<double> rowSlice(static_cast<std::size_t>(pCols)*count,0.);
            std::vector<double> rowSliceNorm2(pCols,0.);
            for(const unsigned width:widths)for(const unsigned height:heights) {
                if(width*pre.supportPitch>p.width*p.pixelPitchRad||height*pre.supportPitch>p.height*p.pixelPitchRad)continue;
                const double areaCells=static_cast<double>(width)*height;
                const double minRawNorm2=128*std::numeric_limits<double>::epsilon()*areaCells*areaCells;
                PrecomputedV1::SupportSizeGroup grp{};
                grp.width=width;grp.height=height;
                grp.widthRad=width*pre.supportPitch;grp.heightRad=height*pre.supportPitch;
                grp.begin=pre.supportRects.size();
                unsigned cachedR0=pRows,cachedR1=pRows;
                for(int row=-static_cast<int>(height);row<=static_cast<int>(pre.supportRows);++row) {
                    const double y=(row+.5*height-pre.supportMargin)*pre.supportPitch-maxAbsY;
                    if(std::abs(y)>maxAbsY)continue;
                    const auto row0=static_cast<unsigned>(std::max(0,row));
                    const auto row1=static_cast<unsigned>(std::min(static_cast<int>(pre.supportRows),row+static_cast<int>(height)));
                    if(row1<=row0)continue;
                    if(row0!=cachedR0||row1!=cachedR1) {
                        const double* pR0=pre.supportPrefix.data()+static_cast<std::size_t>(row0)*stride;
                        const double* pR1=pre.supportPrefix.data()+static_cast<std::size_t>(row1)*stride;
                        for(unsigned cIdx=0;cIdx<pCols;++cIdx) {
                            const double* v0=pR0+static_cast<std::size_t>(cIdx)*count;
                            const double* v1=pR1+static_cast<std::size_t>(cIdx)*count;
                            double* dst=rowSlice.data()+static_cast<std::size_t>(cIdx)*count;
                            double n2=0;
                            for(unsigned j=0;j<count;++j){const double d=v1[j]-v0[j];dst[j]=d;n2+=d*d;}
                            rowSliceNorm2[cIdx]=n2;
                        }
                        cachedR0=row0;cachedR1=row1;
                    }
                    for(int col=-static_cast<int>(width);col<=static_cast<int>(pre.supportColumns);++col) {
                        const double x=(col+.5*width-pre.supportMargin)*pre.supportPitch-maxAbsX;
                        if(std::abs(x)>maxAbsX)continue;
                        const auto col0=static_cast<unsigned>(std::max(0,col));
                        const auto col1=static_cast<unsigned>(std::min(static_cast<int>(pre.supportColumns),col+static_cast<int>(width)));
                        if(col1<=col0)continue;
                        double rawNorm2=0;
                        if(col0==0) {
                            rawNorm2=rowSliceNorm2[col1];
                        } else {
                            const double* s0=rowSlice.data()+static_cast<std::size_t>(col0)*count;
                            const double* s1=rowSlice.data()+static_cast<std::size_t>(col1)*count;
                            for(unsigned j=0;j<count;++j){const double v=s1[j]-s0[j];rawNorm2+=v*v;}
                        }
                        if(rawNorm2<=minRawNorm2)continue;
                        pre.supportRects.push_back(PrecomputedV1::SupportRectEntry{row,col,row0,row1,col0,col1,x,y,1.0/rawNorm2});
                    }
                }
                grp.end=pre.supportRects.size();
                if(grp.end>grp.begin)pre.supportGroups.push_back(grp);
            }
        });
        std::vector<double> dataPrefix(static_cast<std::size_t>(pRows)*pCols,0.);
        for(unsigned r=0;r<pRows;++r) {
            const double* pRow=pre.supportPrefix.data()+static_cast<std::size_t>(r)*stride;
            double* dstRow=dataPrefix.data()+static_cast<std::size_t>(r)*pCols;
            unsigned colIdx=0;
            for(;colIdx+4u<=pCols;colIdx+=4u) {
                reticleDot4(dataPtr,
                    pRow+static_cast<std::size_t>(colIdx)*count,
                    pRow+static_cast<std::size_t>(colIdx+1u)*count,
                    pRow+static_cast<std::size_t>(colIdx+2u)*count,
                    pRow+static_cast<std::size_t>(colIdx+3u)*count,
                    count,dstRow+colIdx);
            }
            for(;colIdx<pCols;++colIdx)
                dstRow[colIdx]=reticleDot(dataPtr,pRow+static_cast<std::size_t>(colIdx)*count,count);
        }
        const double* dpBase=dataPrefix.data();
        const auto* rectPtr=pre.supportRects.data();
        for(const auto& grp:pre.supportGroups) {
            double scaleBest=-1;P4 scaleShape{};
            for(std::size_t idx=grp.begin;idx<grp.end;++idx) {
                const auto& r=rectPtr[idx];
                const std::size_t off0=static_cast<std::size_t>(r.row0)*pCols;
                const std::size_t off1=static_cast<std::size_t>(r.row1)*pCols;
                const double rawCorr=(dpBase[off1+r.col1]-dpBase[off0+r.col1])-(dpBase[off1+r.col0]-dpBase[off0+r.col0]);
                const double explained=(rawCorr*rawCorr)*r.invRawNorm2;
                if(explained>proposed){proposed=explained;proposal={r.x,r.y,grp.widthRad,grp.heightRad};}
                if(explained>scaleBest){scaleBest=explained;scaleShape={{r.x,r.y,grp.widthRad,grp.heightRad}};}
            }
            if(scaleBest>0)supportSeeds.push_back({scaleBest,scaleShape});
        }
        if(proposed>best) {
            const auto exact=spot(proposal[0],proposal[1],p.psfSigmaRad,proposal[2],proposal[3]);
            consider(exact.data(),proposal[0],proposal[1],proposal[2],proposal[3]);refine(pre.supportPitch);
        }
    }
    const auto trustRefine=[&] {
        if(!(bestNorm>0))return;
        // Bounded, scaled damped Gauss-Newton with signed amplitude removed
        // analytically. Independent SciPy TRF fits of the actual frozen ADC
        // waveforms verify this forward model and its analytic Jacobian.
        // Every accepted candidate is projected BACK to the pre-existing
        // pitch/2^18 lattice, then judged by the original exact spot objective.
        const double lattice=p.pixelPitchRad/std::ldexp(1.,18),sigma=std::sqrt(variance);
        const double fullX=p.width*p.pixelPitchRad,fullY=p.height*p.pixelPitchRad;
        const P4 lower{{-.5*fullX,-.5*fullY,0,0}},upper{{.5*fullX,.5*fullY,fullX*fullX,fullY*fullY}};
        const double minimumDamping=64*std::numeric_limits<double>::epsilon();
        double damping=1e-3;
        for(unsigned iteration=0;iteration<64;++iteration) {
            const P4 current{{bestX,bestY,bestWidth*bestWidth,bestHeight*bestHeight}};
            const auto values=spot(bestX,bestY,p.psfSigmaRad,bestWidth,bestHeight);
            const double norm2=inner(values,values);if(!(norm2>0))break;
            const double amplitude=inner(values,data)/norm2;
            const auto derivative=spotJacobianV1(bestX,bestY,bestWidth,bestHeight);
            std::vector<double> residual(count);std::array<std::vector<double>,4> jacobian;
            for(unsigned j=0;j<count;++j)residual[j]=(data[j]-amplitude*values[j])/sigma;
            for(unsigned coordinate=0;coordinate<4;++coordinate) {
                const double amplitudeDerivative=(inner(data,derivative[coordinate])-2*amplitude*inner(values,derivative[coordinate]))/norm2;
                jacobian[coordinate].resize(count);
                for(unsigned j=0;j<count;++j)jacobian[coordinate][j]=
                    (-amplitude*derivative[coordinate][j]-amplitudeDerivative*values[j])/sigma;
            }
            P4 low{},high{};for(unsigned coordinate=0;coordinate<4;++coordinate) {
                low[coordinate]=lower[coordinate]-current[coordinate];high[coordinate]=upper[coordinate]-current[coordinate];
            }
            bool accepted=false;
            for(unsigned trial=0;trial<12;++trial) {
                P4 delta{};if(!dampedBoxStep(jacobian,residual,low,high,damping,delta))break;
                P4 candidate{};for(unsigned coordinate=0;coordinate<4;++coordinate)
                    candidate[coordinate]=std::clamp(current[coordinate]+delta[coordinate],lower[coordinate],upper[coordinate]);
                P4 geometry{{candidate[0],candidate[1],std::sqrt(candidate[2]),std::sqrt(candidate[3])}};
                for(auto& value:geometry)value=std::round(value/lattice)*lattice;
                geometry[0]=std::clamp(geometry[0],lower[0],upper[0]);geometry[1]=std::clamp(geometry[1],lower[1],upper[1]);
                geometry[2]=std::clamp(geometry[2],0.,fullX);geometry[3]=std::clamp(geometry[3],0.,fullY);
                candidate={{geometry[0],geometry[1],geometry[2]*geometry[2],geometry[3]*geometry[3]}};
                if(candidate==current)break;
                const double old=best;
                const auto proposed=spot(geometry[0],geometry[1],p.psfSigmaRad,geometry[2],geometry[3]);
                consider(proposed.data(),geometry[0],geometry[1],geometry[2],geometry[3]);
                if(best>old) {
                    double predictedCost=0;
                    for(unsigned j=0;j<count;++j) {
                        double predicted=residual[j];for(unsigned coordinate=0;coordinate<4;++coordinate)
                            predicted+=jacobian[coordinate][j]*(candidate[coordinate]-current[coordinate]);
                        predictedCost+=predicted*predicted;
                    }
                    const double reduction=inner(residual,residual)-predictedCost;
                    const double ratio=reduction>0?(best-old)/(variance*reduction):0;
                    if(ratio>.75)damping=std::max(minimumDamping,damping/3);
                    else if(ratio<.25)damping=std::min(1/minimumDamping,damping*2);
                    step=lattice;accepted=true;break;
                }
                damping=std::min(1/minimumDamping,damping*10);
            }
            if(!accepted)break;
        }
    };
    if(std::max(0.,dataNorm-best)/variance>residualBound)trustRefine();
    const auto enforcePhysical=[&]() {
        if(!(bestNorm>0)){best=0;return false;}
        const P4 shape{{bestX,bestY,bestWidth,bestHeight}};
        const auto physical=physicalSourceAmplitudeV1(result,shape,bestAmplitude);
        if(!physical){best=0;return false;}
        const bool interior=*physical==bestAmplitude;
        if(!interior) {
            const auto wave=spot(bestX,bestY,p.psfSigmaRad,bestWidth,bestHeight);
            bestAmplitude=*physical;best=2*bestAmplitude*inner(data,wave)-bestAmplitude*bestAmplitude*inner(wave,wave);
        }
        return interior;
    };
    bool sourceInterior=enforcePhysical();
    if((!(best>0)||std::max(0.,dataNorm-best)/variance>residualBound)&&!supportSeeds.empty()) {
        // Each distinct support-scale minimum descends in LOCAL state. A
        // poorer coarse seed must not be prevented by another basin's score
        // from reaching a better physical solution. Numerical effort is
        // bounded; source geometry/labels never supply a seed.
        using State=std::array<double,8>;
        const auto save=[&](){return State{{best,bestX,bestY,bestWidth,bestHeight,bestAmplitude,bestNorm,step}};};
        const auto load=[&](const State& q){best=q[0];bestX=q[1];bestY=q[2];bestWidth=q[3];bestHeight=q[4];bestAmplitude=q[5];bestNorm=q[6];step=q[7];};
        auto selected=save();bool selectedInterior=sourceInterior;
        std::stable_sort(supportSeeds.begin(),supportSeeds.end(),[](const Seed& a,const Seed& b){return a.score>b.score;});
        const auto budget=std::min<std::size_t>(16,supportSeeds.size());
        for(std::size_t index=0;index<budget;++index) {
            const auto& q=supportSeeds[index].shape;
            best=0;bestNorm=0;
            const auto wave=spot(q[0],q[1],p.psfSigmaRad,q[2],q[3]);consider(wave.data(),q[0],q[1],q[2],q[3]);
            if(!(bestNorm>0))continue;
            refine(pre.supportPitch);trustRefine();
            const bool interior=enforcePhysical();
            if(best>selected[0]){selected=save();selectedInterior=interior;}
            if(selected[0]>0&&std::max(0.,dataNorm-selected[0])/variance<=residualBound)break;
        }
        load(selected);sourceInterior=selectedInterior;
    }
    if(!(best>0)||!std::isfinite(best))return;
    result.fittedCentreXRad=bestX;result.fittedCentreYRad=bestY;
    result.fittedWidthRad=bestWidth;result.fittedHeightRad=bestHeight;
    result.fitSnr=std::sqrt(best/variance);result.fitResidual=std::max(0.,dataNorm-best)/variance;
    result.compactFitSnr=result.fitSnr;result.compactFitResidual=result.fitResidual;
    const auto forward=unit(pointing);auto right=unit(cross(forward,unit(up)));
    if(norm(right)<.5)right=unit(cross(forward,std::abs(forward.z)<.9?V{0,0,1}:V{0,1,0}));
    const auto vertical=unit(cross(right,forward));
    const auto emit=[&](double angularVariance,bool precise) {
        ThermalImageFeatureV1 feature;feature.direction=unit(add(forward,add(scale(right,std::tan(bestX)),scale(vertical,std::tan(bestY)))));
        feature.signalToNoise=result.fitSnr;feature.angularVarianceRad2=angularVariance;
        feature.signedContrastElectrons=bestAmplitude;
        feature.extent.widthRad=bestWidth;feature.extent.heightRad=bestHeight;
        feature.centroidColumn=(bestX/p.pixelPitchRad)+.5*p.width;
        feature.centroidRow=(bestY/p.pixelPitchRad)+.5*p.height;
        result.features.push_back(feature);result.bearingResolved=true;result.preciseBearing=precise;
    };
    // A singular local Fisher matrix supplies no local precision bound. A
    // provisional coarse direction uses the squared diameter of the ENTIRE
    // admitted optical field, not an invented width/quantization covariance.
    const double fieldRadius=std::atan(std::hypot(std::tan(.5*p.width*p.pixelPitchRad),std::tan(.5*p.height*p.pixelPitchRad)));
    const double coarseVariance=4*fieldRadius*fieldRadius;
    // A finite strip still provides centering in its compact axis even if
    // its long axis is ambiguous. Field-filling DC remains subject to the
    // same measured modulation/residual and physical background-null gates.
    const bool compactSource=bestWidth<.5*p.width*p.pixelPitchRad||bestHeight<.5*p.height*p.pixelPitchRad;
    // A compact source truncated by the optical stop can fit a wide support.
    // Its best centre remains near the stop; keep the coarse measurement there
    // rather than losing an already detected edge source.
    const bool nearStop=std::hypot(bestX,bestY)>=.75*c.stopRadiusRad;
    // Bound the ENTIRE fixed final angle/support lattice, not merely the
    // number of data-adaptively selected refinement queries. This keeps the
    // nominal Gaussian trial screen honest without changing detectionSigma.
    // Laurent-Massart upper tail for a Gaussian residual, disclosed as a
    // high-count engineering screen, not a calibrated equipment ROC.
    if(result.fitSnr<threshold||degrees<=0||bestNorm<=0||result.fitResidual>residualBound)return;
    const P4 fitted{{bestX,bestY,bestWidth,bestHeight}};
    const auto local=sourceInterior?localAngularVarianceV1(fitted,bestAmplitude,variance,step):std::nullopt;
    if(local) {
        const double lattice=p.pixelPitchRad/std::ldexp(1.,18);
        const auto ray=[&](const P4& q){return unit(add(forward,add(scale(right,std::tan(q[0])),scale(vertical,std::tan(q[1])))));};
        const auto direction=ray(fitted);
        std::vector<std::array<double,5>> modes{{{bestX,bestY,bestWidth,bestHeight,*local}}};
        bool invalidMode=false,profileUnresolved=false;
        const auto alternative=[&](P4 q,unsigned kind) {
            for(auto& value:q)value=std::round(value/lattice)*lattice;
            if(std::abs(q[0])>.5*p.width*p.pixelPitchRad||std::abs(q[1])>.5*p.height*p.pixelPitchRad||
               q[2]<0||q[3]<0||q[2]>p.width*p.pixelPitchRad||q[3]>p.height*p.pixelPitchRad)return;
            const auto other=ray(q);
            const double displacement=std::atan2(norm(cross(direction,other)),dot(direction,other));
            if(displacement<=lattice)return;
            const auto wave=spot(q[0],q[1],p.psfSigmaRad,q[2],q[3]);
            const double squared=inner(wave,wave);if(squared<=128*std::numeric_limits<double>::epsilon())return;
            const double correlation=inner(data,wave),unconstrained=correlation/squared;
            const auto legal=physicalSourceAmplitudeV1(result,q,unconstrained);if(!legal)return;
            const double explained=2*(*legal)*correlation-(*legal)*(*legal)*squared;
            if(!std::isfinite(explained)||best-explained>threshold*threshold*variance)return;
            result.ambiguousAlternativeKind=kind;result.alternativeShape=q;result.alternativeAmplitude=*legal;
            result.alternativeCostDifference=(best-explained)/variance;
            if(const auto budget=rawSkyFitV1(result,spot(q[0],q[1],p.psfSigmaRad,q[2],q[3],false))) {
                result.alternativeRadianceAvailable=true;
                result.alternativeSky=(*budget)[0]-(*budget)[1]*(*legal);
                const double cells=q[2]*q[3]/(p.pixelPitchRad*p.pixelPitchRad);
                result.alternativeSource=cells>0?result.alternativeSky+(*legal)/cells:*legal;
            }
            if(kind==2){profileUnresolved=true;return;}
            const auto modeVariance=*legal==unconstrained?localAngularVarianceV1(q,*legal,variance,lattice):std::nullopt;
            if(!modeVariance){invalidMode=true;return;}
            modes.push_back({{q[0],q[1],q[2],q[3],*modeVariance}});
        };
        // A small finite mirror cluster carries its actual mode displacement,
        // not the entire field. Every reflection is evaluated independently.
        for(const double sx:{-1.,1.})for(const double sy:{-1.,1.}) {
            if(sx==1&&sy==1)continue;
            auto q=fitted;q[0]*=sx;q[1]*=sy;alternative(q,1);
        }
        // Never certify optical-edge observability from detector rectangle
        // containment: an admitted physical stop can be much smaller.
        for(unsigned axis=0;axis<2;++axis) {
            const double full=(axis?p.height:p.width)*p.pixelPitchRad;
            for(const double sign:{-1.,1.}) {
                const double low=std::max(-.5*full-fitted[axis],sign>0?-.5*fitted[axis+2]:.5*(fitted[axis+2]-full));
                const double high=std::min(.5*full-fitted[axis],sign>0?.5*(full-fitted[axis+2]):.5*fitted[axis+2]);
                if(low>high)continue;
                for(const double delta:{low,high}) {
                    auto q=fitted;q[axis]+=delta;q[axis+2]+=2*sign*delta;alternative(q,2);
                }
            }
        }
        if(!invalidMode&&!profileUnresolved) {
            if(const auto envelope=finiteMirrorEnvelopeV1(c,modes)){emit(*envelope,true);return;}
        }
    }
    // The source cleared the detection and residual gates, but its local
    // angle/support Hessian is singular (edge truncation or unresolved extent).
    // A detected source must still produce a coarse measured bearing; its
    // uncertainty covers the entire admitted field, rather than claiming
    // precision from a singular or globally ambiguous local fit.
    if((compactSource||nearStop)&&std::isfinite(coarseVariance)&&coarseVariance>0){emit(coarseVariance,false);return;}
    return;
}
std::optional<std::vector<double>> ThermalReticleReceiverV1::unitGroundFieldV1(
    const ThermalImageConfigV1& p,V pointing,V up,double cutoff,
    const std::optional<std::array<double,4>>& region,bool* accelerated) {
    return unitGroundFieldMemoV2(p,pointing,up,cutoff,region,{},accelerated);
}
std::optional<std::vector<double>> ThermalReticleReceiverV1::unitGroundFieldMemoV2(
    const ThermalImageConfigV1& p,V pointing,V up,double cutoff,
    const std::optional<std::array<double,4>>& region,
    const std::function<const std::vector<double>*()>& nativeBackground,bool* accelerated) {
    return unitGroundFieldProfileV2(p,pointing,up,cutoff,region,nativeBackground,nullptr,accelerated);
}
std::optional<std::vector<double>> ThermalReticleReceiverV1::unitGroundFieldProfileV2(
    const ThermalImageConfigV1& p,V pointing,V up,double cutoff,
    const std::optional<std::array<double,4>>& region,
    const std::function<const std::vector<double>*()>& nativeBackground,ThermalGroundWorkV2* work,bool* accelerated) {
    if(work){*work={};work->fallbackReason=7;}
    UnitGroundClockV2 totalClock(work,false);
    std::unordered_set<std::uint64_t> diagnosticSamples;

    if(accelerated)*accelerated=false;
    if(!ThermalImageReceiverV1::validConfig(p)||!finite(pointing)||norm(pointing)<=0||
       !finite(up)||std::abs(norm(up)-1)>1e-6||!std::isfinite(cutoff)||std::abs(cutoff)>1)return std::nullopt;
    if(region&&(!std::all_of(region->begin(),region->end(),[](double v){return std::isfinite(v)&&std::abs(v)<=1.55;})||
        !((*region)[1]>(*region)[0])||!((*region)[3]>(*region)[2])))return std::nullopt;
    if(work)work->fallbackReason=0;
    if(!region&&!work) {
        bool nearestRounding=std::fegetround()==FE_TONEAREST;
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
        nearestRounding=nearestRounding&&(_mm_getcsr()&0x6000u)==0;
#endif
        if(nearestRounding) {
            struct UnitPrepCacheV1 {
                unsigned width=0,height=0;
                std::array<std::uint64_t,8> keyBits{};
                UnitGroundPrepV1 prep;
                bool ready=false;
            };
            thread_local UnitPrepCacheV1 cache;
            std::array<std::uint64_t,8> bits{};
            const double vals[8]={p.pixelPitchRad,p.psfSigmaRad,pointing.x,pointing.y,pointing.z,up.x,up.y,up.z};
            std::memcpy(bits.data(),vals,sizeof(vals));
            if(!cache.ready||cache.width!=p.width||cache.height!=p.height||cache.keyBits!=bits) {
                cache.prep=UnitGroundPrepV1::build(p,pointing,up);
                cache.width=p.width;cache.height=p.height;cache.keyBits=bits;cache.ready=true;
            }
            if(auto fast=cache.prep.evaluateFullFrame(cutoff,false)) {
                if(accelerated)*accelerated=true;
                return fast;
            }
        }
    }
    const auto forward=unit(pointing),nativeUp=unit(up);
    auto right=unit(cross(forward,nativeUp));
    if(norm(right)<.5)right=unit(cross(forward,std::abs(forward.z)<.9?V{0,0,1}:V{0,1,0}));
    const auto vertical=unit(cross(right,forward));
    auto optical=p;optical.exposureSamples=1;
    const auto original=[&]()->std::optional<std::vector<double>> {
        UnitGroundClockV2 nativeClock(work,true);if(work)++work->originalCalls;
        ThermalSceneFrameV1 hypothesis;hypothesis.up=nativeUp;hypothesis.groundHorizonCosine=cutoff;hypothesis.groundRadianceWm2Sr=1;
        std::optional<ThermalIncidentFieldV1> ground;
        const std::vector<double>* native=nullptr;
        if(nativeBackground)native=nativeBackground();
        else {ground=ThermalImageReceiverV1::incidentFieldProfileV2(hypothesis,forward,optical,nullptr,work?&work->memo:nullptr);if(ground)native=&ground->radiance;}
        if(!native||native->size()!=static_cast<std::size_t>(p.width)*p.height)return std::nullopt;
        auto answer=*native;
        if(region) {
            ThermalRadiancePatchV1 black;black.direction=unit(add(forward,add(scale(right,std::tan(.5*((*region)[0]+(*region)[1]))),
                scale(vertical,std::tan(.5*((*region)[2]+(*region)[3]))))));
            black.widthRad=(*region)[1]-(*region)[0];black.heightRad=(*region)[3]-(*region)[2];black.rangeM=1;black.transmission=1;
            hypothesis.patches.push_back(black);hypothesis.refereeAttributionIndices.emplace_back(std::nullopt);
            const auto outside=ThermalImageReceiverV1::incidentFieldProfileV2(hypothesis,forward,optical,native,work?&work->memo:nullptr);
            if(!outside)return std::nullopt;
            for(std::size_t i=0;i<answer.size();++i)answer[i]-=outside->radiance[i];
        }
        return answer;
    };
    const auto fallback=[&](unsigned reason){if(work)work->fallbackReason=reason;return original();};
    const double pitch=p.pixelPitchRad,sigma=p.psfSigmaRad,margin=9*sigma,pixelArea=pitch*pitch;
    const double halfX=.5*p.width*pitch,halfY=.5*p.height*pitch;
    const double fieldLeft=std::max(-1.55,-halfX-margin),fieldRight=std::min(1.55,halfX+margin);
    const double fieldBottom=std::max(-1.55,-halfY-margin),fieldTop=std::min(1.55,halfY+margin);
    const double left=region?std::max(fieldLeft,(*region)[0]):fieldLeft;
    const double rightEdge=region?std::min(fieldRight,(*region)[1]):fieldRight;
    const double bottom=region?std::max(fieldBottom,(*region)[2]):fieldBottom;
    const double top=region?std::min(fieldTop,(*region)[3]):fieldTop;
    std::vector<double> answer(static_cast<std::size_t>(p.width)*p.height,0);
    if(!(rightEdge>left)||!(top>bottom)){if(accelerated)*accelerated=true;return answer;}
    const double a=dot(forward,nativeUp),b=dot(right,nativeUp),c=dot(vertical,nativeUp);
    // This sufficient monotonicity bound avoids a folded vertical cone. It
    // changes no optical domain: unsupported geometry uses the old renderer.
    const double maxTx=std::max(std::abs(std::tan(left)),std::abs(std::tan(rightEdge)));
    const double maxTy=std::max(std::abs(std::tan(bottom)),std::abs(std::tan(top)));
    const double maxN=std::nextafter(std::abs(a)+std::nextafter(std::abs(b)*maxTx,std::numeric_limits<double>::infinity()),std::numeric_limits<double>::infinity());
    if(!(std::nextafter(c,-std::numeric_limits<double>::infinity())>
        std::nextafter(maxN*maxTy,std::numeric_limits<double>::infinity())))return fallback(1);
    if(region) {
        // A finite support whose edge lands on the horizon cone is a mixed
        // endpoint quadrature. The accelerated interior-node rule cannot
        // represent the disappearing cap, so retain the exact original
        // renderer for this sensitive regional hypothesis.
        const auto horizon=[&](double x){return std::atan((cutoff-a-b*std::tan(x))/c);};
        const double h0=horizon(left),h1=horizon(rightEdge);
        const double low=std::min(h0,h1),high=std::max(h0,h1);
        const bool touchesField=(*region)[0]<=-halfX+1e-12||(*region)[1]>=halfX-1e-12||
            (*region)[2]<=-halfY+1e-12||(*region)[3]>=halfY-1e-12;
        if(touchesField&&(*region)[2]<=low+pitch&&(*region)[3]>=high-pitch)return fallback(2);
    }
    // For an uncut pixel, Gaussian convolution equals E[g(U+sigma Z)],
    // U uniform in the detector interval and Z standard normal. The omitted
    // source beyond the existing9-sigma support is <=2Q(9)<2.3e-19.
    // The rule changes numerical quadrature, never source-grid resolution.
    // Gauss rules: NIST DLMF3.5 (Legendre/Hermite). All weights are positive.
    struct Node {double x,w;};
    const double root3=std::sqrt(3.),root10=std::sqrt(10.),root30=std::sqrt(30.);
    const std::array<Node,2> l2{{{-1/root3,.5},{1/root3,.5}}};
    const std::array<Node,3> l3{{{-std::sqrt(.6),5./18},{0,4./9},{std::sqrt(.6),5./18}}};
    const double l4a=std::sqrt((3-2*std::sqrt(1.2))/7),l4b=std::sqrt((3+2*std::sqrt(1.2))/7);
    const std::array<Node,4> l4{{{-l4b,(18-root30)/72},{-l4a,(18+root30)/72},{l4a,(18+root30)/72},{l4b,(18-root30)/72}}};
    const std::array<Node,3> h3{{{-root3,1./6},{0,2./3},{root3,1./6}}};
    const std::array<Node,5> h5{{{-std::sqrt(5+root10),(7-2*root10)/60},{-std::sqrt(5-root10),(7+2*root10)/60},
        {0,8./15},{std::sqrt(5-root10),(7+2*root10)/60},{std::sqrt(5+root10),(7-2*root10)/60}}};
    const double sq2=std::sqrt(2.),sqPi=std::sqrt(pi);
    const std::array<Node,10> h10{{
        {-3.436159118837737603*sq2,.00000764043285523262063/sqPi},
        {-2.532731674232789797*sq2,.00134364574678123269/sqPi},
        {-1.756683649299881774*sq2,.0338743944554810631/sqPi},
        {-1.036610829789513654*sq2,.240138611082314687/sqPi},
        {-.342901327223704609*sq2,.610862633735325799/sqPi},
        {.342901327223704609*sq2,.610862633735325799/sqPi},
        {1.036610829789513654*sq2,.240138611082314687/sqPi},
        {1.756683649299881774*sq2,.0338743944554810631/sqPi},
        {2.532731674232789797*sq2,.00134364574678123269/sqPi},
        {3.436159118837737603*sq2,.00000764043285523262063/sqPi}}};
    const std::array<Node,8> l8{{{-.960289856497536232,.050614268145188130},{-.796666477413626740,.111190517226687235},
        {-.525532409916328986,.156853322938943644},{-.183434642495649805,.181341891689180991},
        {.183434642495649805,.181341891689180991},{.525532409916328986,.156853322938943644},
        {.796666477413626740,.111190517226687235},{.960289856497536232,.050614268145188130}}};
    const auto primitive=[](double z){return z*.5*std::erfc(-z/std::sqrt(2.))+std::exp(-.5*z*z)/std::sqrt(2*pi);};
    struct Row {double low=0,high=0,sourceLow=0,sourceHigh=0,full=0,constant=0,minCy=0,maxCy=0,minY2=0,maxY2=0;bool active=false;};
    std::array<Row,128> rows{};
    const auto minSquare=[](double low,double high){return low<=0&&high>=0?0.:std::min(low*low,high*high);};
    const auto bounds=[&](const std::array<double,4>& x,const Row& row) {
        const double n0=a+x[0]+row.minCy,n1=a+x[1]+row.maxCy;
        const double d0=std::sqrt(1+x[2]+row.minY2),d1=std::sqrt(1+x[3]+row.maxY2);
        return std::array<double,2>{std::min({n0/d0,n0/d1,n1/d0,n1/d1}),std::max({n0/d0,n0/d1,n1/d0,n1/d1})};
    };
    using Values=std::array<double,128>;
    // Y support, Gaussian primitives and full interval mass are independent
    // of detector column. Retain their exact values once per calibration.
    for(unsigned row=0;row<p.height;++row) {
        auto& info=rows[row];info.low=(row-.5*p.height)*pitch;info.high=info.low+pitch;
        info.sourceLow=std::max(bottom,info.low-margin);info.sourceHigh=std::min(top,info.high+margin);
        if(!(info.sourceHigh>info.sourceLow))continue;
        info.full=ThermalImageReceiverV1::gaussianIntervalV1(info.sourceLow,info.sourceHigh,info.low,info.high,sigma);
        info.constant=sigma*(primitive((info.high-info.sourceLow)/sigma)-primitive((info.low-info.sourceLow)/sigma));
        const double ty0=std::tan(info.sourceLow),ty1=std::tan(info.sourceHigh);
        info.minCy=std::min(c*ty0,c*ty1);info.maxCy=std::max(c*ty0,c*ty1);
        info.minY2=minSquare(ty0,ty1);info.maxY2=std::max(ty0*ty0,ty1*ty1);
    }
    // Exact horizontal coordinates where the horizon crosses the box's top or
    // bottom edge. Splitting adaptive Gauss-Legendre at these C0 kinks avoids
    // deep recursive bisection while preserving the exact continuous integral.
    std::array<double,4> boxKinks{};unsigned boxKinkCount=0;
    if(region) {
        const auto addKinkEdge=[&](double yEdge) {
            const double ty=std::tan(yEdge),ny=a+c*ty;
            const double aa=b*b-cutoff*cutoff,bb=2*ny*b,cc=ny*ny-cutoff*cutoff*(1+ty*ty);
            const auto pushRoot=[&](double tx) {
                if(!std::isfinite(tx))return;
                const double x=std::atan(tx);
                if(x>left&&x<rightEdge&&boxKinkCount<4)boxKinks[boxKinkCount++]=x;
            };
            if(std::abs(aa)<1e-14){if(std::abs(bb)>1e-14)pushRoot(-cc/bb);}
            else {const double disc=bb*bb-4*aa*cc;if(disc>=0){const double sq=std::sqrt(disc);pushRoot((-bb-sq)/(2*aa));pushRoot((-bb+sq)/(2*aa));}}
        };
        addKinkEdge(bottom);
        addKinkEdge(top);
        for(unsigned i=1;i<boxKinkCount;++i)for(unsigned j=i;j>0&&boxKinks[j]<boxKinks[j-1];--j)std::swap(boxKinks[j],boxKinks[j-1]);
    }
    for(unsigned col=0;col<p.width;++col) {
        const double px0=(col-.5*p.width)*pitch,px1=px0+pitch;
        const double sx0=std::max(left,px0-margin),sx1=std::min(rightEdge,px1+margin);
        if(!(sx1>sx0))continue;
        if(!region&&(px0-margin<=-1.55||px1+margin>=1.55))return fallback(3);
        const double ix=ThermalImageReceiverV1::gaussianIntervalV1(sx0,sx1,px0,px1,sigma);
        const double tx0=std::tan(sx0),tx1=std::tan(sx1);
        const std::array<double,4> xBounds{std::min(b*tx0,b*tx1),std::max(b*tx0,b*tx1),minSquare(tx0,tx1),std::max(tx0*tx0,tx1*tx1)};
        unsigned active=0;
        for(unsigned row=0;row<p.height;++row) {
            auto& info=rows[row];info.active=false;
            if(!(info.sourceHigh>info.sourceLow))continue;
            const auto range=bounds(xBounds,info);
            if(range[0]>=cutoff)continue;
            if(range[1]<cutoff){answer[static_cast<std::size_t>(row)*p.width+col]=ix*info.full/pixelArea;continue;}
            info.active=true;++active;
        }
        if(!active)continue;
        if(work){++work->activeColumns;work->activeRowVisits+=active;}
        const auto at=[&](double x) {
            if(work){++work->atEvaluations;std::uint64_t bits;std::memcpy(&bits,&x,sizeof(bits));if(!diagnosticSamples.insert(bits).second)++work->repeatedAtSamples;}
            Values values{};const double tx=std::tan(x),n=a+b*tx;
            std::array<double,4> edges{bottom,top,0,0};unsigned size=2;
            const auto addRoot=[&](double t){const double y=std::atan(t);if(y>bottom&&y<top&&size<4)edges[size++]=y;};
            const double aa=c*c-cutoff*cutoff,bb=2*n*c,cc=n*n-cutoff*cutoff*(1+tx*tx);
            if(std::abs(aa)<1e-14){if(std::abs(bb)>1e-14)addRoot(-cc/bb);}
            else {const double disc=bb*bb-4*aa*cc;if(disc>=0){addRoot((-bb-std::sqrt(disc))/(2*aa));addRoot((-bb+std::sqrt(disc))/(2*aa));}}
            // At most two quadratic roots plus the two interval endpoints.
            // Explicit bounded insertion avoids library-sort speculative
            // accesses beyond this four-element array under GCC16.
            if(edges[1]<edges[0])std::swap(edges[1],edges[0]);
            if(size>2) {
                if(edges[2]<edges[1])std::swap(edges[2],edges[1]);
                if(edges[1]<edges[0])std::swap(edges[1],edges[0]);
            }
            if(size>3) {
                if(edges[3]<edges[2])std::swap(edges[3],edges[2]);
                if(edges[2]<edges[1])std::swap(edges[2],edges[1]);
                if(edges[1]<edges[0])std::swap(edges[1],edges[0]);
            }
            std::array<std::array<double,2>,3> ground{};unsigned pieces=0;
            for(unsigned part=1;part<size;++part) {
                const double y=.5*(edges[part-1]+edges[part]);
                const auto ray=unit(add(forward,add(scale(right,tx),scale(vertical,std::tan(y)))));
                if(dot(ray,nativeUp)<cutoff)ground[pieces++]={edges[part-1],edges[part]};
            }
            for(unsigned row=0;row<p.height;++row)if(rows[row].active) {
                const auto& info=rows[row];
                const auto prefix=[&](double t) {
                    if(t<=info.sourceLow)return 0.;
                    if(t>=info.sourceHigh)return info.full;
                    return std::clamp(info.constant-sigma*(primitive((info.high-t)/sigma)-primitive((info.low-t)/sigma)),0.,std::min(t-info.sourceLow,pitch));
                };
                for(unsigned part=0;part<pieces;++part) {
                    const double lo=std::max(info.sourceLow,ground[part][0]),hi=std::min(info.sourceHigh,ground[part][1]);
                    if(hi>lo)values[row]+=std::max(0.,prefix(hi)-prefix(lo));
                }
            }
            return values;
        };
        Values coarse{},fine{};
        if(!region) {
            const auto rule=[&](const auto& uniform,const auto& normal) {
                if(work)++work->ruleEvaluations;
                Values value{};
                for(const auto& u:uniform)for(const auto& z:normal) {
                    const auto sample=at(.5*(px0+px1)+.5*pitch*u.x+sigma*z.x);
                    for(unsigned row=0;row<p.height;++row)if(rows[row].active)value[row]+=u.w*z.w*sample[row]/pitch;
                }
                return value;
            };
            coarse=rule(l2,h3);fine=rule(l3,h5);
            bool converged=true;for(unsigned row=0;row<p.height;++row)if(rows[row].active&&std::abs(fine[row]-coarse[row])>1e-9)converged=false;
            if(!converged) {
                coarse=fine;fine=rule(l4,h10);
                for(unsigned row=0;row<p.height;++row)if(rows[row].active&&std::abs(fine[row]-coarse[row])>1e-9)return fallback(4);
            }
        } else {
            const auto rule=[&](double lo,double hi,const auto& nodes) {
                if(work)++work->ruleEvaluations;
                Values value{};const double middle=.5*(lo+hi),half=.5*(hi-lo);
                for(const auto& node:nodes) {
                    const double x=middle+half*node.x;
                    const double probability=.5*(std::erf((px1-x)/(std::sqrt(2.)*sigma))-std::erf((px0-x)/(std::sqrt(2.)*sigma)));
                    const auto sample=at(x);
                    for(unsigned row=0;row<p.height;++row)if(rows[row].active)value[row]+=2*half*node.w*probability*sample[row]/pixelArea;
                }
                return value;
            };
            bool converged=true;
            const auto integrate=[&](auto&& self,double lo,double hi,double tolerance,unsigned depth)->Values {
                if(work){++work->adaptiveCalls;work->maxRefinementDepth=std::max(work->maxRefinementDepth,10-depth);}
                const auto small=rule(lo,hi,l4),large=rule(lo,hi,l8);
                bool accepted=true;for(unsigned row=0;row<p.height;++row)if(rows[row].active&&std::abs(large[row]-small[row])>tolerance)accepted=false;
                if(accepted)return large;
                if(depth==0){converged=false;return large;}
                const double middle=.5*(lo+hi);auto first=self(self,lo,middle,.5*tolerance,depth-1);
                const auto last=self(self,middle,hi,.5*tolerance,depth-1);
                for(unsigned row=0;row<p.height;++row)first[row]+=last[row];
                return first;
            };
            std::array<double,6> segPts{};unsigned segCount=0;
            segPts[segCount++]=sx0;
            for(unsigned k=0;k<boxKinkCount;++k) {
                const double x=boxKinks[k];
                if(x>segPts[segCount-1]+1e-14&&x<sx1-1e-14)segPts[segCount++]=x;
            }
            segPts[segCount++]=sx1;
            if(segCount==2) {
                fine=integrate(integrate,sx0,sx1,1e-9,10);
            } else {
                const double segTol=1e-9/static_cast<double>(segCount-1);
                fine={};
                for(unsigned s=0;s+1<segCount&&converged;++s) {
                    const auto part=integrate(integrate,segPts[s],segPts[s+1],segTol,10);
                    for(unsigned row=0;row<p.height;++row)fine[row]+=part[row];
                }
            }
            if(!converged)return fallback(5);
        }
        for(unsigned row=0;row<p.height;++row)if(rows[row].active) {
            if(!std::isfinite(fine[row])||fine[row]<0||fine[row]>1+1e-9)return fallback(6);
            answer[static_cast<std::size_t>(row)*p.width+col]=std::clamp(fine[row],0.,1.);
        }
    }
    if(accelerated)*accelerated=true;
    return answer;
}

void ThermalReticleReceiverV1::fitHorizon(ThermalReticleResultV1& result,V pointing,V up,bool* backgroundExplained,bool nullOnly) const {
    fitHorizonWithMeasuredPriorV2(result,pointing,up,backgroundExplained,nullOnly,nullptr);
}

void ThermalReticleReceiverV1::fitHorizonWithMeasuredPriorV2(ThermalReticleResultV1& result,V pointing,V up,
    bool* backgroundExplained,bool nullOnly,const ThermalReticleMeasuredPriorV2* prior) const {
    auto& observed=result.horizonDiagnostic;observed={};observed.ownUp=up;observed.refusal=1;
    if(backgroundExplained)*backgroundExplained=false;
    if(!precomputed_)return;
    const auto& pre=*precomputed_;
    const auto& c=config_;const auto& p=c.detector;const auto count=c.samplesPerScan;
    const double variance=result.varianceBoundElectrons2;
    if(!std::isfinite(variance)||variance<=0)return;
    auto data=result.measuredElectrons;project(data);const double dataNorm=inner(data,data);
    const double subdivisions=std::ldexp(1.,18);
    const double logTrials=2*std::log(p.width*subdivisions+1)+3*std::log(p.height*subdivisions+1);
    const double threshold=std::sqrt(p.detectionSigma*p.detectionSigma+2*(std::log(2.)+logTrials));
    // Neither a nuisance fit nor a compact alternative can explain more
    // energy than the measured vector. This is exact rejection pruning,
    // not a lower source threshold or an approximate optical shortcut.
    observed.refusal=2;
    if(dataNorm/variance<threshold*threshold)return;
    const double alpha=-std::expm1(-1/(c.frequencyHz*count*p.pixelResponseTimeS));
    const double pixelArea=p.pixelPitchRad*p.pixelPitchRad;
    const auto forward=unit(pointing),verticalUp=unit(up);
    auto right=unit(cross(forward,verticalUp));
    if(norm(right)<.5)right=unit(cross(forward,std::abs(forward.z)<.9?V{0,0,1}:V{0,1,0}));
    const auto vertical=unit(cross(right,forward));
    const double margin=9*p.psfSigmaRad;
    const double halfX=.5*p.width*p.pixelPitchRad,halfY=.5*p.height*p.pixelPitchRad;
    const double radius=std::atan(std::hypot(std::tan(std::min(1.55,halfX+margin)),std::tan(std::min(1.55,halfY+margin))));
    const double tilt=std::acos(std::clamp(dot(forward,verticalUp),-1.,1.));
    const double low=std::cos(std::min(pi,tilt+radius)),high=std::cos(std::max(0.,tilt-radius));
    observed.refusal=3;
    if(!(high>low))return;
    const double initialCutStep=(high-low)/p.height;
    std::optional<std::array<double,5>> priorStart;
    // A prior must already be available before any photon in this capture.
    // Domain checks use only existing instrument bounds and exact chronology.
    if(!nullOnly&&prior&&std::isfinite(prior->captureTimeS)&&prior->captureTimeS>=0&&
       std::isfinite(prior->availableTimeS)&&prior->availableTimeS>=prior->captureTimeS&&
       prior->captureTimeS<result.measurementTimeS&&prior->availableTimeS<=result.exposureStartTimeS&&
       finite(prior->direction)&&norm(prior->direction)>0&&finite(prior->directionRate)&&
       std::isfinite(prior->widthRad)&&prior->widthRad>0&&prior->widthRad<=2*halfX&&
       std::isfinite(prior->heightRad)&&prior->heightRad>0&&prior->heightRad<=2*halfY&&
       std::isfinite(prior->cutoff)&&prior->cutoff>=low&&prior->cutoff<=high) {
        const auto predicted=add(prior->direction,scale(prior->directionRate,result.measurementTimeS-prior->captureTimeS));
        if(finite(predicted)&&norm(predicted)>0) {
            const auto direction=unit(predicted);const double front=dot(forward,direction);
            const double x=std::atan2(dot(right,direction),front),y=std::atan2(dot(vertical,direction),front);
            if(front>0&&std::abs(x)<=halfX&&std::abs(y)<=halfY)
                priorStart=std::array<double,5>{{x,y,prior->widthRad,prior->heightRad,prior->cutoff}};
        }
    }
    // Count the EXISTING search, not a scene-tuned allowance. Adjacent equal
    // low samples can both qualify, but three consecutive samples cannot:
    // their middle would have no strict neighbour. Thus at most n-floor(n/3)
    // coarse entries qualify. Each receives36cutoff evaluations. Joint descent
    // adds at most108cutoffs per start (initial plus up to4 existing starts).
    // Selected curvature adds2; each snapshot cutoff adds at most its two
    // neighbouring derivative cutoffs, shared by all reflected source modes.
    constexpr std::size_t refinementLevels=18,coordinateSweeps=3,additionalJointStarts=4;
    constexpr std::size_t maximumJointStarts=1+additionalJointStarts;
    const std::size_t coarseCount=static_cast<std::size_t>(p.height)+1;
    const std::size_t maximumNullMinima=coarseCount-coarseCount/3;
    constexpr std::size_t trustIterations=64,trustTrials=12;
    const std::size_t oldCutoffBudget=coarseCount+2*refinementLevels*maximumNullMinima+
        2*refinementLevels*coordinateSweeps*maximumJointStarts;
    // Every trust iteration visits at most two new cutoff derivative points
    // and trustTrials proposed cutoffs. Other derivative axes reuse a cutoff.
    // A later finite snapshot tests every visited cutoff, including rejected
    // trust proposals. Each snapshot cutoff may add two derivative neighbours;
    // they are shared by its reflected modes and are not recursively profiled.
    // A valid measured start adds at most the previous accepted cutoff and
    // current null cutoff across up to 3 roll-rotated extent starts ((w,h),
    // (h,w), and equal-side envelope). This is a derived work bound, never a
    // new likelihood, iteration, geometric or false-alarm allowance. No-prior
    // count is exact old.
    const std::size_t priorTrustStarts=priorStart?6u:0u;
    const std::size_t trustCutoffBudget=(maximumJointStarts+priorTrustStarts)*(1+trustIterations*(2+trustTrials));
    const std::size_t maximumCalibrations=3*(oldCutoffBudget+trustCutoffBudget+2);
    struct Calibration {
        double cutoff=0,cost=std::numeric_limits<double>::infinity(),amplitude=0;
        std::vector<double> wave,rawWave;
    };
    // All scenes below are INSTRUMENT HYPOTHESES: unit ground, zero sky,
    // unknown cutoff and optional black support. No actual scene radiance,
    // source rectangle, identity or horizon threshold reaches this inverse.
    struct GroundFieldEntry {double cutoff=0;std::vector<double> field;};
    std::vector<GroundFieldEntry> groundFieldCache;groundFieldCache.reserve(4);
    const UnitGroundPrepV1 groundPrep=UnitGroundPrepV1::build(p,forward,verticalUp);
    const auto groundField=[&](double cutoff)->const std::vector<double>* {
        for(std::size_t idx=0;idx<groundFieldCache.size();++idx) {
            if(cutoff==groundFieldCache[idx].cutoff&&std::signbit(cutoff)==std::signbit(groundFieldCache[idx].cutoff)) {
                if(idx!=0)std::rotate(groundFieldCache.begin(),groundFieldCache.begin()+static_cast<std::ptrdiff_t>(idx),
                    groundFieldCache.begin()+static_cast<std::ptrdiff_t>(idx+1));
                return &groundFieldCache[0].field;
            }
        }
        auto field=groundPrep.evaluateFullFrame(cutoff);if(!field)return nullptr;
        if(groundFieldCache.size()>=4)groundFieldCache.pop_back();
        groundFieldCache.insert(groundFieldCache.begin(),GroundFieldEntry{cutoff,std::move(*field)});
        return &groundFieldCache[0].field;
    };
    const auto waveform=[&](const std::vector<double>& field,bool removeNuisance) {
        std::vector<double> wave(count,0);
        const double* fPtr=field.data();
        const double* wwPtr=pre.weightedWeights.data();
        const double* swwPtr=pre.symWeightedWeights.data();
        const double* rwPtr=pre.rowFullWave.data();
        const unsigned symCols=(p.width+1u)/2u;
        for(unsigned row=0;row<p.height;++row) {
            const double* fRow=fPtr+static_cast<std::size_t>(row)*p.width;
            const double firstVal=fRow[0];
            bool allEqual=true;
            for(unsigned col=1;col<p.width;++col)if(fRow[col]!=firstVal){allEqual=false;break;}
            if(allEqual) {
                if(firstVal==0.)continue;
                const double* rw=rwPtr+static_cast<std::size_t>(row)*count;
                if(firstVal==1.) {
                    for(unsigned j=0;j<count;++j)wave[j]+=rw[j];
                } else {
                    for(unsigned j=0;j<count;++j)wave[j]+=firstVal*rw[j];
                }
                continue;
            }
            bool rowSym=true;
            for(unsigned col=0;col<symCols;++col) {
                if(fRow[col]!=fRow[p.width-1u-col]){rowSym=false;break;}
            }
            if(rowSym) {
                const std::size_t symRowBase=static_cast<std::size_t>(row)*symCols;
                for(unsigned col=0;col<symCols;++col) {
                    const double f=fRow[col];
                    if(f==0.)continue;
                    const double* sww=swwPtr+(symRowBase+col)*count;
                    for(unsigned j=0;j<count;++j)wave[j]+=f*sww[j];
                }
                continue;
            }
            const std::size_t rowBase=static_cast<std::size_t>(row)*p.width;
            for(unsigned col=0;col<p.width;++col) {
                const double f=fRow[col];
                if(f==0.)continue;
                const double* ww=wwPtr+(rowBase+col)*count;
                for(unsigned j=0;j<count;++j)wave[j]+=f*ww[j];
            }
        }
        wave=filtered(std::move(wave),alpha);if(removeNuisance)project(wave);return wave;
    };
    const auto cutoffKey=[](double cutoff) noexcept {
        const double canonical=cutoff==0.?0.:cutoff;
        std::uint64_t bits=0;std::memcpy(&bits,&canonical,sizeof(bits));return bits;
    };
    // Precompute the unprojected sky/transient regression basis ONCE per
    // fitHorizon() call so every calibration(cutoff) evaluates the exact
    // rawSkyFitV1 projection in O(count) with zero allocations or std::pow().
    auto uniform=filtered(pre.flat,alpha);for(auto& value:uniform)value/=pixelArea;
    std::vector<double> transient(count),dark(count,0),remaining0=result.measuredElectrons;
    double darkState=0;
    const double darkInput=static_cast<double>(p.width)*p.height*p.darkCurrentElectronsS*p.integrationTimeS;
    for(unsigned j=0;j<count;++j) {
        transient[j]=std::pow(1-alpha,j+1);darkState+=alpha*(darkInput-darkState);dark[j]=darkState;remaining0[j]-=darkState;
    }
    const double ut=inner(uniform,transient),tt=inner(transient,transient),uu=inner(uniform,uniform),det=uu*tt-ut*ut;
    const bool hasTransient=tt>0;
    const bool skyBasisOk=(uu>0)&&std::isfinite(uu)&&std::isfinite(tt)&&(!hasTransient||(det>128*std::numeric_limits<double>::epsilon()*uu*tt));
    const double skyBase=skyBasisOk?(hasTransient?(inner(remaining0,uniform)*tt-inner(remaining0,transient)*ut)/det:inner(remaining0,uniform)/uu):0.;
    const bool skyBaseFinite=skyBasisOk&&std::isfinite(skyBase);
    double nullExplained=0,nullCutoff=low,nullAmplitude=0,nullCost=std::numeric_limits<double>::infinity();
    std::vector<Calibration> calibrations;calibrations.reserve(64);
    std::unordered_map<std::uint64_t,std::size_t> calibrationIndex;
    // Open-addressed generation-stamped thread-local flat cache for
    // zero-allocation, zero-memset null-hypothesis cutoff search.
    struct NullCacheSlot {
        std::uint64_t stamp=0;
        std::uint64_t key=0;
        double cutoff=0,cost=std::numeric_limits<double>::infinity(),amplitude=0;
    };
    static constexpr std::size_t kNullTableSize=4096u;
    thread_local std::array<NullCacheSlot,kNullTableSize> nullTable{};
    thread_local std::uint64_t nullEpoch=0;
    if(++nullEpoch==0){for(auto& slot:nullTable)slot.stamp=0;nullEpoch=1;}
    const std::uint64_t curNullEpoch=nullEpoch;
    std::size_t uniqueVisitedCutoffs=0;
    const auto nullSlotFind=[&](std::uint64_t key)->NullCacheSlot* {
        std::size_t h=static_cast<std::size_t>((key^(key>>30))*0xbf58476d1ce4e5b9ull)&(kNullTableSize-1u);
        for(std::size_t step=0;step<kNullTableSize;++step) {
            auto& slot=nullTable[(h+step)&(kNullTableSize-1u)];
            if(slot.stamp!=curNullEpoch||slot.key==key)return &slot;
        }
        return nullptr;
    };
    const auto calibration=[&](double cutoff)->const Calibration* {
        const auto key=cutoffKey(cutoff);
        if(const auto found=calibrationIndex.find(key);found!=calibrationIndex.end())return &calibrations[found->second];
        auto* nSlot=nullSlotFind(key);
        const bool alreadyNullVisited=nSlot&&nSlot->stamp==curNullEpoch;
        if(!alreadyNullVisited&&uniqueVisitedCutoffs>=maximumCalibrations)return nullptr;
        Calibration item;item.cutoff=cutoff;
        if(!groundPrep.evaluateRawWaveform(cutoff,pre.weightedWeights.data(),pre.symWeightedWeights.data(),
            pre.rowFullWavePrefix.data(),pre.rowColPrefix.empty()?nullptr:pre.rowColPrefix.data(),
            pre.rowMomentWave,count,alpha,item.rawWave)) {
            const auto* field=groundField(cutoff);
            if(!field)return nullptr;
            item.rawWave=waveform(*field,false);
        }
        item.wave=item.rawWave;project(item.wave);
        calibrationIndex.emplace(key,calibrations.size());
        if(!alreadyNullVisited)++uniqueVisitedCutoffs;
        calibrations.push_back(std::move(item));auto& added=calibrations.back();
        // The null sees EVERY cutoff visited by any alternative, including
        // derivative and alias hypotheses. Improved background fitting can
        // therefore never be credited exclusively to the compact source.
        const double norm2=inner(added.wave,added.wave);
        if(norm2>128*std::numeric_limits<double>::epsilon()) {
            const double correlation=inner(data,added.wave);
            if(!skyBaseFinite)return &added;
            const double projection=hasTransient?(inner(added.rawWave,uniform)*tt-inner(added.rawWave,transient)*ut)/det:inner(added.rawWave,uniform)/uu;
            if(!std::isfinite(projection))return &added;
            const auto physical=physicalBackgroundAmplitudeV1(skyBase,projection,correlation/norm2);
            if(!physical)return &added;
            const double amplitude=*physical;
            double cost=0;for(unsigned j=0;j<count;++j){const double residual=data[j]-amplitude*added.wave[j];cost+=residual*residual;}
            added.cost=cost;added.amplitude=amplitude;
            if(cost<nullCost){nullCost=cost;nullExplained=std::max(0.,dataNorm-cost);nullCutoff=cutoff;nullAmplitude=amplitude;}
        }
        return &added;
    };
    struct NullFit {double cutoff,cost,amplitude;};
    std::array<double,256> stackWave{};
    std::vector<double> heapWave;
    double* nullWavePtr=count<=stackWave.size()?stackWave.data():(heapWave.resize(count),heapWave.data());
    const auto evalNullOnly=[&](double cutoff,NullFit& out)->bool {
        const auto key=cutoffKey(cutoff);
        auto* nSlot=nullSlotFind(key);
        if(nSlot&&nSlot->stamp==curNullEpoch) {
            out={nSlot->cutoff,nSlot->cost,nSlot->amplitude};
            return true;
        }
        if(const auto found=calibrationIndex.find(key);found!=calibrationIndex.end()) {
            const auto& cItem=calibrations[found->second];
            out={cItem.cutoff,cItem.cost,cItem.amplitude};
            return true;
        }
        if(uniqueVisitedCutoffs>=maximumCalibrations)return false;
        double dotUnif=0.,dotTran=0.;
        if(groundPrep.evaluateProjectedNullV1(cutoff,
            pre.projRowFullWavePrefix.data(),pre.projRowMomentWave,
            pre.projSymWeightedWeights.data(),pre.symUniformDot.data(),pre.symTransientDot.data(),
            pre.rowFullUniformDotPrefix.data(),pre.rowFullTransientDotPrefix.data(),
            pre.rowMomentUniformDot,pre.rowMomentTransientDot,
            count,nullWavePtr,dotUnif,dotTran)) {
            ++uniqueVisitedCutoffs;
            double norm2=0.,correlation=0.;
            const double* dPtr=data.data();
            reticleNormDot(dPtr,nullWavePtr,count,norm2,correlation);
            double cost=std::numeric_limits<double>::infinity(),amplitude=0.;
            if(norm2>128*std::numeric_limits<double>::epsilon()&&skyBaseFinite) {
                const double projection=hasTransient?(dotUnif*tt-dotTran*ut)/det:dotUnif/uu;
                if(std::isfinite(projection)) {
                    if(const auto physical=physicalBackgroundAmplitudeV1(skyBase,projection,correlation/norm2)) {
                        amplitude=*physical;
                        cost=reticleResidualNorm2(dPtr,nullWavePtr,amplitude,count);
                        if(cost<nullCost){nullCost=cost;nullExplained=std::max(0.,dataNorm-cost);nullCutoff=cutoff;nullAmplitude=amplitude;}
                    }
                }
            }
            if(nSlot&&nSlot->stamp!=curNullEpoch){nSlot->stamp=curNullEpoch;nSlot->key=key;nSlot->cutoff=cutoff;nSlot->cost=cost;nSlot->amplitude=amplitude;}
            out={cutoff,cost,amplitude};
            return true;
        }
        const auto* item=calibration(cutoff);
        if(!item)return false;
        out={item->cutoff,item->cost,item->amplitude};
        return true;
    };
    std::vector<NullFit> coarse,minima;
    coarse.reserve(static_cast<std::size_t>(p.height)+1u);
    for(unsigned row=0;row<=p.height;++row) {
        NullFit item{};
        if(evalNullOnly(low+(high-low)*row/p.height,item))coarse.push_back(item);
    }
    // A lower coarse sample does not identify the basin containing the best
    // continuous null. Refine EVERY strict measured coarse local minimum in
    // its own state. Flat, unmodulated plateaus cannot improve the null.
    bool nullSearchComplete=coarse.size()==static_cast<std::size_t>(p.height)+1;
    for(std::size_t index=0;index<coarse.size();++index) {
        const auto seed=coarse[index];
        const double left=index?coarse[index-1].cost:std::numeric_limits<double>::infinity();
        const double rightCost=index+1<coarse.size()?coarse[index+1].cost:std::numeric_limits<double>::infinity();
        if(!std::isfinite(seed.cost)||seed.cost>left||seed.cost>rightCost||
           !(seed.cost<left||seed.cost<rightCost))continue;
        auto local=seed;double cutStep=initialCutStep;
        for(std::size_t level=0;level<refinementLevels&&nullSearchComplete;++level) {
            cutStep*=.5;const double centre=local.cutoff;
            for(const double sign:{-1.,1.}) {
                const double cutoff=centre+sign*cutStep;
                if(cutoff<low||cutoff>high)continue;
                NullFit item{};
                if(!evalNullOnly(cutoff,item)){nullSearchComplete=false;break;}
                if(item.cost<local.cost)local=item;
            }
        }
        minima.push_back(local);
        if(!nullSearchComplete)break;
    }
    std::stable_sort(minima.begin(),minima.end(),[](const NullFit& a,const NullFit& b){return a.cost<b.cost;});
    observed.nullCutoff=nullCutoff;observed.nullResidual=nullCost/variance;observed.refusal=4;
    if(!nullSearchComplete&&nullCost/variance>=threshold*threshold){observed.refusal=21;return;}
    if(nullExplained/variance<threshold*threshold)return;
    observed.refusal=5;
    if(nullCost/variance<threshold*threshold){if(backgroundExplained)*backgroundExplained=true;return;}
    // Exceptional exhaustion remains numerical unavailability, never proof
    // that a source-free null failed. Normal work is bounded above by the
    // admitted grid/refinement/start counts rather than an arbitrary512cap.
    observed.refusal=6;
    if(nullOnly)return;
    observed.refusal=7;
    const auto* nullBasis=calibration(nullCutoff);if(!nullBasis)return;
    auto compactData=data;
    for(unsigned j=0;j<count;++j)compactData[j]-=nullAmplitude*nullBasis->wave[j];
    double startExplained=0,startX=0,startY=0,startWidth=0,startHeight=0;
    const auto considerStart=[&](const double* values,double x,double y,double width,double height) {
        double correlation=0,norm2=0;
        for(unsigned j=0;j<count;++j){correlation+=compactData[j]*values[j];norm2+=values[j]*values[j];}
        if(norm2<=128*std::numeric_limits<double>::epsilon())return;
        const double explained=correlation*correlation/norm2;
        if(explained>startExplained){startExplained=explained;startX=x;startY=y;startWidth=width;startHeight=height;}
    };
    const auto scanPointTemplates=[&]() {
        const double* cPtr=compactData.data();
        const double* tBase=pre.templates.data();
        const double* invBase=pre.templateInvNorm2.data();
        for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
            const auto idx=static_cast<std::size_t>(row)*p.width+col;
            const double invN2=invBase[idx];
            if(invN2==0.)continue;
            const double* vals=tBase+idx*count;
            double corr=0;
            for(unsigned j=0;j<count;++j)corr+=cPtr[j]*vals[j];
            const double explained=corr*corr*invN2;
            if(explained>startExplained) {
                startExplained=explained;
                startX=(col+.5-.5*p.width)*p.pixelPitchRad;startY=(row+.5-.5*p.height)*p.pixelPitchRad;
                startWidth=0;startHeight=0;
            }
        }
    };
    scanPointTemplates();
    for(const double w:{0.,1.,2.,4.,8.,16.,32.})for(const double h:{0.,1.,2.,4.,8.,16.,32.}) {
        const auto values=spot(startX,startY,p.psfSigmaRad,w*p.pixelPitchRad,h*p.pixelPitchRad);
        considerStart(values.data(),startX,startY,w*p.pixelPitchRad,h*p.pixelPitchRad);
    }
    using Parameters=std::array<double,5>;
    struct ShapeWave {std::array<double,4> shape;bool projected;std::vector<double> wave;};
    bool cacheShapes=false;std::vector<ShapeWave> shapeWaves;shapeWaves.reserve(128);
    const auto sourceWave=[&](const Parameters& q,bool removeNuisance) {
        const std::array<double,4> shape{{q[0],q[1],q[2],q[3]}};
        if(cacheShapes)for(const auto& item:shapeWaves)
            if(item.shape==shape&&item.projected==removeNuisance)return item.wave;
        auto wave=spot(q[0],q[1],p.psfSigmaRad,q[2],q[3],removeNuisance);
        if(cacheShapes&&shapeWaves.size()<128)shapeWaves.push_back({shape,removeNuisance,wave});
        return wave;
    };
    struct HorizonBasis {std::array<std::vector<double>,2> values;bool sourceRelativeGround=false;};
    const double horizA=dot(forward,verticalUp),horizB=dot(right,verticalUp),horizC=dot(vertical,verticalUp);
    const double fieldLeft=std::max(-1.55,-halfX-margin),fieldRight=std::min(1.55,halfX+margin);
    const double fieldBottom=std::max(-1.55,-halfY-margin),fieldTop=std::min(1.55,halfY+margin);
    const auto boxLevelBounds=[&](double x0,double x1,double y0,double y1)->std::array<double,2> {
        const double xl=std::tan(x0),xr=std::tan(x1),yl=std::tan(y0),yr=std::tan(y1);
        const double nlo=horizA+std::min(horizB*xl,horizB*xr)+std::min(horizC*yl,horizC*yr);
        const double nhi=horizA+std::max(horizB*xl,horizB*xr)+std::max(horizC*yl,horizC*yr);
        const auto nearest=[](double l,double r){return l<=0&&r>=0?0.:std::min(std::abs(l),std::abs(r));};
        const double dx=nearest(xl,xr),dy=nearest(yl,yr);
        const double dlo=std::sqrt(1+dx*dx+dy*dy),dhi=std::sqrt(1+std::max(xl*xl,xr*xr)+std::max(yl*yl,yr*yr));
        const double err=128*std::numeric_limits<double>::epsilon()*(1+std::abs(horizA)+std::abs(horizB)+std::abs(horizC));
        return {{std::nextafter(nlo/(nlo>=0?dhi:dlo)-err,-std::numeric_limits<double>::infinity()),
                 std::nextafter(nhi/(nhi>=0?dlo:dhi)+err,std::numeric_limits<double>::infinity())}};
    };
    struct StaticConicBankEntry {double cutoff=0;std::shared_ptr<conic_moments_v4::Bank> bank;};
    std::vector<StaticConicBankEntry> staticConicBanks;staticConicBanks.reserve(8);
    conic_moments_v4::FieldWorkspaceV5 staticConicWs;
    const bool conicArithOk=conic_moments_v4::arithmeticAvailable();
    const auto conicEligible=[&](const std::array<double,4>& reg)->bool {
        if(!conicArithOk)return false;
        const double leftE=std::max(fieldLeft,reg[0]),rightE=std::min(fieldRight,reg[1]);
        const double botE=std::max(fieldBottom,reg[2]),topE=std::min(fieldTop,reg[3]);
        if(!(rightE>leftE)||!(topE>botE))return true;
        const double maxTx=std::max(std::abs(std::tan(leftE)),std::abs(std::tan(rightE)));
        const double maxTy=std::max(std::abs(std::tan(botE)),std::abs(std::tan(topE)));
        const double maxN=std::nextafter(std::abs(horizA)+std::nextafter(std::abs(horizB)*maxTx,std::numeric_limits<double>::infinity()),
            std::numeric_limits<double>::infinity());
        return std::nextafter(horizC,-std::numeric_limits<double>::infinity())>
            std::nextafter(maxN*maxTy,std::numeric_limits<double>::infinity());
    };
    const auto getStaticConicBank=[&](double cutoff)->std::shared_ptr<conic_moments_v4::Bank> {
        if(!conicArithOk)return nullptr;
        for(std::size_t idx=0;idx<staticConicBanks.size();++idx) {
            if(cutoff==staticConicBanks[idx].cutoff&&std::signbit(cutoff)==std::signbit(staticConicBanks[idx].cutoff)) {
                if(idx!=0)std::rotate(staticConicBanks.begin(),staticConicBanks.begin()+static_cast<std::ptrdiff_t>(idx),
                    staticConicBanks.begin()+static_cast<std::ptrdiff_t>(idx+1));
                return staticConicBanks.front().bank;
            }
        }
        conic_moments_v4::Request req;req.width=p.width;req.height=p.height;
        req.pixelPitchRad=p.pixelPitchRad;req.psfSigmaRad=p.psfSigmaRad;req.cutoff=cutoff;
        req.pointing={{forward.x,forward.y,forward.z}};req.up={{verticalUp.x,verticalUp.y,verticalUp.z}};
        auto bank=std::shared_ptr<conic_moments_v4::Bank>(conic_moments_v4::Bank::prepareSharedV5(req));
        if(staticConicBanks.size()>=8)staticConicBanks.pop_back();
        staticConicBanks.insert(staticConicBanks.begin(),StaticConicBankEntry{cutoff,bank});
        return bank;
    };
    const auto conicSubtractOcculted=[&](const conic_moments_v4::FieldWorkspaceV5& ws,const std::vector<double>& baseRaw,std::vector<double>& outRaw) {
        outRaw=baseRaw;
        if(ws.activeRowCount==0||ws.colEnd<=ws.colBegin)return;
        std::vector<double> occRaw(count,0.);
        const double* fPtr=ws.field.data();
        const double* wwPtr=pre.weightedWeights.data();
        bool anyNonZero=false;
        for(unsigned r=0;r<ws.activeRowCount;++r) {
            const unsigned row=ws.activeRows[r];
            const std::size_t rowBase=static_cast<std::size_t>(row)*p.width;
            for(unsigned col=ws.colBegin;col<ws.colEnd;++col) {
                const double f=fPtr[rowBase+col];
                if(f==0.)continue;
                anyNonZero=true;
                reticleFmadd1(occRaw.data(),wwPtr+(rowBase+col)*count,f,count);
            }
        }
        if(!anyNonZero)return;
        occRaw=filtered(std::move(occRaw),alpha);
        for(unsigned j=0;j<count;++j)outRaw[j]-=occRaw[j];
    };
    std::optional<Parameters> lastOutsideShape;
    std::vector<double> lastOutsideRaw;
    bool lastOutsideRelativeGround=false;
    const auto basis=[&](const Parameters& q,bool removeNuisance)->std::optional<HorizonBasis> {
        const auto* ground=calibration(q[4]);if(!ground)return std::nullopt;
        bool sameOutside=lastOutsideShape.has_value();
        if(sameOutside)for(unsigned coordinate=0;coordinate<5;++coordinate) {
            if(q[coordinate]!=(*lastOutsideShape)[coordinate]||
               std::signbit(q[coordinate])!=std::signbit((*lastOutsideShape)[coordinate])){sameOutside=false;break;}
        }
        std::vector<double> outside;
        bool sourceRelativeGround=false;
        if(sameOutside){outside=lastOutsideRaw;sourceRelativeGround=lastOutsideRelativeGround;}
        else {
            if(q[2]>0&&q[3]>0) {
                const double x0=q[0]-.5*q[2],x1=q[0]+.5*q[2],y0=q[1]-.5*q[3],y1=q[1]+.5*q[3];
                const double cx0=std::max(fieldLeft,x0),cx1=std::min(fieldRight,x1);
                const double cy0=std::max(fieldBottom,y0),cy1=std::min(fieldTop,y1);
                if(!(cx1>cx0)||!(cy1>cy0)) {
                    outside=ground->rawWave;
                } else {
                    const auto levels=boxLevelBounds(cx0,cx1,cy0,cy1);
                    if(levels[0]>=q[4]) {
                        outside=ground->rawWave;
                    } else if(levels[1]<q[4]&&x0>=-halfX-margin&&x1<=halfX+margin&&y0>=-halfY-margin&&y1<=halfY+margin) {
                        // Exact linear change of coordinates: wholly ground-backed
                        // support has contrast relative to ground, identical to
                        // MovingRadiatorInverseV2::basis.
                        outside=ground->rawWave;
                        sourceRelativeGround=true;
                    } else {
                        const std::array<double,4> support{x0,x1,y0,y1};
                        bool usedConic=false;
                        if(conicEligible(support)) {
                            if(const auto bank=getStaticConicBank(q[4]);bank&&bank->evaluateFastIntoV5(support,staticConicWs)) {
                                conicSubtractOcculted(staticConicWs,ground->rawWave,outside);
                                usedConic=true;
                            }
                        }
                        if(!usedConic) {
                            const auto occulted=unitGroundFieldMemoV2(p,forward,verticalUp,q[4],support,
                                [&]()->const std::vector<double>*{return groundField(q[4]);});
                            if(!occulted)return std::nullopt;
                            if(std::all_of(occulted->begin(),occulted->end(),[](double value){return value==0;}))outside=ground->rawWave;
                            else {
                                // Linear waveform subtraction matches waveform(*full - *occulted, false)
                                // while avoiding full-frame groundField recomputation and zero pixels.
                                const auto occWave=waveform(*occulted,false);
                                outside=ground->rawWave;
                                for(unsigned j=0;j<count;++j)outside[j]-=occWave[j];
                            }
                        }
                    }
                }
            } else outside=ground->rawWave;
            lastOutsideShape=q;lastOutsideRaw=outside;lastOutsideRelativeGround=sourceRelativeGround;
        }
        if(removeNuisance)project(outside);
        return HorizonBasis{std::array<std::vector<double>,2>{std::move(outside),sourceWave(q,removeNuisance)},sourceRelativeGround};
    };
    const auto effectiveSource=[&](const Parameters& q,const HorizonBasis& hb,double b,double d) {
        return d-(hb.sourceRelativeGround?q[2]*q[3]/pixelArea*b:0.);
    };
    // Recover a necessary physical radiance budget from the unprojected ADC
    // waveform. Uniform radiance and the unknown initial pole state are fit
    // jointly; no radiance is copied from the actual scene.
    observed.refusal=10;
    if(!(uu>0)||!(tt>=0))return;
    observed.refusal=11;
    if(hasTransient&&!(det>128*std::numeric_limits<double>::epsilon()*uu*tt))return;
    const auto physicalBudget=[&](const Parameters& q,double b,double d,std::array<double,3>* budget=nullptr) {
        const auto raw=basis(q,false);if(!raw)return false;
        const double dEff=effectiveSource(q,*raw,b,d);
        auto remaining=result.measuredElectrons;
        for(unsigned j=0;j<count;++j)remaining[j]-=b*raw->values[0][j]+dEff*raw->values[1][j]+dark[j];
        const double sky=hasTransient?(inner(remaining,uniform)*tt-inner(remaining,transient)*ut)/det:inner(remaining,uniform)/uu;
        const double areaCells=q[2]*q[3]/pixelArea;
        const double ground=sky+b,radiator=areaCells>0?sky+d/areaCells:d;
        if(budget){*budget={sky,ground,radiator};observed.budgetAvailable=true;}
        return std::isfinite(sky)&&std::isfinite(ground)&&std::isfinite(radiator)&&sky>=0&&ground>=0&&radiator>=0;
    };
    double bestCost=std::numeric_limits<double>::infinity(),beta=0,contrast=0;
    Parameters bestQ{startX,startY,startWidth,startHeight,nullCutoff};
    const auto solve=[&](const Parameters& q,std::vector<double>* reduced=nullptr)->std::optional<std::array<double,3>> {
        if(std::abs(q[0])>halfX||std::abs(q[1])>halfY||q[2]<0||q[3]<0||q[2]>2*halfX||q[3]>2*halfY||q[4]<low||q[4]>high)return std::nullopt;
        const auto hb=basis(q,true);if(!hb)return std::nullopt;
        const auto& u=hb->values[0];const auto& v=hb->values[1];
        const double backgroundNorm=inner(u,u),uv=inner(u,v),vv=inner(v,v);
        const double determinant=backgroundNorm*vv-uv*uv;
        if(backgroundNorm<=0||vv<=0||determinant<=128*std::numeric_limits<double>::epsilon()*backgroundNorm*vv)return std::nullopt;
        const double yu=inner(data,u),yv=inner(data,v);
        const double b=(yu*vv-yv*uv)/determinant,d=(yv*backgroundNorm-yu*uv)/determinant;
        if(reduced)reduced->resize(count);
        double cost=0;for(unsigned j=0;j<count;++j) {
            const double residual=data[j]-b*u[j]-d*v[j];cost+=residual*residual;
            if(reduced)(*reduced)[j]=residual/std::sqrt(variance);
        }
        const double dSky=d+(hb->sourceRelativeGround?q[2]*q[3]/pixelArea*b:0.);
        if(!std::isfinite(cost)||!std::isfinite(b)||!std::isfinite(dSky))return std::nullopt;
        return std::array<double,3>{cost,b,dSky};
    };
    const auto consider=[&](const Parameters& q) {
        const auto fit=solve(q);if(!fit)return;
        if((*fit)[0]<bestCost&&physicalBudget(q,(*fit)[1],(*fit)[2])) {
            bestCost=(*fit)[0];bestQ=q;beta=(*fit)[1];contrast=(*fit)[2];
        }
    };
    const double degrees=static_cast<double>(count)-static_cast<double>(pre.nuisance.size())-7,tail=.5*p.detectionSigma*p.detectionSigma;
    const double residualBound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    const auto descend=[&](const Parameters& initial) {
        bestQ=initial;bestCost=std::numeric_limits<double>::infinity();beta=0;contrast=0;
        consider(initial);
        double step=p.pixelPitchRad,cutStep=initialCutStep;
        for(std::size_t level=0;level<refinementLevels;++level) {
            step*=.5;cutStep*=.5;
            for(std::size_t sweep=0;sweep<coordinateSweeps;++sweep) {
                bool moved=false;
                for(unsigned coordinate=0;coordinate<5;++coordinate) {
                    const auto centre=bestQ;
                    const double beforeCost=bestCost;
                    for(const double sign:{-1.,1.}){auto q=centre;q[coordinate]+=sign*(coordinate==4?cutStep:step);consider(q);}
                    if(bestCost<beforeCost)moved=true;
                }
                if(!moved)break;
            }
        }
    };
    const auto start=bestQ;descend(start);
    struct JointFit {Parameters shape;double cost,background,source;};
    JointFit selected{bestQ,bestCost,beta,contrast};
    // Different measured null basins receive independent local descent.
    // Their source centre/support is still selected from measured residuals;
    // neither a target label nor its actual geometry supplies a start.
    const auto jointBudget=std::min(additionalJointStarts,minima.size());
    for(std::size_t index=0;index<jointBudget&&selected.cost/variance>residualBound;++index) {
        const auto& minimum=minima[index];if(minimum.cutoff==nullCutoff)continue;
        const auto* ground=calibration(minimum.cutoff);if(!ground)break;
        compactData=data;for(unsigned j=0;j<count;++j)compactData[j]-=minimum.amplitude*ground->wave[j];
        startExplained=0;startX=0;startY=0;startWidth=0;startHeight=0;
        scanPointTemplates();
        for(const double w:{0.,1.,2.,4.,8.,16.,32.})for(const double h:{0.,1.,2.,4.,8.,16.,32.}) {
            const auto wave=spot(startX,startY,p.psfSigmaRad,w*p.pixelPitchRad,h*p.pixelPitchRad);
            considerStart(wave.data(),startX,startY,w*p.pixelPitchRad,h*p.pixelPitchRad);
        }
        descend({startX,startY,startWidth,startHeight,minimum.cutoff});
        if(bestCost<selected.cost)selected={bestQ,bestCost,beta,contrast};
    }
    if(selected.cost/variance>residualBound) {
        // A single winning point correlation can select a horizon-shaped
        // basin whose unconstrained amplitudes are not physically feasible.
        // Refine the complete two-amplitude reduced residual in independent
        // LOCAL state; only physical candidates compete for selected output.
        const double angularLattice=p.pixelPitchRad/std::ldexp(1.,18);
        const double cutoffLattice=initialCutStep/std::ldexp(1.,18);
        const Parameters lower{{-halfX,-halfY,0,0,low}};
        const Parameters upper{{halfX,halfY,4*halfX*halfX,4*halfY*halfY,high}};
        const auto squared=[](Parameters q){q[2]*=q[2];q[3]*=q[3];return q;};
        const auto geometry=[](Parameters q){q[2]=std::sqrt(std::max(0.,q[2]));q[3]=std::sqrt(std::max(0.,q[3]));return q;};
        const auto quantize=[&](Parameters q) {
            q=geometry(q);
            for(unsigned coordinate=0;coordinate<4;++coordinate)q[coordinate]=std::round(q[coordinate]/angularLattice)*angularLattice;
            q[4]=low+std::round((q[4]-low)/cutoffLattice)*cutoffLattice;
            q[0]=std::clamp(q[0],-halfX,halfX);q[1]=std::clamp(q[1],-halfY,halfY);
            q[2]=std::clamp(q[2],0.,2*halfX);q[3]=std::clamp(q[3],0.,2*halfY);q[4]=std::clamp(q[4],low,high);
            return squared(q);
        };
        const auto record=[&](const Parameters& q,const std::array<double,3>& fit) {
            if(fit[0]<selected.cost&&physicalBudget(q,fit[1],fit[2]))selected={q,fit[0],fit[1],fit[2]};
        };
        const auto needsRegional=[&](const Parameters& q)->bool {
            if(!(q[2]>0&&q[3]>0))return false;
            const double x0=q[0]-.5*q[2],x1=q[0]+.5*q[2],y0=q[1]-.5*q[3],y1=q[1]+.5*q[3];
            const double cx0=std::max(fieldLeft,x0),cx1=std::min(fieldRight,x1);
            const double cy0=std::max(fieldBottom,y0),cy1=std::min(fieldTop,y1);
            if(!(cx1>cx0)||!(cy1>cy0))return false;
            const auto levels=boxLevelBounds(cx0,cx1,cy0,cy1);
            if(levels[0]>=q[4])return false;
            if(levels[1]<q[4]&&x0>=-halfX-margin&&x1<=halfX+margin&&y0>=-halfY-margin&&y1<=halfY+margin)return false;
            return true;
        };
        const auto solvePure=[&](const Parameters& q,const std::vector<double>& groundRawWave,
            const std::vector<double>* fullGroundField,const conic_moments_v4::Bank* bankPtr,
            conic_moments_v4::FieldWorkspaceV5& ws,std::vector<double>& reduced)->bool {
            if(std::abs(q[0])>halfX||std::abs(q[1])>halfY||q[2]<0||q[3]<0||q[2]>2*halfX||q[3]>2*halfY||q[4]<low||q[4]>high)return false;
            std::vector<double> outside;
            bool sourceRelativeGround=false;
            if(q[2]>0&&q[3]>0) {
                const double x0=q[0]-.5*q[2],x1=q[0]+.5*q[2],y0=q[1]-.5*q[3],y1=q[1]+.5*q[3];
                const double cx0=std::max(fieldLeft,x0),cx1=std::min(fieldRight,x1);
                const double cy0=std::max(fieldBottom,y0),cy1=std::min(fieldTop,y1);
                if(!(cx1>cx0)||!(cy1>cy0))outside=groundRawWave;
                else {
                    const auto levels=boxLevelBounds(cx0,cx1,cy0,cy1);
                    if(levels[0]>=q[4])outside=groundRawWave;
                    else if(levels[1]<q[4]&&x0>=-halfX-margin&&x1<=halfX+margin&&y0>=-halfY-margin&&y1<=halfY+margin) {
                        outside=groundRawWave;sourceRelativeGround=true;
                    } else {
                        const std::array<double,4> support{x0,x1,y0,y1};
                        bool usedConic=false;
                        if(bankPtr&&conicEligible(support)&&bankPtr->evaluateFastIntoV5(support,ws)) {
                            conicSubtractOcculted(ws,groundRawWave,outside);
                            usedConic=true;
                        }
                        if(!usedConic) {
                            const auto occulted=unitGroundFieldMemoV2(p,forward,verticalUp,q[4],support,
                                [&]()->const std::vector<double>*{return fullGroundField;});
                            if(!occulted)return false;
                            if(std::all_of(occulted->begin(),occulted->end(),[](double value){return value==0;}))outside=groundRawWave;
                            else {
                                const auto occWave=waveform(*occulted,false);
                                outside=groundRawWave;
                                for(unsigned j=0;j<count;++j)outside[j]-=occWave[j];
                            }
                        }
                    }
                }
            } else outside=groundRawWave;
            project(outside);
            const auto v=spot(q[0],q[1],p.psfSigmaRad,q[2],q[3],true);
            const auto& u=outside;
            const double backgroundNorm=inner(u,u),uv=inner(u,v),vv=inner(v,v);
            const double determinant=backgroundNorm*vv-uv*uv;
            if(backgroundNorm<=0||vv<=0||determinant<=128*std::numeric_limits<double>::epsilon()*backgroundNorm*vv)return false;
            const double yu=inner(data,u),yv=inner(data,v);
            const double b=(yu*vv-yv*uv)/determinant,d=(yv*backgroundNorm-yu*uv)/determinant;
            reduced.resize(count);
            double cost=0;
            for(unsigned j=0;j<count;++j) {
                const double res=data[j]-b*u[j]-d*v[j];cost+=res*res;
                reduced[j]=res/std::sqrt(variance);
            }
            const double dSky=d+(sourceRelativeGround?q[2]*q[3]/pixelArea*b:0.);
            return std::isfinite(cost)&&std::isfinite(b)&&std::isfinite(dSky);
        };
        const unsigned maxJacobianWorkers=gReticleNestedPoolBlocked
            ?1u:std::min(8u,std::max(1u,std::thread::hardware_concurrency()));
        const auto trust=[&](const Parameters& initial) {
            Parameters current=quantize(squared(initial));
            std::vector<double> residual;
            auto local=solve(geometry(current),&residual);if(!local)return;
            record(geometry(current),*local);
            const double minimumDamping=64*std::numeric_limits<double>::epsilon();
            double damping=1e-3;
            for(std::size_t iteration=0;iteration<trustIterations;++iteration) {
                std::array<std::vector<double>,5> jacobian;
                bool derivatives=true;
                bool useParallel=maxJacobianWorkers>=2;
                std::array<Parameters,5> plusCoords{},minusCoords{};
                bool anyRegional03=false;
                for(unsigned coordinate=0;coordinate<5;++coordinate) {
                    const double difference=coordinate<2?p.pixelPitchRad/1024:
                        coordinate<4?p.pixelPitchRad*p.pixelPitchRad/1024:initialCutStep/1024;
                    plusCoords[coordinate]=current;minusCoords[coordinate]=current;
                    plusCoords[coordinate][coordinate]=std::min(upper[coordinate],current[coordinate]+difference);
                    minusCoords[coordinate][coordinate]=std::max(lower[coordinate],current[coordinate]-difference);
                    if(coordinate<4) {
                        if(!(plusCoords[coordinate][coordinate]>minusCoords[coordinate][coordinate]))useParallel=false;
                        else if(needsRegional(geometry(plusCoords[coordinate]))||needsRegional(geometry(minusCoords[coordinate])))
                            anyRegional03=true;
                    }
                }
                std::fenv_t callerEnv{};
                if(useParallel&&anyRegional03&&std::fegetenv(&callerEnv)==0) {
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
                    const unsigned callerCsr=_mm_getcsr();
#endif
#if defined(__GNUC__) && (defined(__i386__) || defined(__x86_64__))
                    unsigned short callerCw=0;__asm__ __volatile__("fnstcw %0":"=m"(callerCw));
#endif
                    const auto restoreEnv=[&](unsigned worker)->bool {
                        if(worker==0)return true;
                        if(std::fesetenv(&callerEnv)!=0)return false;
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
                        _mm_setcsr(callerCsr);
#endif
#if defined(__GNUC__) && (defined(__i386__) || defined(__x86_64__))
                        __asm__ __volatile__("fldcw %0"::"m"(callerCw));
#endif
                        return true;
                    };
                    const auto* gCur=calibration(current[4]);
                    const auto* gfCur=gCur?groundField(current[4]):nullptr;
                    if(!gCur||!gfCur) {
                        derivatives=false;
                    } else {
                        const auto bkCur=getStaticConicBank(current[4]);
                        std::array<std::vector<double>,8> res03;
                        std::array<bool,8> ok03{};
                        std::array<std::exception_ptr,8> err03{};
                        std::array<conic_moments_v4::FieldWorkspaceV5,8> ws03{};
                        std::atomic<unsigned> nextTask{0};
                        const auto run03=[&](unsigned worker) noexcept {
                            try {
                                if(!restoreEnv(worker))return;
                                for(;;) {
                                    const unsigned idx=nextTask.fetch_add(1,std::memory_order_relaxed);
                                    if(idx>=8u)break;
                                    const unsigned coord=idx>>1u;
                                    const Parameters q=geometry((idx&1u)?minusCoords[coord]:plusCoords[coord]);
                                    ok03[idx]=solvePure(q,gCur->rawWave,gfCur,bkCur.get(),ws03[worker],res03[idx]);
                                }
                            } catch(...) {err03[worker]=std::current_exception();}
                        };
                        threadLocalReticlePool().run(maxJacobianWorkers,run03);
                        for(const auto& e:err03)if(e)std::rethrow_exception(e);
                        for(unsigned coord=0;coord<4;++coord) {
                            if(!ok03[2*coord]||!ok03[2*coord+1]){derivatives=false;break;}
                            jacobian[coord].resize(count);
                            const double denom=plusCoords[coord][coord]-minusCoords[coord][coord];
                            for(unsigned j=0;j<count;++j)jacobian[coord][j]=(res03[2*coord][j]-res03[2*coord+1][j])/denom;
                        }
                    }
                    if(derivatives) {
                        if(!(plusCoords[4][4]>minusCoords[4][4])) {
                            derivatives=false;
                        } else {
                            const Parameters gp4=geometry(plusCoords[4]),gm4=geometry(minusCoords[4]);
                            const auto* gP=calibration(gp4[4]);
                            const auto* gfP=gP?groundField(gp4[4]):nullptr;
                            if(!gP||!gfP) {
                                derivatives=false;
                            } else {
                                const std::vector<double> rawP=gP->rawWave,fieldP=*gfP;
                                const auto bkP=getStaticConicBank(gp4[4]);
                                const auto* gM=calibration(gm4[4]);
                                const auto* gfM=gM?groundField(gm4[4]):nullptr;
                                if(!gM||!gfM) {
                                    derivatives=false;
                                } else {
                                    const std::vector<double> rawM=gM->rawWave,fieldM=*gfM;
                                    const auto bkM=getStaticConicBank(gm4[4]);
                                    std::array<std::vector<double>,2> res4;
                                    std::array<bool,2> ok4{};
                                    std::array<std::exception_ptr,2> err4{};
                                    std::array<conic_moments_v4::FieldWorkspaceV5,2> ws4{};
                                    const auto run4=[&](unsigned worker) noexcept {
                                        try {
                                            if(!restoreEnv(worker))return;
                                            if(worker==0)ok4[0]=solvePure(gp4,rawP,&fieldP,bkP.get(),ws4[0],res4[0]);
                                            else ok4[1]=solvePure(gm4,rawM,&fieldM,bkM.get(),ws4[1],res4[1]);
                                        } catch(...) {err4[worker]=std::current_exception();}
                                    };
                                    threadLocalReticlePool().run(2u,run4);
                                    for(const auto& e:err4)if(e)std::rethrow_exception(e);
                                    if(!ok4[0]||!ok4[1])derivatives=false;
                                    else {
                                        jacobian[4].resize(count);
                                        const double denom=plusCoords[4][4]-minusCoords[4][4];
                                        for(unsigned j=0;j<count;++j)jacobian[4][j]=(res4[0][j]-res4[1][j])/denom;
                                    }
                                }
                            }
                        }
                    }
                } else {
                    for(unsigned coordinate=0;coordinate<5;++coordinate) {
                        std::vector<double> a,b;
                        if(!(plusCoords[coordinate][coordinate]>minusCoords[coordinate][coordinate])||
                           !solve(geometry(plusCoords[coordinate]),&a)||!solve(geometry(minusCoords[coordinate]),&b)) {
                            derivatives=false;break;
                        }
                        jacobian[coordinate].resize(count);
                        for(unsigned j=0;j<count;++j)jacobian[coordinate][j]=(a[j]-b[j])/(plusCoords[coordinate][coordinate]-minusCoords[coordinate][coordinate]);
                    }
                }
                if(!derivatives)break;
                Parameters lowerStep{},upperStep{};
                for(unsigned coordinate=0;coordinate<5;++coordinate) {
                    lowerStep[coordinate]=lower[coordinate]-current[coordinate];upperStep[coordinate]=upper[coordinate]-current[coordinate];
                }
                bool accepted=false;
                for(std::size_t trial=0;trial<trustTrials;++trial) {
                    Parameters delta{};
                    if(!dampedBoxStep(jacobian,residual,lowerStep,upperStep,damping,delta))break;
                    auto proposed=current;
                    for(unsigned coordinate=0;coordinate<5;++coordinate)
                        proposed[coordinate]=std::clamp(current[coordinate]+delta[coordinate],lower[coordinate],upper[coordinate]);
                    proposed=quantize(proposed);if(proposed==current)break;
                    std::vector<double> candidateResidual;
                    const auto fit=solve(geometry(proposed),&candidateResidual);
                    if(fit)record(geometry(proposed),*fit);
                    if(fit&&(*fit)[0]<(*local)[0]) {
                        double predictedCost=0;
                        for(unsigned j=0;j<count;++j) {
                            double value=residual[j];
                            for(unsigned coordinate=0;coordinate<5;++coordinate)value+=jacobian[coordinate][j]*(proposed[coordinate]-current[coordinate]);
                            predictedCost+=value*value;
                        }
                        const double predictedReduction=inner(residual,residual)-predictedCost;
                        const double ratio=predictedReduction>0?((*local)[0]-(*fit)[0])/(variance*predictedReduction):0;
                        if(ratio>.75)damping=std::max(minimumDamping,damping/3);
                        else if(ratio<.25)damping=std::min(1/minimumDamping,damping*2);
                        current=proposed;local=fit;residual=std::move(candidateResidual);accepted=true;break;
                    }
                    damping=std::min(1/minimumDamping,damping*10);
                }
                if(!accepted)break;
            }
        };
        // Fixed instrument-centred compact proposals at measured null minima;
        // no source location, source size, label, geometry or RNG seed enters.
        std::vector<double> starts{nullCutoff};
        for(const auto& minimum:minima) {
            if(starts.size()>=maximumJointStarts)break;
            if(std::find(starts.begin(),starts.end(),minimum.cutoff)==starts.end())starts.push_back(minimum.cutoff);
        }
        for(const double cutoff:starts) {
            const double compact=2*p.pixelPitchRad;
            trust({{0,0,compact,compact,cutoff}});
            if(selected.cost/variance<=residualBound)break;
        }
        // Run every original cold-start decision first. Only a remaining failed
        // residual can request independent measured-history continuations.
        // These propose shapes; physical budgets, null updates, curvature,
        // finite mirrors, domain/profile screens and admission remain below.
        if(priorStart&&selected.cost/variance>residualBound) {
            std::vector<double> priorCutoffs{(*priorStart)[4]};
            if(nullCutoff!=(*priorStart)[4])priorCutoffs.push_back(nullCutoff);
            // Ego roll between captures can swap or square the axis-aligned
            // support across the 2f modulation saddle (w == h). Test the prior
            // extent first, then 90-degree swapped (h, w) and equal-side
            // envelope (max, max) using only the causal prior's finite values.
            std::vector<std::array<double,2>> priorExtents{{(*priorStart)[2],(*priorStart)[3]}};
            if((*priorStart)[2]!=(*priorStart)[3]) {
                if((*priorStart)[3]<=2*halfX&&(*priorStart)[2]<=2*halfY)
                    priorExtents.push_back({(*priorStart)[3],(*priorStart)[2]});
                const double side=std::max((*priorStart)[2],(*priorStart)[3]);
                if(side<=2*halfX&&side<=2*halfY)priorExtents.push_back({side,side});
            }
            for(const double cutoff:priorCutoffs) {
                for(const auto& extent:priorExtents) {
                    auto initial=*priorStart;
                    initial[2]=extent[0];initial[3]=extent[1];initial[4]=cutoff;
                    trust(initial);
                    if(selected.cost/variance<=residualBound)break;
                }
                if(selected.cost/variance<=residualBound)break;
            }
        }
    }
    bestQ=selected.shape;bestCost=selected.cost;beta=selected.background;contrast=selected.source;
    const double step=p.pixelPitchRad/std::ldexp(1.,18);
    const double residual=bestCost/variance;
    double evidence=std::sqrt(std::max(0.,nullCost-bestCost)/variance);
    observed.shape=bestQ;observed.fitAvailable=std::isfinite(bestCost);observed.sourceResidual=residual;observed.nullResidual=nullCost/variance;observed.refusal=8;
    result.fitSnr=evidence;result.fitResidual=residual;
    if(evidence<threshold||degrees<=0||residual>residualBound)return;
    observed.refusal=9;
    const auto projected=basis(bestQ,true);if(!projected)return;
    observed.refusal=12;
    if(!physicalBudget(bestQ,beta,contrast,&observed.rawBudget))return;
    // Shape is fixed during background-mode/curvature checks. Reuse exactly
    // identical source waveforms; this changes no floating-point operation
    // within a waveform and bounds transient storage to128phase vectors.
    cacheShapes=true;
    const auto localVariance=[&](const Parameters& q,double background,double source,bool selectedMode)->std::optional<double> {
        const auto values=basis(q,true);if(!values)return std::nullopt;
        const auto prediction=[&](const Parameters& shape)->std::optional<std::vector<double>> {
            const auto wave=basis(shape,true);if(!wave)return std::nullopt;
            const double dEff=effectiveSource(shape,*wave,background,source);
            auto answer=wave->values[0];
            for(unsigned j=0;j<count;++j)answer[j]=background*answer[j]+dEff*wave->values[1][j];
            return answer;
        };
        std::vector<std::vector<double>> nuisance;
        const auto addColumn=[&](std::vector<double> column) {
            for(const auto& previous:nuisance) {
                const double coefficient=inner(column,previous);
                for(unsigned j=0;j<count;++j)column[j]-=coefficient*previous[j];
            }
            const double length=std::sqrt(inner(column,column));
            if(length<=128*std::numeric_limits<double>::epsilon())return false;
            for(auto& value:column)value/=length;
            nuisance.push_back(std::move(column));return true;
        };
        if(selectedMode)observed.refusal=13;
        if(!addColumn(values->values[0])||!addColumn(values->values[1]))return std::nullopt;
        const double derivativeStep=p.pixelPitchRad/1024,cutDerivative=initialCutStep/1024;
        const auto derivative=[&](unsigned coordinate)->std::optional<std::vector<double>> {
            auto plus=q,minus=q;const double h=coordinate==4?cutDerivative:derivativeStep;
            plus[coordinate]+=h;minus[coordinate]-=h;
            if(coordinate==4){plus[4]=std::min(high,plus[4]);minus[4]=std::max(low,minus[4]);}
            const auto a=prediction(plus),b=prediction(minus);
            if(!a||!b||!(plus[coordinate]>minus[coordinate]))return std::nullopt;
            auto answer=*a;
            for(unsigned j=0;j<count;++j)answer[j]=(answer[j]-(*b)[j])/(plus[coordinate]-minus[coordinate]);
            return answer;
        };
        if(selectedMode)observed.refusal=14;
        for(const unsigned coordinate:{2u,3u,4u}) {
            if(coordinate!=4&&q[coordinate]<=derivativeStep)continue;
            const auto column=derivative(coordinate);
            if(!column||!addColumn(*column))return std::nullopt;
        }
        if(selectedMode)observed.refusal=15;
        auto x=derivative(0),y=derivative(1);if(!x||!y)return std::nullopt;
        for(const auto& column:nuisance) {
            const double a=inner(*x,column),b=inner(*y,column);
            for(unsigned j=0;j<count;++j){(*x)[j]-=a*column[j];(*y)[j]-=b*column[j];}
        }
        const double xx=inner(*x,*x),yy=inner(*y,*y),xy=inner(*x,*y);
        const double determinant=xx*yy-xy*xy,trace=xx+yy;
        if(selectedMode) {
            observed.informationAvailable=true;observed.informationDeterminant=determinant;
            observed.informationTrace=trace;observed.refusal=16;
        }
        if(!std::isfinite(determinant)||xx<=0||yy<=0||
           determinant<=128*std::numeric_limits<double>::epsilon()*trace*trace)return std::nullopt;
        const double tx=std::tan(q[0]),ty=std::tan(q[1]),n2=1+tx*tx+ty*ty;
        const double gx=(1+tx*tx)*(1+tx*tx)*(1+ty*ty)/(n2*n2);
        const double gy=(1+ty*ty)*(1+ty*ty)*(1+tx*tx)/(n2*n2);
        const double gxy=-tx*ty*(1+tx*tx)*(1+ty*ty)/(n2*n2);
        const double metric=.5*(gx+gy+std::hypot(gx-gy,2*gxy));
        const double value=metric*variance*(trace+std::hypot(xx-yy,2*xy))/(2*determinant)+step*step/12;
        if(selectedMode){observed.localVariance=value;observed.refusal=17;}
        if(!std::isfinite(value)||value<=0)return std::nullopt;
        return value;
    };
    const auto ray=[&](const Parameters& q){return unit(add(forward,add(scale(right,std::tan(q[0])),scale(vertical,std::tan(q[1])))));};
    const auto direction=ray(bestQ);
    const auto selectedVariance=localVariance(bestQ,beta,contrast,true);
    std::vector<std::array<double,5>> modes;
    bool invalidMode=!selectedVariance||std::any_of(observed.rawBudget.begin(),observed.rawBudget.end(),[](double value){return value<=0;}),profileUnresolved=false;
    if(selectedVariance)modes.push_back({{bestQ[0],bestQ[1],bestQ[2],bestQ[3],*selectedVariance}});
    // Hold cutoff values, never iterators into the bounded calibration cache:
    // evaluating a mode's own derivatives can add calibration entries.
    std::vector<double> cutoffs;cutoffs.reserve(calibrations.size());
    for(const auto& entry:calibrations)cutoffs.push_back(entry.cutoff);
    observed.refusal=18;
    for(const double cutoff:cutoffs)for(const double sx:{-1.,1.})for(const double sy:{-1.,1.}) {
        if(sx==1&&sy==1&&cutoff==bestQ[4])continue;
        auto q=bestQ;q[0]*=sx;q[1]*=sy;q[4]=cutoff;
        const auto alternative=solve(q);
        if(!alternative||(*alternative)[0]>bestCost+threshold*threshold*variance)continue;
        std::array<double,3> budget{};
        if(!physicalBudget(q,(*alternative)[1],(*alternative)[2],&budget))continue;
        if(std::any_of(budget.begin(),budget.end(),[](double value){return value<=0;}))invalidMode=true;
        const auto other=ray(q);
        const double separation=std::atan2(norm(cross(direction,other)),dot(direction,other));
        if(!observed.alternativeAvailable) {
            observed.alternativeAvailable=true;observed.alternativeShape=q;
            observed.alternativeDelta=((*alternative)[0]-bestCost)/variance;observed.alternativeSeparation=separation;
        }
        const auto modeVariance=localVariance(q,(*alternative)[1],(*alternative)[2],false);
        if(!modeVariance){invalidMode=true;continue;}
        modes.push_back({{q[0],q[1],q[2],q[3],*modeVariance}});
    }
    // A finite mirror cluster is different from a continuation that moves an
    // unseen support edge. Such endpoint profiles retain only coarse authority.
    for(unsigned axis=0;axis<2;++axis) {
        const double full=(axis?p.height:p.width)*p.pixelPitchRad;
        for(const double sign:{-1.,1.}) {
            const double first=std::max(-.5*full-bestQ[axis],sign>0?-.5*bestQ[axis+2]:.5*(bestQ[axis+2]-full));
            const double last=std::min(.5*full-bestQ[axis],sign>0?.5*(full-bestQ[axis+2]):.5*bestQ[axis+2]);
            if(first>last)continue;
            for(const double displacement:{first,last}) {
                if(std::abs(displacement)<=step)continue;
                auto q=bestQ;q[axis]+=displacement;q[axis+2]+=2*sign*displacement;
                for(unsigned coordinate=0;coordinate<4;++coordinate)q[coordinate]=std::round(q[coordinate]/step)*step;
                q[0]=std::clamp(q[0],-halfX,halfX);q[1]=std::clamp(q[1],-halfY,halfY);
                q[2]=std::clamp(q[2],0.,2*halfX);q[3]=std::clamp(q[3],0.,2*halfY);
                const auto alternative=solve(q);
                if(alternative&&(*alternative)[0]<=bestCost+threshold*threshold*variance&&
                   physicalBudget(q,(*alternative)[1],(*alternative)[2]))profileUnresolved=true;
            }
        }
    }
    // Every additional calibration still competes as a physical null. Source
    // evidence cannot receive exclusive credit for a better background fit.
    observed.nullCutoff=nullCutoff;observed.nullResidual=nullCost/variance;
    if(nullCost/variance<threshold*threshold) {
        if(backgroundExplained)*backgroundExplained=true;
        observed.refusal=5;return;
    }
    evidence=std::sqrt(std::max(0.,nullCost-bestCost)/variance);result.fitSnr=evidence;
    observed.refusal=19;
    if(evidence<threshold)return;
    const auto envelope=!invalidMode&&!profileUnresolved?finiteMirrorEnvelopeV1(c,modes):std::nullopt;
    const bool compactAxis=bestQ[2]<halfX||bestQ[3]<halfY;
    const bool nearStop=std::hypot(bestQ[0],bestQ[1])>=.75*c.stopRadiusRad;
    if(!envelope&&!compactAxis&&!nearStop)return;
    ThermalImageFeatureV1 feature;feature.direction=direction;
    feature.extent.widthRad=bestQ[2];feature.extent.heightRad=bestQ[3];
    const double fieldRadius=std::atan(std::hypot(std::tan(halfX),std::tan(halfY)));
    feature.signalToNoise=evidence;feature.angularVarianceRad2=envelope?*envelope:4*fieldRadius*fieldRadius;
    feature.signedContrastElectrons=contrast;
    result.features.push_back(feature);result.bearingResolved=true;result.preciseBearing=envelope.has_value();observed.refusal=20;
}

}
