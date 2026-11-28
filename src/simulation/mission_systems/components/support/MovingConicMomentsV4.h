#pragma once

#include <array>
#include <cfenv>
#include <cstddef>
#include <optional>
#include <vector>
#include <memory>

// Private V4 numerical operator. Forward/static V1 paths do not use it.
namespace fsim::ms::components::conic_moments_v4 {
struct Request {
    unsigned width=0,height=0;
    double pixelPitchRad=0,psfSigmaRad=0,cutoff=0;
    std::array<double,3> pointing{},up{};
    std::array<double,4> region{};
};
constexpr double fieldBudget=1e-9;
enum class Refusal {None,Input,ArithmeticEnvironment,Cone,Root,Topology,Nonfinite,Moment,ErrorBudget};
struct Work {
    Refusal refusal=Refusal::None;
    std::size_t preparedColumns=0,degree3Columns=0,degree7Columns=0,analyticColumns=0;
    std::size_t normalPolynomials=0,normalTails=0;
};
struct Result {std::vector<double> field,absoluteError;};
struct FieldWorkspaceV5 {
    std::vector<double> field;
    unsigned width=0,height=0;
    std::array<unsigned,128> activeRows{};
    unsigned activeRowCount=0;
    unsigned colBegin=0,colEnd=0;
    double maxError=0;
    bool dirtyAll=false;
};
struct MidPairV5 {double value=0,error=0;};
/// Cutoff-independent vertical/horizontal Gaussian rectangle factors for
/// 0-ULP bit-identical reuse across multiple horizon cutoffs on the same optics.
struct RegionMomentsV5 {
    std::array<double,4> region{};
    double maxOmittedError=0;
    unsigned width=0,height=0;
    unsigned activeRowCount=0,activeRow0=0;
    unsigned colBegin=0,colEnd=0;
    bool rowsContiguous=false,empty=false,valid=false;
    std::vector<unsigned> activeRows;
    std::vector<MidPairV5> lower,heightArr;
    std::vector<double> lowerV,lowerE;
    std::vector<MidPairV5> mass0;
    std::vector<std::array<MidPairV5,4>> mass3;
    std::vector<unsigned char> mass0Ok,mass3Ok;
};
struct Bounds {double low=0,high=0;};
struct GaussianMemoWorkV5 {std::size_t hits=0,misses=0,entries=0,chargedBytes=0;};
class GaussianMemoV5 {
public:
    explicit GaussianMemoV5(const Request&,std::size_t capacityBytes=64u*1024u*1024u);
    ~GaussianMemoV5();
    GaussianMemoV5(const GaussianMemoV5&)=delete;
    GaussianMemoV5& operator=(const GaussianMemoV5&)=delete;
    GaussianMemoWorkV5 work() const;
private:
    friend class Bank;
    struct Impl;
    std::unique_ptr<Impl> implementation_;
};

bool arithmeticAvailable() noexcept;

/// Scoped x87 64-bit extended-precision guard for foreign host threads (e.g. MSVC
/// CPython, which initializes x87 PC to 53-bit 0x027f instead of MinGW's 64-bit
/// 0x037f). Only upgrades precision control bits when rounding is already
/// FE_TONEAREST and all x87 exceptions are masked; directed rounding or unmasked
/// exception environments remain untouched and continue to refuse acceleration.
class ScopedX87ExtendedPrecision {
public:
    ScopedX87ExtendedPrecision() noexcept {
#if defined(__GNUC__) && (defined(__i386__) || defined(__x86_64__))
        if (std::fegetround() != FE_TONEAREST) return;
        unsigned short cw = 0;
        __asm__ __volatile__("fnstcw %0" : "=m"(cw));
        if ((cw & 0x0c3fu) == 0x003fu && (cw & 0x0300u) != 0x0300u) {
            saved_ = cw;
            const unsigned short extended = static_cast<unsigned short>((cw & ~0x0300u) | 0x0300u);
            __asm__ __volatile__("fldcw %0" : : "m"(extended));
            active_ = true;
        }
#endif
    }
    ~ScopedX87ExtendedPrecision() noexcept {
#if defined(__GNUC__) && (defined(__i386__) || defined(__x86_64__))
        if (active_) {
            __asm__ __volatile__("fldcw %0" : : "m"(saved_));
        }
#endif
    }
    ScopedX87ExtendedPrecision(const ScopedX87ExtendedPrecision&) = delete;
    ScopedX87ExtendedPrecision& operator=(const ScopedX87ExtendedPrecision&) = delete;
private:
    unsigned short saved_ = 0;
    bool active_ = false;
};
// Private numeric calibration seam; no SDK/public V1 layout changes.
std::array<long double,2> outwardForCalibration(long double);
std::optional<Bounds> normalPdfForCalibration(double);
std::optional<Bounds> normalCdfForCalibration(double);
std::optional<std::array<Bounds,8>> momentsForCalibration(
    double low,double high,double detectorLow,double detectorHigh,double sigma,double anchor);

/// Explicit fast arithmetic calibration; retained momentsForCalibration stays exact.
std::optional<std::array<Bounds,8>> fastMomentsForCalibration(
    double low,double high,double detectorLow,double detectorHigh,double sigma,double anchor);

// Bank coefficients depend only on the optical policy/pose/cutoff, never on
// photons, source identity or request history. The ordinary bank is serial.
class Bank {
public:
    static std::unique_ptr<Bank> prepare(const Request&,Work* = nullptr);
    /// Own one bank per cutoff and allow concurrent V5 field evaluations.
    static std::unique_ptr<Bank> prepareSharedV5(const Request&,Work* = nullptr);
    ~Bank();
    Bank(Bank&&) noexcept;
    Bank& operator=(Bank&&) noexcept;
    Bank(const Bank&)=delete;
    Bank& operator=(const Bank&)=delete;
    std::optional<Result> evaluate(const std::array<double,4>& region,Work* = nullptr) const;
    /// Version5 materialized field, same per-component budget and refusal fallback.
    std::optional<Result> evaluateFastMoments(const std::array<double,4>& region,Work* = nullptr) const;
    /// Exact bounded Gaussian-factor reuse; no terrain/final-field caching.
    std::optional<Result> evaluateFastMemoV5(const std::array<double,4>& region,GaussianMemoV5&,Work* = nullptr) const;
    /// Zero-allocation V5 field evaluation into a caller-owned workspace.
    bool evaluateFastIntoV5(const std::array<double,4>& region,FieldWorkspaceV5& workspace,Work* = nullptr) const;
    /// Precompute cutoff-independent Gaussian row and column moments for a region.
    bool prepareRegionV5(const std::array<double,4>& region,RegionMomentsV5& out) const;
    /// Zero-allocation V5 field evaluation reusing precomputed cutoff-independent moments.
    bool evaluateFastPreparedIntoV5(const RegionMomentsV5& pre,FieldWorkspaceV5& workspace) const;
    /// Thread-safe degree-3 column pre-warming for shared V5 banks.
    bool ensureColumnV5(unsigned col) const;
private:
    struct Impl;
    std::unique_ptr<Impl> implementation_;
    explicit Bank(std::unique_ptr<Impl>);
};
} // namespace fsim::ms::components::conic_moments_v4
