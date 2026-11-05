#pragma once

#include <array>
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
struct Bounds {double low=0,high=0;};

bool arithmeticAvailable() noexcept;
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
// photons, source identity or request history. One caller owns one bank.
class Bank {
public:
    static std::unique_ptr<Bank> prepare(const Request&,Work* = nullptr);
    ~Bank();
    Bank(Bank&&) noexcept;
    Bank& operator=(Bank&&) noexcept;
    Bank(const Bank&)=delete;
    Bank& operator=(const Bank&)=delete;
    std::optional<Result> evaluate(const std::array<double,4>& region,Work* = nullptr) const;
    /// Version5 materialized field, same per-component budget and refusal fallback.
    std::optional<Result> evaluateFastMoments(const std::array<double,4>& region,Work* = nullptr) const;
private:
    struct Impl;
    std::unique_ptr<Impl> implementation_;
    explicit Bank(std::unique_ptr<Impl>);
};
} // namespace fsim::ms::components::conic_moments_v4
