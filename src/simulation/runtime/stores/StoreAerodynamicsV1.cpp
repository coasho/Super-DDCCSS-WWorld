#include "simulation/runtime/stores/StoreAerodynamicsV1.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace fsim::session {
namespace {
constexpr double kDegrees = 57.2957795130823208768;
constexpr const char* kNames[] = {"CD", "CY", "CL", "Cl", "Cm", "Cn"};
std::vector<double> numbers(const core::Json& value) {
    std::vector<double> result;
    for (const auto& v : value.asArray()) {
        const double x = v.asNumber();
        if (!std::isfinite(x)) throw std::runtime_error("Nonfinite VLM table value");
        result.push_back(x);
    }
    return result;
}
std::vector<double> axis(const core::Json& value) {
    auto result = numbers(value);
    if (result.size() < 2 || std::adjacent_find(result.begin(), result.end(),
        [](double a, double b) { return a >= b; }) != result.end())
        throw std::runtime_error("VLM table axis must be strictly increasing");
    return result;
}
struct Bracket { std::size_t low; double fraction; };
Bracket bracket(const std::vector<double>& a, double value) {
    value = std::clamp(value, a.front(), a.back());
    const auto upper = std::upper_bound(a.begin(), a.end(), value);
    const auto low = std::min(a.size() - 2, upper == a.begin() ? std::size_t(0) : std::size_t(upper - a.begin() - 1));
    return {low, (value - a[low]) / (a[low + 1] - a[low])};
}
double one(const std::vector<double>& grid, const std::vector<double>& values, double x) {
    if (values.empty()) return 0.;
    const auto b = bracket(grid, x);
    return values[b.low] + b.fraction * (values[b.low + 1] - values[b.low]);
}
double two(const std::vector<double>& values, std::size_t columns, Bracket a, Bracket b) {
    const auto at = [&](std::size_t i, std::size_t j) { return values[i * columns + j]; };
    return (1-a.fraction)*((1-b.fraction)*at(a.low,b.low)+b.fraction*at(a.low,b.low+1)) +
        a.fraction*((1-b.fraction)*at(a.low+1,b.low)+b.fraction*at(a.low+1,b.low+1));
}
} // namespace

std::shared_ptr<const StoreAerodynamicsV1> StoreAerodynamicsV1::load(const std::filesystem::path& path) {
    static std::mutex mutex;
    static std::unordered_map<std::string, std::weak_ptr<const StoreAerodynamicsV1>> cache;
    std::lock_guard<std::mutex> guard(mutex);
    const auto key = path.lexically_normal().string();
    if (const auto found = cache.find(key); found != cache.end())
        if (auto retained = found->second.lock()) return retained;
    if (std::filesystem::file_size(path) > 32*1024*1024) throw std::runtime_error("Oversized VLM table");
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("Missing VLM table: " + key);
    std::ostringstream contents; contents << stream.rdbuf();
    const auto document = core::Json::parse(contents.str(), key);
    if (document.string("schema", "") != "fsim.airframe-aerodynamics.v1" || document.string("method", "") != "vlm")
        throw std::runtime_error("Shared aircraft VLM table required");
    auto result = std::make_shared<StoreAerodynamicsV1>();
    const auto& reference = document.child("reference");
    result->area = reference.number("area_m2", 0.);
    result->span = reference.number("span_m", 0.);
    result->chord = reference.number("chord_m", 0.);
    if (!(result->area > 0. && result->span > 0. && result->chord > 0.) ||
        !std::isfinite(result->area + result->span + result->chord)) throw std::runtime_error("Invalid VLM reference");
    const auto origin = numbers(reference.child("aero_point_body_m"));
    if (origin.size() != 3) throw std::runtime_error("VLM reference origin requires three coordinates");
    std::copy(origin.begin(), origin.end(), result->aeroPoint.begin());
    const auto& tables = document.child("tables");
    result->alpha_ = axis(tables.child("alpha")); result->beta_ = axis(tables.child("beta"));
    const auto& mach = tables.child("mach"); result->mach_ = axis(mach.child("mach"));
    for (unsigned k = 0; k < 6; ++k) {
        for (const auto& row : tables.child("base").child(kNames[k]).asArray()) {
            auto values = numbers(row);
            if (values.size() != result->beta_.size()) throw std::runtime_error("VLM table row length mismatch");
            result->base_[k].insert(result->base_[k].end(), values.begin(), values.end());
        }
        if (result->base_[k].size() != result->alpha_.size()*result->beta_.size()) throw std::runtime_error("VLM base shape mismatch");
        for (unsigned rate = 0; rate < 3; ++rate) {
            const auto& values = tables.child("rates").child(rate==0 ? "p" : rate==1 ? "q" : "r").child(kNames[k]);
            if (!values.isNull()) {
                result->rates_[rate][k] = numbers(values);
                if (result->rates_[rate][k].size() != result->alpha_.size()) throw std::runtime_error("VLM rate shape mismatch");
            }
        }
    }
    auto take = [&](const char* name) {
        auto values = numbers(mach.child(name));
        if (values.size() != result->mach_.size()) throw std::runtime_error("VLM Mach shape mismatch");
        return values;
    };
    for(const auto& row:tables.child("base").child("CDi").asArray()) {
        const auto values=numbers(row);
        if(values.size()!=result->beta_.size())throw std::runtime_error("VLM induced-drag row length mismatch");
        result->inducedBase_.insert(result->inducedBase_.end(),values.begin(),values.end());
    }
    if(result->inducedBase_.size()!=result->alpha_.size()*result->beta_.size())
        throw std::runtime_error("VLM induced-drag shape mismatch");
    for(unsigned rate=0;rate<3;++rate) {
        result->inducedRates_[rate]=numbers(tables.child("rates").child(rate==0?"p":rate==1?"q":"r").child("CDi"));
        if(result->inducedRates_[rate].size()!=result->alpha_.size())throw std::runtime_error("VLM induced-rate shape mismatch");
    }
    result->lift_=take("K_L"); result->side_=take("K_Y"); result->pitch_=take("dCm_dCL");
    result->drag_=take("dCD0"); result->induced_=take("induced_factor");
    const auto& ground=tables.child("ground_effect");
    result->height_=axis(ground.child("h_b"));
    result->groundLift_=numbers(ground.child("lift"));
    result->groundDrag_=numbers(ground.child("drag"));
    if(result->groundLift_.size()!=result->height_.size() || result->groundDrag_.size()!=result->height_.size())
        throw std::runtime_error("VLM ground-effect shape mismatch");
    cache[key]=result;
    return result;
}

std::array<double,6> StoreAerodynamicsV1::coefficients(double mach, double alpha, double beta,
    double speed, const double rates[3], double heightOverSpan) const {
    const double a=alpha*kDegrees,b=beta*kDegrees;
    const auto ab=bracket(alpha_,a),bb=bracket(beta_,b);
    std::array<double,6> c{};
    double circulation=two(inducedBase_,beta_.size(),ab,bb);
    for (unsigned k=0;k<6;++k)c[k]=two(base_[k],beta_.size(),ab,bb);
    const double lowLift=c[2],kl=one(mach_,lift_,mach),ky=one(mach_,side_,mach);
    c[2]*=kl*one(height_,groundLift_,heightOverSpan); for (unsigned k : {1u,3u,5u}) c[k]*=ky;
    c[4]+=lowLift*one(mach_,pitch_,mach);
    const double rateScale[]={span/(2*std::max(speed,1.e-9)),chord/(2*std::max(speed,1.e-9)),span/(2*std::max(speed,1.e-9))};
    for (unsigned rate=0;rate<3;++rate) {
        for (unsigned k=0;k<6;++k)c[k]+=one(alpha_,rates_[rate][k],a)*rates[rate]*rateScale[rate]*kl;
        circulation+=one(alpha_,inducedRates_[rate],a)*rates[rate]*rateScale[rate]*kl;
    }
    c[0]+=one(mach_,drag_,mach)+circulation*(one(mach_,induced_,mach)*one(height_,groundDrag_,heightOverSpan)-1.);
    return c;
}
} // namespace fsim::session
