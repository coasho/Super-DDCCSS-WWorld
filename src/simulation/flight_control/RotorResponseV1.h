#pragma once

// Private estimated small-signal response of the complete built-in cascade.
// This is not a measured bandwidth or a guarantee outside the linear region.
#include "fsim/RotorControllers.h"
#include "fsim/VehicleProfile.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>

namespace fsim::control {
namespace rotor_response_v1 {
using Polynomial = std::array<double, 8>; // ascending powers of s
inline double response(const std::array<double, 10>& k) noexcept {
    const auto [kv, ki, attitude, damping, rate, integral, lag, step, plantDamping, powerRatio] = k;
    for (double x : k) if (!std::isfinite(x) || x < 0) return 0;
    if (!(kv > 0 && ki > 0 && attitude > 0 && rate > 0 && integral > 0 && step > 0)) return 0;
    const double proportional = std::max(rate - damping, .25 * rate);
    const double n0 = attitude * powerRatio * integral, n1 = attitude * powerRatio * (damping + proportional);
    // H_rate=gamma*((d_control+Kr)s+Ir)/(s(s+d_plant)L+gamma*(Kr*s+Ir))
    // L=(1+lag*s)(1+step*s); step is a conservative control-period delay approximation.
    // H_att=A*H_rate/(s+A*H_rate), H_v=(Kv*s+Ki)*H_att/(s^2+(Kv*s+Ki)*H_att).
    Polynomial numerator{ki*n0, kv*n0+ki*n1, kv*n1};
    Polynomial denominator=numerator;
    denominator[2] += n0;
    denominator[3] += n1;
    denominator[3] += powerRatio*integral;
    denominator[4] += plantDamping+powerRatio*proportional;
    denominator[5] += 1+plantDamping*(lag+step);
    denominator[6] += lag+step+plantDamping*lag*step;
    denominator[7] += lag*step;
    std::size_t degree=7;
    while (degree>0 && denominator[degree]==0) --degree;
    // Routh-Hurwitz: reject unstable, marginal and unresolved zero-pivot models.
    std::array<std::array<double, 5>, 8> routh{};
    for(std::size_t j=0;j<=degree;++j) routh[j%2][j/2]=denominator[degree-j];
    if (!(routh[0][0]>0 && routh[1][0]>0)) return 0;
    for(std::size_t i=2;i<=degree;++i) {
        for(std::size_t j=0;j<4;++j)
            routh[i][j]=(routh[i-1][0]*routh[i-2][j+1]-routh[i-2][0]*routh[i-1][j+1])/routh[i-1][0];
        if (!(std::isfinite(routh[i][0]) && routh[i][0]>0)) return 0;
    }
    auto transfer=[&](double frequency) {
        const std::complex<double> s(0,frequency);
        std::complex<double> n=numerator[7],d=denominator[7];
        for(std::size_t i=7;i-->0;){n=n*s+numerator[i];d=d*s+denominator[i];}
        return n/d;
    };
    auto crossing=[&](bool phase) {
        auto below=[&](double f) {
            const auto h=transfer(f);
            return phase ? std::arg(h)<=-0.78539816339744830962 : std::abs(h)<=std::sqrt(.5);
        };
        double previous=std::ldexp(kv,-20);
        if (below(previous)) return 0.;
        for(int j=1;j<=320;++j) {
            const double next=kv*std::exp2(-20.+j/8.);
            if(below(next)) {
                double lo=previous,hi=next;
                for(int iteration=0;iteration<60;++iteration) {
                    const double mid=std::sqrt(lo*hi);
                    if(below(mid))hi=mid;else lo=mid;
                }
                return hi;
            }
            previous=next;
        }
        return 0.;
    };
    // Existing one-third-bandwidth policy, additionally bounded by 45-degree lag.
    return std::min(crossing(false),3*crossing(true));
}
inline double cached(const std::array<double,10>& key) noexcept {
    struct Entry {std::array<double,10> key{}; double value=0; bool valid=false;};
    static thread_local std::array<Entry,32> entries{};
    static thread_local std::size_t next=0;
    for(const auto& entry:entries) if(entry.valid && entry.key==key)return entry.value;
    const double value=response(key);
    entries[next]={key,value,true};next=(next+1)%entries.size();
    return value;
}
} // namespace rotor_response_v1
inline double rotorCourseResponseV1(const RotorVelocity& velocity,const RotorAttitude& attitude,
    const RotorAllocation& allocation,const RotorAxis& roll,const RotorAxis& pitch,double step) noexcept {
    const auto axis=[&](const RotorRateAxis& rate,double gain,const RotorAxis& plant) {
        if (!(std::abs(rate.power)>1e-9 && plant.power/rate.power>0)) return 0.;
        return rotor_response_v1::cached({velocity.horizontalKp,velocity.horizontalKi,gain,
            rate.damping,rate.bandwidth,rate.integral,plant.lagS,step,plant.damping,plant.power/rate.power});
    };
    return std::min(axis(allocation.roll,attitude.rollGain,roll),axis(allocation.pitch,attitude.pitchGain,pitch));
}
} // namespace fsim::control
