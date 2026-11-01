#pragma once

// Private mathematical implementation; no public controller layout changes.
#include <algorithm>
#include <array>
#include <cmath>

namespace fsim::control {
struct RotorTiltAllocationV1 {double pitch=0,roll=0;bool saturated=false;};
inline RotorTiltAllocationV1 allocateRotorThrustV1(double ax,double ay,double support,double maximum,
    double pitchTrim,double rollTrim) noexcept {
    if(support<=0)return {0,0,ax!=0 || ay!=0};
    double pitch=pitchTrim-std::atan2(ax,support);
    double roll=rollTrim+std::atan2(ay*std::cos(pitch),support);
    const double up=std::cos(pitch)*std::cos(roll);
    const bool saturated=up<std::cos(maximum);
    if(saturated) {
        const double x=-std::sin(pitch)*std::cos(roll),y=std::sin(roll),length=std::hypot(x,y);
        if(length>0) {
            const double fx=std::sin(maximum)*x/length,fy=std::sin(maximum)*y/length,fu=std::cos(maximum);
            pitch=std::atan2(-fx,fu);roll=std::atan2(fy,std::hypot(fx,fu));
        } else {pitch=roll=0.;}
    }
    return {pitch,roll,saturated};
}
inline std::array<double,2> rotorIntegralIncrementV1(double ix,double iy,double rx,double ry,bool saturated) noexcept {
    // An unsaturated atan2/tan round trip can have an ULP residual. It is
    // never evidence of saturation and must not remove integral authority.
    if(!saturated)return {ix,iy};
    const double norm=rx*rx+ry*ry,dot=ix*rx+iy*ry;
    if(!std::isfinite(norm)||!std::isfinite(dot))return {0,0};
    if(norm>0 && dot>0){ix-=rx*dot/norm;iy-=ry*dot/norm;}
    return {ix,iy};
}
}
