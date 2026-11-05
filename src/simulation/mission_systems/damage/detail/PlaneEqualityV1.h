#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace fsim::ms::damage::detail {

// Private geometric arithmetic only. The caller supplies the ORIGINAL rounded
// double coefficients, before any extended-precision reduction.
struct PlaneEqualityInputV1 {
    std::array<double,3> normal{};
    double bound=0,offset=0,rate=0,slope=0;
};
struct PlaneEqualityVertexV1 { double time=0,along=0; };
struct PlaneEqualityResultV1 {
    enum class Status { Fallback,Empty,Reduced } status=Status::Fallback;
    std::array<PlaneEqualityVertexV1,2> vertices{};
    std::size_t count=0;
};

inline bool oppositePlaneV1(const std::array<double,3>& a,double boundA,
                            const std::array<double,3>& b,double boundB) noexcept {
    return a[0]==-b[0]&&a[1]==-b[1]&&a[2]==-b[2]&&boundA==-boundB;
}

// Only the triangle pipeline opts in: five triangle planes or its six-plane
// broad box. Full-dimensional and arithmetically unsafe cases retain the old
// polygon path. No geometric epsilon, band or angular cutoff is introduced.
inline PlaneEqualityResultV1 reducePlaneEqualityV1(const PlaneEqualityInputV1* planes,
    std::size_t count,double k,double duration) noexcept {
    using Result=PlaneEqualityResultV1;
    const Result fallback;
    const auto empty=[] { Result out;out.status=Result::Status::Empty;return out; };
    if(!planes||count<2||count>6||!std::isfinite(k)||!std::isfinite(duration)||!(duration>0))return fallback;
    for(std::size_t i=0;i<count;++i) {
        const auto& p=planes[i];
        if(!std::isfinite(p.bound)||!std::isfinite(p.offset)||!std::isfinite(p.rate)||!std::isfinite(p.slope))return fallback;
        for(const auto n:p.normal)if(!std::isfinite(n))return fallback;
    }
    std::size_t left=count,right=count;
    for(std::size_t i=0;i<count&&left==count;++i)for(std::size_t j=i+1;j<count;++j) {
        const auto& a=planes[i];const auto& b=planes[j];
        if(!oppositePlaneV1(a.normal,a.bound,b.normal,b.bound))continue;
        // Do not merge even nominally opposite geometry if its ACTUAL rounded
        // inequalities have unexpectedly lost exact sign symmetry.
        if(b.offset!=-a.offset||b.rate!=-a.rate||b.slope!=-a.slope)return fallback;
        left=i;right=j;break;
    }
    if(left==count)return fallback;
    const auto& equation=planes[left];
    if(equation.rate==0&&equation.slope==0)return equation.offset==0?fallback:empty();

    // Guard arithmetic as well as its inputs. In particular, a platform
    // where long double has only double range must not silently lose a
    // nonzero term. These guards select the ORIGINAL path, not an epsilon.
    const auto multiply=[](long double a,long double b,long double& value) {
        value=a*b;
        return std::isfinite(value)&&!(value==0&&a!=0&&b!=0);
    };
    const auto divide=[](long double numerator,long double denominator,long double& value) {
        if(denominator==0)return false;
        value=numerator/denominator;
        return std::isfinite(value)&&!(value==0&&numerator!=0);
    };
    const long double h=duration;
    std::array<long double,2> coefficient={0,equation.slope};
    if(!multiply(equation.rate,h,coefficient[0])||!std::isfinite(coefficient[1]))return fallback;
    const std::size_t eliminated=std::abs(coefficient[0])>std::abs(coefficient[1])?0u:1u;
    const std::size_t freeIndex=1u-eliminated;
    const long double pivot=coefficient[eliminated];
    if(pivot==0)return fallback;
    long double intercept=0,gradient=0;
    if(!divide(-static_cast<long double>(equation.offset),pivot,intercept)||
       !divide(-coefficient[freeIndex],pivot,gradient))return fallback;
    long double lower=-std::numeric_limits<long double>::infinity();
    long double upper=std::numeric_limits<long double>::infinity();
    enum class Clip { Valid,Empty,Unsafe };
    const auto clip=[&](long double constant,long double x,long double q) {
        const std::array<long double,2> term={x,q};
        long double offsetTerm=0,rateTerm=0;
        if(!multiply(term[eliminated],intercept,offsetTerm)||
           !multiply(term[eliminated],gradient,rateTerm))return Clip::Unsafe;
        const long double offset=constant+offsetTerm;
        const long double rate=term[freeIndex]+rateTerm;
        if(!std::isfinite(offset)||!std::isfinite(rate))return Clip::Unsafe;
        if(rate==0)return offset>0?Clip::Empty:Clip::Valid;
        long double boundary=0;
        if(!divide(-offset,rate,boundary))return Clip::Unsafe;
        if(rate>0)upper=std::min(upper,boundary);
        else lower=std::max(lower,boundary);
        return lower>upper?Clip::Empty:Clip::Valid;
    };
    long double kh=0;
    if(!multiply(k,h,kh))return fallback;
    const std::array<std::array<long double,3>,4> domain={{{0,-1,0},{-1,1,0},{0,0,-1},{-1,-kh,1}}};
    for(const auto& p:domain) {
        const auto result=clip(p[0],p[1],p[2]);
        if(result==Clip::Unsafe)return fallback;
        if(result==Clip::Empty)return empty();
    }
    for(std::size_t i=0;i<count;++i)if(i!=left&&i!=right) {
        const auto& p=planes[i];
        long double rate=0;
        if(!multiply(p.rate,h,rate))return fallback;
        const auto result=clip(p.offset,rate,p.slope);
        if(result==Clip::Unsafe)return fallback;
        if(result==Clip::Empty)return empty();
    }
    if(!std::isfinite(lower)||!std::isfinite(upper)||lower>upper)return fallback;
    Result out;out.status=Result::Status::Reduced;out.count=lower==upper?1u:2u;
    for(std::size_t i=0;i<out.count;++i) {
        const long double value=i==0?lower:upper;
        long double displacement=0;
        if(!multiply(gradient,value,displacement))return fallback;
        std::array<long double,2> point{};point[freeIndex]=value;point[eliminated]=intercept+displacement;
        long double time=0;
        if(!multiply(point[0],h,time)||!std::isfinite(point[1]))return fallback;
        const long double maximum=std::numeric_limits<double>::max();
        if(std::abs(time)>maximum||std::abs(point[1])>maximum)return fallback;
        out.vertices[i]={static_cast<double>(time),static_cast<double>(point[1])};
        const auto& v=out.vertices[i];
        if(!std::isfinite(v.time)||!std::isfinite(v.along)||v.time<0||v.time>duration||
           (time!=0&&v.time==0)||(point[1]!=0&&v.along==0))return fallback;
    }
    // Return BOTH interval ends. The caller retains its original common
    // lex(time,along), normal, closed-skin and exit postselection predicates.
    return out;
}

} // namespace fsim::ms::damage::detail
