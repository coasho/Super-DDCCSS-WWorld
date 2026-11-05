#pragma once

#include "simulation/mission_systems/damage/MaterialContactHistoryV3.h"
#include "simulation/mission_systems/damage/detail/PlaneEqualityV1.h"
#include <algorithm>
#include <cstdint>
#include <cstring>

namespace fsim::ms::damage::detail {
struct MaterialBoundaryVertexV3 {double time=0,along=0;std::uint32_t mask=0;};
struct MaterialBoundaryResultV3 {
    enum class Status {Fallback,Empty,Reduced} status=Status::Fallback;
    std::array<MaterialBoundaryVertexV3,2> vertices{};std::size_t count=0;
};
inline MaterialBoundaryResultV3 reduceMaterialBoundaryV3(const PlaneEqualityInputV1* planes,
    std::size_t count,double k,double duration) noexcept {
    using Result=MaterialBoundaryResultV3;
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
    std::uint32_t lowerMask=0,upperMask=0,coincidentMask=0;
    enum class Clip { Valid,Empty,Unsafe };
    const auto clip=[&](long double constant,long double x,long double q,std::uint32_t mask) {
        const std::array<long double,2> term={x,q};
        long double offsetTerm=0,rateTerm=0;
        if(!multiply(term[eliminated],intercept,offsetTerm)||
           !multiply(term[eliminated],gradient,rateTerm))return Clip::Unsafe;
        const long double offset=constant+offsetTerm;
        const long double rate=term[freeIndex]+rateTerm;
        if(!std::isfinite(offset)||!std::isfinite(rate))return Clip::Unsafe;
        if(rate==0) {if(offset==0)coincidentMask|=mask;return offset>0?Clip::Empty:Clip::Valid;}
        long double boundary=0;
        if(!divide(-offset,rate,boundary))return Clip::Unsafe;
        if(rate>0) {if(boundary<upper)upperMask=mask;else if(boundary==upper)upperMask|=mask;upper=std::min(upper,boundary);}
        else {if(boundary>lower)lowerMask=mask;else if(boundary==lower)lowerMask|=mask;lower=std::max(lower,boundary);}
        return lower>upper?Clip::Empty:Clip::Valid;
    };
    long double kh=0;
    if(!multiply(k,h,kh))return fallback;
    const std::array<std::array<long double,3>,4> domain={{{0,-1,0},{-1,1,0},{0,0,-1},{-1,-kh,1}}};
    for(std::size_t domainIndex=0;domainIndex<domain.size();++domainIndex) {
        const auto& p=domain[domainIndex];
        const auto result=clip(p[0],p[1],p[2],std::uint32_t{1}<<domainIndex);
        if(result==Clip::Unsafe)return fallback;
        if(result==Clip::Empty)return empty();
    }
    for(std::size_t i=0;i<count;++i)if(i!=left&&i!=right) {
        const auto& p=planes[i];
        long double rate=0;
        if(!multiply(p.rate,h,rate))return fallback;
        const auto result=clip(p.offset,rate,p.slope,std::uint32_t{1}<<(4+i));
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
        out.vertices[i]={static_cast<double>(time),static_cast<double>(point[1]),(out.count==1?lowerMask|upperMask:i==0?lowerMask:upperMask)|coincidentMask};
        const auto& v=out.vertices[i];
        if(!std::isfinite(v.time)||!std::isfinite(v.along)||v.time<0||v.time>duration||
           (time!=0&&v.time==0)||(point[1]!=0&&v.along==0))return fallback;
    }
    // Return BOTH interval ends. The caller retains its original common
    // lex(time,along), normal, closed-skin and exit postselection predicates.
    return out;
}

struct MaterialDomainV3 {
    bool valid=false;MaterialBoundaryVertexV3 entry,leave;std::uint32_t initialSliceMask=0,entrySliceMask=0;
};
inline bool materialOrderingV3(const MaterialBoundaryVertexV3& a,const MaterialBoundaryVertexV3& b) {
    return a.time<b.time||(a.time==b.time&&a.along<b.along);
}
inline MaterialDomainV3 materialDomainV3(const PlaneEqualityInputV1* planes,std::size_t count,
    double k,double duration,const std::vector<std::array<double,2>>& native) {
    std::vector<MaterialBoundaryVertexV3> polygon;
    const auto reduced=reduceMaterialBoundaryV3(planes,count,k,duration);
    if(reduced.status==MaterialBoundaryResultV3::Status::Reduced)
        polygon.assign(reduced.vertices.begin(),reduced.vertices.begin()+static_cast<std::ptrdiff_t>(reduced.count));
    else {
        polygon={{0,0,1|4},{duration,0,2|4},{duration,1+k*duration,2|8},{0,1,1|8}};
        for(std::size_t index=0;index<count;++index) {
            if(polygon.empty())return {};
            const auto& plane=planes[index];
            const auto value=[&](MaterialBoundaryVertexV3 p){return plane.offset+plane.rate*p.time+plane.slope*p.along;};
            std::vector<MaterialBoundaryVertexV3> next;auto before=polygon.back();double vb=value(before);
            for(const auto after:polygon) {
                const double va=value(after);
                if((vb<=0)!=(va<=0)) {
                    const double fraction=vb/(vb-va);
                    next.push_back({before.time+fraction*(after.time-before.time),before.along+fraction*(after.along-before.along),
                        (before.mask&after.mask)|(std::uint32_t{1}<<(4+index))});
                }
                if(va<=0) {auto kept=after;if(va==0)kept.mask|=std::uint32_t{1}<<(4+index);next.push_back(kept);}
                before=after;vb=va;
            }
            polygon=std::move(next);
        }
    }
    if(polygon.empty()||polygon.size()!=native.size())return {};
    for(std::size_t i=0;i<polygon.size();++i)
        if(polygon[i].time!=native[i][0]||polygon[i].along!=native[i][1])return {};
    const auto entry=*std::min_element(polygon.begin(),polygon.end(),materialOrderingV3);
    const auto leave=*std::max_element(polygon.begin(),polygon.end(),materialOrderingV3);
    std::uint32_t initialMask=0,entryMask=0;
    for(const auto& vertex:polygon) {
        if(vertex.time==0)initialMask|=vertex.mask;
        if(vertex.time==entry.time)entryMask|=vertex.mask;
    }
    return {true,entry,leave,initialMask,entryMask};
}
inline bool materialPointEqualV3(V3 a,V3 b) noexcept {return a.x==b.x&&a.y==b.y&&a.z==b.z;}
inline bool materialEdgeEqualV3(V3 a,V3 b,V3 c,V3 d) noexcept {
    return (materialPointEqualV3(a,c)&&materialPointEqualV3(b,d))||
        (materialPointEqualV3(a,d)&&materialPointEqualV3(b,c));
}
struct MaterialFacetQueryV3 {
    MaterialFacetKeyV3 key;std::array<V3,3> vertices{};
    double thickness=0;MaterialDomainV3 domain;
    V3 at,velocity,segment;double expansion=0,duration=0;
};

// Outward intervals certify an actual UNIQUE affine shared-edge event; they
// do not alter the native collision root, timing, coordinates or tolerances.
struct MaterialIntervalV3 {long double low=0,high=0;};
inline long double materialDownV3(long double x) {return x==0?-std::numeric_limits<long double>::denorm_min():std::nextafter(x,-std::numeric_limits<long double>::infinity());}
inline long double materialUpV3(long double x) {return x==0?std::numeric_limits<long double>::denorm_min():std::nextafter(x,std::numeric_limits<long double>::infinity());}
inline MaterialIntervalV3 materialAddV3(MaterialIntervalV3 a,MaterialIntervalV3 b) {return {materialDownV3(a.low+b.low),materialUpV3(a.high+b.high)};}
inline MaterialIntervalV3 materialNegV3(MaterialIntervalV3 a) {return {-a.high,-a.low};}
inline MaterialIntervalV3 materialSubV3(MaterialIntervalV3 a,MaterialIntervalV3 b) {return materialAddV3(a,materialNegV3(b));}
inline MaterialIntervalV3 materialMulV3(MaterialIntervalV3 a,MaterialIntervalV3 b) {
    const std::array<long double,4> p={a.low*b.low,a.low*b.high,a.high*b.low,a.high*b.high};
    return {materialDownV3(*std::min_element(p.begin(),p.end())),materialUpV3(*std::max_element(p.begin(),p.end()))};
}
inline MaterialIntervalV3 materialDivV3(MaterialIntervalV3 a,MaterialIntervalV3 b) {
    if(b.low<=0&&b.high>=0)return {-std::numeric_limits<long double>::infinity(),std::numeric_limits<long double>::infinity()};
    return materialMulV3(a,{materialDownV3(1/b.high),materialUpV3(1/b.low)});
}
using MaterialVectorIntervalV3=std::array<MaterialIntervalV3,3>;
inline MaterialVectorIntervalV3 materialVectorV3(V3 a) {return {{{a.x,a.x},{a.y,a.y},{a.z,a.z}}};}
inline MaterialVectorIntervalV3 materialVectorSubV3(MaterialVectorIntervalV3 a,MaterialVectorIntervalV3 b) {
    for(std::size_t i=0;i<3;++i)a[i]=materialSubV3(a[i],b[i]);
    return a;
}
inline MaterialIntervalV3 materialDetV3(const MaterialVectorIntervalV3& a,const MaterialVectorIntervalV3& b,const MaterialVectorIntervalV3& c) {
    return materialAddV3(materialSubV3(materialMulV3(a[0],materialSubV3(materialMulV3(b[1],c[2]),materialMulV3(b[2],c[1]))),
        materialMulV3(a[1],materialSubV3(materialMulV3(b[0],c[2]),materialMulV3(b[2],c[0])))),
        materialMulV3(a[2],materialSubV3(materialMulV3(b[0],c[1]),materialMulV3(b[1],c[0]))));
}
inline bool materialAffineEdgeEventV3(const MaterialFacetQueryV3& query,V3 edge0,V3 edge1) {
    const auto v=materialVectorV3(query.velocity),d=materialVectorV3(query.segment);
    const auto negativeEdge=materialVectorSubV3(materialVectorV3(edge0),materialVectorV3(edge1));
    const auto rhs=materialVectorSubV3(materialVectorV3(edge0),materialVectorV3(query.at));
    const auto determinant=materialDetV3(v,d,negativeEdge);
    if(!std::isfinite(determinant.low)||!std::isfinite(determinant.high)||
       (determinant.low<=0&&determinant.high>=0))return false;
    const auto t=materialDivV3(materialDetV3(rhs,d,negativeEdge),determinant);
    const auto q=materialDivV3(materialDetV3(v,rhs,negativeEdge),determinant);
    const auto u=materialDivV3(materialDetV3(v,d,rhs),determinant);
    const auto maximum=materialAddV3({1,1},materialMulV3({query.expansion,query.expansion},t));
    return std::isfinite(t.low)&&std::isfinite(t.high)&&std::isfinite(q.low)&&std::isfinite(q.high)&&
        std::isfinite(u.low)&&std::isfinite(u.high)&&t.low>=0&&t.high<=query.duration&&q.low>=0&&q.high<=maximum.low&&u.low>=0&&u.high<=1;
}
inline bool sameMaterialBranchV3(const MaterialFacetQueryV3& oldFace,const MaterialFacetQueryV3& newFace) {
    if(!oldFace.domain.valid||!newFace.domain.valid||oldFace.key.victim!=newFace.key.victim||
       oldFace.key.generation!=newFace.key.generation||oldFace.thickness!=newFace.thickness)return false;
    // An entry limited by a PARTICULAR shared physical edge is essential.
    // Time overlap or global shell connectivity alone is never sufficient.
    // A first contact may be an entire finite q-segment at ANY fixed
    // future time. Its shared edge can occur at the other q endpoint, not
    // at the lexicographically first point. Keep the complete entry slice.
    const auto entryEdges=newFace.domain.entrySliceMask;
    for(std::size_t edge=0;edge<3;++edge)if(entryEdges&(std::uint32_t{1}<<(6+edge)))
        for(std::size_t previous=0;previous<3;++previous)
            if(materialEdgeEqualV3(newFace.vertices[edge],newFace.vertices[(edge+1)%3],
                oldFace.vertices[previous],oldFace.vertices[(previous+1)%3])) {
                if(newFace.domain.entry.time==0) {
                    if(oldFace.domain.initialSliceMask&(std::uint32_t{1}<<(6+previous)))return true;
                    continue;
                }
                if(oldFace.domain.leave.time>=newFace.domain.entry.time)return true;
                // Independently rounded face equations may straddle their
                // EXACT common edge by a rounding gap. Matching exit/entry
                // provenance identifies the same affine edge event without
                // a distance, time or normal-angle epsilon.
                if((oldFace.domain.leave.mask&(std::uint32_t{1}<<(6+previous)))&&
                   materialAffineEdgeEventV3(newFace,newFace.vertices[edge],newFace.vertices[(edge+1)%3]))return true;
            }
    return false;
}
inline bool materialBitsEqualV3(double a,double b) noexcept {
    static_assert(sizeof(double)==sizeof(std::uint64_t),"binary64 snapshot required");
    std::uint64_t x=0,y=0;std::memcpy(&x,&a,sizeof(x));std::memcpy(&y,&b,sizeof(y));return x==y;
}
inline bool materialPointBitsEqualV3(V3 a,V3 b) noexcept {return materialBitsEqualV3(a.x,b.x)&&materialBitsEqualV3(a.y,b.y)&&materialBitsEqualV3(a.z,b.z);}
inline bool materialPoseEqualV3(const Target& a,const Target& b) noexcept {
    if(!materialPointEqualV3(a.at,b.at))return false;
    for(std::size_t i=0;i<3;++i)if(!materialPointEqualV3(a.bodyToEcef[i],b.bodyToEcef[i]))return false;
    return true;
}
inline bool materialFieldEqualV3(const FragmentFieldV1& a,const FragmentFieldV1& b) {
    if(a.valid!=b.valid||!materialBitsEqualV3(a.caseKg,b.caseKg)||!materialBitsEqualV3(a.tntKg,b.tntKg)||!materialBitsEqualV3(a.ageS,b.ageS)||a.emittedCount!=b.emittedCount||
       a.applyBlast!=b.applyBlast||!materialPointBitsEqualV3(a.burst.at,b.burst.at)||!materialPointBitsEqualV3(a.burst.axis,b.burst.axis)||
       !materialPointBitsEqualV3(a.burst.velocity,b.burst.velocity)||!materialPointBitsEqualV3(a.burst.angularVelocityEcef,b.burst.angularVelocityEcef)||
       a.projectiles.size()!=b.projectiles.size())return false;
    for(std::size_t i=0;i<a.projectiles.size();++i) {
        const auto& x=a.projectiles[i];const auto& y=b.projectiles[i];
        if(x.sourceOrdinalV1!=y.sourceOrdinalV1||!materialPointBitsEqualV3(x.at,y.at)||!materialPointBitsEqualV3(x.velocity,y.velocity)||
           !materialPointBitsEqualV3(x.end,y.end)||!materialPointBitsEqualV3(x.endVelocity,y.endVelocity)||!materialBitsEqualV3(x.massKg,y.massKg)||
           !materialBitsEqualV3(x.areaM2,y.areaM2)||!materialBitsEqualV3(x.cd,y.cd)||x.segment!=y.segment||!materialBitsEqualV3(x.expansionPerS,y.expansionPerS)||
           !materialBitsEqualV3(x.breakupAgeS,y.breakupAgeS)||!materialBitsEqualV3(x.materialLengthM,y.materialLengthM)||x.crossedFaces!=y.crossedFaces||
           x.struckVictims!=y.struckVictims)return false;
    }
    return true;
}
} // namespace fsim::ms::damage::detail
