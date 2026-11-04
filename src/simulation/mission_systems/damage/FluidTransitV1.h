#pragma once

// Private, policy-free reduced-fluid methods. No authored fuel occupancy, fluid
// coefficient, fragmentation, pressure-wave or ignition policy is selected here.
// All frames/units must agree: metres, seconds, kg; planes use n.x <= offset.
#include "Damage.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace fsim::ms::damage::fluid {

struct Aabb { V3 low,high; };
struct Plane { V3 normal; double offset; }; // Immersed side: dot(normal,x) <= offset.
struct VolumeControls { double absoluteToleranceM3,relativeTolerance; unsigned maxIterations; };
struct SurfaceResult {
    bool valid=false,converged=false;
    double offset=0,volume=0,residualM3=0;
    unsigned iterations=0;
};
struct Interval { bool valid=false,intersects=false; double first=0,last=0; };
struct Point { V3 position,velocity; double massKg; };
// Area is the complete point/rod presented area. Cd is an explicit fluid-law
// input, never an implicit copy of an air coefficient or a hardware default.
struct Medium { V3 velocity; double densityKgM3,dragCoefficient,areaM2; };
struct PointResult {
    V3 position,velocity,mediumImpulse;
    double elapsedS=0,relativeDistance=0,dissipatedEnergyJ=0;
};
struct Rod { V3 end0,end1,velocity0,velocity1; double massKg; };
struct QuadratureControls {
    double absoluteForceToleranceN,relativeTolerance;
    unsigned maxDepth,maxEvaluations;
};
struct RodResult {
    bool valid=false,converged=false;
    V3 force0,force1,force,torqueAboutMidpoint,acceleration0,acceleration1;
    double inertialPowerW=0,relativePowerW=0,estimatedForceErrorN=0;
    unsigned evaluations=0;
};

namespace detail {
inline bool finite(double x) noexcept { return std::isfinite(x); }
inline bool finite(long double x) noexcept { return std::isfinite(x); }
inline bool finite(V3 p) noexcept { return finite(p.x)&&finite(p.y)&&finite(p.z); }
inline double norm(V3 p) noexcept { return std::hypot(p.x,p.y,p.z); }
inline long double scalar(V3 a,V3 b) noexcept {
    return static_cast<long double>(a.x)*b.x+static_cast<long double>(a.y)*b.y+static_cast<long double>(a.z)*b.z;
}
struct Box {
    V3 centre,half;
    double volume;
};
inline std::optional<Box> box(Aabb a) {
    if(!finite(a.low)||!finite(a.high))return {};
    const V3 half=(a.high-a.low)*0.5;
    const V3 centre=a.low*0.5+a.high*0.5;
    const double volume=8*half.x*half.y*half.z;
    if(!finite(half)||!finite(centre)||!(half.x>0&&half.y>0&&half.z>0)||!finite(volume)||!(volume>0))return {};
    return Box{centre,half,volume};
}
struct LocalPlane { V3 normal; double height,extent,normalLength; };
inline std::optional<LocalPlane> plane(const Box& b,Plane p) {
    const double size=norm(p.normal);
    if(!finite(p.normal)||!finite(p.offset)||!finite(size)||!(size>0))return {};
    // Component division also handles subnormal input normals without 1/size overflow.
    const V3 n{p.normal.x/size,p.normal.y/size,p.normal.z/size};
    const double height=static_cast<double>(static_cast<long double>(p.offset)/size-scalar(n,b.centre));
    const double extent=std::abs(n.x)*b.half.x+std::abs(n.y)*b.half.y+std::abs(n.z)*b.half.z;
    if(!finite(height)||!finite(extent))return {};
    return LocalPlane{n,height,extent,size};
}

// Clip the six outward-wound faces, close the cut with its outward cap, and sum
// tetrahedra about an interior point. This avoids small-axis inverse-CDF
// cancellation and cancellation between large tetrahedra for a thin wet cap.
inline std::optional<double> volume(const Box& b,V3 n,double height,double extent) {
    if(height<=-extent)return 0.0;
    if(height>=extent)return b.volume;
    const double x=b.half.x,y=b.half.y,z=b.half.z;
    std::vector<std::vector<V3>> faces{
        {{-x,-y,-z},{-x,-y,z},{-x,y,z},{-x,y,-z}},
        {{x,-y,-z},{x,y,-z},{x,y,z},{x,-y,z}},
        {{-x,-y,-z},{x,-y,-z},{x,-y,z},{-x,-y,z}},
        {{-x,y,-z},{-x,y,z},{x,y,z},{x,y,-z}},
        {{-x,-y,-z},{-x,y,-z},{x,y,-z},{x,-y,-z}},
        {{-x,-y,z},{x,-y,z},{x,y,z},{-x,y,z}}
    };
    std::vector<V3> cap;
    for(auto& face:faces) {
        std::vector<V3> clipped;
        for(std::size_t i=0;i<face.size();++i) {
            const V3 a=face[i],c=face[(i+1)%face.size()];
            const long double da=scalar(n,a)-height,dc=scalar(n,c)-height;
            const bool inA=da<=0,inC=dc<=0;
            if(inA)clipped.push_back(a);
            if(inA!=inC) {
                const double t=static_cast<double>(da/(da-dc));
                const V3 cut=a*(1-t)+c*t;
                if(!finite(cut))return {};
                clipped.push_back(cut);cap.push_back(cut);
            }
        }
        face=std::move(clipped);
    }
    // Do not merge merely close points: a thin but finite box can have genuine
    // corners much closer than epsilon times its largest dimension. Repeated
    // shared-edge points are harmless to the polygon triangulation.
    std::vector<V3> unique;
    for(V3 p:cap) {
        bool duplicate=false;
        for(V3 q:unique)if(p.x==q.x&&p.y==q.y&&p.z==q.z){duplicate=true;break;}
        if(!duplicate)unique.push_back(p);
    }
    if(unique.size()>=3) {
        V3 centre{};for(V3 p:unique)centre+=p*(1.0/static_cast<double>(unique.size()));
        const V3 axis=std::abs(n.x)<=std::abs(n.y)&&std::abs(n.x)<=std::abs(n.z)?V3{1,0,0}:
            (std::abs(n.y)<=std::abs(n.z)?V3{0,1,0}:V3{0,0,1});
        const V3 raw=cross(n,axis),u=raw*(1/norm(raw)),v=cross(n,u);
        std::sort(unique.begin(),unique.end(),[&](V3 a,V3 c){
            return std::atan2(dot(a-centre,v),dot(a-centre,u))<std::atan2(dot(c-centre,v),dot(c-centre,u));
        });
        faces.push_back(std::move(unique));
    }
    V3 origin{};std::size_t count=0;
    for(const auto& face:faces)for(V3 p:face){origin+=p;++count;}
    if(count==0)return 0.0;
    origin=origin*(1.0/static_cast<double>(count));
    long double sum=0;
    for(const auto& face:faces)for(std::size_t i=1;i+1<face.size();++i) {
        const V3 a=face[0]-origin,c=face[i]-origin,d=face[i+1]-origin;
        // Long-double products throughout the scalar triple product.
        sum+=(static_cast<long double>(a.x)*(static_cast<long double>(c.y)*d.z-static_cast<long double>(c.z)*d.y)+
              static_cast<long double>(a.y)*(static_cast<long double>(c.z)*d.x-static_cast<long double>(c.x)*d.z)+
              static_cast<long double>(a.z)*(static_cast<long double>(c.x)*d.y-static_cast<long double>(c.y)*d.x))/6;
    }
    const double result=static_cast<double>(sum);
    const double roundoff=256*std::numeric_limits<double>::epsilon()*b.volume;
    if(!finite(result)||result < -roundoff||result>b.volume+roundoff)return {};
    return std::clamp(result,0.0,b.volume);
}
inline bool mediumValid(Medium m) noexcept {
    return finite(m.velocity)&&finite(m.densityKgM3)&&m.densityKgM3>=0&&
        finite(m.dragCoefficient)&&m.dragCoefficient>=0&&finite(m.areaM2)&&m.areaM2>=0;
}
inline std::optional<double> drag(Medium m,double mass) {
    if(!mediumValid(m)||!finite(mass)||!(mass>0))return {};
    const long double k=static_cast<long double>(m.densityKgM3)*m.dragCoefficient*m.areaM2/(2*static_cast<long double>(mass));
    const double result=static_cast<double>(k);
    if(!finite(result)||(k>0&&result==0))return {};
    return result;
}
inline std::optional<PointResult> finish(Point p,Medium m,V3 relative,double speed,double time,double distance,double ratio) {
    PointResult out;
    out.elapsedS=time;out.relativeDistance=distance;
    out.velocity=m.velocity+relative*ratio;
    out.position=p.position+m.velocity*time+(speed>0?relative*(distance/speed):V3{});
    out.mediumImpulse=relative*(p.massKg*(1-ratio));
    const long double energy=0.5L*p.massKg*speed*speed*(1-static_cast<long double>(ratio))*(1+static_cast<long double>(ratio));
    out.dissipatedEnergyJ=static_cast<double>(energy);
    if(!finite(out.velocity)||!finite(out.position)||!finite(out.mediumImpulse)||!finite(out.dissipatedEnergyJ))return {};
    return out;
}
using Six=std::array<double,6>;
inline Six plus(Six a,const Six& b) noexcept {for(unsigned i=0;i<6;++i)a[i]+=b[i];return a;}
inline Six simpson(const Six& a,const Six& b,const Six& c,double width) noexcept {
    Six out{};for(unsigned i=0;i<6;++i)out[i]=(a[i]+4*b[i]+c[i])*(width/6);return out;
}
struct Integrator {
    const Rod& rod;Medium medium;QuadratureControls controls;
    unsigned evaluations=0;double error=0;Six errors{};bool failed=false;
    Six sample(double s) {
        if(evaluations>=controls.maxEvaluations){failed=true;return {};}
        ++evaluations;
        const V3 relative=rod.velocity0*(1-s)+rod.velocity1*s-medium.velocity;
        const long double scale=-0.5L*medium.densityKgM3*medium.dragCoefficient*medium.areaM2*norm(relative);
        const V3 force{static_cast<double>(scale*relative.x),static_cast<double>(scale*relative.y),static_cast<double>(scale*relative.z)};
        if(!finite(relative)||!finite(force)){failed=true;return {};}
        return {force.x,force.y,force.z,s*force.x,s*force.y,s*force.z};
    }
    struct Cell {
        double a=0,b=0;
        Six fa{},fm{},fb{},leftMiddle{},rightMiddle{},left{},right{},integral{},estimate{};
        unsigned depth=0;
        bool active=true;
    };
    Cell cell(double a,double b,const Six& fa,const Six& fm,const Six& fb,const Six& coarse,unsigned depth) {
        Cell out;out.a=a;out.b=b;out.fa=fa;out.fm=fm;out.fb=fb;out.depth=depth;
        const double middle=a+(b-a)*0.5;
        if(middle==a||middle==b) {
            // A positive represented interval need not contain an interior
            // double. Retain its measure with positive endpoint weights and
            // certify the rule in the same global quadrature error ledger.
            const auto magnitude=[](V3 v){return std::hypot(static_cast<long double>(v.x),static_cast<long double>(v.y),static_cast<long double>(v.z));};
            const auto speed=[&](double s) {
                return std::hypot(static_cast<long double>(rod.velocity0.x)*(1-static_cast<long double>(s))+static_cast<long double>(rod.velocity1.x)*s-medium.velocity.x,
                    static_cast<long double>(rod.velocity0.y)*(1-static_cast<long double>(s))+static_cast<long double>(rod.velocity1.y)*s-medium.velocity.y,
                    static_cast<long double>(rod.velocity0.z)*(1-static_cast<long double>(s))+static_cast<long double>(rod.velocity1.z)*s-medium.velocity.z);
            };
            const long double width=static_cast<long double>(b)-a,maximum=std::max(speed(a),speed(b));
            const long double change=std::hypot(static_cast<long double>(rod.velocity1.x)-rod.velocity0.x,
                static_cast<long double>(rod.velocity1.y)-rod.velocity0.y,static_cast<long double>(rod.velocity1.z)-rod.velocity0.z);
            const long double coefficient=.5L*medium.densityKgM3*medium.dragCoefficient*medium.areaM2;
            const long double epsilon=std::numeric_limits<double>::epsilon();
            const long double velocityRoundoff=8*epsilon*(magnitude(rod.velocity0)+magnitude(rod.velocity1)+magnitude(medium.velocity));
            const long double forceBound=coefficient*(maximum+velocityRoundoff)*(maximum+velocityRoundoff);
            const long double forceRoundoff=coefficient*(2*maximum*velocityRoundoff+velocityRoundoff*velocityRoundoff)+16*epsilon*forceBound;
            const long double derivative=2*coefficient*maximum*change;
            for(unsigned i=0;i<6;++i) {
                const long double integral=(static_cast<long double>(fa[i])+fb[i])*(width/2);
                const long double intervalError=(derivative+(i<3?0:forceBound))*width*width/2+
                    (forceRoundoff+8*epsilon*forceBound)*width;
                out.integral[i]=static_cast<double>(integral);out.estimate[i]=static_cast<double>(intervalError);
                if(intervalError>0&&out.estimate[i]==0)out.estimate[i]=std::numeric_limits<double>::denorm_min();
                if(!finite(out.integral[i])||!finite(out.estimate[i])||!std::isfinite(intervalError)){failed=true;return out;}
            }
            out.depth=controls.maxDepth; // No representable subdivision remains.
            return out;
        }
        out.leftMiddle=sample(a+(middle-a)*0.5);out.rightMiddle=sample(middle+(b-middle)*0.5);
        if(failed)return out;
        out.left=simpson(fa,out.leftMiddle,fm,middle-a);
        out.right=simpson(fm,out.rightMiddle,fb,b-middle);
        out.integral=plus(out.left,out.right);
        for(unsigned i=0;i<6;++i) {
            if(!finite(out.integral[i])||!finite(coarse[i])){failed=true;return out;}
            out.estimate[i]=std::abs(out.integral[i]-coarse[i])/15;
        }
        return out;
    }
    Six integrate(double a,double b,const Six& fa,const Six& fm,const Six& fb,const Six& coarse) {
        // Maintain global component sums and absolute error estimates. Local
        // relative tolerances are insufficient when subintervals cancel.
        // Six lazy priority queues select the largest remaining contribution
        // to whichever global component currently violates its error budget.
        std::vector<Cell> cells;
        using Queue=std::priority_queue<std::pair<double,std::size_t>>;
        std::array<Queue,6> queues;
        std::array<long double,6> total{},totalError{};
        const auto add=[&](Cell value) {
            const std::size_t index=cells.size();cells.push_back(std::move(value));
            const auto& inserted=cells.back();
            for(unsigned i=0;i<6;++i) {
                total[i]+=inserted.integral[i];totalError[i]+=inserted.estimate[i];
                if(inserted.depth<controls.maxDepth&&inserted.estimate[i]>0)
                    queues[i].push({inserted.estimate[i],index});
            }
        };
        Cell initial=cell(a,b,fa,fm,fb,coarse,0);
        if(failed)return {};
        add(std::move(initial));
        while(true) {
            unsigned critical=6;long double worst=0;
            Six result{};
            error=0;
            for(unsigned i=0;i<6;++i) {
                result[i]=static_cast<double>(total[i]);
                errors[i]=static_cast<double>(std::max(0.0L,totalError[i]));
                if(!finite(result[i])||!finite(errors[i])){failed=true;return {};}
                error=std::max(error,errors[i]);
                const double tolerance=controls.absoluteForceToleranceN+controls.relativeTolerance*std::abs(result[i]);
                if(!finite(tolerance)){failed=true;return {};}
                if(totalError[i]>tolerance) {
                    const long double severity=tolerance>0?totalError[i]/tolerance:std::numeric_limits<long double>::infinity();
                    if(critical==6||severity>worst){critical=i;worst=severity;}
                }
            }
            if(critical==6)return result; // Positive quadrature weights preserve dissipation.
            auto& queue=queues[critical];
            while(!queue.empty()&&!cells[queue.top().second].active)queue.pop();
            if(queue.empty()){failed=true;return {};}
            const std::size_t index=queue.top().second;queue.pop();
            // Copy before append, which can reallocate cells. Complete both
            // child estimates before replacing this parent atomically.
            const Cell parent=cells[index];
            if(controls.maxEvaluations-evaluations<4){failed=true;return {};}
            const double middle=parent.a+(parent.b-parent.a)*0.5;
            Cell left=cell(parent.a,middle,parent.fa,parent.leftMiddle,parent.fm,parent.left,parent.depth+1);
            Cell right=cell(middle,parent.b,parent.fm,parent.rightMiddle,parent.fb,parent.right,parent.depth+1);
            if(failed)return {};
            cells[index].active=false;
            for(unsigned i=0;i<6;++i){total[i]-=parent.integral[i];totalError[i]-=parent.estimate[i];}
            add(std::move(left));add(std::move(right));
        }
    }
};
} // namespace detail

inline std::optional<double> immersedVolume(Aabb a,Plane p) {
    const auto b=detail::box(a);if(!b)return {};
    const auto plane=detail::plane(*b,p);if(!plane)return {};
    return detail::volume(*b,plane->normal,plane->height,plane->extent);
}

// Solve actual volume, not an effective density. Offset uses the caller's
// unnormalized plane normal. Finite budgets and residual are always visible.
inline SurfaceResult freeSurface(Aabb a,V3 normal,double actualVolumeM3,VolumeControls controls) {
    SurfaceResult out;
    const auto b=detail::box(a);
    if(!b||!detail::finite(actualVolumeM3)||actualVolumeM3<0||actualVolumeM3>b->volume||
       !detail::finite(controls.absoluteToleranceM3)||controls.absoluteToleranceM3<0||
       !detail::finite(controls.relativeTolerance)||controls.relativeTolerance<0||
       !(controls.absoluteToleranceM3>0||controls.relativeTolerance>0)||controls.maxIterations==0)return out;
    // Use a local plane to avoid cancellation while bisecting far from origin.
    const double size=detail::norm(normal);
    if(!detail::finite(normal)||!detail::finite(size)||!(size>0))return out;
    const V3 n{normal.x/size,normal.y/size,normal.z/size};
    const double extent=std::abs(n.x)*b->half.x+std::abs(n.y)*b->half.y+std::abs(n.z)*b->half.z;
    const double tolerance=controls.absoluteToleranceM3+controls.relativeTolerance*b->volume;
    if(!detail::finite(extent)||!detail::finite(tolerance))return out;
    double low=-extent,high=extent;
    const bool boundary=actualVolumeM3==0||actualVolumeM3==b->volume;
    for(unsigned i=0;i<controls.maxIterations;++i) {
        const double h=boundary?(actualVolumeM3==0?low:high):low+(high-low)*0.5;
        const auto volume=detail::volume(*b,n,h,extent);if(!volume)return {};
        const double offset=static_cast<double>((detail::scalar(n,b->centre)+h)*size);
        if(!detail::finite(offset))return {};
        // Re-evaluate the returned plane: global double offset quantization may
        // prevent requested accuracy. Never report an unattainable local solve.
        const auto represented=immersedVolume(a,{normal,offset});if(!represented)return {};
        out={true,false,offset,*represented,*represented-actualVolumeM3,i+1};
        if(std::abs(out.residualM3)<=tolerance){out.converged=true;return out;}
        if(boundary||h==low||h==high)return out;
        if(*volume<actualVolumeM3)low=h;else high=h;
    }
    return out;
}

inline Interval clipImmersedLine(Aabb a,Plane p,V3 start,V3 end) {
    Interval out;
    const auto box=detail::box(a);if(!box||!detail::finite(start)||!detail::finite(end))return out;
    const auto plane=detail::plane(*box,p);if(!plane)return out;
    const V3 local0=start-box->centre,local1=end-box->centre;
    if(!detail::finite(local0)||!detail::finite(local1))return out;
    double first=0,last=1;
    const auto clip=[&](long double at0,long double at1) {
        if(at0<=0&&at1<=0)return true;
        if(at0>0&&at1>0)return false;
        const double cut=static_cast<double>(at0/(at0-at1));
        if(at0>0)first=std::max(first,cut);else last=std::min(last,cut);
        return first<=last;
    };
    const double coordinates0[]{local0.x,local0.y,local0.z},coordinates1[]{local1.x,local1.y,local1.z};
    const double half[]{box->half.x,box->half.y,box->half.z};
    out.valid=true;
    for(unsigned i=0;i<3;++i) {
        if(!clip(static_cast<long double>(coordinates0[i])-half[i],static_cast<long double>(coordinates1[i])-half[i])||
           !clip(-static_cast<long double>(coordinates0[i])-half[i],-static_cast<long double>(coordinates1[i])-half[i]))return out;
    }
    if(!clip(detail::scalar(plane->normal,local0)-plane->height,detail::scalar(plane->normal,local1)-plane->height))return out;
    out.intersects=true;out.first=first;out.last=last;return out;
}

// Constant medium, constant mass/area/Cd; interval must stay in that medium.
// Integration/event scheduling chooses the interval, not this primitive.
inline std::optional<PointResult> advancePoint(Point p,Medium medium,double elapsedS) {
    const auto k=detail::drag(medium,p.massKg);
    if(!k||!detail::finite(p.position)||!detail::finite(p.velocity)||!detail::finite(elapsedS)||elapsedS<0)return {};
    const V3 relative=p.velocity-medium.velocity;const double speed=detail::norm(relative);
    if(!detail::finite(relative)||!detail::finite(speed))return {};
    const long double exposure=static_cast<long double>(*k)*speed*elapsedS;
    const double ratio=static_cast<double>(1/(1+exposure));
    // Avoid a zero residual caused by underflow masquerading as absorption.
    if(!detail::finite(exposure)||(*k>0&&speed>0&&ratio==0))return {};
    const double distance=*k>0?static_cast<double>(std::log1p(exposure)/ *k):speed*elapsedS;
    if(!detail::finite(distance))return {};
    return detail::finish(p,medium,relative,speed,elapsedS,distance,ratio);
}

// Distance is measured relative to the translating homogeneous medium, not
// an inertial chord. Positive distance cannot be crossed at zero relative speed.
inline std::optional<PointResult> advancePointDistance(Point p,Medium medium,double relativeDistance) {
    const auto k=detail::drag(medium,p.massKg);
    if(!k||!detail::finite(p.position)||!detail::finite(p.velocity)||!detail::finite(relativeDistance)||relativeDistance<0)return {};
    const V3 relative=p.velocity-medium.velocity;const double speed=detail::norm(relative);
    if(!detail::finite(relative)||!detail::finite(speed)||(!(speed>0)&&relativeDistance>0))return {};
    const long double exposure=static_cast<long double>(*k)*relativeDistance;
    const double ratio=static_cast<double>(std::exp(-exposure));
    if(!detail::finite(exposure)||(speed>0&&ratio==0))return {};
    const double time=relativeDistance==0?0:(*k>0?
        static_cast<double>(std::expm1(exposure)/ (static_cast<long double>(*k)*speed)):relativeDistance/speed);
    if(!detail::finite(time))return {};
    return detail::finish(p,medium,relative,speed,time,relativeDistance,ratio);
}

// Uniform mass and area per s in [0,1]. Position and velocity are endpoint
// affine. Integrates only [immersedFirst,immersedLast]. Adaptive Simpson errors
// are estimates, not certified quadrature bounds; failure publishes no loads.
// No rod time step, rigidity projection, breakup, or material policy is implied.
inline RodResult rodLoads(const Rod& rod,Medium medium,double immersedFirst,double immersedLast,QuadratureControls controls) {
    RodResult out;
    if(!detail::finite(rod.end0)||!detail::finite(rod.end1)||!detail::finite(rod.velocity0)||!detail::finite(rod.velocity1)||
       !detail::finite(rod.massKg)||!(rod.massKg>0)||!detail::mediumValid(medium)||
       !detail::finite(immersedFirst)||!detail::finite(immersedLast)||immersedFirst<0||immersedLast>1||immersedFirst>immersedLast||
       !detail::finite(controls.absoluteForceToleranceN)||controls.absoluteForceToleranceN<0||
       !detail::finite(controls.relativeTolerance)||controls.relativeTolerance<0||
       !(controls.absoluteForceToleranceN>0||controls.relativeTolerance>0)||
       controls.maxDepth==0||controls.maxDepth>30||controls.maxEvaluations<5)return out;
    const double rodLength=detail::norm(rod.end1-rod.end0);
    if(!detail::finite(rodLength)||!(rodLength>0))return out;
    out.valid=true;
    if(immersedFirst==immersedLast||medium.densityKgM3==0||medium.dragCoefficient==0||medium.areaM2==0){out.converged=true;return out;}
    detail::Integrator integrator{rod,medium,controls};
    const double middle=immersedFirst+(immersedLast-immersedFirst)*0.5;
    const auto a=integrator.sample(immersedFirst),m=integrator.sample(middle),b=integrator.sample(immersedLast);
    const auto initial=detail::simpson(a,m,b,immersedLast-immersedFirst);
    const auto integral=integrator.failed?detail::Six{}:integrator.integrate(immersedFirst,immersedLast,a,m,b,initial);
    out.evaluations=integrator.evaluations;out.estimatedForceErrorN=integrator.error;
    if(integrator.failed)return out;
    // Cancellation between subintervals must not turn a locally satisfied
    // relative criterion into an unreported global error-budget violation.
    for(unsigned i=0;i<6;++i)if(integrator.errors[i]>
        controls.absoluteForceToleranceN+controls.relativeTolerance*std::abs(integral[i]))return out;
    out.force={integral[0],integral[1],integral[2]};out.force1={integral[3],integral[4],integral[5]};
    out.force0=out.force-out.force1;
    V3 momentDensity;
    const bool endpointRule=middle==immersedFirst||middle==immersedLast;
    if(endpointRule) {
        // Reduce endpoint shape weights directly. Subtracting total-force
        // doubles erases F0 near s=1 and a tiny centre moment near s=1/2.
        const long double first=immersedFirst,last=immersedLast,width=last-first;
        double* force0[]{&out.force0.x,&out.force0.y,&out.force0.z};
        double* force1[]{&out.force1.x,&out.force1.y,&out.force1.z};
        double* moment[]{&momentDensity.x,&momentDensity.y,&momentDensity.z};
        for(unsigned i=0;i<3;++i) {
            *force0[i]=static_cast<double>(((1-first)*a[i]+(1-last)*b[i])*(width/2));
            *force1[i]=static_cast<double>((first*a[i]+last*b[i])*(width/2));
            *moment[i]=static_cast<double>(((2*first-1)*a[i]+(2*last-1)*b[i])*(width/2));
        }
        out.force=out.force0+out.force1;
    }
    out.acceleration0=(out.force0*4-out.force1*2)*(1/rod.massKg);
    out.acceleration1=(out.force1*4-out.force0*2)*(1/rod.massKg);
    const V3 half=(rod.end1-rod.end0)*0.5;
    out.torqueAboutMidpoint=cross(half,endpointRule?momentDensity:out.force1-out.force0);
    out.inertialPowerW=dot(rod.velocity0,out.force0)+dot(rod.velocity1,out.force1);
    out.relativePowerW=dot(rod.velocity0-medium.velocity,out.force0)+dot(rod.velocity1-medium.velocity,out.force1);
    if(!detail::finite(out.force)||!detail::finite(out.force0)||!detail::finite(out.force1)||
       !detail::finite(out.acceleration0)||!detail::finite(out.acceleration1)||!detail::finite(out.torqueAboutMidpoint)||
       !detail::finite(out.inertialPowerW)||!detail::finite(out.relativePowerW)||out.relativePowerW>0) {
        RodResult failed;failed.valid=true;failed.evaluations=out.evaluations;return failed;
    }
    out.converged=true;return out;
}

} // namespace fsim::ms::damage::fluid
