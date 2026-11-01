#include "simulation/mission_systems/weapon/Autopilot.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms::weapon {
namespace {
bool valid(const ControlAxis& a) {
    return std::isfinite(a.forceAngle) && std::isfinite(a.forceControl) &&
        std::isfinite(a.momentAngle) && std::isfinite(a.momentControl) && std::isfinite(a.momentRate) &&
        std::abs(a.momentControl) > 1e-9;
}
double forceSlope(const ControlAxis& a) {
    return a.forceAngle - a.forceControl * a.momentAngle / a.momentControl;
}
double reference(const ControlAxis& a, double force, double pathRate, double forceResidual, double momentResidual) {
    return (force - forceResidual + a.forceControl * (a.momentRate * pathRate + momentResidual) / a.momentControl) / forceSlope(a);
}
double surface(const ControlAxis& a, double inertia, double angle, double rate, double rateDemand,
               double rateTime, double limit, double momentResidual, double couplingMoment) {
    const double delta = (inertia * (rateDemand - rate) / rateTime + couplingMoment -
                          a.momentAngle * angle - a.momentRate * rate - momentResidual) / a.momentControl;
    return delta / limit;
}
FinCommands cascade(const AutopilotInput& in,const FinControlConfigV1& config,const ControlStopsV1& stops) noexcept {
    if (!std::isfinite(in.massKg) || in.massKg <= 0 || !std::isfinite(in.speedMs) || in.speedMs < 1 ||
        !std::isfinite(in.actuatorLagS) || in.actuatorLagS <= 0 ||
        !std::isfinite(in.alphaRad) || !std::isfinite(in.betaRad) ||
        std::cos(in.alphaRad) * std::cos(in.betaRad) <= 0 || !valid(in.pitch) || !valid(in.yaw) ||
        !std::isfinite(in.rollMomentPerRad) ||
        !std::isfinite(in.rollDamping)) return {};
    for (std::size_t k = 0; k < 3; ++k)
        if (!std::isfinite(in.demandBody[k]) || !std::isfinite(in.gravityBody[k]) ||
            !std::isfinite(in.ratesRadS[k]) || !std::isfinite(in.inertiaKgM2[k]) || in.inertiaKgM2[k] <= 0 ||
            !std::isfinite(in.limitsRad[k]) || in.limitsRad[k] < 0 || (k != 0 && in.limitsRad[k] == 0)) return {};
    for (const auto* a : {&in.pitch, &in.yaw}) {
        const double effective = forceSlope(*a);
        if (!std::isfinite(effective) || std::abs(effective) < 1e-6 * std::max(std::abs(a->forceAngle), 1.)) return {};
    }
    // Euler's diagonal-inertia rigid body requires M = J*omega_dot +
    // omega cross (J*omega). Aerodynamic-load feedback does not contain this
    // inertial coupling, including when its aerodynamic measurement is exact.
    const auto& w=in.ratesRadS;
    const auto& inertia=in.inertiaKgM2;
    const std::array<double,3> couplingMoment={
        (inertia[2]-inertia[1])*w[1]*w[2],
        (inertia[0]-inertia[2])*w[0]*w[2],
        (inertia[1]-inertia[0])*w[0]*w[1]};
    for (const double moment : couplingMoment)
        if (!std::isfinite(moment)) return {};
    // Account for loads already present in the native model but absent from
    // its small-incidence derivatives (crossflow, finite fin geometry, etc.).
    // This is a local additive residual, not a replacement force or a fitted
    // effectiveness derivative. Use measured actual fins from the SAME frame.
    double pitchForce=0,yawForce=0,pitchMoment=0,yawMoment=0,rollMoment=0;
    if (in.measured) {
        const auto& m=*in.measured;
        for (std::size_t k=0;k<3;++k)
            if (!std::isfinite(m.forceBodyN[k]) || !std::isfinite(m.momentBodyNm[k]) || !std::isfinite(m.controlRad[k])) return {};
        pitchForce=m.forceBodyN[2]-in.pitch.forceAngle*in.alphaRad-in.pitch.forceControl*m.controlRad[1];
        yawForce=m.forceBodyN[1]-in.yaw.forceAngle*in.betaRad-in.yaw.forceControl*m.controlRad[2];
        pitchMoment=m.momentBodyNm[1]-in.pitch.momentAngle*in.alphaRad-in.pitch.momentRate*in.ratesRadS[1]-in.pitch.momentControl*m.controlRad[1];
        yawMoment=m.momentBodyNm[2]-in.yaw.momentAngle*in.betaRad-in.yaw.momentRate*in.ratesRadS[2]-in.yaw.momentControl*m.controlRad[2];
        rollMoment=m.momentBodyNm[0]-in.rollDamping*in.ratesRadS[0]-in.rollMomentPerRad*m.controlRad[0];
        for (double residual : {pitchForce,yawForce,pitchMoment,yawMoment,rollMoment})
            if (!std::isfinite(residual)) return {};
    }
    // Demand is specific force including gravity compensation. Path curvature
    // instead uses net acceleration. UVWdot is not an accelerometer signal.
    const double qPath = -(in.demandBody[2] + in.gravityBody[2]) / in.speedMs;
    const double rPath = (in.demandBody[1] + in.gravityBody[1]) / in.speedMs;
    // Constant-rate trim also needs the coupling torque. Its balancing fin
    // deflection contributes direct force, so compensate before solving trim.
    double alpha = reference(in.pitch, in.massKg * in.demandBody[2], qPath,pitchForce,pitchMoment-couplingMoment[1]);
    double beta = reference(in.yaw, in.massKg * in.demandBody[1], rPath,yawForce,yawMoment-couplingMoment[2]);
    if (!std::isfinite(alpha) || !std::isfinite(beta)) return {};
    // Configured reduced-order control envelope (legacy default: 15 degrees).
    const double maxIncidence = config.maxIncidenceRad;
    const double incidence = std::hypot(alpha, beta);
    if (!std::isfinite(incidence)) return {};
    if (incidence > maxIncidence) alpha *= maxIncidence / incidence, beta *= maxIncidence / incidence;
    // Estimated bandwidth, explicitly slower than the finite actuator. The
    // nominal 40 ms actuator gives 120 ms rate and 400 ms incidence response.
    const double rateTime = std::max(config.rateTimeMinS, config.rateLagMultiplier * in.actuatorLagS);
    const double incidenceTime = std::max(config.incidenceTimeMinS, config.incidenceRateMultiplier * rateTime);
    if (!std::isfinite(rateTime) || !std::isfinite(incidenceTime)) return {};
    const double qDemand = qPath + (alpha - in.alphaRad) / incidenceTime;
    const double rDemand = rPath - (beta - in.betaRad) / incidenceTime;
    FinCommands out;
    out.elevator = surface(in.pitch, in.inertiaKgM2[1], in.alphaRad, in.ratesRadS[1], qDemand, rateTime, in.limitsRad[1],pitchMoment,couplingMoment[1]);
    out.rudder = surface(in.yaw, in.inertiaKgM2[2], in.betaRad, in.ratesRadS[2], rDemand, rateTime, in.limitsRad[2],yawMoment,couplingMoment[2]);
    // Some authored two-axis canards have no roll effector. Their valid pitch
    // and yaw channels still operate; passive roll damping stays in the plant.
    if (in.limitsRad[0] > 0 && std::abs(in.rollMomentPerRad) > 1e-9)
        out.aileron = (-in.inertiaKgM2[0] * in.ratesRadS[0] / rateTime + couplingMoment[0] - in.rollDamping * in.ratesRadS[0] - rollMoment) /
                       (in.rollMomentPerRad * in.limitsRad[0]);
    if (!std::isfinite(out.elevator) || !std::isfinite(out.rudder) || !std::isfinite(out.aileron)) return {};
    // The authored native mixer takes normalized virtual channels. Its scale
    // is zero-centred: asymmetric negative travel uses |lower|, not max(|stop|).
    const auto normalize=[&](double command,std::size_t axis) {
        if (stops.version==0) return std::clamp(command,-1.,1.);
        const double radians=command*in.limitsRad[axis];
        if (!std::isfinite(radians)) return std::numeric_limits<double>::quiet_NaN();
        const double extent=radians<0 ? -stops.lowerRad[axis] : stops.upperRad[axis];
        return extent>0 ? std::clamp(radians/extent,-1.,1.) : 0.;
    };
    out.elevator = normalize(out.elevator,1);
    out.rudder = normalize(out.rudder,2);
    out.aileron = normalize(out.aileron,0);
    if (!std::isfinite(out.elevator) || !std::isfinite(out.rudder) || !std::isfinite(out.aileron)) return {};
    out.available = true;
    return out;
}
}
FinCommands actuatorAwareCommands(const AutopilotInput& in,const FinControlConfigV1& config,const ControlStopsV1& stops) noexcept {
    auto out=finCommands(in,config,stops);
    if(!out.available || config.version!=1 || !in.measured)return {};
    const double rateTime=std::max(config.rateTimeMinS,config.rateLagMultiplier*in.actuatorLagS);
    const double angleTime=std::max(config.incidenceTimeMinS,config.incidenceRateMultiplier*rateTime);
    const double lag=1/in.actuatorLagS,fast=1/rateTime,slow=1/angleTime;
    const double p2=lag+fast+slow,p1=lag*fast+lag*slow+fast*slow,p0=lag*fast*slow;
    const auto& m=*in.measured;const auto& w=in.ratesRadS;const auto& j=in.inertiaKgM2;
    const double gyro[3]={(j[2]-j[1])*w[1]*w[2],(j[0]-j[2])*w[0]*w[2],(j[1]-j[0])*w[0]*w[1]};
    struct Axis {const ControlAxis* plant;double sign,angle,rate,inertia,delta,forceResidual,momentResidual,path,reference;};
    Axis axes[2]={{&in.pitch,1,in.alphaRad,w[1],j[1],m.controlRad[1],0,0,0,0},
                  {&in.yaw,-1,in.betaRad,w[2],j[2],m.controlRad[2],0,0,0,0}};
    for(unsigned i=0;i<2;++i) {
        auto& x=axes[i];const auto& a=*x.plant;const unsigned forceAxis=i?1u:2u,momentAxis=i?2u:1u;
        x.forceResidual=m.forceBodyN[forceAxis]-a.forceAngle*x.angle-a.forceControl*x.delta;
        x.momentResidual=m.momentBodyNm[momentAxis]-a.momentAngle*x.angle-a.momentRate*x.rate-a.momentControl*x.delta-gyro[momentAxis];
        x.path=-x.sign*(in.demandBody[forceAxis]+in.gravityBody[forceAxis])/in.speedMs;
        x.reference=reference(a,in.massKg*in.demandBody[forceAxis],x.path,x.forceResidual,x.momentResidual);
    }
    const double incidence=std::hypot(axes[0].reference,axes[1].reference);
    if(!std::isfinite(incidence))return {};
    for(unsigned i=0;i<2;++i) {
        auto& x=axes[i];const auto& a=*x.plant;const double mv=in.massKg*in.speedMs;
        if(incidence>config.maxIncidenceRad) {
            x.reference*=config.maxIncidenceRad/incidence;
            // Once angle is constrained, curvature must use the achievable
            // trim force, rather than the rejected high-g request.
            const double denominator=mv-x.sign*a.forceControl*a.momentRate/a.momentControl;
            if(!std::isfinite(denominator)||std::abs(denominator)<1e-9)return {};
            const double force=forceSlope(a)*x.reference+x.forceResidual-a.forceControl*x.momentResidual/a.momentControl;
            x.path=-x.sign*(force+in.massKg*in.gravityBody[i?1:2])/denominator;
        }
        const double trim=-(a.momentAngle*x.reference+a.momentRate*x.path+x.momentResidual)/a.momentControl;
        // Characteristic polynomial of [angle, rate, actual surface] with the
        // finite first-order actuator. Assign three real negative poles using
        // the existing response times, without tuning per-missile gains.
        const double aa=a.forceAngle/mv,b=x.sign,c=a.forceControl/mv;
        const double d=a.momentAngle/x.inertia,e=a.momentRate/x.inertia,f=a.momentControl/x.inertia;
        const double det=aa*e-b*d,kDelta=(p2+aa+e)/lag-1;
        const double u=c,v=f,y=b*f-c*e,z=c*d-aa*f;
        const double determinant=u*z-v*y;
        if(!std::isfinite(determinant)||std::abs(determinant)<1e-12*std::max({std::abs(u*z),std::abs(v*y),1.}))return {};
        const double rhs1=(p1-det+lag*(1+kDelta)*(aa+e))/lag;
        const double rhs0=(p0-lag*(1+kDelta)*det)/lag;
        const double kAngle=(rhs1*z-v*rhs0)/determinant,kRate=(u*rhs0-rhs1*y)/determinant;
        const double command=trim-kAngle*(x.angle-x.reference)-kRate*(x.rate-x.path)-kDelta*(x.delta-trim);
        const unsigned axis=i?2:1;
        const double extent=stops.version==1 ? (command<0?-stops.lowerRad[axis]:stops.upperRad[axis]):in.limitsRad[axis];
        if(!std::isfinite(command)||!std::isfinite(extent)||extent<=0)return {};
        (i?out.rudder:out.elevator)=std::clamp(command/extent,-1.,1.);
    }
    return out;
}
FinCommands finCommands(const AutopilotInput& in) noexcept { return finCommands(in,FinControlConfigV1{}); }
FinCommands finCommands(const AutopilotInput& in,const FinControlConfigV1& requested,const ControlStopsV1& stops) noexcept {
    if (requested.version>1 || stops.version>1 || stops.actuatorVersion>1) return {};
    // Unversioned callers retain every original constant and operation order.
    const auto config=requested.version==0 ? FinControlConfigV1{} : requested;
    if (!std::isfinite(config.maxIncidenceRad) || config.maxIncidenceRad<=0 || config.maxIncidenceRad>=1.5707963267948966
        || !std::isfinite(config.rateTimeMinS) || config.rateTimeMinS<=0
        || !std::isfinite(config.rateLagMultiplier) || config.rateLagMultiplier<=0
        || !std::isfinite(config.incidenceTimeMinS) || config.incidenceTimeMinS<=0
        || !std::isfinite(config.incidenceRateMultiplier) || config.incidenceRateMultiplier<=0) return {};
    if (requested.version==1) {
        if(config.thrustVector && (!std::isfinite(config.thrustVectorLimitRad) || config.thrustVectorLimitRad<=0 ||
            config.thrustVectorLimitRad>=1.5707963267948966 || !std::isfinite(config.thrustVectorRateRadS) ||
            config.thrustVectorRateRadS<=0))return {};
        // Metadata is consumed by the native FCS itself. Check the actual
        // time constant, channel rate and every physical-fin rate; never
        // replace the measured plant input with its card expectation.
        const auto same=[](double actual,double expected) {
            return std::isfinite(actual) && std::isfinite(expected) && actual>0 && expected>0 &&
                std::abs(actual-expected)<=16*std::numeric_limits<double>::epsilon()*std::max(actual,expected);
        };
        if (stops.actuatorVersion!=1 || !same(in.actuatorLagS,config.actuatorLagS)
            || !same(stops.channelRateRadS,config.actuatorRateRadS) || stops.finRateRadS.empty()
            || !std::all_of(stops.finRateRadS.begin(),stops.finRateRadS.end(),
                [&](double actual){return same(actual,config.actuatorRateRadS);})) return {};
    }
    if (stops.version==1) for(std::size_t axis=0;axis<3;++axis) {
        const double low=stops.lowerRad[axis], high=stops.upperRad[axis];
        if (!std::isfinite(low) || !std::isfinite(high) || low>0 || high<0 || low>high
            || std::max(-low,high)!=in.limitsRad[axis]
            || (low==high && (axis!=0 || in.limitsRad[axis]!=0))) return {};
    }
    switch(config.algorithm) {
    case 1:
        switch(config.allocator) {
        case 1: return cascade(in,config,stops); // native authored mixer, no invented per-fin allocator
        default: return {};
        }
    default: return {};
    }
}
}
