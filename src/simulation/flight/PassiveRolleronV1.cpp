#include "PassiveRolleronV1.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

namespace fsim::sim {
namespace {
bool finite(RolleronVectorV1 v) {return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
RolleronVectorV1 cross(RolleronVectorV1 a,RolleronVectorV1 b) {return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
void plus(RolleronVectorV1& a,RolleronVectorV1 b) {a.x+=b.x;a.y+=b.y;a.z+=b.z;}
}
PassiveRolleronV1::PassiveRolleronV1(std::vector<RolleronConfigV1> config):config_(std::move(config)) {
    if(config_.empty()||config_.size()>32)throw std::invalid_argument("Invalid passive rolleron population");
    std::set<std::uint32_t> ids;std::set<std::size_t> indices;
    for(const auto& c:config_) {
        const double positive[]={c.wheelRadiusM,c.captureAreaM2,c.wheelInertiaKgM2,c.wheelDriveCoefficient,
            c.freeSpinRatio,c.wheelDragCoefficient,c.hingeInertiaKgM2,c.hingeStiffnessNmRad,
            c.hingeDampingNmsRad,c.limitRad,c.rateRadS};
        if(!c.finId||!ids.insert(c.finId).second||!indices.insert(c.finIndex).second||
           !finite(c.centreFromOriginBodyM)||!finite(c.hingeAxisBody)||
           c.hingeAxisBody.x!=0||std::abs(c.hingeAxisBody.y*c.hingeAxisBody.y+c.hingeAxisBody.z*c.hingeAxisBody.z-1)>1e-10||
           (c.nativeDeflectionSign!=1&&c.nativeDeflectionSign!=-1)||
           !std::all_of(std::begin(positive),std::end(positive),[](double x){return std::isfinite(x)&&x>0;})||
           !std::isfinite(c.bearingDampingNms)||c.bearingDampingNms<0||
           !std::isfinite(c.hingeAerodynamicStiffnessM3)||c.hingeAerodynamicStiffnessM3<0||
           c.freeSpinRatio>1||c.limitRad>=.5||
           c.wheelDragCoefficient<c.wheelDriveCoefficient*c.freeSpinRatio/4)
            throw std::invalid_argument("Invalid passive rolleron physical inputs");
    }
    reset();
}
void PassiveRolleronV1::reset() {
    state_.assign(config_.size(),{});output_={};output_.fins.reserve(config_.size());
    for(const auto& c:config_)output_.fins.push_back({c.finId,c.finIndex,0,0,0});
}
RolleronOutputV1 PassiveRolleronV1::advance(const RolleronInputV1& in) {
    if(!std::isfinite(in.dtS)||in.dtS<=0||in.dtS>1||!std::isfinite(in.densityKgM3)||in.densityKgM3<0||
       !finite(in.airVelocityBodyMs)||!finite(in.angularRateInertialBodyRadS)||!finite(in.cgFromOriginBodyM))return {};
    const double u=in.airVelocityBodyMs.x,speed=std::abs(u),q=.5*in.densityKgM3*u*u;
    if(!std::isfinite(q)||!std::isfinite(speed))return {};
    auto states=state_;RolleronOutputV1 out;out.fins.reserve(config_.size());
    const double roll=in.angularRateInertialBodyRadS.x;
    for(std::size_t index=0;index<config_.size();++index) {
        const auto& c=config_[index];auto& s=states[index];const auto original=s;
        const double drive=std::copysign(c.wheelDriveCoefficient*q*c.captureAreaM2*c.wheelRadiusM,u);
        const double windDamping=speed>0?std::abs(drive)*c.wheelRadiusM/(c.freeSpinRatio*speed):0;
        const double stiffness=c.hingeStiffnessNmRad+q*c.hingeAerodynamicStiffnessM3;
        const double decay=std::max({(windDamping+c.bearingDampingNms)/c.wheelInertiaKgM2,
            c.hingeDampingNmsRad/c.hingeInertiaKgM2,std::sqrt(stiffness/c.hingeInertiaKgM2)});
        if(!std::isfinite(drive)||!std::isfinite(stiffness)||!std::isfinite(decay))return {};
        const double intervals=std::ceil(in.dtS/std::min(.001,.1/std::max(decay,1.)));
        if(!std::isfinite(intervals)||intervals>8192)return {};
        const auto count=static_cast<unsigned>(std::max(1.,intervals));const double dt=in.dtS/count;
        const auto derivative=[&](State x) {
            double rate=std::clamp(x.rate,-c.rateRadS,c.rateRadS);
            double acceleration=(-stiffness*x.angle-c.hingeDampingNmsRad*rate-c.wheelInertiaKgM2*x.wheel*roll)/c.hingeInertiaKgM2;
            if((rate>=c.rateRadS&&acceleration>0)||(rate<=-c.rateRadS&&acceleration<0))acceleration=0;
            // A blocked hinge has no motion and therefore no damper work.
            // Constrain each RK stage, not only the final reported state.
            if(x.angle>=c.limitRad&&rate>=0){rate=0;if(acceleration>0)acceleration=0;}
            if(x.angle<=-c.limitRad&&rate<=0){rate=0;if(acceleration<0)acceleration=0;}
            return State{(drive-(windDamping+c.bearingDampingNms)*x.wheel)/c.wheelInertiaKgM2,rate,acceleration};
        };
        const auto moved=[](State x,State a,double time) {return State{x.wheel+a.wheel*time,x.angle+a.angle*time,x.rate+a.rate*time};};
        double gyroImpulse=0,wheelBearingImpulse=0;
        for(unsigned step=0;step<count;++step) {
            const auto a=derivative(s),bstate=moved(s,a,dt/2),b=derivative(bstate);
            const auto cstate=moved(s,b,dt/2),third=derivative(cstate),dstate=moved(s,third,dt),fourth=derivative(dstate);
            const State samples[]={s,bstate,cstate,dstate};const double weights[]={1.,2.,2.,1.};
            for(std::size_t k=0;k<4;++k) {
                const auto x=samples[k];const double weight=weights[k]*dt/6;
                // Gyro impulse is integrated below on the accepted bounded trajectory.
                out.harvestedAirPowerW+=weight*(drive-windDamping*x.wheel)*x.wheel/in.dtS;
                out.bearingHeatJ+=weight*c.bearingDampingNms*x.wheel*x.wheel;
                wheelBearingImpulse+=weight*c.bearingDampingNms*x.wheel;
                const double actualRate=derivative(x).angle;
                out.hingeHeatJ+=weight*c.hingeDampingNmsRad*actualRate*actualRate;
            }
            const auto start=s;
            s={s.wheel+dt*(a.wheel+2*b.wheel+2*third.wheel+fourth.wheel)/6,
               s.angle+dt*(a.angle+2*b.angle+2*third.angle+fourth.angle)/6,
               s.rate+dt*(a.rate+2*b.rate+2*third.rate+fourth.rate)/6};
            const double before=.5*c.hingeInertiaKgM2*s.rate*s.rate+.5*stiffness*s.angle*s.angle;
            s.rate=std::clamp(s.rate,-c.rateRadS,c.rateRadS);
            s.angle=std::clamp(s.angle,start.angle-c.rateRadS*dt,start.angle+c.rateRadS*dt);
            if(s.angle>=c.limitRad){s.angle=c.limitRad;if(s.rate>0)s.rate=0;}
            if(s.angle<=-c.limitRad){s.angle=-c.limitRad;if(s.rate<0)s.rate=0;}
            gyroImpulse+=c.wheelInertiaKgM2*.5*(start.wheel+s.wheel)*(s.angle-start.angle);
            out.stopLossJ+=std::max(0.,before-.5*c.hingeInertiaKgM2*s.rate*s.rate-.5*stiffness*s.angle*s.angle);
            if(!std::isfinite(s.wheel)||!std::isfinite(s.angle)||!std::isfinite(s.rate))return {};
        }
        const double drag=c.wheelDragCoefficient*q*c.captureAreaM2;
        const RolleronVectorV1 force{-std::copysign(drag,u),0,0};plus(out.forceBodyN,force);
        plus(out.torqueBodyNm,cross({c.centreFromOriginBodyM.x-in.cgFromOriginBodyM.x,
            c.centreFromOriginBodyM.y-in.cgFromOriginBodyM.y,c.centreFromOriginBodyM.z-in.cgFromOriginBodyM.z},force));
        // Same gyro exchange appears on hinge and body with opposite work.
        out.torqueBodyNm.x+=gyroImpulse/in.dtS;
        // Air supplies the wheel's drive torque externally. Only the equal
        // opposite bearing-friction impulse reaches the body along this axis;
        // the wheel's full spin-up is not an internal reaction-wheel torque.
        const double wheelReaction=wheelBearingImpulse/in.dtS;
        plus(out.torqueBodyNm,{0,-c.hingeAxisBody.z*wheelReaction,c.hingeAxisBody.y*wheelReaction});
        const double hingeReaction=-c.hingeInertiaKgM2*(s.rate-original.rate)/in.dtS;
        plus(out.torqueBodyNm,{c.hingeAxisBody.x*hingeReaction,c.hingeAxisBody.y*hingeReaction,c.hingeAxisBody.z*hingeReaction});
        out.gyroBodyWorkJ+=gyroImpulse*roll;out.gyroHingeWorkJ-=gyroImpulse*roll;
        out.dragPowerW+=drag*speed;
        out.internalEnergyJ+=.5*c.wheelInertiaKgM2*s.wheel*s.wheel+.5*c.hingeInertiaKgM2*s.rate*s.rate+.5*stiffness*s.angle*s.angle;
        out.fins.push_back({c.finId,c.finIndex,s.wheel,c.nativeDeflectionSign*s.angle,c.nativeDeflectionSign*s.rate});
    }
    const double scalars[]={out.dragPowerW,out.harvestedAirPowerW,out.bearingHeatJ,out.hingeHeatJ,
        out.stopLossJ,out.gyroBodyWorkJ,out.gyroHingeWorkJ,out.internalEnergyJ};
    if(!finite(out.forceBodyN)||!finite(out.torqueBodyNm)||
       !std::all_of(std::begin(scalars),std::end(scalars),[](double x){return std::isfinite(x);}))return {};
    out.valid=true;state_=std::move(states);output_=out;return out;
}
}
