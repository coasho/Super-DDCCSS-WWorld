#include "NavigationV1.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fsim::ms::weapon {
namespace {
std::array<double,3> values(V3 v) {return {v.x,v.y,v.z};}
bool finite(V3 v) {return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
V3 interpolate(V3 a,V3 b,double weight) {
    return {a.x+(b.x-a.x)*weight,a.y+(b.y-a.y)*weight,a.z+(b.z-a.z)*weight};
}
}
NavigationV1::NavigationV1(NavigationConfigV1 config,std::uint64_t seed):config_(config),seed_(seed) {
    const double scalars[]={config.imuPeriodS,config.gnssPeriodS,config.accelerometerNoiseDensity,
        config.biasSigmaMs2,config.biasWalkDensity,config.initialPositionSigmaM,
        config.initialVelocitySigmaMs,config.gnssPositionSigmaM,config.gnssVelocitySigmaMs};
    if(!std::all_of(std::begin(scalars),std::end(scalars),[](double v){return std::isfinite(v)&&v>=0;})||
       config.imuPeriodS<=0||config.gnssPeriodS<=0||
       (config.gnssAiding&&(config.gnssPositionSigmaM<=0||config.gnssVelocitySigmaMs<=0)))
        throw std::invalid_argument("Invalid weapon navigation engineering policy");
    reset();
}
void NavigationV1::reset() noexcept {
    initialized_=false;gnssAidingValid_=false;axes_={};previous_={};epoch_=time_=lastImu_=0;
    lastGnss_=std::numeric_limits<double>::quiet_NaN();nextImu_=nextGnss_=1;
    initialRng_=Rng::forVehicle(seed_,0,0x4e4156494e4954ull);
    imuRng_=Rng::forVehicle(seed_,0,0x4e4156494d55ull);
    biasRng_=Rng::forVehicle(seed_,0,0x4e415642494153ull);
    gnssRng_=Rng::forVehicle(seed_,0,0x4e4156474e5353ull);
}
bool NavigationV1::valid(const NavigationInputV1& input) noexcept {
    return std::isfinite(input.timeS)&&input.timeS>=0&&finite(input.position)&&finite(input.velocity)&&finite(input.acceleration);
}
bool NavigationV1::initialize(const NavigationInputV1& input) {
    if(initialized_||!valid(input))return false;
    auto next=*this;
    const auto position=values(input.position),velocity=values(input.velocity);
    for(std::size_t i=0;i<3;++i) {
        auto& axis=next.axes_[i];
        axis.state={position[i]+config_.initialPositionSigmaM*next.initialRng_.normal(),
                    velocity[i]+config_.initialVelocitySigmaMs*next.initialRng_.normal(),0,0};
        axis.actualBias=config_.biasSigmaMs2*next.biasRng_.normal();
        axis.covariance(0,0)=config_.initialPositionSigmaM*config_.initialPositionSigmaM;
        axis.covariance(1,1)=config_.initialVelocitySigmaMs*config_.initialVelocitySigmaMs;
        axis.covariance(2,2)=config_.biasSigmaMs2*config_.biasSigmaMs2;
    }
    next.epoch_=next.time_=input.timeS;next.previous_=input;next.imu(input,true);next.initialized_=true;
    for(const auto& axis:next.axes_)if(!std::isfinite(axis.heldAcceleration)||!std::isfinite(axis.actualBias)||
       !std::all_of(axis.covariance.a.begin(),axis.covariance.a.end(),[](double v){return std::isfinite(v);}))return false;
    const auto snapshot=next.estimate();
    if(!finite(snapshot.position)||!finite(snapshot.velocity)||
       !std::all_of(snapshot.covariance.begin(),snapshot.covariance.end(),[](double v){return std::isfinite(v);}))return false;
    *this=next;return true;
}
void NavigationV1::imu(const NavigationInputV1& input,bool first) {
    const auto acceleration=values(input.acceleration);
    const auto whiteSigma=config_.accelerometerNoiseDensity/std::sqrt(config_.imuPeriodS);
    for(std::size_t i=0;i<3;++i) {
        auto& axis=axes_[i];
        if(!first) {
            axis.actualBias+=config_.biasWalkDensity*std::sqrt(config_.imuPeriodS)*biasRng_.normal();
            axis.covariance(2,2)+=config_.biasWalkDensity*config_.biasWalkDensity*config_.imuPeriodS;
        }
        // Each physical IMU exposure replaces its held white error. Retain
        // this nuisance state between exposures so partial-step propagation
        // and mid-exposure GNSS updates keep its actual correlations.
        axis.state[3]=0;
        for(std::size_t k=0;k<4;++k)axis.covariance(3,k)=axis.covariance(k,3)=0;
        axis.covariance(3,3)=whiteSigma*whiteSigma;
        axis.heldAcceleration=acceleration[i]+axis.actualBias+whiteSigma*imuRng_.normal();
    }
    lastImu_=input.timeS;
}
void NavigationV1::propagate(double dt) {
    const double h2=dt*dt;
    auto transition=track::identity<4>();transition(0,1)=dt;
    for(const std::size_t error:{2u,3u}){transition(0,error)=-.5*h2;transition(1,error)=-dt;}
    for(auto& axis:axes_) {
        const auto acceleration=axis.heldAcceleration-axis.state[2]-axis.state[3];
        axis.state[0]+=axis.state[1]*dt+.5*acceleration*h2;
        axis.state[1]+=acceleration*dt;
        axis.covariance=transition*axis.covariance*track::transpose(transition);
    }
}
void NavigationV1::gnss(const NavigationInputV1& input) {
    if(!config_.gnssAiding||!input.gnssAvailable)return;
    const auto position=values(input.position),velocity=values(input.velocity);
    const double rp=config_.gnssPositionSigmaM*config_.gnssPositionSigmaM;
    const double rv=config_.gnssVelocitySigmaMs*config_.gnssVelocitySigmaMs;
    for(std::size_t i=0;i<3;++i) {
        auto& axis=axes_[i];const auto prior=axis.covariance;
        track::Mat<2,2> innovationCovariance;
        innovationCovariance(0,0)=prior(0,0)+rp;innovationCovariance(0,1)=prior(0,1);
        innovationCovariance(1,0)=prior(1,0);innovationCovariance(1,1)=prior(1,1)+rv;
        track::Mat<2,2> inverse;
        if(!track::inverse(innovationCovariance,inverse))throw std::runtime_error("Singular weapon GNSS covariance");
        track::Mat<4,2> cross;
        for(std::size_t row=0;row<4;++row)for(std::size_t col=0;col<2;++col)cross(row,col)=prior(row,col);
        const auto gain=cross*inverse;
        const double residual[2]={position[i]+config_.gnssPositionSigmaM*gnssRng_.normal()-axis.state[0],
            velocity[i]+config_.gnssVelocitySigmaMs*gnssRng_.normal()-axis.state[1]};
        for(std::size_t row=0;row<4;++row)axis.state[row]+=gain(row,0)*residual[0]+gain(row,1)*residual[1];
        auto residualTransition=track::identity<4>();
        for(std::size_t row=0;row<4;++row)for(std::size_t col=0;col<2;++col)residualTransition(row,col)-=gain(row,col);
        track::Mat<2,2> measurementNoise;measurementNoise(0,0)=rp;measurementNoise(1,1)=rv;
        axis.covariance=residualTransition*prior*track::transpose(residualTransition)+gain*measurementNoise*track::transpose(gain);
    }
    lastGnss_=input.timeS;gnssAidingValid_=true;
}
bool NavigationV1::advance(const NavigationInputV1& input) {
    if(!initialized_||!valid(input)||input.timeS<time_)return false;
    if(input.timeS==time_)return true; // A read/retry cannot redraw an exposure.
    const auto dt=input.timeS-time_;
    if(!std::isfinite(dt)||dt/config_.imuPeriodS>100000||
       (config_.gnssAiding&&dt/config_.gnssPeriodS>100000))return false;
    auto next=*this;
    try {if(!next.advanceImpl(input))return false;}catch(const std::exception&){return false;}
    *this=next;return true;
}
bool NavigationV1::advanceImpl(const NavigationInputV1& input) {
    // A chronological outage invalidates aiding until another actual GNSS
    // exposure succeeds. Availability alone cannot renew a recent posterior.
    if(!input.gnssAvailable)gnssAidingValid_=false;
    const double span=input.timeS-previous_.timeS;
    for(unsigned events=0;events<200002;++events) {
        const auto imuTime=epoch_+static_cast<double>(nextImu_)*config_.imuPeriodS;
        const auto gnssTime=config_.gnssAiding?epoch_+static_cast<double>(nextGnss_)*config_.gnssPeriodS:
            std::numeric_limits<double>::infinity();
        const auto event=std::min(imuTime,gnssTime);
        if(event>input.timeS)break;
        if(event<time_||!std::isfinite(event))return false;
        propagate(event-time_);time_=event;
        const double weight=std::clamp((event-previous_.timeS)/span,0.,1.);
        NavigationInputV1 sample;sample.timeS=event;
        sample.position=interpolate(previous_.position,input.position,weight);
        sample.velocity=interpolate(previous_.velocity,input.velocity,weight);
        sample.acceleration=interpolate(previous_.acceleration,input.acceleration,weight);
        // Unavailability during either end of an unresolved exposure interval
        // cannot synthesize a precise mid-interval availability transition.
        sample.gnssAvailable=previous_.gnssAvailable&&input.gnssAvailable;
        if(event==imuTime){imu(sample,false);++nextImu_;}
        if(event==gnssTime){gnss(sample);++nextGnss_;}
    }
    propagate(input.timeS-time_);time_=input.timeS;previous_=input;
    for(auto& axis:axes_) {
        if(!std::all_of(axis.state.begin(),axis.state.end(),[](double v){return std::isfinite(v);})||
           !std::all_of(axis.covariance.a.begin(),axis.covariance.a.end(),[](double v){return std::isfinite(v);})||
           !std::isfinite(axis.heldAcceleration)||!std::isfinite(axis.actualBias))return false;
        for(std::size_t i=0;i<4;++i) {
            if(axis.covariance(i,i)<-1e-10)return false;
            axis.covariance(i,i)=std::max(0.,axis.covariance(i,i));
            for(std::size_t j=0;j<i;++j)axis.covariance(i,j)=axis.covariance(j,i)=.5*(axis.covariance(i,j)+axis.covariance(j,i));
        }
    }
    return true;
}
NavigationEstimateV1 NavigationV1::estimate() const noexcept {
    NavigationEstimateV1 out;out.valid=initialized_;out.timeS=time_;out.lastImuSampleS=lastImu_;out.lastGnssSampleS=lastGnss_;
    out.gnssAided=initialized_&&config_.gnssAiding&&gnssAidingValid_&&previous_.gnssAvailable&&std::isfinite(lastGnss_)&&time_-lastGnss_<=config_.gnssPeriodS;
    out.position={axes_[0].state[0],axes_[1].state[0],axes_[2].state[0]};
    out.velocity={axes_[0].state[1],axes_[1].state[1],axes_[2].state[1]};
    for(std::size_t axis=0;axis<3;++axis)for(std::size_t row=0;row<2;++row)for(std::size_t col=0;col<2;++col)
        out.covariance[6*(2*axis+row)+2*axis+col]=axes_[axis].covariance(row,col);
    return out;
}
}
