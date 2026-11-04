#include "NavigationStrapdownV2.h"
#include "NavigationCaptureV2.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fsim::ms::weapon {
namespace {
using Q=std::array<double,4>;
using M3=track::Mat<3,3>;
using M21=track::Mat<21,21>;
V3 add(V3 a,V3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
V3 sub(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
V3 mul(V3 a,double s){return {a.x*s,a.y*s,a.z*s};}
double dot(V3 a,V3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
V3 cross(V3 a,V3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double length(V3 a){return std::hypot(std::hypot(a.x,a.y),a.z);}
bool finite(V3 a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
std::array<double,3> values(V3 a){return {a.x,a.y,a.z};}
V3 interpolate(V3 a,V3 b,double w){return add(a,mul(sub(b,a),w));}
Q product(Q a,Q b){return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
    a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
    a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
    a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};}
Q normalize(Q q){
    const double n=std::hypot(std::hypot(q[0],q[1]),std::hypot(q[2],q[3]));
    if(!std::isfinite(n)||n==0)throw std::runtime_error("Invalid strapdown quaternion");
    for(auto& v:q)v/=n;
    return q;
}
Q exponential(V3 rotation){
    const double angle=length(rotation),half=.5*angle;
    const double scale=angle<1e-8?.5-angle*angle/48:std::sin(half)/angle;
    return normalize({std::cos(half),scale*rotation.x,scale*rotation.y,scale*rotation.z});
}
V3 rotate(Q q,V3 v){
    const V3 u{q[1],q[2],q[3]};
    return add(v,mul(add(mul(cross(u,v),q[0]),cross(u,cross(u,v))),2));
}
V3 unrotate(Q q,V3 v){return rotate({q[0],-q[1],-q[2],-q[3]},v);}
Q fromAxes(const std::array<V3,3>& c){
    const double m[3][3]={{c[0].x,c[1].x,c[2].x},{c[0].y,c[1].y,c[2].y},{c[0].z,c[1].z,c[2].z}};
    Q q{};const double trace=m[0][0]+m[1][1]+m[2][2];
    if(trace>0){const double s=2*std::sqrt(1+trace);q={.25*s,(m[2][1]-m[1][2])/s,(m[0][2]-m[2][0])/s,(m[1][0]-m[0][1])/s};}
    else {
        unsigned i=0;if(m[1][1]>m[i][i])i=1;if(m[2][2]>m[i][i])i=2;
        const unsigned j=(i+1)%3,k=(i+2)%3;const double s=2*std::sqrt(1+m[i][i]-m[j][j]-m[k][k]);
        q[0]=(m[k][j]-m[j][k])/s;q[i+1]=.25*s;q[j+1]=(m[j][i]+m[i][j])/s;q[k+1]=(m[k][i]+m[i][k])/s;
    }
    return normalize(q);
}
Q slerp(Q a,Q b,double w){
    double cosine=0;for(unsigned i=0;i<4;++i)cosine+=a[i]*b[i];
    if(cosine<0){for(auto& v:b)v=-v;cosine=-cosine;}
    cosine=std::clamp(cosine,0.,1.);const double angle=std::acos(cosine);
    double left=1-w,right=w;
    if(angle>1e-7){left=std::sin((1-w)*angle)/std::sin(angle);right=std::sin(w*angle)/std::sin(angle);}
    Q out{};for(unsigned i=0;i<4;++i)out[i]=left*a[i]+right*b[i];return normalize(out);
}
std::array<V3,3> axes(Q q){return {rotate(q,{1,0,0}),rotate(q,{0,1,0}),rotate(q,{0,0,1})};}
M3 skew(V3 v){M3 out;out(0,1)=-v.z;out(0,2)=v.y;out(1,0)=v.z;out(1,2)=-v.x;out(2,0)=-v.y;out(2,1)=v.x;return out;}
M3 rotation(Q q){M3 out;const auto c=axes(q);for(unsigned j=0;j<3;++j){const auto a=values(c[j]);for(unsigned i=0;i<3;++i)out(i,j)=a[i];}return out;}
// Left-error injection: log(Exp(dtheta+epsilon) Exp(-dtheta)) = J_left(dtheta) epsilon.
M3 leftJacobian(V3 v){
    const double angle=length(v),a2=angle*angle;const auto s=skew(v);
    const double b=angle<1e-5?.5-a2/24:(1-std::cos(angle))/a2;
    const double c=angle<1e-5?1./6-a2/120:(angle-std::sin(angle))/(a2*angle);
    return track::identity<3>()+b*s+c*(s*s);
}
M21 sparseLeft(const M21& a,const M21& b){
    M21 out;
    for(unsigned i=0;i<21;++i)for(unsigned k=0;k<21;++k){
        const double v=a(i,k);if(v==0)continue;
        for(unsigned j=0;j<21;++j)out(i,j)+=v*b(k,j);
    }
    return out;
}
void symmetrize(M21& p){for(unsigned i=0;i<21;++i)for(unsigned j=0;j<i;++j)p(i,j)=p(j,i)=.5*(p(i,j)+p(j,i));}
}

NavigationStrapdownV2::NavigationStrapdownV2(NavigationStrapdownConfigV2 config,std::uint64_t seed):config_(config),seed_(seed){
    const double scalars[]={config.imuPeriodS,config.gnssPeriodS,config.maximumIntegrationStepS,
        config.accelerometerNoiseDensity,config.gyroNoiseDensity,config.accelerometerBiasSigma,config.gyroBiasSigma,
        config.accelerometerBiasWalkDensity,config.gyroBiasWalkDensity,config.initialPositionSigmaM,
        config.initialVelocitySigmaMs,config.initialAttitudeSigmaRad,config.gnssPositionSigmaM,config.gnssVelocitySigmaMs,
        config.earthRateRadS,config.earthMuM3S2,config.earthEquatorialRadiusM,config.earthJ2};
    if(!std::all_of(std::begin(scalars),std::end(scalars),[](double v){return std::isfinite(v)&&v>=0&&std::isfinite(v*v);})||
       config.imuPeriodS<=0||config.imuPeriodS>1||config.gnssPeriodS<=0||config.maximumIntegrationStepS<=0||
       config.maximumIntegrationStepS>config.imuPeriodS||config.earthEquatorialRadiusM<=0||config.earthJ2>.1||
       (config.gnssAiding&&(!(config.gnssPositionSigmaM*config.gnssPositionSigmaM>0)||!(config.gnssVelocitySigmaMs*config.gnssVelocitySigmaMs>0))))
        throw std::invalid_argument("Invalid weapon strapdown navigation policy");
    reset();
}
void NavigationStrapdownV2::reset() noexcept {
    initialized_=gnssAidingValid_=false;nextImu_=nextGnss_=1;epoch_=time_=lastImu_=0;
    lastGnss_=std::numeric_limits<double>::quiet_NaN();previous_={};sample_={};
    position_=velocity_=accelerometerBias_=gyroBias_=whiteAcceleration_=whiteGyro_={};
    actualAccelerometerBias_=actualGyroBias_={};quaternion_={1,0,0,0};covariance_={};
    initialRng_=Rng::forVehicle(seed_,0,0x535452494e4954ull);
    accelerometerRng_=Rng::forVehicle(seed_,0,0x535452414343ull);gyroRng_=Rng::forVehicle(seed_,0,0x535452475952ull);
    accelerometerBiasRng_=Rng::forVehicle(seed_,0,0x535452414249ull);gyroBiasRng_=Rng::forVehicle(seed_,0,0x535452474249ull);
    gnssRng_=Rng::forVehicle(seed_,0,0x535452474e5353ull);
}
bool NavigationStrapdownV2::valid(const NavigationStrapdownInputV2& s) noexcept {
    if(!std::isfinite(s.timeS)||s.timeS<0||!finite(s.position)||!finite(s.velocity)||!finite(s.acceleration)||!finite(s.angularRateBodyRelativeEcefRadS))return false;
    for(unsigned i=0;i<3;++i){
        if(!finite(s.bodyToEcef[i])||std::abs(dot(s.bodyToEcef[i],s.bodyToEcef[i])-1)>1e-6)return false;
        for(unsigned j=0;j<i;++j)if(std::abs(dot(s.bodyToEcef[i],s.bodyToEcef[j]))>1e-6)return false;
    }
    return std::abs(dot(cross(s.bodyToEcef[0],s.bodyToEcef[1]),s.bodyToEcef[2])-1)<=1e-6;
}
V3 NavigationStrapdownV2::gravity(V3 p) const {
    if(config_.earthMuM3S2==0)return {};
    const double r=length(p);if(!std::isfinite(r)||r<=1)throw std::runtime_error("Invalid strapdown Earth position");
    const double inv=1/r,z2=(p.z*inv)*(p.z*inv),k=1.5*config_.earthJ2*std::pow(config_.earthEquatorialRadiusM*inv,2);
    const double scale=-config_.earthMuM3S2*inv*inv*inv;
    return {scale*p.x*(1+k*(1-5*z2)),scale*p.y*(1+k*(1-5*z2)),scale*p.z*(1+k*(3-5*z2))};
}
M3 NavigationStrapdownV2::gravityGradient(V3 p) const {
    M3 out;if(config_.earthMuM3S2==0)return out;
    const double r=length(p);if(!std::isfinite(r)||r<=1)throw std::runtime_error("Invalid strapdown Earth gradient");
    const auto v=values(p);const double ir=1/r,ir3=ir*ir*ir,ir5=ir3*ir*ir,ir7=ir5*ir*ir,ir9=ir7*ir*ir;
    const double k=1.5*config_.earthJ2*config_.earthEquatorialRadiusM*config_.earthEquatorialRadiusM;
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j){
        const double ai=i==2?3.:1.,delta=i==j?1.:0.;
        out(i,j)=-config_.earthMuM3S2*(delta*ir3-3*v[i]*v[j]*ir5+
            k*(delta*(ai*ir5-5*p.z*p.z*ir7)+v[i]*(-5*ai*v[j]*ir7-(j==2?10*p.z*ir7:0)+35*p.z*p.z*v[j]*ir9)));
    }
    return out;
}
bool NavigationStrapdownV2::initialize(const NavigationStrapdownInputV2& s){
    if(initialized_||!valid(s))return false;
    if(!(s.timeS+config_.imuPeriodS>s.timeS)||
       (config_.gnssAiding&&!(s.timeS+config_.gnssPeriodS>s.timeS)))return false;
    auto next=*this;
    try{
        const auto draw=[&](double sigma){return V3{sigma*next.initialRng_.normal(),sigma*next.initialRng_.normal(),sigma*next.initialRng_.normal()};};
        next.position_=add(s.position,draw(config_.initialPositionSigmaM));next.velocity_=add(s.velocity,draw(config_.initialVelocitySigmaMs));
        next.quaternion_=normalize(product(exponential(draw(config_.initialAttitudeSigmaRad)),fromAxes(s.bodyToEcef)));
        const double sigma[5]={config_.initialPositionSigmaM,config_.initialVelocitySigmaMs,config_.initialAttitudeSigmaRad,config_.accelerometerBiasSigma,config_.gyroBiasSigma};
        for(unsigned group=0;group<5;++group)for(unsigned i=0;i<3;++i)next.covariance_(3*group+i,3*group+i)=sigma[group]*sigma[group];
        next.actualAccelerometerBias_={config_.accelerometerBiasSigma*next.accelerometerBiasRng_.normal(),config_.accelerometerBiasSigma*next.accelerometerBiasRng_.normal(),config_.accelerometerBiasSigma*next.accelerometerBiasRng_.normal()};
        next.actualGyroBias_={config_.gyroBiasSigma*next.gyroBiasRng_.normal(),config_.gyroBiasSigma*next.gyroBiasRng_.normal(),config_.gyroBiasSigma*next.gyroBiasRng_.normal()};
        next.epoch_=next.time_=s.timeS;next.previous_=s;next.imu(s,true);next.initialized_=true;
        if(!next.finiteState())return false;
    }catch(const std::exception&){return false;}
    *this=next;return true;
}
void NavigationStrapdownV2::imu(const NavigationStrapdownInputV2& s,bool first){
    const auto truth=fromAxes(s.bodyToEcef);const V3 earth{0,0,config_.earthRateRadS};
    // a_ECEF = C f_body + gravitation - 2 Omega x v - Omega x (Omega x r).
    const auto specific=unrotate(truth,add(sub(s.acceleration,gravity(s.position)),add(mul(cross(earth,s.velocity),2),cross(earth,cross(earth,s.position)))));
    const auto omega=add(s.angularRateBodyRelativeEcefRadS,unrotate(truth,earth));
    const double rootH=std::sqrt(config_.imuPeriodS);
    const auto biasStep=[&](V3& actual,Rng& rng,double density,unsigned offset){
        if(first)return;
        actual=add(actual,{density*rootH*rng.normal(),density*rootH*rng.normal(),density*rootH*rng.normal()});
        for(unsigned i=0;i<3;++i)covariance_(offset+i,offset+i)+=density*density*config_.imuPeriodS;
    };
    biasStep(actualAccelerometerBias_,accelerometerBiasRng_,config_.accelerometerBiasWalkDensity,9);
    biasStep(actualGyroBias_,gyroBiasRng_,config_.gyroBiasWalkDensity,12);
    const double sa=config_.accelerometerNoiseDensity/rootH,sg=config_.gyroNoiseDensity/rootH;
    sample_.specificForceMs2=add(add(specific,actualAccelerometerBias_),{sa*accelerometerRng_.normal(),sa*accelerometerRng_.normal(),sa*accelerometerRng_.normal()});
    sample_.gyroRadS=add(add(omega,actualGyroBias_),{sg*gyroRng_.normal(),sg*gyroRng_.normal(),sg*gyroRng_.normal()});
    sample_.timeS=lastImu_=s.timeS;sample_.valid=true;whiteAcceleration_=whiteGyro_={};
    for(unsigned i=15;i<21;++i){for(unsigned j=0;j<21;++j)covariance_(i,j)=covariance_(j,i)=0;covariance_(i,i)=i<18?sa*sa:sg*sg;}
}
M21 NavigationStrapdownV2::dynamics(V3 p,const Q& q) const {
    M21 f;const auto c=rotation(q),earth=skew({0,0,config_.earthRateRadS});
    const auto force=skew(rotate(q,sub(sub(sample_.specificForceMs2,accelerometerBias_),whiteAcceleration_)));
    const auto gradient=gravityGradient(p)-earth*earth;
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j){
        f(i,3+j)=i==j?1.:0.;f(3+i,j)=gradient(i,j);f(3+i,3+j)=-2*earth(i,j);
        f(3+i,6+j)=-force(i,j);f(3+i,9+j)=-c(i,j);f(3+i,15+j)=-c(i,j);
        f(6+i,6+j)=-earth(i,j);f(6+i,12+j)=-c(i,j);f(6+i,18+j)=-c(i,j);
    }
    return f;
}
void NavigationStrapdownV2::integrate(double h){
    const V3 earth{0,0,config_.earthRateRadS};
    const auto rate=sub(sub(sample_.gyroRadS,gyroBias_),whiteGyro_);
    const auto specific=sub(sub(sample_.specificForceMs2,accelerometerBias_),whiteAcceleration_);
    const auto attitude=[&](double t){return normalize(product(product(exponential(mul(earth,-t)),quaternion_),exponential(mul(rate,t))));};
    const Q half=attitude(.5*h),end=attitude(h);
    const auto acceleration=[&](V3 p,V3 v,Q q){return sub(add(rotate(q,specific),gravity(p)),add(mul(cross(earth,v),2),cross(earth,cross(earth,p))));};
    const V3 p1=position_,v1=velocity_,a1=acceleration(p1,v1,quaternion_);
    const V3 p2=add(p1,mul(v1,.5*h)),v2=add(v1,mul(a1,.5*h)),a2=acceleration(p2,v2,half);
    const V3 p3=add(p1,mul(v2,.5*h)),v3=add(v1,mul(a2,.5*h)),a3=acceleration(p3,v3,half);
    const V3 p4=add(p1,mul(v3,h)),v4=add(v1,mul(a3,h)),a4=acceleration(p4,v4,end);
    const auto identity=track::identity<21>();
    const auto k1=dynamics(p1,quaternion_);
    const auto k2=sparseLeft(dynamics(p2,half),identity+(.5*h)*k1);
    const auto k3=sparseLeft(dynamics(p3,half),identity+(.5*h)*k2);
    const auto k4=sparseLeft(dynamics(p4,end),identity+h*k3);
    const auto phi=identity+(h/6)*(k1+2*k2+2*k3+k4);
    covariance_=sparseLeft(sparseLeft(phi,covariance_),track::transpose(phi));symmetrize(covariance_);
    position_=add(p1,mul(add(add(v1,mul(v2,2)),add(mul(v3,2),v4)),h/6));
    velocity_=add(v1,mul(add(add(a1,mul(a2,2)),add(mul(a3,2),a4)),h/6));quaternion_=end;
}
void NavigationStrapdownV2::propagate(double dt){
    if(dt==0)return;
    const auto count=static_cast<unsigned>(std::max(1.,std::ceil(dt/config_.maximumIntegrationStepS)));
    const double h=dt/count;for(unsigned i=0;i<count;++i)integrate(h);
}
void NavigationStrapdownV2::gnss(const NavigationStrapdownInputV2& s){
    if(!config_.gnssAiding||!s.gnssAvailable)return;
    track::Mat<6,6> innovation,noise;track::Mat<21,6> crossCovariance;
    const double rp=config_.gnssPositionSigmaM*config_.gnssPositionSigmaM,rv=config_.gnssVelocitySigmaMs*config_.gnssVelocitySigmaMs;
    for(unsigned i=0;i<6;++i){noise(i,i)=i<3?rp:rv;for(unsigned j=0;j<6;++j)innovation(i,j)=covariance_(i,j)+noise(i,j);}
    for(unsigned i=0;i<21;++i)for(unsigned j=0;j<6;++j)crossCovariance(i,j)=covariance_(i,j);
    track::Mat<6,6> inverse;if(!track::inverse(innovation,inverse))throw std::runtime_error("Singular strapdown GNSS covariance");
    const auto gain=crossCovariance*inverse;
    const auto p=values(sub(s.position,position_)),v=values(sub(s.velocity,velocity_));
    track::Mat<6,1> residual;
    for(unsigned i=0;i<3;++i){residual(i,0)=p[i]+config_.gnssPositionSigmaM*gnssRng_.normal();residual(3+i,0)=v[i]+config_.gnssVelocitySigmaMs*gnssRng_.normal();}
    const auto correction=gain*residual;
    const auto delta=[&](unsigned offset){return V3{correction(offset,0),correction(offset+1,0),correction(offset+2,0)};};
    position_=add(position_,delta(0));velocity_=add(velocity_,delta(3));
    quaternion_=normalize(product(exponential(delta(6)),quaternion_));
    accelerometerBias_=add(accelerometerBias_,delta(9));gyroBias_=add(gyroBias_,delta(12));
    whiteAcceleration_=add(whiteAcceleration_,delta(15));whiteGyro_=add(whiteGyro_,delta(18));
    auto residualTransition=track::identity<21>();
    for(unsigned i=0;i<21;++i)for(unsigned j=0;j<6;++j)residualTransition(i,j)-=gain(i,j);
    covariance_=residualTransition*covariance_*track::transpose(residualTransition)+gain*noise*track::transpose(gain);
    auto reset=track::identity<21>();const auto jacobian=leftJacobian(delta(6));
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)reset(6+i,6+j)=jacobian(i,j);
    covariance_=reset*covariance_*track::transpose(reset);symmetrize(covariance_);
    lastGnss_=s.timeS;gnssAidingValid_=true;
}
bool NavigationStrapdownV2::finiteState() const noexcept {
    if(!finite(position_)||!finite(velocity_)||!finite(accelerometerBias_)||!finite(gyroBias_)||!finite(whiteAcceleration_)||!finite(whiteGyro_)||
       !finite(actualAccelerometerBias_)||!finite(actualGyroBias_)||!finite(sample_.gyroRadS)||!finite(sample_.specificForceMs2))return false;
    if(!std::all_of(quaternion_.begin(),quaternion_.end(),[](double x){return std::isfinite(x);})||
       !std::all_of(covariance_.a.begin(),covariance_.a.end(),[](double x){return std::isfinite(x);}))return false;
    for(unsigned i=0;i<21;++i)if(covariance_(i,i)<0)return false;
    return true;
}
V3 navigationGravityVerticalV2(V3 p) noexcept {
    constexpr double a=6378137.,f=1./298.257223563,b=a*(1-f),e2=f*(2-f),ep2=(a*a-b*b)/(b*b);
    const double horizontal=std::hypot(p.x,p.y);
    if(!finite(p)||std::hypot(horizontal,p.z)<=1) {
        const double unknown=std::numeric_limits<double>::quiet_NaN();return {unknown,unknown,unknown};
    }
    const double theta=std::atan2(p.z*a,horizontal*b),st=std::sin(theta),ct=std::cos(theta);
    const double latitude=std::atan2(p.z+ep2*b*st*st*st,horizontal-e2*a*ct*ct*ct);
    const double longitude=std::atan2(p.y,p.x),cl=std::cos(latitude);
    return {-9.80665*cl*std::cos(longitude),-9.80665*cl*std::sin(longitude),-9.80665*std::sin(latitude)};
}
double NavigationStrapdownV2::nextSensorTimeS() const noexcept {
    if(!initialized_)return std::numeric_limits<double>::infinity();
    const double imuTime=epoch_+static_cast<double>(nextImu_)*config_.imuPeriodS;
    const double gnssTime=config_.gnssAiding?epoch_+static_cast<double>(nextGnss_)*config_.gnssPeriodS:std::numeric_limits<double>::infinity();
    return std::min(imuTime,gnssTime);
}
bool NavigationCaptureV2::observe(NavigationStrapdownV2& navigation,const NavigationStrapdownInputV2& s,bool controlRead) {
    if(!initialized_||!NavigationStrapdownV2::valid(s)||s.timeS<previous_.timeS)return false;
    const double span=s.timeS-previous_.timeS;
    for(unsigned events=0;navigation.nextSensorTimeS()<=s.timeS;++events) {
        if(events>=200002||!(span>0))return false;
        const double event=navigation.nextSensorTimeS();
        if(event<previous_.timeS)return false;
        NavigationStrapdownInputV2 capture;capture.timeS=event;
        const double w=(event-previous_.timeS)/span;
        capture.position=interpolate(previous_.position,s.position,w);capture.velocity=interpolate(previous_.velocity,s.velocity,w);
        capture.acceleration=interpolate(previous_.acceleration,s.acceleration,w);
        capture.angularRateBodyRelativeEcefRadS=interpolate(previous_.angularRateBodyRelativeEcefRadS,s.angularRateBodyRelativeEcefRadS,w);
        capture.bodyToEcef=axes(slerp(fromAxes(previous_.bodyToEcef),fromAxes(s.bodyToEcef),w));
        capture.gnssAvailable=previous_.gnssAvailable&&s.gnssAvailable;
        if(!navigation.advance(capture))return false;
    }
    if(controlRead&&!navigation.advance(s))return false;
    previous_=s;return true;
}
bool NavigationStrapdownV2::advance(const NavigationStrapdownInputV2& s){
    if(!initialized_||!valid(s)||s.timeS<time_)return false;
    if(s.timeS==time_)return true;
    const double dt=s.timeS-time_;
    if(!std::isfinite(dt)||dt/config_.imuPeriodS>100000||dt/config_.maximumIntegrationStepS>1000000||
       (config_.gnssAiding&&dt/config_.gnssPeriodS>100000))return false;
    auto next=*this;
    try{if(!next.advanceImpl(s))return false;}catch(const std::exception&){return false;}
    *this=next;return true;
}
bool NavigationStrapdownV2::advanceImpl(const NavigationStrapdownInputV2& s){
    if(!s.gnssAvailable)gnssAidingValid_=false;
    const double span=s.timeS-previous_.timeS;
    for(unsigned events=0;events<200002;++events){
        const double ti=epoch_+static_cast<double>(nextImu_)*config_.imuPeriodS;
        const double tg=config_.gnssAiding?epoch_+static_cast<double>(nextGnss_)*config_.gnssPeriodS:std::numeric_limits<double>::infinity();
        const double event=std::min(ti,tg);if(event>s.timeS)break;
        if(!std::isfinite(event)||event<time_||!(ti>epoch_+static_cast<double>(nextImu_-1)*config_.imuPeriodS)||
           (config_.gnssAiding&&!(tg>epoch_+static_cast<double>(nextGnss_-1)*config_.gnssPeriodS)))return false;
        propagate(event-time_);time_=event;
        NavigationStrapdownInputV2 capture;capture.timeS=event;const double w=std::clamp((event-previous_.timeS)/span,0.,1.);
        capture.position=interpolate(previous_.position,s.position,w);capture.velocity=interpolate(previous_.velocity,s.velocity,w);
        capture.acceleration=interpolate(previous_.acceleration,s.acceleration,w);
        capture.angularRateBodyRelativeEcefRadS=interpolate(previous_.angularRateBodyRelativeEcefRadS,s.angularRateBodyRelativeEcefRadS,w);
        capture.bodyToEcef=axes(slerp(fromAxes(previous_.bodyToEcef),fromAxes(s.bodyToEcef),w));
        capture.gnssAvailable=previous_.gnssAvailable&&s.gnssAvailable;
        if(event==ti){imu(capture,false);++nextImu_;}
        if(event==tg){gnss(capture);++nextGnss_;}
    }
    propagate(s.timeS-time_);time_=s.timeS;previous_=s;return finiteState();
}
NavigationEstimateV1 NavigationStrapdownV2::estimate() const noexcept {
    NavigationEstimateV1 out;out.valid=initialized_;out.timeS=time_;out.lastImuSampleS=lastImu_;out.lastGnssSampleS=lastGnss_;
    out.position=position_;out.velocity=velocity_;
    out.gnssAided=initialized_&&config_.gnssAiding&&gnssAidingValid_&&previous_.gnssAvailable&&std::isfinite(lastGnss_)&&time_-lastGnss_<=config_.gnssPeriodS;
    for(unsigned axis=0;axis<3;++axis)for(unsigned other=0;other<3;++other)for(unsigned i=0;i<2;++i)for(unsigned j=0;j<2;++j)
        out.covariance[6*(2*axis+i)+2*other+j]=covariance_(axis+3*i,other+3*j);
    return out;
}
}
