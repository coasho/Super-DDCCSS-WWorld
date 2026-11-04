#include "simulation/mission_systems/weapon/NavigationStrapdownV2.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
using namespace fsim::ms::weapon;
NavigationStrapdownConfigV2 ideal() {
    NavigationStrapdownConfigV2 c;
    c.accelerometerNoiseDensity=c.gyroNoiseDensity=0;
    c.accelerometerBiasSigma=c.gyroBiasSigma=0;
    c.accelerometerBiasWalkDensity=c.gyroBiasWalkDensity=0;
    c.initialPositionSigmaM=c.initialVelocitySigmaMs=c.initialAttitudeSigmaRad=0;
    return c;
}
NavigationStrapdownInputV2 stationary(double time) {
    NavigationStrapdownInputV2 s;s.timeS=time;s.position={6378137,0,0};return s;
}
double distance(V3 a,V3 b) {
    return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z);
}
NavigationStrapdownInputV2 turning(double time) {
    auto s=stationary(time);const double angle=2*time;
    s.bodyToEcef={V3{std::cos(angle),std::sin(angle),0},V3{-std::sin(angle),std::cos(angle),0},V3{0,0,1}};
    s.angularRateBodyRelativeEcefRadS={0,0,2};
    s.position={6378137+1.25*(1-std::cos(angle)),2.5*time-1.25*std::sin(angle),0};
    s.velocity={2.5*std::sin(angle),2.5*(1-std::cos(angle)),0};
    s.acceleration={5*std::cos(angle),5*std::sin(angle),0};return s;
}
}

TEST_CASE("Strapdown stationary Earth mechanization cancels gravity and Earth rate", "[weapon-strapdown]") {
    NavigationStrapdownV2 nav(ideal(),11);REQUIRE(nav.initialize(stationary(0)));
    REQUIRE(nav.advance(stationary(5)));const auto e=nav.estimate();
    CHECK(distance(e.position,stationary(5).position)<1e-7);CHECK(distance(e.velocity,{})<1e-8);
    const auto q=nav.attitudeBodyToEcef();CHECK(q[0]==Catch::Approx(1).margin(1e-12));
    CHECK(std::abs(q[1])+std::abs(q[2])+std::abs(q[3])<1e-12);
    const auto imu=nav.imuSample();CHECK(imu.valid);CHECK(imu.gyroRadS.z==Catch::Approx(7.292115e-5));
    CHECK(imu.specificForceMs2.x>9.7);CHECK(imu.specificForceMs2.x<9.9);
}

TEST_CASE("Strapdown body force rotates through measured attitude rather than truth correction", "[weapon-strapdown]") {
    auto c=ideal();c.earthRateRadS=0;c.earthMuM3S2=0;
    NavigationStrapdownV2 nav(c,17);REQUIRE(nav.initialize(turning(0)));
    for(unsigned i=1;i<=200;++i)REQUIRE(nav.advance(turning(i*.01)));
    const auto e=nav.estimate();const auto t=turning(2);CHECK(distance(e.position,t.position)<2e-7);
    CHECK(distance(e.velocity,t.velocity)<2e-8);
    const auto q=nav.attitudeBodyToEcef();CHECK(std::abs(q[0])==Catch::Approx(std::abs(std::cos(2))).margin(1e-10));
    CHECK(std::abs(q[3])==Catch::Approx(std::abs(std::sin(2))).margin(1e-10));
}

TEST_CASE("Strapdown unaided native own position and orientation are sensor physics only", "[weapon-strapdown]") {
    auto c=ideal();c.earthRateRadS=0;c.earthMuM3S2=0;
    NavigationStrapdownV2 nav(c,21);REQUIRE(nav.initialize(stationary(0)));
    auto s=stationary(1);s.position={9e7,8e7,7e7};s.velocity={100,200,300};
    s.bodyToEcef={V3{0,1,0},V3{-1,0,0},V3{0,0,1}};
    REQUIRE(nav.advance(s));CHECK(distance(nav.estimate().position,stationary(0).position)<1e-9);
    CHECK(distance(nav.estimate().velocity,{})<1e-9);
    CHECK(nav.attitudeBodyToEcef()[0]==Catch::Approx(1).margin(1e-12));
}

TEST_CASE("Strapdown attitude uncertainty couples into velocity and position covariance", "[weapon-strapdown]") {
    auto c=ideal();c.earthRateRadS=0;c.earthMuM3S2=0;c.initialAttitudeSigmaRad=.01;
    NavigationStrapdownV2 nav(c,23);auto s=stationary(0);s.acceleration={5,0,0};REQUIRE(nav.initialize(s));
    s.timeS=.1;REQUIRE(nav.advance(s));const auto p=nav.errorCovariance();
    // Independent small-angle oracle: dv_y = 5 * theta_z * T, dp_y = 2.5 * theta_z * T^2.
    CHECK(p[21*4+4]==Catch::Approx(.000025).epsilon(.02));
    CHECK(std::abs(p[21*4+8])>1e-5);CHECK(p[21*1+1]>1e-8);
}

TEST_CASE("Strapdown sampled acceleration covariance preserves fractional exposure correlations", "[weapon-strapdown]") {
    auto c=ideal();c.earthRateRadS=0;c.earthMuM3S2=0;c.accelerometerNoiseDensity=.2;
    NavigationStrapdownV2 nav(c,29);REQUIRE(nav.initialize(stationary(0)));REQUIRE(nav.advance(stationary(.025)));
    const auto p=nav.errorCovariance();const double q=.04/.01;
    const double vv=q*(.01*.01+.01*.01+.005*.005);
    const double pp=q*(.0002*.0002+.0001*.0001+.0000125*.0000125);
    CHECK(p[21*3+3]==Catch::Approx(vv).margin(1e-13));
    CHECK(p[0]==Catch::Approx(pp).margin(1e-13));
    CHECK(p[21*3+15]==Catch::Approx(-.005*q).margin(1e-13));
}

TEST_CASE("Strapdown GNSS aiding genuinely corrects the coupled state and supports outages", "[weapon-strapdown]") {
    NavigationStrapdownConfigV2 c;c.gnssAiding=true;c.gnssPeriodS=.2;
    NavigationStrapdownV2 aided(c,31);c.gnssAiding=false;NavigationStrapdownV2 unaided(c,31);
    REQUIRE(aided.initialize(stationary(0)));REQUIRE(unaided.initialize(stationary(0)));
    for(unsigned i=1;i<=500;++i){auto s=stationary(i*.01);REQUIRE(aided.advance(s));REQUIRE(unaided.advance(s));}
    CHECK(aided.estimate().gnssAided);CHECK(aided.errorCovariance()[0]<unaided.errorCovariance()[0]);
    auto s=stationary(5.1);s.gnssAvailable=false;REQUIRE(aided.advance(s));CHECK_FALSE(aided.estimate().gnssAided);
    const double last=aided.estimate().lastGnssSampleS;s.timeS=5.9;REQUIRE(aided.advance(s));
    CHECK(aided.estimate().lastGnssSampleS==last);s.timeS=6;s.gnssAvailable=true;REQUIRE(aided.advance(s));
    CHECK_FALSE(aided.estimate().gnssAided);s.timeS=6.21;REQUIRE(aided.advance(s));CHECK(aided.estimate().gnssAided);
    const auto p=aided.errorCovariance();for(unsigned i=0;i<21;++i){CHECK(p[21*i+i]>=0);for(unsigned j=0;j<21;++j)CHECK(p[21*i+j]==Catch::Approx(p[21*j+i]).margin(1e-10));}
}

TEST_CASE("Strapdown reset retries and rejected captures preserve independent sensor streams", "[weapon-strapdown]") {
    NavigationStrapdownConfigV2 c;NavigationStrapdownV2 a(c,41),b(c,41);
    REQUIRE(a.initialize(stationary(0)));REQUIRE(b.initialize(stationary(0)));
    REQUIRE(a.advance(stationary(.025)));REQUIRE(b.advance(stationary(.025)));
    auto invalid=stationary(.03);invalid.bodyToEcef[0]={2,0,0};CHECK_FALSE(a.advance(invalid));
    invalid=stationary(.03);invalid.angularRateBodyRelativeEcefRadS.x=std::numeric_limits<double>::quiet_NaN();CHECK_FALSE(a.advance(invalid));
    CHECK_FALSE(a.advance(stationary(.01)));REQUIRE(a.advance(stationary(.025)));
    REQUIRE(a.advance(stationary(.1)));REQUIRE(b.advance(stationary(.1)));
    CHECK(a.attitudeBodyToEcef()==b.attitudeBodyToEcef());CHECK(a.errorCovariance()==b.errorCovariance());
    CHECK(distance(a.estimate().position,b.estimate().position)==0);
    a.reset();CHECK_FALSE(a.estimate().valid);REQUIRE(a.initialize(stationary(0)));REQUIRE(a.advance(stationary(.1)));
    NavigationStrapdownV2 clean(c,41);REQUIRE(clean.initialize(stationary(0)));REQUIRE(clean.advance(stationary(.1)));
    for(unsigned i=0;i<4;++i)CHECK(a.attitudeBodyToEcef()[i]==Catch::Approx(clean.attitudeBodyToEcef()[i]).margin(1e-12));
    for(unsigned i=0;i<441;++i)CHECK(a.errorCovariance()[i]==Catch::Approx(clean.errorCovariance()[i]).margin(1e-10));
}

TEST_CASE("Strapdown mid-exposure aiding preserves the sampled white-error posterior", "[weapon-strapdown]") {
    auto c=ideal();c.earthRateRadS=0;c.earthMuM3S2=0;c.accelerometerNoiseDensity=.2;
    c.gnssAiding=true;c.gnssPeriodS=.015;c.gnssPositionSigmaM=.003;c.gnssVelocitySigmaMs=.001;
    NavigationStrapdownV2 nav(c,43);REQUIRE(nav.initialize(stationary(0)));REQUIRE(nav.advance(stationary(.025)));
    const auto p=nav.errorCovariance();
    // Independent SciPy expm + 3-state Joseph oracle in navigation_strapdown_reference.py.
    CHECK(p[0]==Catch::Approx(1.0241984001252432e-9).margin(1e-17));
    CHECK(p[3]==Catch::Approx(2.739632089763676e-7).margin(1e-15));
    CHECK(p[21*3+3]==Catch::Approx(.00018139849114686565).margin(1e-13));
    CHECK(p[21*3+15]==Catch::Approx(-.02).margin(1e-12));
    CHECK(p[15]==Catch::Approx(-.00005).margin(1e-14));
    CHECK(nav.estimate().gnssAided);CHECK(nav.estimate().lastGnssSampleS==.015);
}

TEST_CASE("Strapdown GNSS posterior agrees with an independent information update", "[weapon-strapdown]") {
    auto c=ideal();c.earthRateRadS=0;c.earthMuM3S2=0;c.gnssAiding=true;
    c.initialPositionSigmaM=5;c.initialVelocitySigmaMs=.5;
    NavigationStrapdownV2 nav(c,47);REQUIRE(nav.initialize(stationary(0)));REQUIRE(nav.advance(stationary(1)));
    const double ipp=.25/6.25+1./9,ivv=25.25/6.25+100,ipv=-.25/6.25;
    const double determinant=ipp*ivv-ipv*ipv;const auto p=nav.estimate().covariance;
    for(unsigned axis=0;axis<3;++axis){
        const unsigned i=2*axis,j=i+1;
        CHECK(p[6*i+i]==Catch::Approx(ivv/determinant).margin(1e-11));
        CHECK(p[6*i+j]==Catch::Approx(-ipv/determinant).margin(1e-11));
        CHECK(p[6*j+j]==Catch::Approx(ipp/determinant).margin(1e-11));
    }
}

TEST_CASE("Strapdown noncommuting body rotations retain their physical order", "[weapon-strapdown]") {
    auto c=ideal();c.earthRateRadS=0;c.earthMuM3S2=0;
    auto s=stationary(0);s.angularRateBodyRelativeEcefRadS={std::acos(-1.)/2,0,0};
    NavigationStrapdownV2 nav(c,53);REQUIRE(nav.initialize(s));
    for(unsigned i=1;i<=200;++i){
        const double time=i*.01,a=std::min(time,1.)*std::acos(-1.)/2,b=std::max(time-1.,0.)*std::acos(-1.)/2;
        s.timeS=time;s.angularRateBodyRelativeEcefRadS=time<1?V3{std::acos(-1.)/2,0,0}:V3{0,std::acos(-1.)/2,0};
        // Independent Rx(a) Ry(b), expressed by columns.
        s.bodyToEcef={V3{std::cos(b),std::sin(a)*std::sin(b),-std::cos(a)*std::sin(b)},
            V3{0,std::cos(a),std::sin(a)},V3{std::sin(b),-std::sin(a)*std::cos(b),std::cos(a)*std::cos(b)}};
        REQUIRE(nav.advance(s));
    }
    const auto q=nav.attitudeBodyToEcef();for(double x:q)CHECK(x==Catch::Approx(.5).margin(1e-11));
}

TEST_CASE("Strapdown rejects nonphysical policies and unrepresentable clocks", "[weapon-strapdown]") {
    auto c=ideal();c.gyroNoiseDensity=-1;CHECK_THROWS_AS(NavigationStrapdownV2(c,59),std::invalid_argument);
    c=ideal();c.gnssAiding=true;c.gnssPositionSigmaM=0;CHECK_THROWS_AS(NavigationStrapdownV2(c,59),std::invalid_argument);
    NavigationStrapdownV2 nav(ideal(),59);auto s=stationary(1e100);CHECK_FALSE(nav.initialize(s));
    s=stationary(0);s.bodyToEcef[2]={0,0,-1};CHECK_FALSE(nav.initialize(s));
    REQUIRE(nav.initialize(stationary(0)));const auto before=nav.errorCovariance();
    s=stationary(1);s.acceleration.x=1e308;CHECK_FALSE(nav.advance(s));CHECK(nav.errorCovariance()==before);
}

TEST_CASE("Strapdown Earth rotation is resolved through tilted body axes", "[weapon-strapdown]") {
    auto s=stationary(0);s.bodyToEcef={V3{1,0,0},V3{0,0,1},V3{0,-1,0}};
    NavigationStrapdownV2 nav(ideal(),61);REQUIRE(nav.initialize(s));const auto original=nav.attitudeBodyToEcef();
    const auto imu=nav.imuSample();CHECK(imu.gyroRadS.y==Catch::Approx(7.292115e-5));
    CHECK(std::abs(imu.gyroRadS.x)+std::abs(imu.gyroRadS.z)<1e-15);
    s.timeS=2;REQUIRE(nav.advance(s));for(unsigned i=0;i<4;++i)CHECK(nav.attitudeBodyToEcef()[i]==Catch::Approx(original[i]).margin(1e-12));
    CHECK(distance(nav.estimate().velocity,{})<1e-8);
}

TEST_CASE("Strapdown noisy body gyros create attitude error and coupled inertial drift", "[weapon-strapdown]") {
    auto c=ideal();c.earthRateRadS=0;c.earthMuM3S2=0;
    c.gyroNoiseDensity=.002;c.gyroBiasSigma=.003;c.gyroBiasWalkDensity=.0001;
    NavigationStrapdownV2 nav(c,67);auto s=stationary(0);s.acceleration={5,0,0};REQUIRE(nav.initialize(s));
    s.timeS=.2;s.position.x+=.1;s.velocity.x=1;REQUIRE(nav.advance(s));
    const auto q=nav.attitudeBodyToEcef();CHECK(std::hypot(std::hypot(q[1],q[2]),q[3])>1e-5);
    CHECK(std::hypot(nav.estimate().velocity.y,nav.estimate().velocity.z)>1e-6);
    const auto p=nav.errorCovariance();double trace=0;for(unsigned i=6;i<9;++i)trace+=p[21*i+i];
    double walks=0;for(unsigned k=1;k<20;++k)walks+=k*k;
    const double reference=3*(.002*.002*.2+.003*.003*.2*.2+.0001*.0001*.01*.01*.01*walks);
    CHECK(trace==Catch::Approx(reference).epsilon(.001));
    CHECK(p[21*4+4]>0);CHECK(p[21*4+8]!=0);
}

TEST_CASE("Strapdown GNSS innovations correct observable attitude errors", "[weapon-strapdown]") {
    auto c=ideal();c.earthRateRadS=0;c.earthMuM3S2=0;c.gnssAiding=true;
    c.initialAttitudeSigmaRad=.03;c.gnssPeriodS=.1;c.gnssPositionSigmaM=.01;c.gnssVelocitySigmaMs=.001;
    NavigationStrapdownV2 nav(c,71);REQUIRE(nav.initialize(turning(0)));
    const auto initial=nav.attitudeBodyToEcef();const double initialError=2*std::acos(std::min(1.,std::abs(initial[0])));
    for(unsigned i=1;i<=500;++i)REQUIRE(nav.advance(turning(i*.01)));
    const auto q=nav.attitudeBodyToEcef();const double scalar=std::abs(q[0]*std::cos(5)+q[3]*std::sin(5));
    const double finalError=2*std::acos(std::min(1.,scalar));CHECK(finalError<initialError*.2);
    const auto p=nav.errorCovariance();CHECK(p[21*6+6]+p[21*7+7]+p[21*8+8]<3*.03*.03);
}
