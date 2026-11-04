#include "simulation/mission_systems/weapon/NavigationV1.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
using namespace fsim::ms::weapon;
NavigationConfigV1 noiseless() {
    NavigationConfigV1 c;
    c.accelerometerNoiseDensity=c.biasSigmaMs2=c.biasWalkDensity=0;
    c.initialPositionSigmaM=c.initialVelocitySigmaMs=0;
    return c;
}
NavigationInputV1 state(double time) {
    NavigationInputV1 s;s.timeS=time;s.position={10*time+.5*time*time,0,0};
    s.velocity={10+time,0,0};s.acceleration={1,0,0};return s;
}
}

TEST_CASE("Weapon inertial navigation cannot reacquire true position without fitted aiding", "[weapon-navigation]") {
    NavigationV1 nav(noiseless(),17);
    REQUIRE(nav.initialize(state(0)));
    auto next=state(1);next.position={99999,88888,77777};next.velocity={1234,2345,3456};
    REQUIRE(nav.advance(next));const auto estimate=nav.estimate();REQUIRE(estimate.valid);
    CHECK(estimate.position.x==Catch::Approx(10.5).margin(1e-9));
    CHECK(estimate.velocity.x==Catch::Approx(11).margin(1e-9));
    CHECK(estimate.position.y==0);CHECK_FALSE(estimate.gnssAided);
}

TEST_CASE("Weapon inertial covariance matches exact sampled acceleration integration", "[weapon-navigation]") {
    auto config=noiseless();config.accelerometerNoiseDensity=.2;
    NavigationV1 nav(config,17);REQUIRE(nav.initialize(state(0)));REQUIRE(nav.advance(state(1)));
    const auto estimate=nav.estimate();
    for(std::size_t axis=0;axis<3;++axis) {
        const auto p=2*axis,v=p+1;
        // Independent sum of squared interval impulse weights:
        // qa/h * sum((h*(T-(k+.5)*h))^2), T=1.
        CHECK(estimate.covariance[6*p+p]==Catch::Approx(.04/3-.04*.01*.01/12).margin(1e-12));
        CHECK(estimate.covariance[6*p+v]==Catch::Approx(.02).margin(1e-12));
        CHECK(estimate.covariance[6*v+p]==Catch::Approx(.02).margin(1e-12));
        CHECK(estimate.covariance[6*v+v]==Catch::Approx(.04).margin(1e-12));
    }
}

TEST_CASE("Weapon navigation invalid and repeated epochs preserve the accepted stream", "[weapon-navigation]") {
    NavigationConfigV1 c;NavigationV1 first(c,41),second(c,41);
    REQUIRE(first.initialize(state(0)));REQUIRE(second.initialize(state(0)));
    REQUIRE(first.advance(state(.5)));REQUIRE(second.advance(state(.5)));
    auto invalid=state(1);invalid.acceleration.x=std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(first.advance(invalid));CHECK_FALSE(first.advance(state(.1)));
    auto repeated=state(.5);repeated.position={1e9,1e9,1e9};
    REQUIRE(first.advance(repeated));
    REQUIRE(first.advance(state(1)));REQUIRE(second.advance(state(1)));
    const auto a=first.estimate(),b=second.estimate();
    CHECK(a.position.x==b.position.x);CHECK(a.position.y==b.position.y);CHECK(a.position.z==b.position.z);
    CHECK(a.velocity.x==b.velocity.x);CHECK(a.covariance==b.covariance);
    first.reset();CHECK_FALSE(first.estimate().valid);
    REQUIRE(first.initialize(state(0)));REQUIRE(first.advance(state(1)));
    NavigationV1 clean(c,41);REQUIRE(clean.initialize(state(0)));REQUIRE(clean.advance(state(1)));
    CHECK(first.estimate().position.x==clean.estimate().position.x);
}

TEST_CASE("Weapon GNSS posterior agrees with an independent information-form update", "[weapon-navigation]") {
    auto c=noiseless();c.gnssAiding=true;c.initialPositionSigmaM=5;c.initialVelocitySigmaMs=.5;
    NavigationV1 nav(c,37);REQUIRE(nav.initialize(state(0)));REQUIRE(nav.advance(state(1)));
    const auto e=nav.estimate();REQUIRE(e.gnssAided);CHECK(e.lastGnssSampleS==1);
    // Prior F diag(25,.25) F' = [[25.25,.25],[.25,.25]].
    // Posterior = inverse(inverse(prior) + diag(1/9,100)).
    const double informationP=.25/6.25+1./9,informationV=25.25/6.25+100,informationCross=-.25/6.25;
    const double determinant=informationP*informationV-informationCross*informationCross;
    for(std::size_t axis=0;axis<3;++axis) {
        const auto p=2*axis,v=p+1;
        CHECK(e.covariance[6*p+p]==Catch::Approx(informationV/determinant).margin(1e-10));
        CHECK(e.covariance[6*p+v]==Catch::Approx(-informationCross/determinant).margin(1e-10));
        CHECK(e.covariance[6*v+v]==Catch::Approx(informationP/determinant).margin(1e-10));
    }
}

TEST_CASE("Weapon inertial bias covariance agrees with independent sampled matrix exponential", "[weapon-navigation]") {
    NavigationConfigV1 c;NavigationV1 nav(c,11);
    REQUIRE(nav.initialize(state(0)));REQUIRE(nav.advance(state(2.3)));
    const auto e=nav.estimate();
    // SciPy expm(A*h), A[0,1]=1, A[1,2]=A[1,3]=-1,
    // followed by white-state replacement and bias covariance refresh.
    // tools/validation/weapon_quality/navigation_reference.py also retains
    // the separate continuous Van Loan limit; it is not the sampled oracle.
    for(std::size_t axis=0;axis<3;++axis) {
        const auto p=2*axis,v=p+1;
        CHECK(e.covariance[6*p+p]==Catch::Approx(26.32684971708301).margin(1e-10));
        CHECK(e.covariance[6*p+v]==Catch::Approx(.5779888846766053).margin(1e-11));
        CHECK(e.covariance[6*v+v]==Catch::Approx(.25259904029254554).margin(1e-11));
    }
}

TEST_CASE("Weapon navigation retains held noise covariance through fractional IMU and GNSS epochs", "[weapon-navigation]") {
    auto c=noiseless();c.imuPeriodS=1;c.accelerometerNoiseDensity=.2;
    c.gnssAiding=true;c.gnssPeriodS=.5;
    NavigationV1 split(c,41),whole(c,41);
    REQUIRE(split.initialize(state(0)));REQUIRE(whole.initialize(state(0)));
    REQUIRE(split.advance(state(.25)));const auto quarter=split.estimate();
    CHECK(quarter.covariance[0]==Catch::Approx(.04*std::pow(.25,4)/4).margin(1e-14));
    CHECK(quarter.covariance[1]==Catch::Approx(.04*std::pow(.25,3)/2).margin(1e-14));
    CHECK(quarter.covariance[7]==Catch::Approx(.04*.25*.25).margin(1e-14));
    REQUIRE(split.advance(state(.5)));REQUIRE(split.advance(state(.75)));
    REQUIRE(whole.advance(state(.75)));
    const auto a=split.estimate(),b=whole.estimate();
    CHECK(a.position.x==Catch::Approx(b.position.x).margin(1e-12));
    for(std::size_t k=0;k<a.covariance.size();++k)CHECK(a.covariance[k]==Catch::Approx(b.covariance[k]).margin(1e-13));
    // A single held acceleration error w has prior variance .04. Position
    // and velocity GNSS at t=.5 independently observe .125*w and .5*w.
    const double posterior=1/(1/.04+(.125*.125)/9+(.5*.5)/.01);
    const double positionWeight=.5*.75*.75;
    CHECK(a.covariance[0]==Catch::Approx(positionWeight*positionWeight*posterior).margin(1e-13));
    CHECK(a.covariance[7]==Catch::Approx(.75*.75*posterior).margin(1e-13));
}

TEST_CASE("Weapon navigation realized fractional exposure errors match their reported covariance", "[weapon-navigation]") {
    auto c=noiseless();c.imuPeriodS=1;c.accelerometerNoiseDensity=.2;c.gnssAiding=true;c.gnssPeriodS=.5;
    double squaredPosition=0,squaredVelocity=0;constexpr unsigned seeds=4096;
    for(unsigned seed=0;seed<seeds;++seed) {
        NavigationV1 nav(c,seed);REQUIRE(nav.initialize(state(0)));REQUIRE(nav.advance(state(.75)));
        const auto e=nav.estimate();const auto truth=state(.75);
        squaredPosition+=std::pow(e.position.x-truth.position.x,2)/e.covariance[0];
        squaredVelocity+=std::pow(e.velocity.x-truth.velocity.x,2)/e.covariance[7];
    }
    CHECK(squaredPosition/seeds==Catch::Approx(1).margin(.08));
    CHECK(squaredVelocity/seeds==Catch::Approx(1).margin(.08));
}

TEST_CASE("Weapon GNSS outage preserves measurement age and reacquisition reduces uncertainty", "[weapon-navigation]") {
    NavigationConfigV1 c;c.gnssAiding=true;NavigationV1 nav(c,17);
    REQUIRE(nav.initialize(state(0)));REQUIRE(nav.advance(state(1)));REQUIRE(nav.estimate().gnssAided);
    for(int i=2;i<=20;++i) {auto input=state(i);input.gnssAvailable=false;REQUIRE(nav.advance(input));}
    const auto outage=nav.estimate();CHECK_FALSE(outage.gnssAided);CHECK(outage.lastGnssSampleS==1);
    REQUIRE(nav.advance(state(21)));REQUIRE(nav.advance(state(22)));
    const auto aided=nav.estimate();CHECK(aided.gnssAided);CHECK(aided.lastGnssSampleS==22);
    CHECK(aided.covariance[0]<outage.covariance[0]);
    for(std::size_t axis=0;axis<3;++axis) {
        const auto p=2*axis,v=p+1;
        CHECK(aided.covariance[6*p+p]>=0);CHECK(aided.covariance[6*v+v]>=0);
        CHECK(aided.covariance[6*p+p]*aided.covariance[6*v+v]>=
              aided.covariance[6*p+v]*aided.covariance[6*v+p]-1e-12);
    }
}

TEST_CASE("Fitted weapon aiding improves a frozen independent-seed drift population", "[weapon-navigation]") {
    NavigationConfigV1 inertial;inertial.biasSigmaMs2=.05;
    auto aided=inertial;aided.gnssAiding=true;
    double unaidedSquared=0,aidedSquared=0;
    for(std::uint64_t seed=0;seed<32;++seed) {
        NavigationV1 first(inertial,seed),second(aided,seed);
        REQUIRE(first.initialize(state(0)));REQUIRE(second.initialize(state(0)));
        for(int sample=1;sample<=600;++sample) {
            auto input=state(static_cast<double>(sample)/10);
            REQUIRE(first.advance(input));REQUIRE(second.advance(input));
        }
        const auto truth=state(60).position.x;
        const auto a=first.estimate().position.x-truth,b=second.estimate().position.x-truth;
        unaidedSquared+=a*a;aidedSquared+=b*b;
    }
    INFO("Synthetic RMS inertial/aided: "<<std::sqrt(unaidedSquared/32)<<" / "<<std::sqrt(aidedSquared/32));
    CHECK(aidedSquared<.04*unaidedSquared);
}

TEST_CASE("Weapon navigation rejects malformed policies and overflowing propagation atomically", "[weapon-navigation]") {
    NavigationConfigV1 c;c.imuPeriodS=0;CHECK_THROWS_AS(NavigationV1(c,0),std::invalid_argument);
    c=NavigationConfigV1{};c.gnssAiding=true;c.gnssPositionSigmaM=0;
    CHECK_THROWS_AS(NavigationV1(c,0),std::invalid_argument);
    c=NavigationConfigV1{};NavigationV1 nav(c,7);REQUIRE(nav.initialize(state(0)));
    const auto accepted=nav.estimate();auto huge=state(100);huge.acceleration.x=std::numeric_limits<double>::max();
    CHECK_FALSE(nav.advance(huge));CHECK(nav.estimate().timeS==accepted.timeS);
    CHECK(nav.estimate().covariance==accepted.covariance);
    auto excessive=state(2000);CHECK_FALSE(nav.advance(excessive));CHECK(nav.estimate().timeS==accepted.timeS);
}

TEST_CASE("Weapon GNSS short outage requires a new exposure before aided status returns", "[weapon-navigation][weapon-navigation-short-outage]") {
    NavigationConfigV1 config;config.gnssAiding=true;NavigationV1 nav(config,113);
    REQUIRE(nav.initialize(state(0)));REQUIRE(nav.advance(state(1)));
    REQUIRE(nav.estimate().gnssAided);REQUIRE(nav.estimate().lastGnssSampleS==1);
    auto outage=state(1.2);outage.gnssAvailable=false;REQUIRE(nav.advance(outage));
    const auto accepted=nav.estimate();CHECK_FALSE(accepted.gnssAided);CHECK(accepted.lastGnssSampleS==1);
    auto retry=state(1.2);retry.gnssAvailable=true;REQUIRE(nav.advance(retry));
    const auto repeated=nav.estimate();CHECK_FALSE(repeated.gnssAided);CHECK(repeated.lastGnssSampleS==1);
    CHECK(repeated.position.x==accepted.position.x);CHECK(repeated.velocity.x==accepted.velocity.x);
    CHECK(repeated.covariance==accepted.covariance);
    REQUIRE(nav.advance(state(1.3)));CHECK_FALSE(nav.estimate().gnssAided);CHECK(nav.estimate().lastGnssSampleS==1);
    REQUIRE(nav.advance(state(1.9)));CHECK_FALSE(nav.estimate().gnssAided);CHECK(nav.estimate().lastGnssSampleS==1);
    REQUIRE(nav.advance(state(2)));CHECK(nav.estimate().gnssAided);CHECK(nav.estimate().lastGnssSampleS==2);
    nav.reset();CHECK_FALSE(nav.estimate().gnssAided);CHECK(std::isnan(nav.estimate().lastGnssSampleS));
    REQUIRE(nav.initialize(state(0)));CHECK_FALSE(nav.estimate().gnssAided);
}
