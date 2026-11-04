#include "simulation/flight/PassiveRolleronV1.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <limits>

namespace {
using namespace fsim::sim;
RolleronConfigV1 hardware() {
    RolleronConfigV1 c;c.finId=17;c.finIndex=0;c.nativeDeflectionSign=1;
    c.centreFromOriginBodyM={-1,.2,0};c.wheelRadiusM=.04;c.captureAreaM2=.0008;
    c.wheelInertiaKgM2=2e-5;c.wheelDriveCoefficient=.2;c.freeSpinRatio=.6;
    c.wheelDragCoefficient=.08;c.bearingDampingNms=2e-5;
    c.hingeInertiaKgM2=1e-4;c.hingeStiffnessNmRad=.04;c.hingeDampingNmsRad=.02;
    c.hingeAerodynamicStiffnessM3=1e-6;c.limitRad=.2;c.rateRadS=10;
    return c;
}
RolleronInputV1 wind(double dt=.001) {
    RolleronInputV1 in;in.dtS=dt;in.airVelocityBodyMs={100,0,0};in.densityKgM3=1;
    return in;
}
}

TEST_CASE("Passive rolleron wheel spin follows its independent air torque balance", "[passive-rolleron]") {
    const auto c=hardware();PassiveRolleronV1 roller({c});const auto in=wind();
    RolleronOutputV1 result;
    for(unsigned i=0;i<1000;++i){result=roller.advance(in);REQUIRE(result.valid);}
    REQUIRE(result.fins.size()==1);
    const double q=.5*in.densityKgM3*100*100;
    const double torque=c.wheelDriveCoefficient*q*c.captureAreaM2*c.wheelRadiusM;
    const double damping=torque*c.wheelRadiusM/(c.freeSpinRatio*100)+c.bearingDampingNms;
    const double expected=torque/damping*(1-std::exp(-damping/c.wheelInertiaKgM2));
    CHECK(result.fins[0].wheelRadS==Catch::Approx(expected).epsilon(1e-8));
    CHECK(result.fins[0].angleRad==0);CHECK(result.fins[0].rateRadS==0);
    CHECK(result.dragPowerW>=result.harvestedAirPowerW);
    CHECK(result.forceBodyN.x<0);CHECK(result.forceBodyN.y==0);CHECK(result.forceBodyN.z==0);
}

TEST_CASE("Passive gyro moves an actual hinge and returns the equal coupling impulse", "[passive-rolleron]") {
    auto c=hardware();c.bearingDampingNms=0;PassiveRolleronV1 roller({c});auto in=wind(.0005);
    for(unsigned i=0;i<5000;++i)REQUIRE(roller.advance(in).valid);
    in.angularRateInertialBodyRadS={.1,0,0};
    double gyroWork=0,hingeWork=0;RolleronOutputV1 result;
    for(unsigned i=0;i<200;++i) {
        result=roller.advance(in);REQUIRE(result.valid);
        gyroWork+=result.gyroBodyWorkJ;hingeWork+=result.gyroHingeWorkJ;
        CHECK(result.stopLossJ>=0);CHECK(result.dragPowerW>=result.harvestedAirPowerW-1e-10);
    }
    REQUIRE(result.fins.size()==1);CHECK(result.fins[0].angleRad<0);
    // Independent closed-form forced oscillator, including the wheel's
    // exponential spin-up left over from the 2.5 s pre-spin interval.
    const double torque=c.wheelDriveCoefficient*.5*10000*c.captureAreaM2*c.wheelRadiusM;
    const double b=torque*c.wheelRadiusM/(c.freeSpinRatio*100);
    const double tau=c.wheelInertiaKgM2/b,k=c.hingeStiffnessNmRad+5000*c.hingeAerodynamicStiffnessM3;
    const double force=.1*c.wheelInertiaKgM2*torque/b;
    const double constant=-force/k;
    const double exponential=force*std::exp(-2.5/tau)/(c.hingeInertiaKgM2/(tau*tau)-c.hingeDampingNmsRad/tau+k);
    const double discriminant=std::sqrt(c.hingeDampingNmsRad*c.hingeDampingNmsRad-4*c.hingeInertiaKgM2*k);
    const double r1=(-c.hingeDampingNmsRad+discriminant)/(2*c.hingeInertiaKgM2);
    const double r2=(-c.hingeDampingNmsRad-discriminant)/(2*c.hingeInertiaKgM2);
    const double a=(exponential/tau+r2*(constant+exponential))/(r1-r2),bb=-constant-exponential-a;
    const double expected=constant+exponential*std::exp(-.1/tau)+a*std::exp(r1*.1)+bb*std::exp(r2*.1);
    CHECK(result.fins[0].angleRad==Catch::Approx(expected).margin(2e-8));
    CHECK(std::abs(result.fins[0].angleRad)<=c.limitRad);CHECK(std::abs(result.fins[0].rateRadS)<=c.rateRadS);
    CHECK(std::abs(gyroWork+hingeWork)<1e-12);
    // Force is the wheel's harvested-air drag; the roll stabilization itself
    // comes from the finite patch angle, not a fitted body damping force.
    CHECK(result.forceBodyN.x<0);
}

TEST_CASE("Passive hinge hard stops dissipate energy and release after reversed roll", "[passive-rolleron]") {
    auto c=hardware();c.limitRad=.01;c.rateRadS=.1;PassiveRolleronV1 roller({c});auto in=wind(.001);
    in.angularRateInertialBodyRadS={50,0,0};double loss=0;RolleronOutputV1 result;
    double previous=0;
    for(unsigned i=0;i<1000;++i){result=roller.advance(in);REQUIRE(result.valid);loss+=result.stopLossJ;
        CHECK(std::abs(result.fins[0].angleRad-previous)<=c.rateRadS*in.dtS+1e-14);previous=result.fins[0].angleRad;
        CHECK(std::abs(result.fins[0].angleRad)<=c.limitRad);CHECK(std::abs(result.fins[0].rateRadS)<=c.rateRadS);}
    CHECK(loss>0);CHECK(result.fins[0].angleRad==Catch::Approx(-c.limitRad));
    const auto blocked=roller.advance(in);REQUIRE(blocked.valid);
    CHECK(blocked.fins[0].angleRad==Catch::Approx(-c.limitRad));
    CHECK(blocked.fins[0].rateRadS==0);
    CHECK(blocked.hingeHeatJ==0);CHECK(blocked.gyroHingeWorkJ==0);
    in.angularRateInertialBodyRadS.x=-50;
    for(unsigned i=0;i<1000;++i){result=roller.advance(in);REQUIRE(result.valid);}
    CHECK(result.fins[0].angleRad==Catch::Approx(c.limitRad));
}

TEST_CASE("Passive wheel state resets and rejects unavailable flow without mutation", "[passive-rolleron]") {
    PassiveRolleronV1 roller({hardware()}),fresh({hardware()});auto in=wind();
    for(unsigned i=0;i<20;++i)REQUIRE(roller.advance(in).valid);
    const auto before=roller.snapshot();in.airVelocityBodyMs.x=std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(roller.advance(in).valid);CHECK(roller.snapshot().fins[0].wheelRadS==before.fins[0].wheelRadS);
    roller.reset();in=wind();const auto a=roller.advance(in),b=fresh.advance(in);
    REQUIRE(a.valid);REQUIRE(b.valid);CHECK(a.fins[0].wheelRadS==b.fins[0].wheelRadS);
    CHECK(a.fins[0].angleRad==b.fins[0].angleRad);
}

TEST_CASE("Reversing rolleron airflow reverses wheel spin without producing propulsive air work", "[passive-rolleron]") {
    PassiveRolleronV1 roller({hardware()});auto in=wind();in.airVelocityBodyMs.x=-100;RolleronOutputV1 result;
    for(unsigned i=0;i<1000;++i){result=roller.advance(in);REQUIRE(result.valid);}
    CHECK(result.fins[0].wheelRadS<0);CHECK(result.forceBodyN.x>0);
    CHECK(result.forceBodyN.x*in.airVelocityBodyMs.x<=0);
}

TEST_CASE("External wheel spin-up transmits only actual bearing friction to the airframe", "[passive-rolleron][conservation]") {
    for(const double speed:{-100.,100.})for(const double friction:{0.,2e-5}) {
        CAPTURE(speed,friction);
        auto c=hardware();c.centreFromOriginBodyM={};c.bearingDampingNms=friction;
        PassiveRolleronV1 roller({c});auto in=wind(.01);in.airVelocityBodyMs.x=speed;
        const auto out=roller.advance(in);REQUIRE(out.valid);REQUIRE(out.fins.size()==1);
        const double drive=std::copysign(c.wheelDriveCoefficient*.5*in.densityKgM3*speed*speed*c.captureAreaM2*c.wheelRadiusM,speed);
        const double airDamping=std::abs(drive)*c.wheelRadiusM/(c.freeSpinRatio*std::abs(speed));
        const double damping=airDamping+friction,rate=damping/c.wheelInertiaKgM2;
        // Independent solution Jw*d(omega)/dt = Tair-(Bair+Bbearing)*omega.
        // External air torque accelerates the wheel; only Bbearing*omega
        // crosses an ideal wheel-axis bearing into a stationary airframe.
        const double equilibrium=drive/damping;
        const double spinIntegral=equilibrium*(in.dtS+std::expm1(-rate*in.dtS)/rate);
        const double bearingImpulse=friction*spinIntegral;
        CHECK(out.torqueBodyNm.x==0);CHECK(out.torqueBodyNm.y==0);
        CHECK(out.torqueBodyNm.z*in.dtS==Catch::Approx(bearingImpulse).margin(1e-12));
        if(friction==0)CHECK(out.torqueBodyNm.z==0);
        const double wheelEnergy=.5*c.wheelInertiaKgM2*out.fins[0].wheelRadS*out.fins[0].wheelRadS;
        CHECK(out.harvestedAirPowerW*in.dtS==Catch::Approx(wheelEnergy+out.bearingHeatJ).margin(1e-9));
        PassiveRolleronV1 divided({c});in.dtS=.005;
        const auto first=divided.advance(in),second=divided.advance(in);
        REQUIRE(first.valid);REQUIRE(second.valid);
        CHECK((first.torqueBodyNm.z+second.torqueBodyNm.z)*in.dtS==Catch::Approx(bearingImpulse).margin(1e-12));
    }
}
