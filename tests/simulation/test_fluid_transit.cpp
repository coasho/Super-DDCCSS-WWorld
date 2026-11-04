#include "simulation/mission_systems/damage/FluidTransitV1.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <limits>

namespace {
namespace fluid = fsim::ms::damage::fluid;
using fsim::ms::damage::V3;
using fsim::ms::damage::dot;
using fsim::ms::damage::cross;
const fluid::Aabb cube{{0,0,0},{1,1,1}};
const fluid::VolumeControls volumeControls{1e-12,1e-12,100};
const fluid::QuadratureControls quadratureControls{1e-11,1e-10,20,100000};
void near(V3 actual,V3 expected,double tolerance=1e-10) {
    CHECK(actual.x==Catch::Approx(expected.x).margin(tolerance));
    CHECK(actual.y==Catch::Approx(expected.y).margin(tolerance));
    CHECK(actual.z==Catch::Approx(expected.z).margin(tolerance));
}
}

TEST_CASE("Hydrostatic AABB occupancy conserves actual volume", "[fluid-transit][fluid-volume]") {
    for (const V3 normal : {V3{1,0,0},V3{0,-2,0},V3{0,0,3}}) {
        const auto level=fluid::freeSurface(cube,normal,0.25,volumeControls);
        REQUIRE(level.valid); REQUIRE(level.converged);
        const auto volume=fluid::immersedVolume(cube,{normal,level.offset});
        REQUIRE(volume); CHECK(*volume==Catch::Approx(0.25).margin(2e-12));
    }
    CHECK(fluid::freeSurface(cube,{1,0,0},0.25,volumeControls).offset==Catch::Approx(0.25));
    CHECK(fluid::freeSurface(cube,{0,-2,0},0.25,volumeControls).offset==Catch::Approx(-1.5));
    CHECK(fluid::freeSurface(cube,{0,0,3},0.25,volumeControls).offset==Catch::Approx(0.75));
    // x+y+z <= h is a tetrahedron for 0 <= h <= 1.
    for(double h : {0.01,0.2,0.8,1.0}) {
        const auto v=fluid::immersedVolume(cube,{{1,1,1},h});
        REQUIRE(v); CHECK(*v==Catch::Approx(h*h*h/6).margin(1e-14));
    }
    const auto half=fluid::immersedVolume(cube,{{1,1,1},1.5});
    REQUIRE(half); CHECK(*half==Catch::Approx(0.5).margin(1e-13));
    for(double target : {0.0,0.01,0.5,0.93,1.0}) {
        const auto result=fluid::freeSurface(cube,{1,2,3},target,volumeControls);
        REQUIRE(result.valid); REQUIRE(result.converged);
        CHECK(result.volume==Catch::Approx(target).margin(2e-12));
    }
    // A near-axis plane must not suffer inverse-CDF cancellation.
    const auto nearAxis=fluid::immersedVolume(cube,{{1,1e-15,0},0.25});
    REQUIRE(nearAxis); CHECK(*nearAxis==Catch::Approx(0.25).margin(1e-13));
    const auto thin=fluid::immersedVolume({{0,0,0},{1e-15,1,1}},{{0,1,0},0.25});
    REQUIRE(thin); CHECK(*thin==Catch::Approx(2.5e-16).epsilon(1e-12));
    const auto dry=fluid::clipImmersedLine(cube,{{0,0,1},0.25},{-1,0.5,0.5},{2,0.5,0.5});
    REQUIRE(dry.valid); CHECK_FALSE(dry.intersects);
    const auto wet=fluid::clipImmersedLine(cube,{{0,0,1},0.25},{0.5,0.5,-1},{0.5,0.5,1});
    REQUIRE(wet.valid); REQUIRE(wet.intersects);
    CHECK(wet.first==Catch::Approx(0.5)); CHECK(wet.last==Catch::Approx(0.625));
    const auto reversed=fluid::clipImmersedLine(cube,{{0,0,1},0.25},{0.5,0.5,1},{0.5,0.5,-1});
    REQUIRE(reversed.valid); REQUIRE(reversed.intersects);
    CHECK(reversed.first==Catch::Approx(1-wet.last));CHECK(reversed.last==Catch::Approx(1-wet.first));
}

TEST_CASE("Point liquid drag has exact finite time and path solutions", "[fluid-transit][fluid-point]") {
    const fluid::Point point{{2,3,4},{100,0,0},2};
    const fluid::Medium medium{{0,0,0},800,0.7,0.001};
    const double k=800*0.7*0.001/4;
    const double t=0.15, distance=std::log1p(k*100*t)/k;
    const auto time=fluid::advancePoint(point,medium,t);
    REQUIRE(time); near(time->velocity,{100/(1+k*100*t),0,0});
    near(time->position,{2+distance,3,4});
    CHECK(time->relativeDistance==Catch::Approx(distance));
    const auto path=fluid::advancePointDistance(point,medium,distance);
    REQUIRE(path); CHECK(path->elapsedS==Catch::Approx(t));
    near(path->position,time->position); near(path->velocity,time->velocity);
    CHECK(path->velocity.x==Catch::Approx(100*std::exp(-k*distance)));
    near(time->mediumImpulse,(point.velocity-time->velocity)*point.massKg);
    CHECK(time->dissipatedEnergyJ==Catch::Approx(0.5*2*(10000-time->velocity.x*time->velocity.x)));
    const auto first=fluid::advancePoint(point,medium,t/2); REQUIRE(first);
    const auto second=fluid::advancePoint({first->position,first->velocity,point.massKg},medium,t/2);
    REQUIRE(second); near(second->position,time->position); near(second->velocity,time->velocity);
    CHECK(first->dissipatedEnergyJ+second->dissipatedEnergyJ==Catch::Approx(time->dissipatedEnergyJ));
    const auto vacuum=fluid::advancePoint(point,{{0,0,0},0,0.7,0.001},t);
    REQUIRE(vacuum); near(vacuum->position,{17,3,4}); near(vacuum->velocity,point.velocity);
    CHECK(vacuum->dissipatedEnergyJ==0);
}

TEST_CASE("Point liquid drag respects moving-medium energy and Galilean balance", "[fluid-transit][fluid-point]") {
    const fluid::Point point{{0,0,0},{12,4,-1},3};
    const fluid::Medium medium{{2,1,-1},12,0.8,0.02};
    const auto base=fluid::advancePoint(point,medium,0.3); REQUIRE(base);
    const V3 shift{15,-8,4};
    const auto shifted=fluid::advancePoint({point.position,point.velocity+shift,point.massKg},
        {medium.velocity+shift,medium.densityKgM3,medium.dragCoefficient,medium.areaM2},0.3);
    REQUIRE(shifted); near(shifted->velocity,base->velocity+shift);
    near(shifted->position,base->position+shift*0.3);
    near(shifted->mediumImpulse,base->mediumImpulse);
    CHECK(shifted->dissipatedEnergyJ==Catch::Approx(base->dissipatedEnergyJ));
    const double kineticLoss=0.5*point.massKg*(dot(point.velocity,point.velocity)-dot(base->velocity,base->velocity));
    CHECK(kineticLoss==Catch::Approx(base->dissipatedEnergyJ+dot(medium.velocity,base->mediumImpulse)));
}

TEST_CASE("Explicit liquid Cd half and double screening retains analytic point balances", "[fluid-transit][fluid-point][liquid-cd-sensitivity]") {
    // Cd=1 is a declared screening assumption, not an OEM fit. Test its
    // predeclared half/double uncertainty without changing any scene outcome.
    const V3 moving{3,-2,1};const fluid::Point point{{0,.4,0},moving+V3{40,0,0},2};
    double previousResidual=40,previousWork=0;
    for(const double cd:{.5,1.,2.}) {
        INFO("explicit liquid Cd="<<cd);const fluid::Medium medium{moving,800,cd,.001};
        const double k=.2*cd,distance=.75,ratio=std::exp(-k*distance),expectedSpeed=40*ratio;
        const double expectedTime=std::expm1(k*distance)/(k*40);
        const auto path=fluid::advancePointDistance(point,medium,distance);REQUIRE(path);
        CHECK(path->elapsedS==Catch::Approx(expectedTime).margin(1e-13));
        near(path->velocity,moving+V3{expectedSpeed,0,0});
        near(path->position,point.position+moving*expectedTime+V3{distance,0,0});
        near(path->mediumImpulse,{2*(40-expectedSpeed),0,0});
        CHECK(path->dissipatedEnergyJ==Catch::Approx(1600*(1-ratio*ratio)).margin(1e-10));
        near(point.velocity*point.massKg,path->velocity*point.massKg+path->mediumImpulse);
        const double inertialLoss=.5*point.massKg*(dot(point.velocity,point.velocity)-dot(path->velocity,path->velocity));
        CHECK(inertialLoss==Catch::Approx(path->dissipatedEnergyJ+dot(moving,path->mediumImpulse)).margin(1e-10));
        CHECK(expectedSpeed<previousResidual);CHECK(path->dissipatedEnergyJ>previousWork);
        previousResidual=expectedSpeed;previousWork=path->dissipatedEnergyJ;
        const double time=.02,timeRatio=1/(1+k*40*time);
        const auto timed=fluid::advancePoint(point,medium,time);REQUIRE(timed);
        near(timed->velocity,moving+V3{40*timeRatio,0,0});
        near(timed->position,point.position+moving*time+V3{std::log1p(k*40*time)/k,0,0});
    }
}

TEST_CASE("Explicit liquid Cd half and double screening retains distributed rod power", "[fluid-transit][fluid-rod][liquid-cd-sensitivity]") {
    const V3 moving{2,3,-1};const fluid::Rod rod{{-1,0,0},{1,0,0},moving+V3{0,10,0},moving+V3{0,10,0},2};
    for(const double cd:{.5,1.,2.}) {
        INFO("explicit liquid Cd="<<cd);
        const auto load=fluid::rodLoads(rod,{moving,2,cd,.1},.2,.7,quadratureControls);
        REQUIRE(load.valid);REQUIRE(load.converged);
        // Uniform 10*Cd N force density over s=[.2,.7]; exact shape integrals
        // are integral(N0)=.275 and integral(N1)=.225.
        near(load.force0,{0,-2.75*cd,0});near(load.force1,{0,-2.25*cd,0});
        near(load.force,{0,-5*cd,0});near(load.torqueAboutMidpoint,{0,0,.5*cd});
        near((load.acceleration0+load.acceleration1)*(rod.massKg/2),load.force);
        CHECK(load.relativePowerW==Catch::Approx(-50*cd).margin(1e-10));
        CHECK(load.inertialPowerW==Catch::Approx(load.relativePowerW+dot(moving,load.force)).margin(1e-10));
        const double derivative=rod.massKg/6*(2*dot(rod.velocity0,load.acceleration0)+dot(load.acceleration0,rod.velocity1)+
            dot(rod.velocity0,load.acceleration1)+2*dot(rod.velocity1,load.acceleration1));
        CHECK(derivative==Catch::Approx(load.inertialPowerW).margin(1e-10));
    }
}

TEST_CASE("Partly immersed uniform rod retains consistent mass force moment and power", "[fluid-transit][fluid-rod]") {
    const fluid::Rod rod{{-1,0,0},{1,0,0},{0,10,0},{0,10,0},2};
    const fluid::Medium medium{{0,0,0},2,1,0.1};
    const auto result=fluid::rodLoads(rod,medium,0,0.5,quadratureControls);
    REQUIRE(result.valid); REQUIRE(result.converged);
    // q=.5*rho*Cd*A*v^2 = 10 N per unit material coordinate.
    near(result.force0,{0,-3.75,0}); near(result.force1,{0,-1.25,0});
    near(result.force,{0,-5,0}); near(result.torqueAboutMidpoint,{0,0,2.5});
    CHECK(result.relativePowerW==Catch::Approx(-50));
    near((result.acceleration0+result.acceleration1)*(rod.massKg/2),result.force);
    near(cross(rod.end0-(rod.end0+rod.end1)*0.5,result.force0)+
         cross(rod.end1-(rod.end0+rod.end1)*0.5,result.force1),result.torqueAboutMidpoint);
    // Consistent kinetic-energy derivative, not two endpoint point masses.
    const double derivative=rod.massKg/6*(2*dot(rod.velocity0,result.acceleration0)+
        dot(result.acceleration0,rod.velocity1)+dot(rod.velocity0,result.acceleration1)+
        2*dot(rod.velocity1,result.acceleration1));
    CHECK(derivative==Catch::Approx(result.inertialPowerW));
    CHECK(result.inertialPowerW==Catch::Approx(dot(rod.velocity0,result.force0)+dot(rod.velocity1,result.force1)));
    const auto spinning=fluid::rodLoads({{-1,0,0},{1,0,0},{0,-10,0},{0,10,0},2},medium,0,1,quadratureControls);
    REQUIRE(spinning.valid); REQUIRE(spinning.converged);
    near(spinning.force,{}); near(spinning.torqueAboutMidpoint,{0,0,-2.5});
    // Intrinsic rotation remains part of the mass metric: I=m*L^2/12.
    const double angularAcceleration=(spinning.acceleration1.y-spinning.acceleration0.y)/2;
    CHECK((2*4.0/12)*angularAcceleration==Catch::Approx(-2.5));
    CHECK(spinning.relativePowerW==Catch::Approx(-25));
    const auto dry=fluid::rodLoads(rod,medium,0.5,0.5,quadratureControls);
    REQUIRE(dry.valid); REQUIRE(dry.converged); near(dry.force,{}); CHECK(dry.evaluations==0);
}

TEST_CASE("Positive adjacent double rod intervals retain finite uniform loads", "[fluid-transit][fluid-rod][fluid-adjacent-interval]") {
    const fluid::Rod rod{{-1,0,0},{1,0,0},{0,100,0},{0,100,0},2};const fluid::Medium medium{{},2,1,.1};
    for(const double first:{.5,.84047982744282568,std::nextafter(1.,0.)}) {
        const double last=std::nextafter(first,1.);INFO("first="<<first<<" last="<<last);
        REQUIRE(last>first);REQUIRE((first+(last-first)*.5==first||first+(last-first)*.5==last));
        const auto load=fluid::rodLoads(rod,medium,first,last,quadratureControls);REQUIRE(load.valid);REQUIRE(load.converged);
        const long double width=static_cast<long double>(last)-first,mean=(static_cast<long double>(first)+last)/2;
        const double force=static_cast<double>(-1000*width),moment=static_cast<double>(-1000*width*(2*mean-1));
        const auto relative=[](double actual,double expected){CHECK(std::abs(actual-expected)<=std::abs(expected)*1e-12+std::numeric_limits<double>::denorm_min());};
        relative(load.force.y,force);relative(load.force1.y,static_cast<double>(-1000*width*mean));
        relative(load.force0.y,static_cast<double>(-1000*width*(1-mean)));relative(load.torqueAboutMidpoint.z,moment);
        relative(load.relativePowerW,100*force);CHECK(load.force.y<0);CHECK(load.relativePowerW<0);
        CHECK(load.estimatedForceErrorN<=quadratureControls.absoluteForceToleranceN+quadratureControls.relativeTolerance*std::abs(force));
    }
}

TEST_CASE("Adjacent double varying rod loads retain the global error budget", "[fluid-transit][fluid-rod][fluid-adjacent-interval]") {
    const fluid::Rod rod{{-1,0,0},{1,0,0},{0,50,0},{0,150,0},2};const fluid::Medium medium{{},2,1,.1};
    const double first=.84047982744282568,last=std::nextafter(first,1.);
    const long double width=static_cast<long double>(last)-first,speed=50+100*static_cast<long double>(first);
    const long double square=speed*speed*width+speed*100*width*width+(10000.L/3)*width*width*width;
    const long double weighted=first*square+speed*speed*width*width/2+(200*speed/3)*width*width*width+2500*width*width*width*width;
    const long double cube=speed*speed*speed*width+150*speed*speed*width*width+10000*speed*width*width*width+250000*width*width*width*width;
    const auto load=fluid::rodLoads(rod,medium,first,last,quadratureControls);REQUIRE(load.valid);REQUIRE(load.converged);
    const auto relative=[](double actual,double expected){CHECK(std::abs(actual-expected)<=std::abs(expected)*1e-12+std::numeric_limits<double>::denorm_min());};
    relative(load.force.y,static_cast<double>(-.1L*square));relative(load.force1.y,static_cast<double>(-.1L*weighted));
    relative(load.force0.y,static_cast<double>(-.1L*(square-weighted)));relative(load.relativePowerW,static_cast<double>(-.1L*cube));
    CHECK(load.relativePowerW<0);CHECK(load.evaluations<=quadratureControls.maxEvaluations);
    auto impossible=quadratureControls;impossible.absoluteForceToleranceN=1e-40;impossible.relativeTolerance=0;
    const auto refused=fluid::rodLoads(rod,medium,first,last,impossible);REQUIRE(refused.valid);CHECK_FALSE(refused.converged);
}

TEST_CASE("Rod quadrature resolves affine relative velocity and moving medium", "[fluid-transit][fluid-rod]") {
    const fluid::Rod rod{{0,0,0},{2,1,-1},{-4,3,2},{8,-1,5},1.2};
    const fluid::Medium medium{{2,1,1},800,0.4,0.001};
    const auto coarse=fluid::rodLoads(rod,medium,0.12,0.87,{1e-6,1e-6,20,100000});
    const auto fine=fluid::rodLoads(rod,medium,0.12,0.87,quadratureControls);
    REQUIRE(coarse.valid); REQUIRE(coarse.converged); REQUIRE(fine.valid); REQUIRE(fine.converged);
    CHECK(fine.evaluations<=quadratureControls.maxEvaluations);
    near(coarse.force,fine.force,1e-5); near(coarse.force1,fine.force1,1e-5);
    CHECK(fine.relativePowerW<=0);
    CHECK(fine.inertialPowerW==Catch::Approx(fine.relativePowerW+dot(medium.velocity,fine.force)));
    const V3 boost{20,30,-10};
    const auto shifted=fluid::rodLoads({rod.end0,rod.end1,rod.velocity0+boost,rod.velocity1+boost,rod.massKg},
        {medium.velocity+boost,medium.densityKgM3,medium.dragCoefficient,medium.areaM2},0.12,0.87,quadratureControls);
    REQUIRE(shifted.valid); REQUIRE(shifted.converged); near(shifted.force,fine.force);
    CHECK(shifted.relativePowerW==Catch::Approx(fine.relativePowerW));
    // Independent high-resolution midpoint integration, including a relative-speed minimum.
    V3 force{},moment{}; const int count=100000;
    for(int i=0;i<count;++i) {
        const double s=0.12+(0.87-0.12)*(i+0.5)/count;
        const V3 w=rod.velocity0*(1-s)+rod.velocity1*s-medium.velocity;
        const V3 f=w*(-0.5*medium.densityKgM3*medium.dragCoefficient*medium.areaM2*std::hypot(w.x,w.y,w.z)*(0.87-0.12)/count);
        force+=f; moment+=f*s;
    }
    near(fine.force,force,1e-7); near(fine.force1,moment,1e-7);
}

TEST_CASE("Fluid primitives reject invalid inputs and report numerical budget failure", "[fluid-transit][fluid-invalid]") {
    const double nan=std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(fluid::immersedVolume(cube,{{0,0,0},0}));
    CHECK_FALSE(fluid::immersedVolume({{0,0,0},{0,1,1}},{{0,0,1},0}));
    CHECK_FALSE(fluid::freeSurface(cube,{0,0,1},1.1,volumeControls).valid);
    CHECK_FALSE(fluid::freeSurface(cube,{0,0,1},nan,volumeControls).valid);
    const auto limited=fluid::freeSurface(cube,{1,2,3},0.123456789,{1e-15,0,1});
    REQUIRE(limited.valid); CHECK_FALSE(limited.converged);
    CHECK_FALSE(fluid::clipImmersedLine(cube,{{0,0,1},0},{nan,0,0},{1,1,1}).valid);
    CHECK_FALSE(fluid::advancePoint({{},{1,0,0},0},{{},1,1,1},1));
    CHECK_FALSE(fluid::advancePoint({{},{1,0,0},1},{{},1,-1,1},1));
    CHECK_FALSE(fluid::advancePoint({{},{1,0,0},1},{{},1,1,1},-1));
    CHECK_FALSE(fluid::advancePointDistance({{},{0,0,0},1},{{},1,1,1},1));
    CHECK_FALSE(fluid::advancePointDistance({{},{1,0,0},1},{{},1,1,1},1e5));
    const fluid::Rod rod{{0,0,0},{1,0,0},{-10,0,0},{10,1,0},1};
    CHECK_FALSE(fluid::rodLoads(rod,{{},1,1,1},0.8,0.2,quadratureControls).valid);
    CHECK_FALSE(fluid::rodLoads(rod,{{},1,1,1},0,1,{0,0,20,1000}).valid);
    const auto exhausted=fluid::rodLoads(rod,{{},1,1,1},0,1,{1e-14,1e-14,1,5});
    REQUIRE(exhausted.valid); CHECK_FALSE(exhausted.converged);
    near(exhausted.force,{}); // Failed integration publishes no partial physical loads.
    // Cancellation with relative-only controls is allowed to exhaust a finite
    // budget, but may never publish locally accepted, globally unchecked loads.
    const auto relativeOnly=fluid::rodLoads({{0,0,0},{1,0,0},{-1,1,0},{1,1,0},1},
        {{},1,1,1},0,1,{0,1e-10,8,1000});
    REQUIRE(relativeOnly.valid);CHECK_FALSE(relativeOnly.converged);
    CHECK(relativeOnly.evaluations<=1000);near(relativeOnly.force,{});
}
