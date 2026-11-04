#include "simulation/mission_systems/weapon/OperatingPointControlV1.h"
#include "simulation/mission_systems/components/PlantProvidersV1.h"
#include "simulation/mission_systems/components/interfaces/OperatingPointControlV1.h"
#include "simulation/mission_systems/components/implementations/autopilot/OperatingPointAutopilotV1.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <utility>

namespace {
using namespace fsim::ms::weapon;
OperatingPointInputV1 synthetic() {
    OperatingPointInputV1 in;in.dtS=.01;in.actuatorSubsteps=1;
    in.plant.massKg=100;in.plant.speedMs=500;in.plant.actuatorLagS=.04;
    in.plant.inertiaKgM2={1,100,100};in.plant.limitsRad.fill(.3);
    in.plant.pitch={-10000,-2000,-5000,-10000,-2000};
    in.plant.yaw={-10000,2000,5000,-10000,-2000};
    in.plant.rollMomentPerRad=1000;in.plant.rollDamping=-1;
    in.plant.measured=AeroFeedback{};
    in.stops.version=in.stops.actuatorVersion=1;in.stops.lowerRad.fill(-.3);in.stops.upperRad.fill(.3);
    in.stops.channelRateRadS=4;in.stops.finRateRadS={4,4,4};
    for(unsigned axis=0;axis<3;++axis) {
        OperatingFinV1 fin;fin.minimumRad=-.3;fin.maximumRad=.3;fin.rateRadS=4;fin.mixer[axis]=1;
        fin.anglesRad={-.3,0,.3};
        const double gain=axis==0?1000:-10000;
        for(double angle:fin.anglesRad){std::array<double,3> moment{};moment[axis]=gain*angle;fin.momentsNm.push_back(moment);}
        in.fins.push_back(fin);
    }
    return in;
}
FinControlConfigV1 configuration() {FinControlConfigV1 c;c.version=1;return c;}
}

TEST_CASE("Operating point neutral allocation matches independently requested native moment", "[weapon-operating-point]") {
    auto in=synthetic();const auto out=operatingPointCommandsV1(in,configuration());
    REQUIRE(out.commands.available);CHECK(out.commands.aileron==Catch::Approx(0).margin(1e-12));
    for(unsigned axis=0;axis<3;++axis)CHECK(out.predictedMomentNm[axis]==Catch::Approx(out.requestedMomentNm[axis]).margin(1e-9));
}
TEST_CASE("Operating point reversed physical effectiveness corrects the actual moment direction", "[weapon-operating-point]") {
    auto in=synthetic();in.plant.ratesRadS[0]=1;
    for(std::size_t k=0;k<in.fins[0].anglesRad.size();++k)in.fins[0].momentsNm[k][0]*=-1;
    const auto out=operatingPointCommandsV1(in,configuration());REQUIRE(out.commands.available);
    CHECK(out.commands.aileron>0);CHECK(out.predictedMomentNm[0]<0);
    CHECK(out.predictedMomentNm[0]*in.plant.ratesRadS[0]<0);
}
TEST_CASE("Operating point singular unreachable authority remains bounded without an inverse", "[weapon-operating-point]") {
    auto in=synthetic();in.plant.ratesRadS[0]=10;
    for(auto& fin:in.fins)for(auto& moment:fin.momentsNm)moment={};
    const auto out=operatingPointCommandsV1(in,configuration());REQUIRE(out.commands.available);
    CHECK(std::isfinite(out.residualAccelerationNorm));CHECK(out.residualAccelerationNorm>0);
    CHECK(std::abs(out.commands.aileron)<=1);CHECK(out.predictedMomentNm[0]==0);
}
TEST_CASE("Operating point invalid snapshots refuse without an ideal fallback", "[weapon-operating-point]") {
    auto in=synthetic();in.fins[0].momentsNm[0][0]=std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(operatingPointCommandsV1(in,configuration()).commands.available);
    in=synthetic();in.dtS=0;CHECK_FALSE(operatingPointCommandsV1(in,configuration()).commands.available);
    in=synthetic();in.fins[0].anglesRad[1]=in.fins[0].anglesRad[0];
    CHECK_FALSE(operatingPointCommandsV1(in,configuration()).commands.available);
    in=synthetic();in.channelInputRad[0]=.4;
    CHECK_FALSE(operatingPointCommandsV1(in,configuration()).commands.available);
    in=synthetic();in.fins[0].mixer[1]=std::numeric_limits<double>::infinity();
    CHECK_FALSE(operatingPointCommandsV1(in,configuration()).commands.available);
    in=synthetic();in.plant.inertiaKgM2[0]=0;
    CHECK_FALSE(operatingPointCommandsV1(in,configuration()).commands.available);
    in=synthetic();in.dtS=1;
    CHECK_FALSE(operatingPointCommandsV1(in,configuration()).commands.available);
}
TEST_CASE("Operating point native lag physical rate and asymmetric stops bound actual movement", "[weapon-operating-point]") {
    auto in=synthetic();in.plant.ratesRadS[0]=100;in.dtS=.001;in.actuatorSubsteps=4;
    in.fins[0].minimumRad=-.01;in.fins[0].rateRadS=.1;
    const auto out=operatingPointCommandsV1(in,configuration());REQUIRE(out.commands.available);
    REQUIRE(out.predictedFinRad.size()==in.fins.size());
    CHECK(std::abs(out.predictedFinRad[0])<=in.fins[0].rateRadS*in.dtS+1e-12);
    CHECK(out.predictedFinRad[0]>=in.fins[0].minimumRad);
}
TEST_CASE("Frozen R77M combined fins distinguish reversal from stop limited directional authority", "[weapon-operating-point][r77m-reversal]") {
    // checkpoint19 age 5.8333333333, alpha=-.021927802748855352,
    // Mach2.5103153124. Unchanged XML Cl curves interpolate at that alpha;
    // the common positive Mach/qSb factor is set to one for this oracle.
    auto in=synthetic();in.fins.clear();
    constexpr double h=.08726646259971647,stop=4*h;
    const double values[4][9]={
        {.2949342010273177,.3855509515834842,.28884174896924264,.1452764443541068,0,-.1454902438121822,-.28987051190725316,-.4304099081296193,-.3394589840550972},
        {-.2949342010273269,-.3855509515834843,-.28884174896924325,-.14527644435409942,0,.1454902438121865,.2898705119072825,.4304099081295848,.339458984055103},
        {.2954639861832392,.3863757461516294,.28948681344489724,.14560052485234842,0,-.1458143113522348,-.290513300808914,-.43131390940200554,-.3392949001493489},
        {-.2954639861832544,-.3863757461516187,-.28948681344489724,-.14560052485235747,0,.14581431135223433,.29051330080888615,.43131390940201936,.33929490014939206}};
    const double actual[4]={stop,-stop,-.25196806730633975,.26270059797998985};
    const std::array<double,3> mixer[4]={{{-1,1,-1}},{{1,1,1}},{{-1,1,1}},{{1,1,-1}}};
    for(unsigned i=0;i<4;++i) {
        OperatingFinV1 fin;fin.minimumRad=-stop;fin.maximumRad=stop;fin.actualRad=actual[i];fin.rateRadS=4;fin.mixer=mixer[i];
        for(unsigned j=0;j<9;++j){fin.anglesRad.push_back((static_cast<int>(j)-4)*h);fin.momentsNm.push_back({values[i][j],0,0});}
        in.fins.push_back(fin);
    }
    const auto positive=operatingPointDirectionalMomentV1(in,{{1,0,0}});
    const auto negative=operatingPointDirectionalMomentV1(in,{{-1,0,0}});
    // In the negative direction fins0/1 cannot move further outward. The
    // remaining derivative is independently the two local interval slopes.
    const double expectedNegative=(values[2][2]-values[2][1])/h-(values[3][8]-values[3][7])/h;
    const double expectedPositive=-(values[0][8]-values[0][7])/h+(values[1][1]-values[1][0])/h-expectedNegative;
    CHECK(positive[0]==Catch::Approx(expectedPositive).margin(1e-10));
    CHECK(negative[0]==Catch::Approx(expectedNegative).margin(1e-10));
    CHECK((positive[0]-negative[0])*.5<0); // central differential reverses
    CHECK(std::abs(negative[0])<.1); // physical stops remove nearly all negative-command authority
    CHECK(positive[0]<negative[0]); // the more effective correction has the opposite command sign
    // A reproducible engineering replay uses these captured physical angles,
    // explicit finite actuator states (not an inferred historical lag), and
    // the unchanged frozen Cl curves with positive common load factor one.
    in.dtS=1./120;in.actuatorSubsteps=68;in.plant.ratesRadS[0]=.1;
    in.plant.limitsRad.fill(stop);in.stops.lowerRad.fill(-stop);in.stops.upperRad.fill(stop);
    in.stops.finRateRadS={4,4,4,4};
    const std::array<double,3> channels={(-actual[0]+actual[1]-actual[2]+actual[3])/4,
        (actual[0]+actual[1]+actual[2]+actual[3])/4,(-actual[0]+actual[1]+actual[2]-actual[3])/4};
    in.channelInputRad=in.channelLagRad=in.channelActualRad=channels;in.plant.measured->controlRad=channels;
    const auto interpolated=[&](unsigned i,double angle) {
        unsigned j=0;while(j+2<9&&angle>(static_cast<int>(j)+1-4)*h)++j;
        return values[i][j]+(values[i][j+1]-values[i][j])*(angle-(static_cast<int>(j)-4)*h)/h;
    };
    for(unsigned i=0;i<4;++i)in.plant.measured->momentBodyNm[0]+=interpolated(i,actual[i]);
    const auto prior=operatingPointPredictionV1(in,{channels[0]/stop,channels[1]/stop,channels[2]/stop});
    const auto correction=operatingPointCommandsV1(in,configuration());
    REQUIRE(prior.commands.available);REQUIRE(correction.commands.available);
    CHECK((correction.predictedMomentNm[0]-prior.predictedMomentNm[0])*in.plant.ratesRadS[0]<0);
    double physical=0;for(unsigned i=0;i<4;++i)physical+=interpolated(i,correction.predictedFinRad[i]);
    CHECK(correction.predictedMomentNm[0]==Catch::Approx(physical).margin(1e-10));
}

TEST_CASE("Operating point native lag prediction agrees with an independent two-step recurrence", "[weapon-operating-point]") {
    auto in=synthetic();in.dtS=.02;in.actuatorSubsteps=2;in.plant.ratesRadS[0]=.02;
    in.channelInputRad[0]=.003;in.channelLagRad[0]=.004;in.channelActualRad[0]=.004;
    in.fins[0].actualRad=.004;in.plant.measured->controlRad[0]=.004;
    in.plant.measured->momentBodyNm[0]=4;
    const auto out=operatingPointCommandsV1(in,configuration());REQUIRE(out.commands.available);
    // Native Tustin lag at h=.01, tau=.04: a=1/9, b=7/9.
    // No rate or stop activates for this small request. Final angular position
    // has a closed form, including the previous *input*, not just output.
    const double input=.3*out.commands.aileron;
    const double expected=(25*input+7*.003+49*.004)/81.;
    CHECK(out.predictedFinRad[0]==Catch::Approx(expected).margin(1e-12));
    CHECK(out.predictedMomentNm[0]==Catch::Approx(1000*expected).margin(1e-9));
    CHECK(out.predictedMomentNm[0]==Catch::Approx(out.requestedMomentNm[0]).margin(1e-7));
}
TEST_CASE("Operating point signed mixer allocates coupled roll and pitch without losing axis requests", "[weapon-operating-point]") {
    auto in=synthetic();in.plant.ratesRadS={.02,.01,0};const auto yaw=in.fins[2];in.fins.clear();
    for(double sign:{-1.,1.}) {
        OperatingFinV1 fin;fin.minimumRad=-.3;fin.maximumRad=.3;fin.rateRadS=4;fin.mixer={sign,1,0};
        fin.anglesRad={-.3,0,.3};
        for(double angle:fin.anglesRad)fin.momentsNm.push_back({sign*500*angle,-5000*angle,0});
        in.fins.push_back(fin);
    }
    in.fins.push_back(yaw); // Admit the Euler yaw torque from simultaneous p/q.
    const auto out=operatingPointCommandsV1(in,configuration());REQUIRE(out.commands.available);
    REQUIRE(out.predictedFinRad.size()==3);
    // Independent moments of the two actual physical fins. The opposite roll
    // signs and common pitch signs require a coupled solution, not a sign swap.
    CHECK(out.predictedMomentNm[0]==Catch::Approx(500*(out.predictedFinRad[1]-out.predictedFinRad[0])).margin(1e-10));
    CHECK(out.predictedMomentNm[1]==Catch::Approx(-5000*(out.predictedFinRad[1]+out.predictedFinRad[0])).margin(1e-10));
    for(unsigned axis=0;axis<3;++axis)CHECK(out.predictedMomentNm[axis]==Catch::Approx(out.requestedMomentNm[axis]).margin(1e-6));
    CHECK(out.predictedMomentNm[0]*in.plant.ratesRadS[0]+out.predictedMomentNm[1]*in.plant.ratesRadS[1]<0);
}
TEST_CASE("Operating point full wind loads rotate and shift to current CG with physical wind and body signs", "[weapon-operating-point]") {
    const std::array<double,6> loads={10,20,30,4,5,6};
    const auto straight=operatingPointBodyMomentV1(loads,0,0,{2,3,4});
    CHECK(straight[0]==Catch::Approx(4+3*30+4*20));
    CHECK(straight[1]==Catch::Approx(5+4*10-2*30));
    CHECK(straight[2]==Catch::Approx(6-2*20-3*10));
    // At alpha=pi/2, force is [L,Y,-D], about the reference point;
    // body-axis force power against [0,0,V] is still -D*V.
    const auto vertical=operatingPointBodyMomentV1(loads,1.5707963267948966,0,{2,3,4});
    CHECK(vertical[0]==Catch::Approx(4+3*10+4*20));
    CHECK(vertical[1]==Catch::Approx(5-4*30-2*10));
    CHECK(vertical[2]==Catch::Approx(6-2*20+3*30));
}

TEST_CASE("Operating point native fin failures preserve zero hardover and stuck precedence", "[weapon-operating-point]") {
    for(const auto failure:{OperatingFailureV1{true,false,false},OperatingFailureV1{false,true,false},
                           OperatingFailureV1{false,false,true},OperatingFailureV1{false,true,true},
                           OperatingFailureV1{true,true,true}}) {
        auto in=synthetic();in.plant.ratesRadS[0]=1;in.fins[0].actualRad=.02;
        in.plant.measured->controlRad[0]=.02;in.plant.measured->momentBodyNm[0]=20;
        in.fins[0].failure=failure;
        const auto out=operatingPointCommandsV1(in,configuration());REQUIRE(out.commands.available);
        REQUIRE(out.predictedFinRad.size()==3);
        const double expected=failure.stuck?.02:failure.hardover?(failure.zero||out.commands.aileron>=0?.06:-.02):0;
        CHECK(out.predictedFinRad[0]==Catch::Approx(expected).margin(1e-12));
        CHECK(out.predictedMomentNm[0]==Catch::Approx(1000*expected).margin(1e-9));
        CHECK(std::isfinite(out.residualAccelerationNorm));CHECK(std::abs(out.commands.aileron)<=1);
        CHECK(operatingPointDirectionalMomentV1(in,{1,0,0})[0]==0);
    }
}
TEST_CASE("Operating point native channel failures leave finite lag separate and bound physical fins", "[weapon-operating-point]") {
    for(const auto failure:{OperatingFailureV1{true,false,false},OperatingFailureV1{false,true,false},
                           OperatingFailureV1{false,false,true},OperatingFailureV1{false,true,true},
                           OperatingFailureV1{true,true,true}}) {
        auto in=synthetic();in.plant.ratesRadS[0]=1;
        in.channelActualRad[0]=in.channelLagRad[0]=in.channelInputRad[0]=.02;
        in.fins[0].actualRad=.02;in.plant.measured->controlRad[0]=.02;in.plant.measured->momentBodyNm[0]=20;
        in.channelFailure[0]=failure;
        const auto out=operatingPointCommandsV1(in,configuration());REQUIRE(out.commands.available);
        // Before the channel actuator, native Tustin lag is (u+.02)/9+7*.02/9.
        const double lag=(.3*out.commands.aileron+8*.02)/9;
        const double expected=failure.stuck?.02:failure.hardover?(failure.zero||lag>=0?.06:-.02):0;
        CHECK(out.predictedFinRad[0]==Catch::Approx(expected).margin(1e-12));
        CHECK(out.predictedMomentNm[0]==Catch::Approx(1000*expected).margin(1e-9));
        CHECK(std::isfinite(out.residualAccelerationNorm));CHECK(std::abs(out.commands.aileron)<=1);
        CHECK(operatingPointDirectionalMomentV1(in,{1,0,0})[0]==0);
    }
    // A physical stuck fin overrides even an independently failed upstream
    // channel; the channel's continuing native lag cannot move that fin.
    auto in=synthetic();in.fins[0].actualRad=.02;in.fins[0].failure.stuck=true;
    in.plant.ratesRadS[0]=1;in.plant.measured->controlRad[0]=.02;in.plant.measured->momentBodyNm[0]=20;
    in.channelFailure[0].zero=in.channelFailure[0].hardover=true;
    const auto out=operatingPointCommandsV1(in,configuration());REQUIRE(out.commands.available);
    CHECK(out.predictedFinRad[0]==Catch::Approx(.02));CHECK(out.residualAccelerationNorm>0);
}

TEST_CASE("Operating point desired moment retains the original unscheduled healthy cascade request", "[weapon-operating-point]") {
    auto in=synthetic();in.plant.ratesRadS[1]=.01;
    const auto out=operatingPointCommandsV1(in,configuration());REQUIRE(out.commands.available);
    // Independent retained-law calculation at zero incidence/force/moment:
    // pitch residual=20 Nm, trimmed alpha=-4/9000 rad,
    // q*=alpha/.4=-1/900 rad/s, J(q*-q)/.12=-250/27 Nm.
    // The finite-horizon allocator must not add another actuator pole request.
    CHECK(out.requestedMomentNm[0]==Catch::Approx(0).margin(1e-12));
    CHECK(out.requestedMomentNm[1]==Catch::Approx(-250./27.).margin(1e-10));
    CHECK(out.requestedMomentNm[2]==Catch::Approx(0).margin(1e-12));
    CHECK(out.predictedMomentNm[1]==Catch::Approx(-250./27.).margin(1e-7));
}

namespace {
namespace comp=fsim::ms::components;
struct AuthorityState {
    std::array<bool,5> binding{true,true,true,true,true};
    bool accepts=true;
    int allocation=0;
    unsigned executions=0,allocations=0;
    FinCommands proposal{.1,.2,.3,true};
};
template<class Interface> class AuthorityBinding : public Interface {
public:
    AuthorityBinding(AuthorityState& state,unsigned role):state_(state),role_(role){}
    std::string_view nativeModel() const noexcept override {return "jsbsim:synthetic";}
    bool available(const NativePlantMetadataV1& expected,const NativePlantMetadataV1& actual) const noexcept override {
        return state_.binding[role_]&&expected.schemaVersion==actual.schemaVersion&&
            expected.implementationCode==actual.implementationCode&&expected.fingerprint==actual.fingerprint;
    }
protected:
    AuthorityState& state_;
private:
    unsigned role_;
};
class AuthorityAutopilot final : public AuthorityBinding<comp::AutopilotProviderV1>,public comp::OperatingPointControlV1 {
public:
    explicit AuthorityAutopilot(AuthorityState& state):AuthorityBinding(state,0){}
    FinCommands commands(const AutopilotInput&,const ControlStopsV1&) const noexcept override {return {};}
    OperatingPointResultV1 commandsOperatingPointV1(const OperatingPointInputV1&) const noexcept override {
        ++state_.executions;OperatingPointResultV1 out;out.commands=state_.proposal;return out;
    }
};
class AuthorityOldAutopilot final : public AuthorityBinding<comp::AutopilotProviderV1> {
public:
    explicit AuthorityOldAutopilot(AuthorityState& state):AuthorityBinding(state,0){}
    FinCommands commands(const AutopilotInput&,const ControlStopsV1&) const noexcept override {return state_.proposal;}
};
class AuthorityAllocator final : public AuthorityBinding<comp::AllocatorProviderV1> {
public:
    explicit AuthorityAllocator(AuthorityState& state):AuthorityBinding(state,1){}
    FinCommands allocate(FinCommands command,const ControlStopsV1&) const noexcept override {
        ++state_.allocations;
        switch(state_.allocation) {
        case 1: command.available=false;break;
        case 2: command.aileron=std::numeric_limits<double>::quiet_NaN();break;
        case 3: command.rudder=1.01;break;
        case 4: command.elevator=-.75;break;
        default: break;
        }
        return command;
    }
};
class AuthorityActuator final : public AuthorityBinding<comp::ActuatorProviderV1> {
public:
    explicit AuthorityActuator(AuthorityState& state):AuthorityBinding(state,2){}
    bool accepts(const AutopilotInput&,const ControlStopsV1&) const noexcept override {return state_.accepts;}
};
class AuthorityPropulsion final : public AuthorityBinding<comp::PropulsionProviderV1> {
public:
    explicit AuthorityPropulsion(AuthorityState& state):AuthorityBinding(state,3){}
    bool acceptsEngineCount(unsigned) const noexcept override {return true;}
    bool burning(double thrust) const noexcept override {return thrust>0;}
};
class AuthorityAerodynamics final : public AuthorityBinding<comp::AerodynamicsProviderV1> {
public:
    explicit AuthorityAerodynamics(AuthorityState& state):AuthorityBinding(state,4){}
    std::string_view coefficientSource() const noexcept override {return "synthetic";}
};
comp::PlantProvidersV1 authorityProviders(AuthorityState& state) {
    comp::PlantProvidersV1 result;
    result.autopilot=std::make_unique<AuthorityAutopilot>(state);result.allocator=std::make_unique<AuthorityAllocator>(state);
    result.actuator=std::make_unique<AuthorityActuator>(state);result.propulsion=std::make_unique<AuthorityPropulsion>(state);
    result.aerodynamics=std::make_unique<AuthorityAerodynamics>(state);return result;
}
PlantAdapterV1 authoritySelection() {
    PlantAdapterV1 selected;selected.configured=selected.hasControl=true;selected.control=configuration();
    for(std::size_t slot=0;slot<5;++slot) {
        selected.availability[slot]=comp::AvailabilityV1::Available;
        selected.implementations[slot]=slot==0?3:1;
        selected.expected[slot]={1,static_cast<double>(selected.implementations[slot]),static_cast<double>(100+slot)};
    }
    return selected;
}
}
TEST_CASE("Operating point orchestration preserves every custom provider binding refusal", "[weapon-operating-point][provider-authority]") {
    const auto selected=authoritySelection();NativePlantIdentityV1 actual;actual.components=selected.expected;
    for(std::size_t slot=0;slot<5;++slot) {
        AuthorityState state;state.binding[slot]=false;auto providers=authorityProviders(state);
        CHECK_FALSE(providers.commandsOperatingPointV1(selected,synthetic(),actual).available);
        CHECK(state.executions==0);CHECK(state.allocations==0);
        state.binding[slot]=true;
        switch(slot) {
        case 0: providers.autopilot.reset();break;
        case 1: providers.allocator.reset();break;
        case 2: providers.actuator.reset();break;
        case 3: providers.propulsion.reset();break;
        case 4: providers.aerodynamics.reset();break;
        default: break;
        }
        CHECK_FALSE(providers.commandsOperatingPointV1(selected,synthetic(),actual).available);
        CHECK(state.executions==0);CHECK(state.allocations==0);
    }
}
TEST_CASE("Operating point orchestration preserves actuator and allocator authority", "[weapon-operating-point][provider-authority]") {
    const auto selected=authoritySelection();NativePlantIdentityV1 actual;actual.components=selected.expected;
    AuthorityState state;auto providers=authorityProviders(state);state.accepts=false;
    CHECK_FALSE(providers.commandsOperatingPointV1(selected,synthetic(),actual).available);
    CHECK(state.executions==0);CHECK(state.allocations==0);
    state.accepts=true;
    for(int mode=1;mode<=3;++mode) {
        state.allocation=mode;
        CHECK_FALSE(providers.commandsOperatingPointV1(selected,synthetic(),actual).available);
    }
    CHECK(state.executions==3);CHECK(state.allocations==3);
    state.allocation=4;
    const auto out=providers.commandsOperatingPointV1(selected,synthetic(),actual);REQUIRE(out.available);
    CHECK(out.elevator==-.75);CHECK(out.aileron==.1);CHECK(out.rudder==.3);
}
TEST_CASE("Operating point orchestration refuses malformed sibling outputs stale identity and absent capability", "[weapon-operating-point][provider-authority]") {
    const auto selected=authoritySelection();NativePlantIdentityV1 actual;actual.components=selected.expected;
    AuthorityState state;auto providers=authorityProviders(state);
    for(const auto malformed:{FinCommands{0,0,0,false},FinCommands{2,0,0,true},
                             FinCommands{0,std::numeric_limits<double>::infinity(),0,true}}) {
        state.proposal=malformed;
        CHECK_FALSE(providers.commandsOperatingPointV1(selected,synthetic(),actual).available);
    }
    CHECK(state.executions==3);CHECK(state.allocations==0);
    state.proposal={.1,.2,.3,true};actual.components[0].fingerprint+=1;
    CHECK_FALSE(providers.commandsOperatingPointV1(selected,synthetic(),actual).available);
    CHECK(state.executions==3);CHECK(state.allocations==0);
    actual.components=selected.expected;providers.autopilot=std::make_unique<AuthorityOldAutopilot>(state);
    CHECK_FALSE(providers.commandsOperatingPointV1(selected,synthetic(),actual).available);
    CHECK(state.allocations==0);
}

TEST_CASE("Operating point68substep population closes every clipped branch without a ULP budget stall", "[weapon-operating-point][operating-progress]") {
    const auto begin=std::chrono::steady_clock::now();
    std::uint64_t exhausted=0,retries=0,segments=0,scalar=0,differential=0,maxSegments=0,memo=0,maps=0,fallback=0,pieces=0;
    for(unsigned sample=0;sample<8;++sample) {
        auto in=synthetic();in.actuatorSubsteps=68;in.dtS=1./120;
        in.plant.ratesRadS={.01*(sample+1),-.003*(sample+1),.002*(sample+1)};
        for(unsigned axis=0;axis<3;++axis) {
            const double prior=(sample&1?1.:-1.)*(axis+1)*.013;
            in.channelInputRad[axis]=prior*.83;in.channelLagRad[axis]=prior*1.1;
            in.channelActualRad[axis]=prior;in.fins[axis].actualRad=prior;
            in.plant.measured->controlRad[axis]=prior;
            in.plant.measured->momentBodyNm[axis]=(axis==0?1000:-10000)*prior;
        }
        if(sample>=4) {
            // Signed cross-axis mixer and different physical stops exercise
            // combined rate/stop branches without changing channel limits.
            in.fins[0].mixer={-1,1,-1};in.fins[1].mixer={1,1,1};in.fins[2].mixer={-1,1,1};
            in.fins[0].maximumRad=.08;in.fins[1].minimumRad=-.07;
        }
        OperatingPointSearchStatsV1 stats;OperatingPointSearchOptionsV1 options;options.exhaustiveReference=true;
        const auto out=operatingPointCommandsV1(in,configuration(),stats,options);
        REQUIRE(out.commands.available);
        exhausted+=stats.budgetExhaustions;retries+=stats.boundaryRetries;segments+=stats.lineSegments;
        scalar+=stats.scalarPredictions;differential+=stats.differentialPredictions;
        memo+=stats.memoHits;maps+=stats.mapLines;fallback+=stats.fallbackLines;pieces+=stats.mapPieces;
        maxSegments=std::max(maxSegments,stats.maxLineSegments);
        INFO("sample="<<sample<<" segments="<<stats.lineSegments<<" retries="<<stats.boundaryRetries<<" exhausted="<<stats.budgetExhaustions);
        CHECK(stats.budgetExhaustions==0);
        CHECK(stats.boundaryRetries==0);
        CHECK(stats.completedLines>0);
        CHECK(std::isfinite(out.residualAccelerationNorm));
    }
    const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    std::cout<<"operating-progress68 samples=8 seconds="<<elapsed<<" scalar="<<scalar<<" differential="<<differential
             <<" segments="<<segments<<" retries="<<retries<<" exhausted="<<exhausted<<" max_line="<<maxSegments
             <<" memo_hits="<<memo<<" map_lines="<<maps<<" fallback="<<fallback<<" map_pieces="<<pieces<<'\n';
}
TEST_CASE("Operating point68substep native lag matches an independent held-input closed form", "[weapon-operating-point][operating-progress]") {
    for(unsigned sample=0;sample<8;++sample) {
        auto in=synthetic();in.actuatorSubsteps=68;in.dtS=1./120;
        in.plant.ratesRadS[0]=.01*(sample+1);
        in.channelInputRad[0]=.0003;in.channelLagRad[0]=in.channelActualRad[0]=.0004;
        in.fins[0].actualRad=.0004;in.plant.measured->controlRad[0]=.0004;in.plant.measured->momentBodyNm[0]=.4;
        const auto out=operatingPointCommandsV1(in,configuration());REQUIRE(out.commands.available);
        const double h=in.dtS/68,a=h/(2*.04+h),b=(2*.04-h)/(2*.04+h),u=.3*out.commands.aileron;
        const double closed=u+std::pow(b,67)*(a*(u+.0003)+b*.0004-u);
        CHECK(out.predictedFinRad[0]==Catch::Approx(closed).margin(1e-12));
        CHECK(out.predictedMomentNm[0]==Catch::Approx(1000*closed).margin(1e-9));
    }
}

TEST_CASE("Operating point direct root tracing crosses zero and clipped knot boundaries at68substeps", "[weapon-operating-point][operating-progress]") {
    for(double offset:{0.,std::nextafter(0.,1.),1e-18,-1e-18}) {
        auto in=synthetic();in.actuatorSubsteps=68;in.dtS=1./120;
        in.channelLagRad[0]=in.channelActualRad[0]=offset;in.fins[0].actualRad=offset;
        in.plant.measured->controlRad[0]=offset;in.plant.measured->momentBodyNm[0]=1000*offset;
        // The zero knot, zero-normalization breakpoint and both clipping
        // directions occur in every full [-1,1] line search. Near zero a
        // representable command change need not change a rounded state value.
        in.plant.ratesRadS[0]=.01;
        OperatingPointSearchStatsV1 stats;OperatingPointSearchOptionsV1 options;options.exhaustiveReference=true;
        const auto out=operatingPointCommandsV1(in,configuration(),stats,options);REQUIRE(out.commands.available);
        CHECK(stats.boundaryRetries==0);CHECK(stats.budgetExhaustions==0);
        CHECK(stats.completedLines>0);CHECK(out.predictedMomentNm[0]<0);
        CHECK(std::abs(out.commands.aileron)<=1);
    }
}

namespace {
void sameResult(const OperatingPointResultV1& actual,const OperatingPointResultV1& reference,double tolerance) {
    REQUIRE(actual.commands.available);REQUIRE(reference.commands.available);
    CHECK(actual.commands.aileron==Catch::Approx(reference.commands.aileron).margin(tolerance));
    CHECK(actual.commands.elevator==Catch::Approx(reference.commands.elevator).margin(tolerance));
    CHECK(actual.commands.rudder==Catch::Approx(reference.commands.rudder).margin(tolerance));
    CHECK(actual.residualAccelerationNorm==Catch::Approx(reference.residualAccelerationNorm).epsilon(tolerance).margin(tolerance));
    REQUIRE(actual.predictedFinRad.size()==reference.predictedFinRad.size());
    for(std::size_t i=0;i<actual.predictedFinRad.size();++i)
        CHECK(actual.predictedFinRad[i]==Catch::Approx(reference.predictedFinRad[i]).margin(tolerance));
    for(unsigned axis=0;axis<3;++axis)
        CHECK(actual.predictedMomentNm[axis]==Catch::Approx(reference.predictedMomentNm[axis]).epsilon(tolerance).margin(tolerance));
}
}
TEST_CASE("Incremental operating maps retain every reference candidate and scalar objective at68substeps", "[weapon-operating-point][operating-equivalence]") {
    auto in=synthetic();in.actuatorSubsteps=68;in.dtS=1./120;
    in.plant.ratesRadS={.02,-.003,.002};
    in.channelInputRad={.003,-.004,.002};in.channelLagRad={.004,-.003,.001};
    in.channelActualRad={.002,-.002,.001};
    for(unsigned axis=0;axis<3;++axis) {
        in.fins[axis].actualRad=in.channelActualRad[axis];
        in.plant.measured->controlRad[axis]=in.channelActualRad[axis];
        in.plant.measured->momentBodyNm[axis]=(axis==0?1000:-10000)*in.channelActualRad[axis];
    }
    std::vector<OperatingPointCandidateV1> oldAudit,newAudit;
    OperatingPointSearchOptionsV1 oldOptions;oldOptions.referenceTracer=true;oldOptions.candidateAudit=&oldAudit;
    OperatingPointSearchOptionsV1 newOptions;newOptions.exhaustiveReference=true;newOptions.candidateAudit=&newAudit;
    OperatingPointSearchStatsV1 oldStats,newStats;
    const auto reference=operatingPointCommandsV1(in,configuration(),oldStats,oldOptions);
    const auto actual=operatingPointCommandsV1(in,configuration(),newStats,newOptions);
    // O(N*eps) allows only floating construction roundoff; physical encounter
    // bounds and native acceptance criteria are unchanged.
    const double tolerance=1024*(68+1)*std::numeric_limits<double>::epsilon();
    sameResult(actual,reference,tolerance);
    CHECK(newStats.mapLines>0);CHECK(newStats.fallbackLines==0);CHECK(newStats.memoHits>0);
    REQUIRE(newAudit.size()==oldAudit.size());
    for(std::size_t i=0;i<oldAudit.size();++i) {
        INFO("candidate="<<i);
        for(unsigned axis=0;axis<3;++axis)
            CHECK(newAudit[i].command[axis]==Catch::Approx(oldAudit[i].command[axis]).epsilon(tolerance).margin(tolerance));
        CHECK(newAudit[i].cost==Catch::Approx(oldAudit[i].cost).epsilon(tolerance).margin(tolerance));
        CHECK(newAudit[i].effort==Catch::Approx(oldAudit[i].effort).epsilon(tolerance).margin(tolerance));
    }
}
TEST_CASE("Operating scalar memo and fixed-channel cache are exact and never survive an input snapshot", "[weapon-operating-point][operating-equivalence]") {
    auto in=synthetic();in.actuatorSubsteps=68;in.dtS=1./120;in.plant.ratesRadS={.02,.003,-.002};
    OperatingPointSearchStatsV1 firstStats,uncachedStats,changedStats,repeatedStats;
    OperatingPointSearchOptionsV1 cached;cached.exhaustiveReference=true;
    const auto first=operatingPointCommandsV1(in,configuration(),firstStats,cached);
    OperatingPointSearchOptionsV1 uncached;uncached.exhaustiveReference=true;uncached.memoize=false;uncached.cacheNonVaried=false;
    const auto independent=operatingPointCommandsV1(in,configuration(),uncachedStats,uncached);
    sameResult(first,independent,0);
    CHECK(firstStats.memoHits>0);CHECK(uncachedStats.memoHits==0);
    auto changed=in;changed.plant.measured->momentBodyNm[0]=1000;
    const auto second=operatingPointCommandsV1(changed,configuration(),changedStats,cached);
    REQUIRE(second.commands.available);
    CHECK(second.predictedMomentNm[0]==Catch::Approx(1000+1000*second.predictedFinRad[0]).margin(1e-9));
    const auto repeated=operatingPointCommandsV1(in,configuration(),repeatedStats,cached);
    sameResult(first,repeated,0);
    CHECK(firstStats.scalarPredictions==repeatedStats.scalarPredictions);
    CHECK(firstStats.memoHits==repeatedStats.memoHits);
}
TEST_CASE("Operating incremental maps preserve native malfunction endpoints and bounded overflow fallback", "[weapon-operating-point][operating-equivalence]") {
    for(const auto failure:{OperatingFailureV1{false,false,true},OperatingFailureV1{false,true,true},OperatingFailureV1{true,true,true}}) {
        auto in=synthetic();in.actuatorSubsteps=68;in.dtS=1./120;in.plant.ratesRadS[0]=.03;
        in.channelFailure[0]=failure;in.fins[1].failure=failure;
        OperatingPointSearchStatsV1 referenceStats,newStats;
        OperatingPointSearchOptionsV1 old;old.referenceTracer=true;
        const auto reference=operatingPointCommandsV1(in,configuration(),referenceStats,old);
        OperatingPointSearchOptionsV1 incremental;incremental.exhaustiveReference=true;
        const auto actual=operatingPointCommandsV1(in,configuration(),newStats,incremental);
        sameResult(actual,reference,2e-11);
        if(failure.zero&&!failure.stuck)
            CHECK(actual.predictedFinRad[0]==Catch::Approx(4./120).margin(1e-12));
        if(failure.stuck)CHECK(actual.predictedFinRad[0]==0);
    }
    auto in=synthetic();in.actuatorSubsteps=68;in.dtS=1./120;in.plant.ratesRadS={.02,-.003,.002};
    OperatingPointSearchStatsV1 referenceStats,fallbackStats;
    OperatingPointSearchOptionsV1 old;old.referenceTracer=true;
    const auto reference=operatingPointCommandsV1(in,configuration(),referenceStats,old);
    OperatingPointSearchOptionsV1 tiny;tiny.exhaustiveReference=true;tiny.mapCapacity=1;
    const auto fallback=operatingPointCommandsV1(in,configuration(),fallbackStats,tiny);
    sameResult(fallback,reference,0);
    CHECK(fallbackStats.fallbackLines>0);CHECK(fallbackStats.mapLines==0);
    CHECK(fallbackStats.completedLines==referenceStats.completedLines);
    CHECK(fallbackStats.lineSegments==referenceStats.lineSegments);
}

TEST_CASE("Production sequential allocation has deterministic bounded native work at68substeps", "[weapon-operating-point][operating-production]") {
    const auto begin=std::chrono::steady_clock::now();
    std::uint64_t scalar=0,differential=0,faces=0,iterations=0,polls=0,backtracks=0;
    double kernelSeconds=0,maxCallSeconds=0;
    for(unsigned sample=0;sample<8;++sample) {
        auto in=synthetic();in.actuatorSubsteps=68;in.dtS=1./120;
        in.plant.ratesRadS={.01*(sample+1),-.003*(sample+1),.002*(sample+1)};
        for(unsigned axis=0;axis<3;++axis) {
            const double prior=(sample&1?1.:-1.)*(axis+1)*.013;
            in.channelInputRad[axis]=prior*.83;in.channelLagRad[axis]=prior*1.1;
            in.channelActualRad[axis]=prior;in.fins[axis].actualRad=prior;
            in.plant.measured->controlRad[axis]=prior;
            in.plant.measured->momentBodyNm[axis]=(axis==0?1000:-10000)*prior;
        }
        if(sample>=4) {
            in.fins[0].mixer={-1,1,-1};in.fins[1].mixer={1,1,1};in.fins[2].mixer={-1,1,1};
            in.fins[0].maximumRad=.08;in.fins[1].minimumRad=-.07;
        }
        OperatingPointSearchStatsV1 stats,againStats;
        const auto callBegin=std::chrono::steady_clock::now();
        const auto out=operatingPointCommandsV1(in,configuration(),stats);
        const double callSeconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-callBegin).count();
        kernelSeconds+=callSeconds;maxCallSeconds=std::max(maxCallSeconds,callSeconds);
        REQUIRE(out.commands.available);
        const auto again=operatingPointCommandsV1(in,configuration(),againStats);sameResult(out,again,0);
        CHECK(stats.seeds==11);CHECK(stats.sequentialIterations<=44);CHECK(stats.scalarPredictions<=700);
        CHECK(stats.differentialPredictions<=264);CHECK(stats.qpFaces<=9504);CHECK(stats.backtracks<=352);CHECK(stats.polls<=264);
        CHECK(stats.mapLines==0);CHECK(stats.fallbackLines==0);
        const auto verified=operatingPointPredictionV1(in,{out.commands.aileron,out.commands.elevator,out.commands.rudder});
        REQUIRE(verified.commands.available);
        for(unsigned axis=0;axis<3;++axis)CHECK(out.predictedMomentNm[axis]==verified.predictedMomentNm[axis]);
        CHECK(stats.scalarPredictions==againStats.scalarPredictions);CHECK(stats.qpFaces==againStats.qpFaces);
        scalar+=stats.scalarPredictions;differential+=stats.differentialPredictions;faces+=stats.qpFaces;
        iterations+=stats.sequentialIterations;polls+=stats.polls;backtracks+=stats.backtracks;
    }
    const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    std::cout<<"operating-production68 samples=8 seconds="<<kernelSeconds<<" max_call_s="<<maxCallSeconds
             <<" verification_total_s="<<elapsed<<" repeated=2 scalar="<<scalar<<" differential="<<differential
             <<" qp_faces="<<faces<<" iterations="<<iterations<<" backtracks="<<backtracks<<" polls="<<polls<<'\n';
}
TEST_CASE("Native negative mixer hardover keeps exactzero nextafter and finite sign endpoints", "[weapon-operating-point][operating-production]") {
    auto in=synthetic();in.actuatorSubsteps=68;in.dtS=1./120;in.fins[0].mixer={-1,0,0};in.fins[0].failure.hardover=true;
    for(double command:{0.,std::nextafter(0.,1.),std::nextafter(0.,-1.),-1e-12,1e-12}) {
        const auto predicted=operatingPointPredictionV1(in,{command,0,0});REQUIRE(predicted.commands.available);
        // Native multiplication underflows both nextafter-zero inputs to signed
        // zero. FGActuator's strict input<0 test therefore selects the positive
        // stop; finite positive input flips the negative mixer to negative stop.
        const double expected=command==1e-12?-4./120:4./120;
        CHECK(predicted.predictedFinRad[0]==Catch::Approx(expected).margin(1e-12));
        CHECK(predicted.predictedMomentNm[0]==Catch::Approx(1000*expected).margin(1e-9));
    }
    in.fins[0].failure.zero=true;
    const auto zero=operatingPointPredictionV1(in,{1,0,0});REQUIRE(zero.commands.available);
    CHECK(zero.predictedFinRad[0]==Catch::Approx(4./120).margin(1e-12));
    in.fins[0].failure.stuck=true;
    const auto stuck=operatingPointPredictionV1(in,{1,0,0});REQUIRE(stuck.commands.available);CHECK(stuck.predictedFinRad[0]==0);
}
TEST_CASE("Production bounded solver matches an independent linear coupled optimum and seed safeguards", "[weapon-operating-point][operating-production]") {
    auto in=synthetic();in.actuatorSubsteps=68;in.dtS=1./120;in.plant.ratesRadS={.02,.01,0};
    const auto yaw=in.fins[2];in.fins.clear();
    for(double sign:{-1.,1.}) {
        OperatingFinV1 fin;fin.minimumRad=-.3;fin.maximumRad=.3;fin.rateRadS=4;fin.mixer={sign,1,0};fin.anglesRad={-.3,0,.3};
        for(double angle:fin.anglesRad)fin.momentsNm.push_back({sign*500*angle,-5000*angle,0});
        in.fins.push_back(fin);
    }
    in.fins.push_back(yaw);
    OperatingPointSearchStatsV1 stats;const auto out=operatingPointCommandsV1(in,configuration(),stats);REQUIRE(out.commands.available);
    // The retained cascade's requested roll=-1/6 and pitch=-250/27 (gyro yaw
    // handled independently). Both are reachable in the healthy linear map.
    CHECK(out.requestedMomentNm[0]==Catch::Approx(-1./6).margin(1e-10));
    CHECK(out.requestedMomentNm[1]==Catch::Approx(-250./27).margin(1e-10));
    for(unsigned axis=0;axis<3;++axis)CHECK(out.predictedMomentNm[axis]==Catch::Approx(out.requestedMomentNm[axis]).margin(1e-6));
    const double roll=500*(out.predictedFinRad[1]-out.predictedFinRad[0]);
    const double pitch=-5000*(out.predictedFinRad[1]+out.predictedFinRad[0]);
    CHECK(roll==Catch::Approx(out.predictedMomentNm[0]).margin(1e-10));CHECK(pitch==Catch::Approx(out.predictedMomentNm[1]).margin(1e-10));
    CHECK(roll*in.plant.ratesRadS[0]+pitch*in.plant.ratesRadS[1]<0);
    const auto neutral=operatingPointPredictionV1(in,{0,0,0});REQUIRE(neutral.commands.available);
    double neutralCost=0;for(unsigned axis=0;axis<3;++axis) {
        const double e=(neutral.predictedMomentNm[axis]-out.requestedMomentNm[axis])/in.plant.inertiaKgM2[axis];neutralCost+=e*e;
    }
    CHECK(out.residualAccelerationNorm*out.residualAccelerationNorm<=neutralCost);
}

namespace {
// Independent forward reference for the authored virtual FCS channels. These
// are INPUT-stage tests of the actual compiled R77M provider; retained direct
// operatingPointCommandsV1 tests above keep their instantaneous-moment units.
std::array<double,3> finiteInputChannels(const OperatingPointInputV1& in,const FinCommands& nominal) {
    const std::array<double,3> u={nominal.aileron,nominal.elevator,nominal.rudder};
    auto previous=in.channelInputRad,lag=in.channelLagRad,actual=in.channelActualRad;
    const double h=in.dtS/in.actuatorSubsteps;
    for(unsigned step=0;step<in.actuatorSubsteps;++step)for(unsigned axis=0;axis<3;++axis) {
        const double input=u[axis]*(u[axis]<0?-in.stops.lowerRad[axis]:in.stops.upperRad[axis]);
        // Trapezoidal discretization of tau*y_dot+y=input, before native
        // actuator malfunction/rate/stops. Previous INPUT is independent state.
        lag[axis]=((input+previous[axis])*h+(2*in.plant.actuatorLagS-h)*lag[axis])/
            (2*in.plant.actuatorLagS+h);
        previous[axis]=input;
        const auto& f=in.channelFailure[axis];
        if(f.stuck)continue;
        double demand=f.zero?0:lag[axis];
        if(f.hardover)demand=demand<0?in.stops.lowerRad[axis]:in.stops.upperRad[axis];
        actual[axis]=std::clamp(demand,actual[axis]-h*in.stops.channelRateRadS,actual[axis]+h*in.stops.channelRateRadS);
        actual[axis]=std::clamp(actual[axis],in.stops.lowerRad[axis],in.stops.upperRad[axis]);
    }
    return actual;
}
std::array<double,3> finiteInputTarget(const OperatingPointInputV1& in,const FinCommands& nominal) {
    const auto endpoint=finiteInputChannels(in,nominal);
    const std::array<double,3> gain={in.plant.rollMomentPerRad,in.plant.pitch.momentControl,in.plant.yaw.momentControl};
    auto target=in.plant.measured->momentBodyNm;
    for(unsigned axis=0;axis<3;++axis)target[axis]+=gain[axis]*(endpoint[axis]-in.plant.measured->controlRad[axis]);
    return target;
}
OperatingPointResultV1 existingOperatingProvider(const OperatingPointInputV1& in) {
    fsim::ms::components::OperatingPointAutopilotV1 provider("jsbsim:r77m",configuration());
    return provider.commandsOperatingPointV1(in);
}
}
TEST_CASE("Actual operating provider preserves a healthy linear finite FCS input", "[weapon-operating-point][operating-input-stage]") {
    for(const unsigned substeps:{1u,2u,68u}) {
        auto in=synthetic();in.dtS=1./120;in.actuatorSubsteps=substeps;
        in.plant.ratesRadS={.002,-.001,.0007};
        const auto nominal=finCommands(in.plant,configuration(),in.stops);REQUIRE(nominal.available);
        const auto expected=finiteInputTarget(in,nominal);const auto out=existingOperatingProvider(in);
        REQUIRE(out.commands.available);
        const std::array<double,3> input={out.commands.aileron,out.commands.elevator,out.commands.rudder};
        const std::array<double,3> wanted={nominal.aileron,nominal.elevator,nominal.rudder};
        const double h=in.dtS/substeps,decay=(2*in.plant.actuatorLagS-h)/(2*in.plant.actuatorLagS+h);
        // Closed-form homogeneous-state Tustin endpoint, independent of the
        // recurrence reference and solver. It must be one finite pole, not an
        // instantaneous angle followed by inversion of that same pole.
        const double ca=h/(2*in.plant.actuatorLagS+h),power=std::pow(decay,static_cast<double>(substeps-1));
        const double transfer=ca*power+2*ca*(1-power)/(1-decay);
        for(unsigned axis=0;axis<3;++axis) {
            CHECK(input[axis]==Catch::Approx(wanted[axis]).margin(1e-8));
            CHECK(out.requestedMomentNm[axis]==Catch::Approx(expected[axis]).margin(1e-7));
            CHECK(out.predictedMomentNm[axis]==Catch::Approx(expected[axis]).margin(1e-7));
            const double gain=axis==0?1000:-10000;
            CHECK(expected[axis]==Catch::Approx(gain*.3*wanted[axis]*transfer).margin(1e-9));
        }
    }
}
TEST_CASE("Actual operating provider retains finite input history and asymmetric channel scales", "[weapon-operating-point][operating-input-stage]") {
    auto in=synthetic();in.dtS=.02;in.actuatorSubsteps=2;
    in.stops.lowerRad={-.22,-.18,-.26};in.stops.upperRad={.3,.25,.21};
    in.channelInputRad={-.002,.003,-.001};in.channelLagRad={.001,-.002,.003};
    in.channelActualRad=in.channelLagRad;in.plant.measured->controlRad=in.channelActualRad;
    in.plant.ratesRadS={.002,-.001,.0007};
    for(unsigned axis=0;axis<3;++axis) {
        in.plant.limitsRad[axis]=std::max(-in.stops.lowerRad[axis],in.stops.upperRad[axis]);
        in.fins[axis].minimumRad=in.stops.lowerRad[axis];in.fins[axis].maximumRad=in.stops.upperRad[axis];
        in.fins[axis].actualRad=in.channelActualRad[axis];
        in.plant.measured->momentBodyNm[axis]=(axis==0?1000:-10000)*in.channelActualRad[axis];
    }
    const auto nominal=finCommands(in.plant,configuration(),in.stops);REQUIRE(nominal.available);
    const auto endpoint=finiteInputChannels(in,nominal),expected=finiteInputTarget(in,nominal);
    const auto out=existingOperatingProvider(in);REQUIRE(out.commands.available);
    const std::array<double,3> wanted={nominal.aileron,nominal.elevator,nominal.rudder};
    const std::array<double,3> input={out.commands.aileron,out.commands.elevator,out.commands.rudder};
    for(unsigned axis=0;axis<3;++axis) {
        const double v=wanted[axis]*(wanted[axis]<0?-in.stops.lowerRad[axis]:in.stops.upperRad[axis]);
        const double independent=(25*v+7*in.channelInputRad[axis]+49*in.channelLagRad[axis])/81.;
        CHECK(endpoint[axis]==Catch::Approx(independent).margin(1e-12));
        CHECK(input[axis]==Catch::Approx(wanted[axis]).margin(1e-8));
        CHECK(out.requestedMomentNm[axis]==Catch::Approx(expected[axis]).margin(1e-7));
        CHECK(out.predictedMomentNm[axis]==Catch::Approx(expected[axis]).margin(1e-7));
    }
}
TEST_CASE("Actual operating provider input stage respects native channel failures and finite rates", "[weapon-operating-point][operating-input-stage]") {
    for(const auto failure:{OperatingFailureV1{},OperatingFailureV1{true,false,false},OperatingFailureV1{false,true,false},
            OperatingFailureV1{false,false,true},OperatingFailureV1{false,true,true},OperatingFailureV1{true,true,true}}) {
        auto in=synthetic();in.dtS=1./120;in.actuatorSubsteps=68;in.plant.ratesRadS[0]=100;
        in.channelInputRad[0]=.03;in.channelLagRad[0]=.025;in.channelActualRad[0]=.02;
        in.fins[0].actualRad=.02;in.plant.measured->controlRad[0]=.02;in.plant.measured->momentBodyNm[0]=20;
        in.channelFailure[0]=failure;
        const auto nominal=finCommands(in.plant,configuration(),in.stops);REQUIRE(nominal.available);
        const auto endpoint=finiteInputChannels(in,nominal),expected=finiteInputTarget(in,nominal);
        const auto out=existingOperatingProvider(in);REQUIRE(out.commands.available);
        for(unsigned axis=0;axis<3;++axis) {
            CHECK(out.requestedMomentNm[axis]==Catch::Approx(expected[axis]).margin(1e-7));
            CHECK(out.predictedFinRad[axis]==Catch::Approx(endpoint[axis]).margin(1e-8));
            CHECK(out.predictedMomentNm[axis]==Catch::Approx(expected[axis]).margin(1e-7));
        }
        CHECK(endpoint[0]>=.02-4*in.dtS-1e-12);CHECK(endpoint[0]<=.02+4*in.dtS+1e-12);
        if(failure.stuck)CHECK(endpoint[0]==.02);
        if(failure.zero&&!failure.hardover&&!failure.stuck)CHECK(endpoint[0]==0);
        if(failure.zero&&failure.hardover&&!failure.stuck)CHECK(endpoint[0]==Catch::Approx(.02+4*in.dtS).margin(1e-12));
    }
}

namespace {
OperatingPointInputV1 capturedFirstR77InputStage() {
    // Actual native R77M PRE sequence1 at age0; source control-total-axial/
    // actual-first-output/r-77m/control-pre.json SHA-256 061a3b0936b7cc209b4a431c13de507a7dbeef112525e137ed098309d957bdef
    // Frozen physical input, not a replacement encounter or an OEM calibration.
    OperatingPointInputV1 in;in.fins.reserve(4);
    in.dtS=0.008333333333333333;
    in.actuatorSubsteps=68;
    in.plant.massKg=189.99998269487622;
    in.plant.speedMs=249.99272139675145;
    in.plant.actuatorLagS=0.04;
    in.plant.alphaRad=0.03114027834818433;
    in.plant.betaRad=6.265498243946535e-05;
    in.plant.inertiaKgM2={0.9341598963371854,218.42305708560835,218.42305708560835};
    in.plant.demandBody={0.41418300496562116,-0.9664449621128872,-14.418085289567141};
    in.plant.gravityBody={-0.29302394706259494,0.02502946301542064,9.80223927043862};
    in.plant.ratesRadS={0.0009294866150132739,4.026167362170129e-05,-0.0001360403021333372};
    in.plant.limitsRad={0.3490658503988659,0.3490658503988659,0.3490658503988659};
    in.plant.pitch={-5395.362933036668,-3936.8468416749124,-2628.489157128557,-6476.643406830953,-60.00657899323663};
    in.plant.yaw={-5177.732426232143,3910.5758507492615,2285.090061466569,-6436.32519348667,-60.19467515159478};
    in.plant.rollMomentPerRad=950.0891891772166;
    in.plant.rollDamping=-0.7873306501271884;
    in.plant.measured=AeroFeedback{{-123.57576620258126,-0.3328288412598078,-150.7354063836731},{-0.0008907452803341852,-76.88015130309248,0.14289650414494262},{0.0,0.0,0.0}};
    in.stops.version=in.stops.actuatorVersion=1;
    in.stops.lowerRad={-0.3490658503988659,-0.3490658503988659,-0.3490658503988659};in.stops.upperRad={0.3490658503988659,0.3490658503988659,0.3490658503988659};
    in.stops.channelRateRadS=4.0;in.stops.finRateRadS.reserve(4);for(unsigned i=0;i<4;++i)in.stops.finRateRadS.push_back(4.0);
    in.channelInputRad={0.0,0.0,0.0};
    in.channelLagRad={0.0,0.0,0.0};
    in.channelActualRad={0.0,0.0,0.0};
    {OperatingFinV1 f;f.actualRad=0.0;f.minimumRad=-0.3490658503988659;
     f.maximumRad=0.3490658503988659;f.rateRadS=4.0;f.mixer={-1.0,1.0,-1.0};
     f.anglesRad.reserve(9);for(const double angle:std::array<double,9>{-0.3490658503988659,-0.2617993877991494,-0.17453292519943295,-0.08726646259971647,0.0,0.08726646259971647,0.17453292519943295,0.2617993877991494,0.3490658503988659})f.anglesRad.push_back(angle);
     f.momentsNm.reserve(9);for(const auto& moment:std::array<std::array<double,3>,9>{{{48.92248941283628,325.0590804939826,-317.51129157442585},{61.48675716556142,425.52700916630107,-408.6686014888912},{41.37586702820339,285.0264287253296,-276.8578372842113},{20.76000179055348,142.2785517882877,-139.8134082168759},{0.0,0.0,0.0},{-20.716512006821254,-140.53807937270204,141.37073474575803},{-41.16576344897355,-277.7867958709322,282.76529028385545},{-44.26516382305444,-295.50342750437034,301.18026875779367},{-40.124683694279014,-262.9970021907098,266.5166196586132}}})f.momentsNm.push_back(moment);in.fins.push_back(std::move(f));}
    {OperatingFinV1 f;f.actualRad=0.0;f.minimumRad=-0.3490658503988659;
     f.maximumRad=0.3490658503988659;f.rateRadS=4.0;f.mixer={1.0,1.0,1.0};
     f.anglesRad.reserve(9);for(const double angle:std::array<double,9>{-0.3490658503988659,-0.2617993877991494,-0.17453292519943295,-0.08726646259971647,0.0,0.08726646259971647,0.17453292519943295,0.2617993877991494,0.3490658503988659})f.anglesRad.push_back(angle);
     f.momentsNm.reserve(9);for(const auto& moment:std::array<std::array<double,3>,9>{{{-48.92248941284229,325.05908049238747,317.5112915590716},{-61.486757165559936,425.52700916423964,408.66860147626824},{-41.37586702821657,285.0264287240168,276.857837279227},{-20.76000179055473,142.27855178760095,139.81340821606707},{0.0,0.0,0.0},{20.716512006823198,-140.53807937202905,-141.37073474837212},{41.16576344897345,-277.7867958695528,-282.7652902924172},{44.26516382305384,-295.50342750294516,-301.18026877125203},{40.12468369427693,-262.99700218947527,-266.51661967783974}}})f.momentsNm.push_back(moment);in.fins.push_back(std::move(f));}
    {OperatingFinV1 f;f.actualRad=0.0;f.minimumRad=-0.3490658503988659;
     f.maximumRad=0.3490658503988659;f.rateRadS=4.0;f.mixer={-1.0,1.0,1.0};
     f.anglesRad.reserve(9);for(const double angle:std::array<double,9>{-0.3490658503988659,-0.2617993877991494,-0.17453292519943295,-0.08726646259971647,0.0,0.08726646259971647,0.17453292519943295,0.2617993877991494,0.3490658503988659})f.anglesRad.push_back(angle);
     f.momentsNm.reserve(9);for(const auto& moment:std::array<std::array<double,3>,9>{{{48.973958788197,308.68467883378946,337.1490854890333},{61.303103284215204,409.9751866539387,424.7436759589968},{41.24597085170469,277.9211522629676,284.22183895442},{20.694522663668117,140.42219415732956,141.80495208981844},{0.0,0.0,0.0},{-20.65100998143828,-142.0775104490382,-139.99225970286997},{-41.035052145247334,-284.22803753679943,-276.63149597197855},{-44.152688402917455,-306.43513345180446,-288.26560827249153},{-40.018120939644554,-280.288008569197,-246.13376117749706}}})f.momentsNm.push_back(moment);in.fins.push_back(std::move(f));}
    {OperatingFinV1 f;f.actualRad=0.0;f.minimumRad=-0.3490658503988659;
     f.maximumRad=0.3490658503988659;f.rateRadS=4.0;f.mixer={1.0,1.0,-1.0};
     f.anglesRad.reserve(9);for(const double angle:std::array<double,9>{-0.3490658503988659,-0.2617993877991494,-0.17453292519943295,-0.08726646259971647,0.0,0.08726646259971647,0.17453292519943295,0.2617993877991494,0.3490658503988659})f.anglesRad.push_back(angle);
     f.momentsNm.reserve(9);for(const auto& moment:std::array<std::array<double,3>,9>{{{-48.97395878820338,308.68467883540495,-337.1490855042582},{-61.30310328421347,409.9751866559949,-424.7436759716118},{-41.245970851682415,277.9211522642411,-284.22183895946426},{-20.694522663666515,140.42219415801785,-141.80495209064733},{0.0,0.0,0.0},{20.651009981432956,-142.0775104497382,139.9922597002757},{41.03505214524739,-284.2280375381837,276.6314959634122},{44.1526884029175,-306.43513345324277,288.26560825899895},{40.01812093964602,-280.28800857043524,246.13376115823974}}})f.momentsNm.push_back(moment);in.fins.push_back(std::move(f));}
    return in;
}
}
TEST_CASE("Actual R77M first PRE uses finite virtual response rather than instantaneous input angle", "[weapon-operating-point][operating-input-stage][r77m-input-stage]") {
    const auto in=capturedFirstR77InputStage();
    const auto nominal=finCommands(in.plant,configuration(),in.stops);REQUIRE(nominal.available);
    CHECK(nominal.elevator==Catch::Approx(-.4812923523240495).margin(1e-12));
    const auto expected=finiteInputTarget(in,nominal);
    const auto out=existingOperatingProvider(in);REQUIRE(out.commands.available);
    for(unsigned axis=0;axis<3;++axis)
        CHECK(out.requestedMomentNm[axis]==Catch::Approx(expected[axis]).margin(1e-7));
    // These are separately derived native-clock endpoints, not retuned limits.
    CHECK(expected[1]==Catch::Approx(126.39545653043575).margin(1e-8));
    CHECK(expected[2]==Catch::Approx(-15.049799771409262).margin(1e-8));
    CHECK(std::isfinite(out.residualAccelerationNorm));
    REQUIRE(out.predictedFinRad.size()==in.fins.size());
    for(std::size_t i=0;i<in.fins.size();++i) {
        CHECK(out.predictedFinRad[i]>=in.fins[i].minimumRad);
        CHECK(out.predictedFinRad[i]<=in.fins[i].maximumRad);
    }
}
