// Actual normally generated models and actual JSBSim steps; no replacement dynamics.
#include "simulation/flight/JsbsimModel.h"
#include "simulation/flight/GroundProvider.h"
#include "simulation/flight/NativeStepObservationV1.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropulsion.h>
#include <models/propulsion/FGTurbine.h>
#include <array>
#include <algorithm>
#include <catch2/generators/catch_generators.hpp>
#include <cmath>
#include <filesystem>
#include <memory>
using namespace fsim;
using namespace fsim::sim;
namespace {
const auto stores=std::filesystem::path(FSIM_TEST_DATA_DIR).parent_path().parent_path().parent_path()/"aircraft/stores";
AircraftSpec actual(const char* name){return {name,FSIM_TEST_JSBSIM_ROOT,stores};}
InitialConditions initial(){InitialConditions ic;ic.altitudeMslM=10000;ic.airspeedTrueMs=600;ic.enginesRunning=false;return ic;}
void axial(JsbsimModel& model,double u,double v=0) {
    auto propagation=model.exec().GetPropagate();propagation->SetUVW(1,u/.3048);propagation->SetUVW(2,v/.3048);propagation->SetUVW(3,0);
    // This component boundary precedes FCS sampling. Auxiliary's input is a
    // copy, normally refreshed by FGFDMExec later in the native schedule.
    auto auxiliary=model.exec().GetAuxiliary();auxiliary->in.vUVW=propagation->GetUVW();auxiliary->Run(false);
}
struct InletBalanceV1 {
    JsbsimModel* model;double tau;std::array<double,5> priorIn{},priorOut{};
    double error=0,captureError=0;unsigned count=0;bool finite=true,reverse=false;
    double sampledU=0,sampledMach=0,firstRecaptureTime=-1,firstRecaptureU=0,firstRecaptureMach=0,firstRecaptureFlow=0;
    double maxCaptureNonpositive=0;unsigned reverseSamples=0,positiveSamples=0,nonpositiveSamples=0;
    InletBalanceV1(JsbsimModel& native,double residence):model(&native),tau(residence){syncOpening();}
    void syncOpening() {
        sampledU=model->exec().GetAuxiliary()->GetAeroUVW(1)*.3048;sampledMach=model->property("velocities/mach").get();
    }
    static void observe(void* context,unsigned,unsigned) noexcept {
        auto& b=*static_cast<InletBalanceV1*>(context);
        const char* incomingNames[]={"capture-kg-s","incoming-enthalpy-w","incoming-momentum-x-n","incoming-momentum-y-n","incoming-momentum-z-n"};
        const char* outgoingNames[]={"outflow-kg-s","outflow-enthalpy-w","outflow-momentum-x-n","outflow-momentum-y-n","outflow-momentum-z-n"};
        for(std::size_t k=0;k<5;++k) {
            const double incoming=b.model->property(std::string("fsim/ducted-inlet-v2/")+incomingNames[k]).get();
            const double outgoing=b.model->property(std::string("fsim/ducted-inlet-v2/")+outgoingNames[k]).get();
            const double change=b.tau*(outgoing-b.priorOut[k]);
            const double transport=.5*b.model->exec().GetDeltaT()*(incoming+b.priorIn[k]-outgoing-b.priorOut[k]);
            b.finite=b.finite&&std::isfinite(incoming)&&std::isfinite(outgoing)&&std::isfinite(change)&&std::isfinite(transport);
            if(b.count)b.error=std::max(b.error,std::abs(change-transport)/std::max({1.,std::abs(change),std::abs(transport)}));
            b.priorIn[k]=incoming;b.priorOut[k]=outgoing;
        }
        if(b.reverse) {
            ++b.reverseSamples;const double capture=b.priorIn[0];
            if(b.sampledU<=0){++b.nonpositiveSamples;b.maxCaptureNonpositive=std::max(b.maxCaptureNonpositive,capture);}
            if(b.sampledU>0)++b.positiveSamples;
            const double density=b.model->property("atmosphere/rho-slugs_ft3").get()*515.3788183931961;
            const double area=b.model->property("fsim/ducted-inlet-v2/input-capture_area_m2").get();
            const double lightoff=b.model->property("fsim/ducted-inlet-v2/input-lightoff_mach").get();
            const double expected=b.sampledMach>=lightoff?density*area*std::max(b.sampledU,0.):0.;
            b.finite=b.finite&&std::isfinite(expected)&&std::isfinite(b.sampledU)&&std::isfinite(b.sampledMach);
            b.captureError=std::max(b.captureError,std::abs(capture-expected)/std::max(1.,std::abs(expected)));
            if(capture>0&&b.firstRecaptureTime<0) {
                b.firstRecaptureTime=b.model->exec().GetSimTime();b.firstRecaptureU=b.sampledU;
                b.firstRecaptureMach=b.sampledMach;b.firstRecaptureFlow=capture;
            }
        }
        b.syncOpening();++b.count;
    }
};
}
TEST_CASE("Actual AIM9M rollerons spin and move native aerodynamic patches without command authority", "[variant-actuation-native][passive-rolleron-native]") {
    JsbsimModel model(1./120,std::make_shared<FlatGround>(0));REQUIRE(model.load(actual("aim9m"),initial()));
    REQUIRE(model.property("fsim/passive-rolleron-v1/version").get()==1);
    REQUIRE(model.property("fsim/passive-rolleron-v1/count").get()==4);
    CHECK(model.property("fsim/missile-control/aileron-limit-rad").get()==0);
    ControlInputs in;in.setThrottleAll(0);in.gearDown=0;
    for(unsigned i=0;i<60;++i)model.step(in);
    const double rates[]={.3,0,0};model.setBodyRates(rates);
    // Observe the first response before stored gyro/hinge energy can make
    // the free aircraft cross zero roll rate; a later oscillation is not
    // evidence of reversed initial stabilization.
    model.step(in);
    for(unsigned k=0;k<4;++k){const auto stem="fsim/passive-rolleron-v1/fin-"+std::to_string(k)+"/";
        CHECK(model.property(stem+"wheel-rad-s").get()>0);
        CHECK(std::abs(model.property(stem+"angle-rad").get())>0);
        const auto index=static_cast<unsigned>(model.property(stem+"fin-index").get());
        CHECK(model.property(stem+"angle-rad").get()==model.property("fcs/fin-"+std::to_string(index)+"-delta-rad").get());
    }
    CHECK(model.property("moments/l-aero-lbsft").get()<0);
    for(unsigned i=1;i<30;++i)model.step(in);
    VehicleState state;model.state(state);CHECK_FALSE(state.diverged);
    const double force[]={0,0,100},moment[]={0,0,0};model.setExternalForceBody(force,moment);model.step(in);
    CHECK(model.property("external_reactions/fsim/magnitude").get()==Catch::Approx(100/4.4482216152605));
    CHECK(model.property("external_reactions/fsim_rolleron/magnitude").get()>0);
}
TEST_CASE("Actual passive wheel and hinge reset exactly and cannot leak onto a legacy reload", "[variant-actuation-native][passive-rolleron-native]") {
    auto ground=std::make_shared<FlatGround>(0);JsbsimModel reused(1./120,ground),fresh(1./120,ground);
    REQUIRE(reused.load(actual("aim9m"),initial()));ControlInputs in;in.setThrottleAll(0);
    for(unsigned i=0;i<60;++i)reused.step(in);
    REQUIRE(reused.reset(initial()));REQUIRE(fresh.load(actual("aim9m"),initial()));
    for(unsigned i=0;i<30;++i){reused.step(in);fresh.step(in);
        for(unsigned k=0;k<4;++k){const auto stem="fsim/passive-rolleron-v1/fin-"+std::to_string(k)+"/";
            for(const auto* leaf:{"wheel-rad-s","angle-rad","rate-rad-s"})CHECK(reused.property(stem+leaf).get()==fresh.property(stem+leaf).get());}}
    auto legacy=initial();legacy.airspeedTrueMs=55;
    REQUIRE(reused.load({"c172x",FSIM_TEST_JSBSIM_ROOT},legacy));
    reused.step(in);CHECK(reused.property("external_reactions/fsim_rolleron/magnitude").get()==0);
    CHECK(reused.property("fsim/passive-rolleron-v1/version").get()==0);
}
TEST_CASE("Actual Meteor inlet conserves its finite native air inventory and closes on reverse axial flow", "[variant-actuation-native][ducted-inlet-native]") {
    const double dt=GENERATE(1./120,1./240);
    JsbsimModel model(dt,std::make_shared<FlatGround>(0));REQUIRE(model.load(actual("meteor"),initial()));
    REQUIRE(model.property("fsim/ducted-inlet-v2/version").get()==2);
    const double tau=model.property("fsim/ducted-inlet-v2/residence-s").get();REQUIRE(tau>0);
    model.exec().Setsim_time(20);axial(model,600);ControlInputs in;in.setThrottleAll(0);in.gearDown=0;
    InletBalanceV1 balance(model,tau);
    ScopedNativeStepObserverV1 observe({&model,&balance,InletBalanceV1::observe});
    for(unsigned i=0;i<60;++i)model.step(in);
    const double mass=model.property("fsim/ducted-inlet-v2/resident-air-mass-kg").get();REQUIRE(mass>0);
    const auto nativeMassIndex=static_cast<unsigned>(model.property("fsim/ducted-inlet-v2/resident-pointmass-index").get());
    CHECK(model.property("inertia/pointmass-weight-lbs["+std::to_string(nativeMassIndex)+"]").get()*.45359237==Catch::Approx(mass).epsilon(1e-12));
    CHECK(balance.error<1e-10);CHECK(balance.count==60*static_cast<unsigned>(std::llround(dt/model.exec().GetDeltaT())));
    const double before=model.property("fsim/ducted-inlet-v2/outflow-kg-s").get();
    const unsigned substeps=static_cast<unsigned>(std::llround(dt/model.exec().GetDeltaT()));
    balance.reverse=true;
    // Prescribed inlet component rig, not a free-flight outcome assertion.
    // Advance the actual full JSBSim plant at its already admitted native dt.
    // The existing observation callback stays read-only. Each next boundary
    // is imposed before Run, and every original native transport solve runs.
    for(unsigned step=0;step<121*substeps;++step) {
        axial(model,-600);balance.syncOpening();
        REQUIRE(model.exec().Run());InletBalanceV1::observe(&balance,step%substeps+1,substeps);
    }
    CHECK(balance.reverseSamples==121*substeps);CHECK(balance.positiveSamples==0);
    CHECK(balance.maxCaptureNonpositive==0);CHECK(balance.captureError<1e-10);
    CHECK(balance.finite);CHECK(balance.error<1e-10);
    CHECK(model.property("fsim/ducted-inlet-v2/outflow-kg-s").get()<before*.02);
    const double drainedMass=model.property("fsim/ducted-inlet-v2/resident-air-mass-kg").get();
    CHECK(drainedMass>=0);
    CHECK(model.property("inertia/pointmass-weight-lbs["+std::to_string(nativeMassIndex)+"]").get()*.45359237==Catch::Approx(drainedMass).margin(1e-14));
    REQUIRE(model.reset(initial()));CHECK(model.property("fsim/ducted-inlet-v2/resident-air-mass-kg").get()==0);
}

TEST_CASE("Actual Meteor free flight recaptures air only from its physical positive opening flow", "[variant-actuation-native][ducted-inlet-free-flight]") {
    const double dt=GENERATE(1./120,1./240);
    JsbsimModel model(dt,std::make_shared<FlatGround>(0));REQUIRE(model.load(actual("meteor"),initial()));
    REQUIRE(model.property("fsim/ducted-inlet-v2/version").get()==2);
    const double tau=model.property("fsim/ducted-inlet-v2/residence-s").get();REQUIRE(tau>0);
    model.exec().Setsim_time(20);ControlInputs in;in.setThrottleAll(0);in.gearDown=0;
    InletBalanceV1 balance(model,tau);ScopedNativeStepObserverV1 observe({&model,&balance,InletBalanceV1::observe});
    for(unsigned i=0;i<60;++i)model.step(in);
    axial(model,-600);balance.syncOpening();balance.reverse=true;
    for(unsigned i=0;i<121;++i)model.step(in); // Free native body: no further pose/flow intervention.
    INFO("native_dt="<<model.exec().GetDeltaT()<<" negative_samples="<<balance.nonpositiveSamples<<" positive_samples="<<balance.positiveSamples
        <<" first_recapture_t="<<balance.firstRecaptureTime<<" first_recapture_u="<<balance.firstRecaptureU
        <<" first_recapture_M="<<balance.firstRecaptureMach<<" first_recapture_kg_s="<<balance.firstRecaptureFlow);
    CHECK(balance.nonpositiveSamples>0);CHECK(balance.maxCaptureNonpositive==0);
    CHECK(balance.captureError<1e-10);CHECK(balance.finite);CHECK(balance.error<1e-10);
    if(balance.firstRecaptureTime>=0) {
        CHECK(balance.firstRecaptureU>0);
        CHECK(balance.firstRecaptureMach>=model.property("fsim/ducted-inlet-v2/input-lightoff_mach").get());
    }
    VehicleState state;model.state(state);CHECK_FALSE(state.diverged);
}

TEST_CASE("Actual R73 and PL10 commands move finite native thrust-vector nozzles", "[variant-actuation-native][thrust-vector-native]") {
    for(const auto* name:{"r73","pl10"}) {
        INFO(name);JsbsimModel model(1./120,std::make_shared<FlatGround>(0));auto ic=initial();ic.enginesRunning=true;
        REQUIRE(model.load(actual(name),ic));
        REQUIRE(model.property("fsim/thrust-vector-control/version").get()==1);
        const double limit=model.property("fsim/thrust-vector-control/limit_rad").get();
        const double rate=model.property("fsim/thrust-vector-control/rate_rad_s").get();
        const double sign=model.property("fsim/thrust-vector-control/pitch_gain").get();
        REQUIRE(limit>0);REQUIRE(rate>0);REQUIRE(sign!=0);
        ControlInputs in;in.setThrottleAll(1);in.elevator=.5;in.rudder=-.25;in.gearDown=0;
        struct Motion {JsbsimModel* model;double limit,rate,prior=0,error=0;unsigned count=0;bool bounded=true;};
        Motion motion{&model,limit,rate};
        ScopedNativeStepObserverV1 observe({&model,&motion,[](void* context,unsigned,unsigned)noexcept {
            auto& m=*static_cast<Motion*>(context);const double angle=m.model->property("propulsion/engine/pitch-angle-rad").get();
            m.bounded=m.bounded&&std::isfinite(angle)&&std::abs(angle)<=m.limit+1e-12;
            if(m.count)m.error=std::max(m.error,std::abs(angle-m.prior)-m.rate*m.model->exec().GetDeltaT());
            m.prior=angle;++m.count;
        }});
        for(unsigned i=0;i<30;++i)model.step(in);
        REQUIRE(motion.count>0);CHECK(motion.bounded);CHECK(motion.error<1e-12);CHECK(motion.prior*sign>0);
        CHECK(model.property("propulsion/engine/thrust-lbs").get()>0);
        CHECK(std::abs(model.property("moments/m-prop-lbsft").get())>0);
        VehicleState state;model.state(state);CHECK_FALSE(state.diverged);
    }
}

TEST_CASE("Native finite inlet refuses a timestep that can produce negative resident mass", "[variant-actuation-native][ducted-inlet-admission]") {
    const AircraftSpec contract{"inlet_step_admission",FSIM_TEST_JSBSIM_ROOT,FSIM_TEST_DATA_DIR};
    JsbsimModel rejected(1./120,std::make_shared<FlatGround>(0));CHECK_FALSE(rejected.load(contract,initial()));
    JsbsimModel admitted(.001,std::make_shared<FlatGround>(0));CHECK(admitted.load(contract,initial()));
}

TEST_CASE("Native thermal sibling reports actual gas kind and preserves unavailable rocket exhaust", "[variant-actuation-native][native-thermal]") {
    JsbsimModel aircraft(1./120,std::make_shared<FlatGround>(0));auto ic=initial();ic.airspeedTrueMs=55;ic.enginesRunning=true;
    REQUIRE(aircraft.load({"c172x",FSIM_TEST_JSBSIM_ROOT},ic));NativeThermalSnapshotV1 thermal;
    const NativeThermalSourceV1& source=aircraft;REQUIRE(source.nativeThermalSnapshotV1(thermal));
    CHECK(thermal.ambientAvailable);CHECK(thermal.ambientTemperatureK>0);REQUIRE(thermal.engineCount==1);
    CHECK(thermal.engineGasTemperatureKind[0]==NativeGasTemperatureKindV1::ExhaustGas);CHECK(thermal.engineGasTemperatureK[0]>0);
    const auto exhaust=aircraft.property("propulsion/engine/egt-degF");REQUIRE(exhaust.valid());
    const auto checkLive=[&] {
        REQUIRE(source.nativeThermalSnapshotV1(thermal));REQUIRE(thermal.engineCount==1);
        REQUIRE(thermal.engineGasTemperatureKind[0]==NativeGasTemperatureKindV1::ExhaustGas);
        const double expected=(exhaust.get()-32.)*(5./9.)+273.15;
        REQUIRE(std::isfinite(expected));CHECK(std::abs(thermal.engineGasTemperatureK[0]-expected)<1e-10);
        return expected;
    };
    const double cold=checkLive();ControlInputs powered;powered.setThrottleAll(.7);powered.gearDown=0;
    for(unsigned step=0;step<1200;++step){aircraft.step(powered);if(step%120==119)checkLive();}
    const double hot=checkLive();CHECK(hot>cold+10);CHECK(thermal.engineGasTemperatureK[0]>cold+10);
    auto magnetos=aircraft.property("propulsion/magneto_cmd");REQUIRE(magnetos.valid());magnetos.set(0);
    ControlInputs stopped;stopped.setThrottleAll(0);stopped.gearDown=0;
    for(unsigned step=0;step<1200;++step){aircraft.step(stopped);if(step%120==119)checkLive();}
    const double cooled=checkLive();CHECK(cooled<hot-1);CHECK(cooled>0);
    ic.enginesRunning=false;REQUIRE(aircraft.reset(ic));
    const double reset=checkLive();CHECK(reset<cold+1);
    // Native ResetToIC initializes gas from its loaded inlet state before
    // applying altitude IC. Preserve that native transient and compare a
    // fresh stopped plant, rather than imposing instant ambient equilibrium.
    JsbsimModel fresh(1./120,std::make_shared<FlatGround>(0));REQUIRE(fresh.load({"c172x",FSIM_TEST_JSBSIM_ROOT},ic));
    NativeThermalSnapshotV1 freshThermal;REQUIRE(fresh.nativeThermalSnapshotV1(freshThermal));
    REQUIRE(freshThermal.engineCount==1);CHECK(std::abs(reset-freshThermal.engineGasTemperatureK[0])<1e-10);
    CHECK(std::abs(thermal.ambientTemperatureK-freshThermal.ambientTemperatureK)<1e-10);
    JsbsimModel rocket(1./120,std::make_shared<FlatGround>(0));REQUIRE(rocket.load(actual("meteor"),initial()));
    REQUIRE(rocket.nativeThermalSnapshotV1(thermal));CHECK(thermal.ambientAvailable);
    for(unsigned i=0;i<thermal.engineCount;++i)CHECK(thermal.engineGasTemperatureKind[i]==NativeGasTemperatureKindV1::Unavailable);
}

TEST_CASE("Actual Meteor burns only its reserved fractional final native fuel batch", "[variant-actuation-native][ducted-inlet-fuel]") {
    JsbsimModel model(1./120,std::make_shared<FlatGround>(0));REQUIRE(model.load(actual("meteor"),initial()));
    REQUIRE(model.property("fsim/ducted-inlet-v2/engine[1]/finite-fuel-v2").get()==1);
    model.exec().Setsim_time(20);axial(model,600);ControlInputs in;in.setThrottleAll(0);in.gearDown=0;
    // Run the native inlet/valve at a physical supplied-air condition, then
    // set an independently specified final fuel amount in the actual tank.
    for(unsigned i=0;i<60;++i)model.step(in);
    constexpr double lastKg=1e-5,lb=.45359237;
    model.property("propulsion/tank[1]/contents-lbs").set(lastKg/lb);
    struct Fuel {JsbsimModel* model;double prior=0,total=0,error=0;unsigned dry=0;bool partial=false,heatDry=false;};
    Fuel fuel{&model,lastKg/lb};
    ScopedNativeStepObserverV1 observe({&model,&fuel,[](void* context,unsigned,unsigned)noexcept {
        auto& f=*static_cast<Fuel*>(context);const auto stem=std::string("fsim/ducted-inlet-v2/engine[1]/");
        const double remaining=f.model->property("propulsion/tank[1]/contents-lbs").get();
        const double consumed=f.prior-remaining,reserved=f.model->property(stem+"reserved-fuel-lbs").get();
        const double requested=f.model->property(stem+"requested-fuel-lbs").get();
        f.error=std::max(f.error,std::abs(consumed-reserved));f.total+=reserved;
        if(reserved>0&&reserved<requested)f.partial=true;
        if(reserved==0&&remaining==0){++f.dry;if(f.model->property(stem+"burned-fuel-kg-s").get()!=0)f.heatDry=true;}
        f.prior=remaining;
    }});
    for(unsigned i=0;i<30;++i)model.step(in);
    CHECK(fuel.partial);CHECK(fuel.error<1e-12);CHECK(fuel.total*lb==Catch::Approx(lastKg).margin(1e-12));
    CHECK(fuel.dry>0);CHECK_FALSE(fuel.heatDry);CHECK(model.property("propulsion/engine[1]/fuel-flow-rate-pps").get()==0);
    CHECK(model.property("propulsion/engine[1]/thrust-lbs").get()==0);
    REQUIRE(model.reset(initial()));
    for(const auto* leaf:{"engine[1]/fuel-kg-s","engine[1]/burned-fuel-kg-s","engine[1]/gross-thrust-n","net-propulsion-force-x-n"})
        CHECK(model.property(std::string("fsim/ducted-inlet-v2/")+leaf).get()==0);
    CHECK(model.property("forces/fbx-external-lbs").get()==0);
    JsbsimModel fresh(1./120,std::make_shared<FlatGround>(0));REQUIRE(fresh.load(actual("meteor"),initial()));
    model.exec().Setsim_time(20);fresh.exec().Setsim_time(20);axial(model,600);axial(fresh,600);
    for(unsigned i=0;i<30;++i) {
        model.step(in);fresh.step(in);
        for(const auto* leaf:{"resident-air-mass-kg","resident-enthalpy-j","engine[1]/burned-fuel-kg-s","net-propulsion-force-x-n"})
            CHECK(model.property(std::string("fsim/ducted-inlet-v2/")+leaf).get()==fresh.property(std::string("fsim/ducted-inlet-v2/")+leaf).get());
    }
}

TEST_CASE("Actual Meteor native cycle closes dynamic first law and three dimensional momentum", "[variant-actuation-native][ducted-inlet-energy-momentum]") {
    JsbsimModel model(1./120,std::make_shared<FlatGround>(0));REQUIRE(model.load(actual("meteor"),initial()));
    const auto prefix=std::string("fsim/ducted-inlet-v2/");
    const auto read=[&](const std::string& leaf){return model.property(prefix+leaf).get();};
    const auto input=[&](const char* role){return read(std::string("input-")+role);};
    model.exec().Setsim_time(20);ControlInputs controls;controls.setThrottleAll(0);controls.gearDown=0;
    for(unsigned i=0;i<60;++i)model.step(controls);
    axial(model,600,100);const double rates[]={.2,.1,.15};model.setBodyRates(rates);
    for(unsigned step=0;step<3;++step) {
        model.step(controls);
        const double air=read("outflow-kg-s"),fuel=read("engine[1]/fuel-kg-s"),enthalpy=read("outflow-enthalpy-w");
        REQUIRE(air>0);REQUIRE(fuel>0);
        const double burned=std::min(fuel,air*input("fuel_air_ratio_max"));
        const double temperature=(enthalpy+burned*input("combustion_efficiency")*input("heating_value_j_kg"))/((air+fuel)*input("gas_cp_j_kg_k"));
        CHECK(read("engine[1]/burned-fuel-kg-s")==Catch::Approx(burned).epsilon(1e-12));
        CHECK(read("engine[1]/total-temperature-k")==Catch::Approx(temperature).epsilon(1e-12));
        const double ambient=model.property("atmosphere/T-R").get()*5./9.;
        const double pressure=std::max(1.,input("pressure_recovery")*std::pow(enthalpy/(air*input("air_cp_j_kg_k")*ambient),3.5));
        const double exit=std::sqrt(2*input("gas_cp_j_kg_k")*temperature*(1-std::pow(pressure,-(input("gas_gamma")-1)/input("gas_gamma"))));
        const double gross=(air+fuel)*exit,tau=read("residence-s");
        CHECK(read("engine[1]/gross-thrust-n")==Catch::Approx(gross).epsilon(1e-12));
        const auto omega=model.exec().GetPropagate()->GetPQRi();
        std::array<double,3> momentum{},reaction{},net{},arm{};
        for(std::size_t k=0;k<3;++k)momentum[k]=read(std::string("outflow-momentum-")+"xyz"[k]+"-n");
        double correction=0;
        for(std::size_t k=0;k<3;++k) {
            const auto j=(k+1)%3,l=(k+2)%3;
            const double coupling=tau*(omega(static_cast<unsigned>(j+1))*momentum[l]-omega(static_cast<unsigned>(l+1))*momentum[j]);
            correction+=std::abs(coupling);reaction[k]=-momentum[k]+coupling;
            net[k]=reaction[k]+(k==0?gross:0.);
            CHECK(read(std::string("ram-reaction-")+"xyz"[k]+"-n")==Catch::Approx(reaction[k]).margin(1e-8));
            CHECK(read(std::string("net-propulsion-force-")+"xyz"[k]+"-n")==Catch::Approx(net[k]).margin(1e-7));
        }
        CHECK(correction>0); // Exercise stored transverse momentum, not just axial steady flow.
        const auto massIndex=std::to_string(static_cast<unsigned>(read("resident-pointmass-index")));
        for(std::size_t k=0;k<3;++k) {
            const double point=model.property(std::string("inertia/pointmass-location-")+"XYZ"[k]+"-inches["+massIndex+"]").get();
            const double cg=model.property(std::string("inertia/cg-")+"xyz"[k]+"-in").get();
            arm[k]=(k==1?1.:-1.)*(point-cg)*.0254;
        }
        for(std::size_t k=0;k<3;++k) {
            const auto j=(k+1)%3,l=(k+2)%3;
            const double moment=arm[j]*reaction[l]-arm[l]*reaction[j];
            CHECK(model.property(std::string("moments/")+"lmn"[k]+"-external-lbsft").get()*1.3558179483314==Catch::Approx(moment).margin(1e-7));
        }
    }
}

TEST_CASE("Private native thermal snapshot populates all eight actual B52 engines", "[variant-actuation-native][native-thermal]") {
    JsbsimModel model(1./120,std::make_shared<FlatGround>(0));auto ic=initial();ic.airspeedTrueMs=220;ic.enginesRunning=true;
    REQUIRE(model.load({"b52h",FSIM_TEST_JSBSIM_ROOT,stores.parent_path()},ic));
    REQUIRE(model.exec().GetPropulsion()->GetNumEngines()==8);
    NativeThermalSnapshotV1 thermal;REQUIRE(model.nativeThermalSnapshotV1(thermal));REQUIRE(thermal.engineCount==8);
    CHECK(thermal.ambientAvailable);CHECK(thermal.ambientTemperatureK>0);
    for(unsigned k=0;k<8;++k) {
        const auto native=model.exec().GetPropulsion()->GetEngine(k);
        const auto* turbine=dynamic_cast<const JSBSim::FGTurbine*>(native.get());REQUIRE(turbine!=nullptr);
        CHECK(thermal.engineGasTemperatureKind[k]==NativeGasTemperatureKindV1::ExhaustGas);
        CHECK(thermal.engineGasTemperatureK[k]>0);
        CHECK(thermal.engineGasTemperatureK[k]==turbine->GetEGT()+273.15);
    }
    VehicleState publicState;model.state(publicState);CHECK(publicState.engineCount<=VehicleState::kMaxEngines);
}

TEST_CASE("Fresh native reload preserves explicit pre-load RNG and cached distribution state", "[variant-actuation-native][native-reload]") {
    JsbsimModel model(.001,std::make_shared<FlatGround>(0));model.seed(712345);
    model.exec().GetRandomGenerator()->GetNormalRandomNumber(); // Leave a cached Gaussian variate.
    auto expected=*model.exec().GetRandomGenerator();
    const AircraftSpec fixture{"inlet_step_admission",FSIM_TEST_JSBSIM_ROOT,FSIM_TEST_DATA_DIR};
    REQUIRE(model.load(fixture,initial()));
    for(unsigned i=0;i<10;++i) {
        CHECK(model.exec().GetRandomGenerator()->GetNormalRandomNumber()==expected.GetNormalRandomNumber());
        CHECK(model.exec().GetRandomGenerator()->GetUniformRandomNumber()==expected.GetUniformRandomNumber());
    }
    model.exec().GetRandomGenerator()->GetNormalRandomNumber();expected=*model.exec().GetRandomGenerator();
    REQUIRE(model.load(fixture,initial()));
    for(unsigned i=0;i<10;++i) {
        CHECK(model.exec().GetRandomGenerator()->GetNormalRandomNumber()==expected.GetNormalRandomNumber());
        CHECK(model.exec().GetRandomGenerator()->GetUniformRandomNumber()==expected.GetUniformRandomNumber());
    }
}
