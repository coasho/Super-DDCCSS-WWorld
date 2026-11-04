// Numeric method fixture: actual current C++ entry points, actual card inputs,
// disclosed synthetic SI plant. This is not aircraft identification or a flight.
#include "simulation/mission_systems/Cards.h"
#include "simulation/mission_systems/components/Profile.h"
#include "simulation/mission_systems/components/implementations/guidance/MeasuredPnGuidanceV2.h"
#include "simulation/mission_systems/components/implementations/allocator/JsbsimAuthoredFinMixerV1.h"
#include "simulation/mission_systems/components/implementations/autopilot/OperatingPointAutopilotV1.h"
#include "simulation/mission_systems/weapon/Composition.h"
#include "simulation/mission_systems/weapon/PlantAdapter.h"
#include "simulation/mission_systems/weapon/OperatingPointControlV1.h"
#include "simulation/flight/GroundProvider.h"
#include "simulation/mission_systems/weapon/PulseManagerV1.h"
#include "simulation/flight/JsbsimModel.h"
#include "simulation/flight/PassiveRolleronV1.h"
#include "simulation/flight/NativeStepObservationV1.h"
#include "FGFDMExec.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGRocket.h"
#include "models/propulsion/FGTank.h"
#include "foundation/core/Json.h"
#include "ComponentReferenceRuntimeV1.h"
#include <picosha2.h>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <cwctype>

namespace {
using namespace fsim::ms;
using namespace fsim::ms::weapon;
using namespace fsim::ms::components;
constexpr std::array<const char*,24> cards{{"aim-120c","aim-120d","aim-7m","aim-9m","aim-9x","asraam",
    "iris-t","magic-2","meteor","mica-em","mica-ir","pl-10","pl-12","pl-15","pl-8","r-27er",
    "r-27et","r-27r","r-27t","r-37m","r-60m","r-73","r-77m","super-530d"}};
void allowedNativePath(const std::filesystem::path& path) {
    for(const auto& part:path) {
        auto name=part.wstring();
        std::transform(name.begin(),name.end(),name.begin(),[](wchar_t c){return static_cast<wchar_t>(std::towlower(c));});
        REQUIRE(name!=L".codex");
    }
}
std::string hashReferenceFile(const std::filesystem::path& path) {
    allowedNativePath(path);std::ifstream input(path,std::ios::binary);REQUIRE(input.good());
    picosha2::hash256_one_by_one hash;char buffer[65536];
    while(input.read(buffer,sizeof(buffer))||input.gcount()>0)hash.process(buffer,buffer+input.gcount());
    REQUIRE(input.eof());hash.finish();return picosha2::get_hash_hex_string(hash);
}
struct CaptureHeader {
    std::string epochSha;
    fsim::core::Json executable;
    fsim::core::Json runtimeModules;
};
CaptureHeader captureHeader(bool requireJsbsim=false) {
    const char* source=std::getenv("FSIM_COMPONENT_REFERENCE_EPOCH");REQUIRE(source);REQUIRE(*source);
    CaptureHeader result;result.epochSha=hashReferenceFile(source);
    std::filesystem::path executable;
#if defined(_WIN32)
    std::array<wchar_t,32768> buffer{};
    const DWORD length=GetModuleFileNameW(nullptr,buffer.data(),static_cast<DWORD>(buffer.size()));
    REQUIRE(length>0);REQUIRE(length<buffer.size());
    executable=std::filesystem::path(std::wstring(buffer.data(),length));
#else
    executable=std::filesystem::read_symlink("/proc/self/exe");
#endif
    allowedNativePath(executable);
    result.executable.set("path",executable.generic_string()).set("sha256",hashReferenceFile(executable));
    result.runtimeModules=fsim::testing::componentReferenceRuntimeV1(hashReferenceFile,requireJsbsim);
    return result;
}
std::array<double,3> vector(const V3& v){return {v.x,v.y,v.z};}
std::array<double,3> channels(const FinCommands& v){return {v.aileron,v.elevator,v.rudder};}
template<class Values> void array(std::ostream& stream,const Values& values) {
    stream<<'[';bool first=true;
    for(const double value:values){REQUIRE(std::isfinite(value));if(!first)stream<<',';stream<<value;first=false;}
    stream<<']';
}
double property(fsim::sim::JsbsimModel&,const std::string&);

namespace {
struct FeedSample {
    std::array<double,14> state{};
    // PRE contents, unusable, priority, selected; independently observed POST.
    std::vector<std::array<double,5>> tanks;
};
struct FeedProbe {
    std::vector<unsigned> feeds;
    std::vector<double> previous;
    std::vector<fsim::sim::PropertyHandle> diagnostics;
    std::vector<FeedSample> samples;
    double expectedIntegral=0,requestedIntegral=0,burnedLbs=0;
    bool priorStarved=false;
    double maximumSupplyError=0,maximumForceError=0,maximumFlowError=0,maximumTankError=0;
    std::uint64_t checked=0,shortBatches=0,violations=0;
    void advance(fsim::sim::JsbsimModel& model,JSBSim::FGRocket& engine,double dt,
                 std::uint64_t sample,bool retain) {
        constexpr double lb=.45359237,lbf=4.4482216152605;
        const auto propulsion=model.exec().GetPropulsion();
        FeedSample observation;auto& state=observation.state;
        const double raw=diagnostics[0].get(),request=raw/engine.GetIsp()*dt;
        int priority=static_cast<int>(propulsion->GetNumTanks())+1;
        for(const auto id:feeds) {
            const auto tank=propulsion->GetTank(id);
            if(tank->GetSelected()&&tank->GetPriority()>0&&tank->GetPriority()<=static_cast<int>(previous.size())&&
               previous[id]>tank->GetUnusable())priority=std::min(priority,tank->GetPriority());
        }
        unsigned count=0;
        for(const auto id:feeds) {
            const auto tank=propulsion->GetTank(id);
            if(tank->GetSelected()&&tank->GetPriority()==priority&&previous[id]>tank->GetUnusable())++count;
        }
        double expected=0;bool full=count>0;
        for(unsigned id=0;id<propulsion->GetNumTanks();++id) {
            const auto tank=propulsion->GetTank(id);const double before=previous[id];
            double after=before;
            const bool attached=std::find(feeds.begin(),feeds.end(),id)!=feeds.end();
            if(attached&&tank->GetSelected()&&tank->GetPriority()==priority&&before>tank->GetUnusable()) {
                const double share=request/static_cast<double>(count),usable=before-tank->GetUnusable();
                expected+=std::min(share,usable);full=full&&usable>=share;
                after=std::max(tank->GetUnusable(),before-share);
            }
            // Other engines can share the vehicle, so this oracle checks only
            // this engine's explicitly attached feed tanks.
            if(attached)maximumTankError=std::max(maximumTankError,std::abs(after-tank->GetContents())*lb);
            observation.tanks.push_back({before,tank->GetUnusable(),static_cast<double>(tank->GetPriority()),
                                         tank->GetSelected()?1.:0.,tank->GetContents()});
            previous[id]=tank->GetContents();
        }
        if(full)expected=request;
        const double supplied=diagnostics[2].get();
        const double vacuum=model.property("propulsion/engine[0]/vacuum-thrust_lbs").get();
        const double expectedForce=full?raw:expected/dt*engine.GetIsp();
        maximumSupplyError=std::max(maximumSupplyError,std::abs(expected-supplied)*lb);
        maximumFlowError=std::max(maximumFlowError,std::abs(engine.GetFuelFlowRate()-expected/dt)*lb);
        maximumForceError=std::max(maximumForceError,std::abs(vacuum-expectedForce)*lbf);
        if(diagnostics[3].get()!=1.||engine.GetStarved()!=(count==0))++violations;
        if(expected<request)++shortBatches;
        state={static_cast<double>(sample),raw,diagnostics[1].get(),supplied,engine.GetFuelFlowRate(),vacuum,engine.GetIsp(),
               engine.GetStarved()?1.:0.,diagnostics[3].get(),burnedLbs,expected,static_cast<double>(count),
               count?static_cast<double>(priority):0.,priorStarved?1.:0.};
        priorStarved=engine.GetStarved();
        if(retain||expected<request||sample==1)samples.push_back(std::move(observation));
        expectedIntegral+=expected*lb;requestedIntegral+=request*lb;burnedLbs+=supplied;++checked;
    }
};
struct PoweredProbe {
    fsim::sim::JsbsimModel* model=nullptr;
    std::vector<JSBSim::FGRocket*> engines;
    std::vector<fsim::sim::PropertyHandle> vacuum,extra;
    std::vector<std::string> columns;
    std::vector<std::array<double,64>> samples;
    FeedProbe feed;
    std::array<double,2> peakVacuum{},engineFlowIntegral{};
    double nativeDt=0,initialFuel=0,flowIntegral=0,vacuumImpulse=0,inletStart=-1;
    std::uint64_t nativeSamples=0;
    static void observe(void* context,unsigned substep,unsigned substeps) noexcept {
        auto& p=*static_cast<PoweredProbe*>(context);++p.nativeSamples;
        std::array<double,64> row{};std::size_t at=0;
        row[at++]=static_cast<double>(p.nativeSamples);row[at++]=p.model->exec().GetSimTime();
        row[at++]=p.model->energy().fuelKg;
        for(std::size_t k=0;k<p.engines.size();++k) {
            const auto* engine=p.engines[k];const double flow=engine->GetFuelFlowRate()*.45359237;
            const double thrust=p.vacuum[k].get()*4.4482216152605;
            p.peakVacuum[k]=std::max(p.peakVacuum[k],thrust);
            row[at++]=flow;row[at++]=thrust;row[at++]=engine->GetThrust()*4.4482216152605;
            row[at++]=engine->GetIsp();row[at++]=engine->in.Pressure;
            p.engineFlowIntegral[k]+=flow*p.nativeDt;
            p.flowIntegral+=flow*p.nativeDt;p.vacuumImpulse+=thrust*p.nativeDt;
        }
        if(p.inletStart>=0) {
            for(unsigned axis=1;axis<=3;++axis)row[at++]=p.engines.front()->in.PQRi(axis);
            row[at++]=p.engines.front()->in.Temperature*5./9.;
        }
        for(const auto& value:p.extra)row[at++]=value.get();
        p.feed.advance(*p.model,*p.engines.front(),p.nativeDt,p.nativeSamples,
            substep==substeps||(p.inletStart>=0&&row[1]>=p.inletStart-2*p.nativeDt));
        // All finite-inlet transition samples remain native-substep samples;
        // ordinary and pulse trajectories retain each complete outer sample.
        if(substep==substeps||(p.inletStart>=0&&row[1]>=p.inletStart-2*p.nativeDt))p.samples.push_back(row);
    }
};
}

TEST_CASE("Every actual AAM engine executes an independent powered propulsion reference bench",
          "[component-reference-native][component-propulsion-reference][sim-m03]") {
    const auto header=captureHeader(true);
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto population=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    const char* base=std::getenv("FSIM_COMPONENT_REFERENCE_JSON");
    const char* requested=std::getenv("FSIM_COMPONENT_PROPULSION_JSON");
    std::ofstream file;
    if(requested||base){const auto path=requested?std::string(requested):std::string(base)+".propulsion.json";allowedNativePath(path);file.open(path);REQUIRE(file.good());}
    std::ostream& out=(requested||base)?static_cast<std::ostream&>(file):std::cout;
    out<<std::setprecision(17)<<"{\"schema\":\"fsim.sim-m03.propulsion-reference-native.v1\",\"input_epoch_sha256\":\""<<header.epochSha
       <<"\",\"executable\":"<<header.executable.dump()<<",\"runtime_modules\":"<<header.runtimeModules.dump()<<",\"scope\":\"Actual authored native engines; powered500m/s6000m bench; no encounter outcome claim\",\"models\":[";
    bool first=true;
    for(const char* name:cards) {
        INFO(name);const auto found=population.find(name);REQUIRE(found!=population.end());const auto& card=*found->second;
        ProfileV1 profile;std::string error;REQUIRE(readProfileV1(library/(std::string(name)+".json"),card,profile,error));
        const auto* selected=profile.find("propulsion");REQUIRE(selected);
        fsim::sim::JsbsimModel model(.002,std::make_shared<fsim::sim::FlatGround>(0));
        fsim::sim::AircraftSpec source;source.name=card.model.substr(7);source.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
        source.aircraftDir=std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR)/"stores";
        fsim::sim::InitialConditions initial;initial.altitudeMslM=6000;initial.airspeedTrueMs=500;initial.enginesRunning=true;
        REQUIRE(model.load(source,initial));PoweredProbe probe;probe.model=&model;
        probe.nativeDt=model.exec().GetDeltaT();probe.initialFuel=model.energy().fuelKg;
        REQUIRE(probe.initialFuel>0);REQUIRE(probe.nativeDt>0);probe.columns={"native_sample","native_time_s","fuel_kg"};
        auto propulsion=model.exec().GetPropulsion();REQUIRE(propulsion->GetNumEngines()>0);REQUIRE(propulsion->GetNumEngines()<=2);
        for(unsigned k=0;k<propulsion->GetNumEngines();++k) {
            auto* engine=dynamic_cast<JSBSim::FGRocket*>(propulsion->GetEngine(k).get());REQUIRE(engine);
            probe.engines.push_back(engine);
            const auto vacuum=model.property("propulsion/engine["+std::to_string(k)+"]/vacuum-thrust_lbs");REQUIRE(vacuum.valid());probe.vacuum.push_back(vacuum);
            for(const auto* leaf:{"flow_kg_s","vacuum_thrust_n","delivered_thrust_n","isp_s","pressure_psf"})
                probe.columns.push_back("engine"+std::to_string(k)+"_"+leaf);
        }
        const auto feedEngine=probe.engines.front();probe.feed.priorStarved=feedEngine->GetStarved();
        for(unsigned k=0;k<feedEngine->GetNumSourceTanks();++k)probe.feed.feeds.push_back(feedEngine->GetSourceTank(k));
        for(unsigned k=0;k<propulsion->GetNumTanks();++k)probe.feed.previous.push_back(propulsion->GetTank(k)->GetContents());
        REQUIRE(!probe.feed.feeds.empty());
        for(const auto* leaf:{"requested-vacuum-thrust-lbs","requested-fuel-lbs","supplied-fuel-lbs","valid"}) {
            const auto handle=model.property(std::string("fsim/rocket-feed-v1/engine[0]/")+leaf);
            REQUIRE(handle.valid());probe.feed.diagnostics.push_back(handle);
        }
        const auto add=[&](const std::string& path) {
            const auto handle=model.property(path);REQUIRE(handle.valid());probe.extra.push_back(handle);probe.columns.push_back(path);
        };
        double duration=.2;PulsePolicyV1 pulse;std::string pulseError;
        const bool commanded=configurePulseV1(*selected,pulse,pulseError);
        const auto inletVersion=model.property("fsim/ducted-inlet-v2/version");
        const bool inlet=inletVersion.valid()&&inletVersion.get()==2;
        if(commanded) {
            duration=pulse.secondStartS+.15;
            for(const auto* leaf:{"native-age-s","virtual-age-s","second-command","second-ignition-age-s"})add(std::string("fsim/motor-pulse/")+leaf);
        }
        if(inlet) {
            double boost=0;REQUIRE(parameterV1(*selected,"phase_0_duration_s","s",boost,error));
            probe.inletStart=boost;duration=boost+.2;
            for(const auto* column:{"force_epoch_p_rad_s","force_epoch_q_rad_s","force_epoch_r_rad_s","force_epoch_temperature_k"})
                probe.columns.push_back(column);
            for(const auto* leaf:{"residence-s","capture-kg-s","incoming-enthalpy-w","incoming-momentum-x-n","incoming-momentum-y-n","incoming-momentum-z-n",
                "outflow-kg-s","outflow-enthalpy-w","outflow-momentum-x-n","outflow-momentum-y-n","outflow-momentum-z-n","resident-air-mass-kg",
                "resident-enthalpy-j","resident-momentum-x-kg-m-s","resident-momentum-y-kg-m-s","resident-momentum-z-kg-m-s","engine[1]/fuel-kg-s",
                "engine[1]/gross-thrust-n","engine[1]/burned-fuel-kg-s","engine[1]/exit-speed-m-s","engine[1]/total-temperature-k","engine[1]/pressure-ratio",
                "ram-reaction-x-n","ram-reaction-y-n","ram-reaction-z-n","net-propulsion-force-x-n","net-propulsion-force-y-n","net-propulsion-force-z-n"})
                add(std::string("fsim/ducted-inlet-v2/")+leaf);
            for(const auto* path:{"forces/fbx-prop-lbs","forces/fby-prop-lbs","forces/fbz-prop-lbs"})add(path);
            const auto resident=static_cast<unsigned>(property(model,"fsim/ducted-inlet-v2/resident-pointmass-index"));
            add("inertia/pointmass-weight-lbs["+std::to_string(resident)+"]");
            add("fsim/ducted-inlet-v2/engine[1]/requested-fuel-lbs");
            add("fsim/ducted-inlet-v2/engine[1]/reserved-fuel-lbs");
        }
        REQUIRE(probe.columns.size()<=64);
        const auto steps=static_cast<unsigned>(std::ceil(duration/model.dt()));
        const auto retained=static_cast<std::size_t>(steps)+
            (inlet?static_cast<std::size_t>(std::ceil((steps*model.dt()-probe.inletStart)/probe.nativeDt))+4:0)+8;
        probe.samples.reserve(retained);
        fsim::sim::ControlInputs commands;commands.setThrottleAll(1);
        {
            fsim::sim::ScopedNativeStepObserverV1 observing({&model,&probe,&PoweredProbe::observe});
            for(unsigned step=0;step<steps;++step) {
                if(commanded&&static_cast<double>(step)*model.dt()>=pulse.secondStartS+.05)
                    model.property("fsim/motor-pulse/second-command").set(1);
                model.step(commands);
            }
        }
        REQUIRE(!probe.samples.empty());REQUIRE(probe.flowIntegral>0);REQUIRE(probe.vacuumImpulse>0);
        REQUIRE(probe.feed.checked==probe.nativeSamples);REQUIRE(probe.feed.violations==0);
        REQUIRE(probe.feed.maximumSupplyError<1e-11);REQUIRE(probe.feed.maximumTankError<1e-11);
        REQUIRE(probe.feed.maximumFlowError<1e-10);REQUIRE(probe.feed.maximumForceError<1e-8);
        for(std::size_t k=0;k<probe.engines.size();++k)REQUIRE(probe.peakVacuum[k]>0);
        REQUIRE(probe.nativeSamples==static_cast<std::uint64_t>(steps)*static_cast<std::uint64_t>(std::llround(model.dt()/probe.nativeDt)));
        if(!first)out<<',';
        first=false;
        out<<"{\"model\":\""<<name<<"\",\"native_dt_s\":"<<probe.nativeDt<<",\"initial_fuel_kg\":"<<probe.initialFuel
           <<",\"final_fuel_kg\":"<<model.energy().fuelKg<<",\"flow_integral_kg\":"<<probe.flowIntegral
           <<",\"vacuum_impulse_ns\":"<<probe.vacuumImpulse<<",\"native_samples\":"<<probe.nativeSamples<<",\"columns\":[";
        for(std::size_t k=0;k<probe.columns.size();++k){if(k)out<<',';out<<'"'<<probe.columns[k]<<'"';}
        out<<"],\"samples\":[";
        for(std::size_t k=0;k<probe.samples.size();++k) {
            if(k)out<<',';
            out<<'[';
            for(std::size_t j=0;j<probe.columns.size();++j){if(j)out<<',';REQUIRE(std::isfinite(probe.samples[k][j]));out<<probe.samples[k][j];}
            out<<']';
        }
        out<<"],\"engine_flow_integrals_kg\":";array(out,probe.engineFlowIntegral);
        out<<",\"feed_reference\":{\"schema\":\"fsim.sim-m03.native-feed-forward.v1\","
           <<"\"state_columns\":[\"native_sample\",\"requested_vacuum_lbf\",\"requested_lbs\",\"supplied_lbs\","
           <<"\"actual_flow_lbs_s\",\"actual_vacuum_lbf\",\"isp_s\",\"starved\",\"valid\",\"burned_before_lbs\","
           <<"\"expected_supplied_lbs\",\"group_count\",\"group_priority\",\"prior_starved\"],"
           <<"\"tank_columns\":[\"pre_contents_lbs\",\"unusable_lbs\",\"priority\",\"selected\",\"post_contents_lbs\"],"
           <<"\"checked_native_steps\":"<<probe.feed.checked
           <<",\"violations\":"<<probe.feed.violations<<",\"short_batches\":"<<probe.feed.shortBatches
           <<",\"expected_consumed_kg\":"<<probe.feed.expectedIntegral<<",\"requested_kg\":"<<probe.feed.requestedIntegral
           <<",\"maximum_supply_residual_kg\":"<<probe.feed.maximumSupplyError
           <<",\"maximum_tank_residual_kg\":"<<probe.feed.maximumTankError
           <<",\"maximum_flow_residual_kg_s\":"<<probe.feed.maximumFlowError
           <<",\"maximum_force_residual_n\":"<<probe.feed.maximumForceError<<",\"feeds\":[";
        for(std::size_t k=0;k<probe.feed.feeds.size();++k){if(k)out<<',';out<<probe.feed.feeds[k];}
        out<<"],\"samples\":[";
        for(std::size_t k=0;k<probe.feed.samples.size();++k) {
            if(k)out<<',';out<<"{\"state\":";array(out,probe.feed.samples[k].state);out<<",\"tanks\":[";
            for(std::size_t j=0;j<probe.feed.samples[k].tanks.size();++j){if(j)out<<',';array(out,probe.feed.samples[k].tanks[j]);}
            out<<"]}";
        }
        out<<"]}}";
    }
    out<<"]}\n";REQUIRE(out.good());
}
AutopilotInput benchmark(const FinControlConfigV1& configuration) {
    AutopilotInput input;input.massKg=100;input.speedMs=500;input.actuatorLagS=configuration.actuatorLagS;
    input.inertiaKgM2={1,100,100};input.limitsRad.fill(.3);
    input.alphaRad=.03;input.betaRad=-.02;input.ratesRadS={.03,.02,-.015};
    input.demandBody={0,12,-30};input.gravityBody={0,0,9.80665};
    input.pitch={-10000,-2000,-5000,-10000,-2000};
    input.yaw={-10000,2000,5000,-10000,-2000};
    input.rollMomentPerRad=1000;input.rollDamping=-100;
    AeroFeedback measured;
    measured.forceBodyN={0,-10000*input.betaRad,-10000*input.alphaRad};
    measured.momentBodyNm={-100*input.ratesRadS[0],-5000*input.alphaRad-2000*input.ratesRadS[1],
        5000*input.betaRad-2000*input.ratesRadS[2]};
    input.measured=measured;return input;
}
ControlStopsV1 stops(const FinControlConfigV1& configuration) {
    ControlStopsV1 result;result.version=result.actuatorVersion=1;
    result.lowerRad.fill(-.3);result.upperRad.fill(.3);result.channelRateRadS=configuration.actuatorRateRadS;
    result.finRateRadS.assign(3,configuration.actuatorRateRadS);return result;
}
OperatingPointInputV1 operating(const AutopilotInput& plant,const ControlStopsV1& limits) {
    OperatingPointInputV1 input;input.plant=plant;input.stops=limits;input.dtS=.01;input.actuatorSubsteps=1;
    for(unsigned axis=0;axis<3;++axis) {
        OperatingFinV1 fin;fin.minimumRad=-.3;fin.maximumRad=.3;fin.rateRadS=limits.channelRateRadS;
        fin.mixer[axis]=1;fin.anglesRad={-.3,0,.3};
        const double gain=axis==0?1000:-10000;
        for(const double angle:fin.anglesRad){std::array<double,3> moment{};moment[axis]=gain*angle;fin.momentsNm.push_back(moment);}
        input.fins.push_back(fin);
    }
    return input;
}
double property(fsim::sim::JsbsimModel& model,const std::string& name) {
    const auto value=model.property(name);REQUIRE(value.valid());
    REQUIRE(std::isfinite(value.get()));return value.get();
}
void passiveReference(std::ostream& out,fsim::sim::JsbsimModel& model) {
    const auto version=model.property("fsim/passive-rolleron-v1/version");
    out<<'[';
    if(version.valid()&&version.get()==1) {
        const auto count=static_cast<unsigned>(property(model,"fsim/passive-rolleron-v1/count"));
        for(unsigned k=0;k<count;++k) {
            if(k)out<<',';
            const auto prefix="fsim/passive-rolleron-v1/fin-"+std::to_string(k)+"/";
            const auto get=[&](const char* leaf){return property(model,prefix+leaf);};
            fsim::sim::RolleronConfigV1 c;
            c.finId=static_cast<std::uint32_t>(get("fin-id"));c.finIndex=static_cast<std::size_t>(get("fin-index"));
            c.nativeDeflectionSign=get("native-deflection-sign");
            c.hingeAxisBody={get("hinge-axis-body-x"),get("hinge-axis-body-y"),get("hinge-axis-body-z")};
            c.centreFromOriginBodyM={get("centre-from-origin-body-m-x"),get("centre-from-origin-body-m-y"),get("centre-from-origin-body-m-z")};
            c.wheelRadiusM=get("wheel-radius-m");c.captureAreaM2=get("capture-area-m2");c.wheelInertiaKgM2=get("wheel-inertia-kg-m2");
            c.wheelDriveCoefficient=get("wheel-drive-coefficient");c.freeSpinRatio=get("free-spin-ratio");
            c.wheelDragCoefficient=get("wheel-drag-coefficient");c.bearingDampingNms=get("bearing-damping-nms");
            c.hingeInertiaKgM2=get("hinge-inertia-kg-m2");c.hingeStiffnessNmRad=get("hinge-stiffness-nm-rad");
            c.hingeDampingNmsRad=get("hinge-damping-nms-rad");c.hingeAerodynamicStiffnessM3=get("hinge-aerodynamic-stiffness-m3");
            c.limitRad=get("limit-rad");
            // The passive rate-rad-s port is live hinge velocity after step().
            // The generic fin metadata retains the immutable physical limit.
            c.rateRadS=property(model,"fsim/missile-control/fin-"+std::to_string(c.finIndex)+"-rate-rad_s");
            fsim::sim::PassiveRolleronV1 mechanism({c});
            fsim::sim::RolleronInputV1 input;input.dtS=.001;input.densityKgM3=.7;
            input.airVelocityBodyMs={500,0,0};input.angularRateInertialBodyRadS={.001,0,0};
            fsim::sim::RolleronOutputV1 result;
            for(unsigned n=0;n<10;++n){result=mechanism.advance(input);REQUIRE(result.valid);}
            out<<"{\"config\":";
            array(out,std::array<double,14>{c.nativeDeflectionSign,c.wheelRadiusM,c.captureAreaM2,c.wheelInertiaKgM2,
                c.wheelDriveCoefficient,c.freeSpinRatio,c.wheelDragCoefficient,c.bearingDampingNms,
                c.hingeInertiaKgM2,c.hingeStiffnessNmRad,c.hingeDampingNmsRad,c.hingeAerodynamicStiffnessM3,c.limitRad,c.rateRadS});
            out<<",\"state\":";array(out,std::array<double,3>{result.fins.front().wheelRadS,result.fins.front().angleRad,result.fins.front().rateRadS});
            out<<",\"time_s\":0.01,\"speed_ms\":500,\"density_kg_m3\":0.7,\"roll_rad_s\":0.001}";
        }
    }
    out<<']';
}
void nativePlant(std::ostream& out,const std::string& modelName) {
    fsim::sim::JsbsimModel model(.002,std::make_shared<fsim::sim::FlatGround>(0));
    fsim::sim::AircraftSpec source;source.name=modelName;source.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
    source.aircraftDir=std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR)/"stores";
    fsim::sim::InitialConditions initial;initial.altitudeMslM=6000;initial.airspeedTrueMs=500;
    initial.enginesRunning=false;REQUIRE(model.load(source,initial));
    const double countValue=property(model,"fsim/missile-control/fin-count");
    REQUIRE(countValue>0);REQUIRE(countValue<=32);REQUIRE(countValue==std::floor(countValue));
    const auto count=static_cast<unsigned>(countValue);
    fsim::sim::ControlInputs command;
    for(unsigned warm=0;warm<4;++warm)model.step(command);
    out<<"{\"dt_s\":0.002,\"fins\":[";
    for(unsigned fin=0;fin<count;++fin) {
        if(fin)out<<',';
        const auto prefix="fsim/missile-control/fin-"+std::to_string(fin)+"-";
        const auto kind=model.property(prefix+"actuation-kind-v1");
        out<<"{\"index\":"<<fin<<",\"kind\":"<<(kind.valid()?kind.get():1)
           <<",\"minimum\":"<<property(model,prefix+"min-rad")<<",\"maximum\":"<<property(model,prefix+"max-rad")
           <<",\"rate\":"<<property(model,prefix+"rate-rad_s")<<'}';
    }
    out<<"],\"samples\":[";
    for(unsigned step=0;step<24;++step) {
        const double sign=step<12?1.:-1.;command.aileron=.4*sign;command.elevator=-.3*sign;command.rudder=.2*sign;
        model.step(command);if(step)out<<',';
        out<<"{\"command\":";array(out,std::array<double,3>{command.aileron,command.elevator,command.rudder});
        out<<",\"response\":";std::array<double,3> response{};unsigned axis=0;
        for(const auto* channel:{"aileron","elevator","rudder"})
            response[axis++]=property(model,std::string("fcs/")+channel+"-response-rad");
        array(out,response);out<<",\"fins\":[";
        for(unsigned fin=0;fin<count;++fin){if(fin)out<<',';out<<property(model,"fcs/fin-"+std::to_string(fin)+"-delta-rad");}
        out<<"]}";
    }
    out<<"],\"passive_reference\":";passiveReference(out,model);out<<'}';
}
}

TEST_CASE("All actual AAM card configurations execute numeric guidance and control method references",
          "[component-reference-native][sim-m03]") {
    const auto header=captureHeader();
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto population=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    std::ofstream file;const char* output=std::getenv("FSIM_COMPONENT_REFERENCE_JSON");
    if(output){allowedNativePath(output);file.open(output);REQUIRE(file.good());}
    std::ostream& stream=output?static_cast<std::ostream&>(file):std::cout;
    stream<<std::setprecision(17)<<"{\"schema\":\"fsim.sim-m03.component-native.v1\",\"input_epoch_sha256\":\""<<header.epochSha
          <<"\",\"executable\":"<<header.executable.dump()<<",\"runtime_modules\":"<<header.runtimeModules.dump()<<','
          <<"\"scope\":\"Actual C++ APIs and card selection; synthetic SI plant, not a native flight\",\"models\":[";
    bool first=true;unsigned count=0;
    for(const char* name:cards) {
        INFO(name);const auto found=population.find(name);REQUIRE(found!=population.end());
        const auto& card=*found->second;ProfileV1 profile;std::string error;
        REQUIRE(readProfileV1(library/(std::string(name)+".json"),card,profile,error));
        Spec spec;REQUIRE(configureCompositionV1(card,profile,spec,spec,error));
        PlantAdapterV1 selected;
        REQUIRE(configurePlantV1(*profile.find("autopilot"),*profile.find("allocator"),*profile.find("actuator"),
            *profile.find("propulsion"),*profile.find("aerodynamics"),card,selected,error));
        const auto& config=selected.control;
        SeekerFrameV2 frame;frame.timeS=frame.ageS=1;frame.dtS=.01;
        frame.velocity={550,30,-20};frame.up={0,0,1};
        SeekerTrackV2 track;track.localId=1;track.sampleTimeS=1;track.direction={1,0,0};
        track.directionRate={0,.02,-.01};track.rateAvailable=true;
        if(card.seeker=="active_radar"){track.rangeM=5000;track.rangeRateMs=-450;}
        SeekerResultV2 measured;measured.valid=measured.active=measured.tracking=true;measured.track=track;
        MeasuredPnGuidanceV2 guidance(spec);const auto guided=guidance.updateMeasured(frame,measured);
        REQUIRE(guided.tracking);REQUIRE(guided.phase==WeaponPhase::Homing);
        const auto plant=benchmark(config);const auto limits=stops(config);
        const auto cascade=finCommands(plant,config,limits);REQUIRE(cascade.available);
        const auto aware=actuatorAwareCommands(plant,config,limits);REQUIRE(aware.available);
        const auto opInput=operating(plant,limits);
        const auto prediction=operatingPointPredictionV1(opInput,{{.5,-.4,.3}});
        REQUIRE(prediction.commands.available);REQUIRE(prediction.predictedFinRad.size()==3);
        const auto allocation=operatingPointCommandsV1(opInput,config);REQUIRE(allocation.commands.available);
        OperatingPointResultV1 selectedOperating;
        if(selected.implementations[0]==3) {
            OperatingPointAutopilotV1 provider(card.model,config);
            selectedOperating=provider.commandsOperatingPointV1(opInput);
            REQUIRE(selectedOperating.commands.available);
        }
        const auto selectedCommands=selected.implementations[0]==3?selectedOperating.commands:
            selected.implementations[0]==2?aware:cascade;
        JsbsimAuthoredFinMixerV1 mixer(card.model);const auto mixed=mixer.allocate(cascade,limits);
        REQUIRE(mixed.available);CHECK(mixed.aileron==cascade.aileron);CHECK(mixed.elevator==cascade.elevator);
        CHECK(mixed.rudder==cascade.rudder);
        const std::array<double,6> load{{12,3,40,2,-4,6}};const std::array<double,3> offset{{.1,-.2,.05}};
        const auto moment=operatingPointBodyMomentV1(load,.17,-.09,offset);
        if(!first)stream<<',';
        first=false;
        stream<<"{\"model\":\""<<name<<"\",\"component\":\"guidance\",\"autopilot_code\":"<<selected.implementations[0]
              <<",\"navigation_gain\":"<<spec.navigationGain<<",\"max_g\":"<<spec.maxG
              <<",\"lag_s\":"<<config.actuatorLagS<<",\"rate_rad_s\":"<<config.actuatorRateRadS
              <<",\"guidance_acceleration\":";array(stream,vector(guided.acceleration));
        stream<<",\"cascade\":";array(stream,channels(cascade));
        stream<<",\"actuator_aware\":";array(stream,channels(aware));
        stream<<",\"operating_commands\":";array(stream,channels(allocation.commands));
        stream<<",\"operating_requested_moment\":";array(stream,allocation.requestedMomentNm);
        stream<<",\"operating_predicted_moment\":";array(stream,allocation.predictedMomentNm);
        stream<<",\"operating_residual\":"<<allocation.residualAccelerationNorm;
        stream<<",\"control_benchmark_scope\":\"Synthetic signed SI control plant; card-selected parameters, not model-native flight\""
              <<",\"control_benchmark_mass_kg\":"<<plant.massKg<<",\"control_benchmark_speed_m_s\":"<<plant.speedMs
              <<",\"operating_legacy_entry\":\"operatingPointCommandsV1\""
              <<",\"operating_legacy_target_stage\":\"instantaneous_virtual_angle\""
              <<",\"selected_control_entry\":\""<<(selected.implementations[0]==3?
                  "OperatingPointAutopilotV1::commandsOperatingPointV1/operatingPointInputCommandsV1":
                  selected.implementations[0]==2?"actuatorAwareCommands":"finCommands")<<'"'
              <<",\"selected_control_target_stage\":\""<<(selected.implementations[0]==3?
                  "finite_fcs_input_endpoint":"normalized_fcs_input")<<'"';
        stream<<",\"selected_control_commands\":";array(stream,channels(selectedCommands));
        stream<<",\"selected_operating_point\":";
        if(selected.implementations[0]==3) {
            stream<<"{\"commands\":";array(stream,channels(selectedOperating.commands));
            stream<<",\"requested_moment\":";array(stream,selectedOperating.requestedMomentNm);
            stream<<",\"predicted_moment\":";array(stream,selectedOperating.predictedMomentNm);
            stream<<",\"residual\":"<<selectedOperating.residualAccelerationNorm<<'}';
        } else stream<<"null";
        stream<<",\"predicted_fixed_command_fins\":";array(stream,prediction.predictedFinRad);
        stream<<",\"body_cg_moment\":";array(stream,moment);
        stream<<",\"native_plant\":";nativePlant(stream,card.model.substr(7));
        PulsePolicyV1 pulse;std::string pulseError;
        if(configurePulseV1(*profile.find("propulsion"),pulse,pulseError)) {
            PulseManagerV1 manager(pulse);stream<<",\"pulse_start_s\":"<<pulse.secondStartS<<",\"pulse_states\":[";
            const std::array<double,4> ages{{0,pulse.secondStartS,pulse.secondStartS+.01,pulse.secondStartS+.02}};
            for(unsigned k=0;k<ages.size();++k) {
                PulseInputV1 input;input.timeS=input.nativeAgeS=ages[k];input.speedMs=k<2?2*pulse.reserveSpeedMs:.5*pulse.reserveSpeedMs;input.fuelKg=1;
                const auto decision=manager.update(input);REQUIRE(decision.valid);
                if(k)stream<<',';
                stream<<static_cast<unsigned>(decision.state);
            }
            stream<<']';
        }
        stream<<'}';++count;
    }
    stream<<"],\"model_count\":"<<count<<"}\n";REQUIRE(count==24);REQUIRE(stream.good());
}
