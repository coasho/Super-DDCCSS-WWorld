// Generic, bounded diagnostic recorder. It does not implement flight physics
// or award acceptance; all observations come from the native World and FDM.
#include "simulation/runtime/world/World.h"
#include "simulation/flight/JsbsimModel.h"
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc,char** argv) {
    using namespace fsim;
    try {
        std::string model,output,aircraftDirectory;
        bool qbarComparison = false;
        std::map<std::string,double> values={{"--height",3000},{"--forward-ms",40},{"--down-ms",0},
            {"--settle-s",2},{"--first-s",16},{"--second-s",15},{"--pitch-first",.6},{"--pitch-second",-.8},
            {"--collective",0},{"--engines-running",1},{"--hsa-speed",-1}};
        for(int i=1;i<argc;i+=2) {
            const std::string key=argv[i];if(i+1>=argc)throw std::runtime_error("Missing option value");
            const std::string value=argv[i+1];
            if(key=="--model")model=value;
            else if(key=="--qbar-model") { model=value; qbarComparison=true; }
            else if(key=="--aircraft-dir")aircraftDirectory=value;
            else if(key=="--output")output=value;
            else if(values.count(key)) {
                std::size_t read=0;const double number=std::stod(value,&read);
                if(read!=value.size() || !std::isfinite(number))throw std::runtime_error("Expected finite numeric option");
                values[key]=number;
            } else throw std::runtime_error("Unknown option: "+key);
        }
        if(model.empty() || output.empty() || model.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
            throw std::runtime_error("Required: --model NAME --output CSV; optional: --height --forward-ms --down-ms --settle-s --first-s --second-s --pitch-first --pitch-second --collective --engines-running --hsa-speed (-1 for raw controls)");
        for(const auto& part:std::filesystem::path(output)) {
            auto text=part.string();std::transform(text.begin(),text.end(),text.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
            if(text==".codex")throw std::runtime_error("Excluded output directory");
        }
        if (qbarComparison) {
            if (aircraftDirectory.empty()) throw std::runtime_error("Qbar diagnosis requires --aircraft-dir");
            const auto start=std::chrono::steady_clock::now();
            auto ground=std::make_shared<sim::FlatGround>(0);
            sim::JsbsimModel ordinary(1.0/120.0,ground),thin(1.0/120.0,ground);
            sim::InitialConditions initial;
            initial.altitudeMslM=6000;initial.airspeedTrueMs=350;initial.headingDeg=90;initial.enginesRunning=false;
            sim::AircraftSpec aircraft{model,FSIM_PROBE_JSBSIM_ROOT,std::filesystem::path(aircraftDirectory)};
            if (!ordinary.load(aircraft,initial) || !thin.load(aircraft,initial)) throw std::runtime_error("Cannot load diagnostic native plants");
            thin.setAtmosphere(288.15,101325.0/2);
            const auto parent=std::filesystem::path(output).parent_path();if(!parent.empty())std::filesystem::create_directories(parent);
            std::ofstream file(output,std::ios::binary);if(!file)throw std::runtime_error("Cannot open qbar output");
            const std::vector<std::string> requested={"simulation/dt","fsim/integration-v1/substeps","simulation/sim-time-sec",
                "aero/alpha-rad","aero/beta-rad","velocities/mach","aero/qbar-psf","atmosphere/rho-slugs_ft3",
                "velocities/u-fps","velocities/v-fps","velocities/w-fps","fcs/elevator-pos-rad",
                "aero/force/fsim-fin-x-lbs","aero/force/fsim-fin-y-lbs","aero/force/fsim-fin-z-lbs",
                "aero/moment/fsim-fin-l-lbsft","aero/moment/fsim-fin-m-lbsft","aero/moment/fsim-fin-n-lbsft"};
            file<<std::setprecision(17)<<"plant,stage";for(const auto& name:requested)file<<','<<name;file<<'\n';
            auto row=[&](sim::JsbsimModel& native,const char* name,const char* stage) {
                file<<name<<','<<stage;
                for(const auto& property:requested) {
                    const auto handle=native.property(property);if(!handle.valid())throw std::runtime_error("Missing diagnostic property: "+property);
                    const double value=handle.get();if(!std::isfinite(value))throw std::runtime_error("Nonfinite native diagnostic property");
                    file<<','<<value;
                }
                file<<'\n';
            };
            row(ordinary,"ordinary","before");row(thin,"thin","before");
            sim::ControlInputs input;input.gearDown=0;input.elevator=.05;
            ordinary.step(input);thin.step(input);
            row(ordinary,"ordinary","after");row(thin,"thin","after");
            file.close();std::cout<<"schema=fsim.native-qbar-diagnostic.v1 model="<<model
                <<" elapsed_seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<"\n";
            return 0;
        }
        const double total=values.at("--settle-s")+values.at("--first-s")+values.at("--second-s");
        if(values.at("--settle-s")<0 || values.at("--first-s")<0 || values.at("--second-s")<0 || total<=0 || total>600 ||
           std::abs(values.at("--pitch-first"))>1 || std::abs(values.at("--pitch-second"))>1 ||
           values.at("--collective")<0 || values.at("--collective")>1 || values.at("--hsa-speed") < -1 ||
           (values.at("--engines-running")!=0 && values.at("--engines-running")!=1))throw std::runtime_error("Probe exceeds bounded input domain");
        const auto start=std::chrono::steady_clock::now();
        session::WorldOptions options;options.name="native-flight-probe-v1";options.publish=false;
        options.workers=1;options.pinWorkers=false;options.jsbsimRoot=FSIM_PROBE_JSBSIM_ROOT;
        session::World world(options);session::VehicleSpec spec;spec.name="probe";spec.type="jsbsim:"+model;
        spec.initial.altitudeMslM=values.at("--height");spec.initial.airspeedTrueMs=0;
        spec.initial.velocityNedMs[0]=values.at("--forward-ms");spec.initial.velocityNedMs[1]=0;spec.initial.velocityNedMs[2]=values.at("--down-ms");
        spec.initial.enginesRunning=values.at("--engines-running")==1;
        const auto id=world.createVehicle(spec);if(!id)throw std::runtime_error("Native model creation failed");
        auto* plant=world.model(id);std::map<std::string,PropertyHandle> properties;
        const std::vector<std::string> requested={"fcs/"+model+"/rotor-rpm","propulsion/engine[0]/rotor-rpm",
            "propulsion/engine[0]/torque-lbsft","fcs/"+model+"/power-demand","fcs/"+model+"/power",
            "propulsion/engine[0]/collective-ctrl-rad","propulsion/engine[0]/longitudinal-ctrl-rad",
            "forces/fbx-aero-lbs","forces/fby-aero-lbs","forces/fbz-aero-lbs",
            "fsim/rotor-airframe-v1/drag","fsim/rotor-airframe-v1/lift",
            "fsim/rotor-passive-v1/x-lbs","fsim/rotor-passive-v1/y-lbs","fsim/rotor-passive-v1/z-lbs"};
        const auto catalog=plant->properties("");
        for(const auto& name:requested) {
            auto canonical=name;std::size_t at=0;
            while((at=canonical.find("[0]",at))!=std::string::npos)canonical.erase(at,3);
            if(std::any_of(catalog.begin(),catalog.end(),[&](const auto& entry){return entry.first==canonical;}))properties.emplace(name,plant->property(name));
        }
        const auto parent=std::filesystem::path(output).parent_path();if(!parent.empty())std::filesystem::create_directories(parent);
        std::ofstream file(output,std::ios::binary);if(!file)throw std::runtime_error("Cannot open CSV output");
        file<<std::setprecision(17)<<"time_s,altitude_m,agl_m,u_ms,v_ms,w_ms,north_ms,east_ms,down_ms,roll_rad,pitch_rad,p_rad_s,q_rad_s,r_rad_s,mass_kg,fuel_kg,starved,crashed,diverged,command_pitch_rad";
        for(const auto& name:requested){file<<','<<name;}
        file<<'\n';
        const double dt=world.dt()*world.frameSkip();unsigned samples=0;int phase=-1;
        for(unsigned k=0;k<static_cast<unsigned>(std::ceil(total/dt));++k) {
            const double time=k*dt;const int next=time<values.at("--settle-s")?0:time<values.at("--settle-s")+values.at("--first-s")?1:2;
            if(next!=phase && next) {
                if(values.at("--hsa-speed")>=0) {
                    control::HsaCommand command;command.headingRad=0;command.speed=values.at("--hsa-speed");
                    command.speedReference=static_cast<double>(control::SpeedReference::TrueAirspeed);
                    if(!world.submit(id,command).accepted())throw std::runtime_error("Native HSA request refused");
                } else {
                    control::ActuatorCommand command;command.elevator=values.at(next==1?"--pitch-first":"--pitch-second");
                    command.aileron=command.rudder=0;command.throttle=values.at("--collective");
                    if(!world.submit(id,command).accepted())throw std::runtime_error("Native actuator request refused");
                }
            }
            phase=next;world.step();const auto& state=*world.vehicleState(id);const auto energy=plant->energy();
            const auto demand=world.commandState(id);
            const auto number=[&](double value){if(std::isfinite(value))file<<value;};
            number(state.simTime);for(double value:{state.altitudeMslM,state.altitudeAglM,state.velocityBodyMs[0],state.velocityBodyMs[1],state.velocityBodyMs[2],
                state.velocityNedMs[0],state.velocityNedMs[1],state.velocityNedMs[2],state.eulerRad[0],state.eulerRad[1],
                state.angularRateBodyRadS[0],state.angularRateBodyRadS[1],state.angularRateBodyRadS[2],energy.massKg,energy.fuelKg,
                static_cast<double>(energy.starved),static_cast<double>(state.crashed),static_cast<double>(state.diverged),demand.pitchRad}){file<<',';number(value);}
            for(const auto& name:requested){file<<',';const auto entry=properties.find(name);if(entry!=properties.end())number(entry->second.get());}
            file<<'\n';++samples;
            if(state.crashed || state.diverged)break;
            if(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()>1200)throw std::runtime_error("Probe wall timeout (1200 seconds)");
        }
        file.close();std::cout<<"schema=fsim.native-flight-probe.v1 model="<<model<<" samples="<<samples
            <<" elapsed_seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<"\n";
        return 0;
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
