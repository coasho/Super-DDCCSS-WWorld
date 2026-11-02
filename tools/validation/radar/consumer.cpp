// Standalone installed C++ consumer: native measurement -> fusion -> Agent.
#include <fsim/World.h>
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <filesystem>
#include <cmath>
#include <iostream>
#include <stdexcept>

static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(int argc,char** argv)try {
    check(argc==2,"installed SDK prefix required");
    wchar_t loaded[32768];const auto size=GetModuleFileNameW(GetModuleHandleW(L"libfsim.dll"),loaded,32768);
    check(size>0&&size<32768,"cannot locate loaded SDK");
    check(std::filesystem::equivalent(std::filesystem::path(std::wstring(loaded,size)),std::filesystem::path(argv[1])/"bin/libfsim.dll"),"stale/outside SDK loaded");
    fsim::WorldOptions options;options.name="sim-m01-installed-cpp";options.workers=1;options.pinWorkers=false;options.publish=false;options.seed=3;
    fsim::World world(options);fsim::VehicleSpec spec;spec.type="jsbsim:f16c";spec.name="lead";
    spec.initial.latitudeDeg=40;spec.initial.longitudeDeg=0;spec.initial.headingDeg=90;
    spec.initial.altitudeMslM=5000;spec.initial.airspeedTrueMs=200;
    const auto lead=world.createVehicle(spec);spec.name="target";spec.initial.headingDeg=270;
    spec.initial.longitudeDeg=8000/(6371000*std::cos(40*3.14159265358979323846/180))*180/3.14159265358979323846;
    const auto target=world.createVehicle(spec);(void)target;
    auto agent=world.createAgent(lead,"lead");fsim::ms::StateCommand operate;operate.state=fsim::ms::PayloadState::Operate;
    check(agent.submit(operate).accepted(),"operate refused");world.step();
    fsim::ms::AmtiCommand search;search.azimuthHalfWidthRad=.05;search.bars=2;
    check(agent.submit(search).accepted(),"search refused");
    for(int i=0;i<300&&agent.fusedTracksV1().empty();++i)world.step();
    const auto picture=agent.fusedTracksV1();check(!picture.empty(),"no native fused measurement");
    check(agent.fusedTracksV1().size()==picture.size(),"read consumed report");
    check(agent.fusionStatusV1().rejectedInputs==0,"native measurements rejected");
    for(const auto& row:picture) {
        check(!row.identityKnown&&!row.sources.empty(),"unjustified identity/missing provenance");
        for(double value:row.stateEcef)check(std::isfinite(value),"nonfinite state");
        for(double value:row.covarianceEcef)check(std::isfinite(value),"nonfinite covariance");
    }
    std::cout<<"installed C++ native fusion: "<<picture.size()<<" entities; repeated read and provenance passed\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
