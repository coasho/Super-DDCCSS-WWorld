// A standalone installed-header consumer. The producer copies the reviewed
// public-only behavioral fixture beside this wrapper as functional_cpp.inc.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#define main m03_functional_main
#include "functional_cpp.inc"
#undef main

int main(int argc,char** argv) {
    if(argc!=3)return 2;
    const auto prefix=std::filesystem::weakly_canonical(argv[1]);
    auto data=(prefix/"share/flightsim/jsbsim").string();
    char* functionalArgs[]={argv[0],data.data()};
    const auto status=m03_functional_main(2,functionalArgs);
    if(status)return status;
    std::ofstream identity(argv[2]);if(!identity)return 3;
    identity<<"{\"modules\":[";
    bool first=true;
    for(const auto* name:{"libfsim.dll","libJSBSim.dll"}) {
        const auto module=GetModuleHandleA(name);if(!module)return 4;
        char path[32768];const auto length=GetModuleFileNameA(module,path,32768);
        if(!length||length>=32768)return 5;
        const auto actual=std::filesystem::weakly_canonical(path);
        if(!std::filesystem::equivalent(actual,prefix/"bin"/name))return 6;
        if(!first)identity<<',';
        first=false;identity<<"{\"name\":"<<std::quoted(name)<<",\"path\":"<<std::quoted(actual.generic_string())<<'}';
    }
    identity<<"]}\n";
    return identity?0:7;
}
