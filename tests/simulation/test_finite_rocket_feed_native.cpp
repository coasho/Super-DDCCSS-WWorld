// Mechanism regressions only. Actual24-model evidence remains the separately
// captured normal authored population; this synthetic engine is not equipment.
#include "simulation/flight/JsbsimModel.h"
#include "simulation/flight/GroundProvider.h"
#include "FGFDMExec.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGTank.h"
#include "models/propulsion/FGRocket.h"
#include "input_output/FGPropertyManager.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

namespace {
constexpr double dt=.002,isp=250,vacuumLbf=1000,toleranceLbs=1e-11;
constexpr const char* policy="fsim/rocket-feed-v1/engine[0]/";
std::atomic<unsigned> sequence{0};
struct Fixture {
    std::filesystem::path directory;
    fsim::sim::JsbsimModel model{dt,std::make_shared<fsim::sim::FlatGround>(0)};
    std::shared_ptr<JSBSim::FGPropulsion> propulsion;
    JSBSim::FGRocket* engine=nullptr;
    explicit Fixture(bool optIn=true,bool duplicate=false) {
        directory=std::filesystem::path("D:/ENV/tmp")/(
            "fsim-finite-feed-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+
            "-"+std::to_string(sequence.fetch_add(1)));
        const auto aircraft=directory/"supply_fixture";
        std::filesystem::create_directories(aircraft/"Engines");
        std::ostringstream xml;
        xml<<"<fdm_config name=\"finite feed mechanism fixture\" version=\"2.0\">"
            "<metrics><wingarea unit=\"FT2\">1</wingarea><wingspan unit=\"FT\">1</wingspan><chord unit=\"FT\">1</chord>"
            "<location name=\"AERORP\" unit=\"IN\"><x>0</x><y>0</y><z>0</z></location></metrics>"
            "<mass_balance><ixx unit=\"SLUG*FT2\">10</ixx><iyy unit=\"SLUG*FT2\">10</iyy><izz unit=\"SLUG*FT2\">10</izz>"
            "<emptywt unit=\"LBS\">1000</emptywt><location name=\"CG\" unit=\"IN\"><x>0</x><y>0</y><z>0</z></location></mass_balance>"
            "<ground_reactions/><propulsion><engine file=\"fixture_motor\"><feed>0</feed><feed>"<<(duplicate?0:1)<<"</feed><feed>2</feed>"
            "<thruster file=\"fixture_nozzle\"><location unit=\"IN\"><x>0</x><y>0</y><z>0</z></location>"
            "<orient unit=\"DEG\"><roll>0</roll><pitch>0</pitch><yaw>0</yaw></orient></thruster></engine>";
        for(unsigned k=0;k<3;++k)xml<<"<tank type=\"FUEL\"><location unit=\"IN\"><x>0</x><y>0</y><z>0</z></location>"
            "<capacity unit=\"LBS\">10</capacity><contents unit=\"LBS\">10</contents></tank>";
        xml<<"</propulsion><flight_control name=\"native supply policy\">";
        if(optIn)xml<<"<property value=\"1\">"<<policy<<"version</property>";
        xml<<"</flight_control><aerodynamics>";
        for(const auto* axis:{"DRAG","SIDE","LIFT","ROLL","PITCH","YAW"})
            xml<<"<axis name=\""<<axis<<"\"><function name=\"aero/supply-fixture-zero-"<<axis<<"\"><value>0</value></function></axis>";
        xml<<"</aerodynamics></fdm_config>";
        const auto write=[](const std::filesystem::path& path,const std::string& value){std::ofstream file(path);REQUIRE(file.good());file<<value;REQUIRE(file.good());};
        write(aircraft/"supply_fixture.xml",xml.str());
        write(aircraft/"Engines/fixture_motor.xml","<rocket_engine name=\"disclosed constant source\"><isp>250</isp><builduptime>0</builduptime>"
            "<thrust_table type=\"internal\"><tableData>0 1000\n10000 1000</tableData></thrust_table></rocket_engine>");
        write(aircraft/"Engines/fixture_nozzle.xml","<nozzle name=\"zero pressure area reference\"><area unit=\"FT2\">0</area></nozzle>");
        fsim::sim::InitialConditions initial;initial.altitudeMslM=6000;initial.airspeedTrueMs=500;initial.enginesRunning=true;
        REQUIRE(model.load({"supply_fixture",FSIM_TEST_JSBSIM_ROOT,directory},initial));
        propulsion=model.exec().GetPropulsion();REQUIRE(propulsion->GetNumTanks()==3);REQUIRE(propulsion->GetNumEngines()==1);
        engine=dynamic_cast<JSBSim::FGRocket*>(propulsion->GetEngine(0).get());REQUIRE(engine);
        step();REQUIRE(vacuum()==vacuumLbf);REQUIRE(engine->GetIsp()==isp);
    }
    void step(){fsim::sim::ControlInputs input;input.setThrottleAll(1);model.step(input);}
    double vacuum(){const auto value=model.property("propulsion/engine/vacuum-thrust_lbs");REQUIRE(value.valid());return value.get();}
    double status(){const auto value=model.property(std::string(policy)+"valid");REQUIRE(value.valid());return value.get();}
    void state(const std::array<double,3>& contents,const std::array<int,3>& priority,
               const std::array<double,3>& unusable={}) {
        for(unsigned k=0;k<3;++k) {
            const auto tank=propulsion->GetTank(k);tank->SetPriority(priority[k]);
            tank->SetUnusableVolume(unusable[k]/tank->GetDensity());tank->SetContents(contents[k]);
        }
    }
};

void supplied(Fixture& f,const std::array<double,3>& contents,const std::array<int,3>& priorities,
              const std::array<double,3>& unusable={}) {
    // Independent forward batch law from PRE-state and disclosed source demand.
    // No expected value is formed from an observed final fuel difference.
    f.state(contents,priorities,unusable);int first=4;unsigned count=0;
    for(unsigned k=0;k<3;++k)if(priorities[k]>0&&priorities[k]<=3&&contents[k]>unusable[k])first=std::min(first,priorities[k]);
    for(unsigned k=0;k<3;++k)if(priorities[k]==first&&contents[k]>unusable[k])++count;
    const double requested=vacuumLbf/isp*dt;double actual=0;auto after=contents;
    for(unsigned k=0;k<3;++k)if(priorities[k]==first&&contents[k]>unusable[k]) {
        const double debit=std::min(requested/static_cast<double>(count),contents[k]-unusable[k]);
        after[k]-=debit;actual+=debit;
    }
    f.step();
    REQUIRE(std::abs(f.vacuum()-actual*isp/dt)<1e-8);
    REQUIRE(std::abs(f.engine->GetFuelFlowRate()-actual/dt)<1e-10);
    REQUIRE(f.status()==1);
    for(unsigned k=0;k<3;++k)REQUIRE(std::abs(f.propulsion->GetTank(k)->GetContents()-after[k])<toleranceLbs);
    REQUIRE(std::abs(f.vacuum()*dt-actual*isp)<1e-11);
}
}

TEST_CASE("Finite rocket feed supplies exactly the native equal-share priority batch before force",
          "[finite-rocket-feed][native-method]") {
    SECTION("fractional single feed") {Fixture f;supplied(f,{{.002,1,1}},{{1,0,0}});}
    SECTION("unequal same-priority shares preserve actual debit") {Fixture f;supplied(f,{{.001,.1,1}},{{1,1,0}});}
    SECTION("lower-priority reserve cannot be borrowed") {Fixture f;supplied(f,{{1,.002,1}},{{2,1,0}});}
    SECTION("empty first group falls through to next native priority") {Fixture f;supplied(f,{{0,.1,1}},{{1,2,0}});}
    SECTION("unusable quantity remains untouched") {Fixture f;supplied(f,{{.102,1,1}},{{1,0,0}},{{.1,0,0}});}
    SECTION("deselected tanks cannot supply an impulse") {Fixture f;supplied(f,{{1,1,1}},{{0,0,0}});}
    SECTION("zero contents cannot supply an impulse") {Fixture f;supplied(f,{{0,0,0}},{{1,1,1}});}
    SECTION("full supply retains the original authored force") {Fixture f;supplied(f,{{1,1,1}},{{1,1,1}});}
}

TEST_CASE("Finite rocket feed refuses invalid explicit policy without draining native fuel",
          "[finite-rocket-feed][native-method]") {
    Fixture f;f.state({{1,1,1}},{{1,1,1}});
    SECTION("unknown policy version") {auto version=f.model.property(std::string(policy)+"version");REQUIRE(version.valid());version.set(2);}
    SECTION("nonfinite effective Isp") {f.engine->SetIsp(std::numeric_limits<double>::quiet_NaN());}
    f.step();REQUIRE(f.vacuum()==0);REQUIRE(f.engine->GetFuelFlowRate()==0);REQUIRE(f.status()==0);
    for(unsigned k=0;k<3;++k)REQUIRE(f.propulsion->GetTank(k)->GetContents()==1);
}

TEST_CASE("Absent finite rocket policy retains the established legacy final-batch behavior",
          "[finite-rocket-feed][legacy-compatibility]") {
    Fixture f(false);f.state({{.001,.1,1}},{{1,1,0}});f.step();
    REQUIRE(f.vacuum()==vacuumLbf);
    REQUIRE(std::abs(f.propulsion->GetTank(0)->GetContents())<toleranceLbs);
    REQUIRE(std::abs(f.propulsion->GetTank(1)->GetContents()-.096)<toleranceLbs);
    REQUIRE_FALSE(f.model.property(std::string(policy)+"valid").valid());
}
