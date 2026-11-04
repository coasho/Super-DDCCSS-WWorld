#include "simulation/mission_systems/weapon/PulseManagerV1.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "simulation/flight/JsbsimModel.h"
#include "simulation/flight/GroundProvider.h"
#include <cmath>
#include <limits>
#include <chrono>
#include <filesystem>
#include <fstream>

TEST_CASE("Dual pulse energy management reserves the second pulse until native eligibility", "[weapon-pulse-energy]") {
    fsim::ms::weapon::PulsePolicyV1 policy;
    policy.version=1;policy.secondStartS=10;policy.holdLimitS=30;
    policy.reserveSpeedMs=450;policy.terminalLeadS=6;
    fsim::ms::weapon::PulseManagerV1 manager(policy);
    fsim::ms::weapon::PulseInputV1 input;
    input.timeS=9;input.nativeAgeS=9;input.speedMs=300;input.fuelKg=20;
    const auto early=manager.update(input);REQUIRE(early.valid);CHECK_FALSE(early.ignite);
    input.timeS=input.nativeAgeS=10;
    const auto eligible=manager.update(input);REQUIRE(eligible.valid);CHECK(eligible.ignite);
}

TEST_CASE("Pulse policy rejects invalid epochs and preserves one shot request and expiry", "[weapon-pulse-energy]") {
    using namespace fsim::ms::weapon;
    PulsePolicyV1 p{1,10,30,450,6};PulseManagerV1 m(p);
    PulseInputV1 in{10,10,700,20,std::nullopt};CHECK(m.update(in).state==PulseStateV1::Holding);
    auto invalid=in;invalid.timeS=9;invalid.nativeAgeS=9;invalid.speedMs=100;CHECK_FALSE(m.update(invalid).valid);
    in.timeS=in.nativeAgeS=11;in.timeToGoS=5;CHECK(m.update(in).ignite);
    in.timeS=in.nativeAgeS=50;in.timeToGoS.reset();CHECK(m.update(in).ignite);
    m.reset();CHECK(m.update(in).state==PulseStateV1::Expired);
    in.timeS=in.nativeAgeS=51;in.speedMs=100;CHECK_FALSE(m.update(in).ignite);
    m.reset();in={10,10,100,0,std::nullopt};CHECK(m.update(in).state==PulseStateV1::NoFuel);
    m.reset();in={10,10,700,20,std::nullopt};CHECK_FALSE(m.update(in).ignite);
    in.timeS=in.nativeAgeS=11;CHECK_FALSE(m.update(in).ignite);
}

TEST_CASE("Native commanded pulse holds without consuming reserve and preserves impulse on delayed ignition", "[weapon-pulse-native]") {
    using namespace fsim::sim;
    const double dt=GENERATE(1./120,1./240);
    auto ground=std::make_shared<FlatGround>(0);
    InitialConditions ic;ic.altitudeMslM=1000;ic.airspeedTrueMs=30;ic.enginesRunning=false;
    const AircraftSpec spec{"rocket_commanded_pulse",FSIM_TEST_JSBSIM_ROOT,FSIM_TEST_DATA_DIR};
    JsbsimModel model(dt,ground);REQUIRE(model.load(spec,ic));
    ControlInputs on,off;on.setThrottleAll(1);
    auto property=[&](const char* key){auto h=model.property(key);REQUIRE(h.valid());return h;};
    auto command=property("fsim/motor-pulse/second-command");
    auto ignition=property("fsim/motor-pulse/second-ignition-age-s");
    auto age=property("fsim/motor-pulse/native-age-s");
    auto thrust=property("propulsion/engine/vacuum-thrust_lbs");
    auto fuel=property("propulsion/tank/contents-lbs");
    auto impulse=property("propulsion/engine/total-vac-impulse");
    double heldFuel=0;
    for(int step=0;step<int(.95/dt);++step) {
        model.step(step?off:on);
        if(step*dt>.75) {
            CHECK(thrust.get()==0);CHECK(ignition.get()<0);
            if(!heldFuel)heldFuel=fuel.get();else CHECK(fuel.get()==heldFuel);
        }
    }
    REQUIRE(age.get()>.9);command.set(1);
    for(int step=0;step<int(.4/dt);++step)model.step(off);
    const auto firstIgnition=ignition.get();REQUIRE(firstIgnition>.9);CHECK(thrust.get()==0);
    CHECK(impulse.get()==Catch::Approx(1.6).margin(20*dt));
    CHECK((1-fuel.get())*200==Catch::Approx(impulse.get()).margin(20*dt));
    command.set(0);for(int step=0;step<10;++step)model.step(off);command.set(1);
    for(int step=0;step<10;++step)model.step(off);
    CHECK(ignition.get()==firstIgnition);CHECK(thrust.get()==0);
    REQUIRE(model.reset(ic));CHECK(ignition.get()<0);CHECK(age.get()==0);CHECK(impulse.get()==0);
    // A late command outside the finite native window cannot ignite reserve.
    for(int step=0;step<int(1.3/dt);++step)model.step(step?off:on);
    command.set(1);for(int step=0;step<20;++step)model.step(off);
    CHECK(ignition.get()<0);CHECK(thrust.get()==0);
    CHECK(impulse.get()==Catch::Approx(1).margin(20*dt));
    REQUIRE(model.reset(ic));fuel.set(0);
    for(int step=0;step<30;++step) {
        model.step(step?off:on);CHECK(thrust.get()==0);
    }
    CHECK(impulse.get()==0);CHECK(ignition.get()<0);
}

TEST_CASE("Native commanded pulse rejects invalid variation without fuel creation or nonfinite thrust", "[weapon-pulse-native]") {
    using namespace fsim::sim;
    const std::string field=GENERATE("thrust","total_isp");
    const std::string value=GENERATE("-2","nan","inf");
    // Upstream binds properties before reading its thrust table, so its solid
    // variation setters are unavailable. Exercise actual engine XML inputs
    // through the normal load route rather than inventing a test-only API.
    const auto input=std::filesystem::path(FSIM_TEST_DATA_DIR)/"rocket_commanded_pulse";
#ifdef _WIN32
    const auto temporaryRoot=std::filesystem::path("D:/ENV/tmp");
#else
    const auto temporaryRoot=std::filesystem::temp_directory_path();
#endif
    const auto directory=temporaryRoot/
        ("m03-pulse-invalid-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto output=directory/"rocket_commanded_pulse";std::filesystem::create_directories(output/"Engines");
    for(const auto* name:{"rocket_commanded_pulse.xml","Engines/pulse_nozzle.xml"})
        std::filesystem::copy_file(input/name,output/name);
    std::ifstream source(input/"Engines/pulse_commanded.xml");REQUIRE(source.good());
    auto engine=std::string(std::istreambuf_iterator<char>(source),{});
    const auto end=engine.find("</rocket_engine>");REQUIRE(end!=std::string::npos);
    engine.insert(end,"<variation><"+field+">"+value+"</"+field+"></variation>\n");
    {std::ofstream destination(output/"Engines/pulse_commanded.xml");destination<<engine;REQUIRE(destination.good());}
    InitialConditions initial;initial.altitudeMslM=1000;initial.airspeedTrueMs=30;initial.enginesRunning=false;
    JsbsimModel model(1./120,std::make_shared<FlatGround>(0));
    if(!model.load(AircraftSpec{"rocket_commanded_pulse",FSIM_TEST_JSBSIM_ROOT,directory},initial)) {
        SUCCEED("Malformed nonfinite engine input refused during native load");return;
    }
    const auto fuel=model.property("propulsion/tank/contents-lbs");REQUIRE(fuel.valid());const double before=fuel.get();
    const auto thrust=model.property("propulsion/engine/vacuum-thrust_lbs");REQUIRE(thrust.valid());
    const auto age=model.property("fsim/motor-pulse/native-age-s");REQUIRE(age.valid());
    ControlInputs on;on.setThrottleAll(1);
    for(unsigned tick=0;tick<30;++tick) {
        model.step(on);CHECK(thrust.get()==0);CHECK(fuel.get()==before);CHECK(age.get()==0);
    }
}

TEST_CASE("Native commanded pulse cannot create impulse from deselected or unusable fuel", "[weapon-pulse-native][conservation]") {
    using namespace fsim::sim;
    const bool unusable=GENERATE(false,true);
    InitialConditions initial;initial.altitudeMslM=1000;initial.airspeedTrueMs=30;initial.enginesRunning=false;
    JsbsimModel model(1./120,std::make_shared<FlatGround>(0));
    REQUIRE(model.load({"rocket_commanded_pulse",FSIM_TEST_JSBSIM_ROOT,FSIM_TEST_DATA_DIR},initial));
    const auto fuel=model.property("propulsion/tank/contents-lbs");
    auto priority=model.property("propulsion/tank/priority");
    auto deadVolume=model.property("propulsion/tank/unusable-volume-gal");
    const auto density=model.property("propulsion/tank/density-lbs_per_gal");
    const auto thrust=model.property("propulsion/engine/vacuum-thrust_lbs");
    const auto impulse=model.property("propulsion/engine/total-vac-impulse");
    REQUIRE(fuel.valid());REQUIRE(priority.valid());REQUIRE(deadVolume.valid());REQUIRE(density.valid());
    REQUIRE(thrust.valid());REQUIRE(impulse.valid());REQUIRE(density.get()>0);
    ControlInputs on;on.setThrottleAll(1);model.step(on);model.step(on);model.step(on);
    REQUIRE(thrust.get()>0);REQUIRE(fuel.get()>0);const double beforeFuel=fuel.get(),beforeImpulse=impulse.get();
    if(unusable)deadVolume.set(beforeFuel/density.get());else priority.set(0);
    model.step(on);
    CHECK(fuel.get()==beforeFuel);CHECK(thrust.get()==0);CHECK(impulse.get()==beforeImpulse);
}

TEST_CASE("Commanded pulse impulse cannot exceed actual usable native feed-group consumption", "[weapon-pulse-native][conservation]") {
    using namespace fsim::sim;
    const int otherPriority=GENERATE(0,1,2);
    const auto input=std::filesystem::path(FSIM_TEST_DATA_DIR)/"rocket_commanded_pulse";
    const auto temporary=std::filesystem::path("D:/ENV/tmp")/
        ("m03-pulse-feeds-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto output=temporary/"rocket_commanded_pulse";std::filesystem::create_directories(output);
    std::filesystem::copy(input/"Engines",output/"Engines",std::filesystem::copy_options::recursive);
    std::ifstream source(input/"rocket_commanded_pulse.xml");REQUIRE(source.good());
    auto xml=std::string(std::istreambuf_iterator<char>(source),{});
    const auto feed=xml.find("<feed>0</feed>");REQUIRE(feed!=std::string::npos);
    xml.replace(feed,std::string("<feed>0</feed>").size(),"<feed>0</feed><feed>1</feed>");
    const auto end=xml.find("</propulsion>");REQUIRE(end!=std::string::npos);
    xml.insert(end,"<tank type=\"FUEL\"><location unit=\"IN\"><x>0</x><y>0</y><z>0</z></location>"
                   "<capacity unit=\"LBS\">1</capacity><contents unit=\"LBS\">1</contents>"
                   "<priority>"+std::to_string(otherPriority)+"</priority></tank>");
    {std::ofstream destination(output/"rocket_commanded_pulse.xml");destination<<xml;REQUIRE(destination.good());}
    InitialConditions initial;initial.altitudeMslM=1000;initial.airspeedTrueMs=30;initial.enginesRunning=false;
    JsbsimModel model(1./120,std::make_shared<FlatGround>(0));
    REQUIRE(model.load({"rocket_commanded_pulse",FSIM_TEST_JSBSIM_ROOT,temporary},initial));
    auto first=model.property("propulsion/tank/contents-lbs");const auto second=model.property("propulsion/tank[1]/contents-lbs");
    const auto impulse=model.property("propulsion/engine/total-vac-impulse");REQUIRE(first.valid());REQUIRE(second.valid());REQUIRE(impulse.valid());
    ControlInputs on;on.setThrottleAll(1);model.step(on);model.step(on);model.step(on);
    first.set(1e-6);
    for(unsigned step=0;step<4;++step) {
        const double beforeFuel=first.get()+second.get(),beforeImpulse=impulse.get();model.step(on);
        const double consumed=beforeFuel-first.get()-second.get();
        CAPTURE(otherPriority,step,consumed);
        CHECK(consumed>=0);
        CHECK(impulse.get()-beforeImpulse<=consumed*200+1e-10);
    }
}
