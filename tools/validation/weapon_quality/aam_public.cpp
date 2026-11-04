// Installed C++ SDK only. Actor requests use Agent; truth remains referee data.
#include <fsim/World.h>
#include "aam_public_capture.h"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace {
unsigned checks=0;
void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
constexpr double pi=3.14159265358979323846,deg=pi/180.;
fsim::ms::WeaponRecord weaponRecord(fsim::World& world,std::uint32_t id) {
    for(const auto& row:world.weapons())if(row.id==id)return row;
    throw std::runtime_error("Actual weapon history missing");
}
}
int main(int argc,char** argv) {
    try {
        if(argc!=9)return 2;
        const std::string card=argv[2],nativeModel=argv[3],directory=argv[8];
        const int station=std::stoi(argv[5]);const bool sarh=std::stoi(argv[6])!=0;
        fsim::WorldOptions options;options.name="m03-installed-cpp-"+card;
        options.publish=false;options.terrain=false;options.workers=1;options.pinWorkers=false;
        options.seed=19;options.dt=1./120;options.frameSkip=4;
        options.jsbsimRoot=std::string(argv[1])+"/share/flightsim/jsbsim";
        fsim::World world(options);fsim::VehicleSpec own;
        own.name="launcher";own.type="jsbsim:"+std::string(argv[4]);
        own.initial.latitudeDeg=40;own.initial.longitudeDeg=0;own.initial.altitudeMslM=6000;
        own.initial.headingDeg=90;own.initial.airspeedTrueMs=250;
        auto carrier=world.createVehicle(own);check(carrier.valid(),"Carrier load failed");
        auto launcher=world.createAgent(carrier,"launcher");fsim::control::VelocityCommand hold;
        hold.airspeedMs=250;hold.verticalSpeedMs=0;hold.headingRad=90*deg;
        check(launcher.submit(hold).accepted(),"Actor hold refused");
        fsim::comm::C2NodeSpec node;node.name="approver";node.latitudeRad=39.9*deg;node.altitudeMslM=200;
        const auto c2=world.createC2Node(node),net=world.createNet({});
        check(world.joinNet(carrier.id(),net)&&world.joinNet(c2,net),"Consent network refused");
        auto approver=world.createAgent(c2,"approver");check(approver.declareRole(2,"APPROVER"),"Role refused");
        std::uint16_t payload=0xffff,radar=0xffff;
        for(const auto& p:launcher.payloads()) {
            if(p.family==fsim::ms::Family::StoreStation)payload=p.index;
            if(p.family==fsim::ms::Family::AirRadar)radar=p.index;
        }
        check(payload!=0xffff&&radar!=0xffff,"Required fitted payloads missing");
        check(carrier.loadStore(payload,station,card)==fsim::ms::Reason::None,"Native load refused");
        for(const auto p:{payload,radar})check(launcher.submit(fsim::ms::StateCommand{p,fsim::ms::PayloadState::Operate}).accepted(),"Power refused");
        unsigned ready=0;
        for(unsigned i=0;i<1350&&ready!=2;++i) {
            world.step();ready=0;
            for(const auto& p:launcher.payloads())if((p.index==payload||p.index==radar)&&p.state==fsim::ms::PayloadState::Operate)++ready;
        }
        check(ready==2,"Warmup failed");const auto origin=carrier.state();
        fsim::VehicleSpec opponent;opponent.name="opponent";opponent.type="jsbsim:f16c";
        opponent.initial.latitudeDeg=origin.latitudeRad/deg;
        opponent.initial.longitudeDeg=origin.longitudeRad/deg+5000/(6371000*std::cos(origin.latitudeRad))/deg;
        opponent.initial.altitudeMslM=6000;opponent.initial.airspeedTrueMs=250;opponent.initial.headingDeg=270;
        auto target=world.createVehicle(opponent);check(target.valid(),"Target load failed");
        auto opponentAgent=world.createAgent(target,"opponent");hold.headingRad=270*deg;
        check(opponentAgent.submit(hold).accepted(),"Target actor hold refused");
        fsim::ms::AmtiCommand search;search.payload=radar;search.bars=1;search.azimuthHalfWidthRad=15*deg;search.rangeScaleM=60000;
        check(launcher.submit(search).accepted(),"Actor search refused");std::uint64_t track=0;
        for(unsigned i=0;i<600&&!track;++i) {
            world.step();for(const auto& t:launcher.amtiTracks(radar))if(t.updates>=3){track=t.id;break;}
        }
        check(track!=0,"No actual permitted radar track");
        const auto designation=launcher.designate(radar,track);check(designation!=0,"Designation refused");
        search.task=fsim::ms::AmtiTask::Track;search.mode=fsim::ms::AmtiMode::SingleTargetTrack;search.track=track;search.illuminate=sarh;
        check(launcher.submit(search).accepted(),"STT refused");
        fsim::ms::StrikeCommand strike;strike.payload=payload;strike.store=card;strike.target=designation;
        const auto request=launcher.submit(strike);check(request.accepted()&&world.weapons().empty(),"Strike bypassed consent");
        bool answered=false;
        for(unsigned i=0;i<1350&&!answered;++i) {
            world.step();for(const auto& r:approver.reportConsents().items)if(r.value.request.strike==request.activity) {
                check(r.meta.source==fsim::ma::Source::Dms&&world.weapons().empty(),"Consent bypassed DMS delivery");
                check(approver.answerConsent(r.value.from,r.value.request.requestId,true),"Answer refused");answered=true;break;
            }
        }
        check(answered,"No delivered consent");for(unsigned i=0;i<600&&world.weapons().empty();++i)world.step();
        check(world.weapons().size()==1,"No correlated actual release");
        const auto released=world.weapons().front();const auto weapon=released.id;
        check(released.store==card&&!released.jettisoned&&released.launcher==carrier.id(),"Foreign/unarmed release");
        check(world.vehicle(weapon).type()==nativeModel,"Wrong native model identity");
        const double launch=released.launchTimeS;unsigned samples=0,controls=0,tracking=0,burning=0,damage=0;
        std::ofstream trace(directory+"/trace.csv");check(trace.good(),"Trace open failed");trace<<std::setprecision(17);
        trace<<"model,weapon,time,age,tracking,burning,controller,fin_count,terminal\n";
        while(world.time()<launch+40) {
            world.step();const auto row=weaponRecord(world,weapon);++samples;
            tracking+=row.tracking?1U:0U;burning+=row.burning?1U:0U;
            const auto state=world.weaponControlsV1(weapon);
            if(state&&state->controllerAvailable)++controls;
            trace<<card<<','<<weapon<<','<<world.time()<<','<<world.time()-launch<<','<<row.tracking<<','<<row.burning<<','
                <<(state&&state->controllerAvailable)<<','<<(state?state->fins.size():0)<<','<<static_cast<int>(row.end)<<'\n';
            for(const auto& v:world.vehicles()) {
                const auto& s=v.state();check(!s.diverged,"Native world diverged");
                for(const double x:s.positionEcef)check(std::isfinite(x),"Nonfinite native position");
                for(const double x:s.velocityBodyMs)check(std::isfinite(x),"Nonfinite native velocity");
            }
        }
        std::ofstream events(directory+"/events.csv");check(events.good(),"Events open failed");events<<std::setprecision(17);
        events<<"model,weapon,serial,time,kind,source,target\n";
        for(const auto& e:world.engagementEvents()) {
            events<<card<<','<<weapon<<','<<e.serial<<','<<e.timeS<<','<<e.kind<<','<<e.entity<<','<<e.target<<'\n';
            if(e.entity==weapon&&e.target==target.id()&&e.kind=="damage")++damage;
        }
        events.flush();check(events.good(),"Events write failed");
        const auto final=weaponRecord(world,weapon);trace.flush();check(trace.good(),"Trace write failed");
        trace.close();check(!trace.fail(),"Trace close failed");events.close();check(!events.fail(),"Events close failed");
        check(samples>0&&tracking>0&&burning>0&&controls>0,"Missing actual native acquisition/control/propulsion");
        check(final.end==fsim::ms::WeaponEnd::Detonated&&damage>0,"Native detonation/target damage absent");
        check(aam_public_capture((directory+"/capture.json").c_str(),"cpp",card.c_str(),nativeModel.c_str(),argv[7],weapon,target.id(),checks,
            samples,controls,tracking,burning,damage,"detonated",launch,world.time(),directory.c_str())!=0,"Capture failed");
        return 0;
    } catch(const std::exception& error){std::fprintf(stderr,"Installed C++ AAM: %s\n",error.what());return 1;}
}
