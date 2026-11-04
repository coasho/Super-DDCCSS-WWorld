#include <fsim/World.h>
#include <fsim/WeaponLink.h>
#include <cmath>
#include <cstdio>
#include <memory>
#include <atomic>
#include <thread>
#include <stdexcept>
#ifndef FSIM_TEST_JSBSIM_ROOT
#define FSIM_TEST_JSBSIM_ROOT ""
#endif
using namespace fsim;
namespace {
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
Vehicle aircraft(World& world){VehicleSpec s;s.name="launcher";s.type="jsbsim:f16c";
s.initial.latitudeDeg=40;s.initial.longitudeDeg=0;s.initial.altitudeMslM=6000;s.initial.headingDeg=90;s.initial.airspeedTrueMs=250;return world.createVehicle(s);}
void integration(const char* root,bool resetGeneration=false,bool terminalRetention=false,bool ownerRemoval=false) {
    WorldOptions setup;setup.publish=false;setup.workers=1;setup.pinWorkers=false;setup.seed=19;setup.jsbsimRoot=root;
    setup.name = "installed-fin-cpp";
    setup.terrain = false;
    World world(setup);
    auto carrier = aircraft(world);

    Agent launcher = world.createAgent(carrier, "launcher");
    control::VelocityCommand hold;
    hold.airspeedMs = 250; hold.verticalSpeedMs = 0; hold.headingRad = 90 * 3.14159265358979323846 / 180;
    require(launcher.submit(hold).accepted(), "launcher flight hold refused");
    comm::C2NodeSpec node;
    node.name = "approver"; node.latitudeRad = 39.9 * 3.14159265358979323846 / 180; node.altitudeMslM = 200;
    const auto c2 = world.createC2Node(node);
    const auto net = world.createNet({});
    require(world.joinNet(carrier.id(), net) && world.joinNet(c2, net), "installed consent net could not be joined");
    Agent approver = world.createAgent(c2, "approver");
    require(approver.declareRole(2, "APPROVER"), "approver role could not be declared");
    std::uint16_t stations = 0xffff, radar = 0xffff;
    for (const auto& payload : launcher.payloads()) {
        if (payload.family == ms::Family::StoreStation) stations = payload.index;
        if (payload.family == ms::Family::AirRadar) radar = payload.index;
    }
    require(stations != 0xffff && radar != 0xffff, "installed radar/stations missing");
    require(carrier.loadStore(stations,1,"aim-120d")==ms::Reason::None,"AIM120D explicit store fit refused");
    require(launcher.submit(ms::StateCommand{stations, ms::PayloadState::Operate}).accepted(), "station power refused");
    require(launcher.submit(ms::StateCommand{radar, ms::PayloadState::Operate}).accepted(), "radar power refused");
    bool operating = false;
    for (unsigned i = 0; i < 1200 && !operating; ++i) {
        world.step();
        unsigned ready = 0;
        for (const auto& payload : launcher.payloads())
            if ((payload.index == stations || payload.index == radar) && payload.state == ms::PayloadState::Operate) ++ready;
        operating = ready == 2;
    }
    require(operating, "installed radar/station warmup failed");
    const auto own = carrier.state();
    VehicleSpec targetSpec;
    targetSpec.name = "opponent"; targetSpec.type = "jsbsim:f16c";
    targetSpec.initial.latitudeDeg = own.latitudeRad * 180 / 3.14159265358979323846 + .01;
    targetSpec.initial.longitudeDeg = own.longitudeRad * 180 / 3.14159265358979323846 + .15;
    targetSpec.initial.altitudeMslM = 6000; targetSpec.initial.airspeedTrueMs = 250; targetSpec.initial.headingDeg = 270;
    auto target = world.createVehicle(targetSpec);
    hold.headingRad = 270 * 3.14159265358979323846 / 180;
    require(target.submit(hold).accepted(), "target flight hold refused");
    ms::AmtiCommand scan;
    scan.payload = radar; scan.bars = 1; scan.azimuthHalfWidthRad = 15 * 3.14159265358979323846 / 180;
    scan.rangeScaleM = 60000;
    require(launcher.submit(scan).accepted(), "installed radar search refused");
    std::uint64_t track = 0;
    for (unsigned i = 0; i < 600 && !track; ++i) {
        world.step();
        for (const auto& measured : launcher.amtiTracks(radar)) if (measured.updates >= 3) { track = measured.id; break; }
    }
    require(track != 0, "installed radar did not confirm a track");
    WeaponLinksV1 links(launcher);
    const auto fused=launcher.fusedTracksV1();require(!fused.empty(),"Accepted platform fusion missing");
    const auto designation=links.designateFused(links.instance(),fused.front().id);
    require(designation!=0,"Own fused designation refused");
    require(links.designateFused(links.instance()+1,fused.front().id)==0,"Wrong instance accepted");
    require(links.fusedDesignations().size()==1,"Fused designation read must be retained");
    scan.task = ms::AmtiTask::Track; scan.mode = ms::AmtiMode::SingleTargetTrack; scan.track = track;
    require(launcher.submit(scan).accepted(), "installed radar track refused");
    ms::StrikeCommand strike;
    strike.payload = stations; strike.store = "aim-120d"; strike.target = designation;
    const auto accepted = launcher.submit(strike);
    require(accepted.accepted() && world.weapons().empty(), "legal strike did not wait for delivered consent");
    bool answered = false;
    for (unsigned i = 0; i < 1200 && !answered; ++i) {
        world.step();
        for (const auto& report : approver.reportConsents().items) if (report.value.request.strike == accepted.activity) {
            require(report.meta.source == ma::Source::Dms && world.weapons().empty(), "consent bypassed modeled delivery");
            require(approver.answerConsent(report.value.from, report.value.request.requestId, true), "delivered consent answer refused");
            answered = true; break;
        }
    }
    require(answered, "installed approver received no consent request");
    for (unsigned i = 0; i < 600 && world.weapons().empty(); ++i) world.step();
    require(world.weapons().size() == 1, "delivered consent did not release one armed missile");
    const auto missile = world.weapons().front().id;
    const auto cap=links.capability(missile);
    require(cap.reason==ms::WeaponLinkReasonV1::None&&cap.profile.direction==ms::WeaponLinkDirectionV1::TwoWay,"Exact AIM120D fit lacks admitted direction");
    ms::WeaponAssessmentRequestV1 request;request.instance=links.instance();request.weapon=missile;request.requestId=81;
    const auto receipt=links.assess(request);require(receipt.accepted,"Own assessment refused");
    require(links.assess(request).assessment==receipt.assessment,"Identical retry repeated work");
    require(links.feedback(missile,receipt.assessment).empty(),"Undelivered return report leaked");
    std::vector<ms::WeaponFeedbackV1> reports;
    for(unsigned i=0;i<120&&reports.empty();++i){world.step();reports=links.feedback(missile,receipt.assessment);}
    require(!reports.empty(),"No modeled return delivery");const auto& report=reports.front();
    require(report.receivedTimeS>report.sentTimeS&&report.sentTimeS>=report.sampleTimeS,"Finite feedback timing lost");
    require(report.navigationValid&&report.navigationCovariance[0]>0,"Onboard uncertain navigation missing");
    require(report.navigationTimeS<=report.sampleTimeS,"Navigation state epoch is future");
    require(links.feedback(missile,receipt.assessment).size()==reports.size(),"Repeated read consumed feedback");
    WeaponLinksV1 foreign(approver);
    require(foreign.capability(missile).reason==ms::WeaponLinkReasonV1::NotOwner,"Foreign platform gained capability");
    require(!foreign.assess(request).accepted,"Foreign assessment accepted");
    auto sibling=world.createAgent(carrier,"different service");WeaponLinksV1 other(sibling);
    require(!other.assess(request).accepted,"Another controller on same platform gained the release");
    request.durationS=29;require(links.assess(request).reason==ms::WeaponLinkReasonV1::RequestConflict,"Changed request ID body accepted");
    require(links.cancel(links.instance(),missile,receipt.assessment),"Own assessment cancellation refused");
    const auto before=links.feedback(missile,receipt.assessment).size();world.step(30);
    require(links.feedback(missile,receipt.assessment).size()==before,"Canceled assessment published a later packet");
    require(links.status(missile,receipt.assessment).state==ms::WeaponAssessmentStateV1::Canceled,"Cancellation state lost");
    request.requestId=82;request.durationS=terminalRetention?3600:30;
    const auto live=links.assess(request);require(live.accepted,"Second live assessment refused");
    if(ownerRemoval) {
        std::vector<ms::WeaponFeedbackV1> retained;
        for(unsigned i=0;i<120&&retained.empty();++i){world.step();retained=links.feedback(missile,live.assessment);}
        require(!retained.empty(),"Owner-removal fixture requires actual delivered feedback before closure");
        require(links.capability(missile).reason==ms::WeaponLinkReasonV1::None&&links.assess(request).assessment==live.assessment,
            "The original launcher binding must exist before removal");
        const double removedAt=world.time();require(carrier.remove(),"Native launcher removal refused");
        require(world.time()==removedAt,"Owner-removal refusal must be checked without another World step");
        require(links.capability(missile).reason==ms::WeaponLinkReasonV1::NotOwner,
            "Removing the launcher retained actor capability until the next World step");
        require(links.status(missile,live.assessment).reason==ms::WeaponLinkReasonV1::NotOwner&&links.feedback(missile,live.assessment).empty(),
            "Removing the launcher retained actor status or feedback in the same tick");
        require(links.assess(request).reason==ms::WeaponLinkReasonV1::NotOwner&&!links.cancel(links.instance(),missile,live.assessment),
            "Removing the launcher retained assessment retry or cancel authority in the same tick");
    } else if(terminalRetention) {
        const double deadline=world.time()+240;
        std::optional<ms::WeaponRecord> ended;
        while(world.time()<deadline&&!ended) {
            world.step();
            for(const auto& row:world.weapons())if(row.id==missile&&row.end!=ms::WeaponEnd::InFlight)ended=row;
        }
        require(ended.has_value()&&ended->end!=ms::WeaponEnd::Removed,"Retention fixture requires an actual native terminal event");
        require(std::isfinite(ended->endTimeS)&&cap.profile.retentionS==30,"Native terminal retention policy changed");
        const auto retainedCount=links.feedback(missile,live.assessment).size();
        const double closes=ended->endTimeS+cap.profile.retentionS;
        while(world.time()+world.stepSeconds()<closes)world.step();
        require(links.capability(missile).reason==ms::WeaponLinkReasonV1::None,"Native terminal binding vanished before its retention deadline");
        require(links.assess(request).assessment==live.assessment,"Retained terminal retry lost its original receipt");
        require(links.feedback(missile,live.assessment).size()>=retainedCount,"Retained terminal reports were consumed or prematurely discarded");
        while(world.time()<=closes+world.stepSeconds())world.step();
        require(links.capability(missile).reason==ms::WeaponLinkReasonV1::NotOwner,"A 3600-second request prolonged terminal retention beyond 30 seconds");
        require(links.assess(request).reason==ms::WeaponLinkReasonV1::NotOwner,"Purged assessment disclosed or reused its old binding");
        require(links.status(missile,live.assessment).reason==ms::WeaponLinkReasonV1::NotOwner&&links.feedback(missile,live.assessment).empty(),
            "Purged terminal reports remained actor-visible");
    } else if(resetGeneration) {
        require(world.vehicle(missile).reset(),"Native pose reset refused");
        require(links.capability(missile).reason==ms::WeaponLinkReasonV1::Closed,"Pose reset reused a communication generation");
        require(!links.assess(request).accepted,"Stale-generation assessment was accepted");
        world.step(10);
        require(links.feedback(missile,live.assessment).empty(),"A stale generation published queued telemetry");
    } else {
        world.step(20);require(world.vehicle(missile).remove(),"Native weapon removal refused");
        require(links.assess(request).assessment==live.assessment,"Terminal removal lost original retry receipt");
        request.requestId=83;require(links.assess(request).reason==ms::WeaponLinkReasonV1::Expired,"Removed weapon admitted a new assessment");
        world.step(180);
        require(links.status(missile,live.assessment).state==ms::WeaponAssessmentStateV1::Lost,"Forced removal must end by link timeout");
        for(const auto& row:links.feedback(missile,live.assessment))require(!row.endOfFlight,"Range removal invented onboard end-of-flight");
    }
}

void unsupported_store(const char* root) {
    WorldOptions options;options.publish=false;options.workers=1;options.pinWorkers=false;options.jsbsimRoot=root;
    World world(options);auto carrier=aircraft(world);auto agent=world.createAgent(carrier,"jettison");
    std::uint16_t stations=65535;
    for(const auto& payload:agent.payloads())if(payload.family==ms::Family::StoreStation)stations=payload.index;
    require(stations!=65535,"Station payload missing");
    require(carrier.loadStore(stations,3,"mk82")==ms::Reason::None,"Unguided store fit refused");
    require(agent.submit(ms::StateCommand{stations,ms::PayloadState::Operate}).accepted(),"Station operation refused");
    world.step(180);ms::JettisonCommand command;command.payload=stations;
    const auto release=agent.submit(command);require(release.accepted(),"Own jettison refused");
    std::uint32_t weapon=0;
    for(unsigned i=0;i<180&&!weapon;++i){world.step();const auto status=agent.strikeStatus(release.activity);
        if(status&&!status->weapons.empty())weapon=status->weapons.front();}
    require(weapon!=0,"Own release correlation missing");WeaponLinksV1 links(agent);
    require(links.capability(weapon).reason==ms::WeaponLinkReasonV1::Unsupported,"An own unguided release must explicitly report unsupported equipment");
    ms::WeaponAssessmentRequestV1 request;request.instance=links.instance();request.weapon=weapon;request.requestId=1;
    require(links.assess(request).reason==ms::WeaponLinkReasonV1::Unsupported,"Unguided store cannot acquire return feedback");
}

void lifetime(const char* root){
    std::unique_ptr<WeaponLinksV1> facade;
    WorldOptions o;o.publish=false;o.workers=1;o.pinWorkers=false;o.jsbsimRoot=root;
    auto world=std::make_unique<World>(o);
    auto vehicle=aircraft(*world);auto agent=world->createAgent(vehicle,"lifetime");
    facade=std::make_unique<WeaponLinksV1>(agent);require(facade->instance()!=0,"Instance missing");
    std::atomic<bool> running{false},closed{false};
    std::thread reader([&]{running.store(true);for(;;){try{facade->instance();}catch(const std::runtime_error&){closed.store(true);return;}}});
    while(!running.load())std::this_thread::yield();
    world.reset();reader.join();
    require(closed.load(),"Retained facade did not reject concurrent native World destruction");
}

}
int main(int argc,char** argv){try{const char* root=argc>1?argv[1]:FSIM_TEST_JSBSIM_ROOT;
integration(root);integration(root,true);integration(root,false,false,true);integration(root,false,true);unsupported_store(root);lifetime(root);puts("weapon links public C++: 6 cases passed");return 0;
}catch(const std::exception& e){fprintf(stderr,"%s\n",e.what());return 1;}}
