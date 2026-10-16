// MA-service binding, legal reports and mission episodes from an installed SDK: its public headers and its library,
// nothing else (docs/sdk/mission-training.md, sections 3 to 14).
#include <fsim/Agent.h>
#include <fsim/MAReports.h>
#include <fsim/MAService.h>
#include <fsim/MissionBatch.h>
#include <fsim/MissionEpisode.h>
#include <fsim/World.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace fs = std::filesystem;
using namespace fsim;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
const char* const kHsa = "fsim.guidance.hsa";
const char* const kRoute = "fsim.guidance.route";

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

std::string jsonText(const std::string& s) {
    std::string out = "\"";
    for (const char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out + "\"";
}

void checkModule(const fs::path& prefix, const wchar_t* name) {
#ifdef _WIN32
    const HMODULE module = GetModuleHandleW(name);
    require(module != nullptr, "required runtime DLL was not loaded");
    wchar_t path[32768];
    const DWORD size = GetModuleFileNameW(module, path, 32768);
    require(size != 0 && size < 32768, "cannot inspect loaded runtime DLL");
    const fs::path actual(std::wstring(path, size));
    require(fs::equivalent(actual, prefix / "bin" / name), "loaded DLL came from outside the installed SDK");
    std::printf("installed DLL: %s\n", actual.u8string().c_str());
#else
    (void)prefix;
    (void)name;
    throw std::runtime_error("this package gate requires the Windows UCRT64 runtime");
#endif
}

WorldOptions options(const char* name, const fs::path& data) {
    WorldOptions o;
    o.name = name;
    o.publish = false;
    o.workers = 1;
    o.pinWorkers = false;
    o.jsbsimRoot = (data / "jsbsim").generic_string();
    return o;
}

Vehicle aircraft(World& w, const char* name, double northDeg) {
    VehicleSpec s;
    s.name = name;
    s.type = "jsbsim:f16c";
    s.initial.latitudeDeg = 40.0 + northDeg;
    s.initial.longitudeDeg = 0.0;
    s.initial.altitudeMslM = 6000.0;
    s.initial.airspeedTrueMs = 230.0;
    return w.createVehicle(s);
}

MAServiceBindingV1 binding(std::uint64_t service, const Agent& agent, std::vector<std::string> capabilities, bool ingress, bool egress) {
    MAServiceBindingV1 b;
    b.serviceId = service;
    b.platform = agent.vehicle();
    b.controller = agent.controller();
    b.commandCapabilities = std::move(capabilities);
    b.ownsDmsIngress = ingress;
    b.ownsDmsEgress = egress;
    return b;
}

/// Who owns what on a platform with two services, the agents made in one order or the other: "hsa route ingress egress".
std::string ownership(const fs::path& data, bool flightFirst) {
    World w(options(flightFirst ? "installed-ma-order-a" : "installed-ma-order-b", data));
    Vehicle viper = aircraft(w, "viper", 0.0);
    Vehicle wing = aircraft(w, "wing", 0.02);
    Agent first = w.createAgent(viper, flightFirst ? "flight" : "comms");
    Agent second = w.createAgent(viper, flightFirst ? "comms" : "flight");
    Agent& flight = flightFirst ? first : second;
    Agent& comms = flightFirst ? second : first;
    Agent other = w.createAgent(wing, "other");
    std::string error;
    // before anything is bound, the other service holds the grant and flies: the binding, not that, decides
    control::HsaCommand hsa;
    hsa.headingRad = 1.0, hsa.speed = 230.0, hsa.altitudeM = 6000.0;
    require(comms.requestControl(kHsa) == control::Reason::None && comms.requestControl(kRoute) == control::Reason::None,
            "an open platform refused a grant");
    const control::CommandResult before = comms.submit(hsa);
    require(before.accepted(), "an open platform refused a command");
    // bound in the opposite order as well: neither order decides anything
    const MAServiceBindingV1 one = binding(1, flight, {kHsa, kRoute}, false, false);
    const MAServiceBindingV1 two = binding(2, comms, {}, true, true);
    require(bindMAService(w, flightFirst ? one : two, &error) && bindMAService(w, flightFirst ? two : one, &error), "a binding was refused");
    // an exclusive ownership twice, another platform's controller: refused with the reason, nothing changed
    require(!bindMAService(w, binding(3, comms, {kHsa}, false, false), &error) && !error.empty(), "a second owner of a capability was bound");
    MAServiceBindingV1 foreign = binding(4, other, {"*"}, false, false);
    foreign.platform = viper.id();
    error.clear();
    require(!bindMAService(w, foreign, &error) && !error.empty(), "another platform's controller was bound");
    require(maServiceBindings(w).size() == 2, "a refused binding changed the bindings");
    // what stood before the bindings gave way to them: the grant its owner's, the other's flight ended as revoked
    const auto ended = comms.activity(before.activity);
    require(ended && !ended->live() && ended->reason == control::Reason::Revoked, "another service's flight outlived the binding");
    require(viper.controlStatus(kHsa).granted && viper.controlStatus(kHsa).holder == flight.controller() &&
                viper.controlStatus(kRoute).holder == flight.controller(),
            "a grant held before the binding is not its owner's after it");
    require(comms.requestControl(kHsa) == control::Reason::NotAllowed && viper.requestControl(kHsa) == control::Reason::NotAllowed &&
                viper.requestControl(kHsa, 7) == control::Reason::NotAllowed,
            "a controller that does not own a capability was granted it");
    require(viper.setControlMode(control::ControlMode::Open) == control::Reason::NotAllowed &&
                viper.controlMode() == control::ControlMode::Granted,
            "a bound platform was opened");
    // its capability its owner's to command, and no one else's
    require(flight.submit(hsa).accepted(), "the owner's command was refused");
    require(!comms.submit(hsa).accepted(), "another service commanded a capability it does not own");
    require(flight.controlStatus(kHsa).granted && flight.controlStatus(kHsa).holder == flight.controller(), "the grant is not the owner's");
    comm::DmsMessage message;
    message.topic = comm::topic::kFreeForm + 1;
    message.bytes = {0x01};
    comm::DmsReason why = comm::DmsReason::None;
    require(flight.send(message, &why) == 0 && why == comm::DmsReason::NotOwner, "a service that does not own the DMS egress sent");
    const auto name = [&](control::ControllerId c) { return c == flight.controller() ? "flight" : c == comms.controller() ? "comms" : "none"; };
    std::string out = std::string(name(maCapabilityOwner(w, viper.id(), kHsa))) + " " + name(maCapabilityOwner(w, viper.id(), kRoute));
    for (const MAServiceBindingV1& b : maServiceBindings(w))
        if (b.ownsDmsIngress) out += std::string(" ") + name(b.controller);
    for (const MAServiceBindingV1& b : maServiceBindings(w))
        if (b.ownsDmsEgress) out += std::string(" ") + name(b.controller);
    return out;
}

/// Two worlds mint the same numbers: an Agent binds only in the world that made it.
void worlds(const fs::path& data) {
    World one(options("installed-ma-world-one", data));
    World two(options("installed-ma-world-two", data));
    Vehicle a1 = aircraft(one, "viper", 0.0);
    Vehicle a2 = aircraft(two, "viper", 0.0);
    Agent mine = one.createAgent(a1, "mine");
    Agent theirs = two.createAgent(a2, "theirs");
    require(a1.id() == a2.id() && mine.controller() == theirs.controller(), "the two worlds minted different numbers");
    std::string error;
    require(!bindMAService(one, theirs, 1, {kHsa}, true, true, &error) && !error.empty(), "another world's agent was bound");
    require(maServiceBindings(one).empty() && a1.controlMode() == control::ControlMode::Open && !a1.controlStatus(kHsa).granted,
            "a refused binding changed the world");
    require(bindMAService(one, mine, 1, {kHsa}, true, true, &error) && maServiceBindings(one).size() == 1 &&
                a1.controlStatus(kHsa).holder == mine.controller(),
            "its own agent was not bound");
}

void reports(const fs::path& data) {
    World w(options("installed-ma-reports", data));
    Vehicle viper = aircraft(w, "viper", 0.0);
    comm::C2NodeSpec spec;
    spec.name = "crc", spec.latitudeRad = 39.64 * kDeg, spec.longitudeRad = 0.0, spec.altitudeMslM = 200.0;
    const std::uint32_t crc = w.createC2Node(spec);
    const comm::NetId net = w.createNet({});
    require(w.joinNet(viper.id(), net) && w.joinNet(crc, net), "the net was not joined");
    Agent blue = w.createAgent(viper, "blue-1");
    Agent boss = w.createAgent(crc, "crc");
    require(boss.declareRole(2, "CRC LEAD"), "the C2 role was not declared"); // (a role that may task the platform)
    w.step(60);

    const ma::Report<ma::Navigation> nav = blue.reportNavigation();
    require(nav.availability == ma::Availability::Available && nav.meta.source == ma::Source::OwnNavigation &&
                nav.meta.validity == ma::Validity::Valid,
            "navigation: not an available, valid report of its own navigation");
    require(nav.meta.sampleTimeS == nav.value.ownShip.timeS && nav.meta.receiveTimeS == w.time() &&
                nav.meta.ageS == w.time() - nav.value.ownShip.timeS,
            "navigation: its times are not its own ship's");
    ma::ReportOptions strict;
    strict.staleAfterS = -1.0;
    require(blue.reportNavigation(strict).meta.validity == ma::Validity::Stale, "a report older than the threshold is not stale");
    require(blue.contacts(0).empty() && blue.reportContacts().availability == ma::Availability::Available &&
                blue.reportContacts().items.empty() && blue.reportTracks().items.empty() && blue.reportLinkTracks().items.empty(),
            "a platform whose sensors are off holds something");

    // a task is there once the link delivered it, and not before
    comm::TaskOrder order;
    order.type = comm::TaskOrder::Patrol;
    order.latitudeRad = 40.3 * kDeg, order.longitudeRad = 0.0;
    const double sent = w.time();
    const std::uint64_t task = boss.sendTask(blue.system(), order);
    require(task != 0 && blue.reportTasks().availability == ma::Availability::Available && blue.reportTasks().items.empty(),
            "a task was there before its delivery");
    w.step(120);
    const ma::Collection<comm::ReceivedTask> tasks = blue.reportTasks();
    require(tasks.items.size() == 1 && tasks.items[0].value.order.taskId == task, "the delivered task is not reported");
    const ma::ReportMeta& meta = tasks.items[0].meta;
    require(meta.source == ma::Source::Link && meta.system == boss.system() && meta.receiveTimeS == tasks.items[0].value.receivedS &&
                meta.receiveTimeS > sent,
            "the task's provenance is not its delivery's");
    require(std::isnan(meta.sampleTimeS) && std::isnan(meta.ageS) && std::isnan(tasks.items[0].value.order.altitudeM),
            "an unknown was given a value");
    require(blue.reportTasks().items.size() == 1 && blue.c2Tasks().size() == 1, "reading a report consumed it");
    // the peer it heard and that peer's heartbeat: two provenances, index for index
    const ma::Collection<comm::PeerReport> peers = blue.reportPeers();
    const ma::Collection<comm::PeerReport> beats = blue.reportPeerHeartbeats();
    require(beats.availability == ma::Availability::Available && beats.items.size() == peers.items.size(),
            "the heartbeats are not beside the peers");
    for (std::size_t i = 0; i < beats.items.size(); ++i)
        require(beats.items[i].meta.source == ma::Source::Link && beats.items[i].meta.system == peers.items[i].value.system,
                "a heartbeat's provenance is not its peer's");
    const ma::Report<C2Authority> authority = blue.reportAuthority();
    require(authority.availability == ma::Availability::Available && authority.meta.source == ma::Source::MaCore &&
                authority.value.operators.size() == 1 && authority.value.operators[0].callsign == "CRC LEAD",
            "authority: not what its MA core keeps");

    // a C2 node has links, and neither navigation nor sensors
    require(boss.reportNavigation().availability == ma::Availability::NotApplicable &&
                boss.reportContacts().availability == ma::Availability::NotApplicable &&
                boss.reportPeers().availability == ma::Availability::Available,
            "a C2 node's reports are not what it has");
    require(viper.remove() && blue.reportNavigation().availability == ma::Availability::Unavailable && blue.reportTasks().items.empty(),
            "a removed platform still reports");
    require(std::string(ma::availabilityName(ma::Availability::NotApplicable)) == "not_applicable" &&
                std::string(ma::validityName(ma::Validity::Stale)) == "stale" && std::string(ma::sourceName(ma::Source::Link)) == "link",
            "the names are not the contract's");
}

} // namespace

/// The installed episode's configuration: one aircraft and a C2 node, a tick limit of 3; the env_id a slot's own.
std::string missionConfig(const fs::path& data, int envId) {
    const std::string root = (data / "jsbsim").generic_string();
    return R"({"schema":"ma.config.v1","env_id":")" + std::to_string(envId) + R"(","base_seed":"18446744073709551615",
 "timing":{"dt_s":0.008333333333333333,"frame_skip":4,"decision_steps":30},
 "scenario":{"world":{"name":"installed-mission","capacity":4,"jsbsim_root":)" + jsonText(root) + R"(},
  "vehicles":[{"name":"lead","type":"jsbsim:f16c","initial":{"lat_deg":36.0,"lon_deg":-115.0,"alt_msl_m":7000,"heading_deg":0,"airspeed_ms":230}}]},
 "c2_nodes":[{"name":"crc","latitude_rad":0.6283185307179586,"longitude_rad":-2.007128639793479,"altitude_msl_m":600.0}],
 "systems":[{"platform":"lead","system_id":"1392508929"},{"platform":"crc","system_id":"1392508931"}],
 "nets":[{"name":"l16","members":["lead","crc"]}],
 "services":[{"service_id":"9007199254740995","name":"lead-ma","platform":"lead","command_capabilities":["*"],"owns_dms_ingress":true,"owns_dms_egress":true},
  {"service_id":"4","name":"crc-c2","platform":"crc","command_capabilities":[],"owns_dms_ingress":true,"owns_dms_egress":true}],
 "task":{"criteria":[{"id":"time","kind":"tick_limit","outcome":"truncated","max_ticks":3}]}})";
}

/// A mission episode through fsim/MissionEpisode.h alone: reset, one immutable snapshot per tick read twice, a request
/// queued and executed as the service's own Agent, a message ingested once by its ingress owner, the referee's documents,
/// and the earlier generation's handles refused.
std::string episode(const fs::path& data) {
    const std::string config = missionConfig(data, 7);
    MissionEpisode episode(config);
    const std::string key = episode.reset(41);
    require(key == R"({"env_id":"7","episode_generation":"1","local_episode_index":"41"})", "the episode key is not the reset's");
    MissionEndpoint lead = episode.endpoint(9007199254740995ull), crc = episode.endpoint(4);
    MissionReferee referee = episode.referee();
    const std::string first = lead.observationJsonV1();
    require(first == lead.observationJsonV1(), "a snapshot read twice differs");
    require(first.find(R"("service_id":"9007199254740995")") != std::string::npos, "an identity above 2^53 is not exact");
    const auto actions = [&](const char* service, const char* tick, const std::string& requests) {
        return std::string(R"({"schema":"ma.actions.v1","episode":)") + key + R"(,"service_id":")" + service + R"(","tick":")" + tick +
               R"(","requests":[)" + requests + "]}";
    };
    const std::string ack = lead.submitJsonV1(actions("9007199254740995", "0",
        R"({"request_id":"1","kind":"flight","operation":"new","parameters":{"mode":"hsa","heading_rad":0.5,"altitude_m":7000.0}})"));
    require(ack.find(R"("status":"queued")") != std::string::npos && ack == lead.receiptsJsonV1(), "the request was not queued");
    crc.submitJsonV1(actions("4", "0",
        R"({"request_id":"1","kind":"dms","operation":"send","parameters":{"topic":"1001","destination_system_ids":["1392508929"],"byte_length":3,"data_b64":"AQID"}})"));
    int messages = 0;
    for (int tick = 1; tick <= 3; ++tick) {
        episode.step();
        const std::string o = lead.observationJsonV1();
        for (std::size_t at = o.find(R"("topic":"1001")"); at != std::string::npos; at = o.find(R"("topic":"1001")", at + 1)) ++messages;
    }
    require(messages == 1, "a delivered message was not in exactly one snapshot");
    require(lead.observationJsonV1().find(R"("state":"executing")") != std::string::npos, "the request did not execute natively");
    require(referee.outcomeJsonV1().find(R"("reason":"truncated")") != std::string::npos, "the referee did not judge its criterion");
    require(referee.truthJsonV1().find(R"("schema":"ma.truth.v1")") != std::string::npos, "no truth document");
    require(lead.observationJsonV1().find("truth") == std::string::npos, "truth reached a snapshot");
    episode.reset(41);
    bool stale = false;
    try {
        lead.observationJsonV1();
    } catch (const std::runtime_error&) {
        stale = true;
    }
    require(stale, "an earlier generation's endpoint still answered");
    return key;
}

/// A batch of two slots through fsim/MissionBatch.h alone (M7 core): SameStep autoreset at the tick limit, the ended
/// generation's final documents kept by principal, and the referee's costs.
void batch(const fs::path& data) {
    MissionBatch b({missionConfig(data, 7), missionConfig(data, 8)}, MissionBatchOptions{MissionAutoreset::SameStep, 2, 0});
    b.reset();
    require(b.size() == 2 && b.status(1).generation == 1, "the batch did not reset its slots");
    for (int t = 0; t < 3; ++t) b.step();
    const MissionSlotStatus st = b.status(0);
    require(st.ended && st.reset && st.generation == 2 && st.tick == 0, "the slot did not end and reset in the same step");
    require(b.finalObservationJsonV1(0, 9007199254740995ull).find(R"("tick":"3")") != std::string::npos, "no final observation of the ended generation");
    require(b.finalOutcomeJsonV1(0).find(R"("reason":"truncated")") != std::string::npos, "no final outcome");
    require(b.referee(1).costsJsonV1().find(R"("schema":"ma.costs.v1")") != std::string::npos, "no costs document");
    require(b.endpoint(0, 9007199254740995ull).observationJsonV1().find("ma.costs") == std::string::npos, "costs reached a snapshot");
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: ma_consumer_cpp <installed-prefix>\n");
        return 1;
    }
    try {
        const fs::path prefix = fs::canonical(argv[1]);
        checkModule(prefix, L"libfsim.dll");
        const fs::path data = prefix / "share" / "flightsim";
        const std::string a = ownership(data, true), b = ownership(data, false);
        require(a == b, "ownership depends on the order the agents were made in");
        require(a == "flight flight comms comms", "ownership is not the bindings'");
        worlds(data);
        reports(data);
        const std::string key = episode(data);
        batch(data);
        std::printf("fsim %s installed C++ consumer: ownership \"%s\" in either creation order; legal reports with provenance; "
                    "mission episode %s; a mission batch\n",
                    fsim::version(), a.c_str(), key.c_str());
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed MA consumer (C++) failed: %s\n", error.what());
        return 1;
    }
}
