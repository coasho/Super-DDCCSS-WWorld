// MA-service binding and legal reports from an installed SDK: its public headers and its library, nothing else
// (docs/sdk/mission-training.md, sections 9 and 10).
#include <fsim/Agent.h>
#include <fsim/MAReports.h>
#include <fsim/MAService.h>
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
    // its capability its owner's to command, and no one else's
    control::HsaCommand hsa;
    hsa.headingRad = 1.0, hsa.speed = 230.0, hsa.altitudeM = 6000.0;
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
        reports(data);
        std::printf("fsim %s installed C++ consumer: ownership \"%s\" in either creation order; legal reports with provenance\n",
                    fsim::version(), a.c_str());
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed MA consumer (C++) failed: %s\n", error.what());
        return 1;
    }
}
