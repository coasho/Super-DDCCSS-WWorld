// `ma.config.v1` (MissionConfig.h; docs/sdk/mission-training.md, sections 7, 9, 13 and 14).

#include "training/native/mission/MissionConfig.h"

#include "fsim/Scenario.h"
#include "training/native/mission/MissionJson.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>

namespace fsim::mission {

namespace {

constexpr double kHalfPi = 1.5707963267948966, kPi = 3.141592653589793;
constexpr std::size_t kMaxConfigBytes = 16777216; // (the largest max_document_bytes a configuration may set)

std::string item(const std::string& path, std::size_t i) { return path + "[" + std::to_string(i) + "]"; }

/// The scenario section: its own documented format (scenarios.md), its world section held to what the recipe admits.
Scenario readScenario(const core::Json& v, const std::string& path) {
    Fields f(v, path);
    if (f.has("vecenv")) malformed(path, "a scenario's vecenv section does not configure a mission episode");
    if (const core::Json* world = f.optional("world")) {
        Fields w(*world, f.path("world"));
        if (const core::Json* x = w.optional("name")) nameOf(*x, w.path("name"));
        if (const core::Json* x = w.optional("capacity")) integerOf(*x, w.path("capacity"), 1, 65536);
        if (const core::Json* x = w.optional("jsbsim_root")) stringOf(*x, w.path("jsbsim_root"), 1024);
        if (const core::Json* x = w.optional("terrain")) boolOf(*x, w.path("terrain"));
        if (const core::Json* x = w.optional("terrain_url")) stringOf(*x, w.path("terrain_url"), 1024);
        if (const core::Json* x = w.optional("terrain_zoom")) integerOf(*x, w.path("terrain_zoom"), 0, 24);
        if (const core::Json* x = w.optional("terrain_offline")) boolOf(*x, w.path("terrain_offline"));
        w.close();
    }
    if (const core::Json* e = f.optional("environment"); e && !e->isObject()) malformed(f.path("environment"), "an object is required");
    if (const core::Json* e = f.optional("effects")) {
        const auto& effects = arrayOf(*e, f.path("effects"), 0, ~std::size_t{0});
        for (std::size_t i = 0; i < effects.size(); ++i)
            if (!effects[i].isObject()) malformed(item(f.path("effects"), i), "an object is required");
    }
    const auto& vehicles = arrayOf(f.required("vehicles"), f.path("vehicles"), 0, 4096);
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        if (!vehicles[i].isObject()) malformed(item(f.path("vehicles"), i), "an object is required");
        if (!vehicles[i].find("name")) malformed(item(f.path("vehicles"), i), "name is required");
    }
    f.close();
    try {
        return parseScenario(v.dump(), "ma.config.v1 scenario");
    } catch (const std::exception& e) {
        malformed(path, e.what());
    }
}

std::uint32_t permissionBit(std::size_t i) noexcept {
    static const std::uint32_t bits[] = {comm::permission::kFlight, comm::permission::kControl, comm::permission::kTask, comm::permission::kWeapons,
                                         comm::permission::kOverride, comm::permission::kSettings, comm::permission::kPackage, comm::permission::kRoles};
    return bits[i];
}

Criterion readCriterion(const core::Json& v, const std::string& path) {
    Fields f(v, path);
    Criterion c;
    c.id = nameOf(f.required("id"), f.path("id"));
    const std::size_t kind = enumOf(f.required("kind"), f.path("kind"), {"tick_limit", "region_reached", "c2_task_state"});
    const std::size_t outcome = enumOf(f.required("outcome"), f.path("outcome"), {"success", "failure", "truncated"});
    c.kind = static_cast<Criterion::Kind>(kind);
    c.outcome = static_cast<Criterion::Outcome>(outcome);
    switch (c.kind) {
    case Criterion::Kind::TickLimit:
        if (c.outcome != Criterion::Outcome::Truncated) malformed(f.path("outcome"), "a tick limit truncates");
        c.maxTicks = static_cast<std::uint64_t>(integerOf(f.required("max_ticks"), f.path("max_ticks"), 1, 2147483647));
        break;
    case Criterion::Kind::RegionReached:
        if (c.outcome == Criterion::Outcome::Truncated) malformed(f.path("outcome"), "success or failure");
        c.platform = nameOf(f.required("platform"), f.path("platform"));
        c.latitudeRad = numberOf(f.required("latitude_rad"), f.path("latitude_rad"), within(-kHalfPi, kHalfPi));
        c.longitudeRad = numberOf(f.required("longitude_rad"), f.path("longitude_rad"), within(-kPi, kPi));
        c.radiusM = numberOf(f.required("radius_m"), f.path("radius_m"), above(0.0));
        if (const core::Json* x = f.optional("altitude_min_m")) c.altitudeMinM = numberOf(*x, f.path("altitude_min_m"));
        if (const core::Json* x = f.optional("altitude_max_m")) c.altitudeMaxM = numberOf(*x, f.path("altitude_max_m"));
        if (const core::Json* x = f.optional("dwell_s")) c.dwellS = numberOf(*x, f.path("dwell_s"), atLeast(0.0));
        if (c.altitudeMinM > c.altitudeMaxM) malformed(path, "altitude_min_m is above altitude_max_m");
        break;
    case Criterion::Kind::C2TaskState: {
        if (c.outcome == Criterion::Outcome::Truncated) malformed(f.path("outcome"), "success or failure");
        c.platform = nameOf(f.required("platform"), f.path("platform"));
        c.fromPlatform = nameOf(f.required("from_platform"), f.path("from_platform"));
        static const comm::ReceivedTask::State states[] = {comm::ReceivedTask::Accepted, comm::ReceivedTask::Rejected, comm::ReceivedTask::Completed,
                                                           comm::ReceivedTask::Failed};
        c.state = states[enumOf(f.required("state"), f.path("state"), {"accepted", "rejected", "completed", "failed"})];
        break;
    }
    }
    f.close();
    return c;
}

std::uint32_t limit(Fields& f, std::string_view name, std::int64_t min, std::int64_t max, std::uint32_t fallback) {
    const core::Json* v = f.optional(name);
    return v ? static_cast<std::uint32_t>(integerOf(*v, f.path(name), min, max)) : fallback;
}

/// Section 7's and section 9's rules a schema cannot state (tools/validation/ma_contract/check.py's config_rules).
void crossCheck(const Config& c, const std::vector<std::string>& systemPlatforms, const std::vector<std::uint32_t>& systemIds,
                const std::vector<std::pair<std::string, std::vector<std::string>>>& nets, const std::vector<std::string>& disseminated,
                const std::vector<std::string>& rbac) {
    const std::string root = "ma.config.v1: $";
    std::set<std::string> names;
    for (const std::string& n : c.platforms)
        if (!names.insert(n).second) malformed(root, "platform " + n + " is named twice");
    const auto known = [&](const std::string& name, const std::string& where) {
        if (!names.count(name)) malformed(where, name + " is no platform of the scenario or its C2 nodes");
    };
    const auto once = [&](const std::vector<std::string>& platforms, const char* section) {
        std::set<std::string> seen;
        for (std::size_t i = 0; i < platforms.size(); ++i) {
            known(platforms[i], item(root + "." + section, i));
            if (!seen.insert(platforms[i]).second) malformed(item(root + "." + section, i), platforms[i] + " is configured twice");
        }
    };
    once(systemPlatforms, "systems");
    once(disseminated, "dissemination");
    once(rbac, "rbac");
    if (std::set<std::uint32_t>(systemIds.begin(), systemIds.end()).size() != systemIds.size()) malformed(root + ".systems", "a system_id is assigned twice");
    std::set<std::string> netNames;
    for (std::size_t i = 0; i < nets.size(); ++i) {
        if (!netNames.insert(nets[i].first).second) malformed(root + ".nets", "a net is named twice");
        std::set<std::string> members;
        for (const std::string& m : nets[i].second) {
            known(m, item(root + ".nets", i));
            if (!members.insert(m).second) malformed(item(root + ".nets", i), "a member is listed twice");
        }
    }
    std::set<std::uint64_t> ids;
    std::set<std::string> serviceNames;
    std::map<std::string, std::vector<const ServiceConfig*>> bound;
    for (std::size_t i = 0; i < c.services.size(); ++i) {
        const ServiceConfig& s = c.services[i];
        if (!ids.insert(s.serviceId).second) malformed(root + ".services", "a service_id is used twice");
        if (!serviceNames.insert(s.name).second) malformed(root + ".services", "a name is used twice");
        known(s.platform, root + ".services");
        bound[s.platform].push_back(&s);
        if (std::set<std::string>(s.capabilities.begin(), s.capabilities.end()).size() != s.capabilities.size())
            malformed(root + ".services", "service " + std::to_string(s.serviceId) + " lists a command capability twice");
        if (std::count(s.capabilities.begin(), s.capabilities.end(), "*") && s.capabilities.size() != 1)
            malformed(root + ".services", "\"*\" stands alone");
    }
    for (const auto& [platform, group] : bound) {
        const auto owners = [&](bool ServiceConfig::*side) {
            return std::count_if(group.begin(), group.end(), [&](const ServiceConfig* s) { return s->*side; });
        };
        if (owners(&ServiceConfig::ownsDmsIngress) != 1) malformed(root + ".services", platform + " needs exactly one DMS ingress owner");
        if (owners(&ServiceConfig::ownsDmsEgress) != 1) malformed(root + ".services", platform + " needs exactly one DMS egress owner");
        std::vector<std::string> held;
        for (const ServiceConfig* s : group) held.insert(held.end(), s->capabilities.begin(), s->capabilities.end());
        if (std::count(held.begin(), held.end(), "*") && held.size() != 1)
            malformed(root + ".services", platform + " gives every command capability to one service and some to another");
        if (std::set<std::string>(held.begin(), held.end()).size() != held.size())
            malformed(root + ".services", platform + " gives a command capability to two services");
    }
    std::set<std::string> criteria;
    for (const Criterion& k : c.criteria) {
        if (!criteria.insert(k.id).second) malformed(root + ".task.criteria", "an id is used twice");
        if (!k.platform.empty()) known(k.platform, root + ".task.criteria");
        if (!k.fromPlatform.empty()) known(k.fromPlatform, root + ".task.criteria");
    }
}

} // namespace

const ServiceConfig* Config::service(std::uint64_t id) const noexcept {
    for (const ServiceConfig& s : services)
        if (s.serviceId == id) return &s;
    return nullptr;
}

std::shared_ptr<const Config> parseConfig(std::string_view text) {
    if (text.size() > kMaxConfigBytes) malformed("ma.config.v1", "larger than any configuration's max_document_bytes");
    const core::Json doc = parseStrict(text, "ma.config.v1");
    auto c = std::make_shared<Config>();
    env::EpisodeConfig& e = c->episode;
    Fields f(doc, "ma.config.v1: $");
    if (!f.required("schema").isString() || f.required("schema").asString() != "ma.config.v1") malformed(f.path("schema"), "\"ma.config.v1\" is required");
    c->envId = idOf(f.required("env_id"), f.path("env_id"));
    c->baseSeed = idOf(f.required("base_seed"), f.path("base_seed"));
    e.envId = c->envId, e.baseSeed = c->baseSeed;
    {
        Fields t(f.required("timing"), f.path("timing"));
        e.dtS = numberOf(t.required("dt_s"), t.path("dt_s"), Bounds{0.0, 0.1, true});
        e.frameSkip = static_cast<int>(integerOf(t.required("frame_skip"), t.path("frame_skip"), 1, 1000));
        e.decisionSteps = static_cast<unsigned>(integerOf(t.required("decision_steps"), t.path("decision_steps"), 1, 100000));
        t.close();
    }
    Scenario scenario = readScenario(f.required("scenario"), f.path("scenario"));
    e.name = scenario.world.name;
    for (const ScenarioVehicle& v : scenario.vehicles) {
        if (v.count <= 1) c->platforms.push_back(v.spec.name);
        else
            for (unsigned i = 1; i <= v.count; ++i) c->platforms.push_back(v.spec.name + "-" + std::to_string(i));
    }
    e.scenario = std::move(scenario);

    if (const core::Json* nodes = f.optional("c2_nodes")) {
        const auto& list = arrayOf(*nodes, f.path("c2_nodes"), 0, 64);
        for (std::size_t i = 0; i < list.size(); ++i) {
            Fields n(list[i], item(f.path("c2_nodes"), i));
            comm::C2NodeSpec spec;
            spec.name = nameOf(n.required("name"), n.path("name"));
            spec.latitudeRad = numberOf(n.required("latitude_rad"), n.path("latitude_rad"), within(-kHalfPi, kHalfPi));
            spec.longitudeRad = numberOf(n.required("longitude_rad"), n.path("longitude_rad"), within(-kPi, kPi));
            spec.altitudeMslM = numberOf(n.required("altitude_msl_m"), n.path("altitude_msl_m"));
            n.close();
            c->platforms.push_back(spec.name);
            e.c2Nodes.push_back(std::move(spec));
        }
    }
    std::vector<std::string> systemPlatforms, disseminated, rbacPlatforms;
    std::vector<std::uint32_t> systemIds;
    std::vector<std::pair<std::string, std::vector<std::string>>> netMembers;
    if (const core::Json* systems = f.optional("systems")) {
        const auto& list = arrayOf(*systems, f.path("systems"), 0, 4160);
        for (std::size_t i = 0; i < list.size(); ++i) {
            Fields s(list[i], item(f.path("systems"), i));
            env::EpisodeSystem sys;
            sys.platform = nameOf(s.required("platform"), s.path("platform"));
            sys.system = static_cast<comm::SystemId>(idOf(s.required("system_id"), s.path("system_id"), 32, true));
            if (const core::Json* t = s.optional("tail_number")) sys.tailNumber = boundedString(*t, s.path("tail_number"), 23);
            s.close();
            systemPlatforms.push_back(sys.platform);
            systemIds.push_back(sys.system);
            e.systems.push_back(std::move(sys));
        }
    }
    if (const core::Json* nets = f.optional("nets")) {
        const auto& list = arrayOf(*nets, f.path("nets"), 0, 64);
        for (std::size_t i = 0; i < list.size(); ++i) {
            Fields n(list[i], item(f.path("nets"), i));
            env::EpisodeNet net;
            net.spec.name = nameOf(n.required("name"), n.path("name"));
            const auto& members = arrayOf(n.required("members"), n.path("members"), 0, 4160);
            for (std::size_t m = 0; m < members.size(); ++m) net.members.push_back(nameOf(members[m], item(n.path("members"), m)));
            if (const core::Json* spec = n.optional("spec")) {
                Fields p(*spec, n.path("spec"));
                comm::NetSpec& s = net.spec;
                if (const core::Json* x = p.optional("range_m")) s.rangeM = numberOf(*x, p.path("range_m"), above(0.0));
                if (const core::Json* x = p.optional("frequency_hz")) s.frequencyHz = numberOf(*x, p.path("frequency_hz"), above(0.0));
                if (const core::Json* x = p.optional("line_of_sight")) s.lineOfSight = boolOf(*x, p.path("line_of_sight"));
                if (const core::Json* x = p.optional("latency_s")) s.latencyS = numberOf(*x, p.path("latency_s"), atLeast(0.0));
                if (const core::Json* x = p.optional("jitter_s")) s.jitterS = numberOf(*x, p.path("jitter_s"), atLeast(0.0));
                if (const core::Json* x = p.optional("loss_probability")) s.lossProbability = numberOf(*x, p.path("loss_probability"), within(0.0, 1.0));
                if (const core::Json* x = p.optional("data_rate_bps")) s.dataRateBps = numberOf(*x, p.path("data_rate_bps"), above(0.0));
                if (const core::Json* x = p.optional("max_queue_s")) s.maxQueueS = numberOf(*x, p.path("max_queue_s"), atLeast(0.0));
                if (const core::Json* x = p.optional("reliable_timeout_s")) s.reliableTimeoutS = numberOf(*x, p.path("reliable_timeout_s"), atLeast(0.0));
                if (const core::Json* x = p.optional("retry_s")) s.retryS = numberOf(*x, p.path("retry_s"), above(0.0));
                if (const core::Json* x = p.optional("max_payload_bytes"))
                    s.maxPayloadBytes = static_cast<std::uint32_t>(integerOf(*x, p.path("max_payload_bytes"), 1, 65536));
                if (const core::Json* x = p.optional("header_bytes")) s.headerBytes = static_cast<std::uint32_t>(integerOf(*x, p.path("header_bytes"), 0, 65536));
                p.close();
            }
            n.close();
            netMembers.emplace_back(net.spec.name, net.members);
            e.nets.push_back(std::move(net));
        }
    }
    if (const core::Json* plans = f.optional("dissemination")) {
        const auto& list = arrayOf(*plans, f.path("dissemination"), 0, 4160);
        for (std::size_t i = 0; i < list.size(); ++i) {
            Fields d(list[i], item(f.path("dissemination"), i));
            env::EpisodeDissemination plan;
            plan.platform = nameOf(d.required("platform"), d.path("platform"));
            comm::DisseminationPlan& p = plan.plan;
            if (const core::Json* x = d.optional("position_period_s")) p.positionPeriodS = numberOf(*x, d.path("position_period_s"), atLeast(0.0));
            if (const core::Json* x = d.optional("status_period_s")) p.statusPeriodS = numberOf(*x, d.path("status_period_s"), atLeast(0.0));
            if (const core::Json* x = d.optional("tracks_period_s")) p.tracksPeriodS = numberOf(*x, d.path("tracks_period_s"), atLeast(0.0));
            if (const core::Json* x = d.optional("peer_timeout_s")) p.peerTimeoutS = numberOf(*x, d.path("peer_timeout_s"), above(0.0));
            if (const core::Json* x = d.optional("track_timeout_s")) p.trackTimeoutS = numberOf(*x, d.path("track_timeout_s"), above(0.0));
            d.close();
            disseminated.push_back(plan.platform);
            e.dissemination.push_back(std::move(plan));
        }
    }
    if (const core::Json* rbac = f.optional("rbac")) {
        const auto& list = arrayOf(*rbac, f.path("rbac"), 0, 4160);
        for (std::size_t i = 0; i < list.size(); ++i) {
            Fields r(list[i], item(f.path("rbac"), i));
            env::EpisodeRbac entry;
            entry.platform = nameOf(r.required("platform"), r.path("platform"));
            const auto& roles = arrayOf(r.required("roles"), r.path("roles"), 1, 32);
            for (std::size_t k = 0; k < roles.size(); ++k) {
                Fields role(roles[k], item(r.path("roles"), k));
                comm::Role native;
                native.id = static_cast<std::uint32_t>(idOf(role.required("role_id"), role.path("role_id"), 32, true));
                native.name = boundedString(role.required("name"), role.path("name"), 23);
                if (native.name.empty()) malformed(role.path("name"), "a role has a name");
                const auto& permissions = arrayOf(role.required("permissions"), role.path("permissions"), 0, 8);
                for (std::size_t p = 0; p < permissions.size(); ++p)
                    native.permissions |= permissionBit(enumOf(permissions[p], item(role.path("permissions"), p),
                                                               {"flight", "control", "task", "weapons", "override", "settings", "package", "roles"}));
                if (const core::Json* x = role.optional("approves_strikes")) native.approvesStrikes = boolOf(*x, role.path("approves_strikes"));
                role.close();
                entry.config.roles.push_back(std::move(native));
            }
            r.close();
            rbacPlatforms.push_back(entry.platform);
            e.rbac.push_back(std::move(entry));
        }
    }
    {
        const auto& list = arrayOf(f.required("services"), f.path("services"), 1, 256);
        for (std::size_t i = 0; i < list.size(); ++i) {
            Fields s(list[i], item(f.path("services"), i));
            ServiceConfig svc;
            svc.serviceId = idOf(s.required("service_id"), s.path("service_id"), 64, true);
            svc.name = nameOf(s.required("name"), s.path("name"));
            svc.platform = nameOf(s.required("platform"), s.path("platform"));
            const auto& caps = arrayOf(s.required("command_capabilities"), s.path("command_capabilities"), 0, 256);
            for (std::size_t k = 0; k < caps.size(); ++k) {
                const std::string where = item(s.path("command_capabilities"), k);
                const std::string& cap = stringOf(caps[k], where, 127);
                // ^(?:\*|[a-z][a-z0-9_]*(?:\.[a-z][a-z0-9_]*)+(?::[A-Za-z0-9 _.-]+)?)$
                bool ok = cap == "*";
                if (!ok) {
                    const std::size_t colon = cap.find(':');
                    const std::string head = cap.substr(0, colon);
                    std::size_t parts = 0, from = 0;
                    ok = true;
                    for (;;) {
                        const std::size_t dot = head.find('.', from);
                        ok = ok && lowerIdentifier(head.substr(from, dot == std::string::npos ? std::string::npos : dot - from), 127);
                        ++parts;
                        if (dot == std::string::npos) break;
                        from = dot + 1;
                    }
                    ok = ok && parts >= 2;
                    if (ok && colon != std::string::npos) {
                        const std::string tail = cap.substr(colon + 1);
                        ok = !tail.empty() && std::all_of(tail.begin(), tail.end(), [](char ch) {
                            return (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == ' ' || ch == '_' || ch == '.' ||
                                   ch == '-';
                        });
                    }
                }
                if (!ok) malformed(where, "a discovery id as the platform reports it, or \"*\"");
                svc.capabilities.push_back(cap);
            }
            svc.ownsDmsIngress = boolOf(s.required("owns_dms_ingress"), s.path("owns_dms_ingress"));
            svc.ownsDmsEgress = boolOf(s.required("owns_dms_egress"), s.path("owns_dms_egress"));
            s.close();
            c->services.push_back(std::move(svc));
        }
    }
    if (const core::Json* reports = f.optional("reports")) {
        Fields r(*reports, f.path("reports"));
        if (const core::Json* x = r.optional("stale_after_s")) c->staleAfterS = numberOf(*x, r.path("stale_after_s"), above(0.0));
        r.close();
    }
    if (const core::Json* limits = f.optional("limits")) {
        Fields l(*limits, f.path("limits"));
        Limits& m = c->limits;
        m.completedRequestRetentionTicks = limit(l, "completed_request_retention_ticks", 1, 65536, m.completedRequestRetentionTicks);
        m.maxInFlightRequests = limit(l, "max_in_flight_requests", 1, 65536, m.maxInFlightRequests);
        m.maxRequestsPerDocument = limit(l, "max_requests_per_document", 1, 1024, m.maxRequestsPerDocument);
        m.maxDocumentBytes = limit(l, "max_document_bytes", 1024, 16777216, m.maxDocumentBytes);
        m.maxMessageBytes = limit(l, "max_message_bytes", 1, 65536, m.maxMessageBytes);
        l.close();
    }
    if (const core::Json* task = f.optional("task")) {
        Fields t(*task, f.path("task"));
        const auto& list = arrayOf(t.required("criteria"), t.path("criteria"), 0, 64);
        for (std::size_t i = 0; i < list.size(); ++i) c->criteria.push_back(readCriterion(list[i], item(t.path("criteria"), i)));
        t.close();
    }
    if (const core::Json* execution = f.optional("execution")) {
        Fields x(*execution, f.path("execution"));
        if (const core::Json* v = x.optional("workers")) e.workers = static_cast<unsigned>(integerOf(*v, x.path("workers"), 1, 256));
        if (const core::Json* v = x.optional("diagnostic")) e.diagnostic = boolOf(*v, x.path("diagnostic"));
        if (const core::Json* v = x.optional("publish")) e.publish = boolOf(*v, x.path("publish"));
        if (const core::Json* v = x.optional("record_dir")) e.recordDir = stringOf(*v, x.path("record_dir"), 1024, 1);
        x.close();
        if ((e.publish || !e.recordDir.empty()) && !e.diagnostic)
            malformed(f.path("execution"), "publication and recording are diagnostic only: diagnostic true is required");
    }
    if (const core::Json* extensions = f.optional("extensions")) {
        Fields x(*extensions, f.path("extensions"));
        for (const auto& [key, value] : extensions->asObject())
            malformed(x.path(key), "no installed extension definition registers this key (section 15): refused, not ignored");
    }
    f.close();
    if (text.size() > c->limits.maxDocumentBytes) malformed("ma.config.v1", "larger than its own max_document_bytes");
    std::sort(c->services.begin(), c->services.end(), [](const ServiceConfig& a, const ServiceConfig& b) { return a.serviceId < b.serviceId; });
    crossCheck(*c, systemPlatforms, systemIds, netMembers, disseminated, rbacPlatforms);
    return c;
}

} // namespace fsim::mission
