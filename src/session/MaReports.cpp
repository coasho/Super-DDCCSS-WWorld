#include "session/MaReports.h"

#include "session/Dms.h"
#include "session/World.h"

#include <cmath>
#include <utility>

namespace fsim::ma {

const char* availabilityName(Availability availability) noexcept {
    switch (availability) {
    case Availability::Available: return "available";
    case Availability::Unavailable: return "unavailable";
    case Availability::NotApplicable: return "not_applicable";
    case Availability::NotOwner: return "not_owner";
    }
    return "?";
}

const char* validityName(Validity validity) noexcept {
    switch (validity) {
    case Validity::Valid: return "valid";
    case Validity::Stale: return "stale";
    case Validity::Invalid: return "invalid";
    case Validity::Unknown: return "unknown";
    }
    return "?";
}

const char* sourceName(Source source) noexcept {
    switch (source) {
    case Source::OwnNavigation: return "own_navigation";
    case Source::OwnSensor: return "own_sensor";
    case Source::OwnPlatform: return "own_platform";
    case Source::MaCore: return "ma_core";
    case Source::Dms: return "dms";
    case Source::Link: return "link";
    }
    return "?";
}

} // namespace fsim::ma

namespace fsim::session {

namespace {

/// A held report sampled at `sample` (unknown: its source states none), reached at `receive`, read at `now`.
ma::ReportMeta held(ma::Source source, double sample, double receive, double now, const ma::ReportOptions& options) {
    ma::ReportMeta m;
    m.source = source;
    m.validity = ma::Validity::Valid;
    m.sampleTimeS = sample, m.receiveTimeS = receive;
    if (std::isfinite(sample)) {
        m.ageS = now - sample;
        if (std::isfinite(options.staleAfterS) && m.ageS > options.staleAfterS) m.validity = ma::Validity::Stale;
    }
    return m;
}

} // namespace

std::vector<std::uint16_t> payloadIndices(const World& world, std::uint32_t vehicle) {
    std::vector<std::uint16_t> out;
    if (const ms::Host* host = world.missionSystems(vehicle))
        for (const ms::PayloadInfo& p : host->payloads()) out.push_back(p.index);
    return out;
}

std::vector<std::uint16_t> radarPayloads(const World& world, std::uint32_t vehicle) {
    std::vector<std::uint16_t> out;
    if (const ms::Host* host = world.missionSystems(vehicle))
        for (const ms::PayloadInfo& p : host->payloads())
            if (p.family == ms::Family::AirRadar) out.push_back(p.index);
    return out;
}

std::vector<ms::StrikeStatus> equipmentStrikes(const World& world, const AgentIdentity& agent) {
    std::vector<ms::StrikeStatus> out;
    const VehicleInfo* info = world.info(agent.vehicle);
    if (!info || !info->alive) return out;
    if (const ms::Host* host = world.missionSystems(agent.vehicle))
        for (const ms::ActivityRecord& activity : host->activities()) {
            if (!agentActivity(agent, activity.id, &world)) continue;
            auto strike = world.strikeStatus(activity.id);
            if (!strike) continue;
            const World::Designation* designation = world.designation(strike->target);
            if (!designation || designation->owner != agent.vehicle) strike->target = 0;
            out.push_back(std::move(*strike));
        }
    return out;
}

std::vector<comm::PendingConsent> equipmentConsents(const World& world, const AgentIdentity& agent) {
    const VehicleInfo* info = world.info(agent.vehicle);
    if ((!info || !info->alive) && !world.c2Node(agent.vehicle)) return {};
    if (!agentDmsIngress(agent, world)) return {};
    // The legacy inbox accessor is mutable so answerConsent can mark an item. This read immediately copies const
    // values and does not consume, create or answer anything, matching the existing const C consent reads.
    const auto* consents = const_cast<Dms&>(world.dms()).consentsOf(agent.vehicle);
    return consents ? *consents : std::vector<comm::PendingConsent>{};
}

MaReportMetas reportMetas(const World& world, const AgentIdentity& agent, MaReportKind kind, const ma::ReportOptions& options) {
    MaReportMetas out;
    const std::uint32_t id = agent.vehicle;
    const bool node = world.c2Node(id) != nullptr;
    const VehicleInfo* info = world.info(id);
    const bool vehicle = info && info->alive;
    if (!node && !vehicle) return out; // (its platform is gone: Unavailable, nothing listed)
    const double now = world.simTime();
    out.availability = ma::Availability::Available;
    switch (kind) {
    case MaReportKind::Navigation: {
        const effects::SensedState* sensed = vehicle ? world.sensedState(id) : nullptr;
        if (!sensed) {
            out.availability = node ? ma::Availability::NotApplicable : ma::Availability::Unavailable;
            break;
        }
        ma::ReportMeta m = held(ma::Source::OwnNavigation, sensed->state.simTime, now, now, options);
        if (!sensed->gnssValid || !sensed->airDataValid) m.validity = ma::Validity::Invalid; // (its own sensors say so)
        out.metas.push_back(m);
        break;
    }
    case MaReportKind::Tracks:
        if (!vehicle) {
            out.availability = ma::Availability::NotApplicable;
            break;
        }
        for (const std::uint16_t payload : radarPayloads(world, id))
            for (const ms::AmtiTrack& t : world.amtiTracks(id, payload)) {
                ma::ReportMeta m = held(ma::Source::OwnSensor, t.lastUpdateS, now, now, options);
                m.payload = payload;
                out.metas.push_back(m);
            }
        break;
    case MaReportKind::Contacts:
        if (!vehicle) {
            out.availability = ma::Availability::NotApplicable;
            break;
        }
        for (const std::uint16_t payload : payloadIndices(world, id))
            for (const ms::Contact& c : world.contacts(id, payload)) {
                ma::ReportMeta m = held(ma::Source::OwnSensor, c.timeS, now, now, options);
                m.payload = payload;
                out.metas.push_back(m);
            }
        break;
    case MaReportKind::LinkTracks:
        for (const comm::LinkTrack& t : world.dms().linkTracks(id)) {
            ma::ReportMeta m = held(ma::Source::Link, t.report.timeS, t.receivedS, now, options);
            m.system = t.source, m.net = t.net;
            m.payload = static_cast<std::uint16_t>(t.report.payload);
            out.metas.push_back(m);
        }
        break;
    case MaReportKind::Peers: {
        // The position report a peer holds, by the delivery that brought it - not by a later heartbeat's, which
        // comm::PeerReport's own net and receivedS would give. A peer heard only by its heartbeat holds no position:
        // that heartbeat's delivery, and no sample time.
        const std::vector<comm::PeerReport> peers = world.dms().peers(id);
        const std::vector<Dms::PeerProvenance> from = world.dms().peerProvenance(id);
        for (std::size_t i = 0; i < peers.size(); ++i) {
            const comm::PeerReport& p = peers[i];
            ma::ReportMeta m = p.hasPosition ? held(ma::Source::Link, p.position.timeS, from[i].positionReceivedS, now, options)
                                             : held(ma::Source::Link, ma::kUnknown, from[i].heartbeatReceivedS, now, options);
            m.system = p.system, m.net = p.hasPosition ? from[i].positionNet : from[i].heartbeatNet;
            out.metas.push_back(m);
        }
        break;
    }
    case MaReportKind::PeerHeartbeats: {
        // Each peer's last heartbeat, by its own delivery; for a peer no heartbeat was heard from, nothing is held.
        const std::vector<comm::PeerReport> peers = world.dms().peers(id);
        const std::vector<Dms::PeerProvenance> from = world.dms().peerProvenance(id);
        for (std::size_t i = 0; i < peers.size(); ++i) {
            ma::ReportMeta m;
            if (from[i].heartbeat) {
                m = held(ma::Source::Link, from[i].heartbeatTimeS, from[i].heartbeatReceivedS, now, options);
                m.net = from[i].heartbeatNet;
            } else {
                m.source = ma::Source::Link; // (validity Unknown, its times unknown)
            }
            m.system = peers[i].system;
            out.metas.push_back(m);
        }
        break;
    }
    case MaReportKind::Tasks: {
        if (!vehicle) {
            out.availability = ma::Availability::NotApplicable; // (an MA core gives a C2 node no task)
            break;
        }
        if (const auto* tasks = world.maCore().tasks(id))
            for (const comm::ReceivedTask& t : *tasks) {
                ma::ReportMeta m = held(ma::Source::Link, ma::kUnknown, t.receivedS, now, options); // (an order states no time of its own)
                m.system = t.from;
                out.metas.push_back(m);
            }
        break;
    }
    case MaReportKind::Authority:
        if (!vehicle) {
            out.availability = ma::Availability::NotApplicable;
            break;
        }
        out.metas.push_back(held(ma::Source::MaCore, now, now, now, options)); // (its MA core's state, as it stands now)
        break;
    case MaReportKind::Stores:
        if (!vehicle) {
            out.availability = ma::Availability::NotApplicable;
            break;
        }
        for (const ms::StoreInfo& store : world.stores(id)) {
            ma::ReportMeta meta = held(ma::Source::OwnPlatform, now, now, now, options);
            meta.payload = store.payload;
            out.metas.push_back(meta);
        }
        break;
    case MaReportKind::Strikes:
        if (!vehicle) {
            out.availability = ma::Availability::NotApplicable;
            break;
        }
        for (const auto& strike : equipmentStrikes(world, agent)) {
            (void)strike;
            out.metas.push_back(held(ma::Source::OwnPlatform, now, now, now, options));
        }
        break;
    case MaReportKind::Countermeasures: {
        bool installed = false;
        if (vehicle)
            if (const ms::Host* host = world.missionSystems(id))
                for (const ms::PayloadInfo& payload : host->payloads())
                    installed = installed || payload.family == ms::Family::Countermeasures;
        if (!installed) {
            out.availability = ma::Availability::NotApplicable;
            break;
        }
        out.metas.push_back(held(ma::Source::OwnPlatform, now, now, now, options));
        break;
    }
    case MaReportKind::Consents:
        if (!agentDmsIngress(agent, world)) {
            out.availability = ma::Availability::NotOwner;
            break;
        }
        for (const comm::PendingConsent& consent : equipmentConsents(world, agent)) {
            ma::ReportMeta meta = held(ma::Source::Dms, consent.request.timeS, consent.receivedS, now, options);
            meta.system = consent.from;
            // PendingConsent retains its sender and delivery time, but not its incoming net. Zero stays unknown.
            out.metas.push_back(meta);
        }
        break;
    case MaReportKind::Count: out.availability = ma::Availability::Unavailable; break;
    }
    return out;
}

} // namespace fsim::session
