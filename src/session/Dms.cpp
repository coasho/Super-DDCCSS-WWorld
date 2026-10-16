#include "session/Dms.h"

#include "simulation/mission_systems/rf/Path.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace fsim::session {

namespace {
constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
constexpr std::size_t kSentKept = 256; ///< each system's last sends whose status it can read
constexpr double kSightRefreshS = 1.0; ///< a pair's line of sight is worked out again after this long
} // namespace

// --- the range's ---------------------------------------------------------------------------------------------------

comm::NetId Dms::createNet(const comm::NetSpec& spec) {
    Net n;
    n.info.id = static_cast<comm::NetId>(nets_.size()) + 1;
    n.info.name = spec.name.empty() ? "net" + std::to_string(n.info.id) : spec.name;
    n.info.spec = spec;
    n.info.spec.name = n.info.name;
    nets_.push_back(std::move(n));
    return nets_.back().info.id;
}

comm::SystemId Dms::assign(std::uint32_t entity, comm::SystemId want, const std::string& tailNumber) {
    if (entity == 0) return comm::kNoSystem;
    if (want != comm::kNoSystem) {
        const auto taken = bySystem_.find(want);
        if (taken != bySystem_.end() && systems_[taken->second].entity != entity) return comm::kNoSystem;
    }
    if (System* s = system(entity)) {
        if (want != comm::kNoSystem && want != s->id) return comm::kNoSystem;
        if (!tailNumber.empty()) s->tail = tailNumber;
        return s->id;
    }
    if (want == comm::kNoSystem) {
        while (bySystem_.count(nextSystem_)) ++nextSystem_;
        want = nextSystem_++;
    }
    System s;
    s.id = want, s.entity = entity, s.tail = tailNumber;
    byEntity_[entity] = systems_.size();
    bySystem_[want] = systems_.size();
    systems_.push_back(std::move(s));
    return want;
}

bool Dms::join(std::uint32_t entity, comm::NetId id) {
    Net* n = net(id);
    if (!n || entity == 0) return false;
    if (!system(entity)) assign(entity, comm::kNoSystem, {});
    const std::size_t i = index(entity);
    System& s = systems_[i];
    if (!s.present) return false;
    if (std::find(s.nets.begin(), s.nets.end(), id) != s.nets.end()) return true;
    s.nets.push_back(id);
    s.busyUntil.push_back(-std::numeric_limits<double>::infinity());
    n->members.push_back(i);
    return true;
}

bool Dms::leave(std::uint32_t entity, comm::NetId id) {
    Net* n = net(id);
    System* s = system(entity);
    if (!n || !s) return false;
    const auto it = std::find(s->nets.begin(), s->nets.end(), id);
    if (it == s->nets.end()) return false;
    s->busyUntil.erase(s->busyUntil.begin() + (it - s->nets.begin()));
    s->nets.erase(it);
    const std::size_t i = index(entity);
    n->members.erase(std::remove(n->members.begin(), n->members.end(), i), n->members.end());
    return true;
}

void Dms::removeEntity(std::uint32_t entity) {
    System* s = system(entity);
    if (!s) return;
    while (!s->nets.empty()) leave(entity, s->nets.back());
    s->present = false;
    s->inbox.clear();
}

bool Dms::setPlan(std::uint32_t entity, const comm::DisseminationPlan& plan, double now) {
    System* s = system(entity);
    if (!s) return false;
    s->plan = plan;
    s->nextPosition = s->nextStatus = s->nextTracks = now; // (each from the next step)
    return true;
}

comm::DisseminationPlan Dms::plan(std::uint32_t entity) const noexcept {
    const System* s = system(entity);
    return s ? s->plan : comm::DisseminationPlan{};
}

std::vector<Dms::Due> Dms::due(double now) {
    std::vector<Due> out;
    auto next = [now](double period, double& at) {
        if (period <= 0.0 || at > now) return false;
        at += period;
        if (at <= now) at = now + period; // (a long pause: once, then on period again)
        return true;
    };
    for (System& s : systems_) {
        if (!s.present || s.nets.empty()) continue;
        Due d;
        d.entity = s.entity;
        d.position = next(s.plan.positionPeriodS, s.nextPosition);
        d.status = next(s.plan.statusPeriodS, s.nextStatus);
        d.tracks = next(s.plan.tracksPeriodS, s.nextTracks);
        if (d.position || d.status || d.tracks) out.push_back(d);
    }
    return out;
}

// --- a system's ----------------------------------------------------------------------------------------------------

comm::SystemId Dms::systemOf(std::uint32_t entity) const noexcept {
    const System* s = system(entity);
    return s ? s->id : comm::kNoSystem;
}

std::vector<comm::NetInfo> Dms::netsOf(std::uint32_t entity) const {
    std::vector<comm::NetInfo> out;
    if (const System* s = system(entity))
        for (const comm::NetId id : s->nets) out.push_back(nets_[id - 1].info);
    return out;
}

std::uint64_t Dms::send(std::uint32_t entity, const comm::DmsMessage& message, double now, comm::DmsReason& why, bool native) {
    why = comm::DmsReason::None;
    System* s = system(entity);
    if (!s || s->nets.empty()) {
        why = comm::DmsReason::NoSystem;
        return 0;
    }
    if (!native && message.topic < comm::topic::kFreeForm) {
        why = comm::DmsReason::ReservedTopic;
        return 0;
    }
    bool fits = false;
    for (const comm::NetId id : s->nets) fits = fits || message.bytes.size() <= nets_[id - 1].info.spec.maxPayloadBytes;
    if (!fits) {
        why = comm::DmsReason::TooLarge;
        return 0;
    }
    auto m = std::make_shared<Outgoing>();
    for (const comm::SystemId d : message.destinations) {
        const auto it = bySystem_.find(d);
        bool shared = false;
        if (it != bySystem_.end() && it->second != index(entity))
            for (const comm::NetId id : systems_[it->second].nets)
                shared = shared || std::find(s->nets.begin(), s->nets.end(), id) != s->nets.end();
        if (!shared) {
            why = comm::DmsReason::UnknownSystem;
            return 0;
        }
        if (std::none_of(m->dests.begin(), m->dests.end(), [&](const Outgoing::Dest& x) { return x.system == d; }))
            m->dests.push_back({d, comm::Delivery::Pending, now, false});
    }
    m->sender = index(entity);
    m->id = s->nextMessage++;
    m->message = message;
    m->timeSent = now;
    comm::DmsTransmission t;
    t.message = m->id, t.topic = message.topic, t.timeSent = now, t.published = m->dests.empty();
    for (const auto& d : m->dests) t.destinations.emplace_back(d.system, comm::Delivery::Pending);
    if (s->sent.size() >= kSentKept) s->sent.erase(s->sent.begin());
    s->sent.push_back(std::move(t));
    outgoing_.push_back(std::move(m));
    return s->sent.back().message;
}

bool Dms::subscribe(std::uint32_t entity, comm::Topic topic, bool on) {
    System* s = system(entity);
    if (!s) return false;
    const auto it = std::find(s->topics.begin(), s->topics.end(), topic);
    if (on && it == s->topics.end()) s->topics.push_back(topic);
    if (!on && it != s->topics.end()) s->topics.erase(it);
    return true;
}

bool Dms::subscribeAll(std::uint32_t entity, bool on) {
    System* s = system(entity);
    if (!s) return false;
    s->all = on;
    return true;
}

std::vector<comm::Topic> Dms::subscriptions(std::uint32_t entity) const {
    const System* s = system(entity);
    return s ? s->topics : std::vector<comm::Topic>{};
}

bool Dms::subscribedAll(std::uint32_t entity) const noexcept {
    const System* s = system(entity);
    return s && s->all;
}

const std::deque<comm::DmsReceived>* Dms::inbox(std::uint32_t entity) const noexcept {
    const System* s = system(entity);
    return s ? &s->inbox : nullptr;
}

std::vector<comm::DmsReceived> Dms::take(std::uint32_t entity, std::size_t count) {
    std::vector<comm::DmsReceived> out;
    if (System* s = system(entity)) {
        for (; count > 0 && !s->inbox.empty(); --count) {
            out.push_back(std::move(s->inbox.front()));
            s->inbox.pop_front();
        }
    }
    return out;
}

std::optional<comm::DmsTransmission> Dms::transmission(std::uint32_t entity, std::uint64_t message) const {
    if (const System* s = system(entity))
        for (const auto& t : s->sent)
            if (t.message == message) return t;
    return std::nullopt;
}

std::vector<comm::PeerReport> Dms::peers(std::uint32_t entity) const {
    const System* s = system(entity);
    return s ? s->peers : std::vector<comm::PeerReport>{};
}

std::vector<Dms::PeerProvenance> Dms::peerProvenance(std::uint32_t entity) const {
    const System* s = system(entity);
    std::vector<PeerProvenance> out = s ? s->peerFrom : std::vector<PeerProvenance>{};
    out.resize(s ? s->peers.size() : 0); // (index for index, whatever else)
    return out;
}

std::vector<comm::LinkTrack> Dms::linkTracks(std::uint32_t entity) const {
    const System* s = system(entity);
    return s ? s->linkTracks : std::vector<comm::LinkTrack>{};
}

std::vector<comm::DmsReceived> Dms::takeCore(std::uint32_t entity) {
    std::vector<comm::DmsReceived> out;
    if (System* s = system(entity))
        while (!s->core.empty()) {
            out.push_back(std::move(s->core.front()));
            s->core.pop_front();
        }
    return out;
}

std::vector<std::uint32_t> Dms::entities() const {
    std::vector<std::uint32_t> out;
    for (const System& s : systems_)
        if (s.present && !s.nets.empty()) out.push_back(s.entity);
    return out;
}

std::vector<std::uint32_t> Dms::coreWaiting() const {
    std::vector<std::uint32_t> out;
    for (const System& s : systems_)
        if (!s.core.empty()) out.push_back(s.entity);
    return out;
}

double Dms::lastHeard(std::uint32_t entity, comm::SystemId from) const noexcept {
    if (const System* s = system(entity))
        for (const auto& [system, at] : s->heard)
            if (system == from) return at;
    return std::numeric_limits<double>::quiet_NaN();
}

std::vector<comm::PendingConsent>* Dms::consentsOf(std::uint32_t entity) noexcept {
    System* s = system(entity);
    return s ? &s->consents : nullptr;
}

const std::vector<comm::CommandStatusEntry>* Dms::commandStatuses(std::uint32_t entity) const noexcept {
    const System* s = system(entity);
    return s ? &s->commands : nullptr;
}

const std::vector<comm::PeerReport>* Dms::peersOf(std::uint32_t entity) const noexcept {
    const System* s = system(entity);
    return s ? &s->peers : nullptr;
}

const std::vector<comm::LinkTrack>* Dms::linkTracksOf(std::uint32_t entity) const noexcept {
    const System* s = system(entity);
    return s ? &s->linkTracks : nullptr;
}

const comm::LinkTrack* Dms::linkTrack(std::uint32_t entity, comm::SystemId source, std::uint32_t payload, std::uint64_t track) const noexcept {
    if (const System* s = system(entity))
        for (const auto& t : s->linkTracks)
            if (t.source == source && t.report.payload == payload && t.report.track == track) return &t;
    return nullptr;
}

// --- the referee's -------------------------------------------------------------------------------------------------

std::uint32_t Dms::entityOf(comm::SystemId system) const noexcept {
    const auto it = bySystem_.find(system);
    return it == bySystem_.end() ? 0u : systems_[it->second].entity;
}

std::string Dms::tailNumber(comm::SystemId system) const {
    const auto it = bySystem_.find(system);
    return it == bySystem_.end() ? std::string() : systems_[it->second].tail;
}

std::vector<comm::NetInfo> Dms::nets() const {
    std::vector<comm::NetInfo> out;
    for (const Net& n : nets_) out.push_back(n.info);
    return out;
}

comm::NetStats Dms::stats(comm::NetId id) const noexcept {
    return id >= 1 && id <= nets_.size() ? nets_[id - 1].stats : comm::NetStats{};
}

// --- delivery ------------------------------------------------------------------------------------------------------

Dms::System* Dms::system(std::uint32_t entity) noexcept {
    const auto it = byEntity_.find(entity);
    return it == byEntity_.end() ? nullptr : &systems_[it->second];
}
const Dms::System* Dms::system(std::uint32_t entity) const noexcept {
    const auto it = byEntity_.find(entity);
    return it == byEntity_.end() ? nullptr : &systems_[it->second];
}
std::size_t Dms::index(std::uint32_t entity) const noexcept {
    const auto it = byEntity_.find(entity);
    return it == byEntity_.end() ? kNone : it->second;
}
Dms::Net* Dms::net(comm::NetId id) noexcept { return id >= 1 && id <= nets_.size() ? &nets_[id - 1] : nullptr; }

bool Dms::subscribed(const System& s, comm::Topic topic) const noexcept {
    return s.all || std::find(s.topics.begin(), s.topics.end(), topic) != s.topics.end();
}

comm::DmsTransmission* Dms::status(System& s, std::uint64_t id) noexcept {
    for (auto& t : s.sent)
        if (t.message == id) return &t;
    return nullptr;
}

double Dms::transmit(System& sender, std::size_t i, const Outgoing& m, double at) {
    Net& n = nets_[sender.nets[i] - 1];
    const comm::NetSpec& spec = n.info.spec;
    // its share of the net's rate: the members' time evenly (an estimate: no slot plan)
    const double share = spec.dataRateBps / static_cast<double>(std::max<std::size_t>(1, n.members.size()));
    const double bits = 8.0 * static_cast<double>(m.message.bytes.size() + spec.headerBytes);
    const double start = std::max(at, sender.busyUntil[i]);
    if (start - at > spec.maxQueueS) {
        ++n.stats.queueDropped;
        return -1.0;
    }
    sender.busyUntil[i] = start + bits / std::max(share, 1e-9);
    ++n.stats.transmissions;
    return sender.busyUntil[i];
}

bool Dms::closes(Net& n, const System& from, const System& to, double now, const Locator& where, const sim::GroundProvider* ground, Rng& rng) {
    const comm::NetSpec& spec = n.info.spec;
    ms::rf::Terminal a, b;
    if (!where.locate(from.entity, a.latitudeRad, a.longitudeRad, a.heightM) || !where.locate(to.entity, b.latitudeRad, b.longitudeRad, b.heightM))
        return false;
    double pa[3], pb[3];
    ms::rf::geodeticToEcef(a.latitudeRad, a.longitudeRad, a.heightM, pa);
    ms::rf::geodeticToEcef(b.latitudeRad, b.longitudeRad, b.heightM, pb);
    if (std::hypot(pa[0] - pb[0], pa[1] - pb[1], pa[2] - pb[2]) > spec.rangeM) {
        ++n.stats.outOfRange;
        return false;
    }
    if (spec.lineOfSight) {
        // the radio path's (MS-2): the earth's curvature and the terrain; a pair's, worked out again each second
        const bool forward = from.entity < to.entity;
        const PathKey key{forward ? from.entity : to.entity, forward ? to.entity : from.entity, spec.frequencyHz};
        Sight& sight = sight_[key];
        if (now - sight.timeS >= kSightRefreshS) {
            ms::rf::PathOptions o;
            o.frequencyHz = spec.frequencyHz;
            o.ground = ground;
            const auto p = forward ? ms::rf::computePath(a, b, o) : ms::rf::computePath(b, a, o);
            sight.timeS = now;
            sight.clear = !p.terrainMasked && p.mode == ms::rf::PathMode::LineOfSight;
        }
        if (!sight.clear) {
            ++n.stats.masked;
            return false;
        }
    }
    if (spec.lossProbability > 0.0 && rng.uniform() < spec.lossProbability) {
        ++n.stats.lost;
        return false;
    }
    return true;
}

void Dms::deliver(const InFlight& f, double now) {
    Outgoing& m = *f.message;
    --m.inFlight;
    System& to = systems_[f.receiver];
    Net& n = nets_[f.net - 1];
    if (!to.present) { // (gone while it travelled)
        for (auto& d : m.dests)
            if (d.system == to.id && d.state == comm::Delivery::Pending) d.state = comm::Delivery::Failed, d.inFlight = false;
        if (comm::DmsTransmission* t = status(systems_[m.sender], m.id))
            for (auto& [system, state] : t->destinations)
                if (system == to.id && state == comm::Delivery::Pending) state = comm::Delivery::Failed;
        return;
    }
    if (std::find(m.deliveredTo.begin(), m.deliveredTo.end(), to.id) != m.deliveredTo.end()) {
        ++n.stats.duplicates; // (A-GRA's content filtering: once, however many paths)
        return;
    }
    m.deliveredTo.push_back(to.id);
    ++n.stats.delivered;
    {
        const comm::SystemId from = systems_[m.sender].id;
        auto h = std::find_if(to.heard.begin(), to.heard.end(), [&](const auto& e) { return e.first == from; });
        if (h != to.heard.end()) h->second = now;
        else to.heard.emplace_back(from, now);
    }
    comm::DmsReceived r;
    r.message = m.id, r.source = systems_[m.sender].id, r.topic = m.message.topic, r.net = f.net;
    r.timeSent = m.timeSent, r.timeDelivered = now, r.bytes = m.message.bytes;
    // C2's commands go to its MA core (AB-5), never to the agent's inbox; the rest wait for the agent
    const bool core = r.topic >= comm::topic::kOperator && r.topic <= comm::topic::kToCoreLast;
    std::deque<comm::DmsReceived>& queue = core ? to.core : to.inbox;
    if (!core && r.topic < comm::topic::kFreeForm) picture(to, r);
    if (queue.size() >= comm::kInboxCapacity) { // (full: its oldest waiting goes)
        queue.pop_front();
        ++n.stats.overflowed;
    }
    queue.push_back(std::move(r));
    for (auto& d : m.dests)
        if (d.system == to.id) d.state = comm::Delivery::Delivered, d.inFlight = false;
    if (comm::DmsTransmission* t = status(systems_[m.sender], m.id)) {
        ++t->reached;
        for (auto& [system, state] : t->destinations)
            if (system == to.id) state = comm::Delivery::Delivered;
    }
}

void Dms::picture(System& to, const comm::DmsReceived& r) {
    auto peer = [&]() -> std::size_t { // (its index, in `peers` and in `peerFrom` beside it)
        to.peerFrom.resize(to.peers.size());
        for (std::size_t i = 0; i < to.peers.size(); ++i)
            if (to.peers[i].system == r.source) return i;
        to.peers.emplace_back();
        to.peers.back().system = r.source;
        to.peerFrom.emplace_back();
        return to.peers.size() - 1;
    };
    if (r.topic == comm::topic::kPositionReport) {
        comm::PositionReport p;
        if (!comm::decode(r.bytes, p)) return;
        const std::size_t i = peer();
        comm::PeerReport& e = to.peers[i];
        e.net = r.net, e.receivedS = r.timeDelivered, e.hasPosition = true, e.position = p;
        to.peerFrom[i].positionNet = r.net, to.peerFrom[i].positionReceivedS = r.timeDelivered;
    } else if (r.topic == comm::topic::kSystemStatus) {
        comm::SystemStatusReport h;
        if (!comm::decode(r.bytes, h)) return;
        const std::size_t i = peer();
        comm::PeerReport& e = to.peers[i];
        e.net = r.net, e.receivedS = r.timeDelivered;
        e.tailNumber.assign(h.tailNumber, strnlen(h.tailNumber, sizeof h.tailNumber));
        PeerProvenance& from = to.peerFrom[i];
        from.heartbeat = true, from.heartbeatNet = r.net, from.heartbeatReceivedS = r.timeDelivered, from.heartbeatTimeS = h.timeS;
    } else if (r.topic == comm::topic::kConsentRequest) {
        comm::ConsentRequest c;
        if (!comm::decode(r.bytes, c)) return;
        to.consents.push_back({r.source, r.timeDelivered, false, c});
        if (to.consents.size() > 256) to.consents.erase(to.consents.begin());
    } else if (r.topic == comm::topic::kCommandStatus) {
        comm::CommandStatus c;
        if (!comm::decode(r.bytes, c)) return;
        for (auto& e : to.commands)
            if (e.from == r.source && e.status.commandId == c.commandId && e.status.topic == c.topic) {
                e.receivedS = r.timeDelivered, e.status = c;
                return;
            }
        to.commands.push_back({r.source, r.timeDelivered, c});
        if (to.commands.size() > 256) to.commands.erase(to.commands.begin()); // (its last 256)
    } else if (r.topic == comm::topic::kEntity) {
        comm::EntityReport t;
        if (!comm::decode(r.bytes, t)) return;
        for (auto& l : to.linkTracks)
            if (l.source == r.source && l.report.payload == t.payload && l.report.track == t.track) {
                if (t.timeS >= l.report.timeS) l.net = r.net, l.receivedS = r.timeDelivered, l.report = t; // (an older one late: kept out)
                return;
            }
        to.linkTracks.push_back({r.source, r.net, r.timeDelivered, t});
    }
}

void Dms::step(double now, const Locator& where, const sim::GroundProvider* ground, Rng& rng) {
    for (System& s : systems_) {
        // the picture keeps what is fresh enough
        s.peerFrom.resize(s.peers.size());
        std::size_t kept = 0;
        for (std::size_t i = 0; i < s.peers.size(); ++i) {
            if (now - s.peers[i].receivedS > s.plan.peerTimeoutS) continue;
            if (kept != i) s.peers[kept] = std::move(s.peers[i]), s.peerFrom[kept] = s.peerFrom[i];
            ++kept;
        }
        s.peers.resize(kept), s.peerFrom.resize(kept);
        s.linkTracks.erase(std::remove_if(s.linkTracks.begin(), s.linkTracks.end(),
                                          [&](const comm::LinkTrack& t) { return now - t.receivedS > s.plan.trackTimeoutS; }),
                           s.linkTracks.end());
    }
    // route: each send's attempts due now - one transmission on each of its sender's nets that fits it, to each receiver
    // it may reach there
    for (auto& ptr : outgoing_) {
        Outgoing& m = *ptr;
        System& from = systems_[m.sender];
        if (!from.present) {
            for (auto& d : m.dests) d.state = d.state == comm::Delivery::Pending ? comm::Delivery::Failed : d.state;
            m.routed = true;
            continue;
        }
        const bool published = m.dests.empty();
        if (published && m.routed) continue;
        // the attempt starts when it was sent (the first) or due (a retry): the sender's share from then on
        double base = published ? m.timeSent : std::numeric_limits<double>::infinity();
        for (const auto& d : m.dests)
            if (d.state == comm::Delivery::Pending && !d.inFlight && d.nextAttempt <= now) base = std::min(base, d.nextAttempt);
        if (!(base <= now)) continue;
        bool anyNet = false;
        std::vector<bool> reached(m.dests.size(), false);
        for (std::size_t i = 0; i < from.nets.size(); ++i) {
            Net& n = nets_[from.nets[i] - 1];
            if (m.message.bytes.size() > n.info.spec.maxPayloadBytes) continue;
            // who may get it on this net
            std::vector<std::size_t> receivers;
            for (const std::size_t r : n.members) {
                const System& to = systems_[r];
                if (r == m.sender || !to.present) continue;
                if (published) {
                    if (subscribed(to, m.message.topic)) receivers.push_back(r);
                } else
                    for (const auto& d : m.dests)
                        if (d.system == to.id && d.state == comm::Delivery::Pending && !d.inFlight && d.nextAttempt <= now) receivers.push_back(r);
            }
            if (receivers.empty()) continue;
            const double ends = transmit(from, i, m, base);
            if (ends < 0.0) continue;
            anyNet = true;
            const comm::NetSpec& spec = n.info.spec;
            for (const std::size_t r : receivers) {
                if (!closes(n, from, systems_[r], now, where, ground, rng)) continue;
                const double jitter = spec.jitterS > 0.0 ? rng.uniform(-spec.jitterS, spec.jitterS) : 0.0;
                inflight_.push_back({ptr, r, n.info.id, ends + std::max(0.0, spec.latencyS + jitter)});
                ++m.inFlight;
                for (std::size_t k = 0; k < m.dests.size(); ++k)
                    if (m.dests[k].system == systems_[r].id) reached[k] = true;
            }
        }
        if (published) {
            m.routed = true;
            if (!anyNet) {
                m.queueDropped = true;
                if (comm::DmsTransmission* t = status(from, m.id)) t->queueDropped = true;
            }
            continue;
        }
        // reliable: a destination no net reached this time is tried again, until its timeout
        for (std::size_t k = 0; k < m.dests.size(); ++k) {
            auto& d = m.dests[k];
            if (d.state != comm::Delivery::Pending || d.inFlight || d.nextAttempt > now) continue;
            if (reached[k]) {
                d.inFlight = true;
                continue;
            }
            d.nextAttempt = now + nets_[from.nets.front() - 1].info.spec.retryS;
            double timeout = 0.0;
            for (const comm::NetId id : from.nets) timeout = std::max(timeout, nets_[id - 1].info.spec.reliableTimeoutS);
            if (d.nextAttempt > m.timeSent + timeout) {
                d.state = comm::Delivery::Failed;
                ++nets_[from.nets.front() - 1].stats.failed;
                if (comm::DmsTransmission* t = status(from, m.id))
                    for (auto& [system, state] : t->destinations)
                        if (system == d.system) state = comm::Delivery::Failed;
            }
        }
    }
    // deliver what is due, in the order it arrives (ties: in the order sent)
    std::stable_sort(inflight_.begin(), inflight_.end(), [](const InFlight& a, const InFlight& b) { return a.deliverAt < b.deliverAt; });
    std::size_t done = 0;
    while (done < inflight_.size() && inflight_[done].deliverAt <= now) deliver(inflight_[done++], now);
    inflight_.erase(inflight_.begin(), inflight_.begin() + static_cast<std::ptrdiff_t>(done));
    // a reliable copy lost after it left (none today: loss is decided as it leaves) would be tried again here
    outgoing_.erase(std::remove_if(outgoing_.begin(), outgoing_.end(),
                                   [](const std::shared_ptr<Outgoing>& m) {
                                       if (m->inFlight > 0) return false;
                                       if (m->dests.empty()) return m->routed;
                                       return std::none_of(m->dests.begin(), m->dests.end(),
                                                           [](const Outgoing::Dest& d) { return d.state == comm::Delivery::Pending; });
                                   }),
                    outgoing_.end());
}

} // namespace fsim::session
