#include "ms/Host.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms {

namespace {

constexpr std::size_t kEndedKept = 64;
// A card without them (its family's model not built yet) still warms up and tests: these, until its card says
constexpr double kDefaultWarmUpS = 0.0, kDefaultBitS = 0.0;

} // namespace

namespace {

/// A payload's radar stream from the host's seed.
std::uint64_t radarSeed(std::uint64_t seed, std::uint16_t payload) noexcept {
    std::uint64_t z = seed + 0x9E3779B97F4A7C15ull * (payload + 1ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

Reason unavailableFor(PayloadState state) noexcept {
    switch (state) {
    case PayloadState::Off: return Reason::PayloadOff;
    case PayloadState::Starting: return Reason::PayloadStarting;
    case PayloadState::Standby: return Reason::PayloadStandby;
    case PayloadState::Bit: return Reason::BitInProgress;
    case PayloadState::Failed: return Reason::PayloadFailed;
    default: return Reason::None;
    }
}

constexpr double kPi = 3.14159265358979323846;

} // namespace

void Host::bind(std::uint32_t vehicle, std::shared_ptr<const Fit> fit, std::uint64_t seed) {
    vehicle_ = vehicle;
    fit_ = std::move(fit);
    payloads_.clear(), state_.clear(), cards_.clear(), capabilities_.clear(), ids_.clear(), control_.clear(), statusCapability_.clear();
    radars_.clear(), amtiCapability_.clear();
    live_.clear(), ended_.clear();
    strikeCapability_.clear(), storeCapability_.clear(), stores_.clear(), storeInfo_.clear(), strikes_.clear(), endedStrikes_.clear(),
        releases_.clear(), carriage_.clear(), lostStations_.clear();
    ++storeRevision_;
    if (!fit_) return;
    for (const auto& entry : fit_->payloads) {
        const auto it = fit_->cards.find(entry.card);
        if (it == fit_->cards.end()) continue;
        const Card& card = *it->second;
        PayloadInfo p;
        p.index = static_cast<std::uint16_t>(payloads_.size());
        p.name = entry.name.c_str();
        p.card = card.id.c_str();
        p.designation = card.designation.c_str();
        p.summary = card.summary.c_str();
        p.family = card.family;
        p.count = entry.count;
        const auto source = fit_->sources.find(entry.source);
        p.source = source == fit_->sources.end() ? "" : source->second.title.c_str();
        payloads_.push_back(p);
        state_.emplace_back();
        cards_.push_back(&card);
        const radar::Spec spec = radar::specOf(card);
        radars_.push_back(spec.ok ? std::make_unique<radar::Radar>(spec, radarSeed(seed, p.index)) : nullptr);
        amtiCapability_.push_back(-1);
        strikeCapability_.push_back(-1), storeCapability_.push_back(-1);
    }
    // the instance ids first, so their strings do not move as the list grows
    for (const auto& p : payloads_)
        for (const CapabilityType type : capabilitiesOf(p.family)) ids_.push_back(std::string(capabilityTypeId(type)) + ":" + p.name);
    std::size_t k = 0;
    for (const auto& p : payloads_) {
        for (const CapabilityType type : capabilitiesOf(p.family)) {
            CapabilityInfo c;
            c.index = static_cast<std::uint16_t>(capabilities_.size());
            c.payload = p.index;
            c.type = type;
            c.id = ids_[k++].c_str();
            c.support = capabilitySupport(type, c.stage);
            if (type == CapabilityType::Amti && radars_[p.index]) { // (a radar its card describes: modelled)
                c.support = control::Support::Supported, c.stage = 0;
                amtiCapability_[p.index] = static_cast<int>(capabilities_.size());
            }
            // a store station its card describes (MS-7a): its inventory and releases modelled; launch zones, and the
            // weapons' seekers and guidance, to come
            if (p.family == Family::StoreStation && !cards_[p.index]->stations.empty() &&
                (type == CapabilityType::Strike || type == CapabilityType::StoreManagement)) {
                c.support = control::Support::Partial, c.stage = 7;
                c.missing = type == CapabilityType::Strike ? "the weapons' seekers, guidance and fuzes (MS-7d)" : "launch zones (MS-8)";
                (type == CapabilityType::Strike ? strikeCapability_ : storeCapability_)[p.index] = static_cast<int>(capabilities_.size());
            }
            if (type == CapabilityType::Status) statusCapability_.push_back(capabilities_.size());
            capabilities_.push_back(c);
        }
    }
    control_.resize(capabilities_.size());
    rebuildStoreInfo(); // (its stations empty: a scenario loads them - loadLoadout, load - before or during flight)
}

Reason Host::unavailableReason(PayloadState state) noexcept { return unavailableFor(state); }

void Host::reseed(std::uint64_t seed) {
    for (std::uint16_t i = 0; i < radars_.size(); ++i)
        if (radars_[i]) radars_[i]->reseed(radarSeed(seed, i));
}

const Card* Host::card(std::uint16_t payload) const noexcept { return payload < cards_.size() ? cards_[payload] : nullptr; }

int Host::find(std::string_view id) const noexcept {
    for (std::size_t i = 0; i < capabilities_.size(); ++i)
        if (id == capabilities_[i].id) return static_cast<int>(i);
    return -1;
}

Reason Host::refusal(std::string_view id) noexcept {
    const std::string_view type = id.substr(0, id.find(':'));
    for (std::size_t i = 0; i < static_cast<std::size_t>(CapabilityType::Count); ++i)
        if (type == capabilityTypeId(static_cast<CapabilityType>(i))) return Reason::NotSupported;
    return Reason::UnknownCapability;
}

CapabilityStatus Host::status(std::size_t capability) const noexcept {
    CapabilityStatus out;
    if (capability >= capabilities_.size()) {
        out.availability = control::Availability::Unavailable;
        out.reason = Reason::UnknownCapability;
        return out;
    }
    const CapabilityInfo& c = capabilities_[capability];
    if (c.support == control::Support::NotImplemented || c.support == control::Support::NotSupported) {
        out.availability = control::Availability::Unavailable;
        out.reason = c.support == control::Support::NotImplemented ? Reason::NotImplemented : Reason::NotSupported;
        return out;
    }
    if (control_[capability].restricted != control::Availability::Available) {
        out.availability = control_[capability].restricted;
        out.reason = Reason::Restricted;
        return out;
    }
    const Payload& p = state_[c.payload];
    if (p.state == PayloadState::Failed) {
        out.availability = control::Availability::Faulted;
        out.reason = Reason::PayloadFailed;
        return out;
    }
    if (c.type == CapabilityType::Status) return out; // (it commands the payload's state: always, unless failed)
    switch (p.state) {
    case PayloadState::Operate: break;
    case PayloadState::Off: out.availability = control::Availability::TemporarilyUnavailable, out.reason = Reason::PayloadOff; break;
    case PayloadState::Starting:
        out.availability = control::Availability::TemporarilyUnavailable, out.reason = Reason::PayloadStarting, out.nextAvailableS = p.until;
        break;
    case PayloadState::Standby: out.availability = control::Availability::TemporarilyUnavailable, out.reason = Reason::PayloadStandby; break;
    case PayloadState::Bit:
        out.availability = control::Availability::TemporarilyUnavailable, out.reason = Reason::BitInProgress, out.nextAvailableS = p.until;
        break;
    default: break;
    }
    return out;
}

Reason Host::admit(std::size_t capability, const CommandOptions& options) const noexcept {
    const CapabilityStatus s = status(capability);
    if (s.availability != control::Availability::Available) return s.reason;
    if (options.source == control::Source::Policy && mode_ == control::ControlMode::Granted) {
        const Control& c = control_[capability];
        if (!c.granted) return Reason::NotGranted;
        if (c.holder != options.controller) return Reason::AuthorityHeld;
    }
    return Reason::None;
}

ActivityRecord* Host::live(ActivityId id) noexcept {
    for (auto& a : live_)
        if (a.id == id) return &a;
    return nullptr;
}

ActivityRecord& Host::start(std::size_t capability, std::uint16_t payload, const CommandOptions& options, double simTime) {
    ActivityRecord a;
    a.id = (static_cast<ActivityId>(vehicle_) << 32) | (kActivityBit | ++serial_);
    a.capability = static_cast<std::uint16_t>(capability);
    a.payload = payload;
    a.source = options.source;
    a.controller = options.controller;
    a.commandId = options.commandId;
    a.state = control::ActivityState::Active;
    a.startTime = simTime;
    a.percent = 0.0;
    live_.push_back(a);
    return live_.back();
}

void Host::end(ActivityId id, control::ActivityState state, Reason reason, double simTime) {
    for (auto it = live_.begin(); it != live_.end(); ++it)
        if (it->id == id) {
            ActivityRecord a = *it;
            a.state = state;
            a.reason = reason;
            a.endTime = simTime;
            if (state == control::ActivityState::Completed) a.percent = 100.0;
            live_.erase(it);
            ended_.push_front(a);
            if (ended_.size() > kEndedKept) ended_.pop_back();
            for (std::size_t i = 0; i < state_.size(); ++i) {
                Payload& p = state_[i];
                if (p.activity == id) p.activity = 0;
                if (p.amti == id) { // (its radar stops searching)
                    p.amti = 0;
                    if (radars_[i]) radars_[i]->stop();
                }
            }
            endStrike(id);
            return;
        }
}

void Host::setState(std::uint16_t payload, PayloadState state) {
    state_[payload].state = state;
    payloads_[payload].state = state;
    // a radar switched off, or failed, loses its track files
    if ((state == PayloadState::Off || state == PayloadState::Failed) && payload < radars_.size() && radars_[payload]) radars_[payload]->clearTracks();
    // a search ends as its payload stops operating (a failure fails it)
    if (state != PayloadState::Operate && state_[payload].amti)
        end(state_[payload].amti, state == PayloadState::Failed ? control::ActivityState::Failed : control::ActivityState::Canceled,
            unavailableFor(state), now_);
    stationStateChanged(payload, state);
}

CommandResult Host::submit(const StateCommand& command, const CommandOptions& options, double simTime) {
    now_ = simTime;
    CommandResult r;
    r.commandId = options.commandId;
    if (command.payload >= payloads_.size()) return r.reason = Reason::UnknownPayload, r;
    if (command.state != PayloadState::Off && command.state != PayloadState::Standby && command.state != PayloadState::Operate)
        return r.reason = Reason::InvalidParameter, r;
    const std::size_t capability = statusCapability_[command.payload];
    if ((r.reason = admit(capability, options)) != Reason::None) return r;
    Payload& p = state_[command.payload];
    const PayloadState now = p.state == PayloadState::Starting ? p.target : p.state == PayloadState::Bit ? p.beforeBit : p.state;
    if (now == command.state && p.state != PayloadState::Bit) return r.reason = Reason::AlreadyInState, r;
    if (p.activity) end(p.activity, control::ActivityState::Canceled, Reason::Superseded, simTime); // (a newer state command, or a BIT, gives way)
    ActivityRecord& a = start(capability, command.payload, options, simTime);
    r.status = control::CommandStatus::Accepted;
    r.activity = a.id;
    if (command.state == PayloadState::Off) {
        setState(command.payload, PayloadState::Off);
        end(a.id, control::ActivityState::Completed, Reason::GoalReached, simTime);
        return r;
    }
    if (p.state == PayloadState::Off) { // powered up: it warms up first
        const double warmUp = cards_[command.payload]->value("warm_up_s", kDefaultWarmUpS);
        p.since = simTime, p.until = simTime + warmUp, p.target = command.state;
        setState(command.payload, PayloadState::Starting);
        p.activity = a.id;
        if (warmUp <= 0.0) step(simTime);
        return r;
    }
    if (p.state == PayloadState::Starting) { // already warming up: only where it goes changes
        p.target = command.state;
        p.activity = a.id;
        return r;
    }
    setState(command.payload, command.state); // between standby and operate: at once (a BIT under way is abandoned)
    end(a.id, control::ActivityState::Completed, Reason::GoalReached, simTime);
    return r;
}

CommandResult Host::submit(const BitCommand& command, const CommandOptions& options, double simTime) {
    now_ = simTime;
    CommandResult r;
    r.commandId = options.commandId;
    if (command.payload >= payloads_.size()) return r.reason = Reason::UnknownPayload, r;
    const std::size_t capability = statusCapability_[command.payload];
    if ((r.reason = admit(capability, options)) != Reason::None) return r;
    Payload& p = state_[command.payload];
    if (p.state == PayloadState::Off) return r.reason = Reason::PayloadOff, r;
    if (p.state == PayloadState::Starting) return r.reason = Reason::PayloadStarting, r;
    if (p.state == PayloadState::Bit) return r.reason = Reason::BitInProgress, r;
    if (p.activity) end(p.activity, control::ActivityState::Canceled, Reason::Superseded, simTime);
    ActivityRecord& a = start(capability, command.payload, options, simTime);
    r.status = control::CommandStatus::Accepted;
    r.activity = a.id;
    p.beforeBit = p.state;
    p.since = simTime, p.until = simTime + cards_[command.payload]->value("bit_s", kDefaultBitS);
    p.activity = a.id;
    setState(command.payload, PayloadState::Bit);
    if (p.until <= simTime) step(simTime);
    return r;
}

CommandResult Host::submit(const AmtiCommand& command, const CommandOptions& options, double simTime) {
    now_ = simTime;
    CommandResult r;
    r.commandId = options.commandId;
    if (command.payload >= payloads_.size()) return r.reason = Reason::UnknownPayload, r;
    const int capability = amtiCapability_[command.payload];
    if (capability < 0) return r.reason = Reason::NotSupported, r;
    if ((r.reason = admit(static_cast<std::size_t>(capability), options)) != Reason::None) return r;
    radar::Radar& radar = *radars_[command.payload];
    const bool rotodome = radar.spec().rotationRpm > 0.0;
    const bool finite = std::isfinite(command.azimuthCentreRad) && std::isfinite(command.azimuthHalfWidthRad) &&
                        std::isfinite(command.elevationCentreRad) && std::isfinite(command.rangeScaleM);
    if (!finite || !(command.azimuthHalfWidthRad > 0.0) || command.bars < 1 || command.bars > 8 || !(command.rangeScaleM > 0.0) ||
        static_cast<unsigned>(command.mode) > static_cast<unsigned>(AmtiMode::Surveillance) ||
        rotodome != (command.mode == AmtiMode::Surveillance) || (command.signalLevel && command.mode == AmtiMode::VelocitySearch))
        return r.reason = Reason::InvalidParameter, r;
    Payload& p = state_[command.payload];
    if (p.amti) end(p.amti, control::ActivityState::Canceled, Reason::Superseded, simTime);
    ActivityRecord& a = start(static_cast<std::size_t>(capability), command.payload, options, simTime);
    a.percent = std::numeric_limits<double>::quiet_NaN(); // (a search has no end to measure against)
    p.amti = a.id;
    radar::Search s;
    s.mode = command.mode;
    s.azimuthCentreRad = command.azimuthCentreRad;
    s.azimuthHalfWidthRad = command.azimuthHalfWidthRad;
    s.elevationCentreRad = command.elevationCentreRad;
    s.bars = command.bars;
    s.rangeScaleM = command.rangeScaleM;
    s.signalLevel = command.signalLevel;
    radar.search(s, simTime);
    r.status = control::CommandStatus::Accepted;
    r.activity = a.id;
    return r;
}

CommandResult Host::cancel(control::Caller caller, ActivityId id, double simTime) {
    now_ = simTime;
    CommandResult r;
    r.activity = id;
    ActivityRecord* a = live(id);
    if (!a) {
        r.reason = activity(id) ? Reason::ActivityEnded : Reason::UnknownActivity;
        return r;
    }
    r.commandId = a->commandId;
    if (static_cast<unsigned>(caller.source) < static_cast<unsigned>(a->source)) return r.reason = Reason::AuthorityHeld, r;
    if (caller.source == control::Source::Policy && mode_ == control::ControlMode::Granted && a->controller != caller.controller)
        return r.reason = Reason::AuthorityHeld, r;
    Payload& p = state_[a->payload];
    if (p.state == PayloadState::Starting) setState(a->payload, PayloadState::Off);     // a warm-up given up: off again
    else if (p.state == PayloadState::Bit) setState(a->payload, p.beforeBit);           // a BIT given up: as it was
    end(id, control::ActivityState::Canceled, Reason::Requested, simTime);
    r.status = control::CommandStatus::Canceled;
    return r;
}

Reason Host::setFailed(std::uint16_t payload, bool failed, double simTime) {
    now_ = simTime;
    if (payload >= payloads_.size()) return Reason::UnknownPayload;
    Payload& p = state_[payload];
    if (failed) {
        if (p.activity) end(p.activity, control::ActivityState::Failed, Reason::PayloadFailed, simTime);
        setState(payload, PayloadState::Failed);
    } else if (p.state == PayloadState::Failed) {
        setState(payload, PayloadState::Off);
    }
    return Reason::None;
}

const ActivityRecord* Host::activity(ActivityId id) const noexcept {
    for (const auto& a : live_)
        if (a.id == id) return &a;
    for (const auto& a : ended_)
        if (a.id == id) return &a;
    return nullptr;
}

std::vector<ActivityRecord> Host::activities() const {
    std::vector<ActivityRecord> out(live_.rbegin(), live_.rend());
    out.insert(out.end(), ended_.begin(), ended_.end());
    return out;
}

Reason Host::requestControl(std::size_t capability, control::ControllerId controller) {
    if (capability >= capabilities_.size()) return Reason::UnknownCapability;
    Control& c = control_[capability];
    if (!c.allowed) return Reason::NotAllowed;
    const CapabilityStatus s = status(capability);
    if (s.reason == Reason::NotImplemented || s.reason == Reason::NotSupported || s.reason == Reason::Restricted) return s.reason;
    if (c.granted && c.holder != controller) return Reason::AuthorityHeld;
    c.granted = true, c.holder = controller;
    return Reason::None;
}

Reason Host::releaseControl(std::size_t capability, control::ControllerId controller, double simTime) {
    if (capability >= capabilities_.size()) return Reason::UnknownCapability;
    Control& c = control_[capability];
    if (!c.granted || c.holder != controller) return Reason::NotGranted;
    c.granted = false;
    for (std::size_t i = live_.size(); i-- > 0;)
        if (live_[i].capability == capability && live_[i].source == control::Source::Policy && live_[i].controller == controller)
            end(live_[i].id, control::ActivityState::Canceled, Reason::Released, simTime);
    return Reason::None;
}

Reason Host::revokeControl(std::size_t capability, double simTime) {
    if (capability >= capabilities_.size()) return Reason::UnknownCapability;
    Control& c = control_[capability];
    c.granted = false;
    for (std::size_t i = live_.size(); i-- > 0;)
        if (live_[i].capability == capability && live_[i].source == control::Source::Policy)
            end(live_[i].id, control::ActivityState::Canceled, Reason::Revoked, simTime);
    return Reason::None;
}

Reason Host::setAllowed(std::size_t capability, bool allowed, double simTime) {
    if (capability >= capabilities_.size()) return Reason::UnknownCapability;
    control_[capability].allowed = allowed;
    if (!allowed && control_[capability].granted) revokeControl(capability, simTime);
    return Reason::None;
}

control::ControlStatus Host::controlStatus(std::size_t capability) const noexcept {
    control::ControlStatus out;
    if (capability >= capabilities_.size()) return out;
    out.allowed = control_[capability].allowed;
    out.granted = control_[capability].granted;
    out.holder = control_[capability].holder;
    return out;
}

Reason Host::setAvailability(std::size_t capability, control::Availability availability) {
    if (capability >= capabilities_.size()) return Reason::UnknownCapability;
    control_[capability].restricted = availability;
    return Reason::None;
}

void Host::step(double simTime) {
    now_ = simTime;
    advanceStrikes(simTime);
    for (std::uint16_t i = 0; i < state_.size(); ++i) {
        Payload& p = state_[i];
        if (p.state != PayloadState::Starting && p.state != PayloadState::Bit) continue;
        ActivityRecord* a = p.activity ? live(p.activity) : nullptr;
        const double span = p.until - p.since;
        if (a) a->percent = span > 0.0 ? std::clamp(100.0 * (simTime - p.since) / span, 0.0, 100.0) : 100.0;
        if (simTime + 1e-9 < p.until) continue;
        if (p.state == PayloadState::Starting) {
            setState(i, p.target);
            if (a) end(a->id, control::ActivityState::Completed, Reason::GoalReached, simTime);
        } else {
            setState(i, p.beforeBit); // (a BIT finds a fault only where one is injected: setFailed)
            if (a) end(a->id, control::ActivityState::Completed, Reason::GoalReached, simTime);
        }
    }
}

const radar::Radar* Host::radar(std::uint16_t payload) const noexcept {
    return payload < radars_.size() ? radars_[payload].get() : nullptr;
}

RadarStatus Host::radarStatus(std::uint16_t payload) const noexcept {
    RadarStatus out;
    const radar::Radar* r = radar(payload);
    if (!r) return out;
    const radar::Spec& spec = r->spec();
    const radar::Search& s = r->searching();
    out.searching = r->active();
    out.activity = state_[payload].amti;
    out.mode = s.mode;
    out.beamAzimuthRad = r->beamAzimuthRad();
    out.beamElevationRad = r->beamElevationRad();
    out.framePeriodS = r->framePeriodS();
    out.rotodome = spec.rotationRpm > 0.0;
    out.electronic = spec.aperture.electronic;
    out.azimuthLimitRad = out.rotodome ? kPi : spec.azimuthLimitDeg * kPi / 180.0;
    out.elevationLimitRad = spec.elevationLimitDeg * kPi / 180.0;
    out.azimuthCentreRad = out.rotodome ? 0.0 : std::clamp(s.azimuthCentreRad, -out.azimuthLimitRad, out.azimuthLimitRad);
    out.azimuthHalfWidthRad = out.rotodome ? kPi : s.azimuthHalfWidthRad;
    out.elevationCentreRad = s.elevationCentreRad;
    out.bars = s.bars;
    out.rangeScaleM = s.rangeScaleM;
    out.instrumentedRangeM = spec.instrumentedRangeM;
    out.frequencyHz = spec.frequencyHz;
    out.beamwidthAzimuthRad = r->pattern().beamwidthRad(false);
    out.beamwidthElevationRad = r->pattern().beamwidthRad(true);
    out.dwells = r->dwells();
    out.detections = r->detectionCount();
    out.falseAlarms = r->falseAlarmCount();
    out.tracks = static_cast<std::uint32_t>(r->tracks().size());
    out.tentativeTracks = static_cast<std::uint32_t>(r->tentativeTracks());
    out.signalLevel = s.signalLevel;
    return out;
}

std::vector<AmtiTrack> Host::amtiTracks(std::uint16_t payload, double now, const sim::VehicleState& own) const {
    std::vector<AmtiTrack> out;
    const radar::Radar* r = radar(payload);
    if (!r) return out;
    out.reserve(r->tracks().size());
    for (const auto& t : r->tracks()) {
        out.push_back(r->report(t, now, own));
        out.back().payload = payload;
    }
    return out;
}

bool Host::sensing() const noexcept {
    for (const auto& r : radars_)
        if (r && r->active()) return true;
    return false;
}

void Host::sense(double simTime, const sim::VehicleState& own, const radar::Scene& scene) {
    for (auto& r : radars_)
        if (r && r->active()) r->step(simTime, own, scene);
}

} // namespace fsim::ms
