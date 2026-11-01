// A vehicle's stores (ADR-30, MS-7a): its stations' inventory, loads, and the
// strikes and jettisons that release them - A-GRA's Store Management and Weapon
// Employment sequences as the host's commands and activities.
#include "simulation/mission_systems/Host.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms {

namespace {

constexpr std::size_t kEndedStrikesKept = 64;
// A weapon bay's doors: open in half a second, held open a second after its last store has gone, shut in eight tenths
// (engineering assumptions: no fighter's door timings are published; the F-22's whole cycle is reported to take about
// a second)
constexpr double kDoorsOpenS = 0.5, kDoorsHoldS = 1.0, kDoorsShutS = 0.8;
constexpr double kJettisonRippleS = 0.1; ///< a standard jettison's stores leave one after another (an emergency's together)
// Mounting hardware retains its existing areas. Every store's own six-axis
// loads come from its authored shared-aircraft VLM product, not these areas.
constexpr double kLugsCdsM2 = 0.007;
constexpr double kPylonCdsM2 = 0.024;
constexpr double kRailCdsM2 = 0.0068;

bool gone(StoreStatus s) noexcept {
    return s == StoreStatus::Releasing || s == StoreStatus::Released || s == StoreStatus::Jettisoned || s == StoreStatus::Lost;
}

bool flyable(const StoreCard& card) noexcept { return !card.model.empty(); }

/// How a store leaves its station: a missile under its own power off a rail where the station has one, anything else
/// pushed clear by its rack.
bool offRail(const Station& station, const StoreCard& card) noexcept {
    return card.storeClass == "aam" && station.carriage.find("rail") != std::string::npos;
}

} // namespace

void Host::rebuildStoreInfo() {
    storeInfo_.clear();
    storeInfo_.reserve(stores_.size());
    for (std::size_t i = 0; i < stores_.size(); ++i) {
        const StoreSlot& s = stores_[i];
        StoreInfo out;
        out.index = static_cast<std::uint16_t>(i);
        out.payload = s.payload;
        out.station = s.station->number;
        out.stationName = s.station->name.c_str();
        out.store = s.card->id.c_str();
        out.mnemonic = s.card->designation.c_str();
        out.storeClass = s.card->storeClass.c_str();
        out.carriage = offRail(*s.station, *s.card) ? "rail" : "ejector";
        out.powerable = s.card->powerable;
        out.jettisonable = s.card->jettisonable;
        out.flyable = flyable(*s.card);
        out.status = s.status;
        out.massKg = massOf(s);
        out.fuelKg = s.fuelKg;
        out.weapon = s.weapon;
        for (const auto& d : doors_)
            if (!s.station->bay.empty() && d.bay == s.station->bay) out.bayDoors = d.position;
        storeInfo_.push_back(out);
    }
}

double Host::storesMassKg() const noexcept {
    double kg = 0.0;
    for (const auto& s : stores_)
        if (!gone(s.status)) kg += massOf(s);
    return kg;
}

Reason Host::load(std::uint16_t payload, int station, const std::string& store, int count, double roomKg) {
    if (payload >= payloads_.size()) return Reason::UnknownPayload;
    const Card& card = *cards_[payload];
    const Station* st = card.station(station);
    if (payloads_[payload].family != Family::StoreStation || !st) return Reason::InvalidParameter;
    std::shared_ptr<const StoreCard> type;
    if (!store.empty()) {
        const auto it = fit_->stores.find(store);
        if (it == fit_->stores.end()) return Reason::InvalidParameter;
        type = it->second;
        if (count < 1 || count > st->capacity || std::find(st->takes.begin(), st->takes.end(), type->storeClass) == st->takes.end() ||
            count * (type->value("mass_kg", 0.0) + type->value("fuel_kg", 0.0)) > st->maxKg)
            return Reason::StationCannotTake;
    }
    for (const auto& s : stores_)
        if (s.payload == payload && s.station == st && s.activity) return Reason::StoreBusy;
    // the aircraft's weight: what the load adds over what it replaces, within what it may still take on (a kilogram's grace)
    if (type && count * (type->value("mass_kg", 0.0) + type->value("fuel_kg", 0.0)) - stationMassKg(payload, station) > roomKg + 1.0)
        return Reason::OverMaxWeight;
    // the station's stores replaced (those gone - released, jettisoned - forgotten with them), the order kept: payload,
    // then station number, then place on the rack
    stores_.erase(std::remove_if(stores_.begin(), stores_.end(), [&](const StoreSlot& s) { return s.payload == payload && s.station == st; }),
                  stores_.end());
    auto at = std::find_if(stores_.begin(), stores_.end(), [&](const StoreSlot& s) {
        return s.payload > payload || (s.payload == payload && s.station->number > station);
    });
    if (type) { // (its carriage on from now: a launcher rail for a missile on a rail station, a missile's well in the
                // fuselage, else a pylon and rack)
        const Mount mount = offRail(*st, *type)                                                      ? Mount::Rail
                            : type->storeClass == "aam" && st->name.find("fuselage") != std::string::npos ? Mount::Well
                                                                                                          : Mount::Pylon;
        auto c = std::find_if(carriage_.begin(), carriage_.end(), [&](const Carriage& k) { return k.payload == payload && k.station == st; });
        if (c == carriage_.end()) carriage_.push_back({payload, st, mount});
        else c->mount = mount;
    }
    for (int i = 0; type && i < count; ++i) {
        StoreSlot s;
        s.payload = payload;
        s.station = st;
        s.card = type;
        s.fuelKg = type->value("fuel_kg", 0.0); // (a tank loaded full)
        at = stores_.insert(at, s) + 1;
    }
    ++storeRevision_;
    rebuildStoreInfo();
    return Reason::None;
}

Reason Host::loadLoadout(const std::string& name, double roomKg) {
    const Loadout* l = fit_ ? fit_->loadout(name) : nullptr;
    if (!l) return Reason::InvalidParameter;
    const auto payloadOf = [&](const Loadout::Item& item) {
        std::uint16_t payload = 0;
        while (payload < payloads_.size() && item.payload != payloads_[payload].name) ++payload;
        return payload;
    };
    if (std::isfinite(roomKg)) {
        // the whole loadout against the aircraft's weight first: what it hangs, less what its stations hold now
        double added = 0.0;
        for (const auto& item : l->items) {
            const std::uint16_t payload = payloadOf(item);
            const auto type = fit_->stores.find(item.store);
            if (payload >= payloads_.size() || type == fit_->stores.end()) continue; // (refused for itself below)
            added += item.count * (type->second->value("mass_kg", 0.0) + type->second->value("fuel_kg", 0.0)) - stationMassKg(payload, item.station);
        }
        if (added > roomKg + 1.0) return Reason::OverMaxWeight;
    }
    for (const auto& item : l->items) {
        const std::uint16_t payload = payloadOf(item);
        if (payload >= payloads_.size()) return Reason::UnknownPayload;
        if (const Reason r = load(payload, item.station, item.store, item.count); r != Reason::None) return r;
    }
    return Reason::None;
}

Host::Strike* Host::strike(ActivityId id) noexcept {
    for (auto& s : strikes_)
        if (s.id == id) return &s;
    return nullptr;
}

Reason Host::previewStrike(const StrikeCommand& command, std::vector<std::uint16_t>& selected) const {
    selected.clear();
    if (command.payload >= payloads_.size()) return Reason::UnknownPayload;
    if (strikeCapability_[command.payload] < 0) return Reason::NotSupported;
    if (!(command.rippleS >= 0.0) || !std::isfinite(command.rippleS) || (command.stores.empty() && command.quantity < 1))
        return Reason::InvalidParameter;
    // the stores: those named, or as many free ones of the type as asked for
    std::vector<std::uint16_t> chosen;
    bool unflyable = false;
    if (!command.stores.empty()) {
        for (const std::uint16_t i : command.stores) {
            if (i >= stores_.size() || stores_[i].payload != command.payload || gone(stores_[i].status) ||
                std::count(chosen.begin(), chosen.end(), i))
                return Reason::InvalidParameter;
            if (stores_[i].activity) return Reason::StoreBusy;
            if (!flyable(*stores_[i].card)) return Reason::StoreNotFlyable;
            chosen.push_back(i);
        }
    } else {
        for (std::uint16_t i = 0; i < stores_.size() && chosen.size() < static_cast<std::size_t>(command.quantity); ++i) {
            const StoreSlot& s = stores_[i];
            if (s.payload != command.payload || gone(s.status) || s.activity) continue;
            if (!command.store.empty() && s.card->id != command.store) continue;
            if (!flyable(*s.card)) { unflyable = true; continue; }
            chosen.push_back(i);
        }
        if (chosen.size() < static_cast<std::size_t>(command.quantity))
            return unflyable && chosen.empty() ? Reason::StoreNotFlyable : Reason::NoStore;
    }
    selected = std::move(chosen);
    return Reason::None;
}

CommandResult Host::submit(const StrikeCommand& command, const CommandOptions& options, double simTime) {
    now_ = simTime;
    CommandResult r;
    r.commandId = options.commandId;
    if (command.payload >= payloads_.size()) return r.reason = Reason::UnknownPayload, r;
    const int capability = strikeCapability_[command.payload];
    if (capability < 0) return r.reason = Reason::NotSupported, r;
    if ((r.reason = admit(static_cast<std::size_t>(capability), options)) != Reason::None) return r;
    std::vector<std::uint16_t> chosen;
    if ((r.reason = previewStrike(command, chosen)) != Reason::None) return r;
    ActivityRecord& a = start(static_cast<std::size_t>(capability), command.payload, options, simTime);
    Strike s;
    s.id = a.id;
    s.payload = command.payload;
    s.stores = chosen;
    s.consent = command.consent;
    s.target = command.target;
    s.ripple = command.rippleS;
    for (const std::uint16_t i : chosen) {
        StoreSlot& slot = stores_[i];
        slot.activity = a.id;
        if (slot.card->powerable && slot.status != StoreStatus::Ready) {
            slot.status = StoreStatus::PoweringUp;
            slot.until = simTime + slot.card->value("power_up_s", 0.0);
        } else {
            slot.status = StoreStatus::Ready;
        }
    }
    strikes_.push_back(std::move(s));
    r.status = control::CommandStatus::Accepted;
    r.activity = a.id;
    advanceStrikes(simTime);
    rebuildStoreInfo();
    return r;
}

CommandResult Host::submit(const JettisonCommand& command, const CommandOptions& options, double simTime) {
    now_ = simTime;
    CommandResult r;
    r.commandId = options.commandId;
    if (command.payload >= payloads_.size()) return r.reason = Reason::UnknownPayload, r;
    const int capability = strikeCapability_[command.payload];
    if (capability < 0) return r.reason = Reason::NotSupported, r;
    // a standard jettison needs the stations powered (standby or operate); an emergency one goes in any state - only
    // the authority (a grant) and a platform restriction hold it
    const Control& ctl = control_[static_cast<std::size_t>(capability)];
    if (ctl.restricted != control::Availability::Available) return r.reason = Reason::Restricted, r;
    if (options.source == control::Source::Policy && mode_ == control::ControlMode::Granted) {
        if (!ctl.granted) return r.reason = Reason::NotGranted, r;
        if (ctl.holder != options.controller) return r.reason = Reason::AuthorityHeld, r;
    }
    const PayloadState ps = state_[command.payload].state;
    if (!command.emergency && ps != PayloadState::Standby && ps != PayloadState::Operate)
        return r.reason = ps == PayloadState::Off ? Reason::PayloadOff : ps == PayloadState::Failed ? Reason::PayloadFailed : unavailableReason(ps), r;
    std::vector<std::uint16_t> chosen;
    if (!command.stores.empty()) {
        for (const std::uint16_t i : command.stores) {
            if (i >= stores_.size() || stores_[i].payload != command.payload || gone(stores_[i].status) || !stores_[i].card->jettisonable ||
                std::count(chosen.begin(), chosen.end(), i))
                return r.reason = Reason::InvalidParameter, r;
            if (!flyable(*stores_[i].card)) return r.reason = Reason::StoreNotFlyable, r;
            chosen.push_back(i);
        }
    } else {
        for (std::uint16_t i = 0; i < stores_.size(); ++i) {
            const StoreSlot& s = stores_[i];
            if (s.payload == command.payload && !gone(s.status) && s.card->jettisonable && flyable(*s.card)) chosen.push_back(i);
        }
        if (chosen.empty()) return r.reason = Reason::NoStore, r;
    }
    // a jettison takes its stores from any strike holding them (that strike ends: superseded)
    for (const std::uint16_t i : chosen)
        if (const ActivityId held = stores_[i].activity) end(held, control::ActivityState::Canceled, Reason::Superseded, simTime);
    ActivityRecord& a = start(static_cast<std::size_t>(capability), command.payload, options, simTime);
    Strike s;
    s.id = a.id;
    s.payload = command.payload;
    s.stores = chosen;
    s.jettison = true;
    s.emergency = command.emergency;
    s.consent = true;
    s.ripple = command.emergency ? 0.0 : kJettisonRippleS;
    for (const std::uint16_t i : chosen) {
        StoreSlot& slot = stores_[i];
        slot.activity = a.id;
        if (!command.emergency && slot.card->powerable && (slot.status == StoreStatus::Ready || slot.status == StoreStatus::PoweringUp)) {
            slot.status = StoreStatus::PoweringDown; // (zeroized and powered down first)
            slot.until = simTime + slot.card->value("power_down_s", 0.0);
        } else {
            slot.status = StoreStatus::Ready;
        }
    }
    strikes_.push_back(std::move(s));
    r.status = control::CommandStatus::Accepted;
    r.activity = a.id;
    advanceStrikes(simTime);
    rebuildStoreInfo();
    return r;
}

Reason Host::strikeConsent(ActivityId activity, bool granted, double simTime) {
    now_ = simTime;
    Strike* s = strike(activity);
    if (!s) return this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity;
    if (s->phase != StrikePhase::AwaitingConsent) return Reason::NotAwaitingConsent;
    if (!granted) {
        end(activity, control::ActivityState::Failed, Reason::ConsentDenied, simTime);
        return Reason::None;
    }
    s->consent = true;
    s->phase = StrikePhase::Releasing;
    s->nextAt = simTime;
    return Reason::None;
}

void Host::awaitingConsent(std::vector<ActivityId>& out) const {
    for (const Strike& s : strikes_)
        if (s.phase == StrikePhase::AwaitingConsent && !s.jettison) out.push_back(s.id);
}

std::optional<StrikeStatus> Host::strikeStatus(ActivityId activity) const {
    for (const auto& s : strikes_)
        if (s.id == activity) {
            StrikeStatus out;
            out.activity = s.id;
            out.phase = s.phase;
            out.jettison = s.jettison;
            out.target = s.target;
            out.stores = s.stores;
            out.weapons = s.weapons;
            return out;
        }
    for (const auto& s : endedStrikes_)
        if (s.activity == activity) return s;
    return std::nullopt;
}

Host::Doors& Host::doors(const std::string& bay) {
    for (auto& d : doors_)
        if (d.bay == bay) return d;
    doors_.push_back(Doors{bay});
    return doors_.back();
}

void Host::advanceDoors(double simTime) {
    const double dt = doorsTime_ >= 0.0 ? std::max(simTime - doorsTime_, 0.0) : 0.0;
    doorsTime_ = simTime;
    bool moved = false;
    for (auto& d : doors_) {
        const double was = d.position;
        if (simTime < d.openUntil) d.position = std::min(1.0, d.position + dt / kDoorsOpenS);
        else d.position = std::max(0.0, d.position - dt / kDoorsShutS);
        moved |= d.position != was;
    }
    if (moved) {
        ++doorsRevision_;
        rebuildStoreInfo();
    }
}

void Host::advanceStrikes(double simTime) {
    advanceDoors(simTime);
    bool changed = false;
    for (auto& s : strikes_) {
        if (s.phase == StrikePhase::PoweringUp) {
            bool warm = true;
            for (const std::uint16_t i : s.stores) {
                StoreSlot& slot = stores_[i];
                if (slot.status == StoreStatus::PoweringUp || slot.status == StoreStatus::PoweringDown) {
                    if (simTime + 1e-9 < slot.until) {
                        warm = false;
                        continue;
                    }
                    slot.status = slot.status == StoreStatus::PoweringUp ? StoreStatus::Ready : StoreStatus::Loaded;
                    changed = true;
                }
            }
            if (!warm) continue;
            s.phase = s.consent ? StrikePhase::Releasing : StrikePhase::AwaitingConsent; // (asks: StrikeConsentRequest)
            s.nextAt = simTime;
            changed = true;
        }
        if (s.phase != StrikePhase::Releasing) continue;
        while (s.next < s.stores.size() && s.nextAt <= simTime + 1e-9) {
            const std::uint16_t i = s.stores[s.next];
            StoreSlot& slot = stores_[i];
            if (gone(slot.status)) { // (lost with its station: nothing to release)
                ++s.next;
                continue;
            }
            if (!slot.station->bay.empty()) {
                // from a weapon bay: its doors open first, and stay open a moment after it has gone
                Doors& d = doors(slot.station->bay);
                d.openUntil = std::max(d.openUntil, simTime + kDoorsOpenS + kDoorsHoldS);
                if (d.position < 1.0) break; // (it waits for them)
            }
            ++s.next;
            Release out;
            out.store = i;
            out.activity = s.id;
            out.target = s.target;
            out.jettisoned = s.jettison;
            out.launch.type = slot.card->model;
            out.launch.inert = s.jettison;
            for (int k = 0; k < 3; ++k) out.launch.offsetBodyM[k] = slot.station->positionM[k];
            if (!offRail(*slot.station, *slot.card)) out.launch.ejectionBodyMs[2] = slot.station->ejectionMs; // (pushed down, clear)
            out.launch.ejectionBodyMs[0] = -slot.station->extractionMs; // (a cargo hold's load: aft, out over the ramp)
            releases_.push_back(std::move(out));
            slot.status = StoreStatus::Releasing;
            s.nextAt += s.ripple;
            changed = true;
        }
        if (s.next == s.stores.size()) s.phase = StrikePhase::Done; // (complete when the world has flown them all)
    }
    for (auto& a : live_) // (each strike's share released)
        if (const Strike* s = strike(a.id))
            a.percent = s->stores.empty() ? 100.0 : 100.0 * static_cast<double>(s->confirmed) / static_cast<double>(s->stores.size());
    if (changed) {
        ++storeRevision_;
        rebuildStoreInfo();
    }
}

std::vector<Host::Release> Host::takeReleases() {
    std::vector<Release> out;
    out.swap(releases_);
    return out;
}

void Host::released(std::uint16_t store, std::uint32_t weapon, double simTime) {
    if (store >= stores_.size()) return;
    StoreSlot& slot = stores_[store];
    Strike* s = strike(slot.activity);
    if (!weapon) { // (it could not be flown: the store stays, its strike fails)
        slot.status = StoreStatus::Loaded;
        if (s) end(s->id, control::ActivityState::Failed, Reason::StoreNotFlyable, simTime);
        rebuildStoreInfo();
        return;
    }
    slot.status = s && s->jettison ? StoreStatus::Jettisoned : StoreStatus::Released;
    slot.weapon = weapon;
    ++storeRevision_;
    if (s) {
        s->weapons.push_back(weapon);
        if (++s->confirmed == s->stores.size()) end(s->id, control::ActivityState::Completed, Reason::GoalReached, simTime);
    }
    rebuildStoreInfo();
}

bool Host::releaseRefused(ActivityId activity, Reason reason, double simTime) {
    return refuseRelease(activity,reason,simTime,false);
}

bool Host::releaseExecutionRefused(ActivityId activity,Reason reason,double simTime) {
    return refuseRelease(activity,reason,simTime,true);
}

bool Host::refuseRelease(ActivityId activity,Reason reason,double simTime,bool includeJettison) {
    Strike* s = strike(activity);
    if (!s || (s->jettison && !includeJettison) || reason == Reason::None) return false;
    now_ = simTime;
    bool changed = false;
    for (const auto index : s->stores) {
        auto& slot = stores_[index];
        if (slot.activity == activity && slot.status == StoreStatus::Releasing) {
            slot.status = StoreStatus::Loaded;
            changed = true;
        }
    }
    releases_.erase(std::remove_if(releases_.begin(), releases_.end(),
        [&](const Release& release) { return release.activity == activity; }), releases_.end());
    if (changed) ++storeRevision_;
    end(activity, control::ActivityState::Failed, reason, simTime);
    rebuildStoreInfo();
    return true;
}

void Host::endStrike(ActivityId id) {
    const auto it = std::find_if(strikes_.begin(), strikes_.end(), [&](const Strike& s) { return s.id == id; });
    if (it == strikes_.end()) return;
    // its stores not yet gone stay on their stations (an abort): powered down, free
    for (const std::uint16_t i : it->stores) {
        StoreSlot& slot = stores_[i];
        if (slot.activity == id) slot.activity = 0;
        if (!gone(slot.status)) slot.status = StoreStatus::Loaded;
    }
    // a release already queued this step still leaves (its store has gone from its station)
    StrikeStatus last;
    last.activity = it->id;
    last.phase = StrikePhase::Done;
    last.jettison = it->jettison;
    last.target = it->target;
    last.stores = it->stores;
    last.weapons = it->weapons;
    endedStrikes_.push_front(std::move(last));
    if (endedStrikes_.size() > kEndedStrikesKept) endedStrikes_.pop_back();
    strikes_.erase(it);
    rebuildStoreInfo();
}

void Host::stationStateChanged(std::uint16_t payload, PayloadState state) {
    if (payload >= strikeCapability_.size() || strikeCapability_[payload] < 0) return;
    // the master arm off ends its strikes; a standard jettison ends with its stations' power (an emergency one goes on)
    std::vector<ActivityId> ending;
    for (const auto& s : strikes_) {
        if (s.payload != payload) continue;
        const bool keeps = s.jettison ? (s.emergency || state == PayloadState::Standby || state == PayloadState::Operate) : state == PayloadState::Operate;
        if (!keeps) ending.push_back(s.id);
    }
    for (const ActivityId id : ending)
        end(id, state == PayloadState::Failed ? control::ActivityState::Failed : control::ActivityState::Canceled, unavailableReason(state), now_);
}

double Host::loseStation(int station, double simTime) {
    // (a station's number is the aircraft's own, whichever card lists it)
    double kg = 0.0;
    if (std::find(lostStations_.begin(), lostStations_.end(), station) == lostStations_.end()) {
        lostStations_.push_back(station); // (its carriage gone with it)
        ++storeRevision_;
    }
    for (auto& s : stores_)
        if (s.station && s.station->number == station && !gone(s.status)) {
            kg += massOf(s);
            s.status = StoreStatus::Lost;
            // a strike or jettison that held it fails: it cannot release what has gone
            if (s.activity && strike(s.activity)) end(s.activity, control::ActivityState::Failed, Reason::PayloadFailed, simTime);
            s.activity = 0;
            ++storeRevision_;
        }
    if (kg > 0.0) rebuildStoreInfo();
    return kg;
}

double Host::stationMassKg(std::uint16_t payload, int station) const noexcept {
    double kg = 0.0;
    for (const auto& s : stores_)
        if (s.payload == payload && s.station->number == station && !gone(s.status)) kg += massOf(s);
    return kg;
}

void Host::carriageDrag(std::vector<DragPoint>& out) const {
    out.clear();
    auto lost = [&](const Station* st) { return std::find(lostStations_.begin(), lostStations_.end(), st->number) != lostStations_.end(); };
    std::vector<const std::string*> racks; // (the multiple racks counted so far: one pylon each, whatever hangs on its places)
    for (const auto& c : carriage_) {
        if (!c.station->bay.empty() || lost(c.station)) continue;
        DragPoint p;
        for (int k = 0; k < 3; ++k) p.positionM[k] = c.station->positionM[k];
        p.areaM2 = c.mount == Mount::Pylon ? kPylonCdsM2 : c.mount == Mount::Rail && c.station->name.find("tip") == std::string::npos ? kRailCdsM2 : 0.0;
        if (c.mount == Mount::Pylon && !c.station->rack.empty()) {
            // a place on a multiple rack: its pylon's drag with the first of its places met, the others' only their stores'
            const bool counted = std::any_of(racks.begin(), racks.end(), [&](const std::string* r) { return *r == c.station->rack; });
            if (counted) p.areaM2 = 0.0;
            else racks.push_back(&c.station->rack);
        }
        for (const auto& s : stores_)
            if (s.payload == c.payload && s.station == c.station && !gone(s.status)) {
                p.areaM2 += c.mount == Mount::Pylon ? kLugsCdsM2 : 0.0;
            }
        if (p.areaM2 > 0.0) out.push_back(p);
    }
}

void Host::carriedAirframesV1(std::vector<CarriedAirframeV1>& out) const {
    out.clear();
    for (const auto& store : stores_) {
        if (gone(store.status) || !store.station->bay.empty() ||
            std::find(lostStations_.begin(),lostStations_.end(),store.station->number)!=lostStations_.end()) continue;
        CarriedAirframeV1 point;point.card=store.card.get();
        for(unsigned k=0;k<3;++k)point.positionM[k]=store.station->positionM[k];
        out.push_back(point);
    }
}


double Host::massOf(const StoreSlot& s) noexcept { return s.card->value("mass_kg", 0.0) + s.fuelKg; }

double Host::externalFuelKg() const noexcept {
    double kg = 0.0;
    for (const auto& s : stores_)
        if (!gone(s.status)) kg += s.fuelKg;
    return kg;
}

double Host::drawExternalFuel(double kg) {
    // from every tank still on alike, as far as each has it (a symmetric load stays symmetric)
    if (!(kg > 0.0)) return 0.0;
    double drawn = 0.0;
    for (int pass = 0; pass < 4 && kg - drawn > 1e-9; ++pass) {
        int holding = 0;
        for (const auto& s : stores_)
            if (!gone(s.status) && s.fuelKg > 0.0) ++holding;
        if (!holding) break;
        const double share = (kg - drawn) / holding;
        for (auto& s : stores_)
            if (!gone(s.status) && s.fuelKg > 0.0) {
                const double take = std::min(share, s.fuelKg);
                s.fuelKg -= take, drawn += take;
            }
    }
    if (drawn > 0.0) {
        ++fuelRevision_;
        if (storeInfo_.size() == stores_.size()) // (what the stores show: each tank's fuel and mass now)
            for (std::size_t i = 0; i < stores_.size(); ++i) {
                storeInfo_[i].fuelKg = stores_[i].fuelKg;
                storeInfo_[i].massKg = massOf(stores_[i]);
            }
        else rebuildStoreInfo();
    }
    return drawn;
}

double Host::storeFuelKg(std::uint16_t store) const noexcept { return store < stores_.size() ? stores_[store].fuelKg : 0.0; }

} // namespace fsim::ms
