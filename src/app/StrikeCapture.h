#pragma once

// Private viewer acceptance policy. It observes native events and never changes simulation state.
#include "core/Json.h"
#include "contracts/world_stream/WorldLayout.h"

#include <array>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace fsim::app {

struct StrikeCapturePlan {
    std::uint32_t carrierId = 0, weaponId = 0, targetId = 0;
    std::uint32_t carrierSlot = 0, weaponSlot = 0, targetSlot = 0;
    std::string store;
    double duration = 0.0, launchTime = 0.0, hitTime = 0.0, nativeStep = 0.0;
};

class StrikeCapture {
public:
    explicit StrikeCapture(const core::Json& evidence) {
        require(evidence.isObject() && evidence.string("schema", "") == "native-engagement-v1" &&
                evidence.string("source", "") == "native-sdk", "capture requires native SDK engagement evidence");
        plan_.carrierId = integer(evidence, "carrier_id", false);
        plan_.targetId = integer(evidence, "target_id", false);
        plan_.carrierSlot = integer(evidence, "carrier_slot");
        plan_.targetSlot = integer(evidence, "target_slot");
        plan_.duration = number(evidence, "duration_s");
        const double dt = number(evidence, "native_dt_s");
        const auto frameSkip = integer(evidence, "native_frame_skip", false);
        require(std::abs(dt - 1.0 / 120.0) <= 1e-12 && frameSkip == 4,
                "capture requires the native 120 Hz integration and 30 Hz world cadence");
        plan_.nativeStep = dt * frameSkip;
        const auto& weapons = evidence.child("weapons");
        require(weapons.isArray() && weapons.asArray().size() == 1, "capture requires exactly one consented native guided weapon");
        const auto& weapon = weapons.asArray().front();
        plan_.weaponId = integer(weapon, "id", false);
        plan_.weaponSlot = integer(weapon, "weapon_slot");
        plan_.store = weapon.string("store", "");
        plan_.launchTime = number(weapon, "launch_time_s");
        plan_.hitTime = number(weapon, "end_time_s");
        require(plan_.carrierId != plan_.targetId && plan_.carrierId != plan_.weaponId && plan_.targetId != plan_.weaponId &&
                plan_.carrierSlot != plan_.targetSlot && plan_.carrierSlot != plan_.weaponSlot && plan_.targetSlot != plan_.weaponSlot,
                "native carrier, weapon and target must have distinct IDs and slots");
        require(plan_.weaponSlot < std::numeric_limits<std::uint32_t>::max() &&
                plan_.targetSlot < std::numeric_limits<std::uint32_t>::max(), "native event slot references would overflow");
        require(!plan_.store.empty() && plan_.store.size() < sizeof(ipc::EngagementEvent::name), "capture requires a native store ID");
        const auto seeker = weapon.string("seeker", "");
        require(seeker == "active_radar" || seeker == "semi_active_radar" || seeker == "infrared" || seeker == "imaging_infrared",
                "capture requires supported native seeker guidance");
        const double activity = number(weapon, "activity_id");
        require(activity > 0.0 && activity <= 9007199254740991.0 && activity == std::floor(activity) &&
                integer(weapon, "launcher_id", false) == plan_.carrierId && weapon.boolean("acquired", false) &&
                weapon.string("terminal_kind", "") == "detonated" && integer(weapon, "holding_id", false) == plan_.targetId,
                "capture requires a launched native strike acquired and detonated at the expected target");
        require(plan_.launchTime >= 0.0 && plan_.hitTime > plan_.launchTime + 1.0 && plan_.duration >= plan_.hitTime + 0.3 + plan_.nativeStep,
                "complete native duration must contain both requested captures and the missile flight");
        const auto targetReference = integer(weapon, "target_reference", false);
        unsigned launches = 0, bursts = 0, damages = 0;
        const auto& engagements = evidence.child("engagements");
        require(engagements.isArray(), "native engagement events are missing");
        for (const auto& item : engagements.asArray()) {
            const auto kind = item.string("kind", "");
            if (kind != "launch" && kind != "detonation" && kind != "damage") continue;
            require(integer(item, "entity", false) == plan_.weaponId && integer(item, "launcher", false) == plan_.carrierId &&
                    integer(item, "target", false) == (kind == "launch" ? targetReference : plan_.targetId) &&
                    sameTime(number(item, "time_s"), kind == "launch" ? plan_.launchTime : plan_.hitTime),
                    "native strike event identity or time differs from the weapon record");
            if (kind == "launch") ++launches;
            else if (kind == "detonation") ++bursts;
            else {
                require(item.boolean("destroyed", false), "native damage did not confirm crew incapacitation");
                ++damages;
            }
        }
        require(launches == 1 && bursts == 1 && damages == 1,
                "capture requires one actual native launch, detonation and crew-incapacitating damage event");
    }
    const StrikeCapturePlan& plan() const noexcept { return plan_; }
    void observe(const ipc::EngagementEvent& event) {
        using K = ipc::EngagementKind;
        const auto kind = static_cast<K>(event.kind);
        if (kind == K::None) return;
        require(std::isfinite(event.simTime) && event.simTime + kTimeEpsilon >= plan_.hitTime,
                "destructive effect occurred before the actual native hit");
        if (kind == K::Detonation) {
            require(!burstObserved_ && event.slot == plan_.weaponSlot && event.target == plan_.targetSlot + 1 &&
                    sameTime(event.simTime, plan_.hitTime) && matchesStore(event),
                    "live detonation differs from the expected native weapon, target or time");
            burstObserved_ = true;
        } else if (kind == K::Hit) {
            require(burstObserved_ && !hitObserved_ && event.slot == plan_.targetSlot && event.target == plan_.weaponSlot + 1 &&
                    event.index > 0 && sameTime(event.simTime, plan_.hitTime) && matchesStore(event),
                    "live hit has no matching preceding native detonation and actual fragment damage");
            hitObserved_ = true;
        } else if (kind == K::CrewLost) {
            require(!crewLossObserved_ && event.slot == plan_.targetSlot && sameTime(event.simTime, plan_.hitTime),
                    "live crew loss differs from the expected native target or hit time");
            crewLossObserved_ = true;
        }
        // Native section separation may be published before its same-tick burst and Hit.
        // These effects never authorize a camera transition or a target capture on their own.
    }
    std::uint32_t selectedSlot(double simTime) const {
        requireTime(simTime);
        if (hitObserved_ && simTime + kTimeEpsilon >= plan_.hitTime) return plan_.targetSlot;
        return simTime + kTimeEpsilon >= plan_.launchTime ? plan_.weaponSlot : plan_.carrierSlot;
    }
    std::optional<unsigned> dueCapture(double simTime) const {
        requireTime(simTime);
        for (unsigned index = 0; index < captured_.size(); ++index) {
            if (captured_[index] || (index == 1 && !hitObserved_)) continue;
            const double time = captureTime(index);
            if (simTime + kTimeEpsilon < time) return std::nullopt;
            require(simTime <= time + plan_.nativeStep + kTimeEpsilon, "requested strike capture was missed by more than one native step");
            return index;
        }
        return std::nullopt;
    }
    void acknowledge(unsigned index, double simTime) {
        requireTime(simTime);
        require(index < captured_.size() && !captured_[index] && (index == 0 || (captured_[0] && hitObserved_)),
                "strike capture was duplicated, out of order or lacked an actual hit");
        const double time = captureTime(index);
        require(simTime + kTimeEpsilon >= time && simTime <= time + plan_.nativeStep + kTimeEpsilon,
                "strike capture timestamp is outside its requested native step");
        captured_[index] = true;
    }
    bool completed(double simTime) const {
        requireTime(simTime);
        return hitObserved_ && crewLossObserved_ && captured_[0] && captured_[1] && simTime + kTimeEpsilon >= plan_.duration;
    }

private:
    static constexpr double kTimeEpsilon = 1e-7;
    static void require(bool value, const char* message) {
        if (!value) throw std::runtime_error(message);
    }
    static double number(const core::Json& object, const char* key) {
        const auto* value = object.find(key);
        require(value && value->isNumber() && std::isfinite(value->asNumber()), "native evidence is missing a finite number");
        return value->asNumber();
    }
    static std::uint32_t integer(const core::Json& object, const char* key, bool allowZero = true) {
        const double value = number(object, key);
        require(value >= (allowZero ? 0.0 : 1.0) && value <= std::numeric_limits<std::uint32_t>::max() && value == std::floor(value),
                "native evidence ID or slot is outside its integer range");
        return static_cast<std::uint32_t>(value);
    }
    static bool sameTime(double left, double right) noexcept { return std::abs(left - right) <= kTimeEpsilon; }
    static void requireTime(double simTime) { require(std::isfinite(simTime) && simTime >= 0.0, "invalid native snapshot time"); }
    bool matchesStore(const ipc::EngagementEvent& event) const noexcept {
        return std::strncmp(event.name, plan_.store.c_str(), sizeof(event.name)) == 0;
    }
    double captureTime(unsigned index) const noexcept { return index == 0 ? plan_.launchTime + 1.0 : plan_.hitTime + 0.3; }
    StrikeCapturePlan plan_;
    std::array<bool, 2> captured_{};
    bool burstObserved_ = false, hitObserved_ = false, crewLossObserved_ = false;
};

} // namespace fsim::app
