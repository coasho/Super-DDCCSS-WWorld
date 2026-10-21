#pragma once

#include "ContactFuze.h"
#include "FuzeDelay.h"
#include "MotionHistory.h"
#include "simulation/mission_systems/components/Profile.h"

#include <memory>
#include <string_view>

namespace fsim::ms::weapon {

/// One selected provider and initiation lifecycle per actual native release.
/// Captured paths are inputs; this interface never predicts motion or grants
/// target/pose authority. The selected specification is an owned snapshot.
class FuzeAdapter {
public:
    virtual ~FuzeAdapter() = default;
    virtual std::string_view implementationId() const noexcept = 0;
    virtual const Spec& spec() const noexcept = 0;
    virtual FuzeResult evaluateProximity(const std::vector<MotionSample>& own,
        const std::vector<MotionSample>& target, double ageBeforeS,
        double durationS, double eligibleEndS, bool exclusiveEnd,
        std::uint32_t targetId) = 0;
    virtual ContactFuzeResult evaluateContact(const std::vector<ContactSample>&,
        const damage::AttachedGeometryView&, const ContactWindow&) const = 0;
    virtual bool commitContact(const ContactFuzeResult&) noexcept = 0;
    virtual bool scheduleDelay(double triggerAgeS, std::uint32_t targetId) noexcept = 0;
    virtual FuzeDelayResultV1 dueDelay(double ageBeforeS, double durationS,
        double eligibleEndS, bool exclusiveEnd) noexcept = 0;
    virtual void cancelDelay() noexcept = 0;
    /// Transient history only; never rearms contact or delayed initiation.
    virtual void resetTurnHistory() noexcept = 0;
    /// New actual release only; not Vehicle.reset or seeker reacquisition.
    virtual void resetRelease() noexcept = 0;
    virtual ContactFuzeState& contactState() noexcept = 0;
    virtual const ContactFuzeState& contactState() const noexcept = 0;
    virtual FuzeTurnStateV1& turnState() noexcept = 0;
    virtual const FuzeTurnStateV1& turnState() const noexcept = 0;
    virtual FuzeDelayStateV1& delayState() noexcept = 0;
    virtual const FuzeDelayStateV1& delayState() const noexcept = 0;
};

/// Copies the native specification, configuring only its fuze fields from the
/// explicit descriptor. Refusal never modifies the caller's specification.
std::unique_ptr<FuzeAdapter> makeFuzeAdapter(
    const components::DescriptorV1&, const Spec&, std::string& error);

} // namespace fsim::ms::weapon
