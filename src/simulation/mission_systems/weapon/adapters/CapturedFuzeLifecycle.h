#pragma once
#include "NativeFuze.h"
namespace fsim::ms::weapon {
class CapturedFuzeLifecycleV1 : public FuzeAdapter {
public:
    CapturedFuzeLifecycleV1(std::string id, Spec spec) : id_(std::move(id)), spec_(std::move(spec)) {}
    std::string_view implementationId() const noexcept override { return id_; }
    const Spec& spec() const noexcept override { return spec_; }
    ContactFuzeResult evaluateContact(const std::vector<ContactSample>& path,
        const damage::AttachedGeometryView& geometry, const ContactWindow& window) const override {
        if (window.armingS!=spec_.armingS || window.lifeS!=spec_.lifeS) return {};
        return contactFuze(path,geometry,window);
    }
    bool commitContact(const ContactFuzeResult& result) noexcept override { return contact_.commit(result); }
    bool scheduleDelay(double triggerAgeS, std::uint32_t targetId) noexcept override {
        return delay_.schedule(triggerAgeS,targetId,spec_);
    }
    FuzeDelayResultV1 dueDelay(double ageBeforeS, double durationS, double eligibleEndS, bool exclusiveEnd) noexcept override {
        return delay_.due(ageBeforeS,durationS,spec_,eligibleEndS,exclusiveEnd);
    }
    void cancelDelay() noexcept override { delay_.cancel(); }
    void resetTurnHistory() noexcept override { turn_.reset(); }
    void resetRelease() noexcept override { contact_.reset(); turn_.reset(); delay_.reset(); }
    ContactFuzeState& contactState() noexcept override { return contact_; }
    const ContactFuzeState& contactState() const noexcept override { return contact_; }
    FuzeTurnStateV1& turnState() noexcept override { return turn_; }
    const FuzeTurnStateV1& turnState() const noexcept override { return turn_; }
    FuzeDelayStateV1& delayState() noexcept override { return delay_; }
    const FuzeDelayStateV1& delayState() const noexcept override { return delay_; }
protected:
    std::string id_;
    Spec spec_;
    ContactFuzeState contact_;
    FuzeTurnStateV1 turn_;
    FuzeDelayStateV1 delay_;
};

}
