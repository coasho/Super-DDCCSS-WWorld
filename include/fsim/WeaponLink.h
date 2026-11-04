#pragma once
#include "fsim/Export.h"
#include "fsim/Fusion.h"
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace fsim::ms {
enum class WeaponLinkDirectionV1 : std::uint32_t { Unsupported=0, Uplink=1, TwoWay=2 };
enum class WeaponLinkReasonV1 : std::uint32_t {
    None=0, Unsupported=1, UnknownWeapon=2, NotOwner=3, Invalid=4,
    RequestConflict=5, Capacity=6, Closed=7, UnknownAssessment=8, Expired=9
};
enum class WeaponAssessmentStateV1 : std::uint32_t {
    Pending=0, Active=1, SoftLost=2, Lost=3, EndOfFlight=4, Canceled=5, Expired=6
};
/// All numeric defaults are explicit engineering policy, not equipment measurements.
struct WeaponLinkProfileV1 {
    WeaponLinkDirectionV1 direction=WeaponLinkDirectionV1::Unsupported;
    double updatePeriodS=.2, feedbackPeriodS=.2, latencyS=.05, jitterS=0, dataRateBps=64000;
    double rangeM=200000, frequencyHz=1e9, lossProbability=0;
    double maximumQueueS=.5, maximumAgeS=2, assessmentTimeoutS=5, retentionS=30;
    std::uint32_t queueCapacity=64, reportCapacity=128, assessmentCapacity=16;
    std::string directionSource;
};
struct WeaponLinkCapabilityV1 {
    std::uint64_t instance=0, activity=0;
    std::uint32_t weapon=0;
    WeaponLinkReasonV1 reason=WeaponLinkReasonV1::UnknownWeapon;
    WeaponLinkProfileV1 profile;
};
struct WeaponAssessmentRequestV1 {
    std::uint64_t instance=0, requestId=0;
    std::uint32_t weapon=0;
    double periodS=.2, durationS=30;
};
struct WeaponAssessmentReceiptV1 {
    bool accepted=false;
    std::uint64_t assessment=0, requestId=0;
    WeaponLinkReasonV1 reason=WeaponLinkReasonV1::Invalid;
};
struct WeaponAssessmentStatusV1 {
    std::uint64_t assessment=0, requestId=0;
    WeaponAssessmentStateV1 state=WeaponAssessmentStateV1::Pending;
    WeaponLinkReasonV1 reason=WeaponLinkReasonV1::UnknownAssessment;
    double requestedTimeS=0, expiresTimeS=0;
    std::optional<double> lastReceivedTimeS;
};
struct WeaponFeedbackV1 {
    std::uint64_t assessment=0, sequence=0;
    std::uint32_t weapon=0;
    double sampleTimeS=0, sentTimeS=0, receivedTimeS=0, validUntilS=0;
    bool navigationValid=false, tracking=false, active=false, supported=false, endOfFlight=false;
    double stateEcef[6]={}; ///< onboard navigation only
    double navigationTimeS=0;
    double navigationCovariance[36]={};
    bool navigationGnssAided=false;
    std::optional<double> timeToActiveS, timeToInterceptS;
};
struct FusedDesignationV1 { std::uint32_t target=0;std::uint64_t entity=0;bool valid=false; };
struct WeaponLinkStatsV1 {
    std::uint64_t sent=0, delivered=0, lost=0, masked=0, outOfRange=0,
        queueRefused=0, expired=0, rejected=0, bytes=0;
};
}
namespace fsim {
class Agent;class Vehicle;
/// Explicit opt-in facade. Holding it does not expose a World or referee.
/// A retained lifetime block refuses every call after its native World closes.
class FSIM_API WeaponLinksV1 {
public:
    explicit WeaponLinksV1(const Agent&);
    explicit WeaponLinksV1(const Vehicle&);
    std::uint64_t instance() const;
    ms::WeaponLinkCapabilityV1 capability(std::uint32_t weapon) const;
    std::uint32_t designateFused(std::uint64_t instance,std::uint64_t entity);
    std::vector<ms::FusedDesignationV1> fusedDesignations() const;
    ms::WeaponAssessmentReceiptV1 assess(const ms::WeaponAssessmentRequestV1&);
    bool cancel(std::uint64_t instance,std::uint32_t weapon,std::uint64_t assessment);
    ms::WeaponAssessmentStatusV1 status(std::uint32_t weapon,std::uint64_t assessment) const;
    std::vector<ms::WeaponFeedbackV1> feedback(std::uint32_t weapon,std::uint64_t assessment,std::uint64_t since=0) const;
private:
    struct Access;
    std::shared_ptr<Access> access_;
    std::uint32_t vehicle_=0,controller_=0;
};
}
