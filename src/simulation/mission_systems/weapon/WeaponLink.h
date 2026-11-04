#pragma once
#include "fsim/WeaponLink.h"
#include "fsim/Rng.h"
#include <algorithm>
#include <deque>
#include <limits>
#include <map>

namespace fsim::ms::weapon {
struct WeaponLinkPathV1 {double distanceM=0;bool clear=true;};
struct WeaponLinkResourceV1 {double busyUntilS=0;};
/// Private endpoint-bound finite transport. Neither public actor bytes nor DMS topics enter it.
class WeaponChannelV1 {
public:
    explicit WeaponChannelV1(WeaponLinkProfileV1 profile,std::uint64_t seed);
    const WeaponLinkProfileV1& profile()const noexcept{return profile_;}
    bool sendInput(const FusedEntityV1&,double now,const WeaponLinkPathV1&,WeaponLinkResourceV1* shared=nullptr);
    void advance(double now);
    std::optional<FusedEntityV1> input() const;
    double inputReceivedS()const noexcept{return inputReceived_;}
    WeaponAssessmentReceiptV1 assess(const WeaponAssessmentRequestV1&,double now,const WeaponLinkPathV1& path={},WeaponLinkResourceV1* shared=nullptr);
    bool needsSample(double now,bool terminal=false) const;
    bool cancel(std::uint64_t assessment);
    bool cancel(std::uint64_t assessment,double now,const WeaponLinkPathV1&,WeaponLinkResourceV1* shared=nullptr);
    std::optional<WeaponAssessmentReceiptV1> retry(const WeaponAssessmentRequestV1&) const;
    bool sample(WeaponFeedbackV1,double now,const WeaponLinkPathV1&);
    WeaponAssessmentStatusV1 status(std::uint64_t assessment)const;
    std::vector<WeaponFeedbackV1> feedback(std::uint64_t assessment,std::uint64_t since=0)const;
    const WeaponLinkStatsV1& stats()const noexcept{return stats_;}
    bool finished(double now)const;
private:
    struct Assessment {
        WeaponAssessmentRequestV1 request;
        WeaponAssessmentStatusV1 status;
        double nextSample=0;
        bool terminalSampled=false,onboard=false,cancelDelivered=false;
        double lastSample=-std::numeric_limits<double>::infinity();
        std::uint64_t sequence=0;
        std::deque<WeaponFeedbackV1> reports;
    };
    struct Packet {double due=0;std::optional<FusedEntityV1> input;std::optional<WeaponFeedbackV1> feedback;std::uint64_t assessmentStart=0,assessmentCancel=0;};
    bool transmit(std::size_t bytes,double now,const WeaponLinkPathV1&,bool returning,double& due,WeaponLinkResourceV1* shared=nullptr);
    static bool valid(const FusedEntityV1&,double now);
    void advanceStates(double now);
    WeaponLinkProfileV1 profile_;
    Rng rng_;
    WeaponLinkResourceV1 uplink_,return_;
    double now_=0,inputReceived_=0,lastSentSample_=-std::numeric_limits<double>::infinity();
    bool lastSentLost_=false;
    std::uint64_t boundFusedId_=0;
    std::optional<FusedEntityV1> input_;
    std::deque<Packet> packets_;
    std::map<std::uint64_t,Assessment> assessments_;
    std::map<std::uint64_t,std::pair<WeaponAssessmentRequestV1,WeaponAssessmentReceiptV1>> requests_;
    std::uint64_t nextAssessment_=1;
    WeaponLinkStatsV1 stats_;
};
}
