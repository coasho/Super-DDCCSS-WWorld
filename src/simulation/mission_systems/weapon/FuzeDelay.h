#pragma once

#include "Fuze.h"

namespace fsim::ms::weapon {

struct FuzeDelayResultV1 {
    bool valid=false, pending=false, due=false;
    std::uint32_t targetId=0;
    double ageS=std::numeric_limits<double>::quiet_NaN();
    double fraction=std::numeric_limits<double>::quiet_NaN();
};

/// One initiation channel for one release. A trigger is a captured absolute
/// launch age, never a motion prediction. Call due() in its containing captured
/// decision window and then in every contiguous weapon window, regardless of
/// seeker loss/retargeting. Only a returned due result authorizes lookup of an
/// actual native pose at that age. No pose or future target motion is generated.
/// This state is separate from transient CPA history and contact consumption.
class FuzeDelayStateV1 {
public:
    /// New release only. Clearing transient seeker/turn history must not call it.
    void reset() noexcept { *this=FuzeDelayStateV1{}; }
    /// Earlier authoritative contact/termination seals rather than rearms.
    void cancel() noexcept { phase_=Phase::Sealed; windowSeen_=false; }
    bool pending() const noexcept { return phase_==Phase::Pending; }
    bool schedule(double triggerAgeS,std::uint32_t targetId,const Spec& spec) noexcept {
        if (phase_!=Phase::Fresh || !validSpec(spec) || targetId==0 ||
            !std::isfinite(triggerAgeS) || triggerAgeS<spec.armingS || triggerAgeS>=spec.lifeS)
            return false;
        const double dueAgeS=triggerAgeS+spec.fuzeDelayS;
        if (!std::isfinite(dueAgeS) || dueAgeS<triggerAgeS) return false;
        phase_=Phase::Pending;
        triggerAgeS_=triggerAgeS; dueAgeS_=dueAgeS; targetId_=targetId;
        delayS_=spec.fuzeDelayS; armingS_=spec.armingS; lifeS_=spec.lifeS;
        return true;
    }
    FuzeDelayResultV1 due(double ageBeforeS,double durationS,const Spec& spec,
                          double eligibleEndS,bool exclusiveEnd) noexcept {
        FuzeDelayResultV1 result;
        const double ageAfterS=ageBeforeS+durationS;
        const double cutoffAgeS=ageBeforeS+eligibleEndS;
        if (!validSpec(spec) || !std::isfinite(ageBeforeS) || ageBeforeS<0 ||
            !std::isfinite(durationS) || durationS<=0 || !std::isfinite(ageAfterS) || ageAfterS<=ageBeforeS ||
            !std::isfinite(eligibleEndS) || eligibleEndS<0 || eligibleEndS>durationS || !std::isfinite(cutoffAgeS)) {
            if (pending()) cancel();
            return result;
        }
        result.valid=true;
        if (!pending()) return result;
        if (delayS_!=spec.fuzeDelayS || armingS_!=spec.armingS || lifeS_!=spec.lifeS ||
            (windowSeen_ ? ageBeforeS!=previousEndS_ : (triggerAgeS_<ageBeforeS || triggerAgeS_>ageAfterS)) ||
            dueAgeS_<ageBeforeS) {
            cancel(); return {};
        }
        if (triggerAgeS_>cutoffAgeS || (exclusiveEnd && triggerAgeS_==cutoffAgeS)) {
            cancel(); return result;
        }
        if (dueAgeS_<=cutoffAgeS && dueAgeS_<spec.lifeS &&
            !(exclusiveEnd && dueAgeS_==cutoffAgeS)) {
            const double fraction=(dueAgeS_-ageBeforeS)/durationS;
            if (!std::isfinite(fraction) || fraction<0 || fraction>1) {cancel();return {};}
            result.due=true; result.targetId=targetId_; result.ageS=dueAgeS_; result.fraction=fraction;
            cancel();
            return result;
        }
        // A known native cutoff or weapon expiry cannot be bridged by a future
        // window. An ordinary full live window retains its fixed target/time.
        if (exclusiveEnd || eligibleEndS<durationS || ageAfterS>=spec.lifeS) {
            cancel(); return result;
        }
        windowSeen_=true; previousEndS_=ageAfterS;
        result.pending=true; result.targetId=targetId_;
        return result;
    }

private:
    static bool validSpec(const Spec& spec) noexcept {
        return spec.fuzeModelVersion==1 && fuze_detail::validModel(spec) &&
               std::isfinite(spec.armingS) && spec.armingS>=0 &&
               std::isfinite(spec.lifeS) && spec.lifeS>spec.armingS;
    }
    enum class Phase { Fresh, Pending, Sealed };
    Phase phase_=Phase::Fresh;
    bool windowSeen_=false;
    std::uint32_t targetId_=0;
    double triggerAgeS_=0,dueAgeS_=0,previousEndS_=0;
    double delayS_=0,armingS_=0,lifeS_=0;
};

} // namespace fsim::ms::weapon
