// Synthetic temporal/geometry contracts; these values calibrate no real fuze.
#include "simulation/mission_systems/weapon/Fuze.h"
#include "simulation/mission_systems/weapon/MotionHistory.h"
#include "simulation/mission_systems/weapon/FuzeDelay.h"
#include "simulation/mission_systems/weapon/CaptureEpoch.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

using namespace fsim::ms::weapon;

namespace {
void require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}
bool near(double a, double b) { return std::abs(a - b) < 1e-12; }
Spec spec() {
    Spec s;
    s.armingS = 1;
    s.lifeS = 3;
    s.fuzeM = 1;
    return s;
}
void canonicalCapturedEpochDoesNotSubtractNativeClocks() {
    CaptureEpochV1 epoch;
    constexpr double dt=.01;
    const double duration=7*dt;
    for (unsigned k=0;k<=7;++k) {
        const auto offset=epoch.capture(k,dt,false);
        require(offset && *offset==k*dt,"Actual native sample ordinals must map to exact World offsets");
    }
    const double nativeOpening=1000, nativeClosing=nativeOpening+duration;
    require(nativeClosing-nativeOpening!=duration && epoch.lastOffsetS()==duration,
            "A full captured window must remain exact despite subtractive native-clock roundoff");
    auto selected=spec(); selected.fuzeModelVersion=selected.fuzeAlgorithm=1;
    selected.armingS=0; selected.fuzeDelayS=.1;
    FuzeDelayStateV1 delay;
    require(delay.schedule(.02,7,selected) && delay.due(0,duration,selected,epoch.lastOffsetS(),false).pending,
            "A complete captured window must retain an initiation due in the next window");
    require(delay.due(duration,duration,selected,duration,false).due,
            "The retained initiation must execute only in its contiguous actual captured due window");
    CaptureEpochV1 stopped;
    require(stopped.capture(0,dt,false) && stopped.capture(1,dt,false) && stopped.capture(2,dt,true),
            "The first actual terminal snapshot belongs to its observed integration ordinal");
    require(!stopped.capture(3,dt,true) && !stopped.capture(7,dt,true) && stopped.lastOffsetS()==2*dt,
            "Repeated native terminal no-ops must never become future poses");
    CaptureEpochV1 gap;
    require(!gap.capture(1,dt,false) && gap.capture(0,dt,false) && !gap.capture(2,dt,false),
            "A missing native callback cannot manufacture a contiguous captured history");
}
void armedSweepUsesClosestApproach() {
    const auto result = sweptFuze({-10, 0, 0}, {10, 0, 0}, {0, .5, 0}, {0, .5, 0}, 1, 2, spec());
    require(result.valid && result.triggered && near(result.missM, .5) && near(result.fraction, .5),
            "An armed sweep must burst at closest approach, rather than the first radius crossing");
}
void safeSweepCannotBurst() {
    const auto result = sweptFuze({-10, 0, 0}, {10, 0, 0}, {}, {}, 0, .9, spec());
    require(result.valid && !result.triggered && std::isnan(result.missM) && std::isnan(result.fraction),
            "A wholly safe step has no eligible closest approach");
}
void armingClipsPreArmingClosestApproach() {
    auto s = spec(); s.fuzeM = .5;
    const auto result = sweptFuze({-1, 0, 0}, {3, 0, 0}, {}, {}, .9, 1.1, s);
    require(result.valid && !result.triggered && near(result.missM, 1) && near(result.fraction, .5),
            "A pre-arming crossing must not authorize a later burst at the excluded crossing");
}
void armingRetainsPostArmingClosestApproach() {
    const auto result = sweptFuze({-3, 0, 0}, {1, 0, 0}, {}, {}, .9, 1.1, spec());
    require(result.valid && result.triggered && near(result.missM, 0) && near(result.fraction, .75),
            "Clipping must preserve the original step fraction of a post-arming crossing");
}
void exactArmingIsInclusive() {
    const auto result = sweptFuze({-2, 0, 0}, {}, {}, {}, .5, 1, spec());
    require(result.valid && result.triggered && near(result.missM, 0) && result.fraction == 1,
            "The arming instant at the step endpoint is eligible");
}
void expiryExcludesBoundaryOnlyContact() {
    const auto result = sweptFuze({-2, 0, 0}, {}, {}, {}, 2, 4, spec());
    require(result.valid && !result.triggered,
            "Entering the radius exactly at expiry must not burst, including rounded endpoint evaluation");
}
void expiryExcludesLaterClosestApproach() {
    auto s = spec(); s.fuzeM = .5;
    const auto result = sweptFuze({-3, 0, 0}, {1, 0, 0}, {}, {}, 2, 4, s);
    require(result.valid && !result.triggered,
            "A closest approach after battery expiry must not authorize a burst");
}
void expiryKeepsInteriorOverlap() {
    const auto result = sweptFuze({-2, 0, 0}, {1, 0, 0}, {}, {}, 2, 4, spec());
    require(result.valid && result.triggered && result.fraction < .5 && result.missM < 1,
            "An overlap before expiry must burst at a representable live fraction");
    require(2 + result.fraction * 2 < 3,
            "The selected burst fraction must not round its age back up to expiry");
}
void expiredSweepHasNoActiveInterval() {
    const auto result = sweptFuze({}, {}, {}, {}, 3, 4, spec());
    require(result.valid && !result.triggered && std::isnan(result.missM),
            "An expired step must have no eligible sweep");
}
void stationaryAndMovingTargets() {
    const auto stationary = sweptFuze({}, {}, {0, .5, 0}, {0, .5, 0}, 1, 2, spec());
    require(stationary.valid && stationary.triggered && near(stationary.missM, .5) && stationary.fraction == 0,
            "Stationary relative geometry must use the start of its active interval");
    const auto moving = sweptFuze({}, {2, 0, 0}, {4, 0, 0}, {-2, 0, 0}, 1, 2, spec());
    require(moving.valid && moving.triggered && near(moving.missM, 0) && near(moving.fraction, .5),
            "Both bodies must contribute to relative motion");
}
void invalidInputsFailClosed() {
    const auto check = [](V3 before, V3 after, double begin, double end, const Spec& s) {
        const auto result = sweptFuze(before, after, {}, {}, begin, end, s);
        require(!result.valid && !result.triggered && std::isnan(result.missM) && std::isnan(result.fraction),
                "Invalid fuze inputs must fail closed without usable geometry");
    };
    const auto s = spec();
    check({}, {}, 1, 1, s);
    check({}, {}, 2, 1, s);
    check({}, {}, -1, 2, s);
    check({}, {}, 1, std::numeric_limits<double>::infinity(), s);
    check({std::numeric_limits<double>::quiet_NaN(), 0, 0}, {}, 1, 2, s);
    check({std::numeric_limits<double>::max(), 0, 0}, {-std::numeric_limits<double>::max(), 0, 0}, 1, 2, s);
    for (int field = 0; field < 3; ++field) {
        for (double value : {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
            auto bad = s;
            (field == 0 ? bad.fuzeM : field == 1 ? bad.armingS : bad.lifeS) = value;
            check({}, {}, 1, 2, bad);
        }
    }
    auto noLife = s; noLife.lifeS = noLife.armingS;
    check({}, {}, 1, 2, noLife);
}
void nativePathsKeepCurvedEncounters() {
    auto s=spec(); s.armingS=0; s.lifeS=10; s.fuzeM=.1;
    const std::vector<MotionSample> curved{{0,{-2,2,0}}, {.5,{0,0,0}}, {1,{2,2,0}}};
    const std::vector<MotionSample> stationary{{0,{}},{1,{}}};
    const auto hit=sweptPathFuze(curved,stationary,0,1,s,1,false);
    require(hit.valid && hit.triggered && near(hit.fraction,.5),
            "Native substep path must retain an encounter missed by the outer chord");
    const std::vector<MotionSample> avoided{{0,{-2,0,0}}, {.5,{0,2,0}}, {1,{2,0,0}}};
    const auto miss=sweptPathFuze(avoided,stationary,0,1,s,1,false);
    require(miss.valid && !miss.triggered && miss.missM>1,
            "An outer chord cannot invent an encounter absent from every native segment");
}
void contactAndDecoyExpiryClipBeforeMinimizing() {
    auto s=spec(); s.armingS=1; s.lifeS=10;
    const std::vector<MotionSample> crashed{{0,{-1,0,0}},{.25,{0,0,0}}};
    const std::vector<MotionSample> stationary{{0,{}},{1,{}}};
    const auto safe=sweptPathFuze(crashed,stationary,.5,1,s,.25,true);
    require(safe.valid && !safe.triggered && std::isnan(safe.missM),
            "A crash before arming must not become an armed frozen-pose burst later in a batch");
    s.armingS=0; s.fuzeM=1;
    const std::vector<MotionSample> approach{{0,{-2,0,0}},{1,{1,0,0}}};
    const auto overlap=sweptPathFuze(approach,stationary,0,1,s,.5,true);
    require(overlap.valid && overlap.triggered && overlap.fraction<.5,
            "Clipping an expiring target must preserve an earlier interior overlap");
    const std::vector<MotionSample> tangent{{0,{-2,0,0}},{1,{0,0,0}}};
    const auto expiry=sweptPathFuze(tangent,stationary,0,1,s,.5,true);
    require(expiry.valid && !expiry.triggered,
            "Contact only at target expiry remains excluded");
}
void redundantNativeSamplesPreserveClosestApproach() {
    const std::vector<MotionSample> own{{0,{-2,0,0}},{1,{2,0,0}}};
    const std::vector<MotionSample> target{{0,{0,.5,0}},{1,{0,.5,0}}};
    auto denseOwn=own,denseTarget=target;
    denseOwn.insert(denseOwn.begin()+1,{.3,{-.8,0,0}});
    denseTarget.insert(denseTarget.begin()+1,{.3,{0,.5,0}});
    for(const auto& a : {own,denseOwn}) for(const auto& b : {target,denseTarget}) {
        const auto result=sweptPathFuze(a,b,1,1,spec(),1,false);
        require(result.valid && result.triggered && near(result.fraction,.5) && near(result.missM,.5),
                "Redundant native samples must not alter the same interval's analytic closest approach");
    }
}
void intervalClosestApproachKeepsActiveBoundsAndTies() {
    auto s=spec();s.armingS=0;s.lifeS=10;
    const std::vector<MotionSample> own{{0,{-2,1,0}},{.25,{0,1,0}},{.5,{2,1,0}},
                                       {.75,{0,.5,0}},{1,{2,.5,0}}};
    const std::vector<MotionSample> target{{0,{}},{1,{}}};
    const auto check=[](const FuzeResult& result,double fraction,double miss) {
        require(result.valid && result.triggered && near(result.fraction,fraction) && near(result.missM,miss),
                "Interval CPA must retain the earliest exact tie and exclude geometry beyond active bounds");
    };
    check(sweptPathFuze(own,target,0,1,s,1,false),.75,.5);
    auto tied=own;tied[3].position.y=1;tied[4].position.y=1;
    check(sweptPathFuze(tied,target,0,1,s,1,false),.25,1);
    auto expires=s;expires.lifeS=.6;
    check(sweptPathFuze(own,target,0,1,expires,1,false),.25,1);
    check(sweptPathFuze(own,target,0,1,s,.6,true),.25,1);
}
Spec explicitFuze() {
    auto s=spec();
    s.armingS=0; s.lifeS=10;
    s.fuzeModelVersion=1; s.fuzeAlgorithm=1;
    return s;
}
void explicitLawRejectsApproachingEndpoint() {
    const std::vector<MotionSample> own{{0,{-2,0,0}},{1,{-.5,0,0}}};
    const std::vector<MotionSample> target{{0,{}},{1,{}}};
    auto s=explicitFuze();
    const auto path=sweptPathFuze(own,target,0,1,s,1,false);
    require(path.valid && !path.triggered,
            "Explicit CPA law must not initiate at a still-approaching decision endpoint inside its radius");
    const auto linear=sweptFuze(own.front().position,own.back().position,{}, {},0,1,s);
    require(linear.valid && !linear.triggered,
            "Explicit linear CPA law must not clamp a future stationary point into the captured interval");
    s.fuzeModelVersion=0; s.fuzeAlgorithm=0;
    require(sweptPathFuze(own,target,0,1,s,1,false).triggered,
            "The unversioned interval-minimum convention remains compatible");
}
void explicitLawDoesNotRearmAnEarlierTurn() {
    const std::vector<MotionSample> own{{0,{-1,0,0}},{1,{3,0,0}}};
    const std::vector<MotionSample> target{{0,{}},{1,{}}};
    auto s=explicitFuze(); s.armingS=.5; s.fuzeM=2;
    const auto result=sweptPathFuze(own,target,0,1,s,1,false);
    require(result.valid && !result.triggered,
            "A pre-arming actual CPA must not become a receding boundary initiation at arming");
    s.armingS=.25;
    const auto inclusive=sweptPathFuze(own,target,0,1,s,1,false);
    require(inclusive.valid && inclusive.triggered && near(inclusive.fraction,.25),
            "An actual stationary point exactly at inclusive arming remains eligible");
}
void explicitLawExcludesExpiryAndCutoffClamps() {
    const std::vector<MotionSample> own{{0,{-2,0,0}},{1,{.5,0,0}}};
    const std::vector<MotionSample> target{{0,{}},{1,{}}};
    auto s=explicitFuze(); s.lifeS=.6;
    const auto expiry=sweptPathFuze(own,target,0,1,s,1,false);
    require(expiry.valid && !expiry.triggered,
            "Approaching overlap before expiry is not an actual CPA in the explicit law");
    s.lifeS=10;
    const auto cutoff=sweptPathFuze(own,target,0,1,s,.6,true);
    require(cutoff.valid && !cutoff.triggered,
            "An exclusive native cutoff cannot replace a later actual CPA with its clamped endpoint");
    s.lifeS=.8;
    require(!sweptPathFuze(own,target,0,1,s,1,false).triggered,
            "An actual CPA exactly at exclusive weapon expiry remains excluded");
    s.lifeS=10;
    require(!sweptPathFuze(own,target,0,1,s,.8,true).triggered,
            "An actual CPA exactly at exclusive native cutoff remains excluded");
    const auto included=sweptPathFuze(own,target,0,1,s,.8,false);
    require(included.valid && included.triggered && near(included.fraction,.8),
            "An actual CPA at an inclusive captured endpoint remains eligible");
}
void explicitLawSelectsFirstQualifiedTurn() {
    const std::vector<MotionSample> own{{0,{-2,1,0}},{.25,{0,1,0}},{.5,{2,1,0}},
                                       {.75,{0,.5,0}},{1,{2,.5,0}}};
    const std::vector<MotionSample> target{{0,{}},{1,{}}};
    auto s=explicitFuze();
    const auto first=sweptPathFuze(own,target,0,1,s,1,false);
    require(first.valid && first.triggered && near(first.fraction,.25) && near(first.missM,1),
            "Explicit initiation must select the first qualified captured turn, not a later smaller miss");
    s.fuzeM=.75;
    const auto later=sweptPathFuze(own,target,0,1,s,1,false);
    require(later.valid && later.triggered && near(later.fraction,.75) && near(later.missM,.5),
            "An earlier turn outside radius must not suppress a later qualified turn");
}
void explicitLawRequiresCapturedTargetMotion() {
    const std::vector<MotionSample> own{{0,{-2,0,0}},{1,{2,0,0}}};
    const std::vector<MotionSample> shortTarget{{0,{}},{.25,{}}};
    const auto result=sweptPathFuze(own,shortTarget,0,1,explicitFuze(),1,false);
    require(result.valid && !result.triggered,
            "Explicit CPA must not invent a stationary target beyond its last captured native sample");
}
void explicitLawRejectsUnknownDispatch() {
    auto s=explicitFuze();
    for (const auto mode : {std::pair<unsigned,unsigned>{2,1},{1,0},{1,2},{0,1}}) {
        s.fuzeModelVersion=mode.first; s.fuzeAlgorithm=mode.second;
        const auto result=sweptFuze({-2,0,0},{2,0,0},{},{},0,1,s);
        require(!result.valid && !result.triggered && std::isnan(result.fraction),
                "Unknown or inconsistent explicit fuze model/law must fail closed");
    }
}
void explicitCornerAndLinearSubdivisionsAgree() {
    const std::vector<MotionSample> corner{{0,{-2,0,0}},{.5,{-.5,0,0}},{1,{-2,0,0}}};
    const std::vector<MotionSample> target{{0,{}},{1,{}}};
    auto dense=corner; dense.insert(dense.begin()+1,{.25,{-1.25,0,0}});
    for (const auto& path : {corner,dense}) {
        const auto result=sweptPathFuze(path,target,0,1,explicitFuze(),1,false);
        require(result.valid && result.triggered && near(result.fraction,.5) && near(result.missM,.5),
                "A captured closing-to-opening native corner must retain its exact time under subdivision");
    }
    const auto stationary=sweptPathFuze(target,target,0,1,explicitFuze(),1,false);
    require(stationary.valid && !stationary.triggered,
            "Stationary overlap with no approach-to-recession turn does not establish an explicit CPA trigger");
}
void retainedTurnHistoryClosesDecisionBoundary() {
    const std::vector<MotionSample> whole{{0,{-2,0,0}},{.5,{-.5,0,0}},{1,{-2,0,0}}};
    const std::vector<MotionSample> target{{0,{}},{1,{}}};
    const auto expected=sweptPathFuze(whole,target,0,1,explicitFuze(),1,false);
    FuzeTurnStateV1 state;
    const std::vector<MotionSample> approaching{{0,{-2,0,0}},{.5,{-.5,0,0}}};
    const std::vector<MotionSample> departing{{0,{-.5,0,0}},{.5,{-2,0,0}}};
    const std::vector<MotionSample> stationary{{0,{}},{.5,{}}};
    const auto before=state.evaluate(approaching,stationary,0,.5,explicitFuze(),.5,false,7);
    require(before.valid && !before.triggered,
            "The first decision cannot infer an unobserved turn after its approaching endpoint");
    const auto turn=state.evaluate(departing,stationary,.5,.5,explicitFuze(),.5,false,7);
    require(turn.valid && turn.triggered && turn.fraction==0 &&
            near(.5+turn.fraction*.5,expected.fraction) && near(turn.missM,expected.missM),
            "Contiguous same-target retained motion must confirm the shared corner at its actual boundary epoch");
}
void retainedTurnHistoryCannotBridgeSwitchGapOrReset() {
    const std::vector<MotionSample> approaching{{0,{-2,0,0}},{.5,{-.5,0,0}}};
    const std::vector<MotionSample> departing{{0,{-.5,0,0}},{.5,{-2,0,0}}};
    const std::vector<MotionSample> target{{0,{}},{.5,{}}};
    for (int mismatch=0;mismatch<4;++mismatch) {
        FuzeTurnStateV1 state;
        state.evaluate(approaching,target,0,.5,explicitFuze(),.5,false,7);
        if(mismatch==3) state.reset();
        auto own=departing;
        if(mismatch==2) own.front().position.x=-.25;
        const auto result=state.evaluate(own,target,mismatch==1?.6:.5,.5,
                                        explicitFuze(),.5,false,mismatch==0?8:7);
        require(!result.triggered,
                "A changed target, native epoch gap, position discontinuity or release reset cannot inherit a prior turn");
    }
}
void delayWaitsForCapturedDueWindowAndConsumesOnce() {
    auto s=explicitFuze(); s.fuzeDelayS=.3;
    FuzeDelayStateV1 delay;
    require(delay.schedule(1.25,7,s) && delay.pending(),
            "An admitted trigger must schedule one pending delayed initiation");
    require(!delay.schedule(1.3,8,s),"A pending initiation cannot be replaced by a later target trigger");
    const auto waiting=delay.due(1,.5,s,.5,false);
    require(waiting.valid && waiting.pending && !waiting.due && std::isnan(waiting.fraction),
            "A pending future timestamp must not predict an unavailable native burst pose");
    const auto burst=delay.due(1.5,.5,s,.5,false);
    require(burst.valid && burst.due && !burst.pending && burst.targetId==7 &&
            near(burst.ageS,1.55) && near(burst.fraction,.1) && !delay.pending(),
            "The first containing captured window must return the original target and exact delayed epoch once");
    const auto repeated=delay.due(1.5,.5,s,.5,false);
    require(repeated.valid && !repeated.due && !repeated.pending,
            "A consumed delayed initiation must not detonate twice");
    require(!delay.schedule(2,9,s),"Only a new release reset may schedule again after consumption");
    delay.reset();
    require(delay.schedule(2,9,s),"A new release reset clears delayed-initiation ownership");
}
void delayZeroAndBoundariesUseExactEpochs() {
    auto s=explicitFuze();
    FuzeDelayStateV1 immediate;
    require(immediate.schedule(.25,3,s),"An explicit zero model delay is admitted");
    const auto zero=immediate.due(0,.5,s,.5,false);
    require(zero.valid && zero.due && near(zero.ageS,.25) && near(zero.fraction,.5),
            "Zero delay preserves the actual trigger epoch rather than the decision endpoint");
    s.fuzeDelayS=.25;
    FuzeDelayStateV1 endpoint;
    require(endpoint.schedule(.25,4,s),"A live trigger may schedule at an inclusive captured end");
    const auto included=endpoint.due(0,.5,s,.5,false);
    require(included.valid && included.due && near(included.fraction,1),
            "A live delayed initiation exactly at an inclusive captured endpoint remains eligible");
    for(int boundary=0;boundary<2;++boundary) {
        FuzeDelayStateV1 excluded;
        auto limited=s; if(boundary==0) limited.lifeS=.5;
        require(excluded.schedule(.25,4,limited),"The trigger precedes the terminal boundary");
        const auto result=excluded.due(0,.5,limited,.5,boundary==1);
        require(result.valid && !result.due && !result.pending && !excluded.pending(),
                "Weapon expiry and exclusive native cutoff cancel delayed initiation at their exact boundary");
    }
}
void delayRefusesUnsampledGapAndMalformedInputs() {
    auto s=explicitFuze(); s.fuzeDelayS=1;
    FuzeDelayStateV1 gap;
    require(gap.schedule(.25,7,s),"A valid delayed trigger is admitted before testing the history gap");
    require(gap.due(0,.5,s,.5,false).pending,"The initial captured window retains a future initiation");
    const auto missing=gap.due(.75,.5,s,.5,false);
    require(!missing.valid && !missing.due && !gap.pending(),
            "A gap in captured native decision windows must fail closed rather than backdate a burst");
    for(double value : {-1.0,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        auto bad=s; bad.fuzeDelayS=value;
        FuzeDelayStateV1 invalid;
        require(!invalid.schedule(.25,7,bad) && !invalid.pending(),
                "Negative or nonfinite configured initiation delay must fail closed");
    }
    FuzeDelayStateV1 invalid;
    require(!invalid.schedule(.25,0,s),"A delayed initiation requires an explicit triggering target identity");
    auto unknown=s;unknown.fuzeAlgorithm=2;
    require(!invalid.schedule(.25,7,unknown),"An unknown explicit timing law must fail closed");
    s.armingS=.5;
    require(!invalid.schedule(.25,7,s),"A trigger before inclusive arming cannot schedule delayed initiation");
    require(!invalid.schedule(s.lifeS,7,s),"A trigger at exclusive weapon expiry cannot schedule delayed initiation");
    auto extreme=explicitFuze(); extreme.lifeS=std::numeric_limits<double>::max();
    extreme.fuzeDelayS=std::numeric_limits<double>::max();
    FuzeDelayStateV1 overflow;
    require(!overflow.schedule(std::numeric_limits<double>::max()/2,7,extreme),
            "A finite delay whose scheduled timestamp overflows must fail closed");
    FuzeDelayStateV1 missingTriggerWindow;
    s.armingS=0;
    require(missingTriggerWindow.schedule(.25,7,s),"A valid trigger precedes the deliberately missing native window");
    require(!missingTriggerWindow.due(.5,.5,s,.5,false).valid,
            "The first delay update must contain its captured trigger rather than silently bridge missing history");
}
void delayCancellationSealsOneRelease() {
    auto s=explicitFuze(); s.fuzeDelayS=.5;
    FuzeDelayStateV1 cancelled;
    require(cancelled.schedule(.25,7,s),"The contact-preemption fixture requires a pending initiation");
    cancelled.cancel();
    require(!cancelled.pending() && !cancelled.schedule(.3,8,s),
            "Earlier contact cancellation seals this release instead of rearming it for another target");
    const auto result=cancelled.due(0,1,s,1,false);
    require(result.valid && !result.due && !result.pending,
            "An earlier authoritative contact prevents the scheduled later detonation");
    cancelled.reset();
    require(cancelled.schedule(.25,8,s),"Only an explicit new-release reset opens the cancelled channel");
}
void delayTimestampIsIndependentOfWindowPartition() {
    auto s=explicitFuze(); s.fuzeDelayS=.5;
    FuzeDelayStateV1 whole,split;
    require(whole.schedule(.25,7,s) && split.schedule(.25,7,s),
            "Both partition comparisons start from the same admitted trigger");
    const auto complete=whole.due(0,1,s,1,false);
    const auto first=split.due(0,.5,s,.5,false);
    const auto second=split.due(.5,.5,s,.5,false);
    require(complete.valid && complete.due && first.valid && first.pending && !first.due &&
            second.valid && second.due && near(complete.ageS,second.ageS) &&
            near(complete.fraction,.75) && near(.5+second.fraction*.5,complete.ageS) &&
            complete.targetId==second.targetId,
            "Subdividing captured windows changes only the local fraction, never the scheduled physical timestamp");
}
}

int main(int argc,char** argv) {
    const struct { const char* name; void (*run)(); } tests[] = {
        {"canonical captured epoch", canonicalCapturedEpochDoesNotSubtractNativeClocks},
        {"v1 approaching endpoint", explicitLawRejectsApproachingEndpoint},
        {"v1 arming turn", explicitLawDoesNotRearmAnEarlierTurn},
        {"v1 expiry and cutoff", explicitLawExcludesExpiryAndCutoffClamps},
        {"v1 earliest turn", explicitLawSelectsFirstQualifiedTurn},
        {"v1 captured target", explicitLawRequiresCapturedTargetMotion},
        {"v1 unknown dispatch", explicitLawRejectsUnknownDispatch},
        {"v1 native corner", explicitCornerAndLinearSubdivisionsAgree},
        {"v1 retained corner", retainedTurnHistoryClosesDecisionBoundary},
        {"v1 history boundaries", retainedTurnHistoryCannotBridgeSwitchGapOrReset},
        {"v1 delay lifecycle", delayWaitsForCapturedDueWindowAndConsumesOnce},
        {"v1 delay boundaries", delayZeroAndBoundariesUseExactEpochs},
        {"v1 delay invalid inputs", delayRefusesUnsampledGapAndMalformedInputs},
        {"v1 delay cancellation", delayCancellationSealsOneRelease},
        {"v1 delay partition", delayTimestampIsIndependentOfWindowPartition},
        {"armed closest approach", armedSweepUsesClosestApproach},
        {"safe sweep", safeSweepCannotBurst},
        {"pre-arming closest approach", armingClipsPreArmingClosestApproach},
        {"post-arming closest approach", armingRetainsPostArmingClosestApproach},
        {"inclusive arming", exactArmingIsInclusive},
        {"exclusive expiry contact", expiryExcludesBoundaryOnlyContact},
        {"expired closest approach", expiryExcludesLaterClosestApproach},
        {"pre-expiry overlap", expiryKeepsInteriorOverlap},
        {"expired interval", expiredSweepHasNoActiveInterval},
        {"relative motion", stationaryAndMovingTargets},
        {"invalid inputs", invalidInputsFailClosed},
        {"native curved paths", nativePathsKeepCurvedEncounters},
        {"contact and target lifetime", contactAndDecoyExpiryClipBeforeMinimizing},
        {"native subdivision invariance", redundantNativeSamplesPreserveClosestApproach},
        {"interval closest approach bounds and ties", intervalClosestApproachKeepsActiveBoundsAndTies},
    };
    try {
        require(argc<=2,"Usage: fuze test [exact case name]");
        unsigned selected=0;
        for (const auto& test : tests) {
            if (argc==2 && test.name!=std::string(argv[1])) continue;
            ++selected;
            test.run();
            std::cout << "PASS: " << test.name << '\n';
        }
        require(selected>0,"The fuze selector must match at least one case");
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
