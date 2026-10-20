// Synthetic temporal/geometry contracts; these values calibrate no real fuze.
#include "simulation/mission_systems/weapon/Fuze.h"
#include "simulation/mission_systems/weapon/MotionHistory.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

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
}

int main() {
    const struct { const char* name; void (*run)(); } tests[] = {
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
    };
    try {
        for (const auto& test : tests) {
            test.run();
            std::cout << "PASS: " << test.name << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
