#include "simulation/mission_systems/weapon/LaunchEligibility.h"
#include "simulation/mission_systems/Cards.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace fsim::ms::weapon;
using fsim::ms::Reason;

namespace {
constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}

fsim::sim::VehicleState targetAt(double angleDeg) {
    fsim::sim::VehicleState target;
    target.positionEcef[0] = 1000.0 * std::cos(angleDeg * kPi / 180.0);
    target.positionEcef[1] = 1000.0 * std::sin(angleDeg * kPi / 180.0);
    return target;
}

void default_fov_depends_on_infrared_substring() {
    const fsim::sim::VehicleState own;
    const auto insideHeat = targetAt(24.999);
    const auto outsideHeat = targetAt(25.001);
    fsim::ms::StoreCard card;
    for (const char* seeker : {"infrared", "imaging_infrared", "custom_infrared_suffix"}) {
        card.seeker = seeker;
        require(checkLaunchEligibility(card, own, &insideHeat).reason == Reason::None,
                "Every lowercase infrared substring must include targets inside 25 degrees");
        require(checkLaunchEligibility(card, own, &outsideHeat).reason == Reason::OutOfReach,
                "Every lowercase infrared substring must reject targets outside 25 degrees");
    }
    const auto insideRadar = targetAt(54.999);
    const auto outsideRadar = targetAt(55.001);
    for (const char* seeker : {"active_radar", "semi_active_radar", "", "Infrared"}) {
        card.seeker = seeker;
        require(checkLaunchEligibility(card, own, &outsideHeat).reason == Reason::None,
                "Other seekers must use the wider radar default");
        require(checkLaunchEligibility(card, own, &insideRadar).reason == Reason::None,
                "The radar default must include targets inside 55 degrees");
        require(checkLaunchEligibility(card, own, &outsideRadar).reason == Reason::OutOfReach,
                "The radar default must reject targets outside 55 degrees");
    }
}

void card_override_and_inclusive_boundary() {
    const fsim::sim::VehicleState own;
    fsim::ms::StoreCard card;
    card.seeker = "infrared";
    card.parameters["launch_fov_deg"].value = 10.0;
    const auto inside = targetAt(9.999);
    const auto outside = targetAt(10.001);
    require(checkLaunchEligibility(card, own, &inside).reason == Reason::None,
            "A card's launch FOV must override the seeker default");
    require(checkLaunchEligibility(card, own, &outside).reason == Reason::OutOfReach,
            "The narrower card FOV must reject a target accepted by the default");
    fsim::sim::VehicleState side;
    side.positionEcef[1] = 1000.0;
    card.parameters["launch_fov_deg"].value = 90.0;
    require(checkLaunchEligibility(card, own, &side).reason == Reason::None,
            "The exact 90-degree card boundary must be inclusive");
    card.parameters["launch_fov_deg"].value = 89.999;
    require(checkLaunchEligibility(card, own, &side).reason == Reason::OutOfReach,
            "The same target must be rejected just outside the card boundary");
    card.parameters["launch_fov_deg"].value = 0.0;
    const auto ahead = targetAt(0.0);
    require(checkLaunchEligibility(card, own, &ahead).reason == Reason::None,
            "An exactly aligned target must be included by a zero-degree card FOV");
}

void behind_coincident_and_submetre_targets() {
    const fsim::sim::VehicleState own;
    fsim::ms::StoreCard card;
    card.seeker = "active_radar";
    fsim::sim::VehicleState behind;
    behind.positionEcef[0] = -1000.0;
    require(checkLaunchEligibility(card, own, &behind).reason == Reason::OutOfReach,
            "A target behind the nose must be refused");
    require(checkLaunchEligibility(card, own, &own).reason == Reason::OutOfReach,
            "Coincident geometry must retain its 90-degree off-boresight result");
    fsim::sim::VehicleState nearby;
    nearby.positionEcef[0] = 0.5;
    require(checkLaunchEligibility(card, own, &nearby).reason == Reason::OutOfReach,
            "The one-metre norm floor must be preserved for a half-metre aligned target");
    card.parameters["launch_fov_deg"].value = 90.0;
    require(checkLaunchEligibility(card, own, &own).reason == Reason::None,
            "Coincident geometry must remain finite and pass the inclusive 90-degree boundary");
}

void relative_geometry_uses_body_rotation_and_clamp() {
    fsim::sim::VehicleState own;
    own.positionEcef[0] = 1000.0;
    own.positionEcef[1] = -2000.0;
    own.positionEcef[2] = 3000.0;
    const double turned[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    for (int i = 0; i < 9; ++i) own.rotationBodyToEcef[i] = turned[i];
    auto target = own;
    target.positionEcef[1] += 1000.0;
    fsim::ms::StoreCard card;
    card.seeker = "infrared";
    require(checkLaunchEligibility(card, own, &target).reason == Reason::None,
            "The launch ray must be relative to the own position and rotated into body axes");
    target.positionEcef[1] -= 2000.0;
    require(checkLaunchEligibility(card, own, &target).reason == Reason::OutOfReach,
            "A rotated target behind the nose must be rejected");
    own.rotationBodyToEcef[3] = 1.000000000000001;
    target.positionEcef[1] += 2000.0;
    card.parameters["launch_fov_deg"].value = -1.0;
    require(checkLaunchEligibility(card, own, &target).reason == Reason::OutOfReach,
            "A cosine just above one must clamp to zero angle instead of accepting through NaN");
}

void target_and_fov_refusals_precede_illumination() {
    const fsim::sim::VehicleState own;
    fsim::ms::StoreCard card;
    card.seeker = "semi_active_radar";
    const auto missing = checkLaunchEligibility(card, own, nullptr);
    require(missing.reason == Reason::UnknownTrack && !missing.illuminationRequired,
            "A missing target must be rejected before requesting any illumination lookup");
    const auto outside = targetAt(60.0);
    const auto refused = checkLaunchEligibility(card, own, &outside);
    require(refused.reason == Reason::OutOfReach && !refused.illuminationRequired,
            "An out-of-FOV SAR target must keep OutOfReach precedence over missing illumination");
    const auto ahead = targetAt(0.0);
    const auto accepted = checkLaunchEligibility(card, own, &ahead);
    require(accepted.reason == Reason::None && accepted.illuminationRequired,
            "An in-FOV SAR target must request the second illumination check");
    for (const char* seeker : {"active_radar", "infrared", "semi_active_radar_suffix"}) {
        card.seeker = seeker;
        const auto result = checkLaunchEligibility(card, own, &ahead);
        require(result.reason == Reason::None && !result.illuminationRequired,
                "Only the exact semi_active_radar seeker must request illumination");
    }
}

void illumination_requires_designation_owner_track_and_transmission() {
    constexpr std::uint32_t launcher = 7;
    const IlluminationFacts valid{true, launcher, 0x10000002aULL, true, 0x10000002aULL};
    require(checkLaunchIllumination(launcher, valid) == Reason::None,
            "An own designation of the illuminating radar's track must be eligible");
    auto missing = valid;
    missing.designated = false;
    require(checkLaunchIllumination(launcher, missing) == Reason::UnknownTrack,
            "SAR illumination requires a designation");
    auto otherOwner = valid;
    otherOwner.owner = launcher + 1;
    require(checkLaunchIllumination(launcher, otherOwner) == Reason::UnknownTrack,
            "Another launcher's designation must not supply SAR illumination");
    auto idle = valid;
    idle.illuminating = false;
    require(checkLaunchIllumination(launcher, idle) == Reason::UnknownTrack,
            "A radar holding the track without illuminating it must be refused");
    auto otherTrack = valid;
    otherTrack.radarTrack = 0x20000002aULL;
    require(checkLaunchIllumination(launcher, otherTrack) == Reason::UnknownTrack,
            "Illumination of a different designated track must be refused");
}
} // namespace

int main() {
    try {
        default_fov_depends_on_infrared_substring();
        card_override_and_inclusive_boundary();
        behind_coincident_and_submetre_targets();
        relative_geometry_uses_body_rotation_and_clamp();
        target_and_fov_refusals_precede_illumination();
        illumination_requires_designation_owner_track_and_transmission();
        std::cout << "Launch eligibility tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Launch eligibility test failed: " << e.what() << '\n';
        return 1;
    }
}
