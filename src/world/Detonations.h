#pragma once

#include "ipc/WorldLayout.h"
#include "world/Blackbody.h"
#include "world/Clouds.h"

#include <vsg/all.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <vector>

namespace fsim::world {

/// A warhead's detonation as the viewer draws it (MS-7h; the engagement's 'V' events, ipc::EngagementKind::Detonation):
/// the fireball and the smoke it leaves, held to what is measured of high explosives in air.
///
/// - The fireball: its greatest diameter 4.67 W^0.31 m for W kg of explosive (TNT equivalent) in free air - a 1 kg TNT
///   charge's 5.89 m and a 10 kg one's 12.05 m in high-speed photography (20,000 frames a second), fired at the ground:
///   a hemisphere, which holds the products a sphere 2^(-1/3) as wide holds in free air - larger where the air is
///   thinner by (p0 / p)^(1/3), the detonation products expanding until their pressure is the air's (Sachs scaling).
///   It grows to that in some milliseconds (kGrowS W^(1/3), an estimate: the products' expansion at kilometres a
///   second over metres) and glows as the hot products burn in the air - TNT's oxygen balance is -74 %, half its energy
///   released in that afterburning - its surface near 2,000 K (TNT's, colour pyrometry) for kHoldS W^(1/3), then cools
///   over kCoolS W^(1/3) (estimates, scaled as the cube root the measured durations follow). Its light is Planck's at
///   its temperature, against the sun's disk (5,778 K) in the atmosphere's units: what it is next to the sky, not a
///   brightness chosen to look right.
/// - The smoke: TNT's products are rich in carbon; what the afterburning leaves, kSootShare of the charge's mass (an
///   estimate), is soot absorbing 7.5 m^2/g (fresh soot's mass absorption, Bond and Bergstrom 2006) and scattering
///   little (its single-scattering albedo kSootAlbedo). A Gaussian puff, the fireball's size as it cools, growing as a
///   decaying turbulent puff does (its radius as t^(1/4) once formed, an estimate), carried by the wind; gone once its
///   optical depth through the middle is under kSeen.
/// - Not drawn: the warhead's fragments (their spray is brief and seldom photographed).
///
/// A fuel fire's first fireball (an engagement's Fire event: the fuel a holed tank spills at once) is a hydrocarbon
/// fireball as the process-safety literature has it (CCPS; Roberts 1981): its diameter 5.8 M^(1/3) m and its
/// duration 0.45 M^(1/3) s for M kg of fuel, larger in thinner air as the detonation's (its products at the air's
/// pressure), growing over its first third (an estimate), its surface emissive power some 350 kW/m^2 - a blackbody at
/// kFuelK - fading over its last fifth; its soot kerosene's (kFuelSootYield of the fuel, Tewarson's for a sooty liquid
/// fuel). The QF-4B hit off Point Mugu shows one about the aircraft's length.
class Detonations {
public:
    /// The air it happens in, and the wind that carries the smoke (from, deg; m/s).
    struct Air {
        double seaLevelK = 288.15, seaLevelPa = 101325.0, windFromDeg = 0.0, windMs = 0.0;
    };
    /// What a detonation is at an age (s): the fireball's radius (m) and temperature (K, 0: no light), the smoke's
    /// sigma (m) and its total extinction (m^2: its soot's mass times its absorption).
    struct Burst {
        double radius = 0.0, temperature = 0.0, smokeSigma = 0.0, smokeE = 0.0;
        bool gone = false;
    };
    static constexpr double kSurfaceDiameter1Kg = 5.89; // m: 1 kg TNT's, fired at the ground (a hemisphere)
    static constexpr double kDiameterExponent = 0.31;    // the mass's power (1 and 10 kg)
    /// m, for 1 kg TNT in free air: the hemisphere's products in a sphere
    static double freeAirDiameter1Kg() { return kSurfaceDiameter1Kg / std::cbrt(2.0); }
    static constexpr double kGrowS = 0.004, kHoldS = 0.03, kCoolS = 0.05;  // s per kg^(1/3) (estimates)
    static constexpr double kSurfaceK = 2000.0, kFlashK = 3500.0;          // TNT's fireball surface; the products' at first
    static constexpr double kSootShare = 0.05, kSootAbsorption = 7500.0, kSootAlbedo = 0.2; // -, m^2/kg, -
    static constexpr double kPuffExponent = 0.25, kSeen = 0.03, kLongestS = 240.0;
    static constexpr double kFuelDiameter1Kg = 5.8, kFuelDuration1Kg = 0.45; // m, s for 1 kg of fuel (CCPS)
    static constexpr double kFuelK = 1500.0, kFuelSootYield = 0.04;          // (350 kW/m^2); kg soot per kg fuel
    /// a spill's fuel leaves its holed tank over this long, as the aircraft flies on, its fireball a chain of lobes
    /// along its path (estimates: the QF-4B's photographed some 25 - 35 m long behind it at 200 m/s)
    static constexpr double kReleaseS = 0.15;
    static constexpr int kReleaseLobes = 5;
    /// The fireball's greatest diameter (m) for this much explosive (kg TNT) in air at this pressure (Pa).
    static double fireballDiameter(double tntKg, double pressurePa);
    /// What it is at `age` s, in air at this pressure.
    static Burst at(double tntKg, double pressurePa, double age);
    /// A fuel fireball's (M kg of fuel): its diameter (m) in air at this pressure, and what it is at `age` s.
    static double fuelFireballDiameter(double fuelKg, double pressurePa);
    static Burst fuelAt(double fuelKg, double pressurePa, double age);
    /// The pressure (Pa) at a height in that air (the standard atmosphere from the sea level's).
    static double pressureAt(const Air& air, double altitudeM);

    /// `coarseShading`: the device can shade a draw at a coarser rate than its pixels (render::Viewer::coarseShading).
    Detonations(vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders, const AtmosphereTables& tables,
                bool coarseShading = false);
    bool valid() const noexcept { return static_cast<bool>(draw_); }
    vsg::ref_ptr<vsg::Node> node() const { return root_; }
    void setVisible(bool on);
    /// The window's height (px): a pixel's angle.
    void setScreen(std::uint32_t width, std::uint32_t height);
    void setAir(const Air& air) noexcept { air_ = air; }
    /// An engagement's event: a detonation is drawn from then on; other kinds are not this one's.
    void add(const ipc::EngagementEvent& event, const vsg::EllipsoidModel& ellipsoid);
    /// Forgets them all (a replay's seek, its loop).
    void clear() { bursts_.clear(); }
    std::size_t count() const noexcept { return bursts_.size(); }
    /// Per frame: the world's time (s), the camera (projection, the eye ECEF), the ellipsoid and the sun (ECEF).
    void update(double now, const vsg::dmat4& projection, const vsg::dvec3& eye, const vsg::EllipsoidModel& ellipsoid,
                const vsg::dvec3& sun);
    static constexpr std::size_t kMost = 64; ///< drawn at once (the oldest dropped)

private:
    struct Live {
        vsg::dvec3 at;        ///< where it went off (ECEF)
        double time = 0.0, tntKg = 0.0, pressurePa = 101325.0, altitude = 0.0;
        double seed = 0.0;
        bool fuel = false; ///< a fuel fireball (tntKg its fuel's mass), not a warhead's
        Burst burst(double age) const { return fuel ? fuelAt(tntKg, pressurePa, age) : Detonations::at(tntKg, pressurePa, age); }
    };
    std::vector<Live> bursts_;
    Air air_;
    vsg::ref_ptr<vsg::Group> root_;
    vsg::ref_ptr<vsg::Switch> switch_;
    vsg::ref_ptr<vsg::MatrixTransform> at_;
    vsg::ref_ptr<vsg::Draw> draw_;
    vsg::ref_ptr<vsg::vec4Array> rows_, frame_;
    SunlitBlackbody blackbody_; ///< Planck's light against the sun's
    bool visible_ = true;
    std::uint32_t screenHeight_ = 1080, coarse_ = 1;
    vsg::ref_ptr<vsg::Command> shadingRate_; ///< set before the draw (null: per pixel)
};

} // namespace fsim::world
