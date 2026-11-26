#pragma once
#include "fsim/MissionSystems.h"
#include "simulation/mission_systems/Cards.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace fsim::ms::weapon {

/// Flyout-derived 5-D launch-zone envelope table generated from native 6-DOF
/// JSBSim missile flyouts (SIM-M04 / MS-8). Immutable after loading.
struct LaunchZoneTableV1 {
    std::string schema;
    std::string card;
    std::string model;
    std::string seeker;
    std::string basis;
    std::string tableSha256;
    std::string modelXmlSha256;
    std::string cardSha256;
    bool equipmentCalibrated = false;

    double armingS = 1.0;
    double lifeS = 60.0;
    double activeRangeM = 0.0;
    double maxOffBoresightRad = 1.0471975511965976; // 60 deg default
    double escapeTurnG = 7.0;
    double minTerminalSpeedMs = 120.0;

    // 5-D grid axes (strictly increasing, >= 2 knots each).
    std::vector<double> launchAltitudeM;
    std::vector<double> launchMach;
    std::vector<double> targetAltitudeDeltaM;
    std::vector<double> targetMach;
    std::vector<double> aspectDeg;

    // Off-boresight kinematic energy-loss curve (strictly increasing angles, factors in (0, 1]).
    std::vector<double> offBoresightDeg;
    std::vector<double> offBoresightFactor;

    // Propagated 1-sigma fractional uncertainties from perturbed JSBSim flyouts.
    double uncRMax = 0.15;
    double uncRNe = 0.15;
    double uncRMin = 0.15;
    double uncTof = 0.12;

    // Flat row-major arrays indexed by (iAlt, iLM, iDh, iTM, iAsp).
    std::vector<double> rMaxM;
    std::vector<double> rNeM;
    std::vector<double> rMinM;
    std::vector<double> tofMaxS;
    std::vector<double> tofNeS;
    std::vector<double> tofMinS;

    std::size_t gridSize() const noexcept {
        return launchAltitudeM.size() * launchMach.size() * targetAltitudeDeltaM.size() *
               targetMach.size() * aspectDeg.size();
    }
};

/// Kinematic query inputs for DLZ / WEZ evaluation (platform + own designation estimate).
struct LaunchZoneQueryV1 {
    double launchPosEcef[3] = {0.0, 0.0, 0.0};
    double launchVelEcef[3] = {0.0, 0.0, 0.0};
    double launchForwardEcef[3] = {1.0, 0.0, 0.0};
    double launchAltitudeM = 0.0;
    double launchAirspeedMs = 0.0;

    double targetPosEcef[3] = {0.0, 0.0, 0.0};
    double targetVelEcef[3] = {0.0, 0.0, 0.0};
    double targetAltitudeM = 0.0;
};

/// Standard ISA speed of sound (m/s) at geometric/geopotential altitude (m).
double isaSoundSpeedMs(double altitudeM) noexcept;

/// Exact FIPS 180-4 SHA-256 hex digest (64 lowercase hex characters).
std::string computeSha256Hex(const std::string& bytes);

/// Load and strictly validate a `.dlz-v1.json` envelope table.
/// On failure, returns false, sets `error`, and leaves `out` untouched.
bool loadLaunchZoneTableV1(const std::filesystem::path& path, LaunchZoneTableV1& out, std::string& error);

/// Evaluate real-time flyout-derived DLZ from a verified `.dlz-v1.json` table.
LaunchZoneV2 evaluateLaunchZoneV2(const LaunchZoneTableV1& table, const LaunchZoneQueryV1& query) noexcept;

/// Evaluate aspect-polar WEZ contour from a verified `.dlz-v1.json` table.
LaunchZoneWezV1 evaluateLaunchZoneWezV1(const LaunchZoneTableV1& table, const LaunchZoneQueryV1& query);

/// Legacy assumed advisory fallback when no `.dlz-v1.json` table is available.
LaunchZoneV2 evaluateLegacyLaunchZoneV2(const StoreCard& card, const LaunchZoneQueryV1& query);

/// Legacy assumed aspect-polar WEZ fallback when no `.dlz-v1.json` table is available.
LaunchZoneWezV1 evaluateLegacyLaunchZoneWezV1(const StoreCard& card, const LaunchZoneQueryV1& query);

} // namespace fsim::ms::weapon
