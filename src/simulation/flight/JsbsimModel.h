#pragma once

#include "simulation/flight/FlightModel.h"

#include <memory>
#include <string>
#include <vector>

namespace JSBSim {
class FGFDMExec;
class FGGroundCallback;
}

namespace fsim::sim {

class GroundProvider;

/// JSBSim adapter: one FGFDMExec per vehicle, JSBSim used as-is (design 7,
/// ADR-3). Construct and load on one thread; step() may run on any thread as
/// long as no two threads touch the same instance concurrently.
class JsbsimModel final : public FlightModel, public TerrainStrikeSourceV1, public RigidMassSourceV1, public NativePositionSourceV1 {
public:
    /// @param dt fixed step in seconds (design 6.4)
    /// @param ground terrain source for gear contact and AGL; shared by all vehicles
    JsbsimModel(double dt, std::shared_ptr<const GroundProvider> ground);
    ~JsbsimModel() override;

    JsbsimModel(const JsbsimModel&) = delete;
    JsbsimModel& operator=(const JsbsimModel&) = delete;

    bool load(const AircraftSpec& aircraft, const InitialConditions& ic) override;
    bool reset(const InitialConditions& ic) override;
    void step(const ControlInputs& inputs) override;
    void setEffectors(const EffectorInputs& effectors) override;
    void state(VehicleState& out) const override;
    PropertyHandle property(std::string_view path) override;
    std::vector<std::pair<std::string, double>> properties(std::string_view prefix) const override;

    double dt() const noexcept override { return dt_; }
    bool loaded() const noexcept override { return loaded_; }

    void setWindNed(double north, double east, double down) override;
    void setTurbulence(double intensity, double windSpeed20ftMs) override;
    void setAtmosphere(double temperatureSeaLevelK, double pressureSeaLevelPa) override;
    void setExternalForceBody(const double forceN[3], const double momentNm[3]) override;
    void seed(std::uint64_t value) override;
    EnergyOnBoard energy() const override;
    bool seizeEngine(int engine) override;
    int failChannel(const std::string& channel) override;
    bool removeMass(double kg, const double structuralM[3]) override;
    bool cgFromOriginBodyM(double out[3]) const override;
    double engineFuelFlowKgS(int engine) const override;
    double refuel(double kg) override;
    bool beginMassChange() override;
    bool applyImpulse(const double linearBodyNs[3], const double angularBodyNms[3]) override;
    bool endMassChange(const double partInertia[6], const double impulseBodyNs[3]) override;
    bool terrainStrikeSnapshotV1(TerrainStrikeSnapshotV1& incoming) const noexcept override;
    bool rigidMassPropertiesV1(RigidMassPropertiesV1& out) const noexcept override;
    bool nativePositionV1(const double positionEcefM[3], NativePositionV1& out) const noexcept override;

    /// Direct access for tests and tooling only.
    JSBSim::FGFDMExec& exec() noexcept { return *fdm_; }
    /// Its rigid body as JSBSim holds it now (for tests: no JSBSim types): its mass (kg), its CG's place and velocity
    /// (ECEF: m, m/s), its body axes in ECEF (row-major, columns the axes), its body rates (rad/s), its CG in its
    /// structural frame (m).
    struct Rigid {
        double massKg = 0.0;
        double cgEcef[3] = {0.0, 0.0, 0.0}, velocityEcef[3] = {0.0, 0.0, 0.0};
        double bodyToEcef[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        double ratesBody[3] = {0.0, 0.0, 0.0};
        double cgStructural[3] = {0.0, 0.0, 0.0};
    };
    Rigid rigid() const;
    /// Its body rates set (rad/s; for tests).
    void setBodyRates(const double ratesBody[3]);

private:
    void silenceOutputs();
    void installExternalReaction();
    void startEngines();
    void applyInitialConditions(const InitialConditions& ic);
    void settleOnGround(const InitialConditions& ic);
    void cacheCommandNodes();
    void saveInitialProperties();
    void restoreInitialProperties();
    bool checkDivergence();
    void checkTerrainStrike();
    void checkSpawnInTerrain(const InitialConditions& ic);
    double stopOnTerrain(double liftM); ///< crashed_: crashState_ = its state lifted by liftM along the vertical, at rest; its speed was

    double dt_;
    std::shared_ptr<const GroundProvider> ground_;
    std::unique_ptr<JSBSim::FGFDMExec> fdm_;
    JSBSim::FGGroundCallback* groundCallback_ = nullptr; ///< the terrain's, which fdm_ owns
    bool loaded_ = false;
    bool diverged_ = false;
    bool crashed_ = false;     ///< struck the terrain: frozen where it struck, on the surface (VehicleState::crashed)
    VehicleState crashState_;  ///< its state as it was stopped there
    VehicleState structuralIncoming_{};
    double structuralIncomingMassKg_ = std::numeric_limits<double>::quiet_NaN();
    bool structuralContact_ = false; ///< contiguous STRUCTURE contact, never gear-only landing contact
    std::uint32_t stepCount_ = 0;
    struct LastGood {
        double latDeg = 0, lonDeg = 0, altM = 0, aglM = 0, speedMs = 0;
        bool onGround = false;
    } lastGood_;

    // Cached command nodes (design 12.2: no string lookups in step()).
    PropertyHandle aileronCmd_, elevatorCmd_, rudderCmd_, flapCmd_, gearCmd_, leftBrakeCmd_, rightBrakeCmd_;
    PropertyHandle speedbrakeCmd_, pitchTrimCmd_;
    std::vector<PropertyHandle> throttleCmd_;
    // External reaction injected at load (design 9.5): magnitude + unit direction properties.
    PropertyHandle extForceMag_, extForceX_, extForceY_, extForceZ_, extMomentMag_, extMomentL_, extMomentM_, extMomentN_;
    PropertyHandle extLocX_, extLocY_, extLocZ_, cgX_, cgY_, cgZ_;
    PropertyHandle lefPosDeg_; ///< leading-edge flaps, where the aircraft's FCS has them (fcs/lef-pos-deg)
    /// A battery, where the aircraft's FCS has one (fsim/battery/...: hangar's multirotors and electric wings,
    /// docs/rotorcraft.md, docs/hangar.md)
    PropertyHandle batteryCharge_, batteryCapacity_, batteryPower_;
    /// Per engine: the rotor's speed where its thruster has none of its own -
    /// a direct thruster whose rotor the flight control system spins (a
    /// multirotor's motors: propulsion/engine[i]/rotor-rpm, docs/rotorcraft.md)
    std::vector<PropertyHandle> rotorRpm_;
    /// The properties JSBSim's own reset leaves as the last vehicle left them, as loaded (saveInitialProperties)
    struct InitialProperty;
    std::vector<InitialProperty> initialProperties_;
    /// What damage changed (MS-7g), for a reset to undo: the stuck actuators' flags, the point masses it added
    /// (kept, at no weight, for the next one: JSBSim cannot take a point mass away)
    std::vector<std::string> failedActuators_;
    std::vector<int> damageMasses_;
    std::size_t damageMassesUsed_ = 0;
    /// Its mass balance run now, its inputs from the other models as a step feeds them.
    void runMassBalance();
    double baseCgIn_[3] = {0.0, 0.0, 0.0}; ///< its empty CG (structural frame, in): its model's origin
    double changeCgIn_[3] = {0.0, 0.0, 0.0}; ///< the CG as a mass change began
    bool changing_ = false;
    /// its own base inertia as loaded (slug ft^2, row by row) and as damage has left it; restored by a reset
    std::vector<double> loadedBaseJ_, baseJ_;
};

} // namespace fsim::sim
