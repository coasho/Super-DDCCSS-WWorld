#pragma once
// Passive wind-driven wheel and hinged aerodynamic patch. No target or command input.
#include <cstddef>
#include <cstdint>
#include <vector>

namespace fsim::sim {
struct RolleronVectorV1 {double x=0,y=0,z=0;};
struct RolleronConfigV1 {
    std::uint32_t finId=0;
    std::size_t finIndex=0;
    double nativeDeflectionSign=1;
    RolleronVectorV1 centreFromOriginBodyM,hingeAxisBody{0,1,0};
    double wheelRadiusM=0,captureAreaM2=0,wheelInertiaKgM2=0;
    double wheelDriveCoefficient=0,freeSpinRatio=0,wheelDragCoefficient=0,bearingDampingNms=0;
    double hingeInertiaKgM2=0,hingeStiffnessNmRad=0,hingeDampingNmsRad=0;
    double hingeAerodynamicStiffnessM3=0,limitRad=0,rateRadS=0;
};
struct RolleronInputV1 {
    double dtS=0,densityKgM3=0;
    RolleronVectorV1 airVelocityBodyMs,angularRateInertialBodyRadS,cgFromOriginBodyM;
};
struct RolleronFinV1 {
    std::uint32_t finId=0;
    std::size_t finIndex=0;
    double wheelRadS=0,angleRad=0,rateRadS=0;
};
struct RolleronOutputV1 {
    bool valid=false;
    std::vector<RolleronFinV1> fins;
    RolleronVectorV1 forceBodyN,torqueBodyNm; ///< Interval means; torque about the supplied current CG.
    double dragPowerW=0,harvestedAirPowerW=0,bearingHeatJ=0,hingeHeatJ=0,stopLossJ=0;
    double gyroBodyWorkJ=0,gyroHingeWorkJ=0,internalEnergyJ=0;
};
/// NACA RM SL55C22's roll/hinge coupling, with finite wind-driven wheel spin.
/// The roll projection is deliberately reduced-order; it is not a calibrated
/// six-axis multibody rotor model. Stabilizing load comes from actual patch aero.
class PassiveRolleronV1 {
public:
    explicit PassiveRolleronV1(std::vector<RolleronConfigV1>);
    RolleronOutputV1 advance(const RolleronInputV1&);
    const RolleronOutputV1& snapshot() const noexcept {return output_;}
    void reset();
private:
    struct State {double wheel=0,angle=0,rate=0;};
    std::vector<RolleronConfigV1> config_;
    std::vector<State> state_;
    RolleronOutputV1 output_;
};
}
