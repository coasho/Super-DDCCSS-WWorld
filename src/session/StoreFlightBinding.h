#pragma once

#include "fsim/Property.h"
#include "ms/Host.h"

#include <cstdint>
#include <string>
#include <vector>

namespace fsim::sim {
class FlightModel;
struct VehicleState;
}

namespace fsim::session {

/// A carried-store binding's borrowed property handles and revision state; Entry owns its lifetime.
class StoreFlightBinding {
public:
    void bind(const ms::Host& host, sim::FlightModel& model, const std::vector<std::string>* names);
    void applyMasses(const ms::Host& host, sim::FlightModel& model);
    double weightRoomKg(double maximumKg, double weighedKg, double carriedKg, unsigned tick);
    bool hasStations() const noexcept;
    bool loadChanged() const noexcept;
    void loadPublished() noexcept;
    void transferFuel(ms::Host& host, sim::FlightModel& model, double dt);
    bool addDrag(const sim::VehicleState& s, sim::FlightModel& model, double push[3], double turn[3]) const;

private:
    /// Its stations' point masses in its flight model (MS-7: "station N", hangar's), each set to the stores on it.
    struct StationMass {
        std::uint16_t payload = 0;
        int station = 0;
        PropertyHandle weightLbs;
    };
    std::vector<StationMass> stationMass;
    std::uint64_t massRevision = ~0ull; ///< the store revision its point masses were last set for
    double storesKgAtStep = 0.0;        ///< what its stations held when its flight model last weighed itself (weightRoomKg)
    unsigned storesStamp = ~0u;         ///< the step that was taken at
    /// Where its stores and their carriage add drag (ms::Host::carriageDrag), for the store revision dragRevision;
    /// the dynamic pressure they read.
    std::vector<ms::Host::DragPoint> drag;
    std::uint64_t dragRevision = ~0ull;
    PropertyHandle dragQbar;
    /// The stations' mass revision its host was last told its weight for (control::CapabilityHost::setLoad).
    std::uint64_t loadRevision = ~0ull;
};

} // namespace fsim::session
