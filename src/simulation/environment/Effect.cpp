#include "simulation/environment/Effect.h"

#include "simulation/flight/FlightModel.h"

namespace fsim::effects {

sim::PropertyHandle EffectContext::property(std::string_view path) { return model_.property(path); }

} // namespace fsim::effects
