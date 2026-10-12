#pragma once

// A warning receiver (ADR-30, MS-9a): the payload family `esm`. It listens. Every transmitter in the scene whose
// signal reaches its antennas above what it can detect is an emitter contact: a bearing with its error, the power
// it arrives with, and what the emitter is doing as far as its signal says - scanning past, locked on, lighting the
// carrier for a missile. It takes no task: it works while its payload operates.

#include "ms/Model.h"

namespace fsim::ms::esm {

/// The power (dBm) a receiver on a carrier at `own` takes from an emission, over a path that loses `pathLossDb`
/// one way, through an antenna of `antennaGainDbi`: a pulse's power where the emitter pulses, else its average.
double receivedDbm(const Emission& emission, const sim::VehicleState& own, double pathLossDb, double antennaGainDbi) noexcept;

/// A warning receiver from its card. What the card does not give is its class's, the same for every type
/// (docs/sdk/mission-systems.md, "The warning receiver").
std::unique_ptr<Model> makeReceiver(const Card& card);

} // namespace fsim::ms::esm
