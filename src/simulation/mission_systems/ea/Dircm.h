#pragma once

// Optical/IR Directional Infrared Countermeasures (DIRCM, e.g. AN/AAQ-24 LAIRCM):
// closed-loop turret servo tracking incoming missiles (autonomously while operating
// or cued by a passive optical MAWS/DAS contact via TaskKind::Jam) and emitting a
// modulated MWIR laser beam injected into incoming IR seekers' thermal scenes.

#include "simulation/mission_systems/Model.h"

namespace fsim::ms::ea {

/// Construct a DIRCM optical laser countermeasure model for optical EA cards
/// (e.g. `an-aaq-24` or cards setting `is_dircm > 0.5`). Returns null for RF jammers.
std::unique_ptr<Model> makeDircm(const Card& card, std::uint16_t count);

} // namespace fsim::ms::ea
