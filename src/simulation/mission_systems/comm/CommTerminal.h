#pragma once

// SIM-M07 (MS-11): Tactical Communications Terminals & Mode 5 / Mode S IFF Link Model.
// Covers all 9 comm_terminal cards without reimplementing native DMS/C2 delivery:
// - Models real-band RF jammer interference & FHSS/directional processing gain
//   (Link-16 969-1206 MHz, MADL/IFDL Ku-band, AN/ARC-210 VHF/UHF, U-2S datalinks).
// - Models ICAO Annex 10 / STANAG 4193 1030/1090 MHz Mode 5 IFF link closure on fitted suites
//   (`mids-jtrs`, `an-asq-242`).

#include "simulation/mission_systems/Model.h"

namespace fsim::ms::comm_payload {

inline constexpr double kIffInterrogateHz = 1.030e9;
inline constexpr double kIffReplyHz = 1.090e9;

/// ICAO Annex 10 / STANAG 4193 1030/1090 MHz IFF transponder reply link margin (dB)
/// at receiver given transmit ERP (dBW), path loss (dB), receiver gain (dBi), and sensitivity (dBm).
double iffLinkMarginDb(double txErpDbw, double pathLossDb, double rxGainDbi, double rxSensitivityDbm) noexcept;

/// Construct a communications terminal model from its payload card.
std::unique_ptr<Model> makeCommTerminal(const Card& card, std::uint16_t count);

} // namespace fsim::ms::comm_payload
