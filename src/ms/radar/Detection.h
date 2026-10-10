#pragma once
// A radar's probability of detection (ADR-30, MS-4): one look - a dwell's
// coherent integration, its envelope against a threshold set for the false
// alarm probability - of a target whose RCS fluctuates as its table says.
// Exact closed forms (Meyer and Mayer, "Radar Target Detection", 1973;
// Richards, "Fundamentals of Radar Signal Processing", 2nd ed., ch. 6):
//  - steady (Swerling 0): Marcum's Q, Pd = Q1(sqrt(2 S), sqrt(2 T));
//  - Rayleigh (Swerling 1/2, chi-square of 2 degrees): Pd = Pfa^(1 / (1 + S));
//  - one dominant scatterer (Swerling 3/4, chi-square of 4):
//    Pd = (1 + S T / (2 (1 + S/2)^2)) exp(-T / (1 + S/2)),
// with T = -ln Pfa and S the signal-to-noise ratio (linear).

namespace fsim::ms::radar {

enum class Fluctuation { Steady, Rayleigh, DominantScatterer };

/// The case a table's fluctuation ratio E[s^2]/E[s]^2 stands for: a chi-square of 2/(ratio - 1) degrees of freedom -
/// infinite (steady) at 1, 4 at 1.5, 2 (Rayleigh) at 2 - taken to the nearest of the three.
Fluctuation fluctuationOf(double ratio) noexcept;

/// Marcum's Q function Q1(a, b).
double marcumQ(double a, double b) noexcept;

/// The probability of detecting a target at signal-to-noise ratio `snr` (linear) on one look.
double probabilityOfDetection(double snr, double pfa, Fluctuation f) noexcept;

/// The signal-to-noise ratio (linear) that detects with probability `pd` on one look.
double requiredSnr(double pd, double pfa, Fluctuation f) noexcept;

} // namespace fsim::ms::radar
