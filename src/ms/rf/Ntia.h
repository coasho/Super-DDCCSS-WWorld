#pragma once
// NTIA's reference implementations (third_party/ntia, public domain): ITU-R
// P.528-5 for aeronautical paths with P.676's gaseous absorption, and the
// Irregular Terrain Model (Longley-Rice) over a terrain profile. Each is
// compiled once, in a namespace of its own (NtiaP528.cpp, NtiaItm.cpp): these
// declarations are all the rest of the platform sees of them, none of their
// macros.

namespace fsim::ms::ntia {

/// P.528's answer: its Result structure.
struct P528Result {
    int mode = 0;            ///< 1 line of sight, 2 diffraction, 3 troposcatter
    int warnings = 0;        ///< P.528's warning flags (ERRORS_AND_WARNINGS.md)
    double dKm = 0.0;        ///< the path distance it used (km)
    double lossDb = 0.0;     ///< basic transmission loss A
    double freeSpaceDb = 0.0;///< free-space basic transmission loss A_fs
    double gasDb = 0.0;      ///< median atmospheric absorption A_a
    double thetaH1Rad = 0.0; ///< the ray's elevation at the low terminal
};

/// ITU-R P.528-5, Annex 2: the basic transmission loss between two terminals
/// over a smooth earth, `h1M` the lower's height and `h2M` the higher's
/// (1.5 .. 20 000 m), `dKm` the great-circle distance, 100 .. 30 000 MHz,
/// `pol` 0 horizontal or 1 vertical, `timePercent` 1 .. 99. Returns P.528's
/// code: 0 success, 11 success with warnings, else the invalid input.
int p528(double dKm, double h1M, double h2M, double fMhz, int pol, double timePercent, P528Result& out);

/// The ITM's intermediate values (IntermediateValues).
struct ItmResult {
    double lossDb = 0.0;             ///< basic transmission loss
    long warnings = 0;
    double thetaHorizonRad[2] = {0, 0};
    double horizonDistanceM[2] = {0, 0};
    double effectiveHeightM[2] = {0, 0};
    double deltaHM = 0.0;            ///< terrain irregularity
    double referenceDb = 0.0;        ///< reference attenuation A_ref
    double freeSpaceDb = 0.0;
    double dKm = 0.0;
    int mode = 0;                    ///< 1 line of sight, 2 diffraction, 3 troposcatter
};

/// The ITM point to point, with time, location and situation variability
/// (ITM_P2P_TLS_Ex): `pfl` in its PFL format - [0] the number of intervals,
/// [1] their spacing (m), then the heights (m) from transmitter to receiver.
/// Heights above the ground 0.5 .. 3000 m, 20 .. 20 000 MHz. Returns the ITM's
/// code (0 success, 1 success with warnings).
int itmP2p(double hTxM, double hRxM, const double* pfl, int climate, double n0, double fMhz, int pol, double epsilon,
           double conductivity, int mdvar, double timePercent, double locationPercent, double situationPercent, ItmResult& out);

/// P.676's gaseous attenuation along a slant path through P.835's mean annual
/// global reference atmosphere (SlantPathAttenuation): `h1Km` to `h2Km` above
/// sea level, leaving at `elevationRad`. Returns the loss (dB), NaN when the
/// ray trace fails.
double p676Slant(double fGhz, double h1Km, double h2Km, double elevationRad);

/// Whether P.676's ray traces read their layers' properties from the tables
/// tabulated per frequency (on, the default) or compute each exactly (the
/// reference implementation as published; ~50x slower).
void tabulateLayers(bool on);

/// Whether the terminals' grazing traces (TerminalGeometry) are read from their tables (on, the default, with the
/// layers') or traced each time.
void tabulateTerminals(bool on);

/// Tabulate P.676's layers for a frequency (as P.528 asks for them at `fMhz`) now, rather than in its first path.
void prepare(double fMhz);

/// P.676's specific attenuation (dB/km) of the mean annual global reference
/// atmosphere at a height (km): oxygen and water vapour.
double p676Specific(double fGhz, double hKm);

} // namespace fsim::ms::ntia
