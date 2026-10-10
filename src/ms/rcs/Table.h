#pragma once
// An aircraft type's radar cross section (ADR-30, MS-3): the second moments
// of its scattering matrix over aspect bins and radar bands, computed once
// by tools/rcs (shooting and bouncing rays with physical optics over the
// hangar's model, on the GPU) and looked up per look.
//
// Aspect: the radar's direction from the aircraft in its body axes (x
// forward, y right, z down), as an azimuth from the nose towards the right
// wing (0 .. 360 deg) and an elevation above the body's x-y plane (-90 .. 90
// deg; positive: the radar above, on the -z side). Polarisation: in the
// spherical basis of that direction - theta-hat along increasing elevation's
// opposite (towards body +z at the horizon), phi-hat along increasing azimuth.
//
// Each bin holds the means over its sub-aspects (and both mirror sides where
// the aircraft is symmetric) of |S_tt|^2, |S_pp|^2, |S_tp|^2 and the
// cross moments S_tt S_pp*, S_tt S_tp*, S_pp S_tp* (m^2): what the mean RCS
// in any linear polarisation follows from. And how much the RCS fluctuates
// within the bin: E[sigma^2] / E[sigma]^2 for theta-theta (1 steady, 2 a
// Rayleigh-distributed sum of many scatterers: Swerling 1/2).

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace fsim::ms::rcs {

/// One bin's moments (m^2), in the theta-phi basis.
struct Moments {
    float tt = 0, pp = 0, tp = 0;              ///< E|S_tt|^2, E|S_pp|^2, E|S_tp|^2
    float ttppRe = 0, ttppIm = 0;              ///< E[S_tt S_pp*]
    float tttpRe = 0, tttpIm = 0;              ///< E[S_tt S_tp*]
    float pptpRe = 0, pptpIm = 0;              ///< E[S_pp S_tp*]
    float fluctuation = 1;                     ///< E[sigma_tt^2] / E[sigma_tt]^2 over the bin's samples
};

struct Band {
    double frequencyHz = 0.0;
    std::vector<Moments> bins; ///< [elevation][azimuth]
};

/// The table: bins of `binDeg` - azimuths 0 .. 360 (wrapping), elevations -90 .. 90 at the bins' centres.
struct Table {
    std::string aircraft;
    double binDeg = 3.0;
    int azimuthBins = 120, elevationBins = 61;
    std::vector<Band> bands;
    std::string provenance; ///< JSON: the method, its settings, the model it was computed from

    /// The mean RCS (m^2) towards a direction in the body axes (need not be unit), at the band nearest
    /// the frequency (log-interpolated between bands), the radar transmitting and receiving linear
    /// polarisations at `transmitRad` and `receiveRad` from theta-hat towards phi-hat.
    double sigma(const double directionBody[3], double frequencyHz, double transmitRad, double receiveRad) const noexcept;
    /// The bin's fluctuation ratio there (see above).
    double fluctuation(const double directionBody[3], double frequencyHz) const noexcept;
    /// The bin towards a body direction: azimuth and elevation indices.
    void bin(const double directionBody[3], int& azimuth, int& elevation) const noexcept;
    const Moments& at(std::size_t band, int azimuth, int elevation) const noexcept {
        return bands[band].bins[static_cast<std::size_t>(elevation * azimuthBins + azimuth)];
    }
};

/// The mean RCS a bin's moments give for transmit and receive polarisation angles.
double sigmaOf(const Moments& m, double transmitRad, double receiveRad) noexcept;

/// The binary table (aircraft/<type>/rcs.bin): read; null with `error` set.
std::shared_ptr<const Table> readTable(const std::filesystem::path& file, std::string* error = nullptr);
bool writeTable(const Table& table, const std::filesystem::path& file, std::string* error = nullptr);

} // namespace fsim::ms::rcs
