#pragma once
// Antenna patterns from their aperture's illumination (ADR-30, section 5): the
// far-field pattern is the Fourier transform of the aperture distribution
// (Hankel, for a circular aperture), integrated numerically once and
// tabulated in sine space. Electronic steering moves the pattern in sine
// space, so a scanned beam broadens as it really does; an element factor
// cos^n takes the scan loss. Over the main lobe and near side lobes the
// pattern is the aperture's own; past them a floor from the card (random
// errors, edge diffraction, radome) and behind the face a back-lobe level.

#include <vector>

namespace fsim::ms::rf {

enum class ApertureShape { Rectangular, Circular };

/// The illumination across the aperture. Rectangular: Uniform, Cosine (a cosine on a pedestal), CosineSquared,
/// Taylor (n-bar, side-lobe level). Circular: Uniform, Parabolic and ParabolicSquared ((1 - r^2)^n on a pedestal).
enum class Taper { Uniform, Cosine, CosineSquared, Taylor, Parabolic, ParabolicSquared };

struct ApertureSpec {
    ApertureShape shape = ApertureShape::Rectangular;
    Taper taper = Taper::Uniform;           ///< (for a rectangle, the same in both planes)
    double widthM = 1.0;                    ///< rectangle: horizontal extent; circle: diameter
    double heightM = 1.0;                   ///< rectangle: vertical extent
    double pedestal = 0.0;                  ///< Cosine, Parabolic*: the edge's field relative to the centre's, 0 .. 1
    double taylorSidelobeDb = 30.0;         ///< Taylor: the design side-lobe level below the peak (dB, positive)
    int taylorNbar = 5;                     ///< Taylor: n-bar
    double efficiency = 1.0;                ///< losses besides the taper's (ohmic, mismatch, radome), 0 .. 1
    double peakGainDbi = 0.0;               ///< a published peak gain, used instead of the aperture's when above 0
    double floorDbi = -10.0;                ///< the side-lobe floor (relative to isotropic): random errors, structure
    double backDbi = -20.0;                 ///< behind the face
    bool electronic = false;                ///< a phased array, steered in sine space
    double scanExponent = 1.5;              ///< electronic: the element factor's cos^n (scan loss, power)
};

/// One antenna's pattern at one frequency.
class Pattern {
public:
    Pattern() = default;
    Pattern(const ApertureSpec& spec, double frequencyHz);

    /// The peak gain, dBi.
    double peakGainDbi() const noexcept { return peakDbi_; }
    /// The taper's efficiency (the illumination's directivity against a uniform one).
    double taperEfficiency() const noexcept { return taperEfficiency_; }
    /// The normalised field (amplitude) of one principal plane at `u` = (extent / wavelength) * delta sine; 1 at 0.
    double field(double u, bool vertical = false) const noexcept;
    /// The gain (dBi) towards a direction in the face's frame - x out of the face (the boresight
    /// of a mechanically pointed dish, the normal of an array), y to the right, z down - the
    /// beam steered (electronic) to the direction cosines `steerY`, `steerZ`.
    double gainDbi(const double directionFace[3], double steerY = 0.0, double steerZ = 0.0) const noexcept;
    /// The same gain, linear (no logarithm: for the many directions a radar's clutter needs).
    double gain(const double directionFace[3], double steerY = 0.0, double steerZ = 0.0) const noexcept;
    /// The half-power beamwidth (rad) of a plane at broadside, from the tabulated pattern.
    double beamwidthRad(bool vertical = false) const noexcept;
    /// The first side lobe's level below the peak (dB, negative), of a plane.
    double firstSidelobeDb(bool vertical = false) const noexcept;

private:
    struct Table {
        double extentWavelengths = 1.0; ///< the plane's extent / wavelength
        double du = 0.0;
        std::vector<double> field;      ///< |E(u)| over u = 0, du, 2 du, ... (normalised)
        double at(double u) const noexcept;
    };
    ApertureSpec spec_;
    double wavelengthM_ = 0.03;
    double peakDbi_ = 0.0;
    double peak_ = 1.0, floor_ = 0.1, back_ = 0.01; ///< linear, cached
    double taperEfficiency_ = 1.0;
    Table horizontal_, vertical_;     ///< a circle's single radial table is `horizontal_`
};

} // namespace fsim::ms::rf
