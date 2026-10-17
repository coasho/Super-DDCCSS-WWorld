#pragma once
// The RCS engine (ADR-30, MS-3): shooting and bouncing rays (Ling, Chou and
// Lee, IEEE Trans. AP-37, 1989; Bhalla and Ling, 1993) with physical optics
// radiated at every bounce, on the GPU through Vulkan ray queries.
//
// For a radar direction, a grid of ray tubes (spacing lambda / raysPerWavelength)
// covers the target's projection. Each tube is traced through up to
// maxBounces reflections off a perfect conductor (the field's tangential part
// reversed). At each hit the PO current 2 n x H of its footprint radiates back
// to the radar, integrated exactly over the footprint (a parallelogram of
// linear phase: two sinc factors). The target's faceted skin is read with its
// vertex normals as the curved surface it stands for: the current and the
// reflection follow the interpolated normal, and the radiating point sits on
// the Phong-tessellated surface. Both incident polarisations travel the same
// rays.
//
// The scattering matrix comes out in the theta-phi basis of the radar's
// direction (Table.h); sigma = |S|^2.

#include <complex>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fsim::rcs {

/// A target: triangles in its body axes (x forward, y right, z down; m), with normals at the vertices.
struct Mesh {
    std::vector<float> positions; ///< xyz per vertex
    std::vector<float> normals;   ///< xyz per vertex (unit, outwards)
    std::vector<std::uint32_t> indices;
    /// Per triangle: 0 the skin; 1 a cavity's interior (an intake's duct and engine face, a well, a recess); 2 an
    /// exhaust's metal (a nozzle's petals); 3 an exhaust's cavity (its liner and turbine). Empty: all skin.
    std::vector<std::uint8_t> kinds;
    std::size_t triangles() const noexcept { return indices.size() / 3; }
};

struct Settings {
    double raysPerWavelength = 8.0; ///< ray tubes a wavelength along each side
    int maxBounces = 5;
    /// A cavity's return: the power a ray tube carries into it comes back out diffusely (a Lambertian aperture,
    /// sigma = 4 rho A, A the tubes' cross-section), depolarised - this fraction of it.
    double cavityReflectivity = 1.0;
    /// The skin's reflection loss (dB, >= 0): a radar-absorbent coating. Each bounce's radiated current and the field
    /// it reflects on are scaled by its amplitude, 10^(-loss/20); 0 is the bare conductor.
    double skinLossDb = 0.0;
    /// The exhaust's own: its nozzle's metal (dB) and its cavity's reflectivity - treated or not apart from the rest.
    double exhaustLossDb = 0.0;
    double exhaustReflectivity = 1.0;
    double phongAlpha = 0.5;        ///< how far the radiating point moves onto the Phong surface (0: the flat facet; 0.5 follows a circle to second order)
};

/// A radar direction from the target, in its body axes: azimuth from the nose towards the right wing,
/// elevation above the body's x-y plane (towards -z).
struct Aspect {
    double azimuthRad = 0.0;
    double elevationRad = 0.0;
};

/// S[receive][transmit]: t = theta-hat, p = phi-hat. sigma_qp = |S_qp|^2 (m^2) - the coherent part; and the cavities'
/// diffuse power (m^2, both polarisations together), added incoherently: half of it co-polarised, half crossed.
struct Scattering {
    std::complex<double> tt, tp, pt, pp;
    double diffuse = 0.0;
};

class Engine {
public:
    /// A GPU with ray queries, or null (`error` says why: none, or Vulkan failed).
    static std::unique_ptr<Engine> create(std::string* error = nullptr);
    virtual ~Engine() = default;
    virtual std::string deviceName() const = 0;
    /// The target to trace (its acceleration structure built).
    virtual bool load(const Mesh& mesh, std::string* error = nullptr) = 0;
    /// The scattering matrix towards each aspect at one frequency.
    virtual std::vector<Scattering> evaluate(const std::vector<Aspect>& aspects, double frequencyHz, const Settings& settings) = 0;
    /// Rays traced since created (for the throughput).
    virtual std::uint64_t raysTraced() const = 0;
};

/// Called before each GPU dispatch (null: nothing): a batch's way to stand aside - the tool waits there while a viewer
/// draws. The dispatches and their sums are the same either way.
void setDispatchGate(void (*gate)());

/// The theta-hat and phi-hat of an aspect, and its direction (body axes).
void aspectBasis(const Aspect& a, double s[3], double theta[3], double phi[3]) noexcept;

} // namespace fsim::rcs
