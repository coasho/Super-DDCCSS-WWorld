#pragma once

// The engagement's fire and smoke simulated as a fluid (MS-7h, after the owner rejected fireballs pieced from spheres,
// 2026-10-09): NVIDIA Flow 2 (BSD-3; NVIDIA-Omniverse/PhysX's flow/), a real-time sparse-voxel combustion solver, run
// on its own Vulkan device on the same GPU (world::FlowSolver), its sources the damage chain's events held to what is
// observed of fires (world/FlowFires.h: CCPS's fireballs, the photographed detonations, API RP 521's jet flames).
//
// Each frame the solver's smoke and temperature come back as NanoVDB leaves (8^3 voxels; read back to the CPU - GPU
// interop later drops the copy), are packed as bricks into a 3D texture (each with its neighbours' next voxels, so
// the hardware's trilinear filter is the field's), found by a hash of their place, and drawn page by page (8^3 leaves,
// 32 m: each a box, back to front) - ray-marched as radiative transfer through soot with the world's own sun, sky and
// aerial perspective: soot absorbing and scattering little (its albedo), and emitting Planck's light for its
// temperature where it is hot, against the sun's disk as world::Detonations reckons its glow.
//
// The solver works in the air's frame (east, north, up; metres) round the first event's point, drifting with the
// wind: single precision would not hold an ECEF point, and the wind carries the smoke without the solver's help.
// Built only with FSIM_FLOW_DIR (cmake/Flow.cmake); without it, or without its DLLs at run time, available() is false.

#include "core/Span.h"
#include "ipc/WorldLayout.h"
#include "sim/VehicleState.h"
#include "world/Blackbody.h"
#include "world/Clouds.h"
#include "world/Detonations.h"
#include "world/FlowSolver.h"
#include "world/VehicleVisuals.h"

#include <vsg/all.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace fsim::world {

class FlowVolumes {
public:
    /// `coarseShading`: the device can shade a draw at a coarser rate than its pixels (render::Viewer::coarseShading).
    FlowVolumes(vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders, const AtmosphereTables& tables,
                bool coarseShading = false);
    FlowVolumes(const FlowVolumes&) = delete;
    FlowVolumes& operator=(const FlowVolumes&) = delete;

    /// The solver runs and its draw built.
    bool available() const noexcept { return solver_.running() && static_cast<bool>(draw_); }
    vsg::ref_ptr<vsg::Node> node() const { return root_; }
    void setVisible(bool on);
    /// The window's size (px): a pixel's angle, and whether to shade coarser.
    void setScreen(std::uint32_t width, std::uint32_t height);
    /// The air the fires burn in (its sea level's, the wind that carries the smoke: Detonations::Air).
    void setAir(const Detonations::Air& air) noexcept { air_ = air; }
    /// A slot's aircraft: its span and mass (m, kg; its wake's vortex pair, laid while it burns).
    void setAircraft(std::size_t slot, double wingspanM, double massKg);

    /// An engagement's event: a detonation (its charge), a fire (the fuel its holed tank spills at once, and what leaks
    /// and burns on from its component, as it changes). Other kinds are not this one's.
    void add(const ipc::EngagementEvent& event, const vsg::EllipsoidModel& ellipsoid);
    /// Everything forgotten (a replay's seek, its loop): the grid cleared.
    void clear();
    /// Per frame, after VehicleVisuals::update(): the world's time (s), the vehicles as drawn (their burning
    /// components' places), the camera (projection, the eye ECEF), the ellipsoid and the sun (ECEF). The solver steps
    /// to `now`, its newest finished fields packed and uploaded.
    void update(double now, Span<const sim::VehicleState> states, const std::vector<unsigned char>& alive, const VehicleVisuals& visuals,
                const vsg::dmat4& projection, const vsg::dvec3& eye, const vsg::EllipsoidModel& ellipsoid, const vsg::dvec3& sunEcef);

    /// What the last update cost: the solver's GPU time (ms, its own queue's, its profiler's), the CPU's in update
    /// (ms: the solver's recording, the leaves' packing), the leaves drawn (of those read back), the pages.
    struct Cost {
        double solverGpuMs = 0.0, cpuMs = 0.0, packMs = 0.0;
        std::uint64_t leaves = 0, leavesRead = 0, pages = 0, readbackBytes = 0;
    };
    const Cost& cost() const noexcept { return cost_; }

    static constexpr double kCellM = 0.5;               ///< the solver's voxel (its calibration's: world/FlowFires.h)
    static constexpr std::uint32_t kBricks = 16;        ///< the atlas's bricks a side: 4,096 leaves drawn at most
    static constexpr std::uint32_t kBrick = 9;          ///< a brick's texels a side: its leaf's 8 and the next voxel
    static constexpr std::uint32_t kHash = 8192;        ///< the leaves' hash table's entries
    static constexpr std::uint32_t kPages = 1024;       ///< pages drawn at most
    static constexpr double kSootAlbedo = 0.2;          ///< (Detonations::kSootAlbedo)
    /// The flame's temperature fluctuating below the solver's cells: Blackbody's kFlicker (0.2 rms, measured in large
    /// hydrocarbon fires), its mean lowered so its mean emissive power is CCPS's; its pattern the clouds' noise, eddies
    /// a few metres across (the noise's tile kFlickerM: an estimate), carried by the air and evolving as the flame rises.
    static constexpr double kFlickerM = 12.0;
    /// The rate its smoke thins at (1/s, an e-fold: FlowFires' kSmokeDilution) - where the far trail (world::Airflow's
    /// fire trail, setFireHandOver) takes it over.
    static double nearFieldThinning();

private:
    struct Burst {
        bool detonation = false;
        double time = 0.0, amountKg = 0.0, pressurePa = 101325.0;
        double at[3] = {0.0, 0.0, 0.0};    ///< where (the air's frame, at its time)
        double motion[3] = {0.0, 0.0, 0.0}; ///< a detonation's products' drift, a spill's hole's path (the air's frame)
        std::uint64_t luid = 0;
    };
    struct Leak {
        std::size_t slot = 0;
        std::string component;
        double rateKgS = 0.0, since = 0.0;
        std::uint64_t luid = 0;
    };
    /// ECEF to the air's frame at time t (m), and a velocity (ECEF, m/s) to the air's frame (m/s)
    vsg::dvec3 toAir(const vsg::dvec3& ecef, double t) const;
    vsg::dvec3 velocityToAir(const vsg::dvec3& ecef) const;
    vsg::dvec3 windLocal() const;
    void origin(const vsg::dvec3& at, double t, const vsg::EllipsoidModel& ellipsoid);
    void pack(const vsg::dvec3& eyeLocal);

    FlowSolver solver_;
    Detonations::Air air_;
    std::vector<Burst> bursts_;
    std::vector<Leak> leaks_;
    std::vector<vsg::dvec2> aircraft_; ///< by slot: span (m), mass (kg)
    std::vector<FlowSolver::Sphere> sources_;
    bool haveOrigin_ = false, cleared_ = true;
    vsg::dvec3 origin_;            ///< the frame's origin (ECEF), at originTime_
    vsg::dvec3 east_, north_, up_; ///< its axes (ECEF)
    double originTime_ = 0.0, lastTime_ = -1.0;
    std::uint64_t nextLuid_ = 1;
    Cost cost_;
    std::unordered_map<std::uint64_t, std::uint32_t> slots_; ///< a leaf's brick (by its place)

    vsg::ref_ptr<vsg::Group> root_;
    vsg::ref_ptr<vsg::Switch> switch_;
    vsg::ref_ptr<vsg::MatrixTransform> at_;
    vsg::ref_ptr<vsg::Draw> draw_;
    vsg::ref_ptr<vsg::vec4Array> rows_, frame_;
    vsg::ref_ptr<vsg::uivec4Array> hash_;       ///< (x, y, z, brick + 1) of each leaf drawn; 0 in w: none
    vsg::ref_ptr<vsg::usvec2Array3D> atlas_;    ///< smoke, temperature (half floats; the solver's units)
    vsg::ref_ptr<vsg::Command> shadingRate_;
    SunlitBlackbody blackbody_;
    bool visible_ = true;
    std::uint32_t screenHeight_ = 1080, coarse_ = 1;
};

} // namespace fsim::world
