#pragma once

// NVIDIA Flow 2's solver alone, nothing drawn (MS-7h): its loader, its own Vulkan device, one grid and its parameters;
// sphere sources in, the newest finished smoke, temperature and fuel out (NanoVDB grids read back, as leaves of 8^3
// voxels). What world::FlowVolumes draws from and tools/flow_calibrate measures. Built only with FSIM_FLOW_DIR
// (cmake/Flow.cmake); without it start() is false.
//
// Flow's combustion, per cell (its AdvectionCommon.hlsli): where the temperature (normalized: 0 the air's, held at
// most 1) is above `ignition` and there is fuel, it burns burnPerTemp (T - ignition) a second - at most what the fuel
// allows - consuming fuelPerBurn, heating tempPerBurn, making smokePerBurn of smoke and divergencePerBurn of expansion
// per unit burnt; every cell cools by coolingRate T a second; the hot gas rises by buoyancyPerTemp T against gravity.
// There is no oxygen: fuel burns wherever it is hot enough.

#include <cstdint>
#include <memory>
#include <vector>

namespace fsim::world {

class FlowSolver {
public:
    struct Combustion {
        double ignition = 0.05, burnPerTemp = 4.0, fuelPerBurn = 0.25, tempPerBurn = 5.0, smokePerBurn = 3.0;
        double divergencePerBurn = 0.0, buoyancyPerTemp = 2.0, coolingRate = 1.5;
        double smokeFade = 0.0, smokeDamping = 0.0; ///< a second: absolute, and the share (the solver's advection)
        double velocityDamping = 0.01;              ///< the velocity's share lost a second (Flow's default)
        double keepSpeed = 1.0;                     ///< a block without smoke kept while its gas moves faster (m/s; Flow's default)
        double vorticity = 0.6;                     ///< its confinement's force scale (Flow's default 0.6)
    };
    struct Settings {
        double cellM = 0.5;          ///< the density's voxel (m)
        double stepsPerSecond = 60.0;
        double gravity = 9.81;       ///< m/s^2, down the local frame's z
        Combustion combustion;
        bool exportFuel = false;     ///< read the fuel back too (the calibration's)
    };
    /// A sphere source, for one step (the local frame: m, m/s). Each value the cells inside are drawn towards at
    /// `rate` a second (Flow's couple rates; 0: that value left alone).
    struct Sphere {
        double at[3] = {0.0, 0.0, 0.0};
        double radius = 1.0;
        double velocity[3] = {0.0, 0.0, 0.0};
        double divergence = 0.0, temperature = 0.0, fuel = 0.0, smoke = 0.0;
        double velocityRate = 0.0, divergenceRate = 0.0, temperatureRate = 0.0, fuelRate = 0.0, smokeRate = 0.0;
        std::uint64_t luid = 0;      ///< the same source's the same, step to step
        unsigned traceSamples = 0;   ///< a fast source's path sampled between steps (Flow's trace)
    };
    /// A leaf of the fields: its first voxel's index, its 512 values a field (x slowest, z fastest; null: not read).
    struct Leaf {
        int origin[3];
        const float* smoke;
        const float* temperature;
        const float* fuel;
    };
    struct Fields {
        std::vector<Leaf> leaves;    ///< those with anything in them
        double voxel = 0.0;          ///< m
        double zero[3] = {0.0, 0.0, 0.0}; ///< index (0, 0, 0)'s point (the local frame)
        std::uint64_t bytes = 0;     ///< read back
        std::uint64_t frame = 0;     ///< the solver's frame they are of
    };

    FlowSolver();
    ~FlowSolver();
    FlowSolver(const FlowSolver&) = delete;
    FlowSolver& operator=(const FlowSolver&) = delete;

    /// Loads Flow and makes its device and grid; false: no Flow (not built with it, or its DLLs not found).
    bool start(const Settings& settings);
    bool running() const noexcept;
    const Settings& settings() const noexcept { return settings_; }
    /// One call of the solver: `dt` s (it steps at stepsPerSecond, up to two steps a call) with these sources;
    /// `clear`: the grid emptied first; `wait`: until the GPU is done, the step's own fields then read (else the
    /// newest finished, a frame or two old).
    void step(double dt, const std::vector<Sphere>& sources, bool clear = false, bool wait = false);
    /// The newest finished fields (valid until the next step).
    const Fields& fields() const noexcept { return fields_; }
    /// The solver's GPU time in its last finished frame (ms; its profiler's).
    double gpuMs() const noexcept;

private:
    struct Host;
    std::unique_ptr<Host> host_;
    Settings settings_;
    Fields fields_;
};

} // namespace fsim::world
