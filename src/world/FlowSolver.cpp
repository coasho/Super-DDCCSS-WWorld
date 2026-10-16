#include "world/FlowSolver.h"

#include "foundation/core/Log.h"

#include <algorithm>

#if FSIM_HAS_FLOW
// (Flow's headers as MSVC reads them: its loader converts GetProcAddress's FARPROC to void* implicitly, and PNanoVDB
// adds __forceinline to `static inline`, which MinGW defines with an inline of its own)
#include <windows.h>
#include <cstdarg>
#include <cstdio>
#define GetProcAddress(module, name) reinterpret_cast<void*>(::GetProcAddress(module, name))
#include "NvFlowLoader.h"
#undef GetProcAddress
#pragma push_macro("__forceinline")
#undef __forceinline
#define __forceinline __attribute__((always_inline))
#define PNANOVDB_C
#include "PNanoVDB.h"
#pragma pop_macro("__forceinline")
#endif

namespace fsim::world {

#if FSIM_HAS_FLOW

namespace {

void NV_FLOW_ABI flowLog(NvFlowLogLevel level, const char* format, ...) {
    char text[512];
    va_list args;
    va_start(args, format);
    std::vsnprintf(text, sizeof text, format, args);
    va_end(args);
    if (level == eNvFlowLogLevel_error) LOG_WARN("world") << "Flow: " << text;
    else if (level == eNvFlowLogLevel_warning) LOG_INFO("world") << "Flow: " << text;
}

void flowLoaderError(const char* text, void*) { LOG_WARN("world") << "Flow's loader: " << text; }

/// A NanoVDB float grid as Flow exports it: its leaves, their first voxels and values, and the index's world point
/// (its map: the solver's cell, no rotation).
struct GridRead {
    pnanovdb_buf_t buf{};
    pnanovdb_grid_handle_t grid{};
    pnanovdb_tree_handle_t tree{};
    std::uint32_t type = 0, leaves = 0;
    std::uint64_t leafOffset = 0, leafSize = 0;
    double voxel = 0.0, zero[3] = {0.0, 0.0, 0.0};
    bool open(const std::uint8_t* data, std::uint64_t bytes) {
        if (!data || bytes < PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE) return false;
        buf = pnanovdb_make_buf(const_cast<std::uint32_t*>(reinterpret_cast<const std::uint32_t*>(data)), bytes / 4u);
        grid.address = pnanovdb_address_null();
        type = pnanovdb_grid_get_grid_type(buf, grid);
        if (type != PNANOVDB_GRID_TYPE_FLOAT) return false;
        tree = pnanovdb_grid_get_tree(buf, grid);
        leaves = pnanovdb_tree_get_node_count_leaf(buf, tree);
        leafOffset = pnanovdb_tree_get_node_offset_leaf(buf, tree);
        leafSize = PNANOVDB_GRID_TYPE_GET(type, leaf_size);
        pnanovdb_vec3_t i0{0.0f, 0.0f, 0.0f}, i1{1.0f, 1.0f, 1.0f};
        const pnanovdb_vec3_t w0 = pnanovdb_grid_index_to_worldf(buf, grid, &i0), w1 = pnanovdb_grid_index_to_worldf(buf, grid, &i1);
        zero[0] = w0.x, zero[1] = w0.y, zero[2] = w0.z;
        voxel = static_cast<double>(w1.x - w0.x);
        return leaves > 0 && voxel > 0.0 && tree.address.byte_offset + leafOffset + leafSize * leaves <= bytes;
    }
    pnanovdb_leaf_handle_t leaf(std::uint32_t k) const {
        pnanovdb_leaf_handle_t h;
        h.address = pnanovdb_address_offset64(tree.address, leafOffset + leafSize * k);
        return h;
    }
    pnanovdb_coord_t origin(std::uint32_t k) const { return pnanovdb_leaf_get_bbox_min(buf, leaf(k)); }
    const float* values(std::uint32_t k) const {
        const pnanovdb_address_t a = pnanovdb_leaf_get_table_address(type, buf, leaf(k), 0u);
        return reinterpret_cast<const float*>(reinterpret_cast<const std::uint8_t*>(buf.data) + a.byte_offset);
    }
};

} // namespace

struct FlowSolver::Host {
    NvFlowLoader loader{};
    NvFlowDeviceManager* manager = nullptr;
    NvFlowDevice* device = nullptr;
    NvFlowDeviceQueue* queue = nullptr;
    NvFlowContextInterface context{};
    NvFlowGrid* grid = nullptr;
    NvFlowGridParams* params = nullptr;
    NvFlowGridSimulateLayerParams simulate = NvFlowGridSimulateLayerParams_default;
    NvFlowGridSimulateLayerParams* simulatePtr = &simulate;
    std::vector<NvFlowEmitterSphereParams> spheres;
    std::vector<NvFlowEmitterSphereParams*> spherePtrs;
    NvFlowDatabaseTypeSnapshot types[2]{};
    NvFlowUint64 version = 1;
    double clock = 0.0;      ///< the solver's own time (s): only forwards
    double gpuSeconds = 0.0; ///< the profiler's last report

    ~Host() {
        if (queue) {
            NvFlowContext* c = loader.deviceInterface.getContext(queue);
            loader.deviceInterface.waitIdle(queue);
            if (grid) loader.gridInterface.destroyGrid(c, grid);
            if (params) loader.gridParamsInterface.destroyGridParams(params);
            NvFlowUint64 flushed = 0;
            loader.deviceInterface.flush(queue, &flushed, nullptr, nullptr);
            loader.deviceInterface.waitIdle(queue);
        }
        if (device) loader.deviceInterface.destroyDevice(manager, device);
        if (manager) loader.deviceInterface.destroyDeviceManager(manager);
        if (loader.module_nvflow || loader.module_nvflowext) NvFlowLoaderDestroy(&loader);
    }
    static void NV_FLOW_ABI profile(void* userdata, NvFlowUint64, NvFlowUint count, NvFlowProfilerEntry* entries) {
        double s = 0.0;
        for (NvFlowUint i = 0; i < count; ++i) s += static_cast<double>(entries[i].gpuDeltaTime);
        static_cast<Host*>(userdata)->gpuSeconds = s;
    }
    bool start(const Settings& settings) {
        NvFlowLoaderInit(&loader, flowLoaderError, nullptr);
        if (!loader.module_nvflow || !loader.module_nvflowext) return false;
        manager = loader.deviceInterface.createDeviceManager(NV_FLOW_FALSE, nullptr, 0u);
        if (!manager) return false;
        NvFlowDeviceDesc desc{};
        desc.deviceIndex = 0u;
        desc.enableExternalUsage = NV_FLOW_FALSE; // (the readback first; interop later)
        desc.logPrint = flowLog;
        device = loader.deviceInterface.createDevice(manager, &desc);
        if (!device) return false;
        queue = loader.deviceInterface.getDeviceQueue(device);
        NvFlowContextInterface_duplicate(&context, loader.deviceInterface.getContextInterface(queue));
        NvFlowContext* c = loader.deviceInterface.getContext(queue);
        NvFlowGridDesc gridDesc = NvFlowGridDesc_default;
        grid = loader.gridInterface.createGrid(&context, c, loader.opList_orig, loader.extOpList_orig, &gridDesc);
        params = loader.gridParamsInterface.createGridParams();
        if (!grid || !params) return false;
        loader.deviceInterface.enableProfiler(c, this, profile);

        NvFlowGridSimulateLayerParams& p = simulate;
        p.densityCellSize = static_cast<float>(settings.cellM);
        p.stepsPerSecond = static_cast<float>(settings.stepsPerSecond);
        p.maxStepsPerSimulate = 2u;
        p.nanoVdbExport.enabled = NV_FLOW_TRUE;
        p.nanoVdbExport.readbackEnabled = NV_FLOW_TRUE;
        p.nanoVdbExport.temperatureEnabled = NV_FLOW_TRUE;
        p.nanoVdbExport.smokeEnabled = NV_FLOW_TRUE;
        p.nanoVdbExport.fuelEnabled = settings.exportFuel ? NV_FLOW_TRUE : NV_FLOW_FALSE;
        p.nanoVdbExport.statisticsEnabled = NV_FLOW_FALSE;
        p.advection.gravity = NvFlowFloat3{0.0f, 0.0f, static_cast<float>(-settings.gravity)};
        const Combustion& k = settings.combustion;
        p.advection.ignitionTemp = static_cast<float>(k.ignition);
        p.advection.burnPerTemp = static_cast<float>(k.burnPerTemp);
        p.advection.fuelPerBurn = static_cast<float>(k.fuelPerBurn);
        p.advection.tempPerBurn = static_cast<float>(k.tempPerBurn);
        p.advection.smokePerBurn = static_cast<float>(k.smokePerBurn);
        p.advection.divergencePerBurn = static_cast<float>(k.divergencePerBurn);
        p.advection.buoyancyPerTemp = static_cast<float>(k.buoyancyPerTemp);
        p.advection.coolingRate = static_cast<float>(k.coolingRate);
        p.advection.smoke.fade = static_cast<float>(k.smokeFade);
        p.advection.smoke.damping = static_cast<float>(k.smokeDamping);
        p.advection.velocity.damping = static_cast<float>(k.velocityDamping);
        p.summaryAllocate.speedThreshold = static_cast<float>(k.keepSpeed);
        p.vorticity.forceScale = static_cast<float>(k.vorticity);
        return true;
    }
};

FlowSolver::FlowSolver() = default;
FlowSolver::~FlowSolver() = default;

bool FlowSolver::start(const Settings& settings) {
    settings_ = settings;
    auto host = std::make_unique<Host>();
    if (!host->start(settings)) return false;
    host_ = std::move(host);
    return true;
}

bool FlowSolver::running() const noexcept { return host_ != nullptr; }
double FlowSolver::gpuMs() const noexcept { return host_ ? 1000.0 * host_->gpuSeconds : 0.0; }

void FlowSolver::step(double dt, const std::vector<Sphere>& sources, bool clear, bool wait) {
    if (!host_ || !(dt > 0.0)) return;
    Host& h = *host_;
    const auto F = [](double v) { return static_cast<float>(v); };
    h.spheres.clear();
    for (const Sphere& s : sources) {
        NvFlowEmitterSphereParams e = NvFlowEmitterSphereParams_default;
        e.luid = s.luid;
        e.position = NvFlowFloat3{F(s.at[0]), F(s.at[1]), F(s.at[2])};
        e.radius = F(s.radius);
        e.velocity = NvFlowFloat3{F(s.velocity[0]), F(s.velocity[1]), F(s.velocity[2])};
        e.divergence = F(s.divergence);
        e.temperature = F(s.temperature);
        e.fuel = F(s.fuel);
        e.smoke = F(s.smoke);
        e.burn = 0.0f;
        e.coupleRateVelocity = F(s.velocityRate);
        e.coupleRateDivergence = F(s.divergenceRate);
        e.coupleRateTemperature = F(s.temperatureRate);
        e.coupleRateFuel = F(s.fuelRate);
        e.coupleRateSmoke = F(s.smokeRate);
        e.coupleRateBurn = 0.0f;
        if (s.traceSamples > 0) {
            e.numTraceSamples = s.traceSamples;
            e.traceDeltaTime = F(1.0 / settings_.stepsPerSecond); // (the solver's step: it steps at its own rate)
        }
        h.spheres.push_back(e);
    }
    h.spherePtrs.resize(h.spheres.size());
    for (std::size_t i = 0; i < h.spheres.size(); ++i) h.spherePtrs[i] = &h.spheres[i];
    ++h.version;
    const NvFlowBool32 forceClear = clear ? NV_FLOW_TRUE : NV_FLOW_FALSE;
    h.simulate.forceClear = forceClear;
    h.types[0] = NvFlowDatabaseTypeSnapshot{h.version, &NvFlowGridSimulateLayerParams_NvFlowReflectDataType,
                                            reinterpret_cast<NvFlowUint8**>(&h.simulatePtr), 1u};
    h.types[1] = NvFlowDatabaseTypeSnapshot{h.version, &NvFlowGridEmitterSphereParams_NvFlowReflectDataType,
                                            reinterpret_cast<NvFlowUint8**>(h.spherePtrs.data()), h.spherePtrs.size()};
    h.clock += dt;
    NvFlowGridParamsDescSnapshot snapshot{NvFlowDatabaseSnapshot{h.version, h.types, 2u}, h.clock, F(dt), forceClear, nullptr, 0u};
    h.loader.gridParamsInterface.commitParams(h.params, &snapshot);

    NvFlowContext* c = h.loader.deviceInterface.getContext(h.queue);
    NvFlowGridParamsDesc desc{};
    NvFlowGridParamsSnapshot* taken = h.loader.gridParamsInterface.getParamsSnapshot(h.params, h.clock, 0llu);
    NvFlowGridRenderData render{};
    if (h.loader.gridParamsInterface.mapParamsDesc(h.params, taken, &desc)) {
        h.loader.gridInterface.simulate(c, h.grid, &desc, NV_FLOW_FALSE);
        h.loader.gridInterface.getRenderData(c, h.grid, &render);
        h.loader.gridParamsInterface.unmapParamsDesc(h.params, taken);
    }
    NvFlowUint64 flushed = 0;
    h.loader.deviceInterface.flush(h.queue, &flushed, nullptr, nullptr);
    if (wait) h.loader.deviceInterface.waitForFrame(h.queue, flushed);
    const NvFlowUint64 completed = h.context.getLastFrameCompleted(c);

    // the newest finished readback
    const NvFlowGridRenderDataNanoVdbReadback* readback = nullptr;
    for (NvFlowUint64 i = render.nanoVdb.readbackCount; i-- > 0;)
        if (completed >= render.nanoVdb.readbacks[i].globalFrameCompleted) {
            readback = &render.nanoVdb.readbacks[i];
            break;
        }
    fields_.leaves.clear();
    fields_.bytes = 0;
    GridRead smoke, heat, fuel;
    if (!readback || !smoke.open(readback->smokeNanoVdbReadback, readback->smokeNanoVdbReadbackSize) ||
        !heat.open(readback->temperatureNanoVdbReadback, readback->temperatureNanoVdbReadbackSize) || smoke.leaves != heat.leaves)
        return;
    const bool withFuel = settings_.exportFuel && fuel.open(readback->fuelNanoVdbReadback, readback->fuelNanoVdbReadbackSize) &&
                          fuel.leaves == smoke.leaves;
    fields_.bytes = readback->smokeNanoVdbReadbackSize + readback->temperatureNanoVdbReadbackSize + (withFuel ? readback->fuelNanoVdbReadbackSize : 0u);
    fields_.frame = readback->globalFrameCompleted;
    fields_.voxel = smoke.voxel;
    std::copy(smoke.zero, smoke.zero + 3, fields_.zero);
    // the leaves with anything in them (the grids' leaves alike: Flow writes them from one layout)
    for (std::uint32_t k = 0; k < smoke.leaves; ++k) {
        const pnanovdb_coord_t o = smoke.origin(k), oh = heat.origin(k);
        if (o.x != oh.x || o.y != oh.y || o.z != oh.z) continue;
        const float* s = smoke.values(k);
        const float* t = heat.values(k);
        const float* f = withFuel ? fuel.values(k) : nullptr;
        bool any = false;
        for (std::uint32_t n = 0; n < 512u && !any; ++n) any = s[n] > 1e-4f || t[n] > 1e-4f || (f && f[n] > 1e-4f);
        if (any) fields_.leaves.push_back(Leaf{{o.x, o.y, o.z}, s, t, f});
    }
}

#else

struct FlowSolver::Host {};
FlowSolver::FlowSolver() = default;
FlowSolver::~FlowSolver() = default;
bool FlowSolver::start(const Settings& settings) {
    settings_ = settings;
    return false;
}
bool FlowSolver::running() const noexcept { return false; }
double FlowSolver::gpuMs() const noexcept { return 0.0; }
void FlowSolver::step(double, const std::vector<Sphere>&, bool, bool) {}

#endif

} // namespace fsim::world
