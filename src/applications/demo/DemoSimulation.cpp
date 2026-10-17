#include "applications/demo/DemoSimulation.h"

#include "applications/demo/DemoAutopilot.h"

#include "foundation/core/Log.h"
#include "foundation/core/Rng.h"
#include "foundation/core/Units.h"
#include "foundation/io/AssetResolver.h"
#include "foundation/io/TerrainTiles.h"
#include "foundation/platform/Threads.h"
#include "simulation/flight/GroundProvider.h"
#include "simulation/flight/JsbsimModel.h"
#include "simulation/flight/SimRunner.h"
#include "simulation/flight/VehiclePool.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace fsim::app {

struct DemoSimulation::Impl {
    std::unique_ptr<sim::SimRunner> runner;
};

DemoSimulation::DemoSimulation() : impl_(std::make_unique<Impl>()) {}
DemoSimulation::~DemoSimulation() = default;

std::unique_ptr<DemoSimulation> DemoSimulation::create(const DemoOptions& opt, io::AssetResolver& assets,
                                                       const std::shared_ptr<io::TerrainTiles>& terrain) {
    const auto root = assets.jsbsimRoot(opt.jsbsimRoot);
    if (!root) {
        LOG_ERROR("app") << "JSBSim data root not found; pass --jsbsim-root <dir>";
        return nullptr;
    }
    std::shared_ptr<sim::GroundProvider> ground;
    if (terrain) {
        terrain->prefetch(units::degreesToRadians(opt.latitudeDeg), units::degreesToRadians(opt.longitudeDeg), 1500.0 + opt.spreadDeg * 111000.0);
        ground = terrain;
    } else {
        ground = std::make_shared<sim::FlatGround>(0.0);
    }
    const unsigned physical = platform::physicalCoreCount();
    const unsigned workers = std::min<unsigned>(opt.workers ? opt.workers : std::max(1u, physical > 3 ? physical - 3 : 1u), opt.vehicles);
    auto pool = std::make_unique<sim::VehiclePool>(workers);
    const sim::AircraftSpec aircraft{opt.aircraft, *root, assets.findAircraft(opt.aircraft, *root)};
    auto initialConditions = std::make_shared<std::vector<sim::InitialConditions>>();
    for (unsigned i = 0; i < opt.vehicles; ++i) {
        Rng rng = Rng::forVehicle(opt.seed, 0, i);
        sim::InitialConditions ic;
        ic.latitudeDeg = opt.latitudeDeg + rng.uniform(-opt.spreadDeg, opt.spreadDeg);
        ic.longitudeDeg = opt.longitudeDeg + rng.uniform(-opt.spreadDeg, opt.spreadDeg);
        ic.headingDeg = rng.uniform(0.0, 360.0);
        if (opt.onGround) {
            ic.onGround = true;
            ic.airspeedTrueMs = 0.0;
        } else {
            const double terrainM = ground->heightAboveEllipsoidM(units::degreesToRadians(ic.latitudeDeg), units::degreesToRadians(ic.longitudeDeg));
            ic.altitudeMslM = std::max(opt.altitudeM + rng.uniform(-150.0, 150.0), terrainM + 300.0);
            ic.airspeedTrueMs = opt.speedMs > 0.0 ? opt.speedMs : 58.0 + rng.uniform(-4.0, 4.0);
        }
        auto model = std::make_unique<sim::JsbsimModel>(opt.dt, ground);
        if (!model->load(aircraft, ic)) {
            LOG_ERROR("app") << "vehicle " << i << " failed to load";
            return nullptr;
        }
        if (!opt.onGround && opt.speedMs <= 0.0) {
            // a fighter would start below its stall at 58 m/s: fly off where
            // the aircraft is comfortable, its weight carried at CL 0.5
            const double weightLbf = model->property("inertia/weight-lbs").get();
            const double areaFt2 = model->property("metrics/Sw-sqft").get();
            const double rhoSlugFt3 = model->property("atmosphere/rho-slugs_ft3").get();
            if (weightLbf > 0.0 && areaFt2 > 0.0 && rhoSlugFt3 > 0.0) {
                const double v = 0.3048 * std::sqrt(2.0 * weightLbf / (areaFt2 * rhoSlugFt3 * 0.5));
                if (v > ic.airspeedTrueMs) {
                    ic.airspeedTrueMs = v * (1.0 + rng.uniform(-0.05, 0.05));
                    model->reset(ic);
                }
            }
        }
        pool->add(std::move(model));
        initialConditions->push_back(ic);
    }
    auto autopilot = std::make_shared<DemoAutopilot>(opt.vehicles, opt.throttle, opt.onGround);
    std::unique_ptr<DemoSimulation> demo(new DemoSimulation());
    demo->impl_->runner = std::make_unique<sim::SimRunner>(std::move(pool), opt.frameSkip,
        [autopilot, initialConditions](const sim::SnapshotBatch& prev, sim::VehiclePool& vehicles, std::vector<sim::ControlInputs>& out) {
            for (std::size_t i = 0; i < prev.states.size(); ++i) {
                if (!prev.states[i].diverged) continue;
                LOG_WARN("app") << "vehicle " << i << " diverged; resetting to its initial conditions";
                vehicles.vehicle(i).reset((*initialConditions)[i]);
                autopilot->forget(i);
            }
            autopilot->compute(prev.states, out);
        });
    demo->impl_->runner->setTimeFactor(opt.timeFactor);
    LOG_INFO("app") << "demo: " << opt.vehicles << " x " << opt.aircraft << " on " << workers << " worker(s)";
    return demo;
}

std::size_t DemoSimulation::vehicles() const noexcept { return impl_->runner->vehicles(); }

void DemoSimulation::cgFromOriginBodyM(std::size_t i, double* out) const {
    impl_->runner->pool().vehicle(i).cgFromOriginBodyM(out);
}

void DemoSimulation::start() { impl_->runner->start(); }
void DemoSimulation::stop() { impl_->runner->stop(); }
void DemoSimulation::setPaused(bool paused) noexcept { impl_->runner->setPaused(paused); }
void DemoSimulation::setTimeFactor(double factor) noexcept { impl_->runner->setTimeFactor(factor); }
void DemoSimulation::singleStep() noexcept { impl_->runner->singleStep(); }
const sim::SnapshotBatch* DemoSimulation::acquire() noexcept { return impl_->runner->snapshots().acquire(); }
double DemoSimulation::throughput() const noexcept { return impl_->runner->throughput(); }

} // namespace fsim::app
