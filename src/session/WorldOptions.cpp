#include "session/WorldOptions.h"

#include "fsim/World.h"
#include "session/World.h"

namespace fsim::session {

WorldOptions makeWorldOptions(const fsim::WorldOptions& options) {
    // These exact-arity decompositions fail to compile when either aggregate gains a field. Do not replace them
    // with a partial list of member assignments: a new option must not silently inherit an unrelated default.
    const auto& [name, dt, frameSkip, workers, pinWorkers, seed, capacity, publish, publishIntervalSeconds, jsbsimRoot,
                 terrain, terrainUrl, terrainZoom, terrainOffline, storeDrag, ground, recordPath, recordIntervalSeconds] = options;
    WorldOptions result;
    auto& [nativeName, nativeDt, nativeFrameSkip, nativeWorkers, nativePinWorkers, nativeSeed, nativeCapacity,
           nativePublish, nativeStoreDrag, nativePublishIntervalSeconds, nativeJsbsimRoot, nativeGround, nativeTerrain,
           nativeTerrainUrl, nativeTerrainZoom, nativeTerrainPrefetchRadiusM, nativeTerrainOffline, nativeRecordPath,
           nativeRecordIntervalSeconds] = result;
    nativeName = name;
    nativeDt = dt;
    nativeFrameSkip = frameSkip;
    nativeWorkers = workers;
    nativePinWorkers = pinWorkers;
    nativeSeed = seed;
    nativeCapacity = capacity;
    nativePublish = publish;
    nativePublishIntervalSeconds = publishIntervalSeconds;
    nativeJsbsimRoot = jsbsimRoot;
    nativeTerrain = terrain;
    nativeTerrainUrl = terrainUrl;
    nativeTerrainZoom = terrainZoom;
    nativeTerrainOffline = terrainOffline;
    nativeStoreDrag = storeDrag;
    nativeGround = ground;
    nativeRecordPath = recordPath;
    nativeRecordIntervalSeconds = recordIntervalSeconds;
    // This native-only performance option has no public input. Preserve the session default rather than exposing it
    // through an unrelated lifecycle or presentation setting.
    (void)nativeTerrainPrefetchRadiusM;
    return result;
}

} // namespace fsim::session
