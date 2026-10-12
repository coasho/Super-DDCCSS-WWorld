/* Private World option metadata shared by core and Scenario. */
#ifndef FSIM_NATIVE_WORLD_OPTIONS_H
#define FSIM_NATIVE_WORLD_OPTIONS_H

#include "_native_world.h"

#define WF(f, k) FSIM_PY_FIELD(fsim_world_options, f, k)
static const fsim_py_field world_fields[] = {
    WF(name, FSIM_PY_STR), WF(dt, FSIM_PY_F64), WF(frame_skip, FSIM_PY_I32), WF(workers, FSIM_PY_U32),
    WF(pin_workers, FSIM_PY_I32), WF(seed, FSIM_PY_U64), WF(capacity, FSIM_PY_U32), WF(publish, FSIM_PY_I32),
    WF(publish_interval_s, FSIM_PY_F64), WF(jsbsim_root, FSIM_PY_STR), WF(terrain, FSIM_PY_I32),
    WF(terrain_url, FSIM_PY_STR), WF(terrain_zoom, FSIM_PY_U32), WF(record_path, FSIM_PY_STR),
    WF(record_interval_s, FSIM_PY_F64), WF(terrain_offline, FSIM_PY_I32), WF(store_drag, FSIM_PY_I32)};
#undef WF

#endif /* FSIM_NATIVE_WORLD_OPTIONS_H */
