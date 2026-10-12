/* Private support for fsim._native and its DMS/C2 adapter.
 * The core retains World/Error ownership and all Python type registration. */
#ifndef FSIM_NATIVE_WORLD_H
#define FSIM_NATIVE_WORLD_H

#include "fsim_py.h"
#include "fsim/fsim_c.h"

typedef struct {
    PyObject_HEAD
    fsim_world* world;
    PyObject* owner; /* the VecEnv a borrowed world belongs to; an agent's handle: the range's World */
    int agent;       /* ADR-31: an agent's handle (fsim_agent_create), freed with it */
    int successor;   /* AB-2d: the handle a seal gave (fsim_world_seal), freed with it */
    int own_busy;
    int* busy;
    fsim_py_ref ref;
} WorldObject;

#define WORLD_IDLE(self) fsim_py_idle((self)->busy, "this World")

/* Shared conversions and the bridge to the core's original static Error object. */
PyObject* fail(void);
int check_args(Py_ssize_t n, Py_ssize_t lo, Py_ssize_t hi, const char* name);
int as_u32(PyObject* o, uint32_t* out);
int as_u64(PyObject* o, uint64_t* out);
const char* as_str(PyObject* o, const char* what);
int as_doubles(PyObject* const* args, Py_ssize_t from, Py_ssize_t count, double* out);

/* These handlers are registered only by _native.c's existing World method table. */
PyObject* world_create_c2_node(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_remove_c2_node(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_create_net(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_join_net(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_leave_net(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_assign_system(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dms_system(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dms_nets(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dms_send(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dms_inbox_take(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dms_subscribe(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dms_subscribe_all(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dms_subscriptions(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dms_inbox(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dms_transmission(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_system_entity(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_entity_system(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_net_count(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_net_stats(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_strike_policy(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_strike_policy(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_agent_c2_tasks(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_agent_answer_task(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_agent_overrides(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_c2_send_task(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_c2_set_override(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_c2_consents(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_c2_answer_consent(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_rbac(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_rbac(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_agent_authority(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_agent_comms(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_c2_set_lost_comms(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_c2_package(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_c2_declare_role(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_c2_designate_control(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_c2_command_hsa(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_c2_command_route(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_c2_cancel_command(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_c2_command_statuses(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_dissemination(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dms_publish(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dms_peers(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dms_link_tracks(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_designate_link(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_designation_source(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_link_track_truth(PyObject* o, PyObject* const* args, Py_ssize_t n);

#endif /* FSIM_NATIVE_WORLD_H */
