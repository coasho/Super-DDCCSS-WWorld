/* Private support for fsim._native and its DMS/C2 adapter.
 * The core retains World/Error ownership and all Python type registration. */
#ifndef FSIM_NATIVE_WORLD_H
#define FSIM_NATIVE_WORLD_H

#include "fsim_py.h"
#include "fsim/fsim_c.h"

/* Largest command row accepted by the native control bindings. */
#define FSIM_PY_VALUES 40
#define COUNT(a) (sizeof(a) / sizeof((a)[0]))

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

typedef struct {
    PyObject_HEAD
    fsim_scenario* scenario;
} ScenarioObject;

#define WORLD_IDLE(self) fsim_py_idle((self)->busy, "this World")

/* The mission episode family's types (_native_mission_env.c), added to the module by its init. */
int fsim_py_add_mission_types(PyObject* m);

/* Shared conversions and the bridge to the core's original static Error object. */
PyObject* fail(void);
int check_args(Py_ssize_t n, Py_ssize_t lo, Py_ssize_t hi, const char* name);
int as_u32(PyObject* o, uint32_t* out);
int as_u64(PyObject* o, uint64_t* out);
int as_int(PyObject* o, int* out);
int get_array(PyObject* o, Py_buffer* b, int writable, const char* formats, Py_ssize_t itemsize, const char* what);
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
PyObject* world_bind_ma_service_agent(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_ma_services(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_ma_capability_owner(PyObject* o, PyObject* const* args, Py_ssize_t n);
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


/* Control and Mission handlers registered by the core's original tables. */
PyObject* world_command(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_command_batch(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_command_behavior(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_submit(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_submit_behavior(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_submit_mode(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_submit_route(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_update_route(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_submit_curve(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_update_curve(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_submit_nurbs(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_update_nurbs(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_submit_support(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_update(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_update_behavior(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_update_batch(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_cancel(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_command(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_command_branch(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_info(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_vehicle_activities(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_capabilities(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_progress(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_commanded(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_navigation_report(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_performance_profile(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_navigation(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_navigation(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_qnh(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_qnh(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_state_data(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_ownship(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_create_frame(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_remove_frame(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_frame_point(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_op_point(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_op_points(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_op_point(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_remove_op_point(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_op_zone(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_op_zones(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_op_zone(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_remove_op_zone(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_submit_must_fly(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_update_must_fly(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_op_line(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_op_lines(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_op_line(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_remove_op_line(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_submit_must_fly_line(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_update_must_fly_line(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_op_volume(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_op_volumes(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_op_volume(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_remove_op_volume(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_submit_must_fly_volume(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_update_must_fly_volume(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_setpoint(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_activity_end_points(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_capability_status(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_capability_status_info(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_support(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_availability_ex(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_performance(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_control_revision(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_control_mode(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_control_mode(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_request_control(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_release_control(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_revoke_control(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_allowed(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_control_status(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_capability_precedence(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_capability_precedence(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_availability(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_profile_value(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_profile_section(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_vehicle_default(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_vehicle_default(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_protection(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_protection(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_envelope(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_active_level(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_behavior_finished(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_use_controller(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_controller_parameter(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_controller_parameter(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_last_findings(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_last_endurance(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_last_terrain(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_last_adjustments(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_submit_batch(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_store_task(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_command_task(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_cancel_task(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_remove_task(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_task_status(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_tasks(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_publish_plan(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_validate_plan(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_validate_stored_plan(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_load_plan(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_load_airfield(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_airfields(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_get_airfield(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_plan_command(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_abort_plan(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_remove_plan(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_plan_status(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_intercept_status(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_plans(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_get_plan(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_reason_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_activity_state_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_limit_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_constraint_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_support_features(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_flight_mode_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_command_field_count(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_command_field_count_full(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_support_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_rule_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_rule_description(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_availability_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_reason_description(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_designate(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_undesignate(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_designations(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_designation_truth(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_payloads(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_ms_capabilities(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_ms_capability_status(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_payload_state(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_payload_bit(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_ms_cancel(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_ms_activity(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_ms_activities(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_payload_fail(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_launch_weapon(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_weapons(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_dispense(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_countermeasures(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_launch_zone(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_seeker_target(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_engagement_events(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_decoys(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_stores(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_load_store(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_load_loadout(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_strike(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_jettison(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_strike_consent(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_strike_status(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_mass_kg(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_vehicle_role(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_radio_path(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_radar_cross_section(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_role_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_amti_search(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_radar_status(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_amti_detections(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_amti_tracks(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_contacts(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_agent_report(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_agent_strike_reports(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_agent_consent_reports(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_agent_operator_callsign(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_amti_mode_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_amti_task_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_amti_acm(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_store_status_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_strike_phase_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_weapon_end_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_ms_reason_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_ms_reason_description(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_payload_family_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_payload_state_name(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_ms_capability_type_id(PyObject* o, PyObject* const* args, Py_ssize_t n);

/* Additional World, Scenario and Environment callbacks. */
PyObject* world_step(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_agent_audit(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_agent_identity(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_info(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_time(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_create_vehicle(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_remove_vehicle(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_focus_viewer(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_reset_vehicle(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_find_vehicle(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_vehicle_ids(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_vehicle_info(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_gather_states(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_get_property(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_property(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_comm_create_node(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_comm_send(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_comm_inbox(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_comm_set_medium(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_comm_attach_protocol(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_comm_attach_udp_bridge(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_capsule(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_borrowed_flag(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* scenario_new(PyTypeObject* type, PyObject* args, PyObject* kwargs);
void scenario_dealloc(PyObject* o);
PyObject* scenario_world_options(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* scenario_vehicle_count(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_magnetic_year(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_get_environment(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_set_environment(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_add_effect(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_clear_effects(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* world_terrain(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* mod_magnetic_field(PyObject* m, PyObject* const* args, Py_ssize_t n);
PyObject* mod_decimal_year(PyObject* m, PyObject* const* args, Py_ssize_t n);

#endif /* FSIM_NATIVE_WORLD_H */
