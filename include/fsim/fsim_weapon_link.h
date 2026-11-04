#ifndef FSIM_WEAPON_LINK_H
#define FSIM_WEAPON_LINK_H
#include "fsim/fsim_c.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Additive opt-in V1. Existing enums, layouts, permissions and DMS topics are frozen.
 * IDs are scoped by instance + platform + controller + released weapon.
 * Readers never consume; count reports the complete length even for short buffers.
 * Unknown timing values have *_known=0 (their numeric slots are not measurements). */
enum fsim_weapon_link_direction_v1 {
    FSIM_WEAPON_LINK_UNSUPPORTED_V1=0, FSIM_WEAPON_LINK_UPLINK_V1=1, FSIM_WEAPON_LINK_TWO_WAY_V1=2
};
enum fsim_weapon_link_reason_v1 {
    FSIM_WEAPON_LINK_NONE_V1=0, FSIM_WEAPON_LINK_NOT_SUPPORTED_V1=1,
    FSIM_WEAPON_LINK_UNKNOWN_WEAPON_V1=2, FSIM_WEAPON_LINK_NOT_OWNER_V1=3,
    FSIM_WEAPON_LINK_INVALID_V1=4, FSIM_WEAPON_LINK_REQUEST_CONFLICT_V1=5,
    FSIM_WEAPON_LINK_CAPACITY_V1=6, FSIM_WEAPON_LINK_CLOSED_V1=7,
    FSIM_WEAPON_LINK_UNKNOWN_ASSESSMENT_V1=8, FSIM_WEAPON_LINK_EXPIRED_V1=9
};
enum fsim_weapon_assessment_state_v1 {
    FSIM_WEAPON_ASSESSMENT_PENDING_V1=0, FSIM_WEAPON_ASSESSMENT_ACTIVE_V1=1,
    FSIM_WEAPON_ASSESSMENT_SOFT_LOST_V1=2, FSIM_WEAPON_ASSESSMENT_LOST_V1=3,
    FSIM_WEAPON_ASSESSMENT_END_OF_FLIGHT_V1=4, FSIM_WEAPON_ASSESSMENT_CANCELED_V1=5,
    FSIM_WEAPON_ASSESSMENT_EXPIRED_V1=6
};
typedef struct fsim_weapon_link_capability_v1 {
    uint32_t struct_size,weapon,direction,reason;
    uint64_t instance,activity;
    double update_period_s,feedback_period_s,latency_s,data_rate_bps,range_m,frequency_hz;
    double maximum_age_s,assessment_timeout_s,retention_s;
    char direction_source[256];
    double loss_probability,maximum_queue_s;
    uint32_t queue_capacity,report_capacity,assessment_capacity;
    double jitter_s;
} fsim_weapon_link_capability_v1;
typedef struct fsim_weapon_assessment_request_v1 {
    uint32_t struct_size,weapon;
    uint64_t instance,request_id;
    double period_s,duration_s;
} fsim_weapon_assessment_request_v1;
typedef struct fsim_weapon_assessment_receipt_v1 {
    uint32_t struct_size,accepted,reason;
    uint64_t assessment,request_id;
} fsim_weapon_assessment_receipt_v1;
typedef struct fsim_weapon_assessment_status_v1 {
    uint32_t struct_size,state,reason,last_received_known;
    uint64_t assessment,request_id;
    double requested_time_s,expires_time_s,last_received_time_s;
} fsim_weapon_assessment_status_v1;
typedef struct fsim_weapon_feedback_v1 {
    uint32_t struct_size,weapon,navigation_valid,tracking,active,supported,end_of_flight;
    uint32_t time_to_active_known,time_to_intercept_known;
    uint64_t assessment,sequence;
    double sample_time_s,sent_time_s,received_time_s,valid_until_s;
    double state_ecef[6],time_to_active_s,time_to_intercept_s;
    double navigation_time_s,navigation_covariance[36];
    uint32_t navigation_gnss_aided;
} fsim_weapon_feedback_v1;
typedef struct fsim_weapon_fused_designation_v1 {
    uint32_t struct_size,target,valid;
    uint64_t entity;
} fsim_weapon_fused_designation_v1;
FSIM_API void fsim_weapon_link_capability_v1_init(fsim_weapon_link_capability_v1*);
FSIM_API void fsim_weapon_assessment_request_v1_init(fsim_weapon_assessment_request_v1*);
FSIM_API void fsim_weapon_assessment_receipt_v1_init(fsim_weapon_assessment_receipt_v1*);
FSIM_API void fsim_weapon_assessment_status_v1_init(fsim_weapon_assessment_status_v1*);
FSIM_API void fsim_weapon_feedback_v1_init(fsim_weapon_feedback_v1*);
FSIM_API void fsim_weapon_fused_designation_v1_init(fsim_weapon_fused_designation_v1*);
FSIM_API int fsim_weapon_link_instance_v1(fsim_world*,uint32_t platform,uint64_t* instance);
FSIM_API int fsim_weapon_link_capability_get_v1(const fsim_world*,uint32_t platform,uint32_t weapon,fsim_weapon_link_capability_v1*);
FSIM_API int fsim_weapon_fused_designate_v1(fsim_world*,uint32_t platform,uint64_t instance,uint64_t entity,uint32_t* target);
FSIM_API int fsim_weapon_fused_designations_v1(const fsim_world*,uint32_t platform,fsim_weapon_fused_designation_v1*,uint32_t max,uint32_t* count);
FSIM_API int fsim_weapon_assess_v1(fsim_world*,uint32_t platform,const fsim_weapon_assessment_request_v1*,fsim_weapon_assessment_receipt_v1*);
FSIM_API int fsim_weapon_assessment_cancel_v1(fsim_world*,uint32_t platform,uint64_t instance,uint32_t weapon,uint64_t assessment,uint32_t* canceled);
FSIM_API int fsim_weapon_assessment_status_get_v1(const fsim_world*,uint32_t platform,uint32_t weapon,uint64_t assessment,fsim_weapon_assessment_status_v1*);
FSIM_API int fsim_weapon_feedback_get_v1(const fsim_world*,uint32_t platform,uint32_t weapon,uint64_t assessment,uint64_t since,fsim_weapon_feedback_v1*,uint32_t max,uint32_t* count);
#ifdef __cplusplus
}
#endif
#endif
