#ifndef FSIM_FUSION_H
#define FSIM_FUSION_H
#include "fsim/fsim_c.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Additive V1, no changes to existing report layouts or permission defaults.
 * Reads are non-consuming. Sources are queried separately without truncation.
 * state:0 live,1 coasting,2 lost. kind:0 own radar,1 delivered network report.
 * identity_known=0 means unknown; no global target/truth identity is exposed. */
typedef struct fsim_fused_entity_v1 {
    uint32_t struct_size;
    uint64_t id;
    uint32_t state,identity_known,standard_identity,source_count;
    double time_s,last_update_s,valid_until_s;
    double state_ecef[6],covariance_ecef[36];
} fsim_fused_entity_v1;
typedef struct fsim_fusion_source_v1 {
    uint32_t struct_size,kind,system,payload,diagonal_link_covariance;
    uint64_t track;
    double sample_time_s,received_time_s;
} fsim_fusion_source_v1;
typedef struct fsim_fusion_status_v1 {
    uint32_t struct_size,entity_capacity,source_capacity;
    uint64_t rejected_inputs,capacity_refusals;
    double time_s;
} fsim_fusion_status_v1;
FSIM_API void fsim_fused_entity_v1_init(fsim_fused_entity_v1* value);
FSIM_API void fsim_fusion_source_v1_init(fsim_fusion_source_v1* value);
FSIM_API void fsim_fusion_status_v1_init(fsim_fusion_status_v1* value);
FSIM_API int fsim_vehicle_fused_tracks_v1(const fsim_world*,uint32_t entity,fsim_fused_entity_v1* out,uint32_t max,uint32_t* count);
FSIM_API int fsim_vehicle_fused_sources_v1(const fsim_world*,uint32_t entity,uint64_t fused_id,fsim_fusion_source_v1* out,uint32_t max,uint32_t* count);
FSIM_API int fsim_vehicle_fusion_status_v1(const fsim_world*,uint32_t entity,fsim_fusion_status_v1* out);
#ifdef __cplusplus
}
#endif
#endif
