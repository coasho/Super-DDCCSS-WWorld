/* Public C99 weapon-link integration. Real native release and delivered C2 consent. */
#include <fsim/fsim_c.h>
#include <fsim/fsim_fusion.h>
#include <fsim/fsim_weapon_link.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#ifndef FSIM_TEST_JSBSIM_ROOT
#define FSIM_TEST_JSBSIM_ROOT ""
#endif
#define CHECK(x) do { if(!(x)){fprintf(stderr,"weapon links C99 line %d: %s (%s)\n",__LINE__,#x,fsim_last_error());return 1;} }while(0)
/* Invalid calls must replace a prior thread-local error and preserve outputs. */
static int seed_diagnostic(fsim_world* world,uint32_t platform,unsigned sentinel) {
    uint64_t instance=0;
    CHECK(fsim_weapon_link_instance_v1(world,platform,&instance)==FSIM_OK && instance!=0);
    CHECK(fsim_last_error()[0]=='\0');
    if(sentinel) {
        CHECK(fsim_world_gather_states(NULL,NULL,0,0,NULL)==FSIM_INVALID_ARGUMENT);
        CHECK(strstr(fsim_last_error(),"fsim_world_gather_states")!=NULL);
    }
    return 0;
}
#define INVALID_DIAGNOSTIC(call, function_name) do { \
    unsigned seed; \
    for(seed=0;seed<2;++seed) { \
        CHECK(seed_diagnostic(world,platform,1-seed)==0); \
        CHECK((call)==FSIM_INVALID_ARGUMENT); \
        CHECK(strstr(fsim_last_error(),function_name)!=NULL); \
        CHECK(strstr(fsim_last_error(),"fsim_world_gather_states")==NULL); \
        ++checks; \
    } \
} while(0)
#define INVALID_UNCHANGED(call, function_name, object) do { \
    unsigned char before[sizeof(object)]; \
    memcpy(before,&(object),sizeof(object)); \
    INVALID_DIAGNOSTIC(call,function_name); \
    CHECK(memcmp(before,&(object),sizeof(object))==0); \
} while(0)
static int invalid_diagnostics(const fsim_world_options* base_options) {
    fsim_world_options options=*base_options;fsim_world* world=NULL;fsim_c2_node_spec node;
    fsim_weapon_link_capability_v1 cap;fsim_weapon_assessment_request_v1 request;
    fsim_weapon_assessment_receipt_v1 receipt;fsim_weapon_assessment_status_v1 status;
    fsim_weapon_fused_designation_v1 designation;fsim_weapon_feedback_v1 feedback;
    uint32_t platform=0,target=0xa5a5a5a5U,canceled=0xa5a5a5a5U,count=0;uint64_t instance=UINT64_MAX;
    unsigned checks=0;
    options.name="weapon-link-C-invalid-diagnostics";
    CHECK(fsim_world_create(&options,&world)==FSIM_OK);
    fsim_c2_node_spec_init(&node);node.name="diagnostic endpoint";
    CHECK(fsim_world_create_c2_node(world,&node,&platform)==FSIM_OK && platform!=0);
    INVALID_UNCHANGED(fsim_weapon_link_instance_v1(NULL,platform,&instance),"fsim_weapon_link_instance_v1",instance);
    INVALID_DIAGNOSTIC(fsim_weapon_link_instance_v1(world,platform,NULL),"fsim_weapon_link_instance_v1");
    INVALID_UNCHANGED(fsim_weapon_link_instance_v1(world,UINT32_MAX,&instance),"fsim_weapon_link_instance_v1",instance);
    fsim_weapon_link_capability_v1_init(&cap);cap.weapon=0xa5a5a5a5U;
    INVALID_UNCHANGED(fsim_weapon_link_capability_get_v1(NULL,platform,0,&cap),"fsim_weapon_link_capability_get_v1",cap);
    INVALID_DIAGNOSTIC(fsim_weapon_link_capability_get_v1(world,platform,0,NULL),"fsim_weapon_link_capability_get_v1");
    cap.struct_size=sizeof(cap)-1;
    INVALID_UNCHANGED(fsim_weapon_link_capability_get_v1(world,platform,0,&cap),"fsim_weapon_link_capability_get_v1",cap);
    INVALID_UNCHANGED(fsim_weapon_fused_designate_v1(NULL,platform,0,0,&target),"fsim_weapon_fused_designate_v1",target);
    INVALID_DIAGNOSTIC(fsim_weapon_fused_designate_v1(world,platform,0,0,NULL),"fsim_weapon_fused_designate_v1");
    fsim_weapon_fused_designation_v1_init(&designation);designation.target=0xa5a5a5a5U;count=UINT32_MAX;
    INVALID_UNCHANGED(fsim_weapon_fused_designations_v1(NULL,platform,&designation,1,&count),"fsim_weapon_fused_designations_v1",designation);
    INVALID_UNCHANGED(fsim_weapon_fused_designations_v1(world,platform,&designation,1,NULL),"fsim_weapon_fused_designations_v1",designation);
    INVALID_DIAGNOSTIC(fsim_weapon_fused_designations_v1(world,platform,NULL,1,&count),"fsim_weapon_fused_designations_v1");
    designation.struct_size=sizeof(designation)-1;
    INVALID_UNCHANGED(fsim_weapon_fused_designations_v1(world,platform,&designation,1,&count),"fsim_weapon_fused_designations_v1",designation);
    CHECK(count==0);
    fsim_weapon_assessment_request_v1_init(&request);fsim_weapon_assessment_receipt_v1_init(&receipt);receipt.assessment=UINT64_MAX;
    INVALID_UNCHANGED(fsim_weapon_assess_v1(NULL,platform,&request,&receipt),"fsim_weapon_assess_v1",receipt);
    INVALID_UNCHANGED(fsim_weapon_assess_v1(world,platform,NULL,&receipt),"fsim_weapon_assess_v1",receipt);
    INVALID_DIAGNOSTIC(fsim_weapon_assess_v1(world,platform,&request,NULL),"fsim_weapon_assess_v1");
    request.struct_size=sizeof(request)-1;
    INVALID_UNCHANGED(fsim_weapon_assess_v1(world,platform,&request,&receipt),"fsim_weapon_assess_v1",receipt);
    fsim_weapon_assessment_request_v1_init(&request);receipt.struct_size=sizeof(receipt)-1;
    INVALID_UNCHANGED(fsim_weapon_assess_v1(world,platform,&request,&receipt),"fsim_weapon_assess_v1",receipt);
    INVALID_UNCHANGED(fsim_weapon_assessment_cancel_v1(NULL,platform,0,0,0,&canceled),"fsim_weapon_assessment_cancel_v1",canceled);
    INVALID_DIAGNOSTIC(fsim_weapon_assessment_cancel_v1(world,platform,0,0,0,NULL),"fsim_weapon_assessment_cancel_v1");
    fsim_weapon_assessment_status_v1_init(&status);status.assessment=UINT64_MAX;
    INVALID_UNCHANGED(fsim_weapon_assessment_status_get_v1(NULL,platform,0,0,&status),"fsim_weapon_assessment_status_get_v1",status);
    INVALID_DIAGNOSTIC(fsim_weapon_assessment_status_get_v1(world,platform,0,0,NULL),"fsim_weapon_assessment_status_get_v1");
    status.struct_size=sizeof(status)-1;
    INVALID_UNCHANGED(fsim_weapon_assessment_status_get_v1(world,platform,0,0,&status),"fsim_weapon_assessment_status_get_v1",status);
    fsim_weapon_feedback_v1_init(&feedback);feedback.sequence=UINT64_MAX;count=UINT32_MAX;
    INVALID_UNCHANGED(fsim_weapon_feedback_get_v1(NULL,platform,0,0,0,&feedback,1,&count),"fsim_weapon_feedback_get_v1",feedback);
    INVALID_UNCHANGED(fsim_weapon_feedback_get_v1(world,platform,0,0,0,&feedback,1,NULL),"fsim_weapon_feedback_get_v1",feedback);
    INVALID_DIAGNOSTIC(fsim_weapon_feedback_get_v1(world,platform,0,0,0,NULL,1,&count),"fsim_weapon_feedback_get_v1");
    feedback.struct_size=sizeof(feedback)-1;
    INVALID_UNCHANGED(fsim_weapon_feedback_get_v1(world,platform,0,0,0,&feedback,1,&count),"fsim_weapon_feedback_get_v1",feedback);
    CHECK(count==0 && fsim_world_weapon_count(world)==0);
    CHECK(fsim_weapon_fused_designations_v1(world,platform,NULL,0,&count)==FSIM_OK && count==0);
    fsim_world_destroy(world);
    printf("weapon links public C99 invalid diagnostic checks: %u passed\n",checks);
    return 0;
}
#undef INVALID_UNCHANGED
#undef INVALID_DIAGNOSTIC

static int guided_controls(const fsim_world_options* base_options) {
    const double deg = 3.14159265358979323846 / 180.0;
    fsim_world_options options = *base_options;
    fsim_world *world = NULL, *launcher = NULL, *approver = NULL;
    fsim_vehicle_spec spec;
    fsim_c2_node_spec node;
    fsim_net_spec net_spec;
    fsim_payload_info payload;
    fsim_ms_options command_options;
    fsim_ms_result result;
    fsim_command_result flight;
    fsim_amti_command scan;
    fsim_amti_track tracks[16];
    fsim_strike_command strike;
    fsim_consent_request consent;
    fsim_weapon_info weapon;
    uint32_t carrier = 0, target = 0, c2 = 0, net = 0, designation = 0;
    uint32_t radar = UINT32_MAX, stations = UINT32_MAX, i, j, n, ready = 0;
    uint64_t track = 0, activity = 0;
    int32_t reason = -1;
    int answered = 0;
    double hold[4] = {250, 0, 0, NAN};
    const fsim_vehicle_state* own;
    options.name = "installed-fin-c"; options.terrain = 0;
    CHECK(fsim_world_create(&options, &world) == FSIM_OK);
    fsim_vehicle_spec_init(&spec);
    spec.name = "launcher"; spec.type = "jsbsim:f16c";
    spec.latitude_deg = 40; spec.longitude_deg = 0; spec.altitude_msl_m = 6000; spec.heading_deg = 90; spec.airspeed_ms = 250;
    CHECK(fsim_world_create_vehicle(world, &spec, &carrier) == FSIM_OK);

    CHECK(fsim_agent_create(world, carrier, "launcher", &launcher) == FSIM_OK);
    hold[2] = 90 * deg;
    CHECK(fsim_vehicle_submit(launcher, carrier, FSIM_LEVEL_VELOCITY, hold, 4, NULL, &flight) == FSIM_OK && flight.reason == 0);
    fsim_c2_node_spec_init(&node);
    node.name = "approver"; node.latitude_rad = 39.9 * deg; node.altitude_msl_m = 200;
    CHECK(fsim_world_create_c2_node(world, &node, &c2) == FSIM_OK);
    fsim_net_spec_init(&net_spec);
    CHECK(fsim_world_create_net(world, &net_spec, &net) == FSIM_OK);
    CHECK(fsim_world_join_net(world, carrier, net) == FSIM_OK && fsim_world_join_net(world, c2, net) == FSIM_OK);
    CHECK(fsim_agent_create(world, c2, "approver", &approver) == FSIM_OK);
    CHECK(fsim_c2_declare_role(approver, c2, 2, "APPROVER", NULL, 0, &reason) == FSIM_OK && reason == 0);
    for (i = 0; i < fsim_vehicle_payload_count(launcher, carrier); ++i) {
        fsim_payload_info_init(&payload);
        CHECK(fsim_vehicle_payload(launcher, carrier, i, &payload) == FSIM_OK);
        if (payload.family == FSIM_PAYLOAD_AIR_RADAR) radar = payload.index;
        if (payload.family == FSIM_PAYLOAD_STORE_STATION) stations = payload.index;
    }
    CHECK(radar != UINT32_MAX && stations != UINT32_MAX);
    CHECK(fsim_vehicle_load_store(world,carrier,stations,1,"aim-120d",1,&reason)==FSIM_OK && reason==0);
    fsim_ms_options_init(&command_options); fsim_ms_result_init(&result);
    CHECK(fsim_vehicle_payload_state(launcher, carrier, radar, FSIM_PAYLOAD_OPERATE, &command_options, &result) == FSIM_OK && result.reason == 0);
    CHECK(fsim_vehicle_payload_state(launcher, carrier, stations, FSIM_PAYLOAD_OPERATE, &command_options, &result) == FSIM_OK && result.reason == 0);
    for (i = 0; i < 1200 && ready != 2; ++i) {
        CHECK(fsim_world_step(world, 1) == FSIM_OK);
        ready = 0;
        for (j = 0; j < 2; ++j) {
            fsim_payload_info_init(&payload);
            CHECK(fsim_vehicle_payload(launcher, carrier, j ? stations : radar, &payload) == FSIM_OK);
            if (payload.state == FSIM_PAYLOAD_OPERATE) ++ready;
        }
    }
    CHECK(ready == 2);
    own = fsim_vehicle_state_ptr(world, carrier); CHECK(own != NULL);
    spec.name = "opponent"; spec.latitude_deg = own->latitude_rad / deg + .01; spec.longitude_deg = own->longitude_rad / deg + .15;
    spec.heading_deg = 270;
    CHECK(fsim_world_create_vehicle(world, &spec, &target) == FSIM_OK);
    hold[2] = 270 * deg;
    CHECK(fsim_vehicle_submit(world, target, FSIM_LEVEL_VELOCITY, hold, 4, NULL, &flight) == FSIM_OK && flight.reason == 0);
    fsim_amti_command_init(&scan);
    scan.payload = radar; scan.bars = 1; scan.azimuth_half_width_rad = 15 * deg; scan.range_scale_m = 60000;
    CHECK(fsim_vehicle_amti_search(launcher, carrier, &scan, &command_options, &result) == FSIM_OK && result.reason == 0);
    for (i = 0; i < 600 && !track; ++i) {
        CHECK(fsim_world_step(world, 1) == FSIM_OK);
        for (j = 0; j < 16; ++j) fsim_amti_track_init(&tracks[j]);
        CHECK(fsim_vehicle_amti_tracks(launcher, carrier, radar, tracks, 16, &n) == FSIM_OK);
        for (j = 0; j < n && j < 16; ++j) if (tracks[j].updates >= 3) { track = tracks[j].id; break; }
    }
    CHECK(track != 0);
    { uint64_t instance=0;uint32_t count=0;fsim_fused_entity_v1 rows[16];
      CHECK(fsim_weapon_link_instance_v1(launcher,carrier,&instance)==FSIM_OK && instance);
      fsim_fused_entity_v1_init(rows);
      CHECK(fsim_vehicle_fused_tracks_v1(launcher,carrier,rows,16,&count)==FSIM_OK && count>0);
      CHECK(fsim_weapon_fused_designate_v1(launcher,carrier,instance,rows[0].id,&designation)==FSIM_OK && designation);
      CHECK(fsim_weapon_fused_designate_v1(launcher,carrier,instance+1,rows[0].id,&count)==FSIM_OK && count==0);
    }
    scan.task = FSIM_AMTI_TRACK; scan.mode = FSIM_AMTI_SINGLE_TARGET_TRACK; scan.track = track;
    CHECK(fsim_vehicle_amti_search(launcher, carrier, &scan, &command_options, &result) == FSIM_OK && result.reason == 0);
    fsim_strike_command_init(&strike);
    strike.payload = stations; strike.store = "aim-120d"; strike.target = designation;
    CHECK(fsim_vehicle_strike(launcher, carrier, &strike, &command_options, &result) == FSIM_OK && result.reason == 0);
    activity = result.activity;
    CHECK(fsim_world_weapon_count(world) == 0);
    for (i = 0; i < 1200 && !answered; ++i) {
        CHECK(fsim_world_step(world, 1) == FSIM_OK);
        for (j = 0; j < fsim_c2_consent_count(approver, c2); ++j) {
            fsim_consent_request_init(&consent);
            CHECK(fsim_c2_consent_get(approver, c2, j, &consent) == FSIM_OK);
            if (consent.strike == activity) {
                CHECK(fsim_world_weapon_count(world) == 0 && consent.received_s >= consent.time_s);
                CHECK(fsim_c2_answer_consent(approver, c2, consent.from, consent.request_id, 1, &reason) == FSIM_OK && reason == 0);
                answered = 1; break;
            }
        }
    }
    CHECK(answered);
    for (i = 0; i < 600 && fsim_world_weapon_count(world) == 0; ++i) CHECK(fsim_world_step(world, 1) == FSIM_OK);
    CHECK(fsim_world_weapon_count(world) == 1);
    fsim_weapon_info_init(&weapon); CHECK(fsim_world_weapon(world, 0, &weapon) == FSIM_OK && !weapon.jettisoned);
    { fsim_weapon_link_capability_v1 cap;fsim_weapon_assessment_request_v1 q;
      fsim_weapon_assessment_receipt_v1 receipt,retry;fsim_weapon_feedback_v1 rows[2];
      fsim_weapon_assessment_status_v1 status;uint32_t count=0,canceled=0;
      fsim_weapon_link_capability_v1_init(&cap);
      CHECK(fsim_weapon_link_capability_get_v1(launcher,carrier,weapon.id,&cap)==FSIM_OK && cap.direction==FSIM_WEAPON_LINK_TWO_WAY_V1 && cap.reason==FSIM_WEAPON_LINK_NONE_V1);
      fsim_weapon_assessment_request_v1_init(&q);q.instance=cap.instance;q.weapon=weapon.id;q.request_id=71;
      fsim_weapon_assessment_receipt_v1_init(&receipt);fsim_weapon_assessment_receipt_v1_init(&retry);
      CHECK(fsim_weapon_assess_v1(launcher,carrier,&q,&receipt)==FSIM_OK && receipt.accepted);
      CHECK(fsim_weapon_assess_v1(launcher,carrier,&q,&retry)==FSIM_OK && retry.assessment==receipt.assessment);
      CHECK(fsim_weapon_feedback_get_v1(launcher,carrier,weapon.id,receipt.assessment,0,NULL,0,&count)==FSIM_OK && count==0);
      for(i=0;i<120 && count<3;++i){CHECK(fsim_world_step(world,1)==FSIM_OK);
        CHECK(fsim_weapon_feedback_get_v1(launcher,carrier,weapon.id,receipt.assessment,0,NULL,0,&count)==FSIM_OK);}
      CHECK(count>=3);fsim_weapon_feedback_v1_init(rows);memset(&rows[1],0xa5,sizeof(rows[1]));
      CHECK(fsim_weapon_feedback_get_v1(launcher,carrier,weapon.id,receipt.assessment,0,rows,1,&count)==FSIM_OK && count>0);
      CHECK(rows[1].struct_size==0xa5a5a5a5U && rows[0].received_time_s>rows[0].sent_time_s);
      CHECK(rows[0].navigation_valid && rows[0].navigation_covariance[0]>0 && rows[0].navigation_time_s<=rows[0].sample_time_s);
      CHECK(fsim_weapon_feedback_get_v1(launcher,target,weapon.id,receipt.assessment,0,NULL,0,&count)==FSIM_NOT_ALLOWED);
      CHECK(fsim_weapon_assess_v1(approver,c2,&q,&retry)==FSIM_OK && !retry.accepted && retry.reason==FSIM_WEAPON_LINK_NOT_OWNER_V1);
      q.duration_s=29;CHECK(fsim_weapon_assess_v1(launcher,carrier,&q,&retry)==FSIM_OK && !retry.accepted && retry.reason==FSIM_WEAPON_LINK_REQUEST_CONFLICT_V1);
      CHECK(fsim_weapon_assessment_cancel_v1(launcher,carrier,cap.instance,weapon.id,receipt.assessment,&canceled)==FSIM_OK && canceled);
      fsim_weapon_assessment_status_v1_init(&status);
      CHECK(fsim_weapon_assessment_status_get_v1(launcher,carrier,weapon.id,receipt.assessment,&status)==FSIM_OK && status.state==FSIM_WEAPON_ASSESSMENT_CANCELED_V1);
      { uint32_t before=0,after=0;CHECK(fsim_weapon_feedback_get_v1(launcher,carrier,weapon.id,receipt.assessment,0,NULL,0,&before)==FSIM_OK);
        CHECK(fsim_world_step(world,30)==FSIM_OK);
        CHECK(fsim_weapon_feedback_get_v1(launcher,carrier,weapon.id,receipt.assessment,0,NULL,0,&after)==FSIM_OK && before==after); }
      q.request_id=72;q.duration_s=.6;
      CHECK(fsim_weapon_assess_v1(launcher,carrier,&q,&retry)==FSIM_OK && retry.accepted);
      CHECK(fsim_world_step(world,30)==FSIM_OK);
      CHECK(fsim_weapon_assessment_status_get_v1(launcher,carrier,weapon.id,retry.assessment,&status)==FSIM_OK && status.state==FSIM_WEAPON_ASSESSMENT_EXPIRED_V1);
      CHECK(fsim_weapon_feedback_get_v1(launcher,carrier,weapon.id,retry.assessment,0,NULL,0,&count)==FSIM_OK && count>0);
    }
    fsim_world_destroy(approver);fsim_world_destroy(launcher);fsim_world_destroy(world);
    puts("weapon links public C99: delivered consent, fused input, finite assessment/feedback, sized reads, ownership and cancel passed");
    return 0;
}

int main(int argc,char** argv){fsim_world_options options;fsim_world_options_init(&options);
options.publish=0;options.terrain=0;options.workers=1;options.pin_workers=0;options.seed=19;
options.jsbsim_root=argc>1?argv[1]:FSIM_TEST_JSBSIM_ROOT;
if(invalid_diagnostics(&options))return 1;
return guided_controls(&options);}
