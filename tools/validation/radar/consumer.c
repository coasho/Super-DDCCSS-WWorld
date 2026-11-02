/* Installed C99 consumer: sized copies, source query and own-Agent boundary. */
#include <fsim/fsim_fusion.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(value) do { if(!(value)){fprintf(stderr,"line %d: %s (%s)\n",__LINE__,#value,fsim_last_error());return 1;} } while(0)
int main(int argc,char** argv) {
    fsim_world_options options;fsim_vehicle_spec spec;fsim_world *world=NULL,*agent=NULL;
    fsim_ms_result result;fsim_amti_command search;fsim_fused_entity_v1 first,second;
    fsim_fusion_source_v1 source;fsim_fusion_status_v1 status;
    uint32_t lead=0,target=0,count=0,i;char loaded[32768],expected[32768],absolute[32768];
    CHECK(argc==2);CHECK(GetModuleFileNameA(GetModuleHandleA("libfsim.dll"),loaded,sizeof loaded)>0);
    CHECK(strlen(argv[1])+32<sizeof expected);snprintf(expected,sizeof expected,"%s/bin/libfsim.dll",argv[1]);
    CHECK(GetFullPathNameA(expected,sizeof absolute,absolute,NULL)>0);CHECK(_stricmp(loaded,absolute)==0);
    fsim_world_options_init(&options);options.name="sim-m01-installed-c99";options.publish=0;options.workers=1;options.pin_workers=0;options.seed=3;
    CHECK(fsim_world_create(&options,&world)==FSIM_OK);
    fsim_vehicle_spec_init(&spec);spec.type="jsbsim:f16c";spec.name="lead";spec.latitude_deg=40;spec.longitude_deg=0;spec.heading_deg=90;
    spec.altitude_msl_m=5000;spec.airspeed_ms=200;CHECK(fsim_world_create_vehicle(world,&spec,&lead)==FSIM_OK);
    spec.name="target";spec.heading_deg=270;spec.longitude_deg=8000/(6371000*cos(40*3.14159265358979323846/180))*180/3.14159265358979323846;
    CHECK(fsim_world_create_vehicle(world,&spec,&target)==FSIM_OK);CHECK(fsim_agent_create(world,lead,"lead",&agent)==FSIM_OK);
    fsim_ms_result_init(&result);CHECK(fsim_vehicle_payload_state(agent,lead,0,FSIM_PAYLOAD_OPERATE,NULL,&result)==FSIM_OK);
    CHECK(result.status==FSIM_COMMAND_ACCEPTED);CHECK(fsim_world_step(world,1)==FSIM_OK);
    fsim_amti_command_init(&search);search.azimuth_half_width_rad=.05;search.bars=2;
    CHECK(fsim_vehicle_amti_search(agent,lead,&search,NULL,&result)==FSIM_OK);CHECK(result.status==FSIM_COMMAND_ACCEPTED);
    for(i=0;i<300&&!count;++i){CHECK(fsim_world_step(world,1)==FSIM_OK);CHECK(fsim_vehicle_fused_tracks_v1(agent,lead,NULL,0,&count)==FSIM_OK);}
    CHECK(count>0);fsim_fused_entity_v1_init(&first);fsim_fused_entity_v1_init(&second);
    CHECK(fsim_vehicle_fused_tracks_v1(agent,lead,&first,1,&count)==FSIM_OK);
    CHECK(fsim_vehicle_fused_tracks_v1(agent,lead,&second,1,&count)==FSIM_OK);
    CHECK(first.id==second.id&&!first.identity_known&&first.source_count>0);
    CHECK(memcmp(first.covariance_ecef,second.covariance_ecef,sizeof first.covariance_ecef)==0);
    fsim_fusion_source_v1_init(&source);CHECK(fsim_vehicle_fused_sources_v1(agent,lead,first.id,&source,1,&count)==FSIM_OK);
    CHECK(count==first.source_count&&source.kind==0&&source.track>0);
    CHECK(fsim_vehicle_fused_tracks_v1(agent,target,NULL,0,&count)==FSIM_NOT_ALLOWED);
    second.struct_size=4;CHECK(fsim_vehicle_fused_tracks_v1(agent,lead,&second,1,&count)==FSIM_INVALID_ARGUMENT);
    fsim_fusion_status_v1_init(&status);CHECK(fsim_vehicle_fusion_status_v1(agent,lead,&status)==FSIM_OK);CHECK(status.rejected_inputs==0);
    printf("installed C99 native fusion: count/copy/repeated read/source/provenance/ownership/size checks passed\n");
    fsim_world_destroy(agent);fsim_world_destroy(world);return 0;
}
