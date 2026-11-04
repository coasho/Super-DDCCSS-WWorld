/* Installed C99 SDK only. Actor strike/consent use permission-scoped handles. */
#include <fsim/fsim_c.h>
#include "aam_public_capture.h"
#include <math.h>
#include <stdlib.h>

static unsigned checks=0;
#define CHECK(x) do { ++checks; if(!(x)){fprintf(stderr,"Installed C99 AAM line%d: %s (%s)\n",__LINE__,#x,fsim_last_error());goto failed;} }while(0)
int main(int argc,char** argv) {
    const double pi=3.14159265358979323846,deg=pi/180.;
    fsim_world_options options;fsim_world *world=NULL,*launcher=NULL,*approver=NULL,*opponent_agent=NULL;
    fsim_vehicle_spec spec;fsim_c2_node_spec node;fsim_net_spec net_spec;fsim_payload_info payload;
    fsim_command_result flight;fsim_ms_options command_options;fsim_ms_result result;fsim_amti_command scan;
    fsim_amti_track tracks[16];fsim_strike_command strike;fsim_consent_request consent;fsim_weapon_info record;
    fsim_weapon_controls_v1 control;fsim_weapon_fin_v1 fins[32];fsim_engagement_event event;
    uint32_t carrier=0,target=0,c2=0,net=0,radar=UINT32_MAX,station_payload=UINT32_MAX,designation=0,weapon=0;
    uint32_t i,j,n,ready=0,required=0,ids[512];uint64_t track=0,activity=0;
    int32_t reason=-1;int answered=0,sarh,station;unsigned samples=0,controls=0,tracking=0,burning=0,damage=0;
    char root[32768],path[32768],name[256];double hold[4]={250,0,0,NAN},launch=0;FILE* trace=NULL;
    const fsim_vehicle_state* own;
    if(argc!=9)return 2;
    station=atoi(argv[5]);sarh=atoi(argv[6])!=0;
    CHECK(snprintf(root,sizeof(root),"%s/share/flightsim/jsbsim",argv[1])>0&&strlen(argv[1])+25<sizeof(root));
    CHECK(snprintf(name,sizeof(name),"m03-installed-c99-%s",argv[2])>0&&strlen(argv[2])+25<sizeof(name));
    fsim_world_options_init(&options);options.name=name;options.jsbsim_root=root;
    options.publish=0;options.terrain=0;options.workers=1;options.pin_workers=0;options.seed=19;
    options.dt=1./120;options.frame_skip=4;CHECK(fsim_world_create(&options,&world)==FSIM_OK);
    fsim_vehicle_spec_init(&spec);spec.name="launcher";
    CHECK(snprintf(path,sizeof(path),"jsbsim:%s",argv[4])>0&&strlen(argv[4])+8<sizeof(path));spec.type=path;
    spec.latitude_deg=40;spec.longitude_deg=0;spec.altitude_msl_m=6000;spec.heading_deg=90;spec.airspeed_ms=250;
    CHECK(fsim_world_create_vehicle(world,&spec,&carrier)==FSIM_OK);
    CHECK(fsim_agent_create(world,carrier,"launcher",&launcher)==FSIM_OK);hold[2]=90*deg;
    CHECK(fsim_vehicle_submit(launcher,carrier,FSIM_LEVEL_VELOCITY,hold,4,NULL,&flight)==FSIM_OK&&flight.reason==0);
    fsim_c2_node_spec_init(&node);node.name="approver";node.latitude_rad=39.9*deg;node.altitude_msl_m=200;
    CHECK(fsim_world_create_c2_node(world,&node,&c2)==FSIM_OK);fsim_net_spec_init(&net_spec);
    CHECK(fsim_world_create_net(world,&net_spec,&net)==FSIM_OK);
    CHECK(fsim_world_join_net(world,carrier,net)==FSIM_OK&&fsim_world_join_net(world,c2,net)==FSIM_OK);
    CHECK(fsim_agent_create(world,c2,"approver",&approver)==FSIM_OK);
    CHECK(fsim_c2_declare_role(approver,c2,2,"APPROVER",NULL,0,&reason)==FSIM_OK&&reason==0);
    for(i=0;i<fsim_vehicle_payload_count(launcher,carrier);++i) {
        fsim_payload_info_init(&payload);CHECK(fsim_vehicle_payload(launcher,carrier,i,&payload)==FSIM_OK);
        if(payload.family==FSIM_PAYLOAD_AIR_RADAR)radar=payload.index;
        if(payload.family==FSIM_PAYLOAD_STORE_STATION)station_payload=payload.index;
    }
    CHECK(radar!=UINT32_MAX&&station_payload!=UINT32_MAX);
    CHECK(fsim_vehicle_load_store(world,carrier,station_payload,station,argv[2],1,&reason)==FSIM_OK&&reason==0);
    fsim_ms_options_init(&command_options);fsim_ms_result_init(&result);
    CHECK(fsim_vehicle_payload_state(launcher,carrier,radar,FSIM_PAYLOAD_OPERATE,&command_options,&result)==FSIM_OK&&result.reason==0);
    CHECK(fsim_vehicle_payload_state(launcher,carrier,station_payload,FSIM_PAYLOAD_OPERATE,&command_options,&result)==FSIM_OK&&result.reason==0);
    for(i=0;i<1350&&ready!=2;++i) {
        CHECK(fsim_world_step(world,1)==FSIM_OK);ready=0;
        for(j=0;j<2;++j) {
            fsim_payload_info_init(&payload);CHECK(fsim_vehicle_payload(launcher,carrier,j?station_payload:radar,&payload)==FSIM_OK);
            if(payload.state==FSIM_PAYLOAD_OPERATE)++ready;
        }
    }
    CHECK(ready==2);own=fsim_vehicle_state_ptr(world,carrier);CHECK(own);
    fsim_vehicle_spec_init(&spec);spec.name="opponent";spec.type="jsbsim:f16c";
    spec.latitude_deg=own->latitude_rad/deg;
    spec.longitude_deg=own->longitude_rad/deg+5000/(6371000*cos(own->latitude_rad))/deg;
    spec.altitude_msl_m=6000;spec.airspeed_ms=250;spec.heading_deg=270;
    CHECK(fsim_world_create_vehicle(world,&spec,&target)==FSIM_OK);
    CHECK(fsim_agent_create(world,target,"opponent",&opponent_agent)==FSIM_OK);hold[2]=270*deg;
    CHECK(fsim_vehicle_submit(opponent_agent,target,FSIM_LEVEL_VELOCITY,hold,4,NULL,&flight)==FSIM_OK&&flight.reason==0);
    fsim_amti_command_init(&scan);scan.payload=radar;scan.bars=1;scan.azimuth_half_width_rad=15*deg;scan.range_scale_m=60000;
    CHECK(fsim_vehicle_amti_search(launcher,carrier,&scan,&command_options,&result)==FSIM_OK&&result.reason==0);
    for(i=0;i<600&&!track;++i) {
        CHECK(fsim_world_step(world,1)==FSIM_OK);
        for(j=0;j<16;++j)fsim_amti_track_init(&tracks[j]);
        CHECK(fsim_vehicle_amti_tracks(launcher,carrier,radar,tracks,16,&n)==FSIM_OK);
        for(j=0;j<n&&j<16;++j)if(tracks[j].updates>=3){track=tracks[j].id;break;}
    }
    CHECK(track);CHECK(fsim_vehicle_designate(launcher,carrier,radar,track,&designation)==FSIM_OK&&designation);
    scan.task=FSIM_AMTI_TRACK;scan.mode=FSIM_AMTI_SINGLE_TARGET_TRACK;scan.track=track;scan.illuminate=sarh;
    CHECK(fsim_vehicle_amti_search(launcher,carrier,&scan,&command_options,&result)==FSIM_OK&&result.reason==0);
    fsim_strike_command_init(&strike);strike.payload=station_payload;strike.store=argv[2];strike.target=designation;
    CHECK(fsim_vehicle_strike(launcher,carrier,&strike,&command_options,&result)==FSIM_OK&&result.reason==0);
    activity=result.activity;CHECK(fsim_world_weapon_count(world)==0);
    for(i=0;i<1350&&!answered;++i) {
        CHECK(fsim_world_step(world,1)==FSIM_OK);
        for(j=0;j<fsim_c2_consent_count(approver,c2);++j) {
            fsim_consent_request_init(&consent);CHECK(fsim_c2_consent_get(approver,c2,j,&consent)==FSIM_OK);
            if(consent.strike==activity) {
                CHECK(fsim_world_weapon_count(world)==0&&consent.received_s>=consent.time_s);
                CHECK(fsim_c2_answer_consent(approver,c2,consent.from,consent.request_id,1,&reason)==FSIM_OK&&reason==0);
                answered=1;break;
            }
        }
    }
    CHECK(answered);for(i=0;i<600&&fsim_world_weapon_count(world)==0;++i)CHECK(fsim_world_step(world,1)==FSIM_OK);
    CHECK(fsim_world_weapon_count(world)==1);fsim_weapon_info_init(&record);CHECK(fsim_world_weapon(world,0,&record)==FSIM_OK);
    weapon=record.id;launch=record.launch_time_s;
    CHECK(weapon&&record.launcher==carrier&&!record.jettisoned&&strcmp(record.store,argv[2])==0);
    CHECK(strcmp(fsim_vehicle_type(world,weapon),argv[3])==0);
    CHECK(snprintf(path,sizeof(path),"%s/trace.csv",argv[8])>0&&strlen(argv[8])+11<sizeof(path));
    trace=fopen(path,"wb");CHECK(trace);fputs("model,weapon,time,age,tracking,burning,controller,fin_count,terminal\n",trace);
    while(fsim_world_time(world)<launch+40) {
        CHECK(fsim_world_step(world,1)==FSIM_OK);CHECK(fsim_world_weapon_count(world)==1);
        fsim_weapon_info_init(&record);CHECK(fsim_world_weapon(world,0,&record)==FSIM_OK&&record.id==weapon);++samples;
        tracking+=record.tracking?1U:0U;burning+=record.burning?1U:0U;
        fsim_weapon_controls_v1_init(&control);for(i=0;i<32;++i)fsim_weapon_fin_v1_init(&fins[i]);
        if(record.end==FSIM_WEAPON_IN_FLIGHT) {
            CHECK(fsim_world_weapon_controls_v1(world,weapon,&control,fins,32,&required)==FSIM_OK&&required<=32);
            if(control.controller_available)++controls;
        }
        fprintf(trace,"%s,%u,%.17g,%.17g,%d,%d,%d,%u,%d\n",argv[2],weapon,fsim_world_time(world),fsim_world_time(world)-launch,
            record.tracking,record.burning,control.controller_available,control.fin_count,record.end);
        n=fsim_world_vehicle_ids(world,ids,512);CHECK(n<=512);
        for(i=0;i<n;++i) {
            const fsim_vehicle_state* s=fsim_vehicle_state_ptr(world,ids[i]);CHECK(s&&!s->diverged);
            for(j=0;j<3;++j)CHECK(isfinite(s->position_ecef[j])&&isfinite(s->velocity_body_ms[j]));
        }
    }
    CHECK(!ferror(trace));{const int closed=fclose(trace);trace=NULL;CHECK(closed==0);}
    CHECK(snprintf(path,sizeof(path),"%s/events.csv",argv[8])>0&&strlen(argv[8])+12<sizeof(path));
    trace=fopen(path,"wb");CHECK(trace);fputs("model,weapon,serial,time,kind,source,target\n",trace);
    n=fsim_world_engagement_count(world,0);
    for(i=0;i<n;++i) {
        fsim_engagement_event_init(&event);CHECK(fsim_world_engagement(world,i,&event)==FSIM_OK);
        fprintf(trace,"%s,%u,%llu,%.17g,%s,%u,%u\n",argv[2],weapon,(unsigned long long)event.serial,event.time_s,event.kind,event.entity,event.target);
        if(event.entity==weapon&&event.target==target&&strcmp(event.kind,"damage")==0)++damage;
    }
    CHECK(samples&&tracking&&burning&&controls);CHECK(record.end==FSIM_WEAPON_DETONATED&&damage);
    CHECK(!ferror(trace));{const int closed=fclose(trace);trace=NULL;CHECK(closed==0);}
    CHECK(snprintf(path,sizeof(path),"%s/capture.json",argv[8])>0&&strlen(argv[8])+14<sizeof(path));
    CHECK(aam_public_capture(path,"c99",argv[2],argv[3],argv[7],weapon,target,checks,samples,controls,tracking,burning,damage,
        fsim_weapon_end_name(record.end),launch,fsim_world_time(world),argv[8]));
    fsim_world_destroy(opponent_agent);fsim_world_destroy(approver);fsim_world_destroy(launcher);fsim_world_destroy(world);return 0;
failed:
    if(trace)fclose(trace);
    if(opponent_agent)fsim_world_destroy(opponent_agent);
    if(approver)fsim_world_destroy(approver);
    if(launcher)fsim_world_destroy(launcher);
    if(world)fsim_world_destroy(world);
    return 1;
}
