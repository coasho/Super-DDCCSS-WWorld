#include <fsim/fsim_c.h>
#include <stdio.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"fin C check failed: %s\n",#x); return 1; } } while(0)
int main(void) {
    fsim_world_options options;
    fsim_world* world=NULL;
    fsim_weapon_controls_v1 state;
    uint32_t required=99;
    fsim_world_options_init(&options); options.name="fin-c"; options.publish=0; options.terrain=0;
    CHECK(fsim_world_create(&options,&world)==FSIM_OK);
    fsim_weapon_controls_v1_init(&state);
    CHECK(fsim_world_weapon_controls_v1(world,0,&state,NULL,0,&required)==FSIM_INVALID_ARGUMENT);
    CHECK(required==0);
    CHECK(fsim_world_weapon_controls_v1(world,0,NULL,NULL,0,&required)==FSIM_INVALID_ARGUMENT);
    fsim_world_destroy(world);
    return 0;
}
