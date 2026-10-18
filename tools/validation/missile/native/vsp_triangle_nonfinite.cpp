// Separate finite-guard regression, Super-DDCCSS-WWorld, 2026-10-18.
// It calls the public kernel only and does not modify the native implementation.
#include "tri_tri_intersect.h"
#include "predicates.h"
#include "vsp_triangle_nonfinite_data.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

static const char *boolean(bool value) { return value ? "true" : "false"; }
static bool same_point(const double *a,const double *b)
{
    return a[0]==b[0]&&a[1]==b[1]&&a[2]==b[2];
}

int main()
{
    // Explicit standalone initialization; no lazy VehicleMgr is involved.
    exactinit();
    unsigned passed=0,failed=0,nonfinite_count=0,finite_count=0;
    const double values[3]={std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()};
    const char *names[3]={"quiet_nan","positive_infinity","negative_infinity"};
    for (int coordinate=0;coordinate<18;++coordinate)
    for (int category=0;category<3;++category)
    {
        double input[2][3][3],before[2][3][3];
        std::memcpy(input,nonfinite_base,sizeof(input));
        input[coordinate/9][coordinate%9/3][coordinate%3]=values[category];
        std::memcpy(before,input,sizeof(input));
        double start[3],end[3];
        std::memcpy(start,nonfinite_start_sentinel,sizeof(start));
        std::memcpy(end,nonfinite_end_sentinel,sizeof(end));
        int coplanar=7;
        const int hit=tri_tri_intersection_test_3d(input[0][0],input[0][1],input[0][2],
                                                input[1][0],input[1][1],input[1][2],
                                                &coplanar,start,end);
        const bool unchanged=std::memcmp(input,before,sizeof(input))==0;
        const bool untouched=std::memcmp(start,nonfinite_start_sentinel,sizeof(start))==0&&
                             std::memcmp(end,nonfinite_end_sentinel,sizeof(end))==0;
        const bool good=hit==0&&coplanar==0&&unchanged&&untouched;
        ++nonfinite_count;
        good?++passed:++failed;
        std::printf("{\"case\":\"coordinate_%02d_%s\",\"category\":\"nonfinite_rejection\",\"passed\":%s,\"coordinate\":%d,\"nonfinite_kind\":\"%s\",\"native_hit\":%d,\"native_coplanar\":%d,\"unchanged_input_bits\":%s,\"untouched_endpoint_sentinels\":%s}\n",
            coordinate,names[category],boolean(good),coordinate,names[category],hit,coplanar,
            boolean(unchanged),boolean(untouched));
    }
    // Positive and negative finite controls prevent an always-miss stub from
    // passing the guard fixture. All input/output expectations are exact dyadics.
    for (int kind=0;kind<2;++kind)
    {
        double input[2][3][3],before[2][3][3];
        std::memcpy(input,nonfinite_base,sizeof(input));
        if(kind==1) for(int i=0;i<3;++i) input[1][i][0]+=4;
        std::memcpy(before,input,sizeof(input));
        double start[3],end[3];
        std::memcpy(start,nonfinite_start_sentinel,sizeof(start));
        std::memcpy(end,nonfinite_end_sentinel,sizeof(end));
        int coplanar=0;
        const int hit=tri_tri_intersection_test_3d(input[0][0],input[0][1],input[0][2],
                                                input[1][0],input[1][1],input[1][2],
                                                &coplanar,start,end);
        const bool unchanged=std::memcmp(input,before,sizeof(input))==0;
        const bool endpoints=kind==0 ?
            ((same_point(start,finite_segment_ends[0])&&same_point(end,finite_segment_ends[1]))||
             (same_point(start,finite_segment_ends[1])&&same_point(end,finite_segment_ends[0]))) :
            (std::memcmp(start,nonfinite_start_sentinel,sizeof(start))==0&&
             std::memcmp(end,nonfinite_end_sentinel,sizeof(end))==0);
        const bool good=hit==(kind==0?1:0)&&coplanar==0&&unchanged&&endpoints;
        ++finite_count;good?++passed:++failed;
        std::printf("{\"case\":\"finite_%s\",\"category\":\"finite_control\",\"passed\":%s,\"native_hit\":%d,\"native_coplanar\":%d,\"unchanged_input_bits\":%s,\"exact_endpoint_contract\":%s}\n",
            kind==0?"segment":"separated",boolean(good),hit,coplanar,boolean(unchanged),boolean(endpoints));
    }
    std::printf("VSP_NONFINITE_FIXTURE passed=%u failed=%u total=%u nonfinite_cases=%u finite_controls=%u\n",
        passed,failed,passed+failed,nonfinite_count,finite_count);
    return failed?1:0;
}
