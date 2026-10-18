// Independent closed-contact regression. This file does not modify a kernel.
#include "tri_tri_intersect.h"
#include "predicates.h"
#include "vsp_triangle_contact_data.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

static double distance3( const double *a, const double *b )
{
    return std::hypot( a[0]-b[0], a[1]-b[1], a[2]-b[2] );
}

int main()
{
    // Standalone kernel test: no lazy VehicleMgr initialization is assumed.
    exactinit();
    unsigned passed=0, failed=0, point_cases=0, gap_cases=0;
    for ( const ContactCase &test : contact_cases )
    {
        double a[3][3], b[3][3];
        std::memcpy( a,test.a,sizeof(a) ); std::memcpy( b,test.b,sizeof(b) );
        double start[3]={17,23,31}, end[3]={37,41,43};
        int coplanar=0;
        const int hit=tri_tri_intersection_test_3d( a[0],a[1],a[2],b[0],b[1],b[2],&coplanar,start,end );
        const bool unchanged=std::memcmp(a,test.a,sizeof(a))==0&&std::memcmp(b,test.b,sizeof(b))==0;
        bool finite=true;
        for ( int k=0; k<3; ++k ) { finite=finite&&std::isfinite(start[k])&&std::isfinite(end[k]); }
        double coordinate=0;
        for ( int i=0; i<3; ++i ) for ( int k=0; k<3; ++k )
        { coordinate=std::max(coordinate,std::abs(a[i][k])); coordinate=std::max(coordinate,std::abs(b[i][k])); }
        // Construction check only: this budget NEVER changes hit/miss or
        // expands intervals. Both decisions must equal the exact closed oracle.
        const double budget=64*std::numeric_limits<double>::epsilon()*coordinate;
        double error=0;
        bool endpoints=true;
        if ( test.hit )
        {
            error=std::max(distance3(start,test.point),distance3(end,test.point));
            endpoints=finite&&std::isfinite(error)&&error<=budget;
            ++point_cases;
        }
        else { ++gap_cases; }
        const bool exact_decision=hit==(test.hit?1:0)&&coplanar==0;
        const bool good=exact_decision&&unchanged&&endpoints;
        if (good) {++passed;} else {++failed;}
        std::printf("{\"case\":\"%s\",\"passed\":%s,\"kind\":\"%s\",\"expected_hit\":%s,\"native_hit\":%d,\"native_coplanar\":%d,\"strict_decision_matches\":%s,\"unchanged_inputs\":%s,\"finite_outputs\":%s,\"endpoint_contract\":%s,\"rounding_budget\":%.17g,\"endpoint_error\":",
            test.name,good?"true":"false",test.hit?"exact_point":"positive_gap",test.hit?"true":"false",hit,coplanar,
            exact_decision?"true":"false",unchanged?"true":"false",finite?"true":"false",endpoints?"true":"false",budget);
        if (test.hit&&finite&&std::isfinite(error)) {std::printf("%.17g",error);} else {std::printf("null");}
        std::printf("}\n");
    }
    std::printf("VSP_CONTACT_FIXTURE passed=%u failed=%u total=%u point_cases=%u gap_cases=%u\n",passed,failed,passed+failed,point_cases,gap_cases);
    return failed?1:0;
}
