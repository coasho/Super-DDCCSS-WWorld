// Includes the released 112-case native regression unchanged.
#define main original_fixture_main
#include "vsp_triangle_intersection.cpp"
#undef main
#include "vsp_triangle_boundary_data.h"
#include "predicates.h"

int main()
{
    // Standalone fixture has no VehicleMgr singleton: initialize predicates once.
    exactinit();
    const int base_result=original_fixture_main();
    unsigned passed=0,failed=0;
    for ( const BoundaryCase &test : boundary_cases )
    {
        double a[3][3],b[3][3];
        std::memcpy(a,test.a,sizeof(a)); std::memcpy(b,test.b,sizeof(b));
        double start[3]={17,23,31},end[3]={37,41,43};
        const double sentinel_start[3]={17,23,31},sentinel_end[3]={37,41,43};
        int coplanar=0;
        const int hit=tri_tri_intersection_test_3d(a[0],a[1],a[2],b[0],b[1],b[2],&coplanar,start,end);
        double coordinate=1;
        for ( int i=0; i<3; ++i ) for ( int j=0; j<3; ++j )
        { coordinate=std::max(coordinate,std::abs(a[i][j])); coordinate=std::max(coordinate,std::abs(b[i][j])); }
        const double budget=1024*std::numeric_limits<double>::epsilon()*coordinate;
        const bool unchanged=std::memcmp(a,test.a,sizeof(a))==0&&std::memcmp(b,test.b,sizeof(b))==0;
        bool endpoint_ok=true,filter_ok=true;
        double error=0;
        if ( test.coplanar )
        {
            endpoint_ok=std::memcmp(start,sentinel_start,sizeof(start))==0&&std::memcmp(end,sentinel_end,sizeof(end))==0;
        }
        else if ( test.hit )
        {
            bool finite=true;
            for ( int k=0; k<3; ++k ) { finite=finite&&std::isfinite(start[k])&&std::isfinite(end[k]); }
            const double forward=std::max(distance3(start,test.expected[0]),distance3(end,test.expected[1]));
            const double reverse=std::max(distance3(start,test.expected[1]),distance3(end,test.expected[0]));
            error=std::min(forward,reverse);
            endpoint_ok=finite&&error<=budget;
            filter_ok=hit&&!coplanar&&finite&&inside(a,start)&&inside(a,end)&&inside(b,start)&&inside(b,end);
        }
        const bool good=(hit!=0)==test.hit&&(coplanar!=0)==test.coplanar&&unchanged&&endpoint_ok&&filter_ok;
        if (good) {++passed;} else {++failed;}
        std::printf("{\"case\":\"%s\",\"boundary_control\":true,\"passed\":%s,\"expected_hit\":%s,\"expected_coplanar\":%s,\"native_hit\":%d,\"native_coplanar\":%d,\"endpoint_error\":%.17g,\"rounding_budget\":%.17g,\"native_filters\":%s,\"endpoint_contract\":%s}\n",
            test.name,good?"true":"false",test.hit?"true":"false",test.coplanar?"true":"false",hit,coplanar,error,budget,filter_ok?"true":"false",endpoint_ok?"true":"false");
    }
    std::printf("VSP_BOUNDARY_FIXTURE passed=%u failed=%u total=%u base_exit=%d\n",passed,failed,passed+failed,base_result);
    return base_result||failed ? 1:0;
}
