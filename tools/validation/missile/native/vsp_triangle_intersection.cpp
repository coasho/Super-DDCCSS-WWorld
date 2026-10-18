// Captured exact-working native intersection and authoritative InTri regression.
#include "TMesh.h"
#include "tri_tri_intersect.h"
#include "vsp_triangle_intersection_data.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

static double distance3( const double *a, const double *b )
{
    return std::hypot( a[0]-b[0], a[1]-b[1], a[2]-b[2] );
}

static bool inside( const double vertices[3][3], const double *point )
{
    TNode nodes[3];
    TTri triangle( nullptr );
    for ( int i=0; i<3; ++i )
    {
        nodes[i].m_Pnt=vec3d( vertices[i][0],vertices[i][1],vertices[i][2] );
        triangle.SetTriNode( i, &nodes[i] );
    }
    return triangle.InTri( vec3d( point[0],point[1],point[2] ) );
}

int main()
{
    unsigned failed=0, passed=0;
    for ( const Case &test : cases )
    {
        double a[3][3], b[3][3];
        std::memcpy( a,test.a,sizeof( a ) ); std::memcpy( b,test.b,sizeof( b ) );
        double start[3]={0,0,0}, end[3]={0,0,0};
        int coplanar=0;
        const int hit=tri_tri_intersection_test_3d( a[0],a[1],a[2],b[0],b[1],b[2],&coplanar,start,end );
        double coordinate=1;
        for ( int i=0; i<3; ++i ) for ( int j=0; j<3; ++j )
        { coordinate=std::max( coordinate,std::abs( a[i][j] ) ); coordinate=std::max( coordinate,std::abs( b[i][j] ) ); }
        const double budget=1024*std::numeric_limits<double>::epsilon()*coordinate;
        bool finite=true;
        for ( int i=0; i<3; ++i ) { finite=finite&&std::isfinite( start[i] )&&std::isfinite( end[i] ); }
        const bool unchanged=std::memcmp( a,test.a,sizeof( a ) )==0&&std::memcmp( b,test.b,sizeof( b ) )==0;
        const bool filters=hit&&!coplanar&&finite&&inside( a,start )&&inside( a,end )&&inside( b,start )&&inside( b,end );
        const double forward=std::max( distance3( start,test.expected[0] ),distance3( end,test.expected[1] ) );
        const double reverse=std::max( distance3( start,test.expected[1] ),distance3( end,test.expected[0] ) );
        const double error=std::min( forward,reverse );
        const bool good=unchanged&&(test.hit ? hit&&!coplanar&&finite&&filters&&error<=budget : !hit);
        if ( good ) { ++passed; } else { ++failed; }
        std::printf( "{\"case\":\"%s\",\"passed\":%s,\"expected_hit\":%s,\"native_hit\":%d,\"coplanar\":%d,\"all_InTri_filters\":%s,\"unchanged_inputs\":%s,\"endpoint_error\":%.17g,\"rounding_budget\":%.17g,\"start\":[%.17g,%.17g,%.17g],\"end\":[%.17g,%.17g,%.17g]}\n",
            test.name,good?"true":"false",test.hit?"true":"false",hit,coplanar,filters?"true":"false",unchanged?"true":"false",error,budget,
            start[0],start[1],start[2],end[0],end[1],end[2] );
    }
    std::printf( "VSP_INTERSECTION_FIXTURE passed=%u failed=%u total=%u\n",passed,failed,passed+failed );
    return failed ? 1 : 0;
}
