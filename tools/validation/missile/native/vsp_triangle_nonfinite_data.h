// Independent nonfinite-input rejection contract; no geometric oracle for NaN/Inf.
#pragma once
static const double nonfinite_base[2][3][3] = {
    {{0,0,0},{2,0,0},{0,2,0}},
    {{0.5,0.5,-1},{0.5,0.5,1},{1.5,0.5,0}}
};
static const double nonfinite_start_sentinel[3] = {17,23,31};
static const double nonfinite_end_sentinel[3] = {37,41,43};
static const double finite_segment_ends[2][3] = {{0.5,0.5,0},{1.5,0.5,0}};
