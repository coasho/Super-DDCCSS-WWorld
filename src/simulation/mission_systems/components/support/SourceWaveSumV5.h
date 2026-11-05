#pragma once

// Internal exact-order source-wave summation. This is not a public SDK type.
#include <array>
#include <cfenv>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
#include <xmmintrin.h>
#endif

namespace fsim::ms::components::source_wave_sum_v5 {

inline bool arithmeticV5() noexcept {
#if FLT_EVAL_METHOD != 0 || defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__>0)
    return false;
#endif
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
    if((_mm_getcsr()&0xe040u)!=0)return false; // FTZ, DAZ, or non-nearest.
#else
    return false;
#endif
    return std::numeric_limits<double>::is_iec559&&std::fegetround()==FE_TONEAREST;
}

class OpticsV5 {
    // The inverse borrows these immutable arrays for one synchronous call.
    // No reference escapes the inverse lifetime and no cache is global.
    const std::vector<double>& solidAngles;
    const std::vector<double>& timeWeights;
    const unsigned width,height,exposureSamples;
    const std::size_t pixels,samples;
    const double pixelArea;
    bool domain=false;
public:
    OpticsV5(const std::vector<double>& solid,const std::vector<double>& transmission,
        unsigned columns,unsigned rows,unsigned phases,unsigned exposures,double area):
        solidAngles(solid),timeWeights(transmission),width(columns),height(rows),exposureSamples(exposures),
        pixels(static_cast<std::size_t>(columns)*rows),samples(static_cast<std::size_t>(phases)*exposures),pixelArea(area) {
        if(!arithmeticV5()||width<8||width>128||height<8||height>128||phases<1||phases>512||
           exposureSamples<1||exposureSamples>16||!std::isfinite(pixelArea)||pixelArea<1e-14||pixelArea>1||
           solidAngles.size()!=pixels||timeWeights.size()!=pixels*samples)return;
        for(const auto value:solidAngles)if(!std::isfinite(value)||value<0||value>pixelArea)return;
        for(const auto value:timeWeights)if(!std::isfinite(value)||value<0||value>1)return;
        domain=true;
    }
    bool eligibleOptics() const noexcept {return domain;}

    // False leaves the caller's accumulator untouched and REQUIRES the exact
    // old dense loop. No refusal, threshold, tail cutoff, or replacement value.
    // A double& deliberately preserves addition order ACROSS subexposures.
    bool addSample(double& accumulator,const std::vector<double>& xProbability,
        const std::vector<double>& yProbability,std::size_t timeIndex) const noexcept {
        if(!domain||!arithmeticV5()||timeIndex>=samples||
           xProbability.size()!=samples*width||yProbability.size()!=samples*height||
           !std::isfinite(accumulator)||accumulator<0||(accumulator==0&&std::signbit(accumulator)))return false;
        std::array<unsigned,128> columns{},rows{};std::size_t columnCount=0,rowCount=0;
        // Validate ALL probabilities before skipping ANY term. In particular,
        // a zero in one axis must not conceal NaN/inf in the other axis.
        for(unsigned col=0;col<width;++col) {
            const double value=xProbability[timeIndex*width+col];
            if(!std::isfinite(value)||value<0||value>1)return false;
            if(value!=0)columns[columnCount++]=col;
        }
        for(unsigned row=0;row<height;++row) {
            const double value=yProbability[timeIndex*height+row];
            if(!std::isfinite(value)||value<0||value>1)return false;
            if(value!=0)rows[rowCount++]=row;
        }
        // Ascending row/column lists retain the old row-major NONZERO order.
        // All omitted old terms are exact signed zero. Under nearest, adding
        // either zero is the identity from +0 and every finite positive sum.
        // Keep BOTH native per-term expressions and their operation order.
        for(std::size_t ri=0;ri<rowCount;++ri)for(std::size_t ci=0;ci<columnCount;++ci) {
            const auto row=rows[ri],col=columns[ci];
            const auto index=static_cast<std::size_t>(row)*width+col;
            const double value=yProbability[timeIndex*height+row]*xProbability[timeIndex*width+col]*solidAngles[index]/pixelArea;
            accumulator+=value*timeWeights[timeIndex*pixels+index]/exposureSamples;
        }
        return true;
    }
};
} // namespace fsim::ms::components::source_wave_sum_v5
