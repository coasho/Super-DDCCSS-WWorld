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
    bool powerOfTwoExposure=false;
    double invExposure=0.0;
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
        powerOfTwoExposure=(exposureSamples&(exposureSamples-1u))==0u;
        invExposure=powerOfTwoExposure?(1.0/static_cast<double>(exposureSamples)):0.0;
        domain=true;
    }
    bool eligibleOptics() const noexcept {return domain;}

    // Stack-buffer overload avoiding per-trajectory 512 KB heap allocations
    // while preserving the exact nonzero row-major arithmetic and checks.
    // When subOut is non-null, writes the per-subexposure sum without changing
    // the exact cross-subexposure accumulation order in accumulator.
    bool addSampleRange(double& accumulator,const double* xProb,const double* yProb,
        unsigned colBegin,unsigned colEnd,unsigned rowBegin,unsigned rowEnd,
        std::size_t timeIndex,double* subOut=nullptr,bool arithmeticVerified=false) const noexcept {
        if(!domain||(!arithmeticVerified&&!arithmeticV5())||timeIndex>=samples||!xProb||!yProb||
           !std::isfinite(accumulator)||accumulator<0||(accumulator==0&&std::signbit(accumulator)))return false;
        const unsigned cEnd=colEnd<width?colEnd:width;
        const unsigned rEnd=rowEnd<height?rowEnd:height;
        std::array<unsigned,128> columns{},rows{};std::size_t columnCount=0,rowCount=0;
        for(unsigned col=colBegin;col<cEnd;++col) {
            const double value=xProb[col];
            if(!std::isfinite(value)||value<0||value>1)return false;
            if(value!=0)columns[columnCount++]=col;
        }
        for(unsigned row=rowBegin;row<rEnd;++row) {
            const double value=yProb[row];
            if(!std::isfinite(value)||value<0||value>1)return false;
            if(value!=0)rows[rowCount++]=row;
        }
        const double* twBase=timeWeights.data()+timeIndex*pixels;
        double subAcc=0.0;
        if(powerOfTwoExposure) {
            const double invExp=invExposure;
            for(std::size_t ri=0;ri<rowCount;++ri) {
                const auto row=rows[ri];
                const double yv=yProb[row];
                const std::size_t rowOff=static_cast<std::size_t>(row)*width;
                for(std::size_t ci=0;ci<columnCount;++ci) {
                    const auto col=columns[ci];
                    const auto index=rowOff+col;
                    const double tw=twBase[index];
                    if(tw==0.0)continue;
                    const double value=yv*xProb[col]*solidAngles[index]/pixelArea;
                    const double term=value*tw*invExp;
                    accumulator+=term;
                    if(subOut)subAcc+=term;
                }
            }
        } else {
            for(std::size_t ri=0;ri<rowCount;++ri) {
                const auto row=rows[ri];
                const double yv=yProb[row];
                const std::size_t rowOff=static_cast<std::size_t>(row)*width;
                for(std::size_t ci=0;ci<columnCount;++ci) {
                    const auto col=columns[ci];
                    const auto index=rowOff+col;
                    const double tw=twBase[index];
                    if(tw==0.0)continue;
                    const double value=yv*xProb[col]*solidAngles[index]/pixelArea;
                    const double term=value*tw/exposureSamples;
                    accumulator+=term;
                    if(subOut)subAcc+=term;
                }
            }
        }
        if(subOut)*subOut=subAcc;
        return true;
    }

    bool addSampleSlice(double& accumulator,const double* xProb,const double* yProb,
        std::size_t timeIndex,double* subOut=nullptr) const noexcept {
        return addSampleRange(accumulator,xProb,yProb,0u,width,0u,height,timeIndex,subOut,false);
    }

    // False leaves the caller's accumulator untouched and REQUIRES the exact
    // old dense loop. No refusal, threshold, tail cutoff, or replacement value.
    // A double& deliberately preserves addition order ACROSS subexposures.
    bool addSample(double& accumulator,const std::vector<double>& xProbability,
        const std::vector<double>& yProbability,std::size_t timeIndex) const noexcept {
        if(!domain||!arithmeticV5()||timeIndex>=samples||
           xProbability.size()!=samples*width||yProbability.size()!=samples*height)return false;
        return addSampleSlice(accumulator,xProbability.data()+timeIndex*width,
            yProbability.data()+timeIndex*height,timeIndex);
    }
};
} // namespace fsim::ms::components::source_wave_sum_v5
