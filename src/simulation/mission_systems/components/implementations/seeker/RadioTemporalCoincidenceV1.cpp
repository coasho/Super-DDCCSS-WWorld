#include "RadioTemporalCoincidenceV1.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <unordered_map>

namespace fsim::ms::components {
using namespace measured;
namespace {
bool positive(double x){return std::isfinite(x)&&x>0;}
constexpr std::size_t maxPeaks=4096,maxCells=1048576,maxHypotheses=512;
}
std::optional<RadioPulseWindowV1> radioPulseWindowV1(const radar::signal::Waveform& program,
    const radar::IlluminationClockV1& clock,double first,double capture,double window) {
    if(!clock.illuminating||!std::isfinite(clock.epochS)||!positive(clock.periodS)||clock.periodS<1e-7||clock.periodS>5||
        !std::isfinite(first)||first<0||!std::isfinite(capture)||capture<first||
        !positive(window)||window>.25||program.intervals.empty()||program.intervals.size()>32||
        !positive(program.wavelengthM)||!positive(program.gateM))return std::nullopt;
    RadioPulseWindowV1 result;result.waveform=program;result.waveform.intervals.clear();
    result.observationStartTimeS=std::max(first,capture-window);result.observationEndTimeS=capture;
    result.waveform.rateResolutionMs=0;
    double offset=0;
    for(std::size_t j=0;j<program.intervals.size();++j) {
        const auto& iv=program.intervals[j];
        if(!positive(iv.prfHz)||iv.prfHz<1||iv.prfHz>1e7||iv.pulses<1||iv.pulses>65536||
            !positive(iv.pulseS)||iv.pulseS*iv.prfHz>=1)return std::nullopt;
        // Pulse rounding in the native waveform must not extend a program
        // interval through the next native dwell boundary.
        const double room=clock.periodS-offset-iv.pulseS;
        const double admitted=room>=0?std::min(static_cast<double>(iv.pulses),std::floor(room*iv.prfHz)+1):0;
        bool found=false;double bestStart=0,bestEnd=0;int count=0;
        if(admitted>=4) {
            const double latest=std::floor((capture-clock.epochS-offset)/clock.periodS);
            if(!std::isfinite(latest))return std::nullopt;
            for(int previous=0;previous<2;++previous) {
                const double begin=clock.epochS+(latest-previous)*clock.periodS+offset;
                // No epsilon admits a pulse outside the actual observation.
                double lo=std::max(0.,std::ceil((result.observationStartTimeS-begin)*iv.prfHz));
                double hi=std::min(admitted-1,std::floor((capture-begin-iv.pulseS)*iv.prfHz));
                // Check the actual constructed endpoints. Multiplication
                // followed by ceil/floor can round differently from t=n/PRF
                // exactly at a pulse edge; never import an epsilon of future
                // energy or discard a pulse the explicit clock admits.
                if(lo>0&&begin+(lo-1)/iv.prfHz>=result.observationStartTimeS)--lo;
                if(begin+lo/iv.prfHz<result.observationStartTimeS)++lo;
                if(hi+1<admitted&&begin+(hi+1)/iv.prfHz+iv.pulseS<=capture)++hi;
                if(begin+hi/iv.prfHz+iv.pulseS>capture)--hi;
                if(hi-lo+1>256)return std::nullopt;
                if(hi-lo+1<4)continue;
                const double start=begin+lo/iv.prfHz,end=begin+hi/iv.prfHz+iv.pulseS;
                if(start<result.observationStartTimeS||end>capture)continue;
                if(!found||end>bestEnd){found=true;bestStart=start;bestEnd=end;count=static_cast<int>(hi-lo+1);}
            }
        }
        if(found) {
            auto selected=iv;selected.pulses=count;result.waveform.intervals.push_back(selected);
            result.programIntervals.push_back(static_cast<unsigned>(j));
            result.startTimeS.push_back(bestStart);result.endTimeS.push_back(bestEnd);
            const double energyTime=count/iv.prfHz;result.integratedTimeS+=energyTime;result.energyFraction.push_back(energyTime);
            result.waveform.rateResolutionMs=std::max(result.waveform.rateResolutionMs,program.wavelengthM*iv.prfHz/(2*count));
        }
        offset+=iv.pulses/iv.prfHz;
    }
    if(result.integratedTimeS>0)for(auto& fraction:result.energyFraction)fraction/=result.integratedTimeS;
    return result;
}

void RadioTemporalCoincidenceV1::reset() noexcept {segments_.clear();lastTimeS_=-1;}
bool RadioTemporalCoincidenceV1::update(double capture,double memory,double angularGate,unsigned required,
    const radar::signal::Settings& settings,double gate,const std::vector<RadioTemporalSegmentV1>& incoming,
    std::vector<RadioTemporalDetectionV1>& output) {
    output.clear();
    if(!std::isfinite(capture)||capture<0||capture<=lastTimeS_||!positive(memory)||memory>5||
        !positive(angularGate)||angularGate>=1.5||required<1||required>5||!positive(gate)||
        gate<.01||gate>5000||!positive(settings.rangeScaleM)||settings.rangeScaleM>500000||
        !std::isfinite(settings.minRateMs)||!std::isfinite(settings.maxRateMs)||
        std::abs(settings.minRateMs)>10000||std::abs(settings.maxRateMs)>10000||
        settings.minRateMs>=settings.maxRateMs||settings.coincidence<0||settings.coincidence>2||incoming.size()>32)return false;
    auto next=segments_;
    std::vector<RadioTemporalDetectionV1> accepted;
    next.erase(std::remove_if(next.begin(),next.end(),[&](const auto& s){return capture-s.timeS>memory;}),next.end());
    std::array<bool,32> seen{};
    for(const auto& segment:incoming) {
        if(segment.programInterval>=32||seen[segment.programInterval]||segment.timeS!=capture||
            !positive(segment.rateResolutionMs)||segment.rateResolutionMs<1e-6||
            !positive(segment.waveform.prfHz)||segment.waveform.prfHz<1||segment.waveform.prfHz>1e7||
            !positive(segment.waveform.ambiguousRangeM)||segment.waveform.ambiguousRangeM<.001||segment.waveform.ambiguousRangeM>1e9||
            !positive(segment.waveform.ambiguousRateMs)||segment.waveform.ambiguousRateMs<1e-6||segment.peaks.size()>maxPeaks)return false;
        seen[segment.programInterval]=true;
        for(const auto& p:segment.peaks)if(!std::isfinite(p.foldedRangeM)||p.foldedRangeM<0||
            !std::isfinite(p.foldedRateMs)||!positive(p.snr)||!positive(p.angularVarianceRad2)||
            !finite(p.direction)||std::abs(norm(p.direction)-1)>1e-6)return false;
        const auto old=std::find_if(next.begin(),next.end(),[&](const auto& s){return s.programInterval==segment.programInterval;});
        if(old==next.end())next.push_back(segment);else *old=segment;
    }
    std::sort(next.begin(),next.end(),[](const auto& a,const auto& b){return a.programInterval<b.programInterval;});
    for(std::size_t i=0;i<next.size();++i)for(std::size_t j=0;j<i;++j)
        if(next[i].waveform.prfHz==next[j].waveform.prfHz)return false;
    struct Sample {const RadioTemporalPeakV1* peak;const RadioTemporalSegmentV1* segment;unsigned group;};
    std::vector<Sample> samples;double resolution=0;unsigned nonempty=0;
    for(std::size_t i=0;i<next.size();++i) {
        if(!next[i].peaks.empty()){++nonempty;resolution=std::max(resolution,next[i].rateResolutionMs);}
        for(const auto& p:next[i].peaks)samples.push_back({&p,&next[i],static_cast<unsigned>(i)});
    }
    const auto commit=[&](){segments_=std::move(next);lastTimeS_=capture;output=std::move(accepted);return true;};
    if(nonempty<required||samples.empty())return commit();
    using Key=std::pair<std::int64_t,std::int64_t>;
    struct Hash {std::size_t operator()(const Key& key)const noexcept {
        return std::hash<std::int64_t>{}(key.first)^(std::hash<std::int64_t>{}(key.second)<<1);
    }};
    struct Cell {std::array<int,32> best;Cell(){best.fill(-1);}};
    std::unordered_map<Key,Cell,Hash> cells;
    std::size_t aliases=0;
    for(std::size_t i=0;i<samples.size();++i) {
        const auto& sample=samples[i];const auto& p=*sample.peak;const auto& s=*sample.segment;
        const double age=capture-s.timeS,ra=s.waveform.ambiguousRangeM,va=s.waveform.ambiguousRateMs;
        const double vlo=std::ceil((settings.minRateMs-p.foldedRateMs)/va),vhi=std::floor((settings.maxRateMs-p.foldedRateMs)/va);
        for(double vindex=vlo;vindex<=vhi;++vindex) {
            if(++aliases>maxCells)return false;
            const double velocity=p.foldedRateMs+vindex*va;
            // Unfold first, then migrate by the candidate MEASURED Doppler.
            // No Cartesian target state supplies ambiguity resolution.
            const double rlo=settings.velocityOnly?0:std::max(0.,std::ceil((-p.foldedRangeM-velocity*age)/ra));
            const double rhi=settings.velocityOnly?0:std::floor((settings.rangeScaleM-p.foldedRangeM-velocity*age)/ra);
            for(double rindex=rlo;rindex<=rhi;++rindex) {
                if(++aliases>maxCells)return false;
                const double range=settings.velocityOnly?0:p.foldedRangeM+rindex*ra+velocity*age;
                const auto rc=std::llround(range/gate),vc=std::llround(velocity/resolution);
                const int rangeTolerance=settings.velocityOnly?0:settings.coincidence;
                for(int dr=-rangeTolerance;dr<=rangeTolerance;++dr)
                    for(int dv=-settings.coincidence;dv<=settings.coincidence;++dv) {
                        auto& cell=cells[{rc+dr,vc+dv}];auto& best=cell.best[sample.group];
                        if(best<0||p.snr>samples[static_cast<std::size_t>(best)].peak->snr)best=static_cast<int>(i);
                        if(cells.size()>maxCells)return false;
                    }
            }
        }
    }
    struct Candidate {Key key;std::array<int,32> best;unsigned count=0;double snr=0;};
    std::vector<Candidate> candidates;
    for(const auto& item:cells) {
        Candidate candidate{item.first,item.second.best};int current=-1;
        for(const auto index:candidate.best)if(index>=0) {
            const auto& s=samples[static_cast<std::size_t>(index)];
            if(s.segment->timeS==capture&&(current<0||s.peak->snr>samples[static_cast<std::size_t>(current)].peak->snr))current=index;
        }
        if(current<0)continue;
        const auto ray=samples[static_cast<std::size_t>(current)].peak->direction;
        for(auto& index:candidate.best)if(index>=0) {
            const auto& p=*samples[static_cast<std::size_t>(index)].peak;
            if(dot(ray,p.direction)<std::cos(angularGate)){index=-1;continue;}
            ++candidate.count;candidate.snr+=p.snr;
        }
        if(candidate.count>=required)candidates.push_back(candidate);
    }
    std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){
        if(a.count!=b.count)return a.count>b.count;
        if(a.snr!=b.snr)return a.snr>b.snr;
        return a.key<b.key;
    });
    std::map<std::array<int,32>,std::vector<const Candidate*>> alternatives;
    for(const auto& candidate:candidates)alternatives[candidate.best].push_back(&candidate);
    std::vector<bool> used(samples.size());
    for(const auto& candidate:candidates) {
        bool occupied=false;for(const auto index:candidate.best)if(index>=0&&used[static_cast<std::size_t>(index)])occupied=true;
        if(occupied)continue;
        RadioTemporalDetectionV1 detection;detection.intervals=candidate.count;
        detection.firstObservationS=capture;detection.lastObservationS=capture;
        double w0=0,w1=0,w2=0;V y0{},y1{};int attribution=-1;
        for(const auto index:candidate.best)if(index>=0) {
            const auto& sample=samples[static_cast<std::size_t>(index)];const auto& p=*sample.peak;
            const double dt=sample.segment->timeS-capture,w=1/p.angularVarianceRad2;
            w0+=w;w1+=w*dt;w2+=w*dt*dt;y0=add(y0,scale(p.direction,w));y1=add(y1,scale(p.direction,w*dt));
            detection.snr+=p.snr;detection.firstObservationS=std::min(detection.firstObservationS,sample.segment->timeS);
            if(sample.segment->timeS==capture&&(attribution<0||p.snr>samples[static_cast<std::size_t>(attribution)].peak->snr))attribution=index;
        }
        const double determinant=w0*w2-w1*w1;
        if(determinant>std::numeric_limits<double>::epsilon()*w0*w2*16) {
            detection.direction=unit(scale(subtract(scale(y0,w2),scale(y1,w1)),1/determinant));
            detection.directionRate=scale(subtract(scale(y1,w0),scale(y0,w1)),1/determinant);
            detection.angularVarianceRad2=w2/determinant;
        } else {detection.direction=unit(y0);detection.angularVarianceRad2=1/w0;}
        detection.directionRate=subtract(detection.directionRate,scale(detection.direction,dot(detection.direction,detection.directionRate)));
        if(!finite(detection.direction)||!finite(detection.directionRate)||norm(detection.direction)<.5||
            !positive(detection.angularVarianceRad2))return false;
        for(const auto* alias:alternatives.at(candidate.best)) {
            double range=0,velocity=0,total=0;
            const double rc=static_cast<double>(alias->key.first)*gate,vc=static_cast<double>(alias->key.second)*resolution;
            for(const auto index:alias->best)if(index>=0) {
                const auto& sample=samples[static_cast<std::size_t>(index)];const auto& p=*sample.peak;const auto& iv=sample.segment->waveform;
                const double v=p.foldedRateMs+std::round((vc-p.foldedRateMs)/iv.ambiguousRateMs)*iv.ambiguousRateMs;
                const double drift=v*(capture-sample.segment->timeS);
                const double r=settings.velocityOnly?p.foldedRangeM:
                    p.foldedRangeM+std::round((rc-drift-p.foldedRangeM)/iv.ambiguousRangeM)*iv.ambiguousRangeM+drift;
                range+=p.snr*r;velocity+=p.snr*v;total+=p.snr;
            }
            const std::array<double,2> hypothesis{{range/total,velocity/total}};
            if(std::none_of(detection.hypotheses.begin(),detection.hypotheses.end(),[&](const auto& old){
                return std::abs(old[0]-hypothesis[0])<gate/2&&std::abs(old[1]-hypothesis[1])<resolution/2;
            }))detection.hypotheses.push_back(hypothesis);
            if(detection.hypotheses.size()>maxHypotheses)return false;
        }
        // Metadata is read only AFTER every measurement/association decision.
        if(attribution>=0)detection.refereeAttributionIndex=samples[static_cast<std::size_t>(attribution)].peak->refereeAttributionIndex;
        for(const auto index:candidate.best)if(index>=0)used[static_cast<std::size_t>(index)]=true;
        accepted.push_back(std::move(detection));
    }
    return commit();
}
} // namespace fsim::ms::components
