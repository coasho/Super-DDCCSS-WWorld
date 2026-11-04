#include "visualization/ui/WeaponDiagnosticsGui.h"
#include "visualization/ui/MandatoryPanel.h"
#include "contracts/equipment/EquipmentModels.h"
#include <vsgImGui/imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace fsim::ui {
namespace {
using namespace fsim::ipc;
void heading(const char* text){ImGui::Separator();ImGui::TextUnformatted(text);}
ImU32 heat(double value){
    const auto t=static_cast<float>(std::clamp(value,0.,1.));
    return ImGui::ColorConvertFloat4ToU32({std::clamp(3*t,0.f,1.f),std::clamp(3*t-1,0.f,1.f),std::clamp(3*t-2,0.f,1.f),1});
}
ImU32 labelColour(std::uint32_t label,std::uint32_t selected){
    if(!label)return IM_COL32(12,16,20,255);
    if(label==selected)return IM_COL32(255,220,40,255);
    const auto value=label*2654435761u;
    return IM_COL32(60+value%160,60+(value>>8)%160,60+(value>>16)%160,255);
}
void vector(const char* label,const double* value,const char* unit){ImGui::Text("%s [%+.6g, %+.6g, %+.6g] %s",label,value[0],value[1],value[2],unit);}
double norm(const double* a){return std::hypot(std::hypot(a[0],a[1]),a[2]);}
}
void WeaponDiagnosticsGui::clear(){
    bandReader_.close();selectedBand_=0;lastBandRequestFrame_=0;nextBandOpen_={};
    reader_.close();frame_.reset();source_={};selectedEntity_=0;selectedGeneration_=selectionRequest_=lastRequestFrame_=0;replay_=false;nextOpen_={};historyCount_=historyNext_=0;
}
void WeaponDiagnosticsGui::setSource(Source source){
    if(replay_||source.world!=source_.world||source.publisherPid!=source_.publisherPid||source.publisherIncarnation!=source_.publisherIncarnation){
        clear();
    }
    source_=std::move(source);
}
bool WeaponDiagnosticsGui::selectWeapon(std::uint32_t entity,std::uint64_t generation) const {
    if(replay_||((entity==0)!=(generation==0)))return false;
    if(entity!=selectedEntity_||generation!=selectedGeneration_){historyCount_=historyNext_=0;selectedBand_=0;lastBandRequestFrame_=0;}
    selectedEntity_=entity;selectedGeneration_=generation;selectionRequest_=0;
    if(!reader_.requestSelection(entity,generation,source_.worldFrame,&selectionRequest_))return false;
    lastRequestFrame_=source_.worldFrame;return true;
}
unsigned WeaponDiagnosticsGui::thermalBandCount() const {
    if(replay_||!source_.publisherAlive||!selectedEntity_)return 0;
    ipc::WeaponThermalBandAvailabilityV1 available;
    if(!bandReader_.availability(source_.worldFrame,available)||available.entity!=selectedEntity_||available.generation!=selectedGeneration_)return 0;
    return available.bandCount;
}
bool WeaponDiagnosticsGui::selectThermalBand(unsigned band) const {
    if(band>=thermalBandCount()||!bandReader_.request(source_.worldFrame,selectedEntity_,selectedGeneration_,band))return false;
    selectedBand_=band;lastBandRequestFrame_=source_.worldFrame;return true;
}
void WeaponDiagnosticsGui::update(){
    if(replay_)return;
    if(!source_.publisherAlive||!source_.publisherPid||!source_.publisherIncarnation){reader_.close();bandReader_.close();lastBandRequestFrame_=0;frame_.reset();return;}
    if(!reader_.valid()){
        const auto now=std::chrono::steady_clock::now();if(now<nextOpen_)return;
        nextOpen_=now+std::chrono::milliseconds(500);
        if(!reader_.open(source_.world,source_.publisherPid,source_.publisherIncarnation)){frame_.reset();return;}
        selectionRequest_=0;
    }
    if(!bandReader_.valid()){
        const auto now=std::chrono::steady_clock::now();
        if(now>=nextBandOpen_){nextBandOpen_=now+std::chrono::milliseconds(500);bandReader_.open(source_.world,source_.publisherPid,source_.publisherIncarnation);lastBandRequestFrame_=0;}
    }
    const auto bandCount=thermalBandCount();
    if(bandCount&&(!lastBandRequestFrame_||source_.worldFrame<lastBandRequestFrame_||source_.worldFrame-lastBandRequestFrame_>100))
        selectThermalBand(selectedBand_<bandCount?selectedBand_:0);
    if(selectedEntity_&&(!selectionRequest_||source_.worldFrame-lastRequestFrame_>100))selectWeapon(selectedEntity_,selectedGeneration_);
    auto pending=std::make_shared<WeaponDiagnosticFrameV1>();
    if(reader_.read(source_.worldFrame,*pending)){
        if((!frame_||pending->worldFrame!=frame_->worldFrame)&&pending->entity==selectedEntity_&&pending->generation==selectedGeneration_)captureHistory(*pending);
        frame_=std::move(pending);
    }
}
bool WeaponDiagnosticsGui::setReplayFrame(const WeaponDiagnosticFrameV1& frame,std::uint64_t expected,double time){
    if(!replay_||!frame_||frame.entity!=frame_->entity||frame.generation!=frame_->generation||frame.worldFrame<=frame_->worldFrame||frame.publisherPid!=frame_->publisherPid||frame.publisherIncarnation!=frame_->publisherIncarnation)historyCount_=historyNext_=0;
    reader_.close();bandReader_.close();lastBandRequestFrame_=0;replay_=true;frame_.reset();
    if(frame.worldFrame!=expected||!std::isfinite(time)||std::abs(frame.simTimeS-time)>1e-9||!validateWeaponDiagnosticsV1(frame))return false;
    source_.world="Recorded weapon diagnostics";source_.publisherPid=frame.publisherPid;source_.publisherIncarnation=frame.publisherIncarnation;
    source_.worldFrame=expected;source_.simTimeS=time;source_.publisherAgeS=0;source_.publisherAlive=false;
    selectedEntity_=frame.entity;selectedGeneration_=frame.generation;selectionRequest_=frame.selectionRequest;
    frame_=std::make_shared<WeaponDiagnosticFrameV1>(frame);captureHistory(frame);return true;
}
bool WeaponDiagnosticsGui::stamp(const char* title,const WeaponDiagnosticStampV1& s) const {
    heading(title);const bool sourceCurrent=replay_||(source_.publisherAlive&&(source_.publisherAgeS<=1||source_.paused)&&frame_&&frame_->worldFrame==source_.worldFrame);
    const bool fresh=sourceCurrent&&weaponDiagnosticFreshV1(s,source_.simTimeS);
    ImGui::TextColored(fresh?ImVec4(.4f,1,.5f,1):ImVec4(1,.65f,.25f,1),"%s%s",weaponDiagnosticStatusTextV1(s.status),s.status==WeaponDiagnosticStatusV1::Available&&!fresh?" - historical / stale":"");
    if(s.cause[0])ImGui::TextWrapped("Cause: %s",s.cause);
    if(s.status!=WeaponDiagnosticStatusV1::Available)return false;
    ImGui::TextDisabled("source %llu / frame %llu | sample %.6f s | age %.3f s",static_cast<unsigned long long>(s.sourceId),static_cast<unsigned long long>(s.sourceFrame),s.sampleTimeS,source_.simTimeS-s.sampleTimeS);
    ImGui::TextDisabled("processed %.6f | delivered %.6f | valid to %.6f s",s.processingTimeS,s.deliveredTimeS,s.validUntilS);
    return true;
}
void WeaponDiagnosticsGui::setReplayUnavailable(std::uint64_t expected,double time){
    reader_.close();bandReader_.close();lastBandRequestFrame_=0;replay_=true;frame_.reset();source_.world="Recorded weapon diagnostics unavailable";
    source_.worldFrame=expected;source_.simTimeS=time;source_.publisherAlive=false;source_.publisherAgeS=0;
}
void WeaponDiagnosticsGui::drawThermal(const WeaponDiagnosticThermalV1& t) const {
    if(!stamp("Thermal focal plane / segmentation",t.stamp)){
        if(t.stamp.status!=WeaponDiagnosticStatusV1::Removed)return;
        ImGui::TextDisabled("Retained capture | source frame %llu | original sample %.6f s",static_cast<unsigned long long>(t.stamp.sourceFrame),t.stamp.sampleTimeS);
    }
    if(t.flags&WeaponDiagnosticSegmentationV1)ImGui::Checkbox("Show native segmentation labels",&segmentation_);
    else {segmentation_=false;ImGui::TextDisabled("Native segmentation mask not exported");}
    if(t.flags&WeaponDiagnosticExpectedPixelsV1)ImGui::Checkbox("Show native expected electrons",&expectedPixels_);
    else expectedPixels_=false;
    const auto count=static_cast<std::size_t>(t.width)*t.height;
    if(!count)return;
    const auto* pixels=expectedPixels_?t.expectedElectrons:t.measuredElectrons;
    const auto limits=std::minmax_element(pixels,pixels+count);
    const double low=(t.flags&WeaponDiagnosticNativeDisplayScaleV1)?t.displayMinimumElectrons:*limits.first;
    const double high=(t.flags&WeaponDiagnosticNativeDisplayScaleV1)?t.displayMaximumElectrons:*limits.second,span=high-low;
    ImGui::Text("%u x %u | field +/-(%.2f, %.2f) deg",t.width,t.height,t.horizontalHalfAngleRad*180/std::acos(-1.),t.verticalHalfAngleRad*180/std::acos(-1.));
    ImGui::Text("exposure %.6g s | full well %.6g electrons | ADC %u bit",t.exposureS,t.fullWellElectrons,t.adcBits);
    ImGui::Text("detection %.6g sigma | read noise %.6g electrons | dark %.6g electrons/s",t.detectionSigma,t.readNoiseElectrons,t.darkCurrentElectronsS);
    const float width=std::max(80.f,ImGui::GetContentRegionAvail().x),height=width*static_cast<float>(t.height)/static_cast<float>(t.width);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,ImVec2(0,0));ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,ImVec2(0,0));
    // Separate draw lists keep even a full 128x128 native image within a
    // 16-bit ImGui index buffer. Every native cell is drawn; no resampling.
    for(std::uint32_t first=0;first<t.height;first+=32){
        const auto rows=std::min(32u,t.height-first);const float ch=height/static_cast<float>(t.height),cw=width/static_cast<float>(t.width);
        ImGui::PushID(static_cast<int>(first));
        ImGui::BeginChild("thermal-native-rows",ImVec2(width,ch*static_cast<float>(rows)),false,ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoScrollWithMouse);
        const auto origin=ImGui::GetCursorScreenPos();auto* dl=ImGui::GetWindowDrawList();
        ImGui::InvisibleButton("thermal-cells",ImVec2(width,ch*static_cast<float>(rows)));
        for(std::uint32_t row=0;row<rows;++row)for(std::uint32_t column=0;column<t.width;++column){
            const auto index=static_cast<std::size_t>(first+row)*t.width+column;
            const auto colour=segmentation_?labelColour(t.segmentation[index],t.selectedBlob):heat(span>0?(pixels[index]-low)/span:0);
            const float x=origin.x+static_cast<float>(column)*cw,y=origin.y+static_cast<float>(row)*ch;
            dl->AddRectFilled({x,y},{x+cw,y+ch},colour);
        }
        if((t.flags&WeaponDiagnosticCentroidV1)&&t.centroidRow>=first&&t.centroidRow<first+rows){
            const float x=origin.x+static_cast<float>(t.centroidColumn+.5)*cw,y=origin.y+static_cast<float>(t.centroidRow-static_cast<double>(first)+.5)*ch;
            dl->AddLine({x-7,y},{x+7,y},IM_COL32(40,255,220,255),2);dl->AddLine({x,y-7},{x,y+7},IM_COL32(40,255,220,255),2);
        }
        if(ImGui::IsItemHovered()){
            const auto mouse=ImGui::GetIO().MousePos;
            const auto column=std::min(t.width-1,static_cast<std::uint32_t>(std::max(0.f,(mouse.x-origin.x)/cw)));
            const auto row=first+std::min(rows-1,static_cast<std::uint32_t>(std::max(0.f,(mouse.y-origin.y)/ch)));
            const auto i=static_cast<std::size_t>(row)*t.width+column;
            ImGui::BeginTooltip();ImGui::Text("native pixel (%u,%u)\nmeasured %.9g electrons/pixel/exposure",column,row,static_cast<double>(t.measuredElectrons[i]));
            if(t.flags&WeaponDiagnosticExpectedPixelsV1)ImGui::Text("expected %.9g electrons/pixel/exposure",static_cast<double>(t.expectedElectrons[i]));
            if(t.flags&WeaponDiagnosticSegmentationV1)ImGui::Text("native processor blob %u",t.segmentation[i]);
            ImGui::EndTooltip();
        }
        ImGui::EndChild();ImGui::PopID();
    }
    ImGui::PopStyleVar(2);
    ImGui::TextDisabled("Palette %.6g .. %.6g electrons/pixel/exposure; row zero at top. Display scale only.",low,high);
    if(t.flags&WeaponDiagnosticCentroidV1)ImGui::Text("Strongest current detection centroid (%.3f, %.3f) px | variance %.6g rad^2",t.centroidColumn,t.centroidRow,t.centroidVarianceRad2);
    else ImGui::TextDisabled("No delivered native centroid");
    if(t.flags&WeaponDiagnosticCentroidV1)ImGui::Text("Native SNR amplitude %.6g | signed contrast %.6g electrons",t.centroidSnrAmplitude,t.centroidSignedContrastElectrons);
    if(t.flags&WeaponDiagnosticThermalExtentV1)ImGui::Text("Native component extent %.6g x %.6g rad",t.centroidWidthRad,t.centroidHeightRad);
    if(t.flags&WeaponDiagnosticQuadrantsV1)ImGui::Text("Native quadrant contrasts [%.6g %.6g %.6g %.6g] electrons\nGuard background %.6g electrons | total variance %.6g electrons^2",t.quadrantContrastElectrons[0],t.quadrantContrastElectrons[1],t.quadrantContrastElectrons[2],t.quadrantContrastElectrons[3],t.quadrantGuardBackgroundElectrons,t.quadrantVarianceElectrons2);
    ImGui::TextDisabled("Raw detection is not delivered association; guidance tracking/LOS has separate epochs above.");
}
void WeaponDiagnosticsGui::drawRf(const WeaponDiagnosticRfV1& r) const {
    if(!stamp("RF range-Doppler / CFAR",r.stamp))return;
    ImGui::Text("%.3f GHz | PRF %.6g Hz | bandwidth %.6g Hz",r.frequencyHz/1e9,r.prfHz,r.bandwidthHz);
    ImGui::Text("%u x %u | kept-interval displayed peaks %u | illumination %s",r.rows,r.columns,r.peakCount,(r.flags&WeaponDiagnosticIlluminatedV1)?"present":"unavailable");
    ImGui::TextDisabled("Native linear power / mean thermal noise per filter (not watts)");
    if(r.flags&WeaponDiagnosticRfStatsV1)ImGui::Text("Across dwell intervals: cells %llu | threshold peaks %llu | accepted hits %llu | processing %.3f ms",static_cast<unsigned long long>(r.processedCells),static_cast<unsigned long long>(r.thresholdPeakCount),static_cast<unsigned long long>(r.acceptedHitCount),r.processingWallS*1e3);
    if(!(r.flags&WeaponDiagnosticRangeAxisV1))ImGui::TextColored({1,.7f,.2f,1},"Range axis unavailable; rows are native processing bins");
    const float width=std::max(80.f,ImGui::GetContentRegionAvail().x),cw=width/static_cast<float>(r.columns),ch=std::max(.25f,220.f/static_cast<float>(r.rows));
    const auto stripRows=std::max(1u,4096u/r.columns);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,ImVec2(0,0));ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,ImVec2(0,0));
    for(std::uint32_t first=0;first<r.rows;first+=stripRows){
        const auto rows=std::min(stripRows,r.rows-first);ImGui::PushID(static_cast<int>(first));
        ImGui::BeginChild("native-rf-grid-rows",{width,ch*static_cast<float>(rows)},false,ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoScrollWithMouse);
        const auto origin=ImGui::GetCursorScreenPos();ImGui::InvisibleButton("rf-cells",{width,ch*static_cast<float>(rows)});auto* dl=ImGui::GetWindowDrawList();
        for(std::uint32_t displayed=0;displayed<rows;++displayed)for(std::uint32_t column=0;column<r.columns;++column){
            const auto row=r.rows-1-first-displayed;
            const auto index=static_cast<std::size_t>(row)*r.columns+column;
            const double level=std::clamp(10*std::log10(std::max(static_cast<double>(r.powerNoiseRelative[index]),1e-30))/40,0.,1.);
            const float x=origin.x+static_cast<float>(column)*cw,y=origin.y+static_cast<float>(displayed)*ch;
            dl->AddRectFilled({x,y},{x+cw,y+ch},((r.flags&WeaponDiagnosticRfTestedCellsV1)&&!r.tested[index])?IM_COL32(32,36,44,255):heat(level));
        }
        for(std::uint32_t i=0;i<r.peakCount;++i){const auto& p=r.peaks[i];const auto displayed=r.rows-1-p.row;if(displayed>=first&&displayed<first+rows)dl->AddCircle({origin.x+(static_cast<float>(p.column)+.5f)*cw,origin.y+(static_cast<float>(displayed-first)+.5f)*ch},4,IM_COL32(255,70,220,255),12,1.5f);}
        if(ImGui::IsItemHovered()){
            const auto mouse=ImGui::GetIO().MousePos;
            const auto column=std::min(r.columns-1,static_cast<std::uint32_t>(std::max(0.f,(mouse.x-origin.x)/cw)));
            const auto row=r.rows-1-first-std::min(rows-1,static_cast<std::uint32_t>(std::max(0.f,(mouse.y-origin.y)/ch)));
            const auto index=static_cast<std::size_t>(row)*r.columns+column;
            ImGui::BeginTooltip();ImGui::Text("native cell (%u,%u)",row,column);
            if(r.flags&WeaponDiagnosticRangeAxisV1)ImGui::Text("%s %.6g m",(r.flags&WeaponDiagnosticBistaticDelayAxisV1)?"bistatic delay-equivalent, not target range":"folded range",r.firstRangeM+(row+.5)*r.rangeBinM);
            if(r.flags&WeaponDiagnosticDopplerAxisV1)ImGui::Text("closing %.6g m/s",r.firstClosingMs+(column+.5)*r.closingBinMs);
            ImGui::Text("power %.9g x mean thermal-noise/filter",static_cast<double>(r.powerNoiseRelative[index]));
            const bool tested=!(r.flags&WeaponDiagnosticRfTestedCellsV1)||r.tested[index]!=0;
            if(!tested)ImGui::TextDisabled("Native processor excluded this cell; thresholds unavailable");
            if(tested&&(r.flags&WeaponDiagnosticRfLocalNoiseV1))ImGui::Text("native local noise %.9g (same units)",static_cast<double>(r.localNoiseRelative[index]));
            else ImGui::TextDisabled("Native local CFAR noise not exported");
            if(tested&&(r.flags&WeaponDiagnosticRfThresholdsV1))ImGui::Text("native decision threshold %.9g (CFAR/clutter, same units)",static_cast<double>(r.thresholdNoiseRelative[index]));
            else ImGui::TextDisabled("Native CFAR threshold matrix not exported");
            ImGui::EndTooltip();
        }
        ImGui::EndChild();ImGui::PopID();
    }
    ImGui::PopStyleVar(2);
    ImGui::TextDisabled("Native range rows ascend upward; Doppler columns ascend rightward.");
    for(std::uint32_t i=0;i<r.peakCount;++i){const auto& p=r.peaks[i];ImGui::Text("Native peak %u cell %u/%u: power %.6g x mean noise/filter",i,p.row,p.column,p.powerNoiseRelative);if(p.flags&WeaponDiagnosticRfThresholdsV1)ImGui::Text("Native threshold %.6g (same units)",p.thresholdNoiseRelative);}
}
void WeaponDiagnosticsGui::drawNavigation(const WeaponDiagnosticNavigationV1& n) const {
    if(!stamp("Strapdown attitude / navigation / covariance",n.stamp))return;
    ImGui::Text("body -> ECEF q [%.7f, %.7f, %.7f, %.7f]",n.quaternionBodyToEcef[0],n.quaternionBodyToEcef[1],n.quaternionBodyToEcef[2],n.quaternionBodyToEcef[3]);
    {
        const auto origin=ImGui::GetCursorScreenPos();ImGui::InvisibleButton("native-attitude-triad",{180,140});auto* dl=ImGui::GetWindowDrawList();
        const ImVec2 center(origin.x+90,origin.y+75);const auto* q=n.quaternionBodyToEcef;
        const double c[3][3]={{1-2*(q[2]*q[2]+q[3]*q[3]),2*(q[1]*q[2]+q[0]*q[3]),2*(q[1]*q[3]-q[0]*q[2])},
            {2*(q[1]*q[2]-q[0]*q[3]),1-2*(q[1]*q[1]+q[3]*q[3]),2*(q[2]*q[3]+q[0]*q[1])},
            {2*(q[1]*q[3]+q[0]*q[2]),2*(q[2]*q[3]-q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2])}};
        const ImU32 colours[]={IM_COL32(255,80,80,255),IM_COL32(80,255,120,255),IM_COL32(90,150,255,255)};const char* labels[]={"X body","Y body","Z body"};
        for(unsigned axis=0;axis<3;++axis){const ImVec2 end(center.x+static_cast<float>(.8*c[axis][0]-.6*c[axis][1])*60,center.y+static_cast<float>(.3*c[axis][0]+.4*c[axis][1]-.8660254037844386*c[axis][2])*60);dl->AddLine(center,end,colours[axis],2);dl->AddCircleFilled(end,3,colours[axis],8);dl->AddText(end,colours[axis],labels[axis]);}
        ImGui::SameLine();ImGui::TextWrapped("Native estimated body axes\nFixed ECEF isometric view\nNo UI attitude integration");
    }
    vector("Position ECEF",n.positionEcefM,"m");vector("Velocity ECEF",n.velocityEcefMs,"m/s");
    vector("Measured body gyro",n.measuredGyroBodyRadS,"rad/s");vector("Measured specific force",n.measuredSpecificForceBodyMs2,"m/s^2");
    vector("Estimated accelerometer bias",n.accelerometerBiasBodyMs2,"m/s^2");vector("Estimated gyro bias",n.gyroBiasBodyRadS,"rad/s");
    ImGui::Text("GNSS fitted %s / aided %s | last IMU %.6f s",(n.flags&WeaponDiagnosticGnssFittedV1)?"yes":"no",(n.flags&WeaponDiagnosticGnssAidedV1)?"yes":"no",n.lastImuTimeS);
    if(n.flags&WeaponDiagnosticGnssEverSampledV1)ImGui::Text("Last GNSS exposure %.6f s%s",n.lastGnssTimeS,(n.flags&WeaponDiagnosticGnssAidedV1)?"":" (retained, currently unaided)");
    else ImGui::TextDisabled("No GNSS exposure delivered");
    const char* labels[]={"Position","Velocity","Attitude error","Accel bias","Gyro bias"};const char* units[]={"m","m/s","rad","m/s^2","rad/s"};
    for(unsigned group=0;group<5;++group){double sigma[3]{};for(unsigned i=0;i<3;++i){const auto axis=3*group+i;sigma[i]=std::sqrt(n.errorCovariance[n.errorStateCount*axis+axis]);}vector(labels[group],sigma,units[group]);}
    ImGui::TextDisabled("Above: native 1-sigma covariance, not measured hardware accuracy.");
    if(n.flags&WeaponDiagnosticOwnReferenceV1){double error[3]{};for(unsigned i=0;i<3;++i)error[i]=n.positionEcefM[i]-n.referencePositionEcefM[i];ImGui::TextColored({1,.7f,.2f,1},"REFEREE ownship comparison: position error %.6g m",norm(error));}
    // Full covariance correlation is a display transformation of native P.
    const float size=180,cell=size/static_cast<float>(n.errorStateCount);const auto origin=ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("native-nav-covariance",{size,size});auto* dl=ImGui::GetWindowDrawList();
    for(std::uint32_t i=0;i<n.errorStateCount;++i)for(std::uint32_t j=0;j<n.errorStateCount;++j){
        const double variance=n.errorCovariance[n.errorStateCount*i+i]*n.errorCovariance[n.errorStateCount*j+j];
        const double correlation=variance>0?n.errorCovariance[n.errorStateCount*i+j]/std::sqrt(variance):0;
        const float t=static_cast<float>(std::clamp(std::abs(correlation),0.,1.));
        const auto colour=correlation>=0?ImGui::ColorConvertFloat4ToU32({t,.1f,.1f,1}):ImGui::ColorConvertFloat4ToU32({.1f,.2f,t,1});
        dl->AddRectFilled({origin.x+static_cast<float>(j)*cell,origin.y+static_cast<float>(i)*cell},{origin.x+static_cast<float>(j+1)*cell,origin.y+static_cast<float>(i+1)*cell},colour);
    }
    ImGui::SameLine();ImGui::TextWrapped("Native %u-state correlation\nred positive / blue negative\ndp,dv,dtheta,dba,dbg,[held wa,wg]",n.errorStateCount);
}
void WeaponDiagnosticsGui::drawGuidanceLink(const WeaponDiagnosticGuidanceV1& g,const WeaponDiagnosticLinkV1& l) const {
    if(stamp("Guidance / support / delivered seeker",g.stamp)){
        const char* phases[]={"unguided","midcourse","seeking","homing","lost"};
        ImGui::Text("Native phase %u (%s) | weapon age %.6f s",g.phase,g.phase<5?phases[g.phase]:"unknown",g.weaponAgeS);
        ImGui::Text("Active %s | tracking %s | native support %s | midcourse %s",(g.flags&WeaponDiagnosticSeekerActiveV1)?"yes":"no",(g.flags&WeaponDiagnosticGuidanceTrackingV1)?"yes":"no",(g.flags&WeaponDiagnosticSupportedV1)?"yes":"no",(g.flags&WeaponDiagnosticMidcourseV1)?"yes":"no");
        if(g.flags&WeaponDiagnosticTimeToGoV1)ImGui::Text("Native estimated time to go %.6f s",g.timeToGoS);
        if(g.flags&WeaponDiagnosticTimeToActiveV1)ImGui::Text("Native time to active %.6f s",g.timeToActiveS);
        if(stamp("Original seeker measurement epochs",g.measurementStamp)){
            if(g.flags&WeaponDiagnosticLosV1){
                vector("Admitted navigation-resolved LOS",g.losDirectionEcef,"ECEF unit direction");
                ImGui::Text("Local seeker track %u | original track epoch %.6f s",g.localTrackId,g.trackTimeS);
            }else ImGui::TextDisabled("Delivered LOS unavailable");
            if(g.flags&WeaponDiagnosticLosRateV1)vector("Measured LOS derivative",g.losDirectionRatePerS,"ECEF / s");
            else ImGui::TextDisabled("Measured LOS rate unavailable");
            if(g.flags&WeaponDiagnosticMeasuredRangeV1)ImGui::Text("Measured range %.6g m",g.rangeM);
            else ImGui::TextDisabled("Measured terminal range unavailable (no truth substitute)");
            if(g.flags&WeaponDiagnosticMeasuredRangeRateV1)ImGui::Text("Measured range rate %.6g m/s",g.rangeRateMs);
            if(g.flags&WeaponDiagnosticAngularVarianceV1)ImGui::Text("Native measured angular variance %.6g rad^2",g.angularVarianceRad2);
        }
    }
    if(!stamp("Actual weapon link / accepted delivered input",l.stamp))return;
    const char* directions[]={"unsupported / no energized radio","uplink only","exact-fit two way"};
    ImGui::Text("Direction %s | native channel %s",directions[l.direction],(l.flags&WeaponDiagnosticChannelOpenV1)?"open":"closed");
    if(l.directionSource[0])ImGui::TextWrapped("Fit source: %s",l.directionSource);
    if(l.direction)ImGui::Text("Engineering policy: update %.6g s / feedback %.6g s / latency %.6g s / %.6g bit/s",l.updatePeriodS,l.feedbackPeriodS,l.latencyS,l.dataRateBps);
    if(l.flags&WeaponDiagnosticRadioResourceV1)ImGui::Text("Native radio resource busy until %.6f s",l.resourceBusyUntilS);
    if(l.flags&WeaponDiagnosticChannelStatsV1){
        ImGui::Text("Native sent %llu / delivered %llu / lost %llu / bytes %llu",static_cast<unsigned long long>(l.sent),static_cast<unsigned long long>(l.delivered),static_cast<unsigned long long>(l.lost),static_cast<unsigned long long>(l.bytes));
        ImGui::Text("masked %llu / range %llu / queue refused %llu / expired %llu / rejected %llu",static_cast<unsigned long long>(l.masked),static_cast<unsigned long long>(l.outOfRange),static_cast<unsigned long long>(l.queueRefused),static_cast<unsigned long long>(l.expired),static_cast<unsigned long long>(l.rejected));
    }
    if(!(l.flags&WeaponDiagnosticInputKnownV1)){ImGui::TextDisabled("No accepted delivered fused input");return;}
    ImGui::Text("Accepted fused entity %llu / state %u / %s",static_cast<unsigned long long>(l.fusedEntity),l.fusionState,(l.flags&WeaponDiagnosticInputValidV1)?"valid":"stale / lost");
    ImGui::Text("state epoch %.6f | last update %.6f | radio received %.6f | expires %.6f s",l.stateTimeS,l.lastUpdateS,l.inputReceivedS,l.inputValidUntilS);
    const double position[3]={l.stateEcef[0],l.stateEcef[2],l.stateEcef[4]},velocity[3]={l.stateEcef[1],l.stateEcef[3],l.stateEcef[5]};
    vector("Accepted uplink estimate ECEF",position,"m");vector("Accepted uplink velocity ECEF",velocity,"m/s");
    ImGui::TextDisabled("Original accepted fusion provenance; IDs below are track/service IDs, not target truth:");
    if(l.sourceCount){
        ImGui::BeginChild("native-uplink-provenance",{0,100},true);
        for(std::uint32_t i=0;i<l.sourceCount;++i){const auto& s=l.sources[i];ImGui::Text("%s system %u / payload %u / track %llu",s.kind==0?"local radar":"delivered platform link",s.system,s.payload,static_cast<unsigned long long>(s.track));ImGui::Text("sample %.6f / received %.6f s%s",s.sampleTimeS,s.receivedTimeS,(s.flags&1)?" | diagonal link covariance":"");}
        ImGui::EndChild();
    }else ImGui::TextDisabled("No source provenance rows available");
}
void WeaponDiagnosticsGui::drawControl(const WeaponDiagnosticControlV1& c) const {
    if(!stamp("Control demand / physical response",c.stamp))return;
    if(c.flags&WeaponDiagnosticControllerV1)ImGui::Text("Native controller epoch %.6f s; physical fin epoch %.6f s",c.controllerTimeS,c.stamp.sampleTimeS);
    if(c.flags&WeaponDiagnosticControlNavigationEpochV1)ImGui::Text("Controller read own navigation at %.6f s; solution age %.6g s",c.navigationTimeS,c.controllerTimeS-c.navigationTimeS);
    if(c.flags&WeaponDiagnosticAccelerationDemandV1)vector("Demand body acceleration",c.demandedAccelerationBodyMs2,"m/s^2");
    else ImGui::TextDisabled("Acceleration demand unavailable");
    if(c.flags&WeaponDiagnosticAccelerationAchievedV1)vector("Achieved body acceleration",c.achievedAccelerationBodyMs2,"m/s^2");
    else ImGui::TextDisabled("Achieved acceleration unavailable");
    if(c.flags&WeaponDiagnosticMomentDemandV1)vector("Demand moment",c.demandedMomentBodyNm,"N m");
    if(c.flags&WeaponDiagnosticMomentAchievedV1)vector("Achieved moment",c.achievedMomentBodyNm,"N m");
    if(c.flags&WeaponDiagnosticAllocatorMetricsV1)ImGui::Text("Native allocator iterations %u / reason %u / residual %.6g",c.allocatorIterations,c.allocatorReason,c.allocatorResidual);
    for(std::uint32_t i=0;i<c.finCount;++i){const auto& f=c.fins[i];
        if(!(f.flags&1)){ImGui::TextDisabled("Native fin %u unavailable",f.index);continue;}
        const float position=static_cast<float>((f.actualRad-f.minimumRad)/(f.maximumRad-f.minimumRad));
        char overlay[192];std::snprintf(overlay,sizeof(overlay),"fin %u demand %.4f / actual %.4f rad | rate %.3f rad/s | flags 0x%x",f.index,f.demandRad,f.actualRad,f.rateRadS,f.flags);ImGui::ProgressBar(std::clamp(position,0.f,1.f),{-1,0},overlay);
        const auto low=ImGui::GetItemRectMin(),high=ImGui::GetItemRectMax();const float demand=static_cast<float>(std::clamp((f.demandRad-f.minimumRad)/(f.maximumRad-f.minimumRad),0.,1.));
        const float x=low.x+(high.x-low.x)*demand;ImGui::GetWindowDrawList()->AddLine({x,low.y},{x,high.y},IM_COL32(255,220,80,255),2);
    }
    if(c.flags&WeaponDiagnosticTvcV1)ImGui::Text("TVC pitch demand/actual %.5f / %.5f rad; yaw %.5f / %.5f rad\nlimit %.5f rad / rate %.5f rad/s",c.tvcDemandPitchRad,c.tvcActualPitchRad,c.tvcDemandYawRad,c.tvcActualYawRad,c.tvcLimitRad,c.tvcRateRadS);
    else ImGui::TextDisabled("No configured native TVC in this snapshot");
}
void WeaponDiagnosticsGui::drawPropulsion(const WeaponDiagnosticPropulsionV1& p) const {
    if(!stamp("Propulsion / inlet / ignition",p.stamp))return;
    if(p.flags&WeaponDiagnosticMotorClockV1)ImGui::Text("Actual native motor age %.6f s",p.nativeMotorAgeS);
    else ImGui::TextDisabled("Native motor clock unavailable");
    if(p.flags&WeaponDiagnosticPulsePolicyV1) {
        const char* state="unavailable / invalid";
        switch(p.pulseState) {
        case 1:state="first burn / coast";break;case 2:state="holding reserve";break;
        case 3:state="ignition requested";break;case 4:state="request window expired";break;
        case 5:state="no fuel";break;default:break;
        }
        ImGui::Text("Native reserve-pulse decision: %s",state);
    }
    if(p.flags&WeaponDiagnosticFuelV1)ImGui::Text("Actual fuel %.6g kg",p.fuelKg);
    for(std::uint32_t i=0;i<p.engineCount;++i){ImGui::Text("Engine %u native scalar thrust %.6g N",i,p.thrustN[i]);if(p.flags&WeaponDiagnosticFuelFlowV1)ImGui::Text("Fuel flow %.6g kg/s",p.fuelFlowKgS[i]);}
    if(p.flags&WeaponDiagnosticNetAxialForceV1)ImGui::Text("Actual total body +X propulsion force %.6g N (ram / inventory reactions included)",p.netThrustN);
    else ImGui::TextDisabled("Net body +X propulsion force unavailable; engine scalars are not its substitute");
    ImGui::Text("Burning %s",(p.flags&WeaponDiagnosticBurningV1)?"yes":"no");
    if(p.flags&WeaponDiagnosticPulsePolicyV1){
        ImGui::Text("Native second command %s",(p.flags&WeaponDiagnosticSecondCommandV1)?"issued":"not issued");
        if(p.flags&WeaponDiagnosticSecondIgnitedV1)ImGui::Text("Actual second ignition at native age %.6f s",p.secondIgnitionAgeS);
        else ImGui::TextDisabled("No actual second ignition recorded");
    } else ImGui::TextDisabled("No admitted commanded reserve-pulse policy in this snapshot");
    if(p.flags&WeaponDiagnosticInletV1)ImGui::Text("Inlet %s | air %.6g kg/s | T %.6g K | combustor %.6g K",(p.flags&WeaponDiagnosticInletLitV1)?"lit":"unlit",p.inletAirKgS,p.inletTemperatureK,p.combustorTemperatureK);
    if(p.flags&WeaponDiagnosticValveV1)ImGui::Text("Valve demand/actual %.6g / %.6g",p.valveDemand,p.valveActual);
}
void WeaponDiagnosticsGui::drawEffects(const WeaponDiagnosticEffectsV1& e) const {
    if(!stamp("Fuze / physical source / component damage",e.stamp))return;
    if(e.flags&WeaponDiagnosticFuzeV1)ImGui::Text("Fuze %u | armed %s | arming %.6f s | radius %.6g m",e.fuzeAlgorithm,(e.flags&WeaponDiagnosticArmedV1)?"yes":"no",e.armingTimeS,e.fuzeRadiusM);
    else ImGui::TextDisabled("Native fuze state unavailable");
    if(e.flags&WeaponDiagnosticWarheadV1)ImGui::Text("Native warhead topology %u%s | explosive %.6g kg / case %.6g kg",e.warheadTopology,(e.flags&WeaponDiagnosticConnectedRingV1)?" connected ring":"",e.explosiveKg,e.caseMassKg);
    if(e.flags&WeaponDiagnosticBurstV1)ImGui::Text("Actual burst %.6f s | current native particles shown %u / %u",e.burstTimeS,e.fragmentCount,e.totalFragmentCount);
    else ImGui::TextDisabled("No native burst recorded; no explosion inferred");
    if(e.totalFragmentCount>e.fragmentCount)ImGui::TextDisabled("Bounded native fragment subset shown; full population total retained");
    if(e.fragmentCount){
        const float size=180;const auto origin=ImGui::GetCursorScreenPos();ImGui::InvisibleButton("native-source-fragments",{size,size});auto* dl=ImGui::GetWindowDrawList();double radius=1;
        for(std::uint32_t i=0;i<e.fragmentCount;++i){double dx=e.fragments[i].positionEcefM[0]-e.burstPositionEcefM[0],dy=e.fragments[i].positionEcefM[1]-e.burstPositionEcefM[1];radius=std::max(radius,std::hypot(dx,dy));}
        dl->AddRectFilled(origin,{origin.x+size,origin.y+size},IM_COL32(10,14,18,255));
        for(std::uint32_t i=0;i<e.fragmentCount;++i){const auto& f=e.fragments[i];const float x=origin.x+size*.5f+static_cast<float>((f.positionEcefM[0]-e.burstPositionEcefM[0])/radius)*size*.45f,y=origin.y+size*.5f-static_cast<float>((f.positionEcefM[1]-e.burstPositionEcefM[1])/radius)*size*.45f;dl->AddCircleFilled({x,y},2,IM_COL32(255,180,60,255),6);}
        ImGui::SameLine();ImGui::TextWrapped("Actual native positions\nEpoch %.6f s\nECEF X/Y projection\n+/-%.6g m around burst\nNo client trajectories",e.fragments[0].sampleTimeS,radius);
    }
    ImGui::Text("Distinct displayed components %u | native component-impact records %u",e.damageCount,e.totalDamageCount);
    for(std::uint32_t i=0;i<e.damageCount;++i){const auto& d=e.damage[i];ImGui::Text("victim %u gen %llu / component %u | last native application %.6f s",d.victim,static_cast<unsigned long long>(d.victimGeneration),d.component,d.sampleTimeS);
        if(d.flags&WeaponDiagnosticImpactCountV1)ImGui::Text("Native impact count %u",d.hitCount);else ImGui::TextDisabled("Impact count unavailable");
        if(d.flags&WeaponDiagnosticImpactEnergyV1)ImGui::Text("Native penetrated energy %.6g J",d.penetratedEnergyJ);else ImGui::TextDisabled("Impact energy unavailable");
        if(d.flags&WeaponDiagnosticComponentHealthV1)ImGui::Text("Native component health %.6g",d.health);else ImGui::TextDisabled("Continuous component health unavailable");
        if(d.flags&WeaponDiagnosticRemovedMassV1)ImGui::Text("Actual removed mass %.6g kg",d.removedMassKg);else ImGui::TextDisabled("Removed mass unavailable");
    }
}
void WeaponDiagnosticsGui::drawEvents(const WeaponDiagnosticFrameV1& f) const {
    heading("Native timeline / causes");
    if(!f.eventCount){ImGui::TextDisabled("No native events in this bounded snapshot");return;}
    for(std::uint32_t i=0;i<f.eventCount;++i) {
        const auto& e=f.events[i];
        const auto source=std::to_string(e.source)+":"+((e.flags&ipc::WeaponDiagnosticEventSourceGenerationV1)?std::to_string(e.sourceGeneration):"unknown");
        const auto target=std::to_string(e.target)+":"+((e.flags&ipc::WeaponDiagnosticEventTargetGenerationV1)?std::to_string(e.targetGeneration):"unknown");
        ImGui::TextWrapped("%.6f s | #%llu | %s -> %s | %s",e.timeS,static_cast<unsigned long long>(e.serial),source.c_str(),target.c_str(),e.cause);
        if(e.flags&ipc::WeaponDiagnosticEventCodeV1)ImGui::Text("Native code %u",e.code);
        if(e.flags&ipc::WeaponDiagnosticEventValueV1)ImGui::Text("Native value %.6g %s",e.value,e.unit);
    }
}
void WeaponDiagnosticsGui::captureHistory(const WeaponDiagnosticFrameV1& frame){
    History sample;sample.timeS=frame.simTimeS;
    if(frame.propulsion.stamp.status==WeaponDiagnosticStatusV1::Available){
        if(frame.propulsion.engineCount){for(std::uint32_t i=0;i<frame.propulsion.engineCount;++i)sample.thrustN+=frame.propulsion.thrustN[i];if(std::isfinite(sample.thrustN))sample.availability|=1;}
        if(frame.propulsion.flags&WeaponDiagnosticFuelV1){sample.fuelKg=frame.propulsion.fuelKg;sample.availability|=2;}
    }
    if(frame.navigation.stamp.status==WeaponDiagnosticStatusV1::Available){const auto& n=frame.navigation;double variance=0;for(unsigned i=0;i<3;++i)variance+=n.errorCovariance[n.errorStateCount*i+i];if(std::isfinite(variance)){sample.positionSigmaM=std::sqrt(variance);sample.availability|=4;}}
    history_[historyNext_]=sample;historyNext_=(historyNext_+1)%history_.size();historyCount_=std::min(historyCount_+1,history_.size());
}
void WeaponDiagnosticsGui::drawHistory() const {
    heading("Recorded native sample history (bounded to 256 frames)");
    if(historyCount_<2){ImGui::TextDisabled("Waiting for distinct native frames; no synthetic history");return;}
    const auto first=(historyNext_+history_.size()-historyCount_)%history_.size();
    const auto& start=history_[first];const auto& end=history_[(historyNext_+history_.size()-1)%history_.size()];
    const double span=end.timeS-start.timeS;if(!(span>0))return;
    const char* labels[]={"Native gross engine scalar sum N","Actual fuel kg","Native position sqrt(trace(Pp)) m"};
    for(unsigned metric=0;metric<3;++metric){
        const auto value=[&](const History& s){return metric==0?s.thrustN:metric==1?s.fuelKg:s.positionSigmaM;};
        double low=std::numeric_limits<double>::infinity(),high=-std::numeric_limits<double>::infinity();
        for(std::size_t i=0;i<historyCount_;++i){const auto& s=history_[(first+i)%history_.size()];if(s.availability&(1u<<metric)){low=std::min(low,value(s));high=std::max(high,value(s));}}
        if(!std::isfinite(low)){ImGui::TextDisabled("%s unavailable",labels[metric]);continue;}
        ImGui::Text("%s | %.6g .. %.6g | %.6f .. %.6f s",labels[metric],low,high,start.timeS,end.timeS);
        const auto origin=ImGui::GetCursorScreenPos();const float width=ImGui::GetContentRegionAvail().x,height=65;
        ImGui::InvisibleButton(labels[metric],{width,height});auto* dl=ImGui::GetWindowDrawList();dl->AddRectFilled(origin,{origin.x+width,origin.y+height},IM_COL32(12,16,20,255));
        const auto point=[&](const History& s){return ImVec2(origin.x+static_cast<float>((s.timeS-start.timeS)/span)*width,origin.y+height-(high>low?static_cast<float>((value(s)-low)/(high-low)):.5f)*height);};
        for(std::size_t i=1;i<historyCount_;++i){const auto& a=history_[(first+i-1)%history_.size()];const auto& b=history_[(first+i)%history_.size()];if((a.availability&(1u<<metric))&&(b.availability&(1u<<metric)))dl->AddLine(point(a),point(b),IM_COL32(80,220,180,255),1.5f);}
    }
}
void WeaponDiagnosticsGui::draw() const {
    ImGui::SetNextWindowSize({900,850},ImGuiCond_FirstUseEver);
    if(!beginMandatoryPanel("Weapon diagnostics###weapon-diagnostics-v1")){ImGui::End();return;}
    ImGui::TextUnformatted("REFEREE / Viewer diagnostic stream - never actor knowledge");
    ImGui::Text("%s | frame %llu | t %.6f s%s",source_.world.c_str(),static_cast<unsigned long long>(source_.worldFrame),source_.simTimeS,replay_?" | recorded companion":"");
    if(!replay_&&(!source_.publisherAlive||source_.publisherAgeS>1))ImGui::TextColored({1,.4f,.2f,1},"Publisher unavailable/stale (wall age %.3f s)%s",source_.publisherAgeS,source_.paused?"; world paused":"");
    if(!frame_){ImGui::TextWrapped("Waiting for an exact-frame native weapon diagnostic sidecar. Missing data remain unavailable.");ImGui::End();return;}
    const auto& f=*frame_;
    if(replay_)ImGui::TextDisabled("Playback retains the recorded selection; absent weapon images cannot be reconstructed.");
    else {
        char preview[128];std::snprintf(preview,sizeof(preview),"entity %u / generation %llu",selectedEntity_,static_cast<unsigned long long>(selectedGeneration_));
        if(ImGui::BeginCombo("Actual weapon",selectedEntity_?preview:"Choose released weapon")){
            for(std::uint32_t i=0;i<f.rosterCount;++i){const auto& row=f.roster[i];const auto* model=equipment::model(row.store);char label[192];std::snprintf(label,sizeof(label),"%s (%s) #%u gen %llu",model?model->designation.data():row.store,row.name,row.entity,static_cast<unsigned long long>(row.generation));
                if(ImGui::Selectable(label,row.entity==selectedEntity_&&row.generation==selectedGeneration_))selectWeapon(row.entity,row.generation);}
            ImGui::EndCombo();
        }
    }
    ImGui::TextDisabled("Roster %u / %u native weapons",f.rosterCount,f.totalWeaponCount);
    if(thermalBandCount()>1){
        int band=static_cast<int>(selectedBand_);
        if(ImGui::Combo("Requested thermal channel",&band,"Channel 1\0Channel 2\0"))selectThermalBand(static_cast<unsigned>(band));
        ImGui::TextDisabled("The capture cause identifies the published channel; selection changes display only.");
    }
    if(f.worldFrame!=source_.worldFrame){ImGui::TextColored({1,.5f,.2f,1},"Exact current world frame unavailable; retained diagnostic frame %llu is historical",static_cast<unsigned long long>(f.worldFrame));}
    if(!replay_&&(f.entity!=selectedEntity_||f.generation!=selectedGeneration_||f.selectionRequest!=selectionRequest_)){
        ImGui::TextWrapped("Waiting for native selection acknowledgement. Previous weapon payload is not displayed as the new selection.");ImGui::End();return;
    }
    if(!f.entity){ImGui::TextDisabled("Select an actual native weapon to inspect");ImGui::End();return;}
    ImGui::Text("%s / %s | entity %u gen %llu | %s",f.name,f.store,f.entity,static_cast<unsigned long long>(f.generation),weaponDiagnosticStatusTextV1(f.selectionStatus));
    if(const auto* model=equipment::model(f.store)) {
        ImGui::Text("Catalogue equipment model: %s",model->designation.data());
        if(ImGui::TreeNode("Declared component models")) {
            for(const auto& part:equipment::components)if(part.storeId==model->storeId) {
                ImGui::TextUnformatted(part.name.data());ImGui::TextDisabled("%s",part.selectionId.data());
            }
            ImGui::TreePop();
        }
    }
    ImGui::Columns(2,"weapon-diagnostics-columns",false);
    drawGuidanceLink(f.guidance,f.link);drawThermal(f.thermal);drawRf(f.rf);drawControl(f.control);
    ImGui::NextColumn();drawNavigation(f.navigation);drawPropulsion(f.propulsion);drawEffects(f.effects);
    ImGui::Columns(1);drawHistory();drawEvents(f);ImGui::End();
}
}
