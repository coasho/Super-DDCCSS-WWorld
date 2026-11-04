#include "visualization/ui/WeaponDiagnosticsGui.h"
#include "foundation/transport/WeaponThermalBandV1.h"
#include "foundation/transport/WeaponDiagnosticCaptureBindingV1.h"
#include "foundation/transport/Recording.h"
#include <vsgImGui/imgui.h>
#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <limits>
#include <chrono>
#include <array>

namespace {
struct Context {
    Context(){
        ImGui::CreateContext();auto& io=ImGui::GetIO();io.DisplaySize={1920,1080};io.DeltaTime=1.f/60;io.IniFilename=nullptr;
        unsigned char* pixels=nullptr;int width=0,height=0;io.Fonts->GetTexDataAsRGBA32(&pixels,&width,&height);
    }
    ~Context(){ImGui::DestroyContext();}
};
std::unique_ptr<fsim::ipc::WeaponDiagnosticFrameV1> snapshot(){
    using namespace fsim::ipc;auto f=std::make_unique<WeaponDiagnosticFrameV1>();
    f->publisherPid=77;f->publisherIncarnation=9;f->worldFrame=12;f->simTimeS=1;f->entity=3;f->generation=2;
    f->selectionStatus=WeaponDiagnosticStatusV1::Available;weaponDiagnosticTextV1(f->name,"native weapon");weaponDiagnosticTextV1(f->store,"test");
    auto& t=f->thermal;t.stamp.status=WeaponDiagnosticStatusV1::Available;t.stamp.sourceFrame=12;
    t.stamp.sampleTimeS=.9;t.stamp.processingTimeS=.95;t.stamp.deliveredTimeS=1;t.stamp.validUntilS=1.1;
    t.width=t.height=128;t.horizontalHalfAngleRad=t.verticalHalfAngleRad=.1;
    t.exposureS=.001;t.fullWellElectrons=1e6;t.adcBits=14;
    t.flags=WeaponDiagnosticCentroidV1|WeaponDiagnosticSegmentationV1;t.centroidColumn=64;t.centroidRow=32;t.selectedBlob=7;
    for(unsigned i=0;i<128*128;++i){t.measuredElectrons[i]=static_cast<float>(i)*.00001f;t.segmentation[i]=i%7;}
    auto& r=f->rf;r.stamp=t.stamp;r.rows=r.columns=64;r.frequencyHz=1e10;r.rangeBinM=10;r.closingBinMs=2;
    r.flags=WeaponDiagnosticRangeAxisV1|WeaponDiagnosticDopplerAxisV1;
    for(unsigned i=0;i<64*64;++i){r.powerNoiseRelative[i]=static_cast<float>(i+1)*1e-12f;r.thresholdNoiseRelative[i]=3e-12f;r.localNoiseRelative[i]=1e-12f;}
    r.peakCount=1;r.peaks[0].row=32;r.peaks[0].column=32;r.peaks[0].powerNoiseRelative=4e-9;r.peaks[0].thresholdNoiseRelative=3e-12;r.peaks[0].snrLinear=4000;
    auto& n=f->navigation;n.stamp=t.stamp;n.errorStateCount=21;n.quaternionBodyToEcef[0]=1;
    for(unsigned i=0;i<21;++i)n.errorCovariance[21*i+i]=.01;
    return f;
}
}

TEST_CASE("Weapon diagnostic GUI draws bounded native thermal RF and covariance grids without GPU", "[weapon-diagnostics][render]"){
    Context context;auto gui=fsim::ui::WeaponDiagnosticsGui::create();auto input=snapshot();
    REQUIRE(gui->setReplayFrame(*input,12,1));ImGui::NewFrame();gui->draw();ImGui::Render();
    const auto* data=ImGui::GetDrawData();REQUIRE(data);CHECK(data->CmdLists.Size>1);CHECK(data->TotalVtxCount>1000);
    CHECK(gui->snapshot()->thermal.measuredElectrons[17]==input->thermal.measuredElectrons[17]);
    CHECK(gui->snapshot()->rf.powerNoiseRelative[100]==input->rf.powerNoiseRelative[100]);
    CHECK(gui->snapshot()->navigation.errorCovariance[0]==.01);
}

TEST_CASE("Weapon diagnostic playback refuses frame mismatch and cannot turn replay into live selection", "[weapon-diagnostics][render]"){
    auto gui=fsim::ui::WeaponDiagnosticsGui::create();auto input=snapshot();
    REQUIRE(gui->setReplayFrame(*input,12,1));CHECK_FALSE(gui->selectWeapon(9,10));
    CHECK(gui->thermalBandCount()==0);CHECK_FALSE(gui->selectThermalBand(0));CHECK_FALSE(gui->selectThermalBand(1));
    CHECK_FALSE(gui->setReplayFrame(*input,13,1));CHECK_FALSE(gui->snapshot());
    CHECK_FALSE(gui->setReplayFrame(*input,12,1.1));CHECK_FALSE(gui->snapshot());
    input->thermal.width=129;CHECK_FALSE(gui->setReplayFrame(*input,12,1));CHECK_FALSE(gui->snapshot());
}

TEST_CASE("Diagnostic GUI receives only the verified flight companion and clears invalidated paused replay", "[weapon-diagnostics][render][weapon-diagnostic-binding]"){
    using namespace fsim::ipc;
    const auto prefix=std::filesystem::temp_directory_path()/("binding-ui-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const std::array<std::filesystem::path,2> flights={prefix.string()+"-a.fsrec",prefix.string()+"-b.fsrec"};
    struct Cleanup{std::array<std::filesystem::path,2> files;~Cleanup(){for(const auto& path:files){std::error_code error;
        std::filesystem::remove(path,error);const auto diag=std::filesystem::path(path.string()+".wdiag");std::filesystem::remove(diag,error);
        std::filesystem::remove(weaponDiagnosticCaptureBindingPathV1(diag),error);}}}cleanup{flights};
    for(std::size_t i=0;i<flights.size();++i){
        std::string error;REQUIRE(invalidateWeaponDiagnosticCaptureBindingV1(flights[i].string()+".wdiag",error));
        RecordWriter flight;RecordingHeader header;header.capacity=1;header.dt=.01;REQUIRE(flight.open(flights[i],header));
        VehicleSample position;position.state.positionEcef[0]=100+static_cast<double>(i);flight.writeSnapshot(1,{{0,position}});
        WeaponDiagnosticCaptureCompletionV1 completion;completion.recordingBytes=flight.bytesWritten();completion.recordingClosedSuccessfully=flight.finishV1();
        auto frame=snapshot();frame->thermal.measuredElectrons[17]=static_cast<float>(i)+.25f;
        WeaponDiagnosticsRecorderV1 diagnostic;REQUIRE(diagnostic.open(flights[i].string()+".wdiag",77,9));REQUIRE(diagnostic.append(*frame));
        REQUIRE(diagnostic.finishCaptureV1(completion));REQUIRE(finalizeWeaponDiagnosticCaptureBindingV1(flights[i],flights[i].string()+".wdiag",completion,error));
    }
    auto gui=fsim::ui::WeaponDiagnosticsGui::create();auto frame=std::make_unique<WeaponDiagnosticFrameV1>();std::string error;
    BoundWeaponDiagnosticsReplayV1 joined;
    CHECK_FALSE(joined.open(flights[0],flights[1].string()+".wdiag",error));
    gui->setReplayUnavailable(0,1);CHECK_FALSE(gui->snapshot());
    REQUIRE(joined.open(flights[0],flights[0].string()+".wdiag",error));REQUIRE(joined.atTime(1,*frame));
    REQUIRE(gui->setReplayFrame(*frame,frame->worldFrame,1));CHECK(gui->snapshot()->thermal.measuredElectrons[17]==.25f);
    REQUIRE(invalidateWeaponDiagnosticCaptureBindingV1(flights[0].string()+".wdiag",error));
    CHECK_FALSE(joined.atTime(1,*frame));gui->setReplayUnavailable(0,1);CHECK_FALSE(gui->snapshot());
    CHECK_FALSE(gui->selectWeapon(3,2));CHECK_FALSE(gui->selectThermalBand(0));
}

TEST_CASE("Diagnostic channel selection uses the live publisher and leaves recorded pixels unchanged", "[weapon-diagnostics][render]"){
    using namespace fsim::ipc;
    const auto name="band-ui-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const auto pid=fsim::platform::currentProcessId();
    WeaponDiagnosticsV1 publisher;WeaponThermalBandV1 bands;
    REQUIRE(publisher.create(name,pid,9));REQUIRE(bands.create(name,pid,9));
    auto input=snapshot();input->publisherPid=pid;
    REQUIRE(publisher.publish(*input));REQUIRE(bands.publish(12,3,2,2));
    auto gui=fsim::ui::WeaponDiagnosticsGui::create();
    fsim::ui::WeaponDiagnosticsGui::Source source;
    source.world=name;source.publisherPid=pid;source.publisherIncarnation=9;
    source.worldFrame=12;source.simTimeS=1;source.publisherAlive=true;
    gui->setSource(source);gui->update();REQUIRE(gui->selectWeapon(3,2));
    REQUIRE(gui->thermalBandCount()==2);REQUIRE(gui->selectThermalBand(1));
    unsigned selected=99;REQUIRE(bands.selection(12,3,2,selected));CHECK(selected==1);
    CHECK_FALSE(gui->selectThermalBand(2));
    REQUIRE(bands.selection(12,3,2,selected));CHECK(selected==1);
    REQUIRE(bands.publish(113,3,2,2));source.worldFrame=113;
    gui->setSource(source);gui->update();
    REQUIRE(bands.selection(113,3,2,selected));CHECK(selected==1);
    source.worldFrame=414;gui->setSource(source);gui->update();
    CHECK(gui->thermalBandCount()==0);CHECK_FALSE(gui->selectThermalBand(0));
    source.worldFrame=415;REQUIRE(bands.publish(415,3,3,2));
    gui->setSource(source);gui->update();CHECK(gui->thermalBandCount()==0);
    REQUIRE(gui->selectWeapon(3,3));REQUIRE(gui->selectThermalBand(0));
    REQUIRE(bands.selection(415,3,3,selected));CHECK(selected==0);
    const auto pixel=input->thermal.measuredElectrons[17];
    REQUIRE(gui->setReplayFrame(*input,12,1));CHECK(gui->thermalBandCount()==0);
    CHECK_FALSE(gui->selectThermalBand(0));CHECK_FALSE(gui->selectThermalBand(1));
    CHECK(gui->snapshot()->thermal.measuredElectrons[17]==pixel);
    source.publisherIncarnation=10;gui->setSource(source);gui->update();
    CHECK(gui->thermalBandCount()==0);CHECK_FALSE(gui->selectThermalBand(1));
    gui->clear();CHECK_FALSE(gui->snapshot());CHECK(gui->thermalBandCount()==0);
}

TEST_CASE("A publisher without the optional channel sidecar retains V1 diagnostic playback", "[weapon-diagnostics][render]"){
    using namespace fsim::ipc;
    const auto name="legacy-band-ui-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const auto pid=fsim::platform::currentProcessId();WeaponDiagnosticsV1 publisher;
    REQUIRE(publisher.create(name,pid,9));auto input=snapshot();input->publisherPid=pid;
    REQUIRE(publisher.publish(*input));auto gui=fsim::ui::WeaponDiagnosticsGui::create();
    fsim::ui::WeaponDiagnosticsGui::Source source;source.world=name;source.publisherPid=pid;
    source.publisherIncarnation=9;source.worldFrame=12;source.simTimeS=1;source.publisherAlive=true;
    gui->setSource(source);gui->update();REQUIRE(gui->snapshot());
    CHECK(gui->snapshot()->thermal.measuredElectrons[17]==input->thermal.measuredElectrons[17]);
    CHECK(gui->thermalBandCount()==0);CHECK_FALSE(gui->selectThermalBand(0));
}

TEST_CASE("Removed thermal captures remain inspectable with their original status and epoch", "[weapon-diagnostics][render]"){
    using namespace fsim::ipc;Context context;auto gui=fsim::ui::WeaponDiagnosticsGui::create();auto input=snapshot();
    ImGui::GetIO().DisplaySize={4096,8192};
    input->rf={};input->navigation={};input->selectionStatus=WeaponDiagnosticStatusV1::Removed;
    input->thermal.stamp.status=WeaponDiagnosticStatusV1::Removed;
    REQUIRE(gui->setReplayFrame(*input,12,1));
    ImGui::NewFrame();gui->draw();ImGui::Render();
    ImGui::SetWindowPos("Weapon diagnostics###weapon-diagnostics-v1",{20,20},ImGuiCond_Always);
    ImGui::SetWindowSize("Weapon diagnostics###weapon-diagnostics-v1",{1600,4000},ImGuiCond_Always);
    ImGui::NewFrame();gui->draw();ImGui::Render();
    const auto* data=ImGui::GetDrawData();REQUIRE(data);CHECK(data->TotalVtxCount>=128*128*4);
    CHECK(gui->snapshot()->thermal.stamp.status==WeaponDiagnosticStatusV1::Removed);
    CHECK(gui->snapshot()->thermal.stamp.sampleTimeS==.9);
    input->thermal.width=129;CHECK_FALSE(gui->setReplayFrame(*input,12,1));CHECK_FALSE(gui->snapshot());
}

TEST_CASE("Weapon diagnostic full RF grid retains every cell within 16 bit draw lists", "[weapon-diagnostics][render]"){
    Context context;ImGui::GetIO().DisplaySize={4096,8192};auto gui=fsim::ui::WeaponDiagnosticsGui::create();auto input=snapshot();
    input->rf.rows=512;input->rf.columns=256;
    for(unsigned i=0;i<131072;++i)input->rf.powerNoiseRelative[i]=static_cast<float>(i+1);
    REQUIRE(gui->setReplayFrame(*input,12,1));ImGui::NewFrame();gui->draw();ImGui::Render();
    ImGui::SetWindowPos("Weapon diagnostics###weapon-diagnostics-v1",{20,20},ImGuiCond_Always);
    ImGui::SetWindowSize("Weapon diagnostics###weapon-diagnostics-v1",{1600,4000},ImGuiCond_Always);
    ImGui::NewFrame();gui->draw();ImGui::Render();const auto* data=ImGui::GetDrawData();REQUIRE(data);
    // Four vertices per actual native RF cell; no transport or UI downsample.
    CHECK(data->TotalVtxCount>=131072*4);CHECK(data->CmdLists.Size>=32);
    // ImGui1.92.9 leaves obsolete CmdListsCount zero; the actual vector size
    // also ensures this per-list index-bound assertion is never vacuous.
    if(sizeof(ImDrawIdx)==2)for(int i=0;i<data->CmdLists.Size;++i)CHECK(data->CmdLists[i]->VtxBuffer.Size<=static_cast<int>(std::numeric_limits<ImDrawIdx>::max()));
    CHECK(gui->snapshot()->rf.powerNoiseRelative[131071]==131072.f);
}

TEST_CASE("Weapon diagnostic GUI consumes measured guidance original uplink epochs and net propulsion", "[weapon-diagnostics][render]"){
    using namespace fsim::ipc;Context context;auto gui=fsim::ui::WeaponDiagnosticsGui::create();auto input=snapshot();
    auto& g=input->guidance;g.stamp=input->thermal.stamp;g.measurementStamp=g.stamp;g.phase=3;
    g.flags=WeaponDiagnosticGuidanceTrackingV1|WeaponDiagnosticLosV1|WeaponDiagnosticLosRateV1|WeaponDiagnosticTimeToGoV1;
    g.localTrackId=8;g.trackTimeS=.9;g.losDirectionEcef[0]=1;g.losDirectionRatePerS[1]=.1;g.timeToGoS=2;
    auto& l=input->link;l.stamp=g.stamp;l.direction=2;l.flags=WeaponDiagnosticInputKnownV1|WeaponDiagnosticInputValidV1|WeaponDiagnosticChannelStatsV1;
    weaponDiagnosticTextV1(l.directionSource,"Synthetic UI transport fixture; no real equipment fit");
    l.fusedEntity=99;l.stateTimeS=.8;l.lastUpdateS=.8;l.inputReceivedS=.95;l.inputValidUntilS=2;
    l.sent=3;l.delivered=2;l.lost=1;l.bytes=64;l.sourceCount=1;
    l.sources[0].kind=1;l.sources[0].track=7;l.sources[0].sampleTimeS=.7;l.sources[0].receivedTimeS=.8;
    auto& p=input->propulsion;p.stamp=g.stamp;p.engineCount=1;p.thrustN[0]=100;p.netThrustN=80;p.nativeMotorAgeS=.8;
    p.flags=WeaponDiagnosticMotorClockV1|WeaponDiagnosticPulsePolicyV1|WeaponDiagnosticNetAxialForceV1;
    REQUIRE(gui->setReplayFrame(*input,12,1));ImGui::NewFrame();gui->draw();ImGui::Render();
    REQUIRE(ImGui::GetDrawData());CHECK(ImGui::GetDrawData()->TotalVtxCount>1000);
    CHECK(gui->snapshot()->guidance.trackTimeS==.9);CHECK(gui->snapshot()->link.inputReceivedS==.95);
    CHECK(gui->snapshot()->propulsion.netThrustN==80);CHECK(gui->snapshot()->propulsion.thrustN[0]==100);
}
