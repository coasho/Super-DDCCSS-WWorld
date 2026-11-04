#include "foundation/transport/WeaponDiagnosticsV1.h"
#include "foundation/transport/WeaponDiagnosticCaptureBindingV1.h"
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <atomic>
#include <thread>

namespace {
using namespace fsim::ipc;
std::string uniqueName(){return "weapon-diag-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());}
std::unique_ptr<WeaponDiagnosticFrameV1> frame(std::uint64_t sequence=1){
    auto p=std::make_unique<WeaponDiagnosticFrameV1>();p->worldFrame=sequence;p->simTimeS=static_cast<double>(sequence)*.01;
    p->publisherPid=fsim::platform::currentProcessId();p->publisherIncarnation=7;
    p->entity=42;p->generation=9;p->selectionStatus=WeaponDiagnosticStatusV1::Available;
    weaponDiagnosticTextV1(p->store,"test-native");weaponDiagnosticTextV1(p->name,"actual weapon");
    p->rosterCount=p->totalWeaponCount=1;p->roster[0].entity=42;p->roster[0].generation=9;
    weaponDiagnosticTextV1(p->roster[0].name,"actual weapon");weaponDiagnosticTextV1(p->roster[0].store,"test-native");
    auto& t=p->thermal;t.stamp.status=WeaponDiagnosticStatusV1::Available;t.stamp.sourceFrame=sequence;
    t.stamp.sampleTimeS=t.stamp.processingTimeS=t.stamp.deliveredTimeS=p->simTimeS;t.stamp.validUntilS=p->simTimeS+.1;
    t.width=t.height=2;t.horizontalHalfAngleRad=t.verticalHalfAngleRad=.1;
    t.exposureS=.001;t.fullWellElectrons=1e6;t.adcBits=14;
    t.measuredElectrons[0]=.25f;t.measuredElectrons[1]=.5f;t.segmentation[1]=3;
    t.flags=WeaponDiagnosticCentroidV1|WeaponDiagnosticSegmentationV1;t.selectedBlob=3;t.centroidColumn=1;t.centroidRow=0;
    return p;
}
}

TEST_CASE("Weapon diagnostics retain actual pixel labels stamps and selected generations", "[weapon-diagnostics][ipc]"){
    const auto name=uniqueName();WeaponDiagnosticsV1 publisher,reader,foreign;
    const auto pid=fsim::platform::currentProcessId();REQUIRE(publisher.create(name,pid,7));
    REQUIRE(reader.open(name,pid,7));CHECK_FALSE(foreign.open(name,pid,8));
    std::uint64_t request=0;REQUIRE(reader.requestSelection(42,9,1,&request));
    WeaponDiagnosticSelectionV1 selection;REQUIRE(publisher.selection(1,selection));
    CHECK(selection.entity==42);CHECK(selection.generation==9);CHECK(selection.request==request);
    auto input=frame();input->selectionRequest=request;REQUIRE(publisher.publish(*input));
    auto out=std::make_unique<WeaponDiagnosticFrameV1>();REQUIRE(reader.read(1,*out));
    CHECK(out->thermal.measuredElectrons[1]==.5f);CHECK(out->thermal.segmentation[1]==3);
    CHECK(out->generation==9);CHECK(out->selectionRequest==request);CHECK(out->thermal.stamp.sourceFrame==1);
    CHECK_FALSE(reader.read(2,*out));CHECK(out->worldFrame==1);
    CHECK_FALSE(publisher.selection(302,selection));
    CHECK_FALSE(reader.requestSelection(42,0,2));CHECK_FALSE(publisher.requestSelection(42,9,2));
    publisher.close();CHECK_FALSE(reader.readLatest(*out));
}

TEST_CASE("Weapon diagnostics reject oversized malformed and stale frames transactionally", "[weapon-diagnostics][ipc]"){
    WeaponDiagnosticsV1 publisher,reader;const auto name=uniqueName();const auto pid=fsim::platform::currentProcessId();
    REQUIRE(publisher.create(name,pid,7));REQUIRE(reader.open(name,pid,7));auto input=frame();REQUIRE(publisher.publish(*input));
    auto invalid=frame(2);invalid->thermal.width=129;CHECK_FALSE(publisher.publish(*invalid));
    invalid=frame(2);invalid->thermal.measuredElectrons[0]=std::numeric_limits<float>::quiet_NaN();CHECK_FALSE(publisher.publish(*invalid));
    invalid=frame(2);invalid->thermal.stamp.deliveredTimeS=2;CHECK_FALSE(publisher.publish(*invalid));
    invalid=frame(2);invalid->generation=0;CHECK_FALSE(publisher.publish(*invalid));
    invalid=frame(2);invalid->rosterCount=2;invalid->totalWeaponCount=2;invalid->roster[1]=invalid->roster[0];CHECK_FALSE(publisher.publish(*invalid));
    auto out=std::make_unique<WeaponDiagnosticFrameV1>();REQUIRE(reader.readLatest(*out));CHECK(out->worldFrame==1);
    CHECK_FALSE(publisher.publish(*input));CHECK(weaponDiagnosticFreshV1(out->thermal.stamp,.05));
    CHECK_FALSE(weaponDiagnosticFreshV1(out->thermal.stamp,.2));
}

TEST_CASE("Weapon diagnostics reject active publisher replacement and stale incarnations", "[weapon-diagnostics][ipc]"){
    const auto name=uniqueName();const auto pid=fsim::platform::currentProcessId();WeaponDiagnosticsV1 old,reader,replacement;
    REQUIRE(old.create(name,pid,7));REQUIRE(reader.open(name,pid,7));CHECK_FALSE(replacement.create(name,pid,7));
    old.close();REQUIRE(replacement.create(name,pid,8));auto p=frame();p->publisherIncarnation=8;REQUIRE(replacement.publish(*p));
    auto out=std::make_unique<WeaponDiagnosticFrameV1>();CHECK_FALSE(reader.readLatest(*out));
    REQUIRE(reader.open(name,pid,8));REQUIRE(reader.readLatest(*out));CHECK(out->publisherIncarnation==8);
}

TEST_CASE("Weapon diagnostics companion replay preserves native frames and rejects tampering", "[weapon-diagnostics][recording]"){
    const auto path=std::filesystem::temp_directory_path()/(uniqueName()+".wdiag");
    struct Cleanup{std::filesystem::path path;~Cleanup(){std::error_code error;std::filesystem::remove(path,error);}}cleanup{path};
    WeaponDiagnosticsRecorderV1 writer;const auto pid=fsim::platform::currentProcessId();REQUIRE(writer.open(path.string(),pid,7));
    auto p=frame();REQUIRE(writer.append(*p));p=frame(2);p->thermal.measuredElectrons[1]=9;REQUIRE(writer.append(*p));REQUIRE(writer.close());
    WeaponDiagnosticsReplayV1 reader;REQUIRE(reader.open(path.string()));auto out=std::make_unique<WeaponDiagnosticFrameV1>();
    REQUIRE(reader.next(*out)==WeaponDiagnosticsReplayResultV1::Frame);CHECK(out->worldFrame==1);
    REQUIRE(reader.next(*out)==WeaponDiagnosticsReplayResultV1::Frame);CHECK(out->thermal.measuredElectrons[1]==9.f);
    CHECK(reader.next(*out)==WeaponDiagnosticsReplayResultV1::End);REQUIRE(reader.rewind());
    REQUIRE(reader.next(*out)==WeaponDiagnosticsReplayResultV1::Frame);CHECK(out->worldFrame==1);reader.close();
    REQUIRE(reader.open(path.string()));REQUIRE(reader.atTime(.02,*out));CHECK(out->worldFrame==2);
    CHECK_FALSE(reader.atTime(.015,*out));CHECK(out->worldFrame==2);
    REQUIRE(reader.atTime(.01,*out));CHECK(out->worldFrame==1);reader.close();
    std::fstream file(path,std::ios::binary|std::ios::in|std::ios::out);REQUIRE(file.good());file.seekp(80);file.put('X');file.close();
    REQUIRE(reader.open(path.string()));CHECK(reader.next(*out)==WeaponDiagnosticsReplayResultV1::Invalid);
    CHECK(out->worldFrame==1);
}

TEST_CASE("Diagnostic checked completion retains original writer identity and length", "[weapon-diagnostics][recording][weapon-diagnostic-binding]"){
    const auto path=std::filesystem::temp_directory_path()/(uniqueName()+".wdiag");
    struct Cleanup{std::filesystem::path path;~Cleanup(){std::error_code error;std::filesystem::remove(path,error);}}cleanup{path};
    WeaponDiagnosticsRecorderV1 writer;const auto pid=fsim::platform::currentProcessId();REQUIRE(writer.open(path.string(),pid,7));
    REQUIRE(writer.append(*frame()));WeaponDiagnosticCaptureCompletionV1 completed;
    completed.recordingBytes=123;completed.recordingClosedSuccessfully=true;
    REQUIRE(writer.finishCaptureV1(completed));CHECK(completed.diagnosticsClosedSuccessfully);
    CHECK(completed.publisherPid==pid);CHECK(completed.publisherIncarnation==7);
    CHECK(completed.diagnosticsBytes==std::filesystem::file_size(path));CHECK(completed.recordingBytes==123);CHECK(completed.recordingClosedSuccessfully);
    CHECK_FALSE(writer.finishCaptureV1(completed));CHECK(completed.publisherPid==pid);
    WeaponDiagnosticsReplayV1 reader;REQUIRE(reader.open(path.string()));auto out=std::make_unique<WeaponDiagnosticFrameV1>();
    REQUIRE(reader.next(*out)==WeaponDiagnosticsReplayResultV1::Frame);CHECK(out->generation==9);
}

TEST_CASE("Weapon diagnostic atomic payload never admits a torn image under publication", "[weapon-diagnostics][ipc]"){
    WeaponDiagnosticsV1 publisher,reader;const auto name=uniqueName();const auto pid=fsim::platform::currentProcessId();
    REQUIRE(publisher.create(name,pid,7));REQUIRE(reader.open(name,pid,7));auto first=frame();REQUIRE(publisher.publish(*first));
    std::atomic<bool> done{false},failed{false};
    std::thread writer([&]{for(unsigned i=2;i<=48;++i){auto sample=frame(i);sample->thermal.measuredElectrons[0]=static_cast<float>(i);sample->thermal.segmentation[0]=i;if(!publisher.publish(*sample))failed.store(true);}done.store(true);});
    auto out=std::make_unique<WeaponDiagnosticFrameV1>();unsigned observed=0;
    while(!done.load())if(reader.readLatest(*out)&&out->worldFrame>1){++observed;if(out->thermal.measuredElectrons[0]!=static_cast<float>(out->worldFrame)||out->thermal.segmentation[0]!=out->worldFrame)failed.store(true);}
    writer.join();CHECK_FALSE(failed.load());REQUIRE(reader.readLatest(*out));CHECK(out->worldFrame==48);
    CHECK(out->thermal.measuredElectrons[0]==48.f);CHECK(out->thermal.segmentation[0]==48);
    // Scheduling may let the writer finish before the first read; the final
    // exact-frame check is mandatory while concurrent successes are counted.
    INFO("Concurrent complete frames observed: "<<observed);
}

TEST_CASE("Weapon diagnostics preserve original guidance and delivered fusion provenance", "[weapon-diagnostics][ipc]"){
    auto input=frame(100);auto& g=input->guidance;
    g.stamp.status=WeaponDiagnosticStatusV1::Available;g.stamp.sourceFrame=90;g.stamp.sampleTimeS=.8;
    g.stamp.processingTimeS=.85;g.stamp.deliveredTimeS=.9;g.stamp.validUntilS=1.2;g.measurementStamp=g.stamp;
    g.phase=3;g.flags=WeaponDiagnosticGuidanceTrackingV1|WeaponDiagnosticLosV1;g.localTrackId=7;g.trackTimeS=.8;g.losDirectionEcef[0]=1;
    g.rangeM=999999; // No measured-range flag: an unqualified value is not a measurement.
    auto& l=input->link;l.stamp=g.stamp;l.direction=1;l.flags=WeaponDiagnosticInputKnownV1|WeaponDiagnosticInputValidV1|WeaponDiagnosticChannelStatsV1;
    l.fusedEntity=47;l.stateTimeS=.7;l.lastUpdateS=.7;l.inputReceivedS=.9;l.inputValidUntilS=2;l.sourceCount=1;l.delivered=1;l.bytes=64;
    l.sources[0].kind=0;l.sources[0].system=10;l.sources[0].payload=20;l.sources[0].track=99;l.sources[0].sampleTimeS=.6;l.sources[0].receivedTimeS=.7;
    input->propulsion.nativeMotorAgeS=0;input->propulsion.netThrustN=123;
    const auto name=uniqueName();WeaponDiagnosticsV1 publisher,reader;const auto pid=fsim::platform::currentProcessId();
    REQUIRE(publisher.create(name,pid,7));REQUIRE(reader.open(name,pid,7));REQUIRE(publisher.publish(*input));
    auto out=std::make_unique<WeaponDiagnosticFrameV1>();REQUIRE(reader.read(100,*out));
    CHECK(out->guidance.measurementStamp.sampleTimeS==.8);CHECK(out->guidance.measurementStamp.deliveredTimeS==.9);
    CHECK_FALSE(out->guidance.flags&WeaponDiagnosticMeasuredRangeV1);CHECK(out->link.inputReceivedS==.9);
    CHECK(out->link.sources[0].track==99);CHECK(out->link.sources[0].sampleTimeS==.6);CHECK(out->link.delivered==1);
    CHECK_FALSE(out->propulsion.flags&WeaponDiagnosticMotorClockV1);CHECK_FALSE(out->propulsion.flags&WeaponDiagnosticNetAxialForceV1);
    input->worldFrame=101;input->simTimeS=1.01;input->link.sources[0].receivedTimeS=2;CHECK_FALSE(publisher.publish(*input));
    REQUIRE(reader.readLatest(*out));CHECK(out->worldFrame==100);
    input->link.sources[0].receivedTimeS=.7;input->link.inputValidUntilS=.95;CHECK_FALSE(publisher.publish(*input));
    input->link.flags&=~static_cast<std::uint32_t>(WeaponDiagnosticInputValidV1);input->link.fusionState=2;
    REQUIRE(publisher.publish(*input));REQUIRE(reader.readLatest(*out));CHECK(out->link.fusionState==2);CHECK_FALSE(out->link.flags&WeaponDiagnosticInputValidV1);
}

TEST_CASE("Weapon diagnostic recording preserves radio provenance and rejects future seeker epochs", "[weapon-diagnostics][recording]"){
    auto input=frame(100);input->guidance.stamp=input->thermal.stamp;input->guidance.measurementStamp=input->thermal.stamp;
    input->guidance.losDirectionEcef[0]=1;input->guidance.flags=WeaponDiagnosticLosV1;input->guidance.trackTimeS=2;
    CHECK_FALSE(validateWeaponDiagnosticsV1(*input));input->guidance.trackTimeS=.8;
    input->link.sourceCount=65;CHECK_FALSE(validateWeaponDiagnosticsV1(*input));input->link.sourceCount=1;
    input->link.sources[0].track=87;input->link.sources[0].sampleTimeS=.7;input->link.sources[0].receivedTimeS=.8;
    input->link.flags=WeaponDiagnosticChannelStatsV1;input->link.sent=3;input->link.lost=1;
    const auto path=std::filesystem::temp_directory_path()/(uniqueName()+".wdiag");
    struct Cleanup{std::filesystem::path path;~Cleanup(){std::error_code error;std::filesystem::remove(path,error);}}cleanup{path};
    WeaponDiagnosticsRecorderV1 writer;REQUIRE(writer.open(path.string(),input->publisherPid,7));REQUIRE(writer.append(*input));REQUIRE(writer.close());
    WeaponDiagnosticsReplayV1 replay;REQUIRE(replay.open(path.string()));auto out=std::make_unique<WeaponDiagnosticFrameV1>();
    REQUIRE(replay.atTime(1,*out));CHECK(out->link.sources[0].track==87);CHECK(out->link.sources[0].receivedTimeS==.8);CHECK(out->link.sent==3);CHECK(out->link.lost==1);
}
