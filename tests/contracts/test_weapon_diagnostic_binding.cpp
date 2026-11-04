#include "foundation/transport/WeaponDiagnosticCaptureBindingV1.h"
#include "foundation/transport/WeaponDiagnosticsV1.h"
#include "foundation/transport/Recording.h"
#include <catch2/catch_test_macros.hpp>
#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <thread>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace {
using namespace fsim::ipc;
struct Workspace {
    std::filesystem::path root=std::filesystem::temp_directory_path()/
        ("capture-binding-"+std::to_string(fsim::platform::currentProcessId())+"-"+
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    Workspace(){REQUIRE(std::filesystem::create_directory(root));}
    ~Workspace(){std::error_code error;for(std::filesystem::directory_iterator it(root,error),end;!error&&it!=end;it.increment(error)){
        std::filesystem::remove(it->path(),error);}std::filesystem::remove(root,error);}
};
struct Pair {std::filesystem::path flight,diagnostics;WeaponDiagnosticCaptureCompletionV1 completion;};
std::unique_ptr<WeaponDiagnosticFrameV1> sample(std::uint64_t sequence,float pixel){
    auto out=std::make_unique<WeaponDiagnosticFrameV1>();out->publisherPid=77;out->publisherIncarnation=7;
    out->worldFrame=sequence;out->simTimeS=static_cast<double>(sequence)*.01;
    out->entity=42;out->generation=9;out->selectionStatus=WeaponDiagnosticStatusV1::Available;
    weaponDiagnosticTextV1(out->store,"binding-native");weaponDiagnosticTextV1(out->name,"same reused identity");
    out->rosterCount=out->totalWeaponCount=1;out->roster[0].entity=42;out->roster[0].generation=9;
    weaponDiagnosticTextV1(out->roster[0].store,"binding-native");weaponDiagnosticTextV1(out->roster[0].name,"same reused identity");
    auto& t=out->thermal;t.stamp.status=WeaponDiagnosticStatusV1::Available;t.stamp.sourceFrame=sequence;
    t.stamp.sampleTimeS=t.stamp.processingTimeS=t.stamp.deliveredTimeS=t.stamp.validUntilS=out->simTimeS;
    t.width=t.height=1;t.horizontalHalfAngleRad=t.verticalHalfAngleRad=.1;t.exposureS=.001;t.fullWellElectrons=1e6;t.adcBits=14;
    t.measuredElectrons[0]=pixel;return out;
}
void writeText(const std::filesystem::path& path,const std::string& text){
    std::ofstream out(path,std::ios::binary|std::ios::trunc);REQUIRE(out.good());out.write(text.data(),static_cast<std::streamsize>(text.size()));
    out.close();REQUIRE(out.good());
}
std::string readText(const std::filesystem::path& path){std::ifstream in(path,std::ios::binary);REQUIRE(in.good());return std::string(std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>());}
Pair capture(const std::filesystem::path& root,const std::string& name,double flightPosition,float pixel,bool publish=true){
    Pair out;out.flight=root/(name+".fsrec");out.diagnostics=root/(name+".fsrec.wdiag");std::string error;
    REQUIRE(invalidateWeaponDiagnosticCaptureBindingV1(out.diagnostics,error));
    RecordWriter flight;RecordingHeader header;header.capacity=1;header.dt=.01;REQUIRE(flight.open(out.flight,header));
    RecordedVehicle row;row.id=42;row.generation=9;row.alive=1;flight.writeVehicle(0,row);
    VehicleSample state;state.state.positionEcef[0]=flightPosition;
    flight.writeSnapshot(.01,{{0,state}});flight.writeSnapshot(.02,{{0,state}});
    out.completion.recordingBytes=flight.bytesWritten();out.completion.recordingClosedSuccessfully=flight.finishV1();
    REQUIRE(out.completion.recordingClosedSuccessfully);
    WeaponDiagnosticsRecorderV1 diagnostics;REQUIRE(diagnostics.open(out.diagnostics.string(),77,7));
    REQUIRE(diagnostics.append(*sample(1,pixel)));REQUIRE(diagnostics.append(*sample(2,pixel+1)));
    REQUIRE(diagnostics.finishCaptureV1(out.completion));
    if(publish)REQUIRE(finalizeWeaponDiagnosticCaptureBindingV1(out.flight,out.diagnostics,out.completion,error));
    return out;
}
void unchanged(const WeaponDiagnosticCaptureBindingV1& value){CHECK(value.publisherPid==999);CHECK(value.recording.sha256=="unchanged");}
void actualSymlink(const std::filesystem::path& target,const std::filesystem::path& link){
#ifdef _WIN32
    // UCRT64 libstdc++ exposes create_symlink but returns ENOSYS. Exercise the
    // actual NTFS object through its supported OS API instead of skipping it.
    auto created=CreateSymbolicLinkW(link.wstring().c_str(),target.wstring().c_str(),SYMBOLIC_LINK_FLAG_ALLOW_UNPRIVILEGED_CREATE);
    if(!created&&GetLastError()==ERROR_INVALID_PARAMETER)created=CreateSymbolicLinkW(link.wstring().c_str(),target.wstring().c_str(),0);
    INFO("CreateSymbolicLinkW error="<<GetLastError());REQUIRE(created!=0);
#else
    std::filesystem::create_symlink(target,link);
#endif
}
bool actualSymlinkPresent(const std::filesystem::path& path){
#ifdef _WIN32
    const auto attributes=GetFileAttributesW(path.wstring().c_str());
    return attributes!=INVALID_FILE_ATTRIBUTES&&(attributes&FILE_ATTRIBUTE_REPARSE_POINT)!=0;
#else
    return std::filesystem::is_symlink(std::filesystem::symlink_status(path));
#endif
}
}

TEST_CASE("Bound replay refuses a real foreign same-time recording with reused native identities", "[weapon-diagnostic-binding]"){
    Workspace files;const auto a=capture(files.root,"a",100,.25f),b=capture(files.root,"b",200,.75f);
    // Durable negative control: the unchanged raw reader accepts both captures
    // at the same native time/IDs/generations, so time/identity alone cannot bind them.
    Recording flight;REQUIRE(flight.load(a.flight));CHECK(flight.frames()[0].samples[0].second.state.positionEcef[0]==100);
    WeaponDiagnosticsReplayV1 raw;REQUIRE(raw.open(b.diagnostics.string()));auto frame=std::make_unique<WeaponDiagnosticFrameV1>();
    REQUIRE(raw.atTime(.01,*frame));CHECK(frame->entity==42);CHECK(frame->generation==9);CHECK(frame->thermal.measuredElectrons[0]==.75f);
    BoundWeaponDiagnosticsReplayV1 joined;std::string error;
    CHECK_FALSE(joined.open(a.flight,b.diagnostics,error));CHECK_FALSE(joined.isOpen());CHECK_FALSE(error.empty());
    frame->worldFrame=999;CHECK_FALSE(joined.atTime(.01,*frame));CHECK(frame->worldFrame==999);
    REQUIRE(joined.open(a.flight,a.diagnostics,error));REQUIRE(joined.atTime(.01,*frame));CHECK(frame->thermal.measuredElectrons[0]==.25f);
    CHECK(joined.publisherPid()==77);CHECK(joined.publisherIncarnation()==7);
    REQUIRE(joined.atTime(.02,*frame));CHECK(frame->thermal.measuredElectrons[0]==1.25f);
    CHECK_FALSE(joined.atTime(.015,*frame));CHECK(frame->worldFrame==2);
    REQUIRE(joined.rewind());REQUIRE(joined.next(*frame)==WeaponDiagnosticsReplayResultV1::Frame);CHECK(frame->worldFrame==1);
    REQUIRE(joined.next(*frame)==WeaponDiagnosticsReplayResultV1::Frame);CHECK(frame->worldFrame==2);
    CHECK(joined.next(*frame)==WeaponDiagnosticsReplayResultV1::End);CHECK(frame->worldFrame==2);
}

TEST_CASE("Capture binding follows bytes across copies and renames and leaves legacy flight usable", "[weapon-diagnostic-binding]"){
    Workspace files;auto pair=capture(files.root,"a",100,.25f);std::string error;
    const auto copied=files.root/"copied.fsrec",diagnostics=files.root/"renamed.wdiag";
    std::filesystem::copy_file(pair.flight,copied);std::filesystem::copy_file(pair.diagnostics,diagnostics);
    std::filesystem::copy_file(weaponDiagnosticCaptureBindingPathV1(pair.diagnostics),weaponDiagnosticCaptureBindingPathV1(diagnostics));
    BoundWeaponDiagnosticsReplayV1 joined;REQUIRE(joined.open(copied,diagnostics,error));
    auto frame=std::make_unique<WeaponDiagnosticFrameV1>();REQUIRE(joined.atTime(.01,*frame));CHECK(frame->thermal.measuredElectrons[0]==.25f);joined.close();
    std::filesystem::rename(copied,files.root/"moved.fsrec");REQUIRE(joined.open(files.root/"moved.fsrec",diagnostics,error));joined.close();
    REQUIRE(invalidateWeaponDiagnosticCaptureBindingV1(diagnostics,error));CHECK_FALSE(joined.open(files.root/"moved.fsrec",diagnostics,error));
    Recording legacy;REQUIRE(legacy.load(files.root/"moved.fsrec"));CHECK(legacy.frames().size()==2);
}

TEST_CASE("An open bound reader refuses ordinary capture invalidation or mutation transactionally", "[weapon-diagnostic-binding]"){
    Workspace files;const auto pair=capture(files.root,"a",100,.25f);std::string error;BoundWeaponDiagnosticsReplayV1 reader;
    REQUIRE(reader.open(pair.flight,pair.diagnostics,error));auto out=std::make_unique<WeaponDiagnosticFrameV1>();REQUIRE(reader.atTime(.01,*out));
    SECTION("flight length changes"){std::ofstream append(pair.flight,std::ios::binary|std::ios::app);append.put('x');append.close();}
    SECTION("diagnostic mtime changes"){std::filesystem::last_write_time(pair.diagnostics,std::filesystem::last_write_time(pair.diagnostics)+std::chrono::seconds(2));}
    SECTION("completion invalidated"){REQUIRE(invalidateWeaponDiagnosticCaptureBindingV1(pair.diagnostics,error));}
    SECTION("completion replaced same size and mtime"){
        const auto path=weaponDiagnosticCaptureBindingPathV1(pair.diagnostics);const auto time=std::filesystem::last_write_time(path);
        auto text=readText(path);const auto at=text.find("sha256");REQUIRE(at!=std::string::npos);text[at]='S';writeText(path,text);std::filesystem::last_write_time(path,time);
    }
    out->worldFrame=999;CHECK_FALSE(reader.atTime(.01,*out));CHECK(out->worldFrame==999);CHECK_FALSE(reader.isOpen());
    CHECK(reader.next(*out)==WeaponDiagnosticsReplayResultV1::Invalid);CHECK(out->worldFrame==999);CHECK_FALSE(reader.rewind());
}

TEST_CASE("Flight loading is bracketed by the same capture watch at Viewer startup", "[weapon-diagnostic-binding][capture-startup]"){
    Workspace files;const auto a=capture(files.root,"a",100,.25f),b=capture(files.root,"b",200,.75f);std::string error;
    const auto replace=[&]{
        std::filesystem::copy_file(b.flight,a.flight,std::filesystem::copy_options::overwrite_existing);
        std::filesystem::copy_file(b.diagnostics,a.diagnostics,std::filesystem::copy_options::overwrite_existing);
        std::filesystem::copy_file(weaponDiagnosticCaptureBindingPathV1(b.diagnostics),weaponDiagnosticCaptureBindingPathV1(a.diagnostics),std::filesystem::copy_options::overwrite_existing);
    };
    auto frame=std::make_unique<WeaponDiagnosticFrameV1>();Recording flight;BoundWeaponDiagnosticsReplayV1 joined;
    SECTION("old load-before-binding is an actual mixed-capture negative control"){
        REQUIRE(flight.load(a.flight));replace();REQUIRE(joined.open(a.flight,a.diagnostics,error));REQUIRE(joined.atTime(.01,*frame));
        // Individually valid inputs have identical times and reused identities,
        // but already loaded flight A is now paired with diagnostics B.
        CHECK(flight.frames()[0].samples[0].second.state.positionEcef[0]==100);
        CHECK(frame->thermal.measuredElectrons[0]==.75f);CHECK(frame->simTimeS==flight.frames()[0].simTime);
    }
    SECTION("watch before load and guard after load refuses replacement"){
        REQUIRE(joined.open(a.flight,a.diagnostics,error));replace();REQUIRE(flight.load(a.flight));
        CHECK(flight.frames()[0].samples[0].second.state.positionEcef[0]==200);
        CHECK_FALSE(joined.rewind());CHECK_FALSE(joined.isOpen());frame->worldFrame=999;
        CHECK_FALSE(joined.atTime(.01,*frame));CHECK(frame->worldFrame==999);
    }
    SECTION("unchanged actual load keeps the verified matching companion"){
        REQUIRE(joined.open(a.flight,a.diagnostics,error));REQUIRE(flight.load(a.flight));REQUIRE(joined.rewind());REQUIRE(joined.atTime(.01,*frame));
        CHECK(flight.frames()[0].samples[0].second.state.positionEcef[0]==100);CHECK(frame->thermal.measuredElectrons[0]==.25f);
    }
}

TEST_CASE("Capture admission rejects incomplete closes lengths identities and edited source bytes", "[weapon-diagnostic-binding]"){
    Workspace files;auto pair=capture(files.root,"a",100,.25f,false);std::string error;
    SECTION("flight not closed successfully"){pair.completion.recordingClosedSuccessfully=false;}
    SECTION("diagnostic not closed successfully"){pair.completion.diagnosticsClosedSuccessfully=false;}
    SECTION("partial flight length"){++pair.completion.recordingBytes;}
    SECTION("partial diagnostics length"){++pair.completion.diagnosticsBytes;}
    SECTION("wrong original incarnation"){++pair.completion.publisherIncarnation;}
    SECTION("wrong original publisher"){++pair.completion.publisherPid;}
    CHECK_FALSE(finalizeWeaponDiagnosticCaptureBindingV1(pair.flight,pair.diagnostics,pair.completion,error));
    CHECK_FALSE(std::filesystem::exists(weaponDiagnosticCaptureBindingPathV1(pair.diagnostics)));
    auto valid=capture(files.root,"valid",100,.25f);WeaponDiagnosticCaptureBindingV1 out;out.publisherPid=999;out.recording.sha256="unchanged";
    std::fstream edit(valid.flight,std::ios::binary|std::ios::in|std::ios::out);REQUIRE(edit.good());edit.seekp(-1,std::ios::end);edit.put('X');edit.close();
    CHECK_FALSE(verifyWeaponDiagnosticCaptureBindingV1(valid.flight,valid.diagnostics,out,error));unchanged(out);
}

TEST_CASE("Manifest lexical bounds precede recursive parsing and reject noncanonical documents", "[weapon-diagnostic-binding]"){
    Workspace files;const auto pair=capture(files.root,"a",100,.25f);auto path=weaponDiagnosticCaptureBindingPathV1(pair.diagnostics);
    auto text=readText(path);std::string error,expected;
    SECTION("deep arrays"){text="{\"recording\":"+std::string(1000,'[')+"0"+std::string(1000,']')+"}";expected="does not permit arrays";}
    SECTION("comment quotes cannot hide recursive arrays"){text="{\"recording\": /*\"*/ "+std::string(1000,'[')+"0"+std::string(1000,']')+" /*\"*/}";expected="does not permit comments";}
    SECTION("deep objects"){text=std::string(1000,'{')+std::string(1000,'}');expected="flat object/depth bound";}
    SECTION("duplicate field"){text.insert(1,"\"schema\":\"duplicate\",");}
    SECTION("unknown field"){text.insert(1,"\"unknown\":0,");}
    SECTION("quoted escapes stay bounded"){text="{\"schema\":\"quoted \\\" / [[]] {{{ escaped\"}";expected="missing or extra fields";}
    writeText(path,text);WeaponDiagnosticCaptureBindingV1 out;out.publisherPid=999;out.recording.sha256="unchanged";
    CHECK_FALSE(verifyWeaponDiagnosticCaptureBindingV1(pair.flight,pair.diagnostics,out,error));unchanged(out);CHECK_FALSE(error.empty());
    if(!expected.empty())CHECK(error.find(expected)!=std::string::npos);
}

TEST_CASE("Capture completion publication never replaces an occupied or competing marker", "[weapon-diagnostic-binding]"){
    Workspace files;const auto pair=capture(files.root,"a",100,.25f,false);const auto path=weaponDiagnosticCaptureBindingPathV1(pair.diagnostics);std::string error;
    SECTION("existing marker"){writeText(path,"occupied");CHECK_FALSE(finalizeWeaponDiagnosticCaptureBindingV1(pair.flight,pair.diagnostics,pair.completion,error));CHECK(readText(path)=="occupied");}
    SECTION("dangling symlink"){
        actualSymlink(files.root/"absent",path);REQUIRE(actualSymlinkPresent(path));
        CHECK_FALSE(finalizeWeaponDiagnosticCaptureBindingV1(pair.flight,pair.diagnostics,pair.completion,error));
        CHECK(actualSymlinkPresent(path));REQUIRE(invalidateWeaponDiagnosticCaptureBindingV1(pair.diagnostics,error));
        CHECK_FALSE(actualSymlinkPresent(path));
    }
    SECTION("concurrent publishers"){
        std::atomic<unsigned> ready{0};std::atomic<bool> start{false};bool a=false,b=false;std::string ea,eb;
        const auto publish=[&](bool& out,std::string& reason){ready.fetch_add(1);while(!start.load())std::this_thread::yield();out=finalizeWeaponDiagnosticCaptureBindingV1(pair.flight,pair.diagnostics,pair.completion,reason);};
        std::thread first(publish,std::ref(a),std::ref(ea)),second(publish,std::ref(b),std::ref(eb));
        while(ready.load()!=2){std::this_thread::yield();}
        start.store(true);first.join();second.join();CHECK(a!=b);
        WeaponDiagnosticCaptureBindingV1 binding;REQUIRE(verifyWeaponDiagnosticCaptureBindingV1(pair.flight,pair.diagnostics,binding,error));
    }
}

TEST_CASE("Actual filesystem symlink inputs cannot masquerade as ordinary capture files", "[weapon-diagnostic-binding]"){
    Workspace files;const auto pair=capture(files.root,"a",100,.25f);const auto alias=files.root/"flight-alias.bin";
    actualSymlink(pair.flight,alias);REQUIRE(actualSymlinkPresent(alias));
    WeaponDiagnosticFileDigestV1 out;out.sha256="unchanged";std::string error;
    CHECK_FALSE(weaponDiagnosticFileDigestV1(alias,out,error));CHECK(out.sha256=="unchanged");
}

TEST_CASE("Bounded streaming SHA256 matches standard independent known-answer vectors", "[weapon-diagnostic-binding]"){
    Workspace files;const auto path=files.root/"hash.bin";std::string error;WeaponDiagnosticFileDigestV1 digest;
    writeText(path,"");REQUIRE(weaponDiagnosticFileDigestV1(path,digest,error));CHECK(digest.bytes==0);
    CHECK(digest.sha256=="e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    writeText(path,"abc");REQUIRE(weaponDiagnosticFileDigestV1(path,digest,error));CHECK(digest.bytes==3);
    CHECK(digest.sha256=="ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    writeText(path,std::string(1000000,'a'));REQUIRE(weaponDiagnosticFileDigestV1(path,digest,error));CHECK(digest.bytes==1000000);
    CHECK(digest.sha256=="cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
    const auto expected=digest.sha256;WeaponDiagnosticCaptureLimitsV1 limits;limits.maximumFileBytes=999999;
    CHECK_FALSE(weaponDiagnosticFileDigestV1(path,digest,error,limits));CHECK(digest.sha256==expected);
    RecordWriter closed;CHECK_FALSE(closed.finishV1());WeaponDiagnosticsRecorderV1 absent;WeaponDiagnosticCaptureCompletionV1 completion;
    completion.publisherPid=999;CHECK_FALSE(absent.finishCaptureV1(completion));CHECK(completion.publisherPid==999);
}
