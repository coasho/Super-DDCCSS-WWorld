#include "WeaponDiagnosticCaptureBindingV1.h"
#include "WeaponDiagnosticsV1.h"
#include "contracts/recording/RecordingFormat.h"
#include "foundation/core/Json.h"

// Kept byte-identical to the pinned upstream release. The integration target
// should mark only this third-party include directory SYSTEM.
#include <picosha2.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <set>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>

namespace fsim::ipc {
namespace {
using core::Json;
constexpr std::string_view schema="fsim.weapon-diagnostic-capture-binding.v1";
constexpr std::size_t hashBlockBytes=64*1024;
constexpr std::uint64_t maximumShaBytes=std::numeric_limits<std::uint64_t>::max()/8;

void need(bool accepted,const char* reason) {
    if(!accepted)throw std::runtime_error(reason);
}
struct FileState {
    std::uint64_t bytes=0;
    std::filesystem::file_time_type modified;
};
FileState fileState(const std::filesystem::path& path,std::uint64_t maximum) {
    need(!path.empty(),"Capture input path is empty");
    need(std::filesystem::symlink_status(path).type()==std::filesystem::file_type::regular,
        "Capture input must be an ordinary regular file");
    const auto size=std::filesystem::file_size(path);
    need(size<=maximum&&size<=maximumShaBytes,"Capture input exceeds its checked byte limit");
    return {static_cast<std::uint64_t>(size),std::filesystem::last_write_time(path)};
}
bool same(FileState a,FileState b) {
    return a.bytes==b.bytes&&a.modified==b.modified;
}
struct CaptureWatch {
    FileState recording,diagnostics,manifest;
    std::string markerBytes;
};
CaptureWatch watchCapture(const std::filesystem::path& recording,const std::filesystem::path& diagnostics,
    const WeaponDiagnosticCaptureLimitsV1& limits) {
    CaptureWatch out;
    const auto marker=weaponDiagnosticCaptureBindingPathV1(diagnostics);
    out.recording=fileState(recording,limits.maximumFileBytes);
    out.diagnostics=fileState(diagnostics,limits.maximumFileBytes);
    out.manifest=fileState(marker,static_cast<std::uint64_t>(limits.maximumManifestBytes));
    std::ifstream stream(marker,std::ios::binary);
    need(bool(stream),"Bound capture completion marker is unavailable");
    out.markerBytes.assign(static_cast<std::size_t>(out.manifest.bytes),'\0');
    stream.read(out.markerBytes.data(),static_cast<std::streamsize>(out.markerBytes.size()));
    need(stream.gcount()==static_cast<std::streamsize>(out.markerBytes.size())&&!stream.bad(),
        "Bound capture completion marker read failed");
    need(stream.peek()==std::char_traits<char>::eof()&&!stream.bad(),"Bound capture completion marker grew while reading");
    need(same(out.recording,fileState(recording,limits.maximumFileBytes))&&
        same(out.diagnostics,fileState(diagnostics,limits.maximumFileBytes))&&
        same(out.manifest,fileState(marker,static_cast<std::uint64_t>(limits.maximumManifestBytes))),
        "Bound capture changed while checking its completion marker");
    return out;
}
bool same(const CaptureWatch& a,const CaptureWatch& b) {
    return same(a.recording,b.recording)&&same(a.diagnostics,b.diagnostics)&&
        same(a.manifest,b.manifest)&&a.markerBytes==b.markerBytes;
}
void limitsValid(const WeaponDiagnosticCaptureLimitsV1& limits) {
    need(limits.maximumFileBytes>0&&limits.maximumFileBytes<=maximumShaBytes,
        "Invalid capture file byte limit");
    need(limits.maximumManifestBytes>0&&limits.maximumManifestBytes<=65536,
        "Invalid capture manifest byte limit");
}
WeaponDiagnosticFileDigestV1 digestFile(const std::filesystem::path& path,std::uint64_t maximum) {
    const auto before=fileState(path,maximum);
    std::ifstream stream(path,std::ios::binary);
    need(bool(stream),"Capture input cannot be opened for hashing");
    picosha2::hash256_one_by_one hasher;
    std::array<unsigned char,hashBlockBytes> block{};
    std::uint64_t count=0;
    while(true) {
        stream.read(reinterpret_cast<char*>(block.data()),static_cast<std::streamsize>(block.size()));
        const auto read=stream.gcount();
        need(read>=0,"Capture input returned an invalid read count");
        const auto bytes=static_cast<std::uint64_t>(read);
        need(count<=before.bytes&&bytes<=before.bytes-count,"Capture input grew while hashing");
        if(bytes) {
            hasher.process(block.begin(),block.begin()+static_cast<std::ptrdiff_t>(bytes));
            count+=bytes;
        }
        if(stream.eof())break;
        need(bool(stream),"Capture input read failed");
    }
    need(!stream.bad()&&count==before.bytes,"Capture input was truncated or unreadable");
    need(same(before,fileState(path,maximum)),"Capture input changed while hashing");
    hasher.finish();
    return {count,picosha2::get_hash_hex_string(hasher)};
}
bool equal(const WeaponDiagnosticFileDigestV1& a,const WeaponDiagnosticFileDigestV1& b) {
    return a.bytes==b.bytes&&a.sha256==b.sha256;
}
void members(const Json& object,std::initializer_list<std::string_view> expected) {
    const auto& rows=object.asObject();
    need(rows.size()==expected.size(),"Capture manifest has missing or extra fields");
    std::set<std::string> names;
    for(const auto& row:rows) {
        need(names.insert(row.first).second,"Capture manifest has a duplicate field");
        need(std::find(expected.begin(),expected.end(),std::string_view(row.first))!=expected.end(),
            "Capture manifest has an unknown field");
    }
}
std::uint64_t decimal(const Json& value) {
    const auto& text=value.asString();
    need(!text.empty()&&(text.size()==1||text.front()!='0'),"Capture integer is not canonical unsigned decimal text");
    std::uint64_t out=0;
    for(const char byte:text) {
        need(byte>='0'&&byte<='9',"Capture integer is not unsigned decimal text");
        const auto digit=static_cast<std::uint64_t>(byte-'0');
        need(out<=(std::numeric_limits<std::uint64_t>::max()-digit)/10,"Capture integer overflow");
        out=out*10+digit;
    }
    return out;
}
WeaponDiagnosticFileDigestV1 digestDocument(const Json& doc) {
    members(doc,{"bytes","sha256"});
    WeaponDiagnosticFileDigestV1 out{decimal(doc.child("bytes")),doc.child("sha256").asString()};
    need(out.sha256.size()==64,"Capture SHA256 must have exactly 64 hexadecimal digits");
    for(const char byte:out.sha256)
        need((byte>='0'&&byte<='9')||(byte>='a'&&byte<='f'),"Capture SHA256 must be canonical lowercase hex");
    return out;
}
Json digestDocument(const WeaponDiagnosticFileDigestV1& digest) {
    return Json(Json::Object{{"bytes",std::to_string(digest.bytes)},{"sha256",digest.sha256}});
}
Json document(const WeaponDiagnosticCaptureBindingV1& binding) {
    return Json(Json::Object{{"schema",std::string(schema)},{"algorithm","sha256"},
        {"publisher_pid",std::to_string(binding.publisherPid)},
        {"publisher_incarnation",static_cast<double>(binding.publisherIncarnation)},
        {"recording",digestDocument(binding.recording)},{"diagnostics",digestDocument(binding.diagnostics)}});
}
WeaponDiagnosticCaptureBindingV1 readDocument(const std::filesystem::path& path,const WeaponDiagnosticCaptureLimitsV1& limits) {
    const auto before=fileState(path,static_cast<std::uint64_t>(limits.maximumManifestBytes));
    std::ifstream stream(path,std::ios::binary);
    need(bool(stream),"Capture binding cannot be opened");
    std::string text(static_cast<std::size_t>(before.bytes),'\0');
    stream.read(text.data(),static_cast<std::streamsize>(text.size()));
    need(stream.gcount()==static_cast<std::streamsize>(text.size())&&!stream.bad(),"Capture binding read failed");
    need(stream.peek()==std::char_traits<char>::eof()&&!stream.bad(),"Capture binding grew while reading");
    need(same(before,fileState(path,static_cast<std::uint64_t>(limits.maximumManifestBytes))),
        "Capture binding changed while reading");
    // core::Json is a general recursive parser. Bound this particular schema
    // BEFORE invoking it: one root object and at most two flat digest objects,
    // with no arrays. Braces/colons inside quoted or escaped strings are data.
    unsigned depth=0,objects=0;std::array<unsigned,2> fields{};
    bool quoted=false,escaped=false,rootClosed=false;
    for(const char byte:text) {
        if(quoted) {
            if(escaped)escaped=false;
            else if(byte=='\\')escaped=true;
            else if(byte=='"')quoted=false;
            continue;
        }
        // core::Json also accepts comments. They are not part of this
        // canonical manifest grammar: a quote inside a comment must never
        // conceal recursive parser input from this lexical depth bound.
        need(byte!='/',"Capture binding schema does not permit comments");
        if(byte=='"'){quoted=true;continue;}
        need(byte!='['&&byte!=']',"Capture binding schema does not permit arrays");
        if(byte=='{') {
            need(!rootClosed&&depth<fields.size()&&objects<3,"Capture binding exceeds its flat object/depth bound");
            fields[depth++]=0;++objects;
        } else if(byte=='}') {
            need(depth>0,"Capture binding has an unmatched object close");
            if(--depth==0)rootClosed=true;
        } else if(byte==':') {
            need(depth>0,"Capture binding field is outside its root object");
            need(++fields[depth-1]<=(depth==1?6u:2u),"Capture binding exceeds its schema field bound");
        }
    }
    need(objects>0&&rootClosed&&depth==0&&!quoted&&!escaped,"Capture binding has an incomplete schema object/string");
    const auto doc=Json::parse(text,path.string());
    members(doc,{"schema","algorithm","publisher_pid","publisher_incarnation","recording","diagnostics"});
    need(doc.child("schema").asString()==schema&&doc.child("algorithm").asString()=="sha256",
        "Unsupported capture binding schema or hash algorithm");
    WeaponDiagnosticCaptureBindingV1 out;out.publisherPid=decimal(doc.child("publisher_pid"));
    const double incarnation=doc.child("publisher_incarnation").asNumber();
    need(out.publisherPid&&std::isfinite(incarnation)&&incarnation>=1&&std::floor(incarnation)==incarnation&&
        incarnation<=std::numeric_limits<std::uint32_t>::max(),"Invalid capture publisher identity");
    out.publisherIncarnation=static_cast<std::uint32_t>(incarnation);
    out.recording=digestDocument(doc.child("recording"));out.diagnostics=digestDocument(doc.child("diagnostics"));
    need(out.recording.bytes<=limits.maximumFileBytes&&out.diagnostics.bytes<=limits.maximumFileBytes,
        "Bound capture exceeds the checked byte limit");
    return out;
}
void recordingHeader(const std::filesystem::path& path) {
    std::ifstream stream(path,std::ios::binary);std::array<char,6> prefix{};
    stream.read(prefix.data(),static_cast<std::streamsize>(prefix.size()));
    need(bool(stream)&&std::equal(prefix.begin(),prefix.begin()+5,kRecordingMagic)&&
        static_cast<unsigned char>(prefix[5])==kRecordingVersion,"Bound flight input is not a current native fsrec");
    need(std::filesystem::file_size(path)>=6+sizeof(RecordingHeader),"Bound flight recording header is incomplete");
}
void diagnosticHeader(const std::filesystem::path& path,const WeaponDiagnosticCaptureBindingV1& binding) {
    WeaponDiagnosticsReplayV1 replay;
    need(replay.open(path.string()),"Bound diagnostic recording header is invalid");
    need(replay.publisherPid()==binding.publisherPid&&replay.publisherIncarnation()==binding.publisherIncarnation,
        "Bound diagnostic publisher identity differs from its original file");
}
void distinctFiles(const std::filesystem::path& recording,const std::filesystem::path& diagnostics) {
    need(!std::filesystem::equivalent(recording,diagnostics),"Flight and diagnostic inputs must be distinct files");
}
struct TemporaryManifest {
    std::filesystem::path directory,file;
    ~TemporaryManifest() {
        std::error_code ignored;
        if(!file.empty())std::filesystem::remove(file,ignored);
        if(!directory.empty())std::filesystem::remove(directory,ignored);
    }
};
void publishManifest(const std::filesystem::path& finalPath,const std::string& text,
    const std::filesystem::path& recording,const std::filesystem::path& diagnostics,
    FileState recordingState,FileState diagnosticState,
    std::uint64_t publisherPid,const WeaponDiagnosticCaptureLimitsV1& limits) {
    need(text.size()<=limits.maximumManifestBytes,"Capture binding exceeds its manifest bound");
    need(std::filesystem::symlink_status(finalPath).type()==std::filesystem::file_type::not_found,
        "Capture binding was not invalidated before the new recording");
    TemporaryManifest temporary;static std::atomic<std::uint64_t> next{0};
    for(unsigned attempt=0;attempt<8;++attempt) {
        auto candidate=finalPath;
        candidate+=std::string(".tmp.")+std::to_string(publisherPid)+"."+std::to_string(next.fetch_add(1));
        if(std::filesystem::create_directory(candidate)){temporary.directory=std::move(candidate);break;}
    }
    need(!temporary.directory.empty(),"Cannot create a unique bounded capture binding temporary directory");
    temporary.file=temporary.directory/"binding.json";
    std::ofstream stream(temporary.file,std::ios::binary|std::ios::trunc);
    need(bool(stream),"Capture binding temporary file cannot be opened");
    stream.write(text.data(),static_cast<std::streamsize>(text.size()));stream.flush();
    const bool flushed=bool(stream);stream.close();
    need(flushed&&bool(stream),"Capture binding temporary file flush or close failed");
    need(same(recordingState,fileState(recording,limits.maximumFileBytes))&&
        same(diagnosticState,fileState(diagnostics,limits.maximumFileBytes)),"Capture inputs changed before binding publication");
    // Creating a hard link in this same filesystem atomically installs the
    // already CLOSED regular file and refuses EVERY occupied destination,
    // including a dangling symlink or a concurrent publisher. Check+rename
    // alone would overwrite another marker on POSIX. Unsupported filesystems
    // explicitly refuse this optional diagnostic completion operation.
    std::filesystem::create_hard_link(temporary.file,finalPath);
    // The guard removes only our exact temporary link and directory. The final
    // link retains the complete manifest inode and is never removed here.
}
} // namespace

std::filesystem::path weaponDiagnosticCaptureBindingPathV1(const std::filesystem::path& diagnostics) {
    auto out=diagnostics;out+=".binding-v1.json";return out;
}
bool weaponDiagnosticFileDigestV1(const std::filesystem::path& path,WeaponDiagnosticFileDigestV1& out,
    std::string& error,const WeaponDiagnosticCaptureLimitsV1& limits) {
    error.clear();
    try{limitsValid(limits);auto pending=digestFile(path,limits.maximumFileBytes);out=std::move(pending);return true;}
    catch(const std::exception& refused){error=refused.what();return false;}
}
bool invalidateWeaponDiagnosticCaptureBindingV1(const std::filesystem::path& diagnostics,std::string& error) {
    error.clear();
    try {
        need(!diagnostics.empty(),"Diagnostic capture output path is empty");
        const auto path=weaponDiagnosticCaptureBindingPathV1(diagnostics);
        const auto status=std::filesystem::symlink_status(path);
        if(status.type()!=std::filesystem::file_type::not_found) {
            need(status.type()==std::filesystem::file_type::regular||status.type()==std::filesystem::file_type::symlink,
                "Capture completion marker is not a file");
            need(std::filesystem::remove(path),"Previous capture completion marker could not be removed");
        }
        return true;
    } catch(const std::exception& refused){error=refused.what();return false;}
}
bool finalizeWeaponDiagnosticCaptureBindingV1(const std::filesystem::path& recording,
    const std::filesystem::path& diagnostics,const WeaponDiagnosticCaptureCompletionV1& completed,
    std::string& error,const WeaponDiagnosticCaptureLimitsV1& limits) {
    error.clear();
    try {
        limitsValid(limits);
        need(completed.recordingClosedSuccessfully&&completed.diagnosticsClosedSuccessfully,
            "Both native recording writers must close successfully before binding");
        need(completed.publisherPid&&completed.publisherIncarnation&&completed.recordingBytes&&completed.diagnosticsBytes,
            "Capture completion lacks its original publisher or intended byte counts");
        const auto recordingState=fileState(recording,limits.maximumFileBytes),diagnosticState=fileState(diagnostics,limits.maximumFileBytes);
        distinctFiles(recording,diagnostics);
        WeaponDiagnosticCaptureBindingV1 binding;binding.publisherPid=completed.publisherPid;
        binding.publisherIncarnation=completed.publisherIncarnation;
        binding.recording=digestFile(recording,limits.maximumFileBytes);binding.diagnostics=digestFile(diagnostics,limits.maximumFileBytes);
        need(binding.recording.bytes==completed.recordingBytes&&binding.diagnostics.bytes==completed.diagnosticsBytes,
            "Closed capture length differs from its writer's intended byte count");
        recordingHeader(recording);diagnosticHeader(diagnostics,binding);
        publishManifest(weaponDiagnosticCaptureBindingPathV1(diagnostics),document(binding).dump()+"\n",
            recording,diagnostics,recordingState,diagnosticState,completed.publisherPid,limits);
        return true;
    } catch(const std::exception& refused){error=refused.what();return false;}
}
bool verifyWeaponDiagnosticCaptureBindingV1(const std::filesystem::path& recording,
    const std::filesystem::path& diagnostics,WeaponDiagnosticCaptureBindingV1& out,
    std::string& error,const WeaponDiagnosticCaptureLimitsV1& limits) {
    error.clear();
    try {
        limitsValid(limits);const auto manifest=weaponDiagnosticCaptureBindingPathV1(diagnostics);
        const auto manifestBefore=fileState(manifest,static_cast<std::uint64_t>(limits.maximumManifestBytes));
        auto pending=readDocument(manifest,limits);distinctFiles(recording,diagnostics);
        const auto recordingBefore=fileState(recording,limits.maximumFileBytes),diagnosticBefore=fileState(diagnostics,limits.maximumFileBytes);
        const auto actualRecording=digestFile(recording,limits.maximumFileBytes),actualDiagnostics=digestFile(diagnostics,limits.maximumFileBytes);
        need(equal(pending.recording,actualRecording),"Flight recording bytes do not match this diagnostic capture");
        need(equal(pending.diagnostics,actualDiagnostics),"Diagnostic recording bytes do not match their capture binding");
        recordingHeader(recording);diagnosticHeader(diagnostics,pending);
        need(same(manifestBefore,fileState(manifest,static_cast<std::uint64_t>(limits.maximumManifestBytes))),
            "Capture completion marker changed during verification");
        need(same(recordingBefore,fileState(recording,limits.maximumFileBytes))&&
            same(diagnosticBefore,fileState(diagnostics,limits.maximumFileBytes)),
            "Capture inputs changed during file-pair verification");
        static_assert(std::is_nothrow_move_assignable_v<WeaponDiagnosticCaptureBindingV1>);
        out=std::move(pending);return true;
    } catch(const std::exception& refused){error=refused.what();return false;}
}

struct BoundWeaponDiagnosticsReplayV1::State {
    std::filesystem::path recording,diagnostics;
    WeaponDiagnosticCaptureLimitsV1 limits;
    WeaponDiagnosticCaptureBindingV1 binding;
    CaptureWatch watch;
    WeaponDiagnosticsReplayV1 reader;
};
BoundWeaponDiagnosticsReplayV1::BoundWeaponDiagnosticsReplayV1()=default;
BoundWeaponDiagnosticsReplayV1::~BoundWeaponDiagnosticsReplayV1()=default;
void BoundWeaponDiagnosticsReplayV1::close() noexcept {
    state_.reset();error_.clear();
}
bool BoundWeaponDiagnosticsReplayV1::isOpen() const noexcept {return bool(state_);}
std::uint64_t BoundWeaponDiagnosticsReplayV1::publisherPid() const noexcept {
    return state_?state_->binding.publisherPid:0;
}
std::uint32_t BoundWeaponDiagnosticsReplayV1::publisherIncarnation() const noexcept {
    return state_?state_->binding.publisherIncarnation:0;
}
bool BoundWeaponDiagnosticsReplayV1::open(const std::filesystem::path& recording,
    const std::filesystem::path& diagnostics,std::string& error,const WeaponDiagnosticCaptureLimitsV1& limits) {
    close();error.clear();
    try {
        limitsValid(limits);
        auto pending=std::make_unique<State>();pending->recording=recording;
        pending->diagnostics=diagnostics;pending->limits=limits;
        auto before=watchCapture(recording,diagnostics,limits);
        std::string reason;
        const bool verified=verifyWeaponDiagnosticCaptureBindingV1(recording,diagnostics,pending->binding,reason,limits);
        need(verified,reason.empty()?"Native recording capture binding refused":reason.c_str());
        need(pending->reader.open(diagnostics.string()),"Bound native diagnostic reader could not be opened");
        need(same(before,watchCapture(recording,diagnostics,limits)),"Bound capture changed while opening its reader");
        pending->watch=std::move(before);state_=std::move(pending);return true;
    } catch(const std::exception& refused){error_=refused.what();error=error_;return false;}
}
bool BoundWeaponDiagnosticsReplayV1::checkCurrentV1() {
    if(!state_){error_="No completed native recording capture is open";return false;}
    try {
        need(same(state_->watch,watchCapture(state_->recording,state_->diagnostics,state_->limits)),
            "Bound native recording capture changed or was invalidated");
        return true;
    } catch(const std::exception& refused){state_.reset();error_=refused.what();return false;}
}
bool BoundWeaponDiagnosticsReplayV1::rewind() {
    if(!checkCurrentV1())return false;
    try {
        const bool result=state_->reader.rewind();
        if(!checkCurrentV1())return false;
        if(!result){state_.reset();error_="Bound native diagnostic reader could not rewind";return false;}
        error_.clear();return true;
    } catch(const std::exception& refused){state_.reset();error_=refused.what();return false;}
}
WeaponDiagnosticsReplayResultV1 BoundWeaponDiagnosticsReplayV1::next(WeaponDiagnosticFrameV1& out) {
    if(!checkCurrentV1())return WeaponDiagnosticsReplayResultV1::Invalid;
    try {
        auto pending=std::make_unique<WeaponDiagnosticFrameV1>();
        const auto result=state_->reader.next(*pending);
        if(!checkCurrentV1())return WeaponDiagnosticsReplayResultV1::Invalid;
        if(result==WeaponDiagnosticsReplayResultV1::Invalid) {
            state_.reset();error_="Bound native diagnostic record failed original CRC/schema/epoch validation";return result;
        }
        if(result==WeaponDiagnosticsReplayResultV1::Frame) {
            static_assert(std::is_nothrow_copy_assignable_v<WeaponDiagnosticFrameV1>);
            out=*pending;
        }
        error_.clear();return result;
    } catch(const std::exception& refused){state_.reset();error_=refused.what();return WeaponDiagnosticsReplayResultV1::Invalid;}
}
bool BoundWeaponDiagnosticsReplayV1::atTime(double time,WeaponDiagnosticFrameV1& out) {
    if(!checkCurrentV1())return false;
    try {
        auto pending=std::make_unique<WeaponDiagnosticFrameV1>();
        const bool result=state_->reader.atTime(time,*pending);
        if(!checkCurrentV1())return false;
        if(!result){error_="No matching original native diagnostic capture exists at the requested flight time";return false;}
        static_assert(std::is_nothrow_copy_assignable_v<WeaponDiagnosticFrameV1>);
        out=*pending;error_.clear();return true;
    } catch(const std::exception& refused){state_.reset();error_=refused.what();return false;}
}
} // namespace fsim::ipc
