// Frozen public state ABI and raw transport/file records. Relative C/C++ checks
// alone cannot detect both representations growing together.
#include "fsim/ControlInputs.h"
#include "fsim/VehicleState.h"
#include "fsim/fsim_c.h"
#include "ipc/Recording.h"
#include "ipc/WorldLayout.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

namespace {
using State = fsim::sim::VehicleState;
using CState = fsim_vehicle_state;
using Inputs = fsim::sim::ControlInputs;
using CInputs = fsim_control_inputs;
using Sample = fsim::ipc::VehicleSample;
using Header = fsim::ipc::RecordingHeader;
using Row = fsim::ipc::RecordedVehicle;

// Numeric constants are the accepted layout, not values derived from another
// live declaration. Update only with an explicit version/compatibility design.
#define FREEZE_MEMBER(Type, Member, Offset, ...) \
    static_assert(offsetof(Type, Member) == Offset, #Type "::" #Member " offset changed"); \
    static_assert(std::is_same_v<decltype(Type::Member), __VA_ARGS__>, #Type "::" #Member " type changed")
#define FREEZE_STATE(CppMember, CMember, Offset, CppType, CType) \
    FREEZE_MEMBER(State, CppMember, Offset, CppType); \
    FREEZE_MEMBER(CState, CMember, Offset, CType)
#define FREEZE_INPUT(CppMember, CMember, Offset, FieldType) \
    FREEZE_MEMBER(Inputs, CppMember, Offset, FieldType); \
    FREEZE_MEMBER(CInputs, CMember, Offset, FieldType)

static_assert(sizeof(double) == 8 && alignof(double) == 8);
static_assert(std::numeric_limits<double>::is_iec559);
static_assert(sizeof(float) == 4 && sizeof(int) == 4 && sizeof(bool) == 1);
static_assert(std::is_standard_layout_v<State> && std::is_trivially_copyable_v<State>);
static_assert(std::is_standard_layout_v<CState> && std::is_trivially_copyable_v<CState>);
static_assert(sizeof(State) == 800 && alignof(State) == 8);
static_assert(sizeof(CState) == 800 && alignof(CState) == 8);
static_assert(State::kMaxEngines == 4 && FSIM_MAX_ENGINES == 4);
static_assert(State::kMaxWheels == 8 && FSIM_MAX_WHEELS == 8);
FREEZE_STATE(simTime, sim_time, 0, double, double);
FREEZE_STATE(positionEcef, position_ecef, 8, double[3], double[3]);
FREEZE_STATE(attitudeEcefToBody, attitude_ecef_to_body, 32, double[4], double[4]);
FREEZE_STATE(latitudeRad, latitude_rad, 64, double, double);
FREEZE_STATE(longitudeRad, longitude_rad, 72, double, double);
FREEZE_STATE(altitudeMslM, altitude_msl_m, 80, double, double);
FREEZE_STATE(altitudeAglM, altitude_agl_m, 88, double, double);
FREEZE_STATE(eulerRad, euler_rad, 96, double[3], double[3]);
FREEZE_STATE(velocityBodyMs, velocity_body_ms, 120, double[3], double[3]);
FREEZE_STATE(velocityNedMs, velocity_ned_ms, 144, double[3], double[3]);
FREEZE_STATE(angularRateBodyRadS, angular_rate_body_rad_s, 168, double[3], double[3]);
FREEZE_STATE(accelerationBodyMs2, acceleration_body_ms2, 192, double[3], double[3]);
FREEZE_STATE(airspeedTrueMs, airspeed_true_ms, 216, double, double);
FREEZE_STATE(airspeedCalibratedMs, airspeed_calibrated_ms, 224, double, double);
FREEZE_STATE(mach, mach, 232, double, double);
FREEZE_STATE(alphaRad, alpha_rad, 240, double, double);
FREEZE_STATE(betaRad, beta_rad, 248, double, double);
FREEZE_STATE(loadFactor, load_factor, 256, double, double);
FREEZE_STATE(aileronRad, aileron_rad, 264, double, double);
FREEZE_STATE(elevatorRad, elevator_rad, 272, double, double);
FREEZE_STATE(rudderRad, rudder_rad, 280, double, double);
FREEZE_STATE(flapsRad, flaps_rad, 288, double, double);
FREEZE_STATE(gearPosition, gear_position, 296, double, double);
FREEZE_STATE(engineCount, engine_count, 304, int, std::int32_t);
FREEZE_STATE(speedbrakePosition, speedbrake_position, 308, float, float);
FREEZE_STATE(throttlePosition, throttle_position, 312, double[4], double[4]);
FREEZE_STATE(thrustN, thrust_n, 344, double[4], double[4]);
FREEZE_STATE(fuelKg, fuel_kg, 376, double, double);
FREEZE_STATE(stepCount, step_count, 384, std::uint32_t, std::uint32_t);
FREEZE_STATE(onGround, on_ground, 388, bool, std::uint8_t);
FREEZE_STATE(diverged, diverged, 389, bool, std::uint8_t);
FREEZE_STATE(crashed, crashed, 390, bool, std::uint8_t);
FREEZE_STATE(rotationBodyToEcef, rotation_body_to_ecef, 392, double[9], double[9]);
FREEZE_STATE(engineRpm, engine_rpm, 464, double[4], double[4]);
FREEZE_STATE(engineN2, engine_n2, 496, double[4], double[4]);
FREEZE_STATE(afterburner, afterburner, 528, double[4], double[4]);
FREEZE_STATE(nozzlePosition, nozzle_position, 560, double[4], double[4]);
FREEZE_STATE(leadingEdgeFlapRad, leading_edge_flap_rad, 592, double, double);
FREEZE_STATE(wheelCount, wheel_count, 600, int, std::int32_t);
FREEZE_STATE(wheelCompressionM, wheel_compression_m, 608, double[8], double[8]);
FREEZE_STATE(wheelSteerRad, wheel_steer_rad, 672, double[8], double[8]);
FREEZE_STATE(wheelSpeedMs, wheel_speed_ms, 736, double[8], double[8]);

static_assert(std::is_standard_layout_v<Inputs> && std::is_trivially_copyable_v<Inputs>);
static_assert(std::is_standard_layout_v<CInputs> && std::is_trivially_copyable_v<CInputs>);
static_assert(sizeof(Inputs) == 88 && alignof(Inputs) == 8);
static_assert(sizeof(CInputs) == 88 && alignof(CInputs) == 8);
static_assert(Inputs::kMaxEngines == 4);
FREEZE_INPUT(aileron, aileron, 0, double);
FREEZE_INPUT(elevator, elevator, 8, double);
FREEZE_INPUT(rudder, rudder, 16, double);
FREEZE_INPUT(throttle, throttle, 24, double[4]);
FREEZE_INPUT(flaps, flaps, 56, double);
FREEZE_INPUT(gearDown, gear_down, 64, double);
FREEZE_INPUT(brakeLeft, brake_left, 72, double);
FREEZE_INPUT(brakeRight, brake_right, 80, double);

static_assert(fsim::ipc::kLayoutVersion == 9);
static_assert(std::is_standard_layout_v<Sample> && std::is_trivially_copyable_v<Sample>);
static_assert(sizeof(Sample) == 1000 && alignof(Sample) == 8);
FREEZE_MEMBER(Sample, state, 0, State);
FREEZE_MEMBER(Sample, inputs, 800, Inputs);
FREEZE_MEMBER(Sample, derived, 888, double[14]);
static_assert(fsim::ipc::kDerivedCountV7 == 8 && fsim::ipc::kDerivedCountV8 == 10);
static_assert(fsim::ipc::kDerivedCount == 14);
static_assert(fsim::ipc::kDerivedMassKg == 0 && fsim::ipc::kDerivedLoadRatio == 6);
static_assert(fsim::ipc::kDerivedCgFromOriginM == 7 && fsim::ipc::kDerivedFuelFlowKgS == 10);
static_assert(offsetof(State, engineRpm) == 464, "recording v1 state prefix changed");
static_assert(offsetof(Sample, derived) + 8 * sizeof(double) == 952, "recording v2-v5 sample changed");
static_assert(offsetof(Sample, derived) + 10 * sizeof(double) == 968, "recording v6 sample changed");
static_assert(464 + sizeof(Inputs) + 8 * sizeof(double) == 616, "recording v1 sample changed");
static_assert(std::is_standard_layout_v<fsim_recorded_sample> && std::is_trivially_copyable_v<fsim_recorded_sample>);
static_assert(sizeof(fsim_recorded_sample) == 896 && alignof(fsim_recorded_sample) == 8);
FREEZE_MEMBER(fsim_recorded_sample, slot, 0, std::uint32_t);
FREEZE_MEMBER(fsim_recorded_sample, state, 8, CState);
FREEZE_MEMBER(fsim_recorded_sample, inputs, 808, CInputs);

static_assert(std::is_standard_layout_v<Header> && std::is_trivially_copyable_v<Header>);
static_assert(sizeof(Header) == 88 && alignof(Header) == 8);
FREEZE_MEMBER(Header, capacity, 0, std::uint32_t);
FREEZE_MEMBER(Header, dt, 8, double);
FREEZE_MEMBER(Header, frameSkip, 16, std::int32_t);
FREEZE_MEMBER(Header, name, 20, char[64]);
static_assert(std::is_standard_layout_v<Row> && std::is_trivially_copyable_v<Row>);
static_assert(sizeof(Row) == 440 && alignof(Row) == 8);
FREEZE_MEMBER(Row, id, 0, std::uint32_t);
FREEZE_MEMBER(Row, generation, 8, std::uint64_t);
FREEZE_MEMBER(Row, alive, 16, std::uint8_t);
FREEZE_MEMBER(Row, controlLevel, 17, std::uint8_t);
FREEZE_MEMBER(Row, name, 18, char[64]);
FREEZE_MEMBER(Row, type, 82, char[64]);
FREEZE_MEMBER(Row, model, 146, char[256]);
FREEZE_MEMBER(Row, initialLatitudeDeg, 408, double);
FREEZE_MEMBER(Row, initialLongitudeDeg, 416, double);
FREEZE_MEMBER(Row, initialAltitudeMslM, 424, double);
FREEZE_MEMBER(Row, initialHeadingDeg, 432, double);

// The same protocol version must retain header/table/mission/event strides as well as flight samples.
#define FREEZE_WIRE(Type, Size, Alignment) \
    static_assert(sizeof(fsim::ipc::Type) == Size && alignof(fsim::ipc::Type) == Alignment, #Type " wire layout changed")
using Environment = fsim::sim::EnvironmentState;
using WorldHeader = fsim::ipc::WorldHeader;
using VehicleRecord = fsim::ipc::VehicleRecord;
using SlotHeader = fsim::ipc::StateSlotHeader;
using Event = fsim::ipc::EngagementEvent;
using Store = fsim::ipc::MissionStore;
using Mission = fsim::ipc::MissionRecord;
static_assert(sizeof(Environment) == 112 && alignof(Environment) == 8);
FREEZE_MEMBER(Environment, epochUtcSeconds, 0, double);
FREEZE_MEMBER(Environment, timeFactor, 8, double);
FREEZE_MEMBER(Environment, temperatureSeaLevelK, 16, double);
FREEZE_MEMBER(Environment, pressureSeaLevelPa, 24, double);
FREEZE_MEMBER(Environment, humidity, 32, double);
FREEZE_MEMBER(Environment, windDirectionDeg, 40, double);
FREEZE_MEMBER(Environment, windSpeedMs, 48, double);
FREEZE_MEMBER(Environment, windGustMs, 56, double);
FREEZE_MEMBER(Environment, turbulence, 64, double);
FREEZE_MEMBER(Environment, visibilityM, 72, double);
FREEZE_MEMBER(Environment, cloudBaseM, 80, double);
FREEZE_MEMBER(Environment, cloudCover, 88, double);
FREEZE_MEMBER(Environment, precipitation, 96, double);
FREEZE_MEMBER(Environment, revision, 104, std::uint64_t);
FREEZE_WIRE(WorldHeader, 384, 64);
FREEZE_MEMBER(WorldHeader, magic, 0, std::uint32_t);
FREEZE_MEMBER(WorldHeader, layoutVersion, 4, std::uint32_t);
FREEZE_MEMBER(WorldHeader, headerSize, 8, std::uint32_t);
FREEZE_MEMBER(WorldHeader, capacity, 12, std::uint32_t);
FREEZE_MEMBER(WorldHeader, publisherPid, 16, std::uint64_t);
FREEZE_MEMBER(WorldHeader, name, 24, char[64]);
FREEZE_MEMBER(WorldHeader, dt, 88, double);
FREEZE_MEMBER(WorldHeader, frameSkip, 96, std::int32_t);
FREEZE_MEMBER(WorldHeader, focusSequence, 100, std::atomic<std::uint32_t>);
FREEZE_MEMBER(WorldHeader, tableOffset, 104, std::uint64_t);
FREEZE_MEMBER(WorldHeader, slotOffset, 112, std::uint64_t);
FREEZE_MEMBER(WorldHeader, slotStride, 120, std::uint64_t);
FREEZE_MEMBER(WorldHeader, totalSize, 128, std::uint64_t);
FREEZE_MEMBER(WorldHeader, alive, 136, std::atomic<std::uint32_t>);
FREEZE_MEMBER(WorldHeader, readers, 140, std::atomic<std::uint32_t>);
FREEZE_MEMBER(WorldHeader, tableGeneration, 144, std::atomic<std::uint64_t>);
FREEZE_MEMBER(WorldHeader, latestSlot, 152, std::atomic<std::uint32_t>);
FREEZE_MEMBER(WorldHeader, focusVehicle, 156, std::atomic<std::uint32_t>);
FREEZE_MEMBER(WorldHeader, publishSequence, 160, std::atomic<std::uint64_t>);
FREEZE_MEMBER(WorldHeader, lastPublishWallNs, 168, std::atomic<std::int64_t>);
FREEZE_MEMBER(WorldHeader, vehicleSteps, 176, std::atomic<std::uint64_t>);
FREEZE_MEMBER(WorldHeader, worldSteps, 184, std::atomic<std::uint64_t>);
FREEZE_MEMBER(WorldHeader, environmentSeq, 192, fsim::ipc::SeqLock);
FREEZE_MEMBER(WorldHeader, incarnation, 196, std::atomic<std::uint32_t>);
FREEZE_MEMBER(WorldHeader, environment, 200, Environment);
FREEZE_MEMBER(WorldHeader, missionOffset, 312, std::uint64_t);
FREEZE_MEMBER(WorldHeader, missionStride, 320, std::uint64_t);
FREEZE_MEMBER(WorldHeader, engagementOffset, 328, std::uint64_t);
FREEZE_WIRE(VehicleRecord, 440, 8);
FREEZE_MEMBER(VehicleRecord, seq, 0, fsim::ipc::SeqLock);
FREEZE_MEMBER(VehicleRecord, id, 4, std::uint32_t);
FREEZE_MEMBER(VehicleRecord, generation, 8, std::uint64_t);
FREEZE_MEMBER(VehicleRecord, alive, 16, std::uint8_t);
FREEZE_MEMBER(VehicleRecord, controlLevel, 17, std::uint8_t);
FREEZE_MEMBER(VehicleRecord, reserved, 18, std::uint8_t[6]);
FREEZE_MEMBER(VehicleRecord, name, 24, char[64]);
FREEZE_MEMBER(VehicleRecord, type, 88, char[64]);
FREEZE_MEMBER(VehicleRecord, model, 152, char[256]);
FREEZE_MEMBER(VehicleRecord, initialLatitudeDeg, 408, double);
FREEZE_MEMBER(VehicleRecord, initialLongitudeDeg, 416, double);
FREEZE_MEMBER(VehicleRecord, initialAltitudeMslM, 424, double);
FREEZE_MEMBER(VehicleRecord, initialHeadingDeg, 432, double);
FREEZE_WIRE(StateSlotHeader, 32, 8);
FREEZE_MEMBER(SlotHeader, seq, 0, fsim::ipc::SeqLock);
FREEZE_MEMBER(SlotHeader, reserved, 4, std::uint32_t);
FREEZE_MEMBER(SlotHeader, sequence, 8, std::uint64_t);
FREEZE_MEMBER(SlotHeader, simTime, 16, double);
FREEZE_MEMBER(SlotHeader, wallNs, 24, std::int64_t);
FREEZE_WIRE(MissionPayload, 136, 4);
FREEZE_WIRE(MissionDetection, 32, 8);
FREEZE_WIRE(MissionTrack, 48, 4);
FREEZE_WIRE(MissionRadar, 5480, 8);
FREEZE_WIRE(MissionRangeDoppler, 8384, 8);
FREEZE_WIRE(MissionStore, 40, 1);
FREEZE_MEMBER(Store, station, 0, std::uint8_t);
FREEZE_MEMBER(Store, status, 1, std::uint8_t);
FREEZE_MEMBER(Store, bayDoors, 2, std::uint8_t);
FREEZE_MEMBER(Store, reserved, 3, std::uint8_t);
FREEZE_MEMBER(Store, mnemonic, 4, char[20]);
FREEZE_MEMBER(Store, store, 24, char[16]);
FREEZE_WIRE(MissionRecord, 22840, 8);
FREEZE_MEMBER(Mission, seq, 0, fsim::ipc::SeqLock);
FREEZE_MEMBER(Mission, vehicleId, 4, std::uint32_t);
FREEZE_MEMBER(Mission, revision, 8, std::uint64_t);
FREEZE_MEMBER(Mission, simTime, 16, double);
FREEZE_MEMBER(Mission, payloadCount, 24, std::uint32_t);
FREEZE_MEMBER(Mission, radarCount, 28, std::uint32_t);
FREEZE_MEMBER(Mission, payloads, 32, fsim::ipc::MissionPayload[16]);
FREEZE_MEMBER(Mission, radars, 2208, fsim::ipc::MissionRadar[2]);
FREEZE_MEMBER(Mission, rangeDoppler, 13168, fsim::ipc::MissionRangeDoppler);
FREEZE_MEMBER(Mission, storeCount, 21552, std::uint32_t);
FREEZE_MEMBER(Mission, reserved3, 21556, std::uint32_t);
FREEZE_MEMBER(Mission, stores, 21560, Store[32]);
FREEZE_WIRE(EngagementEvent, 176, 8);
FREEZE_MEMBER(Event, kind, 0, std::uint8_t);
FREEZE_MEMBER(Event, reserved, 1, std::uint8_t[3]);
FREEZE_MEMBER(Event, slot, 4, std::uint32_t);
FREEZE_MEMBER(Event, target, 8, std::uint32_t);
FREEZE_MEMBER(Event, index, 12, std::uint32_t);
FREEZE_MEMBER(Event, simTime, 16, double);
FREEZE_MEMBER(Event, positionEcef, 24, double[3]);
FREEZE_MEMBER(Event, velocityEcef, 48, double[3]);
FREEZE_MEMBER(Event, amountKg, 72, double);
FREEZE_MEMBER(Event, massKg, 80, double);
FREEZE_MEMBER(Event, rateKgS, 88, double);
FREEZE_MEMBER(Event, name, 96, char[32]);
FREEZE_MEMBER(Event, normalEcef, 128, double[3]);
FREEZE_MEMBER(Event, energyJ, 152, double);
FREEZE_MEMBER(Event, angleRad, 160, double);
FREEZE_MEMBER(Event, loadG, 168, double);
FREEZE_WIRE(EngagementRing, 45064, 8);
FREEZE_MEMBER(fsim::ipc::EngagementRing, written, 0, std::atomic<std::uint64_t>);
FREEZE_MEMBER(fsim::ipc::EngagementRing, events, 8, Event[256]);

#undef FREEZE_WIRE

#undef FREEZE_INPUT
#undef FREEZE_STATE
#undef FREEZE_MEMBER

using Bytes = std::vector<unsigned char>;

void putUnsigned(Bytes& bytes, std::size_t offset, std::uint64_t value, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) bytes.at(offset + i) = static_cast<unsigned char>(value >> (i * 8));
}
void putDouble(Bytes& bytes, std::size_t offset, double value) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, 8);
    putUnsigned(bytes, offset, bits, 8);
}
void putText(Bytes& bytes, std::size_t offset, const std::string& value) {
    for (std::size_t i = 0; i < value.size(); ++i) bytes.at(offset + i) = static_cast<unsigned char>(value[i]);
}
void append(Bytes& bytes, const Bytes& part) { bytes.insert(bytes.end(), part.begin(), part.end()); }
void appendU32(Bytes& bytes, std::uint32_t value) {
    Bytes part(4);
    putUnsigned(part, 0, value, 4);
    append(bytes, part);
}

// Literal extents also exercise old files independently of the current structs.
Bytes recordingFixture(unsigned char version) {
    Bytes bytes{'F', 'S', 'R', 'E', 'C', version};
    Bytes header(88);
    putUnsigned(header, 0, 16, 4);
    putDouble(header, 8, 0.125);
    putUnsigned(header, 16, 4, 4);
    putText(header, 20, "frozen-layout");
    append(bytes, header);
    bytes.push_back('T');
    appendU32(bytes, 5);
    Bytes row(440);
    putUnsigned(row, 0, 101, 4);
    putUnsigned(row, 8, 0x123456789abcdef0ULL, 8);
    row[16] = 1;
    row[17] = 3;
    putText(row, 18, "frozen-aircraft");
    putText(row, 82, "jsbsim:test");
    putText(row, 146, "visual.glb");
    putDouble(row, 408, 12.0);
    putDouble(row, 416, 23.0);
    putDouble(row, 424, 3456.0);
    putDouble(row, 432, 67.0);
    append(bytes, row);
    bytes.push_back('S');
    Bytes snapshot(16);
    putDouble(snapshot, 0, 42.5);
    putUnsigned(snapshot, 8, 2, 4);
    append(bytes, snapshot);
    for (unsigned i = 0; i < 2; ++i) {
        appendU32(bytes, 5 + i);
        Bytes sample(version == 1 ? 616 : version < 6 ? 952 : version == 6 ? 968 : 1000);
        putDouble(sample, 0, 42.5 + i);
        putDouble(sample, 8, 1234.0 + i);
        putDouble(sample, 216, 250.0 + i);
        putUnsigned(sample, 304, 2, 4);
        putUnsigned(sample, 384, 123 + i, 4);
        sample[388] = 1;
        putDouble(sample, 392, 1.0);
        if (version != 1) {
            putDouble(sample, 464, 1500.0 + i);
            putDouble(sample, 560, 0.75);
            putUnsigned(sample, 600, 3, 4);
            putDouble(sample, 736, 30.0 + i);
        }
        const std::size_t inputs = version == 1 ? 464 : 800;
        putDouble(sample, inputs, -0.25);
        putDouble(sample, inputs + 24, 0.625);
        putDouble(sample, inputs + 80, 0.5);
        const std::size_t derived = version == 1 ? 552 : 888;
        const unsigned count = version < 6 ? 8 : version == 6 ? 10 : 14;
        for (unsigned k = 0; k < count; ++k) putDouble(sample, derived + k * 8, 100.0 + k + i);
        append(bytes, sample);
    }
    return bytes;
}

struct ScratchRecording {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("fsim-frozen-layout-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".fsrec");
    ~ScratchRecording() { std::error_code error; std::filesystem::remove(path, error); }
};
} // namespace

TEST_CASE("public state and recording representations preserve frozen layouts", "[boundary][layout]") {
    // Two sample strides and alignment rounding must not use an enlarged state.
    REQUIRE(fsim::ipc::Layout::slotStride(1) == 1088);
    REQUIRE(fsim::ipc::Layout::slotStride(2) == 2112);
    REQUIRE(fsim::ipc::Layout::slotStride(8) == 8064);

    ScratchRecording file;
    for (unsigned char version = 1; version <= 7; ++version) {
        INFO("frozen FSREC version " << static_cast<unsigned>(version));
        const auto bytes = recordingFixture(version);
        {
            std::ofstream stream(file.path, std::ios::binary | std::ios::trunc);
            REQUIRE(stream.is_open());
            stream.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
            REQUIRE(stream.good());
        }
        fsim::ipc::Recording recording;
        std::string error;
        const bool loaded = recording.load(file.path, &error);
        INFO(error);
        REQUIRE(loaded);
        REQUIRE(recording.header().capacity == 16);
        REQUIRE(recording.header().dt == 0.125);
        REQUIRE(recording.header().frameSkip == 4);
        REQUIRE(std::string(recording.header().name) == "frozen-layout");
        REQUIRE(recording.frames().size() == 1);
        const auto& frame = recording.frames().front();
        REQUIRE(frame.simTime == 42.5);
        REQUIRE(frame.tableChanges.size() == 1);
        REQUIRE(frame.tableChanges[0].first == 5);
        const auto& row = frame.tableChanges[0].second;
        REQUIRE(row.id == 101);
        REQUIRE(row.generation == 0x123456789abcdef0ULL);
        REQUIRE(row.alive == 1);
        REQUIRE(row.controlLevel == 3);
        REQUIRE(std::string(row.name) == "frozen-aircraft");
        REQUIRE(std::string(row.type) == "jsbsim:test");
        REQUIRE(std::string(row.model) == "visual.glb");
        REQUIRE(row.initialLatitudeDeg == 12.0);
        REQUIRE(row.initialLongitudeDeg == 23.0);
        REQUIRE(row.initialAltitudeMslM == 3456.0);
        REQUIRE(row.initialHeadingDeg == 67.0);
        REQUIRE(frame.samples.size() == 2);
        for (unsigned i = 0; i < 2; ++i) {
            REQUIRE(frame.samples[i].first == 5 + i);
            const auto& sample = frame.samples[i].second;
            REQUIRE(sample.state.simTime == 42.5 + i);
            REQUIRE(sample.state.positionEcef[0] == 1234.0 + i);
            REQUIRE(sample.state.airspeedTrueMs == 250.0 + i);
            REQUIRE(sample.state.engineCount == 2);
            REQUIRE(sample.state.stepCount == 123 + i);
            REQUIRE(sample.state.onGround);
            REQUIRE(sample.state.rotationBodyToEcef[0] == 1.0);
            REQUIRE(sample.state.engineRpm[0] == (version == 1 ? 0.0 : 1500.0 + i));
            REQUIRE(sample.state.nozzlePosition[0] == (version == 1 ? 0.0 : 0.75));
            REQUIRE(sample.state.wheelCount == (version == 1 ? 0 : 3));
            REQUIRE(sample.state.wheelSpeedMs[0] == (version == 1 ? 0.0 : 30.0 + i));
            REQUIRE(sample.inputs.aileron == -0.25);
            REQUIRE(sample.inputs.throttle[0] == 0.625);
            REQUIRE(sample.inputs.brakeRight == 0.5);
            const unsigned count = version < 6 ? 8 : version == 6 ? 10 : 14;
            for (unsigned k = 0; k < 14; ++k) REQUIRE(sample.derived[k] == (k < count ? 100.0 + k + i : 0.0));
        }
    }

    // The writer must retain v7, not silently write enlarged records under it.
    fsim::ipc::RecordWriter writer;
    REQUIRE(writer.open(file.path, Header{}));
    writer.writeVehicle(0, Row{});
    writer.writeSnapshot(0.0, {{0, Sample{}}, {1, Sample{}}});
    writer.close();
    std::ifstream stream(file.path, std::ios::binary);
    const std::vector<char> written{std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
    REQUIRE(written.size() == 2564);
    REQUIRE(std::string(written.data(), 5) == "FSREC");
    REQUIRE(static_cast<unsigned char>(written[5]) == 7);
}
