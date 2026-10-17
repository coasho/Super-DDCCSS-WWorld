// A5: public batch and recording contracts before adapter extraction.
#include "fsim/fsim_c.h"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {
#define CHECK(condition) do { if (!(condition)) { \
    std::cerr << __FILE__ << ':' << __LINE__ << ": " #condition \
              << " (" << fsim_last_error() << ")\n"; return 1; \
} } while (false)

struct Batch {
    fsim_vecenv* value = nullptr;
    ~Batch() { fsim_vecenv_destroy(value); }
};
struct World {
    fsim_world* value = nullptr;
    ~World() { fsim_world_destroy(value); }
};
struct Recording {
    fsim_recording* value = nullptr;
    ~Recording() { fsim_recording_destroy(value); }
};
struct File {
    std::filesystem::path path;
    ~File() { std::error_code error; std::filesystem::remove(path, error); }
};

int run(const char* root) {
    fsim_options options;
    fsim_options_init(&options);
    options.publish = 0;
    options.workers = 1;
    options.num_envs = 2;
    options.seed = 7;
    options.jsbsim_root = root;
    Batch batch;
    CHECK(fsim_vecenv_create(&options, &batch.value) == FSIM_OK);
    fsim_world* borrowed = fsim_vecenv_world(batch.value);
    CHECK(borrowed && borrowed == fsim_vecenv_world(batch.value));
    fsim_world_destroy(borrowed); // The owner retains a borrowed C handle.
    uint32_t ids[2]{};
    CHECK(fsim_vecenv_vehicle_ids(batch.value, ids, 2) == 2);
    CHECK(std::strcmp(fsim_vehicle_name(borrowed, ids[1]), "env1/0") == 0);
    CHECK(fsim_vecenv_world(nullptr) == nullptr);

    fsim_buffers initial{};
    initial.struct_size = sizeof(initial);
    CHECK(fsim_vecenv_buffers(batch.value, &initial) == FSIM_OK);
    CHECK(initial.observations && initial.rewards && initial.terminated && initial.truncated);
    CHECK(initial.final_observations && initial.episode_steps);
    std::vector<float> actions(initial.num_envs * initial.vehicles_per_env * initial.action_size, 0.0f);
    CHECK(fsim_vecenv_set_autoreset(batch.value, 99) == FSIM_INVALID_ARGUMENT);
    const std::string mode_error = fsim_last_error();
    CHECK(mode_error.find("fsim_vecenv_set_autoreset") != std::string::npos);
    CHECK(fsim_vecenv_autoreset(batch.value) == FSIM_AUTORESET_NEXT_STEP);
    CHECK(fsim_vecenv_reset(nullptr, 1) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_vecenv_step(nullptr, actions.data(), actions.size()) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_vecenv_set_action_ranges(batch.value, nullptr) == FSIM_INVALID_ARGUMENT);
    CHECK(mode_error == fsim_last_error());
    fsim_buffers short_buffers = initial;
    short_buffers.struct_size = sizeof(short_buffers) - 1;
    const fsim_buffers untouched = short_buffers;
    CHECK(fsim_vecenv_buffers(batch.value, &short_buffers) == FSIM_INVALID_ARGUMENT);
    CHECK(std::memcmp(&short_buffers, &untouched, sizeof(untouched)) == 0);
    CHECK(mode_error == fsim_last_error());
    CHECK(fsim_vecenv_set_autoreset(batch.value, FSIM_AUTORESET_SAME_STEP) == FSIM_OK);
    CHECK(fsim_vecenv_autoreset(batch.value) == FSIM_AUTORESET_SAME_STEP);
    CHECK(mode_error == fsim_last_error()); // Direct setters do not clear it.
    fsim_options short_options = options;
    short_options.struct_size = sizeof(short_options) - 1;
    fsim_vecenv* retained = batch.value;
    CHECK(fsim_vecenv_create(&short_options, &retained) == FSIM_INVALID_ARGUMENT);
    CHECK(retained == batch.value);
    CHECK(std::strstr(fsim_last_error(), "fsim_vecenv_create"));
    CHECK(fsim_vecenv_reset(batch.value, 7) == FSIM_OK);
    CHECK(std::strlen(fsim_last_error()) == 0);
    CHECK(fsim_vecenv_step(batch.value, actions.data(), actions.size() - 1) == FSIM_INVALID_ARGUMENT);
    CHECK(std::strstr(fsim_last_error(), "fsim_vecenv_step"));
    CHECK(fsim_vecenv_step(batch.value, actions.data(), actions.size()) == FSIM_OK);
    CHECK(std::strlen(fsim_last_error()) == 0);
    fsim_buffers after{};
    after.struct_size = sizeof(after);
    CHECK(fsim_vecenv_buffers(batch.value, &after) == FSIM_OK);
    CHECK(initial.observations == after.observations && initial.rewards == after.rewards);
    CHECK(initial.terminated == after.terminated && initial.truncated == after.truncated);
    CHECK(initial.final_observations == after.final_observations && initial.episode_steps == after.episode_steps);

    File file{std::filesystem::temp_directory_path() / ("a5-adapter-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".fsrec")};
    const std::string path = file.path.string();
    World world;
    fsim_world_options wo;
    fsim_world_options_init(&wo);
    wo.name = "batch-recording-adapter";
    wo.publish = 0;
    wo.workers = 1;
    wo.pin_workers = 0;
    wo.jsbsim_root = root;
    wo.record_path = path.c_str();
    CHECK(fsim_world_create(&wo, &world.value) == FSIM_OK);
    fsim_vehicle_spec spec;
    fsim_vehicle_spec_init(&spec);
    spec.name = "recorded";
    spec.altitude_msl_m = 1200.0;
    spec.airspeed_ms = 60.0;
    uint32_t id = 0;
    CHECK(fsim_world_create_vehicle(world.value, &spec, &id) == FSIM_OK);
    CHECK(fsim_world_step(world.value, 3) == FSIM_OK);
    fsim_world_destroy(world.value);
    world.value = nullptr;
    Recording recording;
    CHECK(fsim_recording_load(path.c_str(), &recording.value) == FSIM_OK);
    CHECK(fsim_recording_frame_count(recording.value) == 3);
    const char* name = fsim_recording_world_name(recording.value);
    uint32_t count = 0;
    const auto* samples = fsim_recording_samples(recording.value, 1, &count);
    CHECK(samples && count == 1);
    const auto* events = fsim_recording_events(recording.value, 0, &count);
    CHECK(events && count == 1 && events[0].id == id && events[0].alive == 1);
    const char* event_name = events[0].name;
    const char* event_type = events[0].type;
    const char* event_model = events[0].model;
    const std::string type = event_type, model = event_model;
    CHECK(std::strcmp(event_name, "recorded") == 0 && !type.empty());
    fsim_recording* retained_recording = recording.value;
    CHECK(fsim_recording_load(nullptr, &retained_recording) == FSIM_INVALID_ARGUMENT);
    CHECK(retained_recording == recording.value);
    CHECK(fsim_recording_load(path.c_str(), nullptr) == FSIM_INVALID_ARGUMENT);
    const std::string argument_error = fsim_last_error();
    CHECK(!argument_error.empty());
    CHECK(fsim_recording_frame_count(nullptr) == 0 && fsim_recording_frame_time(nullptr, 0) == 0.0);
    CHECK(fsim_recording_dt(nullptr) == 0.0 && fsim_recording_frame_skip(nullptr) == 0);
    CHECK(std::strcmp(fsim_recording_world_name(nullptr), "") == 0);
    for (const auto* r : {static_cast<const fsim_recording*>(nullptr),
                         static_cast<const fsim_recording*>(recording.value)}) {
        count = 99;
        CHECK(fsim_recording_samples(r, 99, &count) == nullptr && count == 0);
        count = 99;
        CHECK(fsim_recording_events(r, 99, &count) == nullptr && count == 0);
        CHECK(fsim_recording_samples(r, 99, nullptr) == nullptr);
        CHECK(fsim_recording_events(r, 99, nullptr) == nullptr);
        CHECK(fsim_recording_frame_time(r, 99) == 0.0);
    }
    CHECK(argument_error == fsim_last_error());
    Recording failed;
    failed.value = recording.value;
    const std::string missing = path + ".missing";
    CHECK(fsim_recording_load(missing.c_str(), &failed.value) == FSIM_ERROR);
    CHECK(failed.value == nullptr && std::strstr(fsim_last_error(), "fsim_recording_load"));
    Recording other;
    CHECK(fsim_recording_load(path.c_str(), &other.value) == FSIM_OK);
    CHECK(std::strlen(fsim_last_error()) == 0);
    CHECK(samples == fsim_recording_samples(recording.value, 1, nullptr));
    CHECK(events == fsim_recording_events(recording.value, 0, nullptr));
    CHECK(name == fsim_recording_world_name(recording.value));
    CHECK(std::strcmp(name, wo.name) == 0 && std::strcmp(event_name, "recorded") == 0);
    CHECK(type == event_type && model == event_model);
    fsim_recording_destroy(nullptr);
    return 0;
}
} // namespace

int main(int argc, char** argv) { return run(argc > 1 ? argv[1] : nullptr); }
