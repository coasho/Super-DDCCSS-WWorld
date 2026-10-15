# Mission training contract, version 1

Status: **frozen contract, implemented in part**. Generic M0 froze this page, its
[schema](ma-contract.schema.json) and its [fixtures](../../tests/fixtures/mission_training/contract/manifest.json) on
2026-10-12 against main `e1945b98732586a31b8a8d05c276fc34f4a2d737` (delivered baseline
`baf476f841970f1f3c90b8de0bf630e8a5478738`). Each package of the
[implementation plan](../superpowers/plans/2026-10-12-ma-system-training-readiness.md) implements its part against
this contract and moves its entry points from "proposed" here into [boundary.toml](boundary.toml) when they are
exported.

| Part | State |
| --- | --- |
| Explicit MA-service binding ([section 9](#9-service-binding)) | **Implemented** (M1) in C++, C and Python |
| Legal noncombat reports with their provenance ([section 10](#legal-report-calls-noncombat-m2)): navigation, own tracks, passive contacts, link tracks, peers, tasks and authority as Agent calls | **Implemented** (noncombat M2) in C++, C and Python. The `ma.observation.v1` document that carries them is not |
| Complete native-state reset fixture input ([fixture](native-state-reset-fixture.md)) | **Supplied, 2026-10-15**, and applied to the M4 episode reset by the `mission_episode` gate |
| Independent episode ownership, full fresh reset, generation-bound children and seed streams ([sections 4](#4-ownership-generation-lifetime-and-move) and [8](#8-seed-streams)) | **Implemented** (M4) as the internal native owner `fsim::env::MissionEpisode` (`src/env/MissionEpisode.h`). Not exported: no public header, C function or Python class yet |
| Everything else: the public mission episode, endpoint and referee family, every `ma.*.v1` document (`ma.config.v1` parsing among them), the gateway, receipts, criteria, isolation | **Proposed.** No header, export, C function or Python class for it exists; `fsim.isolation` has its present `AgentProcess` only |

The [SDK's principals](boundary.md) list what the SDK really exports.

This is a local SDK contract for the [accepted design](../superpowers/specs/2026-10-12-ma-system-training-readiness-design.md).
It is not an ASK or UCI wire format and claims no ASK compliance. The upper-layer side - components, arbitration and
lifecycle hooks - is in [ma-composition.md](ma-composition.md). What the SDK was before any of it is in the
[compatibility baseline](ma-compatibility-baseline.md).

Words used with one meaning throughout: **frozen** - a later package may not change it without a new version;
**proposed** - named and specified here, not yet built; **pending** - waits for an input or a runtime that is not
in place yet ([section 15](#15-extension-seam-and-pending-gates)); **native** - what the existing SDK does today, which
this contract transports and never redefines.

## 1. Scope

In version 1: the generic envelope of every document; explicit MA-service binding; the legal noncombat reports of a
bound Agent (navigation, command capabilities, sensors, own tracks, passive contacts, link tracks, peers,
designations, activities, nets, authority, C2 tasks, overrides, its own sends and command statuses); the noncombat requests (flight, sensing,
designation, P2P messages, MA task decisions, typed C2 commands); receipts; the referee's outcome and truth
documents; configuration, capacity limits, errors, lifetime and seed streams.

Not in version 1: strike and jettison requests, countermeasure inventory and dispense, launch-zone queries, stores
and strike status reports, release consent requests and answers, strike policy and the `training.authorization.v1`
profile, weapon and engagement truth, and reward definitions. Their documents are frozen separately in the
[engagement extension contract](ma-engagement-contract.md); their runtime is not installed, so each stays refused
explicitly. They enter through the extension seam of [section 15](#15-extension-seam-and-pending-gates) without
changing a version 1 field.

## 2. Identities

Each identity is a separate field with one owner. None is derived from another, and none is an entity's truth
identifier unless the table says so.

| Field | Width | Assigned by | Scope and meaning |
| --- | --- | --- | --- |
| `env_id` | 64 | configuration | One environment slot of a batch; stable for the slot's life. Part of every seed |
| `local_episode_index` | 64 | `reset` caller | Which episode of that slot; with `base_seed` and `env_id` it fixes every random stream |
| `episode_generation` | 64 | the episode | Counts resets of one `MissionEpisode` object from 1. Lifetime only: never seeds anything, and is the one field normalised when a reset is compared with a fresh construction |
| `service_id` (`ma_service_id`) | 64 | configuration | One MA service: one SDK Agent, one controller, one endpoint. Nonzero, unique in a configuration |
| `controller_id` | 32 | range (minted) | The service's controller, from `0x40000001` (`World::createAgent`). Never named by a document that comes in |
| `platform_id` | 32 | range | The bound vehicle's or C2 node's entity id: the service's own platform only (`Agent::vehicle`) |
| `system_id` | 32 | range | A platform's or C2 node's DMS SystemID (from `0x53000001` unless assigned). How every other system is named |
| `component_id` | 64 | upper layer | An upper-layer component behind one service. Echoed, never authenticated |
| `request_id` | 64 | upper layer | One request of one service in one generation. Nonzero, strictly increasing |
| `activity_id` | 64 | native | A VI activity (vehicle in the high 32 bits) or an MS activity (bit 31 of the low half set). Own platform's only |
| `message_id` | 64 | native | A DMS send, in its sender's numbering |
| `command_id`, `task_id` | 64 | native | A C2 command or task, in its sending system's MA core's numbering |
| `target_id` | 32 | native | A designation of the service's own platform, from `0x80000001`. The only way a request names another aircraft |
| `track_id`, `contact_id` | 64 | native | A sensor's own numbering; a link track's is its source radar's |
| `net_id`, `role_id`, `package_id`, `payload`, `topic` | 32 or 16 | range or native | As the existing SDK numbers them |

`system_id` in the design's identity list is this SystemID. A platform, an MA service, a component and a learned
actor are four different things: one platform may carry several services, one service may front any number of
components, a C2 node's service has no aircraft, and no count implies another.

## 3. Principals and proposed entry points

Every entry point of this contract belongs to one principal of [ADR-31](../agent-boundary.md).

Implemented, exported and classified in `boundary.toml`:

| Entry point | Principal | Package |
| --- | --- | --- |
| `fsim::bindMAService` (by binding, and by `Agent`), `fsim::maServiceBindings`, `fsim::maCapabilityOwner` (`fsim/MAService.h`) | range | M1 |
| `fsim_world_bind_ma_service`, `fsim_world_bind_ma_service_agent`, `fsim_world_ma_service_count`, `fsim_world_ma_service`, `fsim_world_ma_capability_owner` (`fsim/fsim_c.h`; `fsim_ma_service_binding_init` is a value initialiser) | range | M1 |
| `fsim.World.bind_ma_service`, `ma_services`, `ma_capability_owner` (`fsim.MAService` is their record) | range | M1 |
| `fsim::Agent::contacts`, `reportNavigation`, `reportTracks`, `reportContacts`, `reportLinkTracks`, `reportPeers`, `reportPeerHeartbeats`, `reportTasks`, `reportAuthority` (`fsim/Agent.h`; their records in `fsim/MAReports.h`) | agent | noncombat M2 |
| `fsim_agent_report`, `fsim_agent_operator_callsign` (`fsim/fsim_c.h`; `fsim_report_meta_init` and the three `fsim_report_*_name` functions are values) | agent | noncombat M2 |
| `fsim.Agent.contacts`, `report_navigation`, `report_tracks`, `report_contacts`, `report_link_tracks`, `report_peers`, `report_peer_heartbeats`, `report_tasks`, `report_authority`, `operator_callsigns` (`fsim.Report`, `ReportCollection`, `ReportItem`, `ReportMeta` are their records) | agent | noncombat M2 |

Proposed. These rows become `boundary.toml` rules, by exact name, in the package that exports them; `sdk_boundary`
rejects a rule for a name the SDK does not have, so they are not in that file yet:

| Proposed entry point | Principal | Package |
| --- | --- | --- |
| `fsim::MissionEpisode`: constructor from `std::string_view`, destructor, move construction and assignment, `reset`, `step`, `endpoint`, `referee` (`fsim/MissionEpisode.h`) | range | M4, M5 core |
| `fsim::MissionEndpoint`: destructor, moves, `observationJsonV1`, `submitJsonV1`, `receiptsJsonV1` | agent | M5 core |
| `fsim::MissionReferee`: destructor, moves, `outcomeJsonV1`, `truthJsonV1` | referee | M5 core |
| `fsim_mission_episode_create`, `_destroy`, `_reset`, `_step` (`fsim/fsim_mission_env.h`) | range | M5 core |
| `fsim_mission_endpoint_open`, `fsim_mission_referee_open` | range | M5 core |
| `fsim_mission_endpoint_destroy`, `_submit`, `_observation_json`, `_receipts_json` | agent | M5 core |
| `fsim_mission_referee_destroy`, `_outcome_json`, `_truth_json` | referee | M5 core |
| `fsim.mission.MissionEpisode`: `reset`, `step`, `endpoint`, `referee`, `close` (`python/fsim/mission.py`) | range | M5 core |
| `fsim.mission.MissionEndpoint`: `observation_json`, `submit_json`, `receipts_json`, `close` | agent | M5 core |
| `fsim.mission.MissionReferee`: `outcome_json`, `truth_json`, `close` | referee | M5 core |
| `fsim.isolation.MissionEndpointProcess` (the range's side of a process boundary) | range | M5 core |
| `fsim.isolation.RemoteMissionEndpoint`: `observation_json`, `submit_json`, `receipts_json` (the actor's side) | agent | M5 core |

`receiptsJsonV1` is one addition to the plan's C++ sketch: the same non-consuming read the C function gives. The
batch family of M7 (`fsim/MissionBatch.h`) is not frozen here; it consumes these documents unchanged.

The frozen signatures. Each C++ class is exported, holds one pointer to its implementation and nothing else, and has
no inline member that depends on that implementation's layout:

    // fsim/MissionEpisode.h (proposed)
    class MissionEpisode {
    public:
        explicit MissionEpisode(std::string_view configJsonV1);
        ~MissionEpisode();
        MissionEpisode(MissionEpisode&&) noexcept;
        MissionEpisode& operator=(MissionEpisode&&) noexcept;
        MissionEpisode(const MissionEpisode&) = delete;
        MissionEpisode& operator=(const MissionEpisode&) = delete;
        std::string reset(std::uint64_t localEpisodeIndex);   // the new generation's episode key
        void step();
        MissionEndpoint endpoint(std::uint64_t serviceId);
        MissionReferee referee();
    };
    class MissionEndpoint {                                    // movable, not copyable, as above
    public:
        std::string observationJsonV1() const;
        std::string submitJsonV1(std::string_view actionsJsonV1);
        std::string receiptsJsonV1() const;
    };
    class MissionReferee {                                     // movable, not copyable, as above
    public:
        std::string outcomeJsonV1() const;
        std::string truthJsonV1() const;
    };

    /* fsim/fsim_mission_env.h (proposed): int results are enum fsim_status values of fsim_c.h */
    int  fsim_mission_episode_create(const char* config_json, size_t config_bytes, fsim_mission_episode** episode);
    void fsim_mission_episode_destroy(fsim_mission_episode* episode);
    int  fsim_mission_episode_reset(fsim_mission_episode* episode, uint64_t local_episode_index);
    int  fsim_mission_episode_step(fsim_mission_episode* episode);
    int  fsim_mission_endpoint_open(fsim_mission_episode* episode, uint64_t service_id, fsim_mission_endpoint** endpoint);
    int  fsim_mission_referee_open(fsim_mission_episode* episode, fsim_mission_referee** referee);
    void fsim_mission_endpoint_destroy(fsim_mission_endpoint* endpoint);
    void fsim_mission_referee_destroy(fsim_mission_referee* referee);
    int  fsim_mission_endpoint_submit(fsim_mission_endpoint* endpoint, const char* actions_json, size_t actions_bytes);
    int  fsim_mission_endpoint_observation_json(fsim_mission_endpoint* endpoint, char* buffer, size_t capacity, size_t* needed);
    int  fsim_mission_endpoint_receipts_json(fsim_mission_endpoint* endpoint, char* buffer, size_t capacity, size_t* needed);
    int  fsim_mission_referee_outcome_json(fsim_mission_referee* referee, char* buffer, size_t capacity, size_t* needed);
    int  fsim_mission_referee_truth_json(fsim_mission_referee* referee, char* buffer, size_t capacity, size_t* needed);

The three C types are separate opaque structs; none is an `fsim_world`. A document passed in is a pointer and a
byte count, not NUL-terminated. A document read reports in `*needed` its bytes including the NUL terminator: with a
null `buffer` or `capacity` 0 that is a successful size query (`FSIM_OK`); with a capacity below `*needed` it returns
`FSIM_INVALID_ARGUMENT` and writes nothing; otherwise it writes the whole document. No read consumes a message,
advances a cursor or submits anything, however often and in whatever order it is called. `submit` stores its
acknowledgment for `_receipts_json`.

In Python each `*_json` method returns `str` and `submit_json` takes `str` or `bytes`; the text is the contract, so
identities stay exact whatever a caller parses it with.

Range obtains endpoints and referees and hands each actor its endpoint alone. No endpoint member returns, reaches or
accepts a World, a Vehicle, a Referee or another service's endpoint. A referee document never enters an observation,
a receipt, a mask or an actor's infos.

## 4. Ownership, generation, lifetime and move

A `MissionEpisode` owns exactly one World at a time, and with it every mutable thing of the episode: Host, guidance,
damage, DMS, MA core, task state, report and message cursors, receipts and random streams. Immutable assets may be
shared between episodes; nothing mutable is.

- **Construction** parses and validates `ma.config.v1` and builds no World: generation 0. `endpoint` and `referee`
  fail until the first reset.
- **`reset(local_episode_index)`** destroys the owned World, if any, and constructs a fresh one from the
  configuration's recipe: scenario, C2 nodes, systems, nets, dissemination, RBAC, each array in its own order, then
  one Agent per service in ascending `service_id` and its binding. The generation becomes the previous one plus 1, the tick 0, and snapshot 0
  is frozen. It never calls the legacy vehicle reset in place of reconstruction; `Vehicle.reset` and
  `fsim_world_reset_vehicle` keep their existing semantics for existing callers. C++ returns the schema's
  `episode_key` object; C reports it in the first observation.
- **Handles are bound to a generation.** An endpoint or referee opened in generation g answers only while the
  episode is in generation g and alive. After a reset every earlier handle is stale: each call fails as below and
  touches nothing. The new generation's handles are opened again by the range.
- **A control block outlives the World.** Parent and children share a small block holding the closed flag and the
  current generation. A child checks it before it touches the episode; destroying the parent sets closed first, then
  releases the World, so a child that is still held fails cleanly and never dereferences freed state. The block is
  freed with its last holder.
- **Move.** All three classes are movable and not copyable; destructors and noexcept move operations are exported.
  Moving a parent keeps its identity: its children stay valid and keep working. A moved-from object is closed.
- **References die with their generation.** A request, activity, target, task, message or command id of an earlier
  generation means nothing in a later one; documents carry the episode key so that they are refused, not re-read.
  Process endpoints of an earlier generation are closed, or re-bound explicitly by the range.
- **One boundary.** `reset`, `step` and every endpoint or referee call of one episode are serialised at the episode.
  A call that arrives while another holds the boundary fails as busy; it never waits and never races the World.

**Implemented internally (M4).** `fsim::env::MissionEpisode` is the native owner the public classes above will wrap
(M5 core). It takes a typed `EpisodeConfig` - the section 7 recipe in native form, sections in the table's order, plus
a range-side `native` recipe step for setup the document has no field for - and implements this section as written:

- Construction validates and builds no World (generation 0). `reset(local_episode_index)` advances the generation
  first, so every earlier child is stale, then destroys the judge, the Agents and the World - its Mission Systems,
  guidance, damage, DMS, MA core, publication and recording with it - and builds a new World with the episode's world
  seed, applies the recipe, mints one Agent per service in ascending `service_id` (named `episode-service-<id>`) and
  binds it, and makes a new task/judge from the configuration's factory. The decision tick returns to 0. A recipe that
  fails leaves the episode with no World until a later reset succeeds. The legacy vehicle reset is neither called nor
  changed.
- `step()` runs `decision_steps` World steps, then the judge, and advances the tick.
- Children are `EpisodeService` (a service's World and Agent identity, for M5's endpoint) and `EpisodeReferee` (the
  World read-only, for M5's referee documents). They share the control block: `closed`, the current generation and
  the boundary flag. A child answers `ok` only in its own generation; otherwise `stale`, `closed`, `not_reset` or
  `unknown_service`, and its callback is never called. A call that finds the boundary held answers `busy` at once.
- Destroying the episode sets `closed` under the boundary, then releases the World; moving it keeps the control block
  and its state, so children stay valid; a moved-from episode is closed.
- A diagnostic episode's World is named `<name>-e<env_id>-g<generation>` and records to that name's `.fsrec` in its
  record directory, so neither publication nor recording is ever shared between episodes or generations.

The `mission_episode` gate applies the supplied [native-state fixture](native-state-reset-fixture.md) to this reset:
after crew loss, fire, an in-flight weapon, a decoy, a consumed store, pending C2 task and consent, a queued message
and a removed entity, a reset to the same local episode equals a fresh episode of the same inputs and an independent
World of the same options, seed and recipe - field for field, with nothing normalised (the episode generation is not
part of the native state) - immediately, after 1, 4 and 30 more steps, and after the whole contamination is replayed.

## 5. Errors

Failures of the API and of lifetime are errors of the call. Refusals of a well-formed request are documents.

| Condition | C++ | C (`enum fsim_status`, `fsim_last_error()` set) | Python | Effect |
| --- | --- | --- | --- | --- |
| A malformed document ([section 6](#6-documents)): not UTF-8, not strict JSON, schema-invalid, an unknown field, a non-finite number, over `max_document_bytes`, base64 that is not canonical, a `byte_length` that is not the decoded length, request ids that do not strictly increase, more requests than `max_requests_per_document` | `std::invalid_argument` | `FSIM_INVALID_ARGUMENT` | `fsim.Error` | nothing queued, nothing changed |
| Null handle or output pointer; buffer too small for a document | n/a | `FSIM_INVALID_ARGUMENT`, no partial document | n/a | nothing consumed |
| Episode not yet reset; handle closed, moved-from or of an earlier generation | `std::runtime_error` | `FSIM_INVALID_ARGUMENT` | `fsim.Error` | nothing |
| The episode's boundary is held by another call | `std::runtime_error` | `FSIM_ERROR` | `fsim.Error` | nothing; retry is the caller's |
| A live handle of another principal's kind | not expressible | `FSIM_NOT_ALLOWED` | not expressible | nothing |
| A service the configuration does not bind (`endpoint`) | `std::invalid_argument` | `FSIM_INVALID_ARGUMENT` | `fsim.Error` | nothing |
| A well-formed document for another generation, tick or service | returns `ma.receipts.v1`, `submission.accepted` false | `FSIM_OK`; the same document through `_receipts_json` | returns it | nothing queued |
| A well-formed request the gateway or the native SDK refuses, among them a message whose decoded bytes exceed `max_message_bytes` (`message_too_large`) | a receipt, `state` `rejected` | the same | the same | as the receipt says; the document's other requests are queued |

`destroy` and `close` accept null and an already closed object. A destroyed C handle must not be used again;
separately live children stay safe after their parent is destroyed. No new status value is added to `fsim_status`.

## 6. Documents

Six documents, each a UTF-8 JSON object with a `schema` member: `ma.config.v1`, `ma.observation.v1`,
`ma.actions.v1`, `ma.receipts.v1`, `ma.outcome.v1` and `ma.truth.v1`. The [schema](ma-contract.schema.json) is JSON
Schema 2020-12, restricted to the keywords [check.py](../../tools/ma_contract/check.py) lists; it documents the
contract and drives the repository check. The SDK does not load it and gains no dependency: adapters validate with
code written to this page.

- **Strict JSON.** RFC 8259 text, UTF-8, no byte order mark, no comments, no trailing commas, no duplicate member
  names, no `NaN` or `Infinity`, no number that overflows a double. The existing native parser (`fsim::core::Json`)
  is lenient by design - it takes comments, trailing commas, numbers such as `01` or `1.`, overflow to infinity and
  repeated names, and checks no UTF-8 - so it cannot validate a mission document as it is. Its value type may hold a
  parsed document; a strict reading is new internal work of M5, not a new dependency. The scenario embedded in a
  configuration is read strictly too.
- **Identities are strings.** Every identity or correlation, of any width, and every native 64-bit integer is an
  unsigned decimal string without sign, space or leading zero, bounded by its width. The native JSON value holds
  numbers as doubles, which cannot carry an id above 2^53. Quantities of 32 bits or fewer (counts, masks, codes,
  priorities) are JSON integers.
- **Unknown is null.** A reported number the bound Agent does not know - a native NaN, a value not measured, a
  report not held - is `null`, and its record's `meta.validity` or `availability` says which. It is never NaN, never
  zero and never filled from truth. A native infinite duration is `null` with its `*_unbounded` flag true. A request
  field left out keeps the native hold or default; `null` is accepted in a request only where the schema says so and
  means the same as leaving it out.
- **Bytes are bounded base64.** RFC 4648 section 4: standard alphabet, padded, no line breaks, canonical. Each carries
  `byte_length`, which must equal the decoded length. A document that breaks one of those rules, or whose text
  decodes to more than the schema's 65,536 bytes, is malformed and refused whole. Size against a limit is a
  different matter and never makes a document malformed: a `dms.send` whose decoded bytes exceed the configured
  `limits.max_message_bytes` is that one request's refusal (`message_too_large`, a rejected receipt), and a message
  within it that a net's own `max_payload_bytes` cannot carry is the native send's refusal (`too_large`). A
  delivered message in a snapshot is as large as its net let it be; `max_message_bytes` does not apply to it.
- **Closed objects.** A document that comes in (`ma.config.v1`, `ma.actions.v1`) with a member this version does not
  define is refused whole. A document that goes out carries only version 1 members; additions arrive as a new
  version or under `extensions`, whose unknown keys a consumer ignores.
- **Units.** SI, radians, simulation seconds, WGS-84 geodetic coordinates, as the C++ Agent API. The one exception
  is the embedded scenario, which keeps its own documented format and units.
- **Size.** A document is at most `limits.max_document_bytes` as UTF-8, and a document the SDK writes is never
  truncated: a C read reports the bytes needed.
- **Order.** Arrays are in a stated stable order: receipts and requests by `request_id`, collections in the native
  call's order, services by `service_id`.

## 7. `ma.config.v1`: configuration

Given once to the episode's constructor. Every field is explicit; the only defaults are the ones listed.

| Field | Required | Meaning and native mapping |
| --- | --- | --- |
| `env_id`, `base_seed` | yes | Seed inputs ([section 8](#8-seed-streams)) |
| `timing.dt_s`, `timing.frame_skip` | yes | `WorldOptions::dt` and `frameSkip` |
| `timing.decision_steps` | yes | `World::step()` calls per `MissionEpisode::step()`; one decision tick lasts `dt_s * frame_skip * decision_steps` seconds |
| `scenario` | yes | An existing [scenario document](scenarios.md) applied by `applyScenario`. Its `world` section may hold only `name`, `capacity`, `jsbsim_root` and the `terrain*` keys; `vecenv` is refused. A vehicle with `count` above 1 gives platforms `<name>-1` .. `<name>-N` |
| `c2_nodes` | no | `World::createC2Node` each, in order |
| `systems` | no | `World::assignSystem` (SystemID, tail number); otherwise the next free id when the platform first joins a net |
| `nets` | no | `World::createNet` with `comm::NetSpec` (a field left out keeps its native default), then `joinNet` for each member in order |
| `dissemination` | no | `World::setDissemination` (`comm::DisseminationPlan`) |
| `rbac` | no | `World::setRbac` roles; a platform not listed keeps the native A-GRA defaults |
| `services` | yes | One [binding](#9-service-binding) each |
| `reports.stale_after_s` | no | A held report whose `age_s` is above it is marked `stale` ([section 10](#10-maobservationv1-the-actors-snapshot)). Left out: nothing is marked stale |
| `limits` | no | [Section 14](#14-capacity-limits) |
| `task.criteria` | no | [Section 13](#13-referee-documents) |
| `execution.workers` | no | World worker threads. Default 1 |
| `execution.diagnostic`, `publish`, `record_dir` | no | Default: not diagnostic, not published, not recorded. `publish` true or a `record_dir` requires `diagnostic` true |
| `extensions` | no | [Section 15](#15-extension-seam-and-pending-gates). A key no installed definition registered fails construction |

A platform is named by its scenario vehicle's or C2 node's name; names are unique across both. The recipe is
applied section by section in the table's order. Two kinds of order must not be confused:

- **Array order is native creation order.** Within `scenario.vehicles`, `c2_nodes` and `nets`, entries are created
  in the order the array gives, and the native ids follow it: a vehicle's entity id, a C2 node's entity id (from
  `0x60000001`), a net's id (from 1), and a SystemID the range does not assign (the next free one when the platform
  first joins a net, so the order of `nets` and of each net's `members` matters too). Reordering one of these arrays
  is a different configuration with different native ids. Documents name platforms by name and systems by the
  SystemID a snapshot reports, never by an assumed position.
- **Service order decides nothing.** The order of `services`, the order in which their Agents happen to be created
  and the order in which the range later opens endpoints never determine who owns a capability, the DMS ingress or
  the DMS egress: the binding does, by `service_id`. The episode creates the Agents in ascending `service_id`, which
  fixes each minted `controller_id`; reorder the array and every owner, grant and controller is the same.

No field presets a package: a team exists natively only through a C2 system's `c2.form_package` message, authorised
by its role, so a configured C2 service forms it at run time. No field declares a role for a C2 service either: it
sends `c2.declare_role` itself. The platform's strike policy is not generic configuration.

A diagnostic episode is labelled non-isolated and is refused by strict isolation. Its published world is named
`<scenario.world.name>-e<env_id>-g<episode_generation>` and its recording is
`<record_dir>/<scenario.world.name>-e<env_id>-g<episode_generation>.fsrec`, so two episodes, or two generations of
one, never share a segment or a file. A batch's worker budget is the batch's own explicit setting (M7), not a sum of
these.

## 8. Seed streams

Random streams depend on `base_seed`, `env_id`, `local_episode_index` and a stable stream, service or component id,
and on nothing else: not the generation, not another environment's resets, not a worker schedule, not the order in
which services or components were registered. With `next(x)` the existing `fsim::Rng(x).next()` (SplitMix64) and
all arithmetic modulo 2^64:

    stream(class)                 = next(next(next(next(base_seed) ^ env_id) ^ local_episode_index) ^ class)
    world seed                    = stream(0)                  -> WorldOptions::seed
    service seed(service_id)      = next(stream(1) ^ service_id)
    component seed(component_id)  = next(service seed(service_id) ^ component_id)

Classes 2 to 15 are reserved for later SDK streams. The service and component seeds are offered to the upper layer
(the composition example uses them); the SDK itself consumes the world seed. The fixtures' manifest carries test
vectors, and `next(0)` is `16294208416658607535`.

Implemented by `fsim::env::episode_seeds` (M4) and checked against the manifest's vectors; an episode's
`worldSeed()`, `serviceSeed(id)` and `componentSeed(id, component)` are these streams for its current local episode.

## 9. Service binding

Implemented by M1. `MAServiceBindingV1` ([fsim/MAService.h](../../include/fsim/MAService.h)) is the native form of
one `services` entry; `fsim_ma_service_binding` and `fsim.World.bind_ma_service` are the same binding in C and
Python:

    struct MAServiceBindingV1 {
        std::uint64_t serviceId;
        std::uint32_t platform;                         // the vehicle's or C2 node's entity id
        control::ControllerId controller;               // minted by World::createAgent for this service
        std::vector<std::string> commandCapabilities;
        bool ownsDmsIngress;
        bool ownsDmsEgress;
    };
    bool bindMAService(World&, const MAServiceBindingV1&, std::string* error);
    bool bindMAService(World&, const Agent&, std::uint64_t serviceId, const std::vector<std::string>& commandCapabilities,
                       bool ownsDmsIngress, bool ownsDmsEgress, std::string* error);
    std::vector<MAServiceBindingV1> maServiceBindings(World&);                        // ascending service id
    control::ControllerId maCapabilityOwner(World&, std::uint32_t platform, const std::string& capability);

`bindMAService` is the range's. It accepts only a controller the World minted for an Agent of that platform, creates
no controller identity, and refuses (false, with the reason; nothing changed) a binding that overlaps one already
made, a platform that is gone, and a capability the platform does not offer for command. The two read calls were
added with the implementation; an Agent's handle reaches none of the three.

Two Worlds mint the same entity ids and controller numbers, so numbers alone cannot say whose an Agent is. The forms
that take the Agent itself - `bindMAService(world, agent, ...)`, `fsim_world_bind_ma_service_agent(world, agent_handle,
&binding)` and Python's `world.bind_ma_service(agent, ...)`, which has no other - read the platform and the controller
from the Agent and refuse one another World made, and anything that is not an Agent, with nothing changed. The forms
that take numbers bind the controller of that number in the World they are called on.

What the binding does natively, using only mechanisms the SDK already had:

- **Grants carry ownership.** A bound platform is under the existing `ControlMode::Granted`, in which only a grant's
  holder commands a capability, and stays there: setting another mode is refused `not_allowed` while it is bound.
  Each command capability's grant is its bound owner's or no one's, and an unowned capability's is no one's. Only the
  owner is granted a capability: `requestControl` by any other controller - another Agent, the default policy
  (controller 0), a number of the range's choosing - is `not_allowed`, so another service's NEW is refused
  `not_granted` and its UPDATE or CANCEL of an activity `authority_held`, all existing reasons. A range script there
  commands as the platform's own source, which needs no grant, or binds itself a service.
- **What stood before a binding gives way to it.** Each binding asks at once for the grants of what it names, and
  first takes back whatever is not the bindings': for every command capability of the platform, flight and Mission
  Systems alike, a grant held by a controller that is not its bound owner, or a live or waiting policy activity of
  such a controller, is revoked by the platform's existing act - the grant ends and that capability's policy
  activities end `canceled`, reason `revoked`. That covers another Agent's grant, the default policy's, a grant on a
  capability no service owns, and what an open platform's policies flew without any grant. The revoke is of the
  capability, not of a controller: it ends every policy activity of that capability, the newly bound owner's own
  among them. The owner's activities therefore go on under its grant only on a capability no other controller held a
  grant of or had a live or waiting policy activity on. Where one did - the owner flying an HSA on an open platform
  while another controller has an HSA scheduled, say - a successful binding ends both controllers' activities of that
  capability `canceled`, reason `revoked`, and the owner, now granted, commands anew. The platform's own sources are
  not policy and are not touched. A capability
  the platform does not allow or cannot give when it is bound - one flown on the ground only, say - stays ungranted
  and is its owner's to request later. So a successful binding always leaves ownership, grants and activities
  agreeing; a refused one changes none of them. Bind every service of a platform before any of them acts, and no
  activity is ever at stake: services are bound one at a time, so a service flying before its own binding is made is
  ended by an earlier binding of the same platform and granted again by its own, and an owner sharing a capability
  with another controller loses its activity to the revoke that removes the other's.
- **C2 reaches the owners.** A control designation's grants go to each allowed capability's bound owner, a C2
  override is applied to each bound service's own live HSA, and a lost-communication plan is activated as the owner
  of the platform's waypoint-following capability (with no such owner, the alert alone).
- **Each DMS side has one owner.** The inbox and subscriptions answer the ingress owner alone: any other Agent of
  the platform reads an empty inbox and may not take or subscribe. Sends of every kind answer the egress owner alone:
  another's is refused with the new `comm::DmsReason::NotOwner` (`FSIM_DMS_NOT_OWNER`, 5), and its task answer
  returns false. The range's own handle is no service and is not restricted.
- **Lifetime.** Removing a platform ends its bindings; a platform made later binds afresh, with new controllers. The
  legacy vehicle reset keeps bindings, control mode and ownership, as it keeps the rest of the authority state.

The native bind accepts at most one ingress and one egress owner per platform. That each bound platform has exactly
one of each is the configuration's rule ([section 7](#7-maconfigv1-configuration)), checked by the episode. The
gateway reasons `capability_not_bound`, `dms_ingress_not_owned` and `dms_egress_not_owned` of
[section 12](#12-mareceiptsv1-acknowledgments-and-receipts) are the gateway's own check before dispatch (M5); the
native refusals above stand behind them.

| Rule | Contract |
| --- | --- |
| Identity | One service is one Agent and one controller on one platform. The configuration names no controller; the episode mints it |
| Command capabilities | Each entry is a discovery id exactly as the platform reports it - a VI capability id (`fsim.guidance.hsa`) or an MS capability instance id (`fsim.ms.amti:radar`) - or the single entry `*`: every command capability of the platform. A capability belongs to at most one service; `*` on a platform means no other service there holds any |
| Flight and sensor requests | Dispatched only for a capability bound to the requesting service; otherwise `capability_not_bound` |
| C2 effects on the platform | A C2 control designation's grants, a C2 override applied to a live HSA and a lost-communication plan activation go to the configured owners of the capabilities concerned. Today's legacy path gives all three to the platform's most recently created Agent. A C2 direct command flies under C2's own controller in both paths |
| DMS ingress | Exactly one service per bound platform. The gateway takes the platform's inbox once per tick for it; `messages` and subscription requests are its alone (`dms_ingress_not_owned` for the others) |
| DMS egress | Exactly one service per bound platform. Free-form sends, native publications, MA task answers and every C2 message leave through it alone (`dms_egress_not_owned` for the others) |
| Shared reports | Every service bound to a platform reads that platform's reports alike. Sharing a report grants no command capability |
| Activities | An activity belongs to the service whose request made it. Only that service's `update`, `cancel` or `gateway.close` addresses it; anything else gets the existing native refusal |
| Duplicates | Two services with one id or name, an unknown platform, a second ingress or egress owner, or a capability given twice fail construction |

Internal components behind one service are one SDK principal and need no SDK identity. A common upper-layer wrapper
may share what one platform's services legally hold; it may not carry one platform's or C2 node's view to another
except through the modelled links.

The legacy path is unchanged: a World without bindings behaves exactly as today, including the documented limitation
that a C2 designation, a live override and a lost-communication plan go to the platform's most recently created
Agent. Mission episodes always bind explicitly and never rely on it.

## 10. `ma.observation.v1`: the actor's snapshot

One immutable snapshot per service per decision tick, frozen when the tick opens. Reading it any number of times,
from any language or process, returns the same bytes and consumes nothing. It holds only what the service's bound
Agent can legally read; the table names the source of each part. No referee call, no `VehicleState`, no
`TaskState.custom`, no reward and no other platform's Agent contributes to it.

| Part | Native source (C++; the C and Python forms are the existing equivalents) | Notes |
| --- | --- | --- |
| `episode`, `tick`, `time_s` | episode state; `Agent::time` | `time_s` is the snapshot's simulation time |
| `service` | binding; `Agent::vehicle`, `controller`, `system`, `valid` | `system_id` null until the range assigned the platform a SystemID or put it on a net (native 0); `alive` false once the platform is gone |
| `reports.navigation.own_ship` | `Agent::ownship` (`fsim_vehicle_ownship`, `agent.ownship`) | As navigation and air data report it. `meta.validity` is `valid` only when `gnss_valid` and `air_data_valid`; values are passed as reported. `not_applicable` and null for a C2 node |
| `reports.navigation.endurance` | `Agent::navigationReport` | Native NaN to null; infinite to null with `*_unbounded` |
| `reports.capabilities` | `Agent::capabilities`, `capabilityStatus`, `controlStatus`; `missionCapabilities`, `missionCapabilityStatus` | Each command capability by its discovery id, its availability and reason as native names, whether the policy is allowed it and holds its grant, and `owner_service_id` from the binding. What a C2 designation revoked shows here |
| `reports.sensors` | `Agent::payloads`, `radarStatus` | `radar` null for a payload that is not an air radar |
| `reports.tracks` | `Agent::amtiTracks` for each air radar | The entity behind a track is never present |
| `reports.contacts` | `Agent::contacts` (`fsim_vehicle_contacts`, `agent.contacts`) for each payload | `first_time_s` is when it was first held, the sample time when it was last seen. The entity behind a contact is never present |
| `reports.link_tracks` | `Agent::linkTracks` | Present only after modelled delivery; dropped when the native picture drops them |
| `reports.peers` | `Agent::peers` | `position` null until a position report arrived; `tail_number` null until a heartbeat did. The item's `net_id` is the net the peer was last heard by, whichever message that was. Its `meta` is one message's alone: the position report it holds - that report's net, delivery time and sample time - or, for a peer heard only by its heartbeat, that heartbeat's net and delivery time with a null sample time. A later heartbeat by another net does not relabel a held position |
| `reports.designations` | `Agent::designations` | `source_system_id` null for its own radar's track |
| `reports.activities` | `Agent::activities`, `missionActivities` | `owner_service_id` from the gateway's own records; null for C2's and the platform's own |
| `reports.nets` | `Agent::nets` | |
| `reports.transmissions` | `Agent::transmission` for each open send receipt | The egress owner's; `not_owner` otherwise |
| `reports.authority` | `Agent::authority` | Native 0 system to null; `package` null with no team |
| `reports.tasks` | `Agent::c2Tasks` | Native NaN order fields to null. The MA core keeps the last 256 received tasks |
| `reports.overrides` | `Agent::overrides` | `expiry_s` null: until canceled |
| `reports.command_statuses` | `Agent::commandStatuses` | What MAs answered to this system's C2 messages: one last status per answering system, command id and topic, the last 256 kept. A role declaration's answer has command id 0 |
| `messages` | `Agent::receive`, taken once per tick by the gateway | The ingress owner's; `not_owner` otherwise. Each message appears in exactly one snapshot |
| `receipts` | the gateway's records for this service | [Section 12](#12-mareceiptsv1-acknowledgments-and-receipts) |

The episode builds the snapshot natively, from what the C++ Agent of the service reads. Where today's C or Python
Agent wrappers expose less than that Agent does - an operator's callsign, a net's payload limit, whether the
platform is alive - the document is complete all the same; those wrappers are not its source.

**Availability** of a part is `available`; `unavailable` (the SDK cannot give it now - the platform is gone, or the
call is not delivered yet); `not_applicable` (the platform has no such thing - a C2 node's navigation); or
`not_owner` (another service of the platform holds it). A part that is not `available` lists no items.

**`meta`** is on every item that has a source of its own, and on the singular reports:

| Field | Meaning |
| --- | --- |
| `source` | `own_navigation`, `own_sensor`, `own_platform`, `ma_core`, `dms` or `link`; with `system_id`, `net_id` or `payload` where the native report names them |
| `sample_time_s` | When the source measured or stated it: a track's last update, a link track's or peer's report time, a status's own time. Null when the source states none (a received task, a peer heard only by its heartbeat) |
| `receive_time_s` | When it reached this platform: a link's delivery time; for an own report, the snapshot's time |
| `age_s` | `time_s` less `sample_time_s`; null exactly when the sample time is |
| `validity` | `valid` (held now, and its source reports it good); `stale` (held now, with an `age_s` above the configured `reports.stale_after_s`); `invalid` (its own source flags it: navigation with a sensor invalid); `unknown` (the report holds nothing: never received, or not applicable) |

A report is in the snapshot exactly while the native picture holds it: the native dissemination plan drops a peer
after its `peer_timeout_s` and a link track after its `track_timeout_s`, and a dropped report is gone, not marked.
`stale` is this contract's own, explicit and configured mark on a report that is still held; without
`reports.stale_after_s` nothing is marked, and `age_s` says the same thing in seconds. A designation carries its
native `valid` flag instead. Nothing ages a report but simulation time, and nothing refreshes one but a modelled
sensor update or a delivered message.

**Messages** are ingested once: at each tick's freeze the gateway takes what waits in the ingress owner's system
inbox, and those messages are that snapshot's `messages`, oldest first. They are the inbox as it is: free-form
topics, and the native-typed messages the DMS also places there (topics 1 to 3 and 50 to 52), each as its native
bytes. Version 1 decodes none of them; a consent request's bytes (topic 52) are extension content. The legacy
`Agent::receive` stays a consuming call for legacy code and is not used on a bound platform by anything but the
gateway. The native inbox holds 1,024 messages and drops its oldest beyond that; an Agent is not told about a drop,
so neither is a snapshot.

**Masks.** Version 1 defines no SDK-computed action mask. A mask an adapter builds may depend only on the fields of
this document.

### Legal report calls (noncombat M2)

Seven of the parts above exist today as calls on an Agent, in all three languages. They are the values the Agent's
existing reads give, by the same paths and under the same rules, each with the `meta` of this section. Nothing else
feeds them: no referee call, no `VehicleState`, no other platform's Agent.

A `meta` describes one report and never mixes two. A peer is the one item made of two messages - a position report
and a heartbeat, which may arrive by different nets at different times - so it has two calls: one for the position
it holds, one for its last heartbeat. The native `comm::PeerReport` (`fsim_peer_report`, `fsim.PeerReport`) is
unchanged: its `net` and `receivedS` remain those of whichever message was heard last.

| Part | C++ (`fsim/Agent.h`) | C kind (`fsim_agent_report`) | Python (`fsim.Agent`) | `meta` |
| --- | --- | --- | --- | --- |
| `reports.navigation` | `reportNavigation` | `FSIM_REPORT_NAVIGATION` | `report_navigation` | `own_navigation`; sampled at the own ship's reported time; `invalid` while `gnss_valid` or `air_data_valid` is false |
| `reports.tracks` | `reportTracks` | `FSIM_REPORT_TRACKS` | `report_tracks` | `own_sensor` with the radar's `payload`; sampled at the track's last update |
| `reports.contacts` | `reportContacts` | `FSIM_REPORT_CONTACTS` | `report_contacts` | `own_sensor` with the `payload`; sampled when last seen |
| `reports.link_tracks` | `reportLinkTracks` | `FSIM_REPORT_LINK_TRACKS` | `report_link_tracks` | `link` with the source `system_id`, the `net_id` and the source's `payload`; sampled at the source's report time, received at delivery |
| `reports.peers` | `reportPeers` | `FSIM_REPORT_PEERS` | `report_peers` | The position report each peer holds: `link` with `system_id` and the `net_id` that report came by, sampled at its own time, received when it was delivered. For a peer heard only by its heartbeat: that heartbeat's net and delivery, the sample time null |
| `reports.peers` | `reportPeerHeartbeats` | `FSIM_REPORT_PEER_HEARTBEATS` | `report_peer_heartbeats` | Each peer's last heartbeat, beside the peers index for index: `link` with `system_id` and the `net_id` it came by, sampled at the time it states, received when it was delivered. `unknown`, its times null, for a peer no heartbeat was heard from |
| `reports.tasks` | `reportTasks` | `FSIM_REPORT_TASKS` | `report_tasks` | `link` with the sending `system_id`; received at delivery; the sample time null, so never `stale` |
| `reports.authority` | `reportAuthority` | `FSIM_REPORT_AUTHORITY` | `report_authority` | `ma_core`; sampled at the time read |

- **C++** returns the value with its metadata: `ma::Report<T>` (availability, `meta`, value) for the singular
  reports, `ma::Collection<T>` (availability, items of `meta` and value) for the others. An unknown time is NaN
  (`ma::kUnknown`), no payload `ma::kNoPayload`, no system or net 0.
- **C** returns the metadata, item for item beside the existing getters: `fsim_agent_report(world, entity, kind,
  stale_after_s, &availability, metas, max, &count)` fills up to `max` `fsim_report_meta`, `metas[0].struct_size`
  bytes apart, and `*count` with how many there are; `metas` NULL with `max` 0 gives the count alone. Item `i` is
  item `i` of `fsim_vehicle_amti_tracks` (each air radar in payload order), `fsim_vehicle_contacts` (each payload in
  order), `fsim_dms_link_track_get`, `fsim_dms_peer_get` or `fsim_agent_c2_task_get`; the singular kinds have one.
- **Python** returns `fsim.Report(availability, meta, value)` and `fsim.ReportCollection(availability, items)` of
  `fsim.ReportItem(meta, value)`; `fsim.ReportMeta` holds None where the contract has null.
- **Availability** is `available`, `unavailable` once the platform is gone, or `not_applicable` for what a C2 node
  does not have (navigation, tracks, contacts, tasks, authority). `not_owner` is not produced by these seven: every
  Agent of a platform reads them alike ([section 9](#9-service-binding)).
- **`stale`** takes the caller's threshold (`ma::ReportOptions::staleAfterS`, `stale_after_s`); the gateway will pass
  the configured `reports.stale_after_s`. With none (NaN, None) nothing is marked. A report with no sample time has no
  age and is never stale.
- **Reading consumes nothing** and changes nothing, however often a report is read, in any language.
- `Agent::contacts` and `fsim.Agent.contacts` are the Agent's read of `fsim_vehicle_contacts`, with `truth` 0 as
  for its detections and tracks. An operator's declared callsign is `C2Authority::Operator::callsign` in C++,
  `fsim_agent_operator_callsign` in C and `agent.operator_callsigns()` in Python.

The other parts of the table - capabilities, sensors, designations, activities, nets, transmissions, overrides,
command statuses, messages and receipts - have their existing native reads and no report call of their own; the
gateway assembles them, and the document, in M5.

## 11. `ma.actions.v1`: requests

`episode`, `service_id` and `tick` name the snapshot the requests answer; `requests` may be empty, which is how a
service waits. A request has a `request_id`, a `kind`, an `operation`, `parameters`, and optionally `component`
(the upper layer's provenance: echoed in the receipt, never read for authorization). The endpoint authenticates the
service: no parameter selects a sender, a source system, a controller or a platform, and a member that tries is an
unknown field.

Each operation maps to exactly one existing SDK call, made as the service's own Agent. A field left out is the native
hold or default. Targets are the service's own designations, never entity ids. Native validation, authority and
refusal reasons are the SDK's and are transported unchanged.

| Operation | Native call (C++ / C / Python) | Needs | Parameters, briefly |
| --- | --- | --- | --- |
| `flight.new` | `Agent::submit` with `HsaCommand`, `RouteCommand` + waypoints, or `BehaviorCommand` / `fsim_vehicle_submit_mode`, `_submit_route`, `_submit_behavior` / `submit_hsa`, `submit_route`, `submit_behavior` | capability | `mode` `hsa`, `route` or `behavior`; `CommandOptions::commandId` is the `request_id`; `interrupt` optional |
| `flight.update` | `Agent::update` / `fsim_activity_update*` / `AgentActivity.update`, `update_route` | owning service | `activity_id` and the command's fields; a behaviour's update carries `params` alone, never a target, and natively only parameters the behaviour declares updatable |
| `flight.cancel` | `Agent::cancel` / `fsim_activity_cancel` / `AgentActivity.cancel` | owning service | `activity_id` |
| `sensor.set_state` | `Agent::submit(ms::StateCommand)` / `fsim_vehicle_payload_state` / `set_payload_state` | capability | `payload`, `state` off, standby or operate |
| `sensor.search` | `Agent::submit(ms::AmtiCommand)` / `fsim_vehicle_amti_search` / `amti_search` (task volume), `amti_acquire` (cued), `amti_track` (track) | capability | `payload`, `mode`, optional volume, `task`, `track_id`; never `illuminate` |
| `sensor.cancel` | `Agent::cancelMission` / `fsim_ms_activity_cancel` / `cancel_mission` | owning service | `activity_id` |
| `sensor.designate` | `Agent::designate` / `fsim_vehicle_designate` / `designate` | capability of that radar | `payload`, `track_id`; the receipt carries `target_id` |
| `sensor.designate_link` | `Agent::designateLink` / `fsim_vehicle_designate_link` / `designate_link` | any bound service | `source_system_id`, `payload`, `track_id`; the receipt carries `target_id` |
| `sensor.undesignate` | `Agent::undesignate` / `fsim_vehicle_undesignate` / `undesignate` | any bound service | `target_id` |
| `dms.send` | `Agent::send` / `fsim_dms_send` / `send` | egress | `topic` at least 1000, `byte_length`, `data_b64`, optional `destination_system_ids` (none: published) |
| `dms.publish_native` | `Agent::publishPosition`, `publishStatus`, `publishTracks` / `fsim_dms_publish` / `publish_position`, `publish_status`, `publish_tracks` | egress | `report` position, status or tracks; `payload` for tracks |
| `dms.subscribe` | `Agent::subscribe` / `fsim_dms_subscribe` (on) / `subscribe` | ingress | `topic` |
| `dms.unsubscribe` | `Agent::unsubscribe` / `fsim_dms_subscribe` (off) / `unsubscribe` | ingress | `topic` |
| `dms.subscribe_all` | `Agent::subscribeAll` / `fsim_dms_subscribe_all` / `subscribe_all` | ingress | `enabled` |
| `task.accept` | `Agent::acceptTask` / `fsim_agent_answer_task` / `accept_task` | egress | `from_system_id`, `task_id` |
| `task.reject` | `Agent::rejectTask` / `fsim_agent_answer_task` / `reject_task` | egress | and `code`, the MA's own reason |
| `task.report` | `Agent::reportTask` / `fsim_agent_answer_task` / `report_task` | egress | `state` accepted (progress), completed or failed; `percent` |
| `c2.declare_role` | `Agent::declareRole` / `fsim_c2_declare_role` / `declare_role` | egress | `role_id`, `callsign` (27 bytes at most) |
| `c2.designate_control` | `Agent::designateControl` / `fsim_c2_designate_control` / `designate_control` | egress | `ma_system_id`, `modes_mask` (1 << `control::FlightMode` each) |
| `c2.command_hsa` | `Agent::commandHsa` / `fsim_c2_command_hsa` / `command_hsa` | egress | `ma_system_id` and the six HSA fields |
| `c2.command_route` | `Agent::commandRoute` / `fsim_c2_command_route` / `command_route` | egress | `ma_system_id`, 1 to 16 `points` |
| `c2.cancel_command` | `Agent::cancelCommand` / `fsim_c2_cancel_command` / `cancel_command` | egress | `ma_system_id`, `command_id` |
| `c2.send_task` | `Agent::sendTask` / `fsim_c2_send_task` / `send_task` | egress | `ma_system_id`, `type`, optional place, window, network-track `target`, `text` (63 bytes at most); the receipt carries `task_id` |
| `c2.set_override` | `Agent::setOverride` / `fsim_c2_set_override` / `set_override` | egress | `ma_system_id`, `override` speed or altitude, and either `value` (with optional `expiry_s`) or `cancel` true |
| `c2.set_lost_comms` | `Agent::setLostComms` / `fsim_c2_set_lost_comms` / `set_lost_comms` | egress | `ma_system_id`, `threshold_s` (0 clears the contingency natively), `plan_id` |
| `c2.form_package` | `Agent::formPackage` / `fsim_c2_form_package` / `form_package` | egress | `package_id`, `leader_system_id`, up to 16 members |
| `c2.dissolve_package` | `Agent::dissolvePackage` / `fsim_c2_dissolve_package` / `dissolve_package` | egress | `package_id`, `system_ids` |
| `gateway.close` | none: the transport's own | the request's service | `request_id` of an earlier request to stop following |

`c2.send_task` with `type` `strike` is still only a typed task's transport: the native MA core requires the weapons
permission of its sender, as today, and an MA that receives it gets a task to decide on, nothing more. Whether a role
may send any of the `c2` operations is decided natively by the receiving MA's RBAC configuration; an unauthorised
command is answered `unauthorized` and has no effect. A flight request on a platform C2 holds is refused natively
(`authority_held`), and the receipt names C2's activity.

Version 1 flight requests cover HSA, routes with a subset of waypoint fields, and registered behaviours with numeric
parameters. Curves, patterns, marshalling, must-fly, intercepts, support commands, stored tasks and route plans stay
available through the existing Agent API and are not requests of this version.

A kind outside this table is an extension's. It passes the envelope and is refused per request with
`unsupported_kind` until its frozen definition and handler are installed.

## 12. `ma.receipts.v1`: acknowledgments and receipts

`submit` validates and queues; it executes nothing. It returns `ma.receipts.v1`, which `receiptsJsonV1` /
`fsim_mission_endpoint_receipts_json` / `receipts_json` read again without submitting again. The same `receipts`
array, frozen with the tick, is in the observation.

**Timing.** With the episode at tick T:

1. Snapshot T is frozen and readable.
2. Endpoints submit `ma.actions.v1` for tick T. Each is checked whole (generation, tick, service) and its requests
   are queued. A service that submits nothing waits; what it started earlier continues natively.
3. `step` closes tick T, dispatches every queued request in ascending `service_id`, then ascending `request_id`,
   each through its service's Agent, advances the World `decision_steps` steps, reads the native statuses of open
   receipts, takes each ingress owner's inbox, and freezes snapshot T + 1.

Arrival order, endpoint order, proxy order and thread scheduling never decide an outcome. Timeouts, ages and windows
are simulation time; wall-clock throttling and captures are outside it.

**The submission** (`submission` member) answers the latest submit:

| Reason | Stage | Meaning |
| --- | --- | --- |
| `stale_generation` | submit | The document's episode key is not the current one. Nothing queued |
| `late_tick` | submit | Its tick is below the episode's. Nothing queued |
| `future_tick` | submit | Its tick is above the episode's. Nothing queued |
| `wrong_service` | submit | Its `service_id` is not the endpoint's. Nothing queued |
| `request_id_reuse` | submit, per request | The id was used with a different body |
| `retry_expired` | submit, per request | The id is not above every id this service has used in the generation, and its record is no longer retained. Never re-executed |
| `capacity` | submit, per request | The service already has `max_in_flight_requests` open receipts |
| `unsupported_kind` | submit, per request | No installed definition handles the kind or operation |
| `capability_not_bound` | dispatch | The service does not own the capability the request commands |
| `dms_ingress_not_owned` | dispatch | A subscription request from a service that is not the ingress owner |
| `dms_egress_not_owned` | dispatch | A send, a task answer or a C2 message from a service that is not the egress owner |
| `message_too_large` | submit, per request | Decoded bytes over `max_message_bytes`. A net's own limit stays a native refusal (`too_large`) |
| `not_applicable` | dispatch | The platform has no such function: a flight or sensor request on a C2 node |
| `service_not_alive` | dispatch | The service's platform is gone |
| `unknown_request` | dispatch | `gateway.close` of a request this service does not have open |

The four whole-document reasons are in `submission.reason`, and such a document queues nothing. A request refused at
submit for `capacity`, `unsupported_kind` or `message_too_large` gets a closed `rejected` receipt of its own, so an
identical retry is a `duplicate` of that refusal and a new attempt needs a new id. `request_id_reuse` and
`retry_expired` are reported in the submission alone: that id's record is another request's, or gone. A reason of
the dispatch stage is the `gateway_reason` of the request's receipt in the next snapshot.

**Deduplication.** A request is identified by its service, the generation and its `request_id`, and compared by its
canonical body (kind, operation, parameters, component). Ids are issued in strictly increasing order within a
document and across a service's documents. Submitting an id again with the identical body returns status
`duplicate` and the original receipt and repeats no effect, at any later tick while the receipt is open or retained.
With a different body it is `request_id_reuse`. Once a closed receipt has been retained for
`completed_request_retention_ticks` ticks it is forgotten, and that id, like any id not above the service's highest,
is `retry_expired`. An open receipt is never forgotten, however old.

**A receipt** carries what the transport did and what the native call answered:

| `state` | Meaning | `closed` |
| --- | --- | --- |
| `queued` | Accepted by submit, not dispatched yet | false |
| `rejected` | Refused by the gateway (`gateway_reason`) or by the native call (`native.reason`) | true |
| `accepted` | The native call succeeded and the transport expects nothing further of it | true |
| `pending` | Natively accepted and waiting: a deferred or disabled activity, a send not settled, a C2 command not answered | false, until its service closes it |
| `executing` | A native activity is active, or an MA accepted a command that has an end | false, until its service closes it |
| `completed` | The native activity or exchange ended as completed | true |
| `failed` | It ended as failed | true |
| `canceled` | It ended as canceled | true |

No state says that a mission, a task criterion or an engagement succeeded: `completed` is a native activity's or
exchange's own end, and outcomes are the referee's. `closed` means the gateway follows the request no longer.
`gateway.close` closes an open receipt in its last known state; the native activity, send or command it stood for
goes on under native semantics. Closing frees its place under `max_in_flight_requests`.

`native` is null until dispatch and for a gateway refusal. Otherwise it carries the native answer by domain, and the
receipt's state follows the native record named below, read once per tick after the World steps:

| `native.domain` | Operations | `native.status` | `native.reason` | `ids` |
| --- | --- | --- | --- | --- |
| `vi` | `flight.*` | The command's answer - `accepted`, `rejected` or `canceled`, this contract's names for `control::CommandStatus` - then, while an activity is followed, its `control::activityStateName` | `control::reasonName` and its code | `activity_id`, `other_activity_id` (the native `other`), `command_id` |
| `ms` | `sensor.set_state`, `sensor.search`, `sensor.cancel` | The same, with `ms::ActivityRecord::state` | `ms::reasonName` and its code | `activity_id` |
| `designation` | `sensor.designate`, `sensor.designate_link`, `sensor.undesignate` | `designated`, `undesignated` or `not_held` | none natively | `target_id` |
| `dms` | `dms.*` | `refused`, `pending`, `delivered`, `failed`, `published`, `done` or `status_evicted` | A refused send's `comm::DmsReason`: `no_system`, `reserved_topic`, `too_large`, `unknown_system` | `message_id`, `message_count` |
| `task` | `task.*` | `answered`, or `not_held_or_decided` where the native call returns false | none natively | none |
| `c2` | `c2.*` | `refused`, `sent`, then the answering MA's `accepted`, `rejected`, `unauthorized`, `completed`, `failed` or `canceled` | A refused send's `comm::DmsReason`; an answer's reason code | `command_id`, `task_id`, `from_system_id`, `activity_id` |

How each operation's state moves:

- **`flight.new`, `sensor.set_state`, `sensor.search`.** A native `rejected` is `rejected`. Accepted, the receipt
  follows the activity: pending and disabled are `pending`, active is `executing`, completed is `completed`, failed
  is `failed`, canceled and deleted are `canceled`.
- **`flight.update`, `flight.cancel`, `sensor.cancel`.** One-shot: `accepted` and closed when the native call
  answers `accepted` or `canceled`, `rejected` otherwise. The effect on the activity shows in the receipt of the
  request that made it.
- **Designations, subscriptions, native publications, task answers.** One-shot: `accepted` and closed, or
  `rejected` with the status above. A native publication's `ids` give its first message and their count.
- **`dms.send`, reliable** (with destinations). `pending` until every destination is delivered (`completed`) or one
  failed with none still pending (`failed`), as `comm::DmsTransmission` reports.
- **`dms.send`, published** (no destinations). One-shot: `accepted` and closed once the native DMS took it. How many
  systems it reached and whether its queue dropped it are in `reports.transmissions` while the receipt is retained;
  the native `queue_dropped` is also set when no receiver was eligible, so it is reported, not judged.
- **`c2.command_hsa`, `c2.command_route`, `c2.send_task`.** `pending` once sent; `executing` when the addressed MA
  answers `accepted`; then `completed`, `failed` or `canceled` as it ends; `rejected` on `rejected` or
  `unauthorized`. The native picture keeps one last status per answering system, command id and topic, and the
  answer to a `c2.cancel_command` reuses the cancelled command's id: once a receipt is `executing`, a later
  `rejected` or `unauthorized` under its id answers that cancel and does not move it.
- **`c2.designate_control`, `c2.set_override`, `c2.set_lost_comms`.** `pending` once sent; closed by the MA's first
  answer: `accepted`, or `rejected` on `rejected` or `unauthorized`.
- **`c2.cancel_command`, `c2.declare_role`, `c2.form_package`, `c2.dissolve_package`.** One-shot on send: `accepted`
  and closed with status `sent`, or `rejected` if the native DMS refused the send. What the addressed systems
  answered is in `reports.command_statuses` (a role declaration's under command id 0, a cancel's under the cancelled
  command's id).

A system that never answers - a C2 node, a platform out of reach - leaves a receipt `pending`; nothing times it
out but its service's `gateway.close`. `reports.transmissions` lists the native status of each send whose receipt is
open or still retained.

The native DMS keeps a system's last 256 send statuses, a window its MA core's own periodic publications and C2
messages share, and its last 256 command statuses. A receipt whose native status is no longer held is closed in its
last known state with `native.status` `status_evicted`: the transport reports what it last knew and claims neither
delivery nor failure.

## 13. Referee documents

`ma.outcome.v1` and `ma.truth.v1` come from the referee handle only. They are the range's and the trainer's:
reward, termination, curriculum and a centralised critic. They never reach an endpoint, an endpoint process, an
observation, a receipt or a mask.

**Noncombat task criteria** (`task.criteria`) are evaluated by the referee on truth, once per tick after the World
steps. Each has an `id`, a `kind` and an `outcome`:

| Kind | Satisfied when | Outcome it may give |
| --- | --- | --- |
| `tick_limit` | The tick reached `max_ticks` | `truncated` |
| `region_reached` | The platform's true position has been within `radius_m` over the ground of the point, and within the optional altitude band, for `dwell_s` simulation seconds without leaving | `success` or `failure` |
| `c2_task_state` | The platform's MA core holds a task from `from_platform`'s system in the given state | `success` or `failure` |

A criterion once satisfied stays satisfied, with its first tick. The episode's `reason` is `failure` if any failure
criterion is satisfied; else `success` if there is at least one success criterion and all of them are; else
`truncated` if a truncation criterion is; else null. `episode_done` is true exactly when `reason` is not null. With
no criteria an episode never ends by itself. `success` is this configured criterion and nothing else: it is not a
native activity's `completed`, a task's `completed` state by itself, a release or a detonation.

**Roles.** One entry per service: `active` (its platform is alive and the episode is not done); `terminated` (its
platform is gone, or the episode ended in success or failure); `truncated` (the episode ended truncated and the role
had not terminated); `final_observation` (it ended: its last snapshot is its final one); `bootstrap` (truncated and
not terminated: a value estimate may bootstrap from the final observation). A lost platform ends its role and leaves
the episode running. The batch layer (M7) builds autoreset on these fields without changing them.

**Truth** lists each entity (id, name, kind, alive, SystemID, a part of the true `VehicleState`), each service with
its entity and controller, and each net's `comm::NetStats`. Weapon, engagement, damage and consent truth, and
versioned rewards, are extension content.

## 14. Capacity limits

| Limit | Default | Bounds | Where it applies |
| --- | --- | --- | --- |
| `limits.max_document_bytes` | 1,048,576 | 1,024 to 16,777,216 | Every document, either direction |
| `limits.max_requests_per_document` | 64 | 1 to 1,024 | `ma.actions.v1.requests` |
| `limits.max_in_flight_requests` | 256 | 1 to 65,536 | Open receipts of one service; over it, `capacity` |
| `limits.completed_request_retention_ticks` | 64 | 1 to 65,536 | How long a closed receipt is kept and deduplicated |
| `limits.max_message_bytes` | 1,024 | 1 to 65,536 | Decoded bytes of one message a service sends; over it, that request is refused `message_too_large`. The native `NetSpec::maxPayloadBytes` (default 1,024) of each net applies as well |
| Services in a configuration | - | 1 to 256 | `services` |
| Nets, C2 nodes | - | up to 64 each | `nets`, `c2_nodes` |
| Criteria | - | up to 64 | `task.criteria` |
| Messages in one snapshot | - | up to 1,024 | The native `comm::kInboxCapacity`; its oldest are dropped natively beyond it |
| Native send and command statuses | - | 256 each | The native DMS's own retention; the send window is shared with the MA core's own messages ([section 12](#12-mareceiptsv1-acknowledgments-and-receipts)) |
| Received C2 tasks | - | 256 | The native MA core's own retention; its oldest are erased beyond it |
| C2 route points, package members | - | 16 each | Native `comm::FlightCommand`, `comm::PackageCommand` |
| Callsign, task text, tail number | - | 27, 63, 23 bytes | Native fixed fields less their terminator |
| Flight route waypoints | - | 1 to 256 | The native route's own bound |
| Destinations of one send | - | up to 64 | This contract's bound |

Reaching a bound is an explicit refusal or an explicit native drop. Nothing is truncated silently.

## 15. Extension seam and pending gates

The generic schema owns the envelope. Extension definitions stay in files of their own and enter at three places,
all by registered reference `<family>.<name>.v<N>`:

- `extensions` in a configuration, an observation, a receipt, an outcome and a truth document: a key is a
  registered reference, its value that definition's document;
- a request `kind` (and its operations and parameters) outside the generic table of
  [section 11](#11-maactionsv1-requests);
- a handler registration that maps the kind to an existing permission-scoped Agent path and returns native
  receipts, whose validation, authority and correlation semantics the gateway preserves.

Until a frozen definition and its handler are both installed, an extension key fails construction and an extension
request is refused `unsupported_kind`. A frozen definition without its runtime changes neither. No provisional
behaviour stands in.

Supplied since M0 (main `c23adf4`), as documents only: the
[engagement extension contract](ma-engagement-contract.md), its [schema](ma-engagement-contract.schema.json), its
[registration manifest](ma-engagement-registry.json) and fixtures. They freeze seven registrations
(`training.authorization.v1`, `training.combat.v1`, `engagement.reports.v1`, `engagement.correlation.v1`,
`engagement.launch_zone.v1`, `engagement.outcome.v1`, `engagement.truth.v1`) and five request kinds (`strike`,
`countermeasure`, `equipment`, `consent`, `engagement_policy`) with their operations and strict parameters, and a
language-independent handler boundary (validate, dispatch, refresh, close). The manifest states
`runtime_installed: false`. None of it executes: every row below that says pending still decides what the SDK does.

| Input | Needed by | Current state |
| --- | --- | --- |
| Extension schemas, registrations and request-kind names | full M5, full M7 | **Supplied, frozen as documents.** Not installed: an extension key still fails construction, and a request of those kinds is still refused `unsupported_kind` |
| `training.authorization.v1` profile and `training.combat.v1` criteria and reward configuration shapes | full M5, full M7 | **Supplied, frozen as documents.** The range-side profile loading and the typed activation operation are proposed there and do not exist |
| Runtime handlers for the five request kinds, and the concrete C++ ownership and registration seam for them | full M5 | **Pending.** The supplied contract fixes the handler's four steps and leaves the seam to be confirmed before facade integration; no handler is installed |
| Actor report producers for `engagement.reports.v1` (stores, strikes, countermeasures, delivered consent requests; C++ and Python Agent parity for countermeasure and launch-zone calls) | full M5 | **Native calls supplied** in `9f18037`; [equipment access/reports](engagement.md#agent-equipment-access-and-reports) retain native permission and provenance. Mission document refresh and gateway handler integration remain pending |
| Outcome reduction: evaluated combat criteria, rewards, team classification and termination from native evidence | full M7 | **Pending.** The supplied contract freezes inputs and evidence only and requires a separately agreed, versioned reduction contract. The generic `reason` and `episode_done` rule of [section 13](#13-referee-documents) is unchanged |
| Full native-state reset fixture (crew loss, fire, an in-flight weapon, a decoy, a consumed store, pending C2 tasks and consent, queued messages, a removed entity) | full M4, M5 core, first package | **Supplied, 2026-10-15**, and applied to the M4 episode reset (`mission_episode`). M5 core acceptance remains pending |
| Dated combat joint and Viewer acceptance evidence (J5, J9, combat J10, the rest of J3 and J8) | full M8 | **Pending.** Not supplied; the [Viewer record](viewer-validation.md) is a diagnostic baseline only |

## 16. Isolation

Strict isolation exports the endpoint whitelist of [section 3](#3-principals-and-proposed-entry-points) and nothing
else: `observation_json`, `submit_json`, `receipts_json`. It refuses an episode that is diagnostic, published or
recorded, refuses to carry a Range, World, Referee or referee document across, and closes or explicitly re-binds a
process endpoint at each new generation. In-process endpoints are an accidental-access boundary; a result that
claims decentralised execution uses the process boundary. A diagnostic run with a Viewer or a recording is labelled
non-isolated, and neither kind of run is evidence for the other.

## 17. Checks and evidence

- `ctest --preset ucrt64-headless -L ma-training --no-tests=error` runs `ma_contract` (the schema's dialect, every
  fixture's expectation, read-back, the seed vectors, this page's tables against the schema, ASCII and local links)
  and `ma_contract_checker` (the checker's own refusals), and the implemented parts' own acceptance:
  `ma_service_binding` (C++), `ma_reports` (C++), `ma_c_abi` (C99) and `mission_package_consumer` (C++, C99 and
  Python consumers built and run against installed packages alone). The Python cases are `test_ma_service` and
  `test_ma_reports` of `python_sdk`.
- The fixtures cover identities above 2^53 up to 2^64 - 1, null numeric reports with their validity, message bytes
  that are not UTF-8, every generic operation, an extension kind and an oversize message as per-request refusals of
  a well-formed document, and one malformed document per rule above.
- The [compatibility baseline](ma-compatibility-baseline.md) records what existing consumers rely on.

The document checks hold the contract's documents to the contract. They are not evidence that an interface exists
or works: that is each implementing package's own acceptance, which today covers the service binding and the legal
report calls and nothing proposed.
