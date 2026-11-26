# SIM-M08 (MS-12): OMS / A-GRA / UCI Standards & Open-Source Alignment

This note records the public standards, open-source references, design tradeoffs,
and strict layering boundaries governing the **SIM-M08 (MS-12)** external
OMS / A-GRA / UCI protocol adapter in `src/adapters/oms/`.

## 1. Authoritative Public Standards & Pinned References

| Standard / Reference | Version / Anchor | Scope Used in SIM-M08 |
| --- | --- | --- |
| **USAF Open Mission Systems (OMS) & UCI** | UCI 2.3 message semantics (`ServiceStatusMT`, `SubsystemStatusMT`, `CapabilityStatusMT`, `ActivityStatusMT`, `CommandResponseMT`, `SubsystemStateCommandMT`, `SubsystemBITCommandMT`, `AMTI_CommandMT`, `PO_*`, `EA_*`, `SAR_*`, `WeatherRadar*`) | Message taxonomy, service lifecycle/heartbeat health transitions, synchronous command-response correlation (`CommandID` -> `ActivityID`), and explicit refusal/cannot-comply propagation. |
| **Open-Arsenal A-GRA (Autonomy Government Reference Architecture)** | ASK 6.0a MS L1 Interface Volume (`31 July 2026`) & `A-GRA_MessageDefinitions_v6_0_a.xsd`, upstream commit `aaabf2cd5c341002e6c62d1448460b55b5634931` (pinned in [ms-work-allocation.md](../../architecture/modules/simulation/ms-work-allocation.md)) | Exact mapping between SDK Mission Systems APIs (`Agent::payloads`, `missionCapabilities`, `missionActivities`, `submit`, `cancelMission`, `ownship`, `amtiTracks`, `contacts`, `payloadReport`, `stores`) and A-GRA status/command/report message forms. |
| **RFC 8259 (JSON) & NDJSON (`application/x-ndjson`)** | IETF RFC 8259 / NDJSON streaming convention | Strict UTF-8 JSON payloads with zero non-standard numeric tokens (`NaN`/`Infinity` forbidden; unknown/unavailable numbers encoded as JSON `null`) and deterministic line-delimited or 4-byte big-endian length-prefixed framing. |
| **IEEE 754 64-bit Identifier Rule** | Project gateway contract (`AGENTS.md`, `MissionJson.h`) | Every 64-bit identifier (`message_id`, `correlation_id`, `service_id`, `command_id`, `activity_id`, `track_id`, `contact_id`) is encoded as an unsigned base-10 decimal string to prevent silent precision loss in JSON parsers using 64-bit floats. |

## 2. Evaluation of Mature Open-Source Transports & Serialization Candidates

Before authoring `src/adapters/oms/`, the following open-source middleware and
serialization stacks were evaluated against the project's build environment
(Windows MSYS2 UCRT64, offline/deterministic CI, strict headless dependency
rules, and zero simulation-core coupling):

| Candidate | Strengths | Limitations in This Repository | Disposition |
| --- | --- | --- | --- |
| **Eclipse Cyclone DDS / eProsima Fast DDS** | Native OMG DDS/RTPS wire protocol commonly used underneath UCI deployments. | Heavy external C/C++ build and runtime dependency graph; network-socket timing and discovery jitter complicate deterministic unit/CI fixtures. | **Deferred to upper-layer bridges.** `IOmsTransport` provides a clean byte-stream seam so an external DDS or RTPS bridge can plug in without modifying simulation or adapter logic. |
| **Protocol Buffers / gRPC** | Strong schema code generation and compact binary framing. | Requires `protoc` codegen toolchain in MSYS2 UCRT64 and adds generated-file maintenance outside existing CMake presets. | **Not adopted for core adapter.** Kept pluggable above `IOmsTransport`. |
| **Existing `fsim::core::Json` + Versioned Envelope (`fsim.oms.v1`)** | Zero external dependencies, already vetted in `fsim_core`, fully deterministic, human-auditable wire fixtures, supports both NDJSON and 4-byte length-prefixed framing. | Textual JSON has slightly larger wire payload than binary CDR/Protobuf, which is negligible at mission-system command/report rates (1-20 Hz). | **Selected** for `src/adapters/oms/` (`fsim_oms` / `fsim::oms`). |

## 3. Architectural & Authority Boundaries

Per [plan.md](../../architecture/modules/simulation/plan.md) (section 5) and
[ms-work-allocation.md](../../architecture/modules/simulation/ms-work-allocation.md):

1. **Strict upward-only adapter placement**: `fsim_oms` (`src/adapters/oms`) sits
   in the `adapters` domain (`tools/architecture/policy.json`) and links ONLY
   `fsim::sdk` (public SDK headers) and `fsim::core` (private JSON parser/writer).
   No simulation, foundation, training, or `libfsim.dll` target depends on
   `fsim::oms`.
2. **Strict `fsim::Agent` binding (zero referee truth leakage)**:
   `OmsServiceAdapter` accepts ONLY an `fsim::Agent&` reference — never `World&`
   or `Referee&`. It is structurally incapable of reading referee truth
   (`truth` fields are omitted from `AMTI_TrackMT` and `ContactMT`), injecting
   weapon release consent, or reading other platforms' internal state.
3. **Explicit separation from modeled RF DMS**:
   Every serialized OMS envelope carries `"transport_domain": "external_oms"` and
   `"authority_scope": "agent_only"`. External OMS frames model software-bus
   exchanges between an onboard Mission Computing Environment (or external OMS
   client) and the platform's own SDK `Agent`; they NEVER substitute for, bypass,
   or duplicate modeled RF datalink delivery (`World::createNet` / `Dms`).
4. **Honest scope and refusal fidelity**:
   Native SDK refusals (`PayloadOff`, `PayloadStandby`, `NotGranted`, `OutOfReach`,
   `NotImplemented`, `NotSupported`, `InvalidParameter`, etc.) propagate verbatim
   in `CommandResponseMT` with their native `reason` and `reason_description`.
   Unfitted or unimplemented capabilities are reported honestly in
   `CapabilityStatusMT` rather than fabricated.

## 4. Service Lifecycle, Idempotency & Watchdog Contract

- **Service registration**: `ServiceRegistrationMT` announces the adapter's
  `service_id`, `service_name`, `vehicle_id`, `platform_type`, `heartbeat_period_s`,
  and `peer_heartbeat_timeout_s`.
- **Periodic heartbeat & peer watchdog**:
  - `OmsServiceAdapter::poll(nowS)` emits `ServiceHeartbeatMT` at every
    `heartbeat_period_s` interval.
  - When a remote peer registers or sends heartbeats, the adapter tracks
    `nowS - lastPeerHeartbeatS`:
    - `<= peer_heartbeat_timeout_s`: `OPERATIONAL`
    - `> peer_heartbeat_timeout_s` and `<= 2 * peer_heartbeat_timeout_s`: `DEGRADED`
    - `> 2 * peer_heartbeat_timeout_s`: `FAULTED`
    - Transport disconnected: `OFFLINE`
    - Receipt of a new valid peer `ServiceHeartbeatMT` restores `OPERATIONAL`
      and emits a state-transition `ServiceStatusMT`.
- **Bounded idempotent command correlation**:
  - Every command carries a non-zero 64-bit `command_id` (decimal string).
  - Retrying the exact same `command_id` with an identical canonical command
    fingerprint returns the original `CommandResponseMT` marked `"duplicate": true`
    without re-submitting to `Agent`.
  - Reusing an existing `command_id` with a different command type or body is
    explicitly rejected (`REJECTED`, `invalid_parameter`, `"command_id_conflict"`).
  - Command history is bounded (`maxCommandHistory`, default `1024`) with FIFO
    eviction while keeping the adapter deterministic and memory-bounded.
