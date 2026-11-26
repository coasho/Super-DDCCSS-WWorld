# SIM-M08 (MS-12): External OMS / A-GRA / UCI Protocol Adapter

**SIM-M08** delivers the external OMS / A-GRA / UCI protocol adapter library
(`fsim_oms` / `fsim::oms` in `src/adapters/oms/`), strictly layered above the
public C++ SDK (`fsim::sdk`) and foundation JSON parser (`fsim::core`). See
[open-source-alignment.md](../research/sim-m08/open-source-alignment.md) for the
pinned standards evaluation and architectural boundaries.

## 1. Layering & Authority Guarantees

- **Strict upward-only adapter target**: `fsim_oms` (`tools/architecture/policy.json`
  module `oms`, promoted from `planned_modules`) lives in `src/adapters/oms/`,
  exports its headers through `fsim_include_view(fsim_oms)`, and links ONLY
  `PUBLIC fsim::sdk PRIVATE fsim::core`. Neither `libfsim.dll` nor any
  simulation, foundation, or training target depends on `fsim::oms`.
- **Verified include isolation**: `include_view_oms` proves at compile time that
  `fsim_oms` exposes only `adapters/oms/*.h` and cannot include private headers
  from `simulation/runtime`, `simulation/mission_systems`, or `adapters/sdk/c`.
- **Strict `fsim::Agent` binding**: `OmsServiceAdapter` takes ONLY an
  `fsim::Agent&` and `IOmsTransport&` (never `World&` or `Referee&`). It cannot
  read referee truth, inject weapon release consent, or access other platforms.
- **Explicit separation from modeled RF DMS**: Every wire envelope stamps
  `"transport_domain": "external_oms"` and `"authority_scope": "agent_only"`.
  External OMS frames never traverse, bypass, or inject into modeled RF DMS
  networks (`World::createNet` / `Dms`).

## 2. Framing & Transport (`OmsTransport.{h,cpp}`)

- **Wire framing modes**:
  - `FrameMode::Ndjson`: UTF-8 JSON line terminated by `'\n'` (rejects embedded
    CR/LF and empty lines).
  - `FrameMode::LengthPrefixed`: 4-byte big-endian unsigned payload length
    followed by UTF-8 JSON bytes.
- **Bounded incremental decoding**: `FrameDecoder` handles arbitrary chunk
  fragmentation and fails closed on empty or oversized frames (`maxFrameBytes`,
  default `262144` bytes).
- **Pluggable transport seam**: `IOmsTransport` abstracts byte-stream I/O so
  upper-layer DDS, ZeroMQ, pipe, or socket bridges can connect without coupling
  simulation targets to networking dependencies. `LoopbackOmsTransport::createPair()`
  provides a deterministic in-memory duplex pair for protocol fixtures.

## 3. Versioned Schema & Message Coverage (`OmsSchema.{h,cpp}`, `OmsAdapter.{h,cpp}`)

Envelope schema: **`fsim.oms.v1`** (closed top-level and command key sets;
all 64-bit identifiers encoded strictly as unsigned base-10 decimal strings;
all non-finite doubles encoded as JSON `null`).

| Category | Message Types | SDK / Native Mapping |
| --- | --- | --- |
| **Service Lifecycle & Health** | `ServiceRegistrationMT`, `ServiceHeartbeatMT`, `ServiceStatusMT` | Periodic own-service heartbeat (`heartbeatPeriodS`) and remote peer watchdog (`OPERATIONAL` -> `DEGRADED` -> `FAULTED` -> `OPERATIONAL` recovery; `OFFLINE` on disconnect) |
| **Discovery & Activity Status** | `SubsystemStatusMT`, `CapabilityStatusMT`, `ActivityStatusMT` | `Agent::payloads()`, `missionCapabilities()`, `missionCapabilityStatus()`, `missionActivities()` |
| **Commands & Synchronous Response** | `SubsystemStateCommandMT`, `SubsystemBITCommandMT`, `AMTI_CommandMT`, `PayloadTaskCommandMT`, `StrikeCommandMT`, `JettisonCommandMT`, `ActivityCancelCommandMT`, `ControlGrantCommandMT` -> `CommandResponseMT` | `Agent::submit(...)`, `cancelMission(...)`, `requestControl(...)`, `releaseControl(...)` with bounded idempotent `command_id` replay (`duplicate: true`), conflict rejection (`command_id_conflict`), and verbatim native refusal propagation |
| **Agent-Scoped Reports (No Truth)** | `OwnShipMDT`, `AMTI_TrackMT`, `ContactMT`, `PayloadReportMT`, `StoreInventoryMT` | `Agent::ownship()`, `amtiTracks()`, `contacts()`, `payloadReport()`, `stores()` — `"truth"` keys strictly omitted |
| **Protocol Diagnostics** | `ProtocolErrorMT` | Explicit diagnostics for framing errors, invalid envelopes, unsupported inbound types, or invalid command bodies |

## 4. Verification Summary

Verified with zero compiler warnings (`-Werror`) on both `ucrt64-headless` and
`ucrt64-release` (2026-11-26):

| Check | `ucrt64-headless` | `ucrt64-release` | Scope |
| --- | --- | --- | --- |
| `oms_adapter` (`tests/integration/test_oms_adapter.cpp`) | Passed (`7.06 s`) | Passed (`6.12 s`, 4/4 cases, 304/304 assertions) | NDJSON & LengthPrefixed chunked framing, schema/64-bit/closed-key rejection, registration/heartbeat/watchdog degradation & recovery, F-35A EOTS/radar/BIT/LRF/cancel, idempotency & conflict rejection, zero `"truth"` leakage, C-130J `ControlMode::Granted` weather radar, U-2S SAR dead-ahead `out_of_reach` vs cross-track `Image`, and zero modeled RF DMS packet leakage |
| `include_view_oms` | Passed (`7.15 s`) | Passed (`6.96 s`) | Compile-time verification that `fsim_oms` exports only `adapters/oms/*.h` and cannot see private simulation/SDK headers |
| `sdk_boundary` | Passed (`2.37 s`) | Passed (`2.86 s`) | 2,053 public entry points unchanged, 0 errors |
| Regression (`ms11_families`, `optical_ir`, `esm_ea_comint`, `ma_equipment`, `ma_equipment_c`, `c_abi`) | Passed (6/6) | Passed (4/4 selected) | Confirms zero regressions across existing Mission Systems, C99 ABI, and MA equipment suites |
