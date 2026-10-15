# Native-state reset fixture

Status: supplied source-level test input, 2026-10-15. Full M4 episode/reset/lifetime acceptance remains pending.
Starting baseline: main `9f18037`. No production source, public API/ABI, permissions or simulation behavior changes.

## Files and callable contract

- [NativeStateReset.h](../../tests/fixtures/mission_training/NativeStateReset.h): owned IDs/evidence and the callable contract.
- [NativeStateReset.cpp](../../tests/fixtures/mission_training/NativeStateReset.cpp): deterministic scenario and native state setup.
- [NativeStateResetCapture.cpp](../../tests/fixtures/mission_training/NativeStateResetCapture.cpp): owned fieldwise observation map.
- [test_native_state_reset_fixture.cpp](../../tests/test_native_state_reset_fixture.cpp): acceptance and oracle controls.

The test-support target is `fsim_native_state_reset_fixture`; the CTest gate is `native_state_reset_fixture`.
It is reusable by other same-build native tests and is not installed as an SDK library or public header.

```cpp
#include "fixtures/mission_training/NativeStateReset.h"

namespace fixture = fsim::test::native_reset;
fsim::World world(fixture::options("reset-fixture"));
const auto scenario = fixture::configure(world);
const auto clean = fixture::capture(world);
const auto dirty = fixture::contaminate(world, scenario);
fixture::requireDirty(world, scenario, dirty);
const auto observed = fixture::capture(world);
```

`configure` requires an empty World at time zero. It is the scenario recipe to apply identically after complete
owner reconstruction. `contaminate` creates its Agents locally and returns only IDs, correlations and owned values;
no Agent, Vehicle, pointer, reference or World wrapper escapes. `requireDirty` checks that all required states
coexist now. `requireDeliveredMarker` verifies the exact payload and delivery provenance after native stepping;
it refuses the still-queued marker. Both fail explicitly on missing outcomes or wrong correlations. `capture` does not consume events or
messages, create Agents, advance time or normalize native data.

For an internal owner of `session::World`, use the existing explicitly internal `World::borrowed` facade inside a
scope, then destroy that facade before resetting the owner. The supplied borrowed-facade case verifies this seam.
Do not retain a facade or SDK child handle across reconstruction; M4 must independently validate its generation,
closed-state, destruction and move contracts.

The fixture links the SDK facade and internal session libraries from the same build. Public Agent/C2 operations
remain the actor execution path. Internal access is limited to Range damage setup and referee-side inspection.
This white-box link includes duplicate internal static implementations and uses stock built-ins only, without
custom registry/plugin mutation. It is not installed-consumer, ABI, strict actor isolation or Viewer evidence.

## Scenario and simultaneous state

The default recipe uses seed 17, 120 Hz, frame skip four, one unpinned worker and no publication or recording.
UTC is explicitly `2026-01-01T00:00:00Z`; wall-clock initialization is never normalized away. The additional
seed/worker case exercises seeds 0, 17 and `0x123456789abcdef0` with two workers. These are verified cases, not a
claim that every possible seed produces identical damage outcomes.

Creation order is fixed: launcher, reference target, damage victim, removable aircraft, then the aircraft-free C2
node. Aircraft are stock F-16C models at longitude zero, latitude 40.0/40.2/40.5/40.8 degrees, 6000 m MSL,
230 m/s and heading zero. The launcher carries two AIM-120C stores at stations one and two. The C2 node is at
latitude 39.9 degrees and 200 m MSL. A lossless modeled net has 0.05 s latency and no jitter; periodic native
dissemination is disabled. Nothing bypasses transmission, delivery or native role checks.

`contaminate` binds services 4101/4102 before either acts, declares the existing C2 role 2 and waits for delivery.
It operates the store and countermeasure payloads and waits for native startup. The first actor strike requests
consent, receives an actual modeled C2 answer and releases one store. A second strike remains unanswered.
The released active-radar missile has native guidance state, without a supplied target designation; this is a
live-state fixture, not evidence of an intended-target intercept or a complete strike-chain acceptance case.

| Required state | Setup and assertions at the final checkpoint |
| --- | --- |
| Crew loss | Two fixed external native AIM-120C bursts beside the separate victim. Actual native CrewLost event and `VehicleDamage::crewLost()` are required. No flags are assigned. |
| Active fire | Actual native Fire event, a burning holed tank, positive remaining fuel and positive native leakage at the current load factor and measured dynamic pressure. Three subsequent World steps apply native crew/fuel/fire consequences. |
| In-flight weapon | Exact nonzero weapon ID, launcher/activity/store correlation, `InFlight`, armed release, `active_radar` seeker and a phase other than `Unguided`. |
| Consumed store | Exact native store index, `Released` state and the same released weapon ID. |
| Pending consent | Distinct second activity in `AwaitingConsent`, no released weapon, exact delivered request/requester/activity and unanswered state. |
| Pending C2 task | Delivered native-assigned task ID and sender, `Received`, priority seven and exact task text; no acceptance or completion is invented. |
| Live decoy | One native chaff cloud, exact ID and owner, age zero, inventory reduced from 30 to 29 chaff. |
| Queued message | Reliable native send with exact message ID, topic and destination, still `Pending` and absent from the receiving inbox. A separate continuation verifies delivery of the exact binary payload, including NUL and `0xff`. |
| Removed entity | Exact removed aircraft ID and name absent; other aircraft and the C2 node remain. Fresh recipe construction restores it. |

The external burst offsets are body-axis metres `(-1, -4, 0.5)` then `(3.7, -1, -0.55)`, transformed using the
victim's current rotation and velocity, with a forward closing velocity of 600 m/s. They exercise the existing
card warhead, fragment/penetration and damage RNG path. The victim is over 50 km from the launcher, so those
bursts cannot supply the actor's release or damage the communication actors. This is explicit Range setup, not
a missile trajectory or an actor action. No Range consent call fulfills either actor strike.

Dispensing and marker enqueue happen last. There is no step after the final enqueue, so the decoy, live weapon,
fire, pending task/consent, queued message and removal all coexist. Native task/request IDs are returned by the
SDK; an input document never assigns or predicts them.

## Fresh-equivalence oracle and limits

The snapshot owns path/value strings and compares explicit fields rather than object padding or pointers.
Finite floating-point values use classic-locale hexadecimal formatting. Unknown/NaN and infinity have explicit
tokens; unavailable values are not replaced with zero. Native collection order is retained. Native IDs,
correlations, serials, inventory, timestamps, outcomes and generation fields are not normalized by this helper.

Captured fields include World clock/steps/environment; vehicle identities, initial conditions, truth/sensed
states, controls, energy and numeric propulsion/FCS properties; damage, stores, activities, grants, payloads,
radar tracks/status and designations; weapon/decoy/event records; C2 nodes, task/authority/override/RBAC state;
service bindings and Agent audits; DMS inbox bytes, consent, retained send statuses, peers and net statistics.

This is an observation oracle, not an arbitrary internal-state serializer. Private RNG state, guidance integrators,
filters, consent-ledger storage, next-ID counters, core-message contents and arbitrary outgoing/retry queues are
not directly enumerable. Retained send scanning covers this small fixture's contiguous history only. The older
IPC damage-event queue has a consuming accessor and is deliberately excluded from `capture`.

Tests therefore compare exact observations immediately and after successive replay intervals of one, four and
30 World steps. Active replay then repeats contamination and delivery on reconstructed/reference Worlds, comparing
new weapon, decoy, activity, task, consent and send identities as well as the complete captured values. Tests also
verify queued-message delivery, repeat capture without consumption, isolation from a
second World, and absence of old native IPC events after reconstruction. Future M4 must compare the same recipe
and effective seed, with only its episode generation metadata normalized, and additionally exercise its own
task/judge state, cursors, stale handles/requests and lifecycle contracts. If an effective seed lacks a required
native outcome, report the fixture failure; never force flags or relax the assertion to close the gate.

The legacy-reset negative control proves that `Vehicle.reset` retains World time, in-flight weapon/decoy state,
bindings, C2 tasks, queued send and removed entity state. A separately labeled oracle control destroys a dirty
World and constructs a fresh one, then compares it to an independent fresh reference and replay. That manual
replacement proves the supplied comparison input works; it is not an implementation or acceptance of M4 reset.

## Verification

Both existing UCRT64 configurations built the fixture with warnings as errors. The missing-header regression
failed to build before the fixture was supplied. Final results on 2026-10-15:

| Check | Current evidence |
| --- | --- |
| Fixture-specific executable | 7 cases, 68 assertions passed. Includes simultaneous state, binary delivery, deterministic dirty replay, legacy-reset negative control, reconstructed/fresh active replay, borrowed facade, independent World and sampled seed/worker coverage. |
| Headless focused regression | 18/18 passed in 70.05 s: fixture, native engagements, countermeasures, launch eligibility, service binding/reports, C ABI/equipment, contracts, SDK/architecture checks and legacy MA reset preservation. |
| Release focused regression | 4/4 passed in 12.50 s: fixture, C++/C equipment and legacy MA reset preservation. |
| Frozen baseline integrity | All 66 listed artifacts unchanged; baseline was not recaptured. |
| Runtime binary identity | Headless and Release `libfsim.dll` and `libJSBSim.dll` SHA-256 hashes exactly match their verified `9f18037` installed equipment-consumer copies. |
| Change boundary | Only fixture support/tests, test registration and documentation changed. No production source, public header, Python adapter, model, card, generated store or schema changed. |

Reproduce the standalone fixture gates with the existing presets, using `D:/ENV/tmp` for TMP, TEMP and
TORCH_EXTENSIONS_DIR:

```powershell
cmake --build --preset ucrt64-headless --target fsim_native_state_reset_fixture_test
ctest --preset ucrt64-headless -R '^native_state_reset_fixture$' --no-tests=error --output-on-failure
cmake --build --preset ucrt64-release --target fsim_native_state_reset_fixture_test
ctest --preset ucrt64-release -R '^native_state_reset_fixture$' --no-tests=error --output-on-failure
```

Raw build/test/baseline logs are under `D:/ENV/tmp/ma-native-reset-fixture-20261015/`. A read-only independent
source review checked the actor consent path, simultaneous state and oracle limits; its two precision findings
were corrected before the final gates (actual dynamic pressure/load factor, and separate queued-status versus
delivered-byte assertions).

The full suites, installed episode consumers, process isolation and Viewer/GPU scenarios were not run in this
test-input delivery. Full M4 and M5 remain pending. The next implementation must independently validate this
input and apply it to the actual episode reset/lifetime implementation before closing those gates.
