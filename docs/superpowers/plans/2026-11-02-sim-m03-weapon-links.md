# SIM-M03 weapon links implementation window

Baseline: main `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`. This is the complete
weapon communication, fused-target consumption and assessment/feedback vertical
feature assigned by the root. Existing unrelated changes are preserved.

## Contract

The SDK binds private weapon channels to an actual release, launcher platform,
controller and immutable fitted profile. Channels model finite byte capacity,
serialization, propagation, latency, range/LOS, deterministic loss, expiry and
delivery. Public DMS membership/topics/layouts do not change. Only delivered
platform fusion with original provenance can supply updates; no target truth
lookup supplies midcourse data. A fused designation is an additive native tagged
binding in the existing designation-ID namespace, not a radar track number.

Assessment requests are scoped, deduplicated and bounded. Return telemetry is
sampled onboard and becomes readable only after transport delivery. Assessment
SOFT_LOST (no recent delivered report), LOST (assessment timeout) and
END_OF_FLIGHT (delivered onboard terminal notification) are distinct from seeker
Lost/reacquisition. Cancellation suppresses later publication. Bounded terminal
retention survives weapon removal, without inferring a hit or mission result.
Sample/receive/validity epochs remain distinct; unknown numbers are optional.

Public access uses additive V1 C++/C99 and an explicit Python facade. Existing
Python Agent methods and strict actor default callable lists stay unchanged.
The generic unavailable-only engagement.reports.v1 registry/schema stay frozen;
the native handoff does not claim installation of generic handlers.

Actual return fits require primary direction evidence. AIM-120D has a two-way
link in the US Department of Defense FY2026 weapon book. Meteor integration is
platform-specific: Saab explicitly identifies Gripen C/D two-way capability.
No timing, range or payload values are asserted to be hardware measurements.
All numeric channel policy values are disclosed engineering assumptions.

## Exclusive source allowlist

- `include/fsim/WeaponLink.h`, `include/fsim/fsim_weapon_link.h`
- `include/fsim/Agent.h`, `include/fsim/World.h`
- `src/simulation/api/Agent.cpp`, `src/simulation/api/World.cpp`, new `WeaponLink.cpp`
- `src/simulation/runtime/world/World.h`, `World.cpp`, `WorldEngagements.cpp`,
  new `WorldWeaponLinks.cpp` in that directory
- `src/simulation/runtime/services/AgentRules.h`, `AgentRules.cpp` if needed
- new `src/simulation/mission_systems/weapon/WeaponLink.h`, `WeaponLink.cpp`
- `src/adapters/sdk/c/fsim_mission_c.cpp`
- `src/adapters/sdk/python/_native_world.h`, `_native.c`, `_native_mission.c`
- new `python/fsim/weapon_link.py`, `python/fsim/__init__.py`
- `docs/sdk/boundary.toml`, generated `docs/sdk/boundary.md`
- `docs/sdk/weapon-feedback-validation.md`, new `docs/sdk/weapon-links.md`
- new focused native/public/C99/Python tests under `tests/simulation/` and
  `tests/compatibility/`, and new public example under `examples/`
- this plan; root receives exact CMake/install registration patches.

Owned symbols: WeaponLink V1 public values/facade/functions; native channel,
binding, input/assessment/report retention; World fused designation and weapon
link prepare/finish hooks; additive native/C/Python mapping functions.

Read-only reuse: `Fusion.h` and fusion runtime, `Comm.h`, DMS and RF propagation.
No seeker/private sensor/generator edits. Equipment card edits wait for the
sensor owner's release and are recorded separately.

## Additional integration seams

The root supplies private `weapon/NavigationV1.h/.cpp`; this feature owns only
its World captured-state integration and public onboard estimate mapping.
`WorldDamage.cpp` is exclusively the effects owner's file. The effects owner
requested `World::scriptedDetonationSequence_` in the owned World.h and that
field is included here without changing damage implementation.

After the sensor owner released the card window, the metadata allowlist is
`assets/stores/{aim-120c,aim-120d,aim-7m,aim-9m,aim-9x,asraam,iris-t,magic-2,meteor,mica-em,mica-ir,pl-10,pl-12,pl-15,pl-8,r-27er,r-27et,r-27r,r-27t,r-37m,r-60m,r-73,r-77m,super-530d}.json`.
Only `guidance_capabilities.note` changes in this window. The byte replacement
preserves every other field, including all sensor, connected-ring and pulse
descriptors/parameters. The before/after hashes and structural preservation
check are `link-card-note-changes.json` in the current raw evidence directory.

The explicit optional Python process facade is also feature-owned in
`python/fsim/weapon_link.py`: it reuses the existing AgentProcess channel with a
separate method allowlist and supplies only RemoteWeaponLinksV1 to its policy.
Existing generic isolation modules, default methods and schemas are unchanged.

The protocol review retains separate onboard command-delivery state and local
subscription state. SOFT_LOST closes publication without magically informing
the emitter; cancellation consumes a finite uplink command. Receiver deadlines
are evaluated chronologically at each packet arrival. The zero-default jitter
policy admits a deterministic synthetic reorder control, without declaring an
equipment jitter measurement.

All feature-owned non-generated sources follow the existing `.gitattributes`
LF policy. `World.h` contained CRLF in the recorded HEAD and receives mechanical
newline normalization alongside its reviewed semantic changes. Generated/card
byte contracts are not normalized in this window.

## Verification sequence

Write missing-interface/behavior tests first and request the root's exclusive
native RED run. Implement core/channel, real World delivery and public facades,
then request focused native/public tests. Cover analytic queue timing, all-loss,
stale/reordered/duplicate input, fused loss/provenance, assessment dedup/cancel,
terminal delayed delivery/removal, cross-owner refusal, bounded repeated C reads,
Python facade/default actor list, reset/lifetime and seeded worker replay.
Root owns configure/build/CTest, installed consumers and full integration gates.
No independent native command is permitted in this window.
