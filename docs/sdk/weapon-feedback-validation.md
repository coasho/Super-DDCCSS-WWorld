# Weapon feedback and actor visibility

Date: 2026-10-20. Baseline: main
`0d4f25a5df3fdd23696e696644cbd0ee32ab23af`.

This is a bounded SIM-M03 actor visibility correction and a record of the
remaining feedback/input gates. It does not establish calibrated seeker,
autopilot, fuze, lethality or weapon data-link behavior, or complete SIM-M03.

## Direct strike status

An Agent may read the status of an activity on its platform under the existing
activity permission checks. A Range-created strike may name a true target entity;
that identity is not Agent knowledge. `Agent::reportStrikes` already replaces
such a target with zero, and retains a designation only when it belongs to the
Agent's platform. The direct `Agent::strikeStatus`, C
`fsim_strike_status_get` on an Agent handle, and Python `Agent.strike_status`
must apply the same target rule.

The correction changes only the returned copy. Range status, native targeting,
activity identity/phase, ordered store indices, released weapon correlations and
command permissions retain their existing meanings. A lapsed own designation is
still an own designation; the read does not renew it or make it a current sensor
observation. Foreign-platform activities remain refused. Released IDs grant no
weapon telemetry. Existing public signatures, layouts, enum values and caller-sized
C result-buffer behavior remain unchanged.

The regression set covers a Range-created true target through all three public
language paths, repeated reads, Range preservation, foreign activity refusal and
C short result buffers. The C++ control also obtains native radar tracks through
Agent observations and verifies preservation of an own designation, including
after it lapses, and redaction of another platform's designation.

Before the correction, the serial native verification reproduced the disclosure:
the C++ regression failed both target-redaction assertions, the C99 regression
failed `visible.target == 0`, and the Python regression returned target 2 instead
of zero. The implementation now applies the existing report mapping in both
direct getters. The final direct C++ getter case, C99 equipment fixture and
Python engagement/equipment modules passed. The complete public/installed
selection passed 8/8 CTest cases; the core selection passed 9/9. Exact commands,
loaded installed-consumer provenance and logs are in the
[package validation record](weapon-fidelity-validation.md).

## Existing weapon input and truth

`World::prepareEngagements` consumes a launcher-owned valid designation. A local
designation follows its radar track; a network designation follows an entity
report actually delivered to the platform's DMS. Unsupported input stops
refreshing the guidance estimate, which then extrapolates its last accepted
observation. Raw entity targets do not synthesize midcourse observations.

This is currently an idealized midcourse update: designation validity directly
sets support. There is no explicit launcher-to-weapon transport queue, update
latency, link loss or per-message resource charge in that path. Network reports
are retained by time since delivery. `comm::EntityReport` explicitly describes an
un-fused radar track and carries no fused Entity ID, status, standard identity or
explicit loss update.

World weapon records, engagement events, decoy positions and on-rail target entity
queries remain Range/referee interfaces. C Agent handles refuse these calls;
count-style refusals return zero. The frozen `engagement.reports.v1` representation
continues to require `unavailable / no_modeled_feedback` and null weapon telemetry.
Filtering World records by launcher identity would not create a modeled actor
feedback path. No schema or gateway registration is changed by this correction.

## Pending complete feedback feature

The allocation's [weapon employment crosswalk](../architecture/modules/simulation/ms-work-allocation.md#78-weapon-employment-13)
assigns receive-weapon-flyout semantics to SIM-M03: assessment accept/reject,
permitted periodic `MA_EntityMT` feedback including engagement and timing, and
cessation at the specified `LOST`, `SOFT_LOST` or `END_OF_FLIGHT` states. Existing
native `WeaponPhase::Lost` is a seeker state that may reacquire; it must not be
silently equated with an assessment/publication terminal state.

A complete native delivery must freeze the fitted weapon/link profile, direction
of communication, report/request identities, assessment lifetime, periodicity,
sample/receive time, stale/unknown behavior and exact state mapping. Feedback
must originate as a modeled onboard report, travel over a supported native
communication path, and enter a retained platform report only after actual
delivery. Failed, delayed, duplicate, reordered, expired and canceled deliveries
need explicit behavior. Terminal reporting must survive the weapon's native
removal without inferring a hit, damage, crew loss or mission success.

Acceptance must exercise the public C++/C99/Python Agent paths, repeatable
non-consuming count/value reads, configured service boundaries, unrelated
platform refusal, supported versus unsupported fits, deterministic replay and
generation/reset lifetime. New versioned types/entry points must preserve old
public types and enum values. Actual equipment models lacking admitted return
links stay unavailable.

## Pending fused input feature and ownership

The allocation assigns the fused report feature to Claude/SIM-M01 and its weapon
consumption to Codex/SIM-M03. Their accepted seam must carry fused Entity
identity, source, status, identity evidence and kinematics, with sample time,
receive time, validity/uncertainty and loss semantics. A weapon may consume only
updates that reached its configured platform and then reached the weapon through
its supported modeled path. A missing, stale, lost or unknown input cannot be
filled from target truth.

Claude retains the generic gateway, existing generic schemas, registration,
receipts, snapshot fanout, actor process isolation and installed generic
consumer acceptance. A future modeled feedback feature requires a reviewed
versioned extension contract and native handoff to that owner; it cannot change
the frozen unavailable-only v1 report or mark its uninstalled registry installed.

## Primary-source and physical closure

The allocation pins the original MS Interface Volume v6.0a and companion schema
with source commit and SHA-256 values. Its section 7.8 records flyout state on
printed page 126 and weapon entity input on page 130. This correction consumes
that tracked crosswalk; it does not claim a fresh independent examination of the
original PDF/schema or completed OMS wire conformance.

Before physical acceptance, freeze primary manufacturer/operator/government
evidence for each actual fitted variant and data-link direction, update rate,
latency, range and permitted feedback content. Separately admit seeker response,
control dynamics, fuze and lethality inputs with conditions, units, uncertainty,
independent reference cases and tolerances. Missing public data remains unknown
or explicitly assumed and prevents a calibrated actual-model acceptance claim.
SIM-M02 flight-model admission and the SIM-M01 fused-report seam remain their
owners' dependencies. This bounded visibility correction closes none of them.
