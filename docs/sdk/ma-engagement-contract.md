# MA Engagement Extension Contract v1

This page freezes the engagement extension definitions for the generic
[mission training contract](mission-training.md). The separate
[engagement schema](ma-engagement-contract.schema.json) validates these definitions.
The [registration manifest](ma-engagement-registry.json) records their containers
and request operations, with `runtime_installed: false`.
The generic `ma-contract.schema.json` envelope remains unchanged.

**Contract validation and native Agent equipment access/reports are implemented; mission gateway integration is proposed.**
Schema and fixture checks establish document shape and declared boundary rules only.
They do not establish runtime permission enforcement, installed handlers, reset
coverage, physical simulation behavior, or Viewer acceptance. Existing native SDK
operations are identified separately below.

## Registration and document rules

The extension registrations are:

| Registration | Generic document | Purpose |
| --- | --- | --- |
| `training.authorization.v1` | `ma.config.v1` | Optional policy profiles loaded by the range |
| `training.combat.v1` | `ma.config.v1` | Referee criteria and reward configuration |
| `engagement.reports.v1` | `ma.observation.v1` | Reports available to the authenticated service |
| `engagement.correlation.v1` | `ma.receipts.v1` | Native correlations for an individual receipt |
| `engagement.launch_zone.v1` | `ma.receipts.v1` | Advisory query results for an individual receipt |
| `engagement.outcome.v1` | `ma.outcome.v1` | Referee evidence and availability-qualified signals |
| `engagement.truth.v1` | `ma.truth.v1` | Native weapon records and event truth |

Each value appears under its registered key in the enclosing document's
`extensions` object. Receipt extensions belong to the individual request's receipt;
they must not be combined across services or requests. Registration definitions
remain separate from the generic schema. A schema definition alone does not install
a runtime registration.

All identity and correlation fields use the frozen generic decimal-string rules,
including entity, payload, store index, activity, track, system, policy,
revision, request and event serial identifiers. Native counters and measured numbers
retain their documented numeric types. Range `platform` and `launcher` selectors
are configuration names using the generic name syntax; station numbers are native
integers. Unknown measurements use `null`; they are
never replaced with zero or filled from referee truth. Finite values, UTF-8, document
limits, strict field validation, episode generation, stable order, and retry semantics
remain those of the generic contract.

Request `kind` and `operation` are separate simple identifiers. For example,
`kind: "strike"` with `operation: "new"` is valid; a dotted request kind is not.
The generic `c2` operation enumeration is not extended. A request retains the
generic `request_id` and optional `component` provenance. The enclosing service
identity authenticates the caller; request parameters cannot replace it.

An absent registration remains unavailable. An unknown configuration extension is
refused at construction, and an uninstalled request kind is refused per request as
`unsupported_kind`. Neither case executes provisional behavior.

## Range configuration

### `training.authorization.v1`

`profiles` is a collection of optional platform policy profiles:

| Field | Meaning |
| --- | --- |
| `platform` | Configured platform name selected by the range |
| `policy_id`, `revision` | Nonzero policy identity and revision |
| `initially_active` | Explicit initial activation state |
| `strike_policy` | `auto_approve`, `auto_reject`, or `requires_approval` |
| `request_validity_s` | Positive, finite validity duration in simulation seconds |
| `authorizer_role_ids` | Configured roles eligible to answer a profile's release-consent request |
| `activation_role_ids` | Configured roles eligible to request activation |

Profile loading is a proposed range-only operation. Activation is a proposed typed
Agent operation delivered through native C2/DMS authority. A profile never adds a
role, grants a capability, changes a permission bit, or expands the default RBAC
configuration. An activating sender must have the existing weapons permission and
membership in `activation_role_ids`. A release authorizer must satisfy the existing
native role, `approvesStrikes`, weapons permission and requested-sender checks,
as well as membership in `authorizer_role_ids`.

Without an installed active profile, the native legacy strike policy continues.
The existing policy values are `AutoApprove`, `AutoReject`, and `RequiresApproval`;
the default configuration remains `RequiresApproval`. No configuration makes a
mission endpoint the range or grants it direct release consent.

The `(platform, policy_id, revision)` binding and each role list must be unique.
At most one profile per platform can be initially active. Configured role identifiers
must resolve to existing native RBAC roles at loading; standalone schema validation
does not prove this binding. The consent deadline is the request simulation time
plus that active profile revision's `request_validity_s`; there is no independent
policy-expiry field. Policy identity, revision and this deadline participate in
consent correlation. Activation, replacement and request expiry must be revalidated
when a delayed answer is received; a
recorded answer never authorizes another activity, generation or policy revision.
The concrete native load/activation API and dispatch implementation are proposed,
not existing public entry points.

### `training.combat.v1`

`criteria` entries contain `id`, `kind`, `platform`, `outcome`, optional `launcher`,
and optional `weapon_end`. `kind` is one of `damage`, `crew_loss`, `incapacitation`,
`collision`, `friendly_damage`, or `weapon_end`; `outcome` is `success` or `failure`.
Criterion identities are unique. `weapon_end` is required exactly when the kind is
`weapon_end`, and selects `impact`, `removed`, `detonated`, or `expired`. It uses
the native weapon-end names, not a synthetic kill
state. A launcher selector constrains attribution only when native evidence supplies
that attribution.

The optional `rewards` object admits finite weights for `damage`, `crew_loss`,
`incapacitation`, `collision`, `friendly_damage`, `miss`, and `mission_success`.
Weights are explicit configuration values; there is no implicit
kill reward, team label, friendly-fire classification, or terminal condition. A
criterion's target identity is range/referee configuration, never an actor target
shortcut.

The schema freezes the configuration shape. Runtime evidence reduction, reward
calculation, team classification and combat termination remain proposed and require
independent validation. No reward or criterion is satisfied without the required
native evidence. Missing evidence is unavailable, not a negative measurement or a
successful event. A release, an accepted command, a completed activity, a lost
track, or a detonation alone does not satisfy mission success.

The frozen generic outcome derives `reason` and `episode_done` from its noncombat
criteria. This extension does not override that rule or add generic criterion
kinds. Evaluated combat criteria, rewards and termination need a separately agreed
versioned reduction contract before integration. The definitions here freeze inputs
and evidence only.

## Actor reports: `engagement.reports.v1`

Reports originate from the authenticated service's own Agent or from messages
actually delivered to its owned ingress. Snapshot provenance, report age and
non-consuming reads follow the generic contract. Global truth filtered by launcher
or platform is not a substitute for a legal Agent report.

Native store, strike, countermeasure and delivered-consent report calls are available through the
[Agent equipment interfaces](engagement.md#agent-equipment-access-and-reports). These calls do not install the
enclosing mission document or its gateway handlers; the registration manifest remains `runtime_installed: false`.

| Collection | Frozen content and boundary |
| --- | --- |
| `stores` | Own inventory: index, payload, station, station name, store, mnemonic, store class, carriage, powerable, jettisonable, flyable, status, mass and released weapon identity |
| `strikes` | Own strike/jettison activity identity, native phase, jettison flag, own target designation, ordered store indices and released weapon identities |
| `countermeasures` | Own inventory and availability; chaff and flare counts are native values |
| `consents` | Requests actually delivered to the service's approving C2 system, their received time, answered flag and native request correlations |
| `weapon_feedback` | Explicit unavailable feedback, as specified below |

The store fields above are the current common C++/C/Python minimum. Native C++
`StoreInfo` also contains fuel and bay-door values; this contract does not claim
that the existing C or Python representations expose those fields. Any later
addition must be versioned and validated across all language adapters.

Strike target identities are the service's own designations or zero. A released
weapon identity is a native release correlation, not access to that weapon's
position, seeker, target truth or outcome. Read access to an activity does not grant
another service permission to cancel or update it.

Each consent request retains the requesting MA system and native
`consent_request_id`, strike activity identity, store and quantity, target track's
source system, payload and track identity, launcher's sensed navigation position,
request time, received time and answered state. These are the contents of a native
delivered request, not a global list of pending approvals. DMS ingress ownership
still applies, including when several services belong to one platform.

Store, strike and countermeasure metadata use `source: "own_platform"`; consent
metadata uses `source: "dms"`. Collection counts match their items, identities are
unique within their native scopes, and unavailable collections have no items.
Countermeasure inventory is null unless available. Sample time and age are known
together; their age agrees with the enclosing observation time. A stale report
must retain its age. Received consent times cannot precede request times.

Current native weapon feedback has no modeled actor report. Its frozen representation
is `availability: "unavailable"`, `reason: "no_modeled_feedback"`; `holding_id`,
`tracking`, `supported`, `miss_m`, `time_to_go_s`, and `time_to_active_s` are all
`null`. Native `WeaponRecord` fields are referee truth. They cannot become actor
telemetry by selecting records whose launcher happens to be the actor's platform.

Capability `support` and `supportTable` reports describe SDK feature support. They
must not be relabeled as weapon illumination, midcourse support, seeker tracking
or a predicted hit.

## Typed requests

Parameters are strict objects. Undeclared fields are refused, including `consent`,
`source`, `controller`, `service`, `service_id`, `platform`, and aliases that try
to replace the authenticated caller or grant direct authority.

| `kind` | `operation` | Parameters | Native mapping and current status |
| --- | --- | --- | --- |
| `strike` | `new` | `payload`, `target_id`; optional `store`, `quantity`, `stores`, `ripple_s` | Existing `Agent::submit(ms::StrikeCommand)`, `fsim_vehicle_strike`, Python `Agent.strike`; extension handler proposed |
| `strike` | `cancel` | `activity_id` | Existing `Agent::cancelMission`, `fsim_ms_activity_cancel`, Python `Agent.cancel_mission`; own activity only; extension handler proposed |
| `strike` | `jettison` | `payload`; optional `stores`, `emergency` | Existing `Agent::submit(ms::JettisonCommand)`, `fsim_vehicle_jettison`, Python `Agent.jettison`; extension handler proposed |
| `countermeasure` | `dispense` | `kind`: `chaff` or `flare` | Existing `Agent::dispense`, own-scoped `fsim_world_dispense`, Python `Agent.dispense`; extension handler proposed |
| `equipment` | `launch_zone` | `store`, nonzero `target_id` | Existing `Agent::launchZone`, own-scoped `fsim_world_launch_zone`, Python `Agent.launch_zone`; extension handler proposed |
| `consent` | `answer` | `ma_system_id`, `consent_request_id`, `granted` | Existing `Agent::answerConsent`, `fsim_c2_answer_consent`, Python `Agent.answer_consent`; extension handler proposed |
| `engagement_policy` | `activate` | `ma_system_id`, `policy_id`, `revision` | Proposed typed Agent/C2 operation; no existing public activation API |

For `strike/new`, a nonempty `stores` list selects exact native store indices and
takes precedence over store type and quantity as the native command does. Omitted
fields retain native defaults. The target is zero or a designation owned by the
service's platform; another aircraft's true entity identity is refused. The handler
always commands through the bound Agent and cannot set `StrikeCommand::consent`.

Countermeasure dispense is exactly one native operation for the selected kind.
It does not install an automatic response or a salvo scheduler. The existence of
`ms::PayloadTask` data fields for salvos and intervals does not establish a modeled
public operation, and these fields are not admitted by this request.

Consent answers require the service's DMS egress and a matching delivered unanswered
request. A successful send means only that native DMS accepted the answer. The
receiving MA independently checks its sender's declared role, approval flag,
weapons permission, asked-system membership, live activity and profile state. The
first authorized processed answer retires that native request; repeated or late answers cannot
release another store. Raw reserved-topic bytes and range `strikeConsent` are not
actor alternatives.

## Receipt extensions

### `engagement.correlation.v1`

This extension contains `ma_system_id`, `consent_request_id`, `activity_id`,
`policy_id`, `policy_revision`, and `reason_domain`. Only correlations returned or
held by native execution are recorded. Unknown correlations remain null. Policy
identity and revision are known together; a consent request requires its MA system
scope. Command, message and sender identities use the existing generic
`receipt.native.ids` fields when returned. No topic field is introduced, and the
extension does not expand the generic `native.domain` or `ids` enumerations.

The gateway request is identified by service, episode generation and request id.
A native consent request is scoped by its requesting MA system; its request number
alone is not globally unique. A DMS message is scoped by its sender, and C2
command/task identities by their sending system. These scopes must survive receipt
transport and delayed delivery.

Native reasons are transported with their actual domain, name where known and
numeric code. MS activity refusals use `ms::Reason`; DMS send refusals use
`comm::DmsReason`. A consent answer's C2 status uses `control::Reason` for the
authorization refusal branch and `ms::Reason` for the subsequent `strikeConsent`
result. Its send refusal uses `comm::DmsReason`. The reason domain follows the
actual branch; neither the C2 topic nor a numeric code alone identifies the enum.

### `engagement.launch_zone.v1`

The receipt carries `basis: "assumed"`, availability (`available`, `unavailable`,
`not_applicable`, or `not_owner`), and `maximum_m`, `no_escape_m`, `minimum_m`.
Distances are null unless available. The handler validates a nonzero own designation
and a carried applicable store before interpreting native values in the proposed
endpoint query. This adds no check to the existing native query API. Default zero
distances are not evidence that a query was supported.

The current native advisory uses the own platform's launch state and the target
designation's estimate. It is an energy envelope with assumed reference values,
not a second trajectory integrator. It supplies no `hit_probability`, release
consent or guaranteed intercept. Existing launch eligibility, station state,
illumination and authorization checks still run when a strike is submitted.

## Referee outcomes and truth

`engagement.outcome.v1` carries native event evidence and signals whose
`availability` qualifies `value` (`null` or boolean). It is exposed only through
the referee outcome document. `engagement.truth.v1` exposes typed native event and
weapon records only through referee truth. Neither extension belongs in actor
observations or an actor endpoint's transport whitelist.

Event evidence preserves native serial, simulation time, kind, reason, entity,
launcher, target, ECEF position and velocity. The native `destroyed` flag is
transported as `crew_lost` to preserve its actual meaning. The minimum weapon
projection preserves identity, launcher, activity, designation, `end`, `phase`,
holding identity, tracking/support flags and nullable terminal/advisory
measurements. Native weapon-end names are `in_flight`, `impact`, `removed`,
`detonated`, `expired`; phases are `unguided`, `midcourse`, `seeking`, `homing`, `lost`.

Signal kinds are `damage`, `crew_loss`, `incapacitation`, `collision`,
`friendly_damage`, `structural_breakup`, `crash`, and `miss`. Administrative entity
removal is not a signal kind; native weapon-end `removed` remains distinct.
`EngagementEvent::destroyed` means actual
crew loss in the current SDK. It does not prove deletion, breakup, loss of all
flight control or terrain impact. Engine failure alone is not automatic operational
incapacitation. No configured criterion changes physics to make its signal true.

Detonation is the fuze/warhead event, not a kill. Removal can be an administrative
operation. A collision or crash needs its actual native evidence. Friendly damage,
miss, attribution and mission success require their stated native evidence and
classification; if these are not modeled or not evidenced, the signal remains
unavailable and provides no inferred reward.

Each signal's value is boolean exactly when available; otherwise it is null.
Event serials and weapon identities are unique; each `(kind, subject_id)` signal
is unique. An event reference resolves within the outcome's evidence. An available
true crew-loss signal attributed to an event requires a native `damage` event with
`crew_lost: true` for that same target. A null event reference can represent an
independent referee-state sample; static validation cannot prove its provenance.
An in-flight weapon has a null end time; a terminal weapon has a nonnegative native
end time. These checks do not prove runtime evidence production.

## Handler contract

The frozen handler boundary is language-independent:

1. **Validate** the registered contract, strict parameters and authenticated
   service/generation context. Produce a document refusal, gateway refusal, or a
   validated queued request. Validation performs no native command or world mutation.
2. **Dispatch** at the generic step boundary, in its stable service/request order,
   through the bound Agent. Return the existing generic native result plus actual
   extension correlation or advisory-query data. Authority is checked at execution.
3. **Refresh** the service's own activity and held native statuses once per tick,
   preserving the generic receipt lifecycle and non-consuming reads. Native-status
   eviction remains explicit; no synthetic timeout claims an unobserved result.
4. **Close** detaches gateway tracking only. It does not grant consent, cancel a
   native strike, remove a weapon, or rewrite an outcome. Cancellation remains an
   explicit typed native operation.

The concrete C++ internal ownership/registration seam must be confirmed before
facade integration. This document introduces no plugin vtable, public type layout,
exported handler API, or claim that the proposed mission handlers are installed.

## Verification boundary

The schema, documentation and fixture checks validate this frozen contract. They
do not execute simulation or validate runtime authority. Independent episodes,
full native-state reset fixtures, runtime handlers, installed consumer execution,
combat outcome/reward validation, and combat joint/Viewer acceptance evidence
remain pending integration gates. The existing
[Viewer validation record](viewer-validation.md) is a diagnostic baseline.

Run `python -B tools/validation/ma_engagement/check.py` and
`python -B tools/validation/ma_engagement/test_check.py` from the repository root, or select
the CTest `ma-training` label. The [fixtures](../../tests/training/fixtures/mission_training/engagement/manifest.json)
cover every registered request operation and extension, including deliberate
boundary refusals. The checker also validates complete generic envelopes with these
proposed registrations and preserves generic limits and outcome rules. It is a
repository tool, not an actor request executor or native handler registry.

Existing implementation references:

- [Agent and referee API](../../include/fsim/Agent.h): own reports, typed requests,
  consent inbox and referee truth.
- [Mission Systems types](../../include/fsim/MissionSystems.h): native stores,
  activities, consent-dependent strike phases, weapon records and event semantics.
- [Communication types](../../include/fsim/Comm.h): RBAC, native consent payloads,
  send refusals and C2 correlations.
- [C mission adapters](../../src/adapters/sdk/c/fsim_mission_c.cpp): own-scoped countermeasure
  and launch-zone paths, store parity and range-only direct consent.
- [Agent rules](../../src/simulation/runtime/services/AgentRules.cpp) and
  [native strike dispatch](../../src/simulation/runtime/world/World.cpp): own designations and direct
  consent refusal for minted Agent controllers.
- [MA core](../../src/simulation/runtime/services/MaCore.cpp) and
  [consent ledger](../../src/simulation/runtime/services/MaConsentLedger.h): modeled DMS authorization,
  first-answer retirement and native request scope.
- [Native engagement implementation](../../src/simulation/runtime/world/WorldEngagements.cpp):
  single dispense, advisory launch zones and retained event provenance.
- [Python Agent facade](../../python/fsim/agent.py): current delegated store/strike
  reports and typed C2 consent access.
