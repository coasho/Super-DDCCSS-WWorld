# SDK Missile Engagement Design

Date: 2026-10-11. Approved for implementation.

## Goal and ownership

Complete release, guidance, countermeasures, fuze, damage and authoritative events in the SDK. The game server owns sessions, scenario setup, command forwarding, serialization and scoring from SDK outcomes. The browser owns rendering, UI, player input, sound and display interpolation.

Neither game layer may integrate missile flight, roll decoy probabilities, synthesize warnings from enemy truth or replace SDK weapon entities. Stop after one complete, verified engagement chain.

## Verified starting point

SDK: D:/Projects/Cpp/Super-DDCCSS-WWorld. Game: D:/Projects/Mix/BVRGame.

The game jettisons a store, deletes the SDK entity and creates a local Missile. Flight, seeker behavior, decoy response, launch zones and instant kill judgments live in the game.

The SDK defines guidance phases but Strike remains partial: guidance, seekers and fuzes are unimplemented. Its model factory constructs ESM receivers only. Existing native damage code must be connected and exposed. Python records omit guidance fields.

The SDK initially had neither Git metadata nor built binaries. Establish a local source baseline first. Preserve actual dependency sources rather than broken submodule pointers.

During continuation, the original local repository history was restored. The implementation preserves that history from HEAD `02d1683`; it does not replace it with the initial temporary snapshot. See [delivery validation](../../sdk/engagement-validation.md) for the final verification record.

## Selected architecture

Extend native SDK weapon entities and the existing flight model, with compatible C/Python bindings. Do not create a Python-only alternate simulation. Engineering estimates are labeled assumptions, not real-world calibration.

## Runtime contract

- Use designate and strike. Midcourse follows radar estimates, including stale/false tracks, never exact target truth. Validate release conditions in the SDK.
- Acceptance is distinct from release. Power-up, consent and bay doors may delay departure. Inventory, mass, entities and effects follow actual release. Rejection changes none of them.
- Active radar progresses through midcourse, seeking, homing and lost. Semi-active requires illumination. Infrared acquisition observes aspect, range and field of regard. Lost observations never permit invisible truth updates.
- SDK dispensers own inventory, intervals, motion and lifetime. Decoy effects depend on applicable observations; random streams are separate and reproducible.
- Fuzes require safe arming and swept relative-motion proximity detection. Handle impact and expiry. Jettison remains unarmed. Detonation connects to existing damage and is not an automatic kill.
- Seeker radiation feeds SDK receivers. Infrared missiles do not generate radar warnings from truth.
- Specify state timing; use one dynamics integrator. Preserve fixed steps, worker determinism and unrelated flight behavior.

## Public contracts

Extend WeaponRecord with phase, seeker, holding, miss_m and necessary motor/support telemetry. Append C ABI fields using struct_size compatibility. Document names and units.

Expose SDK dispense commands, inventory and renderable decoy state. Engagement events have monotonic serials, simulation time, entity IDs and reasons. Cursor reads are repeatable and non-consuming; document retention and expiry.

Separate referee truth from player observations. Do not leak enemy target IDs or unobservable telemetry. Provide capability/version checks and clear incompatibility errors without local simulation fallback.

Move launch-zone estimation to the SDK and document assumptions. Update public C++, C and Python interfaces together.

## Game migration

Keep simlink as the platform entry point. Replace fire-by-jettison with strike activities. Read SDK inventory and weapon entities. Remove local integration, SDK-body deletion and decoy adjudication. Preserve ac/ms/me/ev presentation through adapters where possible. AI consumes SDK observations and launch estimates. Scoring consumes actual damage outcomes; effects remain in the browser.

## Acceptance

- Rejection, cancellation, consent, release and mass changes keep inventory/entities consistent.
- Test active activation/acquisition/loss/reacquisition, illumination on/off and infrared aspect with suitable/unsuitable decoys.
- Test dispenser exhaustion, duplicate commands, intervals and lifetime. Probability claims require seeded batches.
- Test swept proximity, safe arming, inert jettison, impact and expiry; one terminal event per weapon.
- Distinguish damage, explosion and kill. RWR reflects received signals.
- Repeat runs and worker counts agree. Preserve flight/radar baselines without engagements.
- Run native, C ABI, Python and full ctest coverage; server smoke, recorded client smoke and a browser engagement.
- Audit game code for local integration, decoy probability, truth warnings and aerodynamic launch-zone calculation.
- Report build/hardware/entity counts, step time and snapshot size before/after, with explicit unverified limits.

## Delivery constraints

All project code, comments, tests, documentation and UI text are English. Use existing UCRT64/CMake. Tools and caches stay under D:/ENV; TMP, TEMP and TORCH_EXTENSIONS_DIR use D:/ENV/tmp. Read dependency support requirements first. Background jobs write raw logs with one entry per job.

Deliver SDK and game changes, public API documentation, test evidence and a runnable SDK entry point. Headers, compilation alone and stubbed graphics do not establish completion.
