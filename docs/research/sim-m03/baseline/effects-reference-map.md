# Effects implementation and reference map

Executable generic effects with real geometry, causal lifetime and native component consequences exist. Shared-provider/card differentiation is not a complete measured reconstruction of every named warhead, fuze, material or aircraft vulnerability.

This is a read-only source/reference map. It separates adopted mechanisms, external sources reviewed, numerical verification and hardware validation. It changes no production file and claims no solver installation or run.

## Observable behavior and production execution

| Mechanism | Observable behavior | Executed method and exact symbols | Limits |
| --- | --- | --- | --- |
| finite-source | Adding another victim does not mint another metal population or draw a new spray. | One release-specific field samples finite material positions/velocities, splits exactly the case mass across integer projectiles and includes source rigid rotation. [warheadOf](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/Damage.cpp:129); [emitFragmentFieldV1](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/Damage.cpp:803); [FragmentFieldV1](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/Damage.h:241) | Case dimensions, equivalent fragment shapes, angular laws and drag are engineering priors; no per-variant measured fragment census or arena distribution. |
| ordered-flight | An upstream skin slows/stops the same projectile before a downstream component/body can receive it; a finite rod can strike between endpoints. | assessSceneV1 orders actual crossings globally across admitted victims, retains residual material/visited faces and uses analytic constant-air drag with symmetric gravity kicks. finiteLineContact clips the finite-line time/material domain against convex faces. [assessSceneV1](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/Damage.cpp:885); [fragmentFlight](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/Damage.cpp:724); [finiteLineContact](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/Damage.cpp:740); [thorResidual](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/Damage.cpp:240) | Captured affine segments default at most2ms; frozen local field atmosphere, box/quad reduced geometry, generic aluminium skin relation, first solid-component absorption; no deforming material-stack or fracture solver. |
| connected-rods | The four R27 variants expand a connected annulus with a central gap; R60M/R73/R77M remain discrete finite rods. | Neighboring connected elements initially share endpoints/velocities. Actual propagated material length bounds opening, including drag; a local collision breaks that element while preserving its residual material. [emitFragmentFieldV1(connectedRod)](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/Damage.cpp:803); [assessSceneV1 rod opening](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/Damage.cpp:941) | Homogenized continuum, not individual weld deformation, hinge shock, progressive nonuniform fracture or calibrated tumbling. A Talos-referenced inherited speed prior is not an R27 measurement. |
| fuze-contact | Unarmed native contact remains mechanical; admitted armed contact/proximity or a scheduled delay can initiate once at an actually captured pose. | sweptFuze/sweptPathFuze and FuzeTurnStateV1 evaluate captured armed/live motion; contactFuze intersects authored attached contact geometry; ContactFuzeState::commit and FuzeDelayStateV1 own one-shot initiation/delay across contiguous windows. World chooses actual contact/terrain/proximity/lifetime order. [sweptFuze](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/weapon/Fuze.h:49); [sweptPathFuze](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/weapon/MotionHistory.h:34); [FuzeTurnStateV1](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/weapon/MotionHistory.h:131); [contactFuze](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/weapon/ContactFuze.cpp:222); [ContactFuzeState::commit](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/weapon/ContactFuze.cpp:300); [FuzeDelayStateV1](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/weapon/FuzeDelay.h:20); [World::finishEngagements](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/runtime/world/WorldEngagements.cpp:488) | Configured kinematic initiation law and authored delay, not a reconstructed RF fuze waveform/receiver or variant-specific adaptive burst optimization. Publication epoch is a World boundary, not an invented future hit. |
| causal-attribution | The blast is published once; fragments damage only across elapsed captured intervals. Removing/reusing the weapon slot does not erase or replace its original cause. | WorldDamageContinuationV1 owns the resumable field and immutable weapon/launcher/store/activity/sample correlation, plus historical source slot. advanceDetonationEffects feeds actual subsequent motion; applyDamageWithSource preserves native Hit wire meaning. Legacy detonate remains synchronous diagnostic execution. [WorldDamageContinuationV1](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/runtime/world/WorldDamage.cpp:21); [World::detonateAt](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/runtime/world/WorldDamage.cpp:415); [World::advanceDetonationEffects](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/runtime/world/WorldDamage.cpp:620); [World::applyDamageWithSource](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/runtime/world/WorldDamage.cpp:254) | One-second field lifetime and150m admitted recipient reach are bounded model scope, not lethality guarantees. Domain is aircraft and owned debris; no weapon-interception or ground-structure lethality assertion. |
| blast | A burst applies orientation/area-dependent panel impulses that can tear skin and load roots independently of fragment Hit counts. | blastAt evaluates retained scaled peak/impulse fits; reflected supplies a local algebraic face correction; panelBlastImpulse supplies the same projected half-area loading to aggregate/root/separation accounting. [blastAt](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/Damage.cpp:226); [panelBlastImpulse](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/Damage.cpp:278); [assessSceneV1 once-only blast](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/Damage.cpp:1107) | No mesh pressure field, finite pressure-wave arrival, resolved reflection/diffraction/occlusion or pressure-time structural history. An algebraic reflection multiplier is not a CFD-reflected wave solution. |
| component-fire | Actual intercepted component geometry can produce tank holes/fire, engine seizure, control loss or crew loss; weakened structure can fail under later native load. | VehicleDamage::apply uses component-local fragment energy/hits and explicit estimate probabilities, panel tearing/weakening and root impulse response. leakKgS and World::stepDamage remove actual fuel and publish ongoing component fire; World applies native engine/channel/crew consequences. [VehicleDamage::apply](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/State.cpp:128); [VehicleDamage::leakKgS](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/State.cpp:278); [VehicleDamage::loads](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/State.cpp:348); [World::applyDamageWithSource](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/runtime/world/WorldDamage.cpp:254); [World::stepDamage](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/runtime/world/WorldDamage.cpp:687) | Component probabilities, ignition/cracking/tear/leak/burning inputs are disclosed estimates; no measured vulnerability maps, combustion kinetics, chemical fire CFD, fatigue or FEM fracture. Native destroyed retains crew-state meaning, not mission success. |
| owned-debris | A departed body inherits its existing holes/fire/fuel and can shield or receive a later burst; it cannot keep acting as parent geometry or a second fuel body. | VehicleDamage::detachedState preserves original source indices and translates geometry to the child origin. separatedAt/launchPiece transfer current tank/store mass and component state into its native JSBSim body; synchronizeDetachedFuel writes the same aggregate fuel mass/centroid back after leakage. [VehicleDamage::detachedState](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/State.cpp:38); [VehicleDamage::attachedGeometry](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/State.cpp:103); [World::separatedAt](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/runtime/world/WorldDamage.cpp:777); [World::launchPiece](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/runtime/world/WorldDamage.cpp:890); [synchronizeDetachedFuel](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/runtime/world/WorldDamage.cpp:71) | Separate native rigid body, not deforming mesh; empirical section-ejection spring/cap and tumbling aero remain estimates. Reset preserves owned section identity while clearing accumulated damage. |
| mass-and-work | Section loss changes native mass/CG/inertia consistently; added skin drag always removes work from actual air-relative motion. | MassModel builds one nonnegative spatial distribution shared by parent and section writers. engineering.mass_partition_audit reads serialized origins/full tensors and all retained complements. separatedAt applies equal/opposite internal impulses at the loaded piece point. VehicleDamage::aeroLoss points added drag opposite actual air-relative velocity. [MassModel::_reconcile_authored_distribution](D:/Projects/Cpp/Super-DDCCSS-WWorld/tools/models/hangar/hangar/mass.py:269); [damage_data](D:/Projects/Cpp/Super-DDCCSS-WWorld/tools/models/hangar/hangar/damage.py:440); [generate](D:/Projects/Cpp/Super-DDCCSS-WWorld/tools/models/hangar/hangar/engineering.py:661); [mass_partition_audit](D:/Projects/Cpp/Super-DDCCSS-WWorld/tools/models/hangar/hangar/engineering.py:510); [piece_xml](D:/Projects/Cpp/Super-DDCCSS-WWorld/tools/models/hangar/hangar/pieces.py:110); [VehicleDamage::aeroLoss](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/mission_systems/damage/State.cpp:285); [World::separatedAt](D:/Projects/Cpp/Super-DDCCSS-WWorld/src/simulation/runtime/world/WorldDamage.cpp:777) | Conserving rigid-body bookkeeping does not establish measured internal density, aeroelastic fracture or component material accuracy. Geometry inference has nonzero uncertainty. |

Fragment bookkeeping keeps source material, recipients and air distinct: initial material momentum plus external gravity impulse is accounted for by recipient impulse, air transfer, surviving material and absorbed comoving material. This is a ledger checked by independent native fixtures; it does not imply that arbitrary inertial kinetic energy is conserved during drag, penetration or absorption. The target-relative energy and finite-rod intrinsic angular momentum are separate terms.

## References actually checked

### brown-meyer

[Brown and Meyer, The Talos Continuous-Rod Warhead, JHU/APL Technical Digest3(2),1982,pp157-159](https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/V03-N02/03-02-Brown.pdf)

**Status:** reviewed; connected-topology concept adopted, hardware values not calibrated.

Publisher PDF read, including rod topology, continuity and development limitations.

**Connection to this implementation:** Supports distinguishing connected rods from discrete fragments; generic finite shared-vertex model is the project implementation.

**Boundary:** No Talos weld/material/fragmentation or fuze hardware model is transplanted into R27. Paper describes nonuniform/premature breakup; code uses homogenized material-length bounds. No paper arena reproduction was run.

### nirt

[BRL-CAD: Interactive Raytracing, The nirt Command](https://brlcad.org/docs/cad/articles/nirt.html)

**Status:** reviewed; not adopted or executed.

Official ray partition/entry/LOS/normal and backout examples read.

**Connection to this implementation:** Reviewed as a possible independent geometry oracle; forward-origin/inside-solid semantics require an explicit reference transform.

**Boundary:** No NIRT executable, pinned binary, comparison scene or result is claimed. SDK slab/quad/finite-line code does not call BRL-CAD.

### blastfoam

[blastFoam immutable upstream source and support requirements](https://github.com/synthetik-technologies/blastfoam/tree/4e6ee07a0c1fc4629ee7206804f4f1fe802ec64c)

**Status:** reviewed; not installed, adopted or executed.

Pinned README, COPYING and actual applications/solvers/blastFoam/blastFoam.C read. The main advances a mesh-based compressible system; README requires OpenFOAM9. COPYING/file headers specify GPLv3-or-later. README self-labels version6.0; commit hash is the authoritative pin here.

**Connection to this implementation:** Reviewed to identify what a resolved pressure-wave solver and its numerical validation would add beyond the SDK fit.

**Boundary:** No code copied, runtime dependency, CFD run or solver agreement is claimed.

Pinned files: [readme](https://raw.githubusercontent.com/synthetik-technologies/blastfoam/4e6ee07a0c1fc4629ee7206804f4f1fe802ec64c/README.md), [solver](https://raw.githubusercontent.com/synthetik-technologies/blastfoam/4e6ee07a0c1fc4629ee7206804f4f1fe802ec64c/applications/solvers/blastFoam/blastFoam.C), [license](https://raw.githubusercontent.com/synthetik-technologies/blastfoam/4e6ee07a0c1fc4629ee7206804f4f1fe802ec64c/COPYING).

### blastfoam-kb

[Pinned blastFoam Kingery-Bulmash wedge reference case](https://github.com/synthetik-technologies/blastfoam/blob/4e6ee07a0c1fc4629ee7206804f4f1fe802ec64c/validation/blastFoam/KingeryBulmash/KingeryBulmash_wedge/README.md)

**Status:** reviewed; not executed.

Case is a hemispherical TNT surface-burst comparison of peak pressure, impulse and arrival. Its source domain differs from a free-air missile/aircraft scene.

**Connection to this implementation:** Relevant candidate for separately qualifying blast-method numerics, with matching source geometry.

**Boundary:** Its upstream success and timing are not SDK results or aircraft vulnerability validation.

### blastfoam-sedov

[Pinned blastFoam Sedov3D reference case](https://github.com/synthetik-technologies/blastfoam/blob/4e6ee07a0c1fc4629ee7206804f4f1fe802ec64c/validation/blastFoam/Sedov/Sedov_3D/README.md)

**Status:** reviewed; not executed.

Read strong-shock test description, including deposited-energy correction when changing mesh.

**Connection to this implementation:** Distinguishes solver verification from hardware lethality validation.

**Boundary:** No Sedov, CFD or mesh convergence result is claimed for this SDK.

### game-angular-data

[Pinned extracted game angular distribution, AIM120C7](https://raw.githubusercontent.com/gszabi99/War-Thunder-Datamine/ea96f3af75afe1d858f80ac13f212db218bc69ca/aces.vromfs.bin_u/gamedata/weapons/rocketguns/us_aim_120c_7.blkx)

**Status:** reviewed; angular shares adopted for explicitly selected V2 source only.

Read rocket.damage.shatter.segment angles/countPortion at the immutable URL and compared the selection to kFiniteZonesV2.

**Connection to this implementation:** Supplies complete seven-sector angular shares for the explicitly selected finite-zoned AIM120C source.

**Boundary:** Game damage/radius/penetration multipliers are not imported. No equipment measurement or comparative game-fidelity claim follows.

### legacy-attributions

Inherited constitutive attributions in current production source

**Status:** implementation inspected; original publication pages not re-retrieved in this review.

Damage.cpp labels Gurney/Cooper source velocity, Swisdak1994/Kingery-Bulmash blast fits and THOR TR47 aluminium residual penetration; State.h labels material and component estimates.

**Connection to this implementation:** These laws are present in execution; source inspection verifies their use, not the empirical validity of coefficients for every material or weapon.

**Boundary:** Do not report a fresh primary-document coefficient verification, MMPDS material qualification, DDESB test reproduction or measured vulnerability calibration from this map.

## Variant differentiation and remaining measurement gaps

The flat current card population has53 configurations:26 admit a warhead and27 mark it nonapplicable. Only R27R/ER/T/ET select connected-ring topology. R60M, R73 and R77M select discrete finite rods. AIM120C explicitly selects the finite seven-sector V2 source; remaining applicable cards retain their declared generic source law and their own card mass/charge inputs. The JSON companion lists every card and exact hash. Provider names and common laws do not establish proprietary variant-specific fragment distributions or component lethality.

Generic material laws and assumed component-local failure/ignition probabilities remain important model-order limits. Measured explosive/case spectra, rod weld/fracture behavior, real multi-material thickness stacks, RF fuze timing/receiver data and aircraft-specific vulnerability/fire data are not supplied by the references reviewed here. No exact-equipment percentage, DCS superiority claim or calibration status is inferred.

## What the retained execution evidence establishes

- [checkpoint72-effects.log](D:/Projects/Cpp/Super-DDCCSS-WWorld/build/architecture-reviews/sim-m03-complete-20261102/checkpoint72-effects.log) — SHA256 `a7ef33658a4482f9fb83eab7a07109e4ef59cfe5f310b7f4ea2aa9328b562f40`.
- [checkpoint72-showcase.log](D:/Projects/Cpp/Super-DDCCSS-WWorld/build/architecture-reviews/sim-m03-complete-20261102/checkpoint72-showcase.log) — SHA256 `1080ac1271430a2472c4b695af0432a65ce710638f1275c1c2535e63cb82f024`.
- [checkpoint78-b52.log](D:/Projects/Cpp/Super-DDCCSS-WWorld/build/architecture-reviews/sim-m03-complete-20261102/checkpoint78-b52.log) — SHA256 `d2f56be797446985f145b0a1643b20f0c0772d9dfc005d4f3653337d8eb0f7ac`.
- [generation.json](D:/Projects/Cpp/Super-DDCCSS-WWorld/build/architecture-reviews/sim-m03-complete-20261102/checkpoint78-b52/generation.json) — SHA256 `8c88e8ff21af603a1e84c9be2d1b2475100641de31e2b9f29e5e98c444289324`.

Checkpoint72 exercised real scene effects and the unchanged positive-Hit showcase/event-chain contracts. Its source-removal repair preserves the original historical wire slot and immutable weapon ID without changing hit count, store, physics or publication epoch. Independent conservation/interception/RK4 cadence fixtures check method implementation; they do not qualify real weapon effectiveness.

Checkpoint78 is a normal forced B52 engineering generation plus serialized mass audit, not a native B52 flight campaign. It reports five sections and31 physical retained complements; empty mass error is0kg, maximum first-moment error is3.3719631e-10kg m, and maximum full-tensor error is3.7252903e-08kg m². The damage origin agrees with the actual native empty CG/VRP, including the small nonzero lateral residue. These are algebraic/source consistency results, not measured inertia accuracy.

No NIRT, blastFoam, CFD, FEM or hydrocode execution was performed. SourceG full production, installed consumers, full regression and actual fullscreen visual review remain separate root-owned gates.
