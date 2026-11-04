# Modeled weapon links V1

This additive opt-in feature implements native communication between an actual
released weapon and its bound launcher. The SDK owns production, transmission,
delivery, guidance consumption and retained assessment reports. The actor sees
only delivered onboard reports. The frozen unavailable-only
`engagement.reports.v1` representation is unchanged.

## Equipment direction and engineering policy

The [US DoD FY2026 weapon book](https://comptroller.defense.gov/Portals/45/Documents/defbudget/FY2026/FY2026_Weapons.pdf)
explicitly identifies a two-way link and GPS-enhanced inertial navigation for
AIM-120D. It does not publish the radio's rate, delay, frequency, uncertainty or
its proprietary telemetry protocol. The modeled AIM-120D return payload is an
engineering interface, not a reconstruction of that protocol. Launcher
integration selector is the repository's F-16C. [AFMC's 2011 report](https://www.afmc.af.mil/News/Article-Display/Article/153727/edwards-test-squadron-returns-to-red-flag)
identifies F-16 integration work with AIM-120D. Other launcher combinations are
receive-only until their particular return integration is admitted.

[Saab's Gripen C/D Meteor announcement](https://www.saab.com/newsroom/press-releases/2010/future-missile-system-for-gripen)
identifies two-way aircraft/missile communication. [Saab's integration discussion](https://www.saab.com/markets/india/gripen-for-india/technology/meteor-and-gripen-the-gamechanger-duo)
also distinguishes platform integrations. The repository's generic `gripen`
model's XML identifies JAS 39C and uses that sourced integration. This does not enable a
Meteor return link on every aircraft. [NAVAIR's AMRAAM description](https://www.navair.navy.mil/product/AMRAAM)
supports launcher-to-missile target updates at family level; it does not establish
a return transmitter on AIM-120C-7.

The immutable native profile records its direction source. Receive-only fits
follow the existing explicit authored `launcher_updates` capability; these
remain declared engineering fits where exact hardware documentation is absent.
Every unknown return fit refuses assessment with `Unsupported`.

| Current store | Receive fit | Return fit |
| --- | --- | --- |
| aim-120c | Authored receive update | Unsupported |
| aim-120d | Authored receive update | Sourced F-16 integration only |
| aim-7m | Unsupported | Unsupported |
| aim-9m | Unsupported | Unsupported |
| aim-9x | Unsupported; current card is Block I | Unsupported |
| asraam | Unsupported | Unsupported |
| iris-t | Unsupported | Unsupported |
| magic-2 | Unsupported | Unsupported |
| meteor | Authored receive update | Declared Gripen C/D integration only |
| mica-em | Authored receive update | Unsupported |
| mica-ir | Authored receive update | Unsupported |
| pl-10 | Unsupported | Unsupported |
| pl-12 | Authored receive update | Unsupported |
| pl-15 | Authored receive update | Unsupported |
| pl-8 | Unsupported | Unsupported |
| r-27er | Authored radio correction | Unsupported |
| r-27et | Unsupported | Unsupported |
| r-27r | Authored radio correction | Unsupported |
| r-27t | Unsupported | Unsupported |
| r-37m | Authored receive update | Unsupported |
| r-60m | Unsupported | Unsupported |
| r-73 | Unsupported | Unsupported |
| r-77m | Authored receive update | Unsupported |
| super-530d | Unsupported | Unsupported |

Numeric defaults are engineering assumptions: 0.2 s update/report period,
0.05 s transport delay with zero default jitter, 64 kbit/s, 200 km range gate, 1 GHz propagation
evaluation, 0.5 s maximum serialization wait, 2 s data freshness, 5 s assessment
timeout, 30 s terminal retention, 64 queued packets and 128 retained reports per
assessment. None is a measured equipment value. Native synthetic fixtures vary
these policies independently; real equipment parameter calibration is separate.

## Finite transport and measurement epochs

Each weapon is bound at release to its launcher, original activity/controller,
store, target reference and fitted profile. Arbitrary actor payloads cannot
change that binding. Public DMS topics, layouts and subscriptions do not install
weapon radios or inject their messages.

For a packet with `B` bytes, rate `R` bits/s and range `r`, its start is
`max(send_time, resource_busy_until)`, serialization is `8*B/R`, and delivery is
`start + serialization + max(0, latency + jitter) + r/299792458`. Jitter, when
explicitly configured, is uniform and seeded per release. A shared per-launcher transmit
resource serializes its weapon updates. Each weapon has a separate return
resource. Range/LOS/queue refusal precedes transmission. A transmitted lost
packet still consumes its byte/time resource and cannot create a report.
Canonical packet sizes are 384 bytes plus 48 bytes per fusion source for input,
64 bytes for an assessment command, and 512 bytes for feedback including full
navigation covariance. RF eligibility reuses native terrain and
curved-Earth path evaluation. Per-release RNG streams are independent of worker
scheduling and ordinary platform DMS traffic.

The input is the accepted platform `FusedEntityV1`, including ECEF posterior,
full covariance, fused ID, state, identity availability and every contributing
source's original sample/delivery epoch. An existing radar designation maps to
its admitted fusion source; an explicit fused designation binds a fused ID
without creating a radar track. No target-truth lookup fills missing fusion.

The posterior's `timeS` describes its coordinates. `lastUpdateS` describes the
latest underlying measurement. Source records retain both sample and platform
receive times; weapon delivery adds a distinct receive epoch. Guidance receives
coordinates and their matching epoch. Propagation, repeated reads and duplicate
transmission do not create measurements. Lost inputs, expiry, malformed
covariance, future epochs and reordered reports cannot renew support. A valid
onboard seeker remains independent of a failed uplink.

## Assessment lifecycle

An assessment request carries world instance, own released weapon, request ID,
period and duration. Only the release's controller may assess it, except a
Range's explicit diagnostic facade. Native acceptance means an assessment was
admitted; the assessment command must then reach the weapon before onboard
publication can start, and a report must separately reach the launcher.
Requests are bounded. Identical
retries return the original receipt; changed-body reuse returns
`RequestConflict`. Full tombstone capacity refuses new work rather than
re-executing an evicted request.

| State | Meaning |
| --- | --- |
| Pending | Accepted, no return report delivered |
| Active | A permitted report arrived |
| SoftLost | No recent return report within freshness policy; this assessment's publication stops |
| Lost | Assessment communication timeout; publication stops |
| EndOfFlight | An onboard terminal notification actually arrived; publication stops |
| Canceled | Owner canceled locally; queued reports cannot reopen publication; a finite uplink command stops the onboard emitter after delivery |
| Expired | Requested assessment lifetime elapsed; publication stops |

These states describe assessment publication, not seeker phases. A seeker may
lose and reacquire while an assessment remains active. Terminal feedback is
sampled from the onboard shutdown event before native removal and may arrive
later. Forced Range removal has no onboard transmit event and therefore closes
through timeout rather than inventing an end notification. Retained bounded
reports survive the weapon entity; launcher removal or full World replacement
invalidates the binding. Nothing here infers a hit, target damage or success.

`SoftLost`, `Lost` and `EndOfFlight` cannot be reopened by a late packet. Local
publication and onboard emission are distinct: receiver timeout is not magical
information at the weapon. Until its cancellation command arrives or its known
lifetime ends, an onboard emitter may still consume radio resources even though
the local subscription no longer publishes its reports. Receiver deadlines are
applied at each actual packet arrival epoch; infrequent reads or batched
advancement cannot erase successful past deliveries or heal a publication gap.

A native weapon pose reset closes its old communication generation: queued old
updates and requests cannot resume radio support or report publication. Its
navigation and seeker histories restart through their existing reset boundary.
This does not change Vehicle.reset's authority/task/C2 ownership contracts or
replace the generic full-episode generation reset.

Onboard feedback contains its own navigation estimate and uncertainty, measured
tracking/support/activation state and optional onboard timing predictions.
Navigation has its own state epoch and full covariance. Current authored cards
select `strapdown_body_ecef.v1`: the [body-IMU strapdown estimator](weapon-navigation-strapdown-v2.md)
uses captured native motion to generate finite-cadence noisy inertial samples,
integrates estimated attitude, velocity and position, and applies optional noisy
GNSS aiding. Only AIM-120D enables GNSS aiding here. The retained resolved-ECEF
`NavigationV1` model remains the compatibility path when a card does not select
the strapdown model. Both native guidance and delivered feedback consume the
selected onboard estimate. Sensor scene formation separately uses physical
geometry; navigation errors cannot move the actual target. These are disclosed
engineering mechanisms, not calibrated reconstructions of proprietary IMU,
attitude or GNSS hardware.
There is no true target ID, referee holding,
damage, crew loss or World engagement event in the return payload. Missing
numbers remain optional in C++/Python and have explicit availability flags in C.

## Public use and compatibility

C++ constructs `WeaponLinksV1(agent)` explicitly. Python constructs
`fsim.WeaponLinksV1(agent)`. C99 includes `fsim/fsim_weapon_link.h` and calls the
versioned functions on the existing permission-scoped Agent handle. An instance
token is mandatory for designation/request/cancel mutations. Caller-supplied
sender identities and controllers are not accepted.

```python
links = fsim.WeaponLinksV1(agent)
target = links.designate_fused(agent.fused_tracks_v1()[0].id)
# Release still goes through the existing Agent strike and delivered C2 consent.
capability = links.capability(released_weapon)
receipt = links.assess(released_weapon, request_id=1)
if receipt.accepted:
    reports = links.feedback(released_weapon, receipt.assessment)
```

All reads are non-consuming. C count-only and short value reads return the total
required count. `struct_size` is the array stride; old public structs and enums
are untouched. C++ facades retain a closed control block and refuse calls after
their native World is destroyed. Python's Agent handle retains the native World
under existing lifetime rules. Mission generation/busy guards remain the
generic gateway owner's boundary.

The facade deliberately adds no methods to Python `Agent`, whose default strict
actor callable list is generated by reflection. Mission actors still receive
only the existing endpoint's observation/submit/receipt methods. A generic
gateway integration requires a separately reviewed versioned extension; no
existing registry is relabeled installed by this feature.

`from fsim.weapon_link_process import WeaponLinkProcessV1` explicitly loads the optional host.
`WeaponLinkProcessV1(world, agent, "module:Class")` is an explicit optional actor
host. It reuses the established HMAC connection check, JSON codec and lockstep
process loop, with a separate eight-method callable list. The policy receives
only `RemoteWeaponLinksV1`. The host verifies that its supplied Range World is
the same native instance as the original bound Agent, refuses publishing and
recording, and permits no generic Agent/activity/World/referee call on that
channel. It preserves the original release controller rather than minting an
Agent during proxy construction. The generic default actor list remains
unchanged. Operating-system file/process access remains outside this SDK
interface contract, as in the existing isolation API.

The V3 World measurement seam uses target body orientation for infrared source
aspect, separately from target velocity, and uses actual seeker/illuminator
frequency for RCS lookup. SARH currently uses the admitted monostatic target RCS
as an explicit bistatic engineering proxy; it does not claim measured bistatic
scattering. Actual antenna face/beam steering and both physical propagation
legs remain in the native scene formation. Delayed scene attribution is kept
only in bounded World histories and never enters a sensor or actor payload.

## Verification record

The initial normal Headless compilation fails on the missing private
`weapon/WeaponLink.h` as intended, before implementation. The root retained
`build/architecture-reviews/sim-m03-complete-20261102/link-red-build.log`.
Current implementation and public tests require root-controlled compilation,
focused execution, installed-consumer execution and full integration before a
passing delivery can be asserted.
