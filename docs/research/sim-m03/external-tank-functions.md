# External tank native fuel functions

This population covers the existing fuel functions of all 19 maintained external
tank cards. Seeker, fuze and warhead N/A entries do not admit these functions.
The 32-airframe/149-internal-tank liquid products describe a different population.
This work does not establish carried/free-store fragment puncture vulnerability,
OEM suspension hardware compatibility, calibrated pump performance, or SIM-M10
store/task/MDF completion.

The native fixture independently enumerates the actual store library, selects
the 19 `tank` cards, and resolves each real authored loadout station. Eighteen
cards occur in current authored fits. PTB-1150 has no authored fit: it uses the
existing F-16C station 4 class/maximum-mass admission as an explicitly generic
diagnostic mount. This is a software composition fixture, not an assertion that
Soviet/NATO suspension hardware or the real aircraft supports that combination.
No card, fit, TOML, generated XML or production behavior is changed.

Each model receives wet and empty native cases. The World loads the real store
on its actual station, applies normal fuel pumping and releases it through an
emergency jettison. The fixed-boundary component observation calls the existing
SDK-owned `World::transferFuel`, which invokes `StoreFlightBinding::transferFuel`
with the actual native Host/flight model and `WorldOptions::dt`. It uses
`ceil(requested interval / native dt)` nominal transfer calls, bounded by 100000,
without stepping physics or advancing the World clock. The Host remains a const
borrowed observation; no mutable facade or const cast is introduced. Engine
consumption therefore cannot conceal transfer losses. The requested wet
interval is 1 s. The empty-control interval is `card fuel kg / 10 + 1 s`, with independently
checked receiving capacity; the existing estimated 10 kg/s law therefore moves
all remaining external fuel into the real internal tanks. This is a component
interval, not an assertion that the World clock advanced by that duration.
Empty fuel is never manufactured by deleting a point mass or retuning a card.

Release mass and first moments are measured in the unchanged native structural
frame. The fixture reads JSBSim's actual carrier mass/CG, each internal tank's
contents and actual contents-dependent `FGTank::GetXYZ` structural location,
and the actual loaded station point-mass
location. Subtracting internal-fuel mass and first-moment changes isolates the
removed store despite concurrent pumping/engine consumption. The mass budget is
`1e-6 * max(1, before mass, after mass)` kg and the first-moment budget is
`2e-6 * max(1, before mass * |CG|, after mass * |CG|)` kg m. These numerical
budgets are declared before execution. They do not waive a stale native cache or
move an assertion to a later favorable outcome.

The released native model retains the actual Host fuel quantity in its existing
`fuel` point mass. Independent admission parses the actual loaded/current
generated XML and compares dry mass, fuel-point location and composite CG.
Wet/empty native mass and first moments are also compared directly with a
`1e-7` scale budget. Both bodies then follow native flight/terrain contact from
the predeclared hard-impact carrier conditions of 120 m/180 m/s/pitch -60 deg.
Actual released native downspeed must exceed 100 m/s before any terminal
outcome is observed. Within 20 s the
wet tank must produce one positive-energy Impact and one fuel-bounded Fire; the
empty control must produce the Impact and zero fuel Fire. Both jettisons remain
inert, with zero Detonation and zero continuing pool burn after store removal.
The original F-16 three-tank/two-bomb impact acceptance and its existing burn
thresholds remain unchanged and independently required.

The original new level-drop fixture failed on the empty A-10 600-gallon shell:
after 20.0333 s it rested at 0.243618 m AGL with speed 4.77e-5 m/s, without a
native crash, Impact or Fire, and its native weapon record remained InFlight.
The wet case struck and burned. This is retained as a failed destructive-contact
premise, not relabeled a passing level-drop gate. The separate hard-impact pair
preserves all strict terminal/mass/CG predicates and the 20 s bound; no native
spring, damping, aerodynamic or impact rule is changed. Grounded noncrashed
Role::Weapon lifetime is a distinct existing limitation outside this fuel
function closure. A gentle contact is never manufactured into an Impact.

`FSIM_EXTERNAL_TANK_POPULATION_DIR` selects raw diagnostic output only. The
native test writes actual process binary/loaded JSBSim module paths and hashes,
one complete wet/empty row per real card, and a marker after the complete row
population is written. Nonfatal Catch checks can fail before that marker, so a
separately passing exact native report is independently mandatory. Interrupted,
partial or failed-test output cannot be admitted. No loaded
`libfsim.dll` is inferred: the session may be statically linked into the test.

`external_tanks.py` independently re-enumerates cards, binds current authored
fits/payloads, parses current XML, recomputes every fuel/mass/first-moment
predicate, and requires the exact nonzero native case plus all 19 unique rows
in both maintained presets. Frozen source epochs, actual module/binary bytes,
raw evidence and reports are rechecked during final admission. Its synthetic
negative tests are admission-method evidence only.

The first scoped hard-impact runs passed in both maintained presets: one
Catch case and 4,422 assertions per preset, with all 19 wet/empty pairs. The
unchanged original jettisoned-tank/bomb impact case separately passed all
18 assertions. Raw first-run reports and the earlier level-drop failure
remain under `external-tanks` in the dated production evidence root. These
scoped observations do not substitute for the final fresh campaign.

The mandatory campaign command `native-external-tanks` freezes its current
source epoch after normal production and before both full regressions. It
binds both exact reports and raw populations. Final report admission verifies
the epoch path/hash, independently re-admits current bytes and adds the joined
receipt to every tank's existing applicability gate and the top-level binding.
The dedicated mandatory join supplies its native/method evidence; it does not
rename the inherited inert-store gate IDs. Existing
authored-fit and installed-public proofs remain independently required. Root
owns final shared registration and whole-program acceptance; complete visual,
installed and full-regression gates are not implied by this scoped result.
