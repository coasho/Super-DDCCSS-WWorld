# Missile card public-data matrix and per-model component baseline

Date: 2026-10-23. Baseline: main `004e1f7`, branch `opencode/sim-m03`.

This record covers the identity and admitted provenance of the 24 air-to-air
missiles the store cards describe, and the first per-model correction pass made
from public sources. It is not a physical or classified-data calibration. Every
number is either tied to a public source recorded here and in the card, or
carried by the card as an explicit engineering estimate with its method and
uncertainty.

## Why the component parameters were crude

The 24 cards already declared the right *kinds* of components, but their numeric
component parameters were largely one class-default set:

* every high off-boresight missile carried the same 45 degree gimbal as a
  legacy reticle missile;
* the pre-release seeker field was 25 degrees on missiles published at 90;
* seeker reach, fuze radius and arming, guidance gains and actuator lag were
  copied per class rather than per type;
* one card (R-60M) declared a 13 km seeker reach for an 8 km missile.

The facts gate below makes those drifts visible; the applied corrections fix
the identity-level errors and leave the genuinely unknown values labelled.

## Identity matrix (factual component types)

| Card | Seeker | Warhead | Fuze | Motor |
| --- | --- | --- | --- | --- |
| AIM-120C-7 | active radar | blast-fragmentation | TDD + FZU-49 safe/arm, proximity/contact | solid, booster only |
| AIM-120D | active radar | blast-fragmentation | FZU-49 proximity/contact | solid, booster only |
| AIM-7M | semi-active radar | blast-fragmentation | proximity/contact | solid |
| AIM-9M | non-imaging infrared | annular blast-fragmentation | active optical target detector | solid |
| AIM-9X Block I | imaging infrared | annular blast-fragmentation | active optical target detector | solid |
| ASRAAM | imaging infrared | blast-fragmentation | laser proximity/contact | dual-burn solid |
| IRIS-T | imaging infrared | dual-layer HE fragmentation | active radar proximity/contact | thrust-vector solid |
| Magic 2 | infrared | pre-fragmented | RF proximity/contact | solid |
| Meteor | active radar | blast-fragmentation | proximity/contact | throttleable ducted rocket |
| MICA EM | active radar | blast-fragmentation | proximity/contact | solid |
| MICA IR | imaging infrared | blast-fragmentation | proximity/contact | solid |
| PL-8 | infrared | blast-fragmentation | proximity/contact | solid |
| PL-10 | imaging infrared | blast-fragmentation | laser proximity | solid |
| PL-12 | active radar | blast-fragmentation | proximity/contact | dual-thrust solid |
| PL-15 | active radar | fragmentation | proximity/contact | dual-pulse solid |
| R-27ER | semi-active radar | continuous rod | radar proximity/contact | dual-mode solid |
| R-27ET | infrared | continuous rod | radar proximity/contact | dual-mode solid |
| R-27R | semi-active radar | continuous rod | radar proximity/contact | solid |
| R-27T | infrared | continuous rod | radar proximity/contact | solid |
| R-37M | active radar | fragmentation | proximity/contact | dual-pulse solid |
| R-60M | infrared | continuous rod | proximity/contact | solid |
| R-73 | infrared | rod | proximity/contact | solid |
| R-77M | active radar | rod | laser proximity | dual-pulse solid |
| Super 530D | semi-active radar | blast-fragmentation | RF proximity/contact | solid |

The table records the cards' claimed equipment. Where a public source is
ambiguous (for example R-77M's warhead construction) the card's existing entry
is retained rather than replaced by a guess.

## Published corrections applied

Only values a public source states are written as `basis: published`. The
fact table (`tools/validation/payload_models/missile_facts.json`) records the
value, uncertainty, source id and a note; the source record is copied into the
card's own `sources` object.

| Card | Parameter | Before | After | Source |
| --- | --- | ---: | ---: | --- |
| AIM-9X Block I | seeker gimbal, launch field | 45, 25 deg | **90, 90 deg** | `odin-aim9` (U.S. Army ODIN: up to 90 deg off-boresight); `navair-aim9x` (service description) |
| ASRAAM | seeker gimbal, launch field | 45, 25 deg | **90, 90 deg** | `hushkit-sraam` (secondary reporting of +/-90 deg) |
| IRIS-T | seeker gimbal, launch field | 45, 25 deg | **90, 90 deg** | `deagel-iris-t` (secondary; Saab publishes type and dimensions) |
| PL-10 | seeker gimbal, launch field | 45, 25 deg | **90, 90 deg** | `twz-china-aam` (secondary, "reportedly") |
| R-73 | seeker gimbal, launch field | 45, 25 deg | **45, 45 deg** | `roe-r73e` (Rosoboronexport: plus/minus 45 deg; the gimbal was already 45 but only as an assumption) |

A launch field narrower than the published gimbal is now rejected by the gate:
a missile that can be cued to 90 degrees must retain that field before release.

One derived correction replaces an inconsistent assumption, and one legacy
estimate is narrowed to the missile's real generation:

| Card | Parameter | Before | After | Method |
| --- | --- | ---: | ---: | --- |
| R-60M | seeker reach | 13 000 m | **6 000 m** | 0.75 x the card's published 8 km maximum launch range; basis `derived`, source `wp-r60`, uncertainty 1 500 m |
| R-60M | seeker gimbal | 45 deg | **20 deg** | The R-60's OGS-60TI seeker is published at 12 deg; the R-60M's Komar-M limit is unpublished, so 20 deg is an engineering estimate (basis `assumed`, uncertainty 8 deg) |

## Values that remain engineering estimates

No public source found for these, so they keep `basis: assumed` (or `derived`
where a published range is the only anchor) and their method note:

* seeker reach for every missile except the R-60M correction: a launch-range
  fraction by seeker class, not a measured acquisition range;
* fuze detection radius, arming time and delay: the existing generic
  lethality-radius rule, not a published TDD/fuze sensitivity;
* guidance navigation gain, activation ratio, cue cone, loft, g-max and
  battery life: class-typical values, not manufacturer software data;
* actuator lag, slew rate and control-loop time constants: generic plant
  assumptions;
* warhead fill mass, Gurney velocity, fragment distribution and target
  vulnerability: only publicly bounded, otherwise labelled estimates.

These are exactly the inputs the physical-acceptance matrix in
[sim-m03-reference-continuation.md](sim-m03-reference-continuation.md) still
lists as missing. Labelling them is not closing them.

## Sources

| Source id | Kind | Title and publisher |
| --- | --- | --- |
| `navair-aim9x` | government publication | AIM-9X Sidewinder, U.S. Navy NAVAIR |
| `odin-aim9` | government publication | AIM-9 Sidewinder, U.S. Army ODIN Worldwide Equipment Guide |
| `roe-r73e` | manufacturer publication | R-73E, Rosoboronexport |
| `mbda-mica` | manufacturer publication | MICA family, MBDA (identity and dimensions) |
| `saab-iris-t` | manufacturer publication | IRIS-T, Saab (identity and dimensions) |
| `deagel-iris-t` | secondary reference | IRIS-T, Deagel (reported look angle) |
| `hushkit-sraam` | secondary reference | Short-range missile survey, Hush-Kit |
| `twz-china-aam` | secondary reference | China air-to-air missile inventory, The War Zone |
| `weaponsystems-r60` | secondary reference | Molniya R-60, Weaponsystems.net (12 deg seeker angle, rod warhead) |

URLs and access dates are in the fact table. Secondary references are admitted
for identity and order-of-magnitude capability only; no classified or
export-controlled figure is used or implied.

## Verification

```powershell
D:/ENV/python/python.exe -B tools/validation/payload_models/check_missile_facts.py --repo .
D:/ENV/python/python.exe -B tools/validation/payload_models/apply_missile_facts.py --repo . --check
```

`missile_card_facts` and `missile_card_facts_apply_check` run both as CTest
cases. They check:

* the AAM card set equals the fact-table set (no unlisted missile card);
* seeker mode, warhead kind, fuze arrangement and motor type are declared and
  agree with the reviewed entry;
* every published fact is present on the card with the same value, basis and
  copied source record;
* every component role exists, every `parameter_refs` entry resolves to a real
  parameter, and availability is a known state;
* every published parameter names a source the card carries;
* physical bounds: positive ranges, gimbal within (0, 180], seeker reach not
  grossly beyond the kinematic range, explosive below warhead mass, warhead
  below launch mass.

The gate is green on all 24 cards. It does not test any engagement outcome and
does not promote an estimate to published data.

Focused execution on 2026-10-23: the new gates plus the affected native cases
(`missile_card_facts`, `missile_card_facts_apply_check`, `payload_providers`,
`payload_native`, `payload_components`, `representative_weapon_native`,
`weapon_guidance`, `weapon_employment_core`, `contact_fuze_native`, and the
store launch-policy case) pass **10/10**, zero skipped. The showcase and
engagement selection (`showcase_hit_matching`, `showcase_native`,
`showcase_event_chain`, `engagement_native`) passes **4/4**.

The corrected fields invalidated four test fixtures that had encoded the old
45/25 degree field as a premise. Each was repaired without weakening its
assertion or geometry:

| Fixture | Why it changed | What changed |
| --- | --- | --- |
| `weapon_employment_core`, `selected atomic` and `release fov loss` | AIM-9X at 40 degrees is now inside its published 90 degree field | the infrared store is the 20 degree R-60M; the 40 degree geometry and every assertion are unchanged |
| `test_stores.cpp`, launch-policy case | AIM-9X at 40 degrees is no longer an FOV refusal | the same R-60M substitution |
| `test_contact_fuze_native.cpp` | its lost-seeker guard places the target outside 45 degrees | the same R-60M substitution; the guard, placement and contact assertions are unchanged |
| `test_payload_composition_contract.cpp` | its legacy baseline read only legacy `seeker_*` keys, which the migrated cards do not carry | the baseline falls back to the card's `component_*` parameter, so it compares composition with the card's declared value instead of a hard-coded class default |

Full Release suite, `004e1f7` plus this work, 911 cases: five failures were
found on the unmodified tree (verified by stashing this branch's changes and
re-running the same cases) and four are now repaired:

* `package_consumer`: the vendored JSBSim tree had no `COPYING`, so the install
  rule silently skipped `LICENSE-JSBSim.txt`. The LGPL-2.1 text is now retained
  at `third_party/jsbsim/COPYING` (upstream v1.3.1, hash recorded in
  `THIRD_PARTY_NOTICES.md`) and the consumer installs it;
* `hangar test_stores.StoresTest`, `hangar test_missiles.Missiles`: the fifteen
  guided stores' committed JSBSim XML differed from the current generator
  (`-0.25000000000000006` versus `-0.25`). The XMLs were regenerated;
* `hangar test_damage.Current`: the `c172`/`skua` damage data and the
  aerodynamic-model argument the gate passes now match the model stage;
* the Python package now stages NVIDIA Flow's `nvflow.dll` and `nvflowext.dll`,
  which the vision renderer opens at run time and the package omitted.

`testing_scope_checker` passes after the Headless preset is rebuilt on this
tree. `sdk_boundary` still fails on this host: `import fsim.vision` from
`build/ucrt64-release/python/fsim/bin` fails with WinError 1114 while the
byte-identical DLL set loads from `build/ucrt64-release/bin`; the SDK vision
test skips under the same condition. It is recorded as an unresolved
runtime/environment issue, not masked by changing the check.

## Remaining work

The next increment replaces the class-generic detection and lethality
algorithms with sourced physics:

1. active/semi-active radar range equation with per-type frequency, aperture,
   power, noise and processing loss;
2. infrared detection from target intensity and seeker noise-equivalent
   irradiance (Johnson-criterion style) instead of the current linear aspect
   blend;
3. Gurney/Mott fragment field with public case/fill bounds and target
   penetration;
4. proximity-fuze link budget and arming-distance derivation;
5. motor internal ballistics from published burn time and total impulse.

Each needs its own frozen reference cases before it can be called calibrated.
Until then the matrix above is the admitted boundary: identity facts and
published corrections are verified, physical performance is not.
