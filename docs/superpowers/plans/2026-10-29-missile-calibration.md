# Missile public-source calibration and component simulation

Baseline: `main` at `004e1f7`, with the existing uncommitted Hangar
calibration/aerodynamics work preserved. This plan defines the evidence and
execution standard for missile geometry, aerodynamics, propulsion, seekers,
warheads and fuzes. It does not promote an estimate to a measurement.

## Objective

Replace class-generic missile assumptions with the best public evidence
available, and implement component algorithms whose inputs and limits are
auditable. The first pilot covers four airframe/seeker classes:

| Pilot | Class | Why |
| --- | --- | --- |
| AIM-120C-7 | active radar, booster-only solid motor | current showcase missile; official geometry/mass; public Sparrow/AMRAAM leads |
| AIM-7M | semi-active radar, solid motor | NASA TP-1078 Sparrow III wind-tunnel data and DREV report lead |
| AIM-9M | non-imaging IR, canard control | NACA Sidewinder flight/wind-tunnel reports and ACC fact sheet |
| R-73 | imaging IR, canard + gas-dynamic control | manufacturer geometry and distinct control configuration |

The remaining 20 AAM cards follow after the pilot method and gates are
accepted. No card is changed merely to make an engagement pass.

## Evidence hierarchy

Every numeric parameter must carry one of these bases:

1. `published` - a public source states the value for the named variant, with
   a locator and test/configuration conditions.
2. `derived` - a documented calculation from published values; the formula and
   inputs are recorded.
3. `simulated` - a named tool and version (VLM, DATCOM-style method, CFD) run
   on the authored geometry; validation case and residuals are recorded.
4. `assumed` - only after a dated search finds no usable public value; the
   method, bounds and sensitivity are recorded.

A source is authoritative only for what it actually measures or states. A
generic missile report does not validate a named variant; a manufacturer
dimension does not supply drag; a CFD result is not a wind-tunnel measurement.

## Public-source register (first pass)

| Source | What it supports | Limits |
| --- | --- | --- |
| [NASA CR-2835](https://ntrs.nasa.gov/citations/19770021140), Nichols, 1977 | Compilation of declassified missile wind-tunnel data, Mach 0.2-4.63, body/fin/control configurations | Generic configurations, not named AIM/R-73 variants; digitized figures |
| [NASA CR-2836](https://ntrs.nasa.gov/citations/19770021141), Burkhalter, 1977 | Performance analysis using wind-tunnel data; unknown data estimated | Estimates are explicitly not measurements |
| [NASA TP-1078](https://ntrs.nasa.gov/citations/19780004098), Monta, 1977 | Supersonic aerodynamic characteristics of a Sparrow III type model with wing controls | Sparrow III type, not AIM-7M; model-scale conditions |
| [NASA TP-2157](https://ntrs.nasa.gov/citations/19830019688), Blair et al., 1983 | Canard-controlled missile tail-fin span, Mach 1.60-3.50 | Sidewinder-like generic model; no propulsion/seeker data |
| [NACA RM SL55C22](https://ntrs.nasa.gov/citations/20090023599), Nason et al., 1955 | Early Sidewinder roll stabilization, flight and estimated derivatives | Early Sidewinder, not AIM-9M; limited coefficients |
| [AIAA 3.62226](https://arc.aiaa.org/doi/10.2514/3.62226), Allen and Blair, 1982 | Analytical/experimental forward-control missile aerodynamics | Generic configuration; no named-variant data |
| [NATO STO-MP-AVT-307/316](https://publications.sto.nato.int/publications/STO%20Meeting%20Proceedings/STO-MP-AVT-307/MP-AVT-307-24.pdf) | CFD verification/validation practice for generic missile vortex flows | Method guidance, not equipment coefficients |
| [Missile DATCOM manuals](https://apps.dtic.mil/sti/pdfs/AD1000581.pdf) | Public semi-empirical method and input definitions | Code distribution is export-controlled; use methods/public data only |
| [US Navy AIM-120 fact sheet](https://www.navy.mil/Resources/Fact-Files/Display-FactFiles/Article/2168352/aim-120-advanced-medium-range-air-to-air-missile-amraam) | AIM-120C length, diameter, span, weight, propulsion/guidance class | Speed/range classified; no aero coefficients |
| [USAF AIM-7 fact sheet](https://www.af.mil/About-Us/Fact-Sheets/Display/Article/104575/aim-7-sparrow) | AIM-7M length, diameter, span, launch weight, motor, warhead class | Thrust, speed, range classified |
| [ACC AIM-9M fact sheet](https://www.acc.af.mil/Portals/92/Docs/Fact%20Sheets%20-%202020%20Update/AIM-9M_Formatted.pdf) | AIM-9M/X configuration, control and warhead classes | No numeric aero/propulsion |
| [MBDA Meteor datasheet](https://mbdainc.com/wp-content/uploads/2024/07/2018-METEOR-datasheet.pdf) | Meteor mass, length, diameter, propulsion/seeker/warhead classes | No thrust curve, drag or seeker sensitivity |
| [Rosoboronexport R-73E](http://roe.ru/eng/catalog/aerospace-systems/air-to-air-missile/r-73e/) | R-73E geometry, mass, warhead, seeker gimbal/rate | Export variant; no aero/propulsion curves |
| [AIM-120C-5 performance brief](https://www.generalstaff.org/BBOW/Progs/FlySIM/AIM120C5-Performance-Assessment-rev2.pdf) | Secondary study with CAD motor reconstruction and estimated burn/thrust | Not an authoritative measurement; useful only as a documented lead |
| [RAND RM-1572](https://www.rand.org/content/dam/rand/pubs/research_memoranda/2008/RM1572.pdf) | IR range equation and noise-equivalent input formulation | ICBM-era context; not an AIM-9M seeker |
| [NIST radio proximity fuze](https://nvlpubs.nist.gov/nistpubs/jres/37/jresv37n1p1_A1b.pdf) | Proximity-fuze principles, arming and radio-link context | Historical artillery/bomb fuzes, not missile TDD sensitivity |

## Known public-data gaps

The following are not public for most named AAMs and will not be presented as
measured:

- drag/lift/moment coefficient tables over Mach and angle of attack;
- motor thrust-time curve, propellant grain geometry and nozzle data;
- seeker antenna gain, transmitter power, receiver noise figure, detection
  threshold and tracking-loop bandwidth;
- IR seeker NEI, spectral band, target intensity and background clutter;
- warhead case/fill geometry, fragment-size distribution and target
  vulnerability;
- fuze radiation pattern, sensitivity, arming distance and ECM behaviour.

The public reports above can validate a method or a generic configuration.
They cannot certify a named variant. Where a named-variant value is absent, the
card will carry `derived` or `assumed` with the search date, method and
uncertainty, and the calibration report will state the residual risk.

## Execution phases

### Phase A: source and parameter audit

1. For each pilot, list geometry, mass, motor, seeker, warhead and fuze fields.
2. Record the exact source, locator, units, conditions and variant.
3. Search official manufacturer/government sources first; then NASA/DTIC/AIAA
   reports; then reputable secondary sources only as leads.
4. Mark every field `published`, `derived`, `simulated` or `assumed`.
5. Freeze the audit before changing a card or a runtime model.

### Phase B: geometry and mass calibration

1. Correct length, diameter, span, mass and component identity from official
   sources.
2. Keep the authored geometry and mass ledgers consistent; do not change a
   visual mesh to satisfy a card value.
3. Add a regression that rejects an unsourced change to a `published` field.

### Phase C: aerodynamics

1. Use public wind-tunnel data where it matches the authored configuration
   (Sparrow III type, Sidewinder-like canard model, generic fin sets).
2. Otherwise generate a documented `simulated` table from the authored
   geometry with the existing VLM, and validate the method against the public
   generic cases; report residuals and domain limits.
3. Do not call a VLM/CFD table a wind-tunnel measurement. Keep the existing
   `StoreAerodynamicsV1` binding and table contract.
4. Add holdout cases that never select coefficients.

### Phase D: component algorithms

Implement in this order, each with a frozen public validation case:

1. radar range equation for active/SARH seekers, with per-type frequency,
   aperture, power, noise and processing loss where public; otherwise explicit
   estimates;
2. IR detection from target intensity and seeker NEI instead of the current
   linear aspect blend;
3. Gurney/Mott fragment field with public case/fill bounds and penetration;
4. proximity-fuze link budget and arming-distance derivation;
5. motor internal ballistics from published burn time and total impulse.

Each algorithm keeps the existing public contracts and adds a versioned
provider or an additive configuration path; old cards and binaries keep their
behaviour.

### Phase E: validation and evidence

1. Unit tests for each formula and refusal path.
2. Public-reference comparisons for generic configurations; report absolute
   and relative residuals.
3. Native, installed C++/C99/Python consumer checks for any new public
   interface.
4. A written calibration report with exact commands, fingerprints and known
   gaps.

## Acceptance rules

- A `published` value has a public source and a variant match; a generic
  source is not relabelled as the named variant.
- A `derived` value shows its formula and published inputs.
- A `simulated` value names the tool, version, geometry input, validation case
  and residuals.
- An `assumed` value has a dated search record and a non-zero uncertainty;
  zero uncertainty never asserts measurement accuracy.
- No parameter is selected from an engagement outcome.
- No failed public comparison is hidden by relaxing a tolerance or changing
  the scene.
- Existing card values, scenes and acceptance assertions are preserved unless
  a public source proves them wrong; each change has its own regression.

## First write package

Pilot order: AIM-120C-7, AIM-7M, AIM-9M, R-73.

The first package stops at the source/parameter audit and the geometry/mass
corrections that have an exact public source. Aerodynamic and component
algorithm changes follow as separate reviewed packages after the audit is
frozen. The current working tree contains unrelated uncommitted calibration
work; this package does not overwrite or revert it.
