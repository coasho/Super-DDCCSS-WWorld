# Versioned finite side-case warhead model

## Scope and source basis

`damage::warheadOf` and `damage::assess` support an explicitly configured
finite cylindrical side-case source. Version 1 is a generic analytical
approximation, not a calibrated WDU-41/B fragment distribution or a measured
fragment-size law. It adds no fitted spray width, zone weights, fuze delay,
Taylor correction or Mott parameter. Aircraft damage and real native outcome
acceptance remain separate verification requirements.

The [official DLA catalog for MIL-HDBK-1211](https://quicksearch.dla.mil/qsDocDetails.aspx?ident_number=116871)
identifies the 17 July 1995 base handbook. Section 2-2.4.1.3 and Figure 2-19,
printed pages 2-16 to 2-18, describe radial fragments approximately perpendicular
to the missile axis and the dynamic velocity construction: add missile velocity,
then subtract target velocity. The original handbook is also available as a
[primary-document mirror](https://citeseerx.ist.psu.edu/document?doi=f62ec0b260506d92aec61b0d14b5f1f818dfd276&repid=rep1&type=pdf).
These principles support a consistent generic source; they do not determine
a modern missile's angular probability density or equipment effectiveness.

The existing cylinder Gurney speed is retained. A primary general review is
J. E. Kennedy, Sandia report SC-RR-70-790, December 1970,
[Gurney Energy of Explosives: Estimation of the Velocity and Impulse Imparted to Driven Metal](https://www.osti.gov/servlets/purl/4441869).
The finite volume, uniform sampling and equivalent-cube construction below are
declared simulator assumptions. Their algebra conserves the configured mass;
it does not infer actual casing construction from a public assembly diagram.

## Required card inputs

All inputs are scalar `StoreCard::parameters`. Version 1 requires
`warhead_kind="fragmentation"`; a rod, unknown version/source, absent parameter,
nonfinite value or invalid geometry returns unavailable (`nullopt`). No missing
V1 parameter falls back to the legacy constants or angular zones.
Required unit strings are exactly `1` for version/source/fractions/drag,
`kg` for masses, `m` for geometry/stations, `km/s` for Gurney speed and
`kg/m^3` for densities. Missing or mismatched units are unavailable; values
are never silently reinterpreted or converted under a wrong declaration.

| Parameter | Unit / accepted value | Meaning |
| --- | --- | --- |
| `warhead_model_version` | 1 | Complete versioned configuration |
| `warhead_fragment_source` | 1 | Finite cylindrical side-case, radial local-normal ejection |
| `warhead_kg` | kg, positive | Complete warhead mass |
| `explosive_kg` | kg, strictly between zero and complete mass | Modeled charge mass |
| `diameter_m`, `length_m` | m, positive | Missile envelope used to validate the source |
| `gurney_kms` | km/s, positive | Explosive Gurney speed |
| `warhead_tnt_equivalent_kg` | kg, positive | Independently declared blast equivalent, with explicit basis and uncertainty |
| `warhead_structure_fraction` | dimensionless, `[0,1)` | Noncharge mass reserved for structure/end plates/safe-arm hardware |
| `warhead_diameter_ratio` | dimensionless, `(0,1]` | Side-case outer diameter divided by missile diameter |
| `warhead_case_density_kg_m3` | kg/m³, positive | Side-case material density |
| `warhead_explosive_density_kg_m3` | kg/m³, positive | Modeled charge density |
| `warhead_structure_density_kg_m3` | kg/m³, positive | Reserved structure's effective density |
| `warhead_fragment_cd` | dimensionless, positive | Existing straight-flight fragment-drag model's coefficient |
| `warhead_station_x_m` | m aft of authored nose | Fixed source centre in structural axes |
| `warhead_station_y_m` | m right of authored nose | Fixed lateral station |
| `warhead_station_z_m` | m up from authored nose | Fixed vertical station |

The whole axial source interval must lie within `[0,length_m]` from the nose;
the station's lateral/vertical offset plus outer radius must fit within half
`diameter_m`. Invalid fits are rejected rather than clamped. This is a finite
cylindrical envelope check, not a proof of clearance inside a tapered authored
missile body. Card sources, basis and uncertainties must state what is published,
derived or assumed. A family illustration's assembly midpoint is not a measured
variant-specific explosive centroid.

`Warhead::stationStructuralM` exposes the optional fixed centre to World.
It is available only for a complete valid V1 configuration. World transforms
the structural nose-relative station using the actual authored nose and fixed
model origin, current captured CG and attitude. The common burst velocity must
be the same-epoch velocity of that point, including its offset from the CG and
native angular rate. The contact nose sensor and warhead centre are distinct.
`Burst::at` remains the common centre for blast; it is not silently moved to the
contact tip or separately randomized for blast.

## Mass budget and geometry

For complete mass `W`, charge `C`, structure fraction `f`, densities
`rho_c`, `rho_e`, `rho_s`, and configured outer radius `Ro`:

```text
Mcase = (1-f) (W-C)
Mstructure = f (W-C)
V = C/rho_e + Mcase/rho_c + Mstructure/rho_s
L = V / (pi Ro^2)
Ri^2 = Ro^2 - Mcase/(rho_c pi L)
t = (Ro^2-Ri^2)/(Ro+Ri)
```

The exact annulus has mass `rho_c*pi*L*(Ro²-Ri²)=Mcase`. The structure volume
shares the inner region with the charge in this generic construction; it is
not emitted as side-case fragments. The former thin-wall thickness expression
is retained only for unversioned legacy cards.

The nominal equivalent cube has side `t`. Set
`N=round(Mcase/(rho_c*t³))`, then set actual fragment mass to `Mcase/N` and
mean presented area to `1.5*(fragment_mass/rho_c)^(2/3)`. Thus integerizing `N`
does not create or lose side-case mass. Zero, nonfinite or unrepresentable counts
are unavailable. Equal cubes and this thickness-derived count are assumptions;
they do not claim a measured preformed-fragment count or Mott distribution.

Initial radial speed is the established cylinder expression
`gurney_speed/sqrt(Mcase/C+1/2)`. Version 1 takes blast equivalence solely from
the required `warhead_tnt_equivalent_kg`; fragment-driving Gurney energy cannot
determine blast equivalence. A card retaining the former charge-mass times
squared Gurney-speed ratio must identify it as an uncalibrated blast proxy,
not measured equipment TNT equivalence. Only the legacy branch still derives
that proxy automatically against its established 2440 m/s reference.

## Sampling, motion and torque

Every fragment consumes exactly three uniforms, in this order:

```text
axial = L*(U1-1/2)
azimuth = 2*pi*U2
radius = sqrt(Ri^2 + U3*(Ro^2-Ri^2))
radial = e1*cos(azimuth) + e2*sin(azimuth)
source_offset = nose*axial + radial*radius
origin = burst_centre + source_offset
velocity_relative_to_target = burst_point_velocity + cross(angular_velocity_ecef, source_offset)
                              + radial*Gurney_speed - target_velocity
```

`e1,e2` are the maintained perpendicular basis around the normalized nose.
Uniform `r²` samples uniform casing volume rather than uniform radius. Position
and velocity use the same radial direction. There are no V1 angular zone
weights; modifying the legacy `zones` vector cannot redirect V1 fragments.
`Burst::angularVelocityEcef` is the same-epoch native rigid-body angular
velocity in ECEF radians per second. Its default is zero. The cross product
uses the finite source offset from the common centre, so common-point motion
and individual case-point rotation are each included once. The legacy point
source ignores this added field.

The existing intersection, penetration, fragment stopping and blast algorithms
remain in use. Each fragment's intersections start at its own body-frame source
point. Only transferred target momentum contributes to target impulse; angular
impulse uses the actual ray origin crossed with that transferred momentum.
The common centre still defines blast and diagnostic miss distance. The existing
straight-line drag approximation, frozen target attitude during fragment flight,
equal-cube fragmentation and absent axial end effects remain model limitations.

Cards without `warhead_model_version` and without any V1 `warhead_` parameter
other than the established legacy `warhead_kg` preserve the existing point source,
constants, zone distribution, three-draw ordering and legacy fields. Explicit
unsupported versions and orphan versioned fields cannot select that fallback.

## Independent verification

`tests/simulation/test_warhead_model.cpp`, selector `[warhead-model]`, supplies
synthetic analytical inputs rather than fitted encounter snapshots. Before
production changes, root recorded **1/5 cases passing, 52/64 assertions failing**
in `build/architecture-reviews/sim-m03-warhead-red.log`; the unchanged legacy
case passed. Cases cover strict configuration/source fit, exact annular mass,
integer fragment mass, an independently reconstructed finite radial ray,
per-origin torque, zero-air-drag Galilean relative mechanics and exact RNG draw
count. Post-implementation GREEN, native scenes, card provenance and continued
physical damage verification are pending until recorded by their owning gates.

The follow-up `[warhead-boundary]` fixtures first recorded **1/3 cases passing,
43/53 assertions failing** in `build/architecture-reviews/sim-m03-warhead-boundary-red.log`.
They independently expose wrong/missing unit acceptance and orphan-namespace
fallback, and cover overflowing/collapsed geometry plus a valid zero-structure
control. The legacy fixture now explicitly removes the entire V1 namespace;
it does not retain orphan source/station fields to bypass the new boundary.

The additional `[warhead-spin]` mechanism fixture independently checks the
finite point's zero/nonzero-spin ray, kinetic energy, impulse and torque.
`[warhead-blast]` checks that changing Gurney speed changes fragment speed but
not declared blast equivalence, and changing blast equivalence leaves fragment
speed fixed. The legacy fixture checks that the new angular field is ignored.
These additional fixtures await the combined native GREEN gate.

The later root-owned [scoped regression](sim-m03-five-km-repair.md#current-scoped-regression-evidence)
records all synthetic V1 mechanism/boundary/spin/blast cases passing in both
presets. Current physical-model acceptance remains failed: the unchanged
showcase has zero sampled intersections, and the 6 m side-burst fixture has no
component hits after its existing shielding/penetration approximation. Neither
failure is bypassed by a Hit or by contact-induced structural loss. The source
station, scatter, charge, shielding and scene values are not fitted to these
results. Passing mass/geometry/RNG algebra does not establish equipment fidelity.
