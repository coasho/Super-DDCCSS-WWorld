# SIM-M10: Guns, Loaded-Store RCS & Air-to-Ground Ballistic Validation — Public Literature & Open-Source Alignment

This note records the authoritative public engineering literature, open-source
references, physical derivations, and honest uncertainty boundaries governing
**SIM-M10** (`Family::Gun` across all 11 maintained gun cards, B-33 ammunition
mass conservation, B-37 loaded-store RCS contribution, and B-50 Mk 82 free-fall
bomb ballistic validation).

## 1. Authoritative Public Literature & References

| Subsystem / Backlog Item | Authoritative Reference | Physical Model & Scope Applied in SIM-M10 |
| --- | --- | --- |
| **Exterior Ballistics & Projectile Drag** (`src/simulation/mission_systems/gun/Gun.{h,cpp}`, `World.cpp`) | R. L. McCoy, *Modern Exterior Ballistics* (Schiffer, 2nd ed., 2012); NATO STANAG 4355 (Modified Point Mass Trajectory Model); ICAO Doc 7488 ISA | Quadratic atmospheric drag $\frac{d\mathbf{v}}{dt} = -k \frac{\rho(h)}{\rho_0} |\mathbf{v}|\mathbf{v} + \mathbf{g}$ with $k = \frac{\rho_0 C_D A}{2 m}$ derived from published projectile caliber, mass, and supersonic-averaged G7 drag coefficient ($C_D \approx 0.28\text{--}0.34$), plus Gaussian muzzle dispersion ($\sigma_{\text{disp}} \approx 1.5\text{--}3.0\text{ mrad}$). |
| **Aircraft Cannon Specifications (11/11 Cards)** | USAF / General Dynamics OTS (M61A1/M61A2, GAU-8/A, GAU-22/A), Rheinmetall (BK-27), Nexter/GIAT (DEFA 554, 30M791), KBP (GSh-23, GSh-30-1, GSh-30-2), US Army TM 9-1005-224-10 (M60D) | Published caliber, cyclic rate of fire, muzzle velocity, projectile mass, cartridge mass, explosive fill (HEI vs. solid ball), and magazine capacities across all 17 gun-equipped airframes. |
| **Projectile Penetration, HEI Effects & Survivability** | Project THOR (BRL TR-47, 1961); Kingery-Bulmash / Swisdak (1994); R. E. Ball, *The Fundamentals of Aircraft Combat Survivability* (AIAA, 2nd ed.) | Swept 3-D projectile-segment intersection against target aircraft damage bounding/component geometry; kinetic energy & Project THOR aluminum skin penetration (`ms::damage::thorResidual`), plus HEI overpressure when `explosiveKg > 0`, applied exclusively through native `VehicleDamage::apply` and `EngagementEvent`. |
| **Ammunition Mass Conservation (B-33)** | JSBSim Point Mass dynamics (`inertia/pointmass-weight-lbs`) | Each fired round sheds projectile + propellant/expended mass via `Model::takeShedKg()`, wired through `StoreFlightBinding` directly onto each airframe's authored `"gun ammunition"` JSBSim point mass (clamped $\ge 0$). |
| **Loaded-Store RCS Contribution (B-37)** | E. F. Knott, J. F. Shaeffer & M. T. Tuley, *Radar Cross Section* (SciTech, 2nd ed., 2004); G. T. Ruck et al., *Radar Cross Section Handbook* (Plenum, 1970) | Incoherent power sum $\sigma_{\text{total}} = \sigma_{\text{airframe}} + \sum_i \sigma_{\text{store}, i}$ over externally carried stores (and internal-bay stores only while bay doors are open). Uses precomputed `rcs.bin` tables where available (e.g. `aircraft/mk82/rcs.bin`), and otherwise physical-optics cylinder + nose-ogive + pylon dihedral approximation derived from each store card's published `length_m`, `diameter_m`, and `storeClass`. |
| **Mk 82 Free-Fall Bomb Ballistics (B-50)** | US Navy NAVAIR 00-110-1 / USAF TO 1F-16C-34-1-1 ballistic characteristics; Los Alamos LA-UR-89-1936 (`87 kg` Tritonal); Hangar shared VLM (`assets/stores/mk82.json`) | Independent 4th-order Runge-Kutta (RK4) ISA drag-gravity trajectory solver cross-checked against live 6-DOF JSBSim `mk82` release from `F-16C` across release altitudes, speeds, and crosswind conditions. |

## 2. Gun Card Parameter Table (All 11 Maintained `Family::Gun` Cards)

| Card | Designation | Calibre (mm) | Rate (rpm / gun) | Muzzle (m/s) | Proj Mass (kg) | Shed Mass (kg/rd) | HEI Fill (kg) | Default Cap (rds) | Basis |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `m61a1` | M61A1 Vulcan | 20.0 | 6,000 | 1,050 | 0.101 | 0.155 | 0.011 | 500 | Published (cal/rof/v0/proj), derived shed |
| `m61a2` | M61A2 Vulcan | 20.0 | 6,000 | 1,050 | 0.101 | 0.155 | 0.011 | 480 | Published (cal/rof/v0/proj), derived shed |
| `gau-22a` | GAU-22/A | 25.0 | 3,300 | 1,040 | 0.184 | 0.285 | 0.032 | 182 | Published (cal/rof/v0/proj), derived shed |
| `gau-8a` | GAU-8/A Avenger | 30.0 | 3,900 | 1,010 | 0.360 | 0.495 | 0.058 | 1,174 | Published (cal/rof/v0/proj), derived shed |
| `bk-27` | Mauser BK-27 | 27.0 | 1,700 | 1,025 | 0.260 | 0.515 | 0.040 | 150 | Published (cal/rof/v0/proj/cartridge) |
| `defa-554` | DEFA 554 (×2 fit) | 30.0 | 1,800 | 815 | 0.244 | 0.420 | 0.048 | 125 / gun | Published (cal/rof/v0/proj/cartridge) |
| `giat-30m791` | GIAT 30M791 | 30.0 | 2,500 | 1,025 | 0.275 | 0.550 | 0.050 | 125 | Published (cal/rof/v0/proj/cartridge) |
| `gsh-23` | GSh-23 | 23.0 | 3,400 | 715 | 0.175 | 0.325 | 0.019 | 200 | Published (cal/rof/v0/proj/cartridge) |
| `gsh-30-1` | GSh-30-1 | 30.0 | 1,500 | 860 | 0.390 | 0.832 | 0.048 | 150 | Published (cal/rof/v0/proj/cartridge) |
| `gsh-30-2` | GSh-30-2 | 30.0 | 3,000 | 870 | 0.390 | 0.832 | 0.048 | 250 | Published (cal/rof/v0/proj/cartridge) |
| `m60d` | M60D (×2 door fit) | 7.62 | 550 | 853 | 0.0095 | 0.025 | 0.000 | 500 / gun | Published (NATO 7.62×51 mm ball) |

*Note on shed mass*: Linkless closed-loop rotary cannons (`m61a1`, `m61a2`,
`gau-22a`, `gau-8a`) retain spent brass casings onboard so shed mass equals
projectile + propellant (~`1.4–1.55×` projectile mass); conventional ejecting
cannons (`bk-27`, `defa-554`, `giat-30m791`, `gsh-*`, `m60d`) eject spent cases
overboard so shed mass equals full cartridge mass.

## 3. Honest Boundaries & Retained Backlog Scope

- **Closed / Verified in SIM-M10**:
  - `Family::Gun` execution (`TaskKind::Fire`, 11/11 gun cards), deterministic
    3-D ballistic projectile flyout with drag/gravity/dispersion, swept
    air-to-air hit detection & native `VehicleDamage::apply` integration, and
    B-33 `"gun ammunition"` JSBSim point-mass conservation.
  - B-37 loaded-store RCS addition in `World::radarCrossSection` (exact
    `rcs.bin` where present, physical-optics cylinder/ogive/pylon fallback
    otherwise, with strict internal-bay stealth shielding when bay doors are
    shut).
  - B-50 Mk 82 free-fall bomb ballistic trajectory cross-check against an
    independent ISA drag-gravity RK4 reference solver (altitude, airspeed,
    impact velocity, and crosswind drift).
- **Retained honestly as out-of-scope / separate future work**:
  - Ground targets, SAMs, ships, and ground emitters remain excluded by ADR-30 D7.
  - Per-store GPU ray-traced `rcs.bin` generation for all 53 stores (`flightsim-rcs`)
    and rotorcraft/cargo/viewer-only backlog items (`B-20..B-32`, `B-34`,
    `B-39`, `B-43..B-65`) remain explicitly tracked in [backlog.md](../../backlog.md).
