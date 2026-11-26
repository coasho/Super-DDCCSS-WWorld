# SIM-M05 (MS-9b): Public-Literature and Open-Source Alignment

This note records how SIM-M05's Electronic Attack (RF noise jamming), Enhanced ESM/DF moving-platform geolocation, COMINT datalink interception, and communications jamming models align with authoritative public EW/radar textbooks and mature open-source reference projects, and documents parameter provenance (`published` vs. `assumed`).

## 1. Reference Baseline and Open-Source Lineage

| Subsystem | Authoritative Public / Open-Source Reference | Alignment in `Super-DDCCSS-WWorld` |
| --- | --- | --- |
| Radar Noise Jamming & Burn-Through | **Skolnik**, *Radar Handbook* (3rd ed.); **Adamy**, *EW 101–104*; **FERS** (`m-brooker/FERS` / `c-innes/FERS`, already cross-checked in `tests/simulation/test_signal.cpp`); **AFSIM `WSF_EW`** & **MIXR** | `ea::receivedJammerDensityWHz` and `Radar::dwell` / `Radar::dwellSignal` raise effective receiver noise PSD from $N_0 = k_B T_0 F$ to $N_0 + J_0$, degrading SNR by $10\log_{10}(1 + J_0 / N_0)\text{ dB}$ where $J_0 = \sum_i \frac{P_i G_{t,i} G_{r,i}}{L_i B_{j,i}} \cdot \text{overlap}(f_i, B_{j,i}; f_r, B_r)$. Victim antenna gain $G_{r,i}$ evaluates the radar's actual mainbeam/sidelobe pattern along the jammer line of sight, and short-range burn-through emerges natively from $R^{-4}$ two-way echo vs. $R^{-2}$ one-way jamming. |
| Passive Bearing-Only Geolocation & CRLB | **Nicholas A. O'Donoughue**, *Emitter Detection and Geolocation for Electronic Warfare* (Artech House, 2019) & open-source Python reference **`nodonoughue/emitter-detection-python`** (`ewgeo.triang`: Stansfield WLS, iterative ML, FIM/CRLB); **KrakenSDR** (`krakenrf/krakensdr_doa`) | `esm::triangulateBearings` (`src/simulation/mission_systems/esm/Receiver.cpp`) implements a 3-stage pipeline: (1) weighted orthogonal-projection (Stansfield-class) linear initializer in shifted ECEF, (2) damped Gauss-Newton Maximum Likelihood refinement on unit-sphere angular residuals $J_{\text{ML}}(x) = \frac{1}{2}\sum_k \sigma_k^{-2}\|\hat{d}_k(x) - u_k\|^2$, and (3) exact 3-D Fisher Information Matrix $F(x^*) = \sum_k \sigma_k^{-2} r_k^{-2}(I - \hat{d}_k\hat{d}_k^T)$ inversion for 3-D CRLB RMS and local East-North horizontal elliptical $\text{CEP}_{50} = 0.5887(\sigma_a + \sigma_b)$, published in `Contact::sizeM` and `payloadReport("geo_cep_min")`. |
| Direction-Finding SNR Scaling | **O'Donoughue** Ch. 7 (interferometric DF CRLB $\sigma_\theta \propto 1/\sqrt{\text{SNR}}$); **Adamy** *EW 102* | Interferometric/SIGINT suites (`rivet-joint`, `an-alq-218`, `e3-esm`, `e7-esm`, `an-alr-94`, `an-asq-239`, `spectra`, `praetorian`, `gripen-ews`) tighten angular error with excess SNR above sensitivity down to an explicit calibration/aperture floor (`bearing_floor_deg`), while legacy amplitude-comparison RWRs (`an-alr-56m`) preserve their constant `10 deg` floor bit-for-bit. |
| RF Propagation & Terrain Masking | **ITU-R P.528 / P.676** (NTIA/ITS atmospheric absorption & aeronautical propagation already modeled in `World::pathLossDb`) | Jammer-to-radar, emitter-to-ESM, and jammer-to-DMS links all reuse `World::pathLossDb` and horizon/terrain line-of-sight checks — never a parallel free-space shortcut. |
| Tactical Datalink Carrier & COMINT | **Link-16 / JTIDS / MIDS** public RF band (`969–1206 MHz`, L-band TDMA/FHSS); **MIL-STD-6016** conceptual separation of RF carrier vs. encrypted payload | `Dms::activeCarriers` exposes only physical RF carrier metadata (`entity`, `frequencyHz`, `bandwidthHz`, `powerW`) so COMINT receivers (`EmitterMode::Datalink`) detect transmissions and bearings without ever leaking DMS topics or encrypted message bytes. |

## 2. Analytical Verification Against Literature Equations

`tests/simulation/test_esm_ea_comint.cpp` (`[sim-m05][alignment]`) verifies exact quantitative agreement with closed-form literature formulas:

1. **Skolnik / Adamy SSJ Burn-Through Law**:
   - Unit verification confirms one-way jammer density scales strictly as $R^{-2}$ (`+6.0206 dB` per range halving to `1e-5 dB`), giving `+6.02 dB` degradation growth in the jam-dominated regime ($J_0 \gg N_0$).
   - Live `World` `Radar::dwell` cross-check (`f16c` AN/APG-68(V)9 vs. `ea18g` full-band omnidirectional barrage inside burn-through at `8 km` vs. `4 km`) confirms unjammed SNR increases by $\approx 12\text{ dB}$ ($R^{-4}$) while jammed SNR increases by $\approx 6\text{ dB}$ ($R^{-2}$ burn-through), matching the textbook `6 dB` slope difference.
2. **O'Donoughue `ewgeo.triang` Bearing-Only ML & CRLB**:
   - On a 9-look symmetric moving baseline (`12 km` baseline, `40 km` emitter range), `esm::triangulateBearings` recovers the noiseless 3-D emitter position to `< 1 mm`.
   - Doubling angular DF noise $\sigma_\theta$ scales both `crlbRmsM` and `crlbCep50M` by exactly `2.0x` (`< 1e-6` relative error).
   - Doubling both range and baseline (constant parallax geometry) scales `crlbRmsM` and `crlbCep50M` by exactly `2.0x` (`< 1e-4` relative error), verifying exact GDOP scaling.
   - Live `World` moving-platform `rc135w` triangulation keeps `rangeM` and `sizeM` (`CEP50`) as `NaN` before the `2 km` / `4 deg` baseline-parallax gates are met, then converges to a finite position and CRLB `CEP50`.

## 3. Parameter Provenance Ledger

| Parameter Group | Basis | Notes |
| --- | --- | --- |
| Radar jamming equation ($N_0 + J_0$), spectral overlap, burn-through physics | `derived` (physics) | Exact radiometric / Skolnik / Adamy formulation integrated directly into `Radar::dwell` and `Radar::dwellSignal`. |
| Bearing-only ML solver & 3-D FIM/CRLB CEP50 | `derived` (statistics) | Exact Gauss-Newton ML on unit-sphere residuals and Cramér-Rao Lower Bound covariance projection (`O'Donoughue 2019`). |
| AN/ALQ-99 / Compass Call / fighter SPJ ERP, bandwidths, beamwidths | `assumed` | Specific operational ERP and jamming waveforms are classified; class defaults in `Jammer.cpp` are engineering bounds overridden whenever a payload card declares measured fields. |
| SIGINT / interferometric ESM sensitivity (`-70` to `-85 dBm`) and DF floor (`0.5–1.0 deg`) | `assumed` | Engineering class defaults in `Receiver.cpp`; ordinary RWRs remain at legacy `-60 dBm` / `10 deg`. |
| Tactical DMS net jam margin (`10 dB` $J/S$) | `assumed` | Represents uncoded/moderately spread tactical net disruption threshold; future FHSS/anti-jam per-net margin fields remain cleanly additive. |
| Optical DIRCM (`an-aaq-24`) | `not_implemented` | Explicitly returns `nullptr` / `NotImplemented` in RF jammer factory rather than fabricating RF emissions; deferred to optical/IR countermeasures. |
