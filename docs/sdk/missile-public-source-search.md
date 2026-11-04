# SIM-M02 public-source search: aero, motor and actuator data

Date: 2026-10-30. Scope: publicly accessible, legally usable sources for
per-model missile aerodynamic forces/moments, motor curves and actuator
response. No leaked, classified or export-controlled material was searched or
used. This is a source inventory, not an admission of calibration data.

## Result in one line

One public exact-model dataset was retrieved for the AIM-7M: DREV-TM-1999-050,
a trisonic wind-tunnel report on a 1:7 AIM-7M model. It provides tabulated
normal-force slope, pitching-moment slope and centre-of-pressure data over Mach
1.5-4.0, alpha -10 to +10 deg, two roll orientations and wing deflections up to
20 deg. Its drag data are incomplete because the axial-force balance failed.
No public exact-model coefficient table, thrust-time curve or actuator response
was found for the AIM-120C, AIM-9M/X or R-73. Generic reports and vendor
datasheets can support method validation and representative actuator models only.

## Sources found

| Model / need | Source | What it provides | Limit / status |
| --- | --- | --- | --- |
| AIM-7M aero | [DREV-TM-1999-050, Fournier, July 1999](https://archive.org/details/micro_IA40243311_1824) ([PDF](https://archive.org/download/micro_IA40243311_1824/micro_IA40243311_1824.pdf), [OCR](https://archive.org/download/micro_IA40243311_1824/micro_IA40243311_1824_djvu.txt)) | 1:7 AIM-7M model; Mach 1.5-4.0; alpha -10 to +10 deg; 0/45 deg roll; wing deflections 0/10/20 deg; Table C-1 to C-12: CNalpha, CMalpha, XCP; drag incomplete | Public/unclassified/unlimited. Model-scale, low-Reynolds, sting interference 10-15%; drag data missing after balance failure; not full-scale or control-derivative validation |
| AIM-7M context | [NASA TP-1078](https://ntrs.nasa.gov/citations/19780004098), Sparrow III type model with wing controls | Supersonic force/moment data for a Sparrow III-type model | Not AIM-7M; model-scale, different variant |
| AIM-7M system | [AIM-7M/P NTSP](https://www.globalsecurity.org/military/library/policy/navy/ntsp/Aim-7p.pdf) | System/functional description | No aerodynamic coefficient table |
| AIM-9M geometry/identity | [NAVAIR AIM-9M](https://www.navair.navy.mil/product/AIM-9M-Sidewinder) | 2.87 m, 85.5 kg, 0.13 m diameter, 0.63 m span, Hercules MK-36, Mach 2.5, 10-18 mi | Official geometry/identity; no coefficients |
| AIM-9M control architecture | [AIM-9M NTSP](https://www.globalsecurity.org/military/library/policy/navy/ntsp/aim-9m.pdf) | Seeker/electronics/servo assembly, torque-balance control, wings/rollerons | No numerical servo lag/rate response |
| AIM-9X control architecture | [AIM-9X NTSP](https://www.globalsecurity.org/military/library/policy/navy/ntsp/AIM-9X.pdf) | Four tail fins plus four jet vanes; closed-loop position servo | Confirms TVC mechanism; no actuator response data |
| AIM-9M motor lead | [Airbattle discussion of Mk36](https://groups.io/g/airbattle/topic/aim_9/1640171) | Mk36 Mod 5: ~2,740 lbf, 4.7 s, 14,250 lb-s; 96.7 lb motor, 35.2 lb after firing | Secondary forum, partly Chapparal Mk50 data; not authoritative; verify against a primary motor report |
| AIM-9M actuator lead | [The Sidewinder Story](https://www.ausairpower.net/TE-Sidewinder-94.html) | Improved actuator up to 100 lb-ft torque | Secondary; no rate/lag/stops; not sufficient for a response model |
| AIM-120C geometry | [USN AMRAAM fact file](https://www.navy.mil/Resources/Fact-Files/Display-FactFiles/Article/2168352/aim-120-advanced-medium-range-air-to-air-missile-amraam) | 12 ft length, 7 in diameter, 19 in C/D span, 356 lb C-5/6/7/D | Official; span conflict with Raytheon C-7 brochure remains open |
| AIM-120C geometry cross-check | [DTIC ADA355385](https://apps.dtic.mil/sti/pdfs/ADA355385.pdf), Figure 1 | AIM-120C wing span 17.5 in, control-surface span 17.5 in, length 144 in, nominal weight 345 lb | Public 1998 USN report; supports the 17.5 in figure but conflicts with the Navy 19 in C/D listing. Variant and span definition remain unresolved; no aero coefficients |
| AIM-120C geometry cross-check 2 | [USAF image caption](https://www.af.mil/About-Us/The-Book/Enlisted-Ranks/igphoto/2000424178) | AIM-120A/B wing span 1 ft 9 in; AIM-120C 1 ft 5 in | Official caption; supports a 17 in C span, not 19 in. Still a caption, not a dimensioned drawing; no model change |
| AIM-120 family | [Raytheon AMRAAM-ER wind-tunnel press release](https://raytheon.mediaroom.com/2019-02-18-Wind-tunnel-testing-moves-AMRAAM-ER-missile-closer-to-production) | 1,700+ wind-tunnel runs on AMRAAM-ER | No public coefficient data; not AIM-120C-7 |
| AIM-120 simulation | [DTIC ADA355385](https://apps.dtic.mil/sti/pdfs/ADA355385.pdf) | F/A-18 + AMRAAM distributed simulation/HWIL description | No aerodynamic coefficients or motor curve |
| AIM-120 program | [DOT&E AMRAAM reports](https://www.dote.osd.mil/Portals/97/pub/reports/FY2013/af/2013amraam.pdf) | Program/test status, motor supplier issues | No thrust-time data |
| AIM-120C-5 estimate | [Zaretto performance brief](https://zaretto.com/sites/zaretto.com/files/missile-aerodynamic-data/AIM120C5-Performance-Assessment-rev2.pdf) | Estimated drag curve, CAD motor reconstruction, 7.75 s burn estimate | Secondary/CFD/engineering estimate; not authoritative measurement |
| R-73 design/aero context | [NASA NTRS 20080014230](https://ntrs.nasa.gov/citations/20080014230), "Innovation in Aerodynamic Design Features of Soviet Missiles" | NACA/NASA wind-tunnel experience with Soviet missile designs | Public paper; not a verified R-73 coefficient table |
| R-73 geometry/identity | [Rosoboronexport R-73E](https://roe.ru/en/production/air-to-air-guided-missiles/r-73e) | Launch/warhead weight 105/8 kg; 30 km class; 45 deg off-boresight | Manufacturer identity; no coefficients/thrust curve |
| Generic missile aero | [NASA CR-2835](https://ntrs.nasa.gov/citations/19770021140), [NASA CR-2836](https://ntrs.nasa.gov/citations/19770021141) | Declassified generic missile wind-tunnel data, Mach 0.2-4.63 | Generic configurations; not named variants |
| Generic missile body | [NASA TP-1579](https://ntrs.nasa.gov/citations/19800004746), [NACA TN 3300](https://ntrs.nasa.gov/citations/19930084038) | Body/AGARD-B force and moment data | Method validation only |
| Generic missile body | [DTIC ADA193525](https://apps.dtic.mil/sti/tr/pdf/ADA193525.pdf) | Elliptical missile body force/moment/pressure data, Mach 0.4-1.3 | Generic geometry; not a named missile |
| Fin actuator structure | [DTIC ADA436368](https://apps.dtic.mil/sti/tr/pdf/ADA436368.pdf) | Second-order fin actuator, reaction-jet lag/dead time, blending | Generic model; no specific missile constants |
| Actuator limits | [JHUAPL, Overview of Missile Flight Control Systems](https://www.jhuapl.edu/techdigest/content/techdigest/pdf/V29-N01/29-01-Jackson.pdf) | Rate/position limits, first-order lag, control-loop context | Generic; no equipment data |
| Actuator example | [Marotta control actuation systems](https://marotta.com/products/control-systems/control-actuation-systems) | Example datasheet: 60 Hz +/-1 deg -3 dB, +/-20 deg, >1800 deg/s | Vendor example, not AIM-120/R-73 hardware |
| Actuator test method | [BIT frequency-response testing for missile actuators](https://pure.bit.edu.cn/en/publications/frequency-response-testing-method-for-missile-actuator-based-on-f) | FM pulse-sweep frequency-response method | Method only; no data |

## Confirmed AIM-7M dataset

DREV-TM-1999-050 is public, unclassified and approved for unlimited
distribution. Local copies for verification:
`build/architecture-reviews/sim-m02-sources/aim7m-drev-tm-1999-050.pdf` and
`build/architecture-reviews/sim-m02-sources/aim7m-drev-tm-1999-050-ocr.txt`.
The scan is 80 pages; the report body is printed pages 1-19,
with Tables C-1 to C-12 and Figures D-1 onward. The model is approximately
1:7 scale (30 mm body, 531 mm length), tested at Mach 1.5, 2.0, 2.5, 3.0, 3.5
and 4.0, alpha -10 to +10 deg, roll 0/45 deg, and wing deflections 0/10/20 deg.
The balance failed during the wing-deflection series, so many drag values are
missing; normal force and pitching moment were repeated with the overload
balance. Sting interference is stated as 10-15% for drag.

OCR-extracted slopes at zero wing deflection (values as printed; verify against
the PDF before admission). The report defines the slopes over -5 to +5 deg;
the printed magnitudes are consistent with per-radian slopes.

| Mach | Roll | CNalpha | CMalpha | XCP (cal) |
|---:|---:|---:|---:|---:|
| 1.5 | 0 | 42.710 | -57.149 | 1.3381 |
| 2.0 | 0 | 33.264 | -46.155 | 1.3875 |
| 2.5 | 0 | 27.167 | -35.027 | 1.2893 |
| 3.0 | 0 | 24.448 | -27.093 | 1.1082 |
| 3.5 | 0 | 21.841 | -23.047 | 1.0552 |
| 4.0 | 0 | OCR missing (Table C-6) | OCR missing | OCR missing |
| 1.5 | 45 | 42.504 | -50.963 | 1.1990 |
| 2.0 | 45 | 33.467 | -43.848 | 1.3102 |
| 2.5 | 45 | 26.189 | -33.976 | 1.2973 |
| 3.0 | 45 | 24.176 | -26.146 | 1.0815 |
| 3.5 | 45 | 21.544 | -23.760 | 1.1029 |
| 4.0 | 45 | 19.251 | -16.438 | 0.8539 |

The 10/20 deg wing-deflection values are in Tables C-2 to C-5 and C-8 to C-11;
they must be transcribed from the PDF/OCR and checked before use. The report
also compares its normal-force and pitching-moment results with AIM-7F data:
normal force is generally within 2-10%; 45 deg roll pitching moment differs by
up to about 20% at some angles. Those comparisons support the dataset's
credibility but do not make it full-scale AIM-7M validation.

The current SDK baseline against the 0 deg roll / 0 deg wing tables is recorded
in [aim7m-drev-validation.md](aim7m-drev-validation.md): CNalpha NRMSE 38.03%,
CMalpha 29.78%, XCP 22.81% over Mach 1.5-3.5. The model is low in both force and
pitch slopes; XCP is close at low Mach and drifts aft at higher Mach. A geometry
triage against Figure B-4 shows that the source-derived smaller control fin
would reduce CNalpha further, so the remaining gap is in the shared VLM
lift/carryover method, not in the assumed control-fin size. This is a correction
baseline, not a calibration claim.

## AIM-120C span status

Public sources now split into two groups:

- 17.5 in: DTIC ADA355385 Figure 1 (AIM-120C wing and control-surface span).
- 17 in: USAF image caption (AIM-120C 1 ft 5 in; A/B 1 ft 9 in).
- 19 in: USN/NAVAIR C/D wingspan line.

The difference is likely a definition (wing span versus control-surface span
versus overall span) or a variant distinction, not a simple typo. No model
change is justified until a dimensioned C-7 drawing or a C-7-specific primary
source resolves which measurement the 19 in line describes.

## What can be admitted now

- **AIM-7M normal-force and pitching-moment slopes** from DREV-TM-1999-050,
  within the report's 1:7 scale, Mach 1.5-4.0, alpha -10 to +10 deg, roll
  0/45 deg and wing-deflection conditions. This is not a drag or full-envelope
  dataset.
- **AIM-9M geometry/mass/motor identity**: NAVAIR page is directly usable.
- **AIM-9M/AIM-9X control architecture**: NTSP documents are usable for
  mechanism descriptions, not numerical response.
- **Generic method validation**: NASA CR-2835/2836, TP-1578/TP-1078/TP-2157,
  NACA TN 3300, DTIC ADA193525.
- **Actuator model structure**: DTIC ADA436368 and JHUAPL; constants remain
  representative/assumed unless a specific datasheet is found.

## What remains blocked

- AIM-120C-7 exact-model aero coefficients.
- AIM-9M/AIM-9X exact-model aero coefficients.
- R-73 exact-model aero coefficients.
- AIM-120C, AIM-9M and R-73 thrust-time curves.
- Any specific missile actuator lag/rate/stops.
- AIM-7M drag coefficients for most conditions; only normal-force,
  pitching-moment and XCP slopes are available from the DREV tables.

## Next search actions

1. Verify the DREV PDF pages, Table C-1 to C-12 and the printed slope units;
   transcribe the 10/20 deg wing-deflection cases and the missing Mach 4.0
   0 deg values from the PDF, not OCR.
2. Search DTIC/NTRL/NRC Canada for the same DREV report and any AIM-7F
   reference database cited by it (JTCG/ME, March 1979).
3. Search NTRS for early Sidewinder/AIM-9 wind-tunnel reports; do not map them
   to AIM-9M without variant evidence.
4. Search NASA/DTIC for AIM-120 captive-carry/store-separation reports; they may
   contain carriage loads but not free-flight coefficients.
5. Search public actuator vendor datasheets for representative rate/lag/stops;
   keep them labelled as representative, not equipment-specific.

## Search log

Queries included: AIM-7M DREV trisonic wind tunnel; AIM-120 AMRAAM wind tunnel
DTIC; AIM-9M NAVAIR/NTSP; R-73 wind tunnel NASA; AGARD-B missile force/moment;
missile fin actuator frequency response; AIM-120C-5 motor thrust curve; AIM-7M
DREV report number. Leak, classified and export-controlled sites were not
searched or used.
