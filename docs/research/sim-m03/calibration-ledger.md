# Individual AAM calibration declarations

As of 2026-11-03, all 24 maintained AAM cards contain an individual calibration statement: **24 statements, 0 empty; 0 calibrated and 24 uncalibrated**. This counts declarations, not equipment accuracy or engineering completion. Functional work continues against the unchanged component/native/installed/visual gates.

The [machine-readable ledger](calibration-ledger.json) supplies the exact 24 `records` accepted by `component_matrix.calibration_ledger`. Each record binds its card before/after hash, all nine selected component identities and canonical resolved-parameter hashes, retained source classifications/uncertainties, actual authored geometry/motor inputs and model-specific theory/method routes. Projection/runtime proofs keep their original epochs and require fresh admission after any card change.

No physical value, provider, binding, source record, uncertainty or existing non-calibration JSON field changed. Stripping only the inserted statement (and its empty new parent where absent), or restoring the prior AIM-120C statement, restores all original parsed content for 24/24 models. The reversible byte patch also restores all 24 original files exactly.

An equipment calibration admission must contain a same-model/card-epoch receipt, independent unchanged holdout, actual parameter difference, measured before-to-after error improvement, nonzero uncertainty and source/raw hash bindings. Mathematical identity checks, shared mechanisms, another variant's result and successful engagement execution cannot substitute for these measurements.

Source labels below describe the retained card records; this metadata edit does not freshly validate the external publications or upgrade an assumed/game/secondary record. Motor phase schedules, generated tables and own component methods are explicit model inputs, not OEM curves.

## AIM-120C-7 AMRAAM (`aim-120c`)

AIM-120C-7 AMRAAM is uncalibrated. The C-7 bulk mass/span lineage is distinct from the assumed WDU-41/FZU-49 internal reconstruction. Its turning-CPA sensor station and finite V2 annulus/count, rather than a V0 point initiation or free fragment cloud, require their own burst timing and fragment-spectrum validation. Retained card anchors: mass 161.5 kg (published; source usn-aim120; uncertainty 1.0), length 3.66 m (published; source usaf-aim120; uncertainty 0.01), diameter 0.1778 m (published; source usaf-aim120; uncertainty 0.001), warhead 20.0 kg (published; source wp-aim120; uncertainty 2.0), explosive 7.17 kg (assumed; source usmc-yellowbook-va49; uncertainty 1.08). Current receiver rf_signal.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.turning_cpa.v1; warhead native.warhead.finite_zoned_case.v2. Current finite RF aperture/waveform and delivered-support PN are engineering instruments; the C-7 receiver, actuator and WDU-41 geometry are not fitted hardware response curves. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`aim120c.toml`](../../../aircraft/aim120c/aim120c.toml). Exact current component selections:

- seeker: `equipment.aim-120c-7.seeker` -> `fsim.seeker.active_radar.v3`; parameter SHA-256 `e7ad26a202c12753855b5a91df439b04680dc9397add0aad89f1b9a41ae719bb`.
- guidance: `equipment.aim-120c-7.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `fbf3c62b35fd77a9df5a230dc4132a924207042189a267a4034eaa6325131c9f`.
- autopilot: `equipment.aim-120c-7.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `a5c093cbc2a421275ad7bf0a8159c55d2be1357c10baed7c68b3fa8bbd1ce2f8`.
- allocator: `equipment.aim-120c-7.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `847ed2a9b72481601341b94ee75104af8cfbee7130615173fbb044f4e019e436`.
- actuator: `equipment.aim-120c-7.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `2cac31a21b5817e72b0569cae74897de0a6bdf40ff781b59683a8d60cbb8eba6`.
- aerodynamics: `equipment.aim-120c-7.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `b1ce2ed4f6657af11f893addae269a9b7cff37e704aa75ed78ae35bd8f90ae15`.
- propulsion: `equipment.aim-120c-7.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `32f361bbc04352843ed7df5518064a53a1d31e41d7d12aa48bb3dffd34905206`.
- fuze: `equipment.aim-120c-7.fuze` -> `native.fuze.turning_cpa.v1`; parameter SHA-256 `ff55455def74ac1b58f355fec31994d45e91a92a3cfbffecadaa62698e7f5862`.
- warhead: `equipment.aim-120c-7.warhead` -> `native.warhead.finite_zoned_case.v2`; parameter SHA-256 `9b24592f378faaed949bc45ecee3ab8fb606af14540a48f2448e2ba56fd28853`.

Source/uncertainty lineage: `apl-homing-guidance`, `hughes-amraam-cutaway`, `mil-hdbk-1211`, `nasa-control-allocation`, `usaf-aim120`, `usmc-yellowbook-va49`, `usn-aim120`, `wp-aim120`, `wt-aam-seven-zones-ea96f3af`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## AIM-120D AMRAAM (`aim-120d`)

AIM-120D AMRAAM is uncalibrated. The D model retains its own longer authored sustain schedule, supported ARH/strapdown path and receive-fit declaration. Its swept-path V0 initiation and zoned case are different from the C-7 finite-case package; C-7 outcomes cannot calibrate this D configuration. Retained card anchors: mass 161.5 kg (published; source usn-aim120; uncertainty 1.0), length 3.66 m (published; source usaf-aim120; uncertainty 0.01), diameter 0.1778 m (published; source usaf-aim120; uncertainty 0.001), warhead 20.0 kg (published; source wp-aim120; uncertainty 2.0), explosive 7.17 kg (published; source wt-datamine; uncertainty 1.08). Current receiver rf_signal.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. Public AMRAAM bulk figures do not establish D-specific RF/INS errors, motor impulse or the card game-reference explosive value as equipment measurements. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`aim120d.toml`](../../../aircraft/aim120d/aim120d.toml). Exact current component selections:

- seeker: `equipment.aim-120d.seeker` -> `fsim.seeker.active_radar.v3`; parameter SHA-256 `54836e2c858069e4ef89367cd9d76946641e87fcea08ab70eb0c0d7e4c44a7a8`.
- guidance: `equipment.aim-120d.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `177b6ca4c80f31ad9ed7fd902e4c28cf2ee19ec4103225cc2c6f22f65a6c079c`.
- autopilot: `equipment.aim-120d.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `746c40997ebd875c5bfefc23611760738470640faad229cf5e94671adb78569f`.
- allocator: `equipment.aim-120d.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `6613da0d0118c4501b0a990bf5d1fcd58d271ac8edfd687c082bcbac6b956264`.
- actuator: `equipment.aim-120d.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `08b4e87e40cd9b8794e92077ca46da32abbcbe9267e6c14949f98cd3f55b9c99`.
- aerodynamics: `equipment.aim-120d.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `b9f8295da2403cb30f291f8f74a000979363b5bb93f6423111e04659a0368b8a`.
- propulsion: `equipment.aim-120d.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `588c1ca182a0edd9b3f33776748e40a40cc96f98f96e91c4fa403715fe2f73b9`.
- fuze: `equipment.aim-120d.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `1a9d0c576e3bf81da99b392b0cdec73c8f0e10df9d42d6984a9371b357c36781`.
- warhead: `equipment.aim-120d.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `edee1156a72b838e0b9460f8e4b7aa7ac0cc0a4557b2a0804c022a7ac58cb65f`.

Source/uncertainty lineage: `usaf-aim120`, `usn-aim120`, `wp-aim120`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## AIM-7M Sparrow (`aim-7m`)

AIM-7M Sparrow is uncalibrated. The Sparrow uses bistatic/SARH illumination and separate transmitter/receiver geometry, not autonomous ARH range authority. Its 225 kg airframe, Mk 58 description and authored boost/sustain phases change the actual mass, lever arms and control/energy history. Retained card anchors: mass 225.0 kg (published; source usaf-aim7; uncertainty 5.0), length 3.64 m (published; source usaf-aim7; uncertainty 0.01), diameter 0.203 m (published; source usaf-aim7; uncertainty 0.002), warhead 39.9 kg (published; source wp-aim7; uncertainty 1.0), explosive 11.79 kg (published; source wt-datamine; uncertainty 1.77). Current receiver rf_signal.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. Government Sparrow identity/bulk sources do not calibrate the modeled seeker noise/illumination transfer, Mk 58 thrust history, swept fuze or fragment effectiveness. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`aim7m.toml`](../../../aircraft/aim7m/aim7m.toml). Exact current component selections:

- seeker: `equipment.aim-7m.seeker` -> `fsim.seeker.semi_active_radar.v3`; parameter SHA-256 `f7dc340def1ad096fdb31d00bd4effa7d6efcdcba80e2043ee9b630fbccf08ae`.
- guidance: `equipment.aim-7m.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `55b90d42b0babe8ddb5227a2e9168b0a542647df9fbc89bd277cbf96a55b46ab`.
- autopilot: `equipment.aim-7m.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `5abb72636512691e89c958ffd9342999f3685edf321248dc7fef183c0fa06af9`.
- allocator: `equipment.aim-7m.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `d09e01f46f47474f539ed4c414e367a41389726804c1d815c2e9a2a2a4591593`.
- actuator: `equipment.aim-7m.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `2fbf03e3afd79f9c6b5257bb1d543d84ba7d8770d0662801956786dd3d6c71d6`.
- aerodynamics: `equipment.aim-7m.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `490a65cbd67842f8e3acb96749e7e190c664821dc5bc3bc43a8b8f40e05ef791`.
- propulsion: `equipment.aim-7m.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `f695b3617e42e21ca62af0e231d1b83a3d5d2d64b507f3f63c983e1f6a494d66`.
- fuze: `equipment.aim-7m.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `767ba2fcda0befcec949761c67979920699859e3cc421eaa31b8da27c0ac6c7b`.
- warhead: `equipment.aim-7m.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `2db284716d8ffd42f3a07c50cf87cdb3b24f56fdbb18ba67b130dfd800b9f4c5`.

Source/uncertainty lineage: `usaf-aim7`, `wp-aim7`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## AIM-9M Sidewinder (`aim-9m`)

AIM-9M Sidewinder is uncalibrated. The Sidewinder-M selects a scalar conical-scan reticle with seven-model shared software optics but its own 85.5 kg geometry, reduced-smoke motor and passive rolleron surfaces. Reticle carrier/background inference is not the old quadrant mechanism or AIM-9X imaging. Retained card anchors: mass 85.5 kg (published; source usaf-aim9; uncertainty 1.0), length 2.87 m (published; source usaf-aim9; uncertainty 0.01), diameter 0.127 m (published; source usaf-aim9; uncertainty 0.002), warhead 9.43 kg (derived; source usn-aim9x; uncertainty 0.2), explosive 3.58 kg (published; source wt-datamine; uncertainty 0.54). Current receiver thermal_conscan_reticle.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. The 9.43 kg warhead entry is a declared cross-variant derivation; exact M reticle/noise, rolleron bearing friction and detonation spectra remain inferred rather than established by the bulk USAF fact sheet. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`aim9m.toml`](../../../aircraft/aim9m/aim9m.toml). Exact current component selections:

- seeker: `equipment.aim-9m.seeker` -> `fsim.seeker.nonimaging_ir.v3`; parameter SHA-256 `d10455f1bfd2e195a73e467aeed201fcd15f2a6ee3009418d6b45f236b89f288`.
- guidance: `equipment.aim-9m.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `51a1799852ff2f44a56736ea61f9252115338b35fc0a856b8d4c010a7de6ab1b`.
- autopilot: `equipment.aim-9m.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `a3f308ee6e3339784446f201bbfe1655aee1c6247483da6f2439b2f15ba7125b`.
- allocator: `equipment.aim-9m.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `608daef6fabedd4ab0cba071f1c907038f29dc5a63225f6e16b9974df75528ed`.
- actuator: `equipment.aim-9m.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `c32f1af95d1a94960281a2ef1801e6dbf7d496ea34dc837631eb275b3d376320`.
- aerodynamics: `equipment.aim-9m.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `cf08cd90f93bf28f0c2bed071a5b8bad092cfdd3f0587ed1cf2d317a16a7d461`.
- propulsion: `equipment.aim-9m.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `e6db75a7de0ccfd26b78128eb7c53539b19b441fe63c6c8a5702988f580d2975`.
- fuze: `equipment.aim-9m.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `c880bbdc9dda7117fd6533e960a0b60043d25fcf84b59fe606ca854a317e5282`.
- warhead: `equipment.aim-9m.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `79a3c50125053e273acd31f32f55797991b2bbe5d84090afad265505f7a4ac9a`.

Source/uncertainty lineage: `native-conscan-engineering-v1`, `usaf-aim9`, `usn-aim9x`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## AIM-9X Block I Sidewinder (`aim-9x`)

AIM-9X Block I Sidewinder is uncalibrated. The Block I X selects an imaging receiver and its own airframe/finite vectoring-nozzle geometry; it is not the M scalar reticle. The card records the M motor/warhead family relationship without copying M seeker/control accuracy. Retained card anchors: mass 84.0 kg (published; source usn-aim9x; uncertainty 1.0), length 3.018 m (published; source usn-aim9x; uncertainty 0.02), diameter 0.127 m (published; source usn-aim9x; uncertainty 0.002), warhead 9.36 kg (published; source usn-aim9x; uncertainty 0.1), explosive 3.58 kg (published; source wt-datamine; uncertainty 0.54). Current receiver thermal_image.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. US Navy bulk mass/warhead entries constrain this model, while focal-plane response, equivalent TVC dynamics, generated force tables and game-reference explosive mass remain distinct validation questions. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`aim9x.toml`](../../../aircraft/aim9x/aim9x.toml). Exact current component selections:

- seeker: `equipment.aim-9x-block-i.seeker` -> `fsim.seeker.imaging_ir.v3`; parameter SHA-256 `876bc671addb5ec1fc9e7f1a4c2e9d919a5d799653fe665f3b0a41d33f44deef`.
- guidance: `equipment.aim-9x-block-i.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `b233c0b68de448c6bcdc8423472f9035f2abe959396f71a546b3995ba5da7dbb`.
- autopilot: `equipment.aim-9x-block-i.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `140d98489422cbc0da305809294bd7f0d1fa7e2bba46b3d1509ceed8cea67870`.
- allocator: `equipment.aim-9x-block-i.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `57d4ba926bde6b27b7f388d19d35c01713fc19fc07c852b91600e5b2597306f0`.
- actuator: `equipment.aim-9x-block-i.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `61cd4fccdd8ca2fad2ea27ca686c2cc97ef245a42a6d959c5a920d308a1482ac`.
- aerodynamics: `equipment.aim-9x-block-i.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `b293875ab3cd36752f9fc7bd856b073a9fa80e1fb1a2178db867d1ba6f19f3bd`.
- propulsion: `equipment.aim-9x-block-i.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `28098ac6af0ae3c79938e9528df15a2e1da047904ac57ed4fae621191c140530`.
- fuze: `equipment.aim-9x-block-i.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `d1ac2cc60cbf51a03062056e88926bcbd1e0a0f2de518363528329230e298755`.
- warhead: `equipment.aim-9x-block-i.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `e2d6902af9477810d7d0de28cf5740a787087c562d975dbca98a32915c5f7664`.

Source/uncertainty lineage: `native-vector-engineering`, `usn-aim9x`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## AIM-132 ASRAAM (`asraam`)

AIM-132 ASRAAM is uncalibrated. ASRAAM retains its 88 kg/166 mm body, assumed charge, imaging receiver and two-phase authored high-impulse motor. Its 25 kN/6 kN phase prescription changes mass flow and reach independently of the other short-range imaging models. Retained card anchors: mass 88.0 kg (published; source mbda-asraam-identity; uncertainty 0.5), length 2.9 m (published; source mbda-asraam-identity; uncertainty 0.01), diameter 0.166 m (published; source mbda-asraam-identity; uncertainty 0.001), warhead 10.0 kg (published; source wp-asraam; uncertainty 1.0), explosive 3.69 kg (assumed; source unassigned; uncertainty 0.92). Current receiver thermal_image.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. Manufacturer identity/bulk information does not measure this model's optical contrast, finite actuator transfer, inferred charge or synthetic motor/airframe curves. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`asraam.toml`](../../../aircraft/asraam/asraam.toml). Exact current component selections:

- seeker: `equipment.asraam.seeker` -> `fsim.seeker.imaging_ir.v3`; parameter SHA-256 `328641597eaaa1551f90a1e63c9707c4410d5b54d1b86749dbd52015998a0e6c`.
- guidance: `equipment.asraam.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `1e3cd5eef54af36643370af822f17b3bf94cfdd97ba7815340d6d9263b26db18`.
- autopilot: `equipment.asraam.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `c3485570c9e4d48439e3acd77c10bf34374fcb71ee8c039f60f681f02d18c992`.
- allocator: `equipment.asraam.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `2555458f20ee7e0e76b6de8587583a1f54730f3ad2c43e3b5d13acf8027674df`.
- actuator: `equipment.asraam.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `7c6131bd0521b4a1548e7cc1324e45231489e3368998d3133d76348314ba94df`.
- aerodynamics: `equipment.asraam.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `a013425df04dfb766e728d70efc68913c9bf19104bf18b05d9de276228accb7f`.
- propulsion: `equipment.asraam.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `1f3f3a9fee786fef6af62ac9e5647bbb892ab939a4abac1e422659f27ade659b`.
- fuze: `equipment.asraam.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `9bda81a0ed014546e29e21da3d4669fcadd6b174108ba29254c312cd8865721f`.
- warhead: `equipment.asraam.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `3a9ff045dcea44fa4b1634ac20d18377bc86066c02d09952fd16ead222ab1db8`.

Source/uncertainty lineage: `mbda-asraam-identity`, `wp-asraam`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## IRIS-T (`iris-t`)

IRIS-T is uncalibrated. IRIS-T retains its assumed 87.4 kg mass, own fin/vectoring-nozzle layout, imaging path and 11.4 kg warhead parameterization. Its five-second authored solid-motor phase is not an ASRAAM impulse curve or a calibrated IRIS-T thrust trace. Retained card anchors: mass 87.4 kg (assumed; source wp-irist; uncertainty 2.0), length 2.936 m (published; source saab-iris-t; uncertainty 0.001), diameter 0.127 m (published; source saab-iris-t; uncertainty 0.001), warhead 11.4 kg (published; source wp-irist; uncertainty 0.5), explosive 4.21 kg (assumed; source unassigned; uncertainty 1.05). Current receiver thermal_image.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. Published identity/warhead statements coexist with assumed mass/charge, detector constants, native fin force/moment tables and equivalent TVC limits. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`irist.toml`](../../../aircraft/irist/irist.toml). Exact current component selections:

- seeker: `equipment.iris-t.seeker` -> `fsim.seeker.imaging_ir.v3`; parameter SHA-256 `f1d498294fd7306ec8c543fe5dac15e16dbc499b201f77a312e98a2407cf189a`.
- guidance: `equipment.iris-t.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `04143ce3d884319bfcdb524f49fb80902cf10d63ba05f38f2f6705c2d289ea52`.
- autopilot: `equipment.iris-t.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `51f7f85131840407f8ada65abeef1f6d4bb5e2ad81d4ed9921d55f7f745f3928`.
- allocator: `equipment.iris-t.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `e41c61457a5b37e92755c33246fcf85005a653a6b2a39f5c6acebc2d2a38b7d8`.
- actuator: `equipment.iris-t.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `af007c83d454d6a42d28e756cb18a2c74026c69793faf0060558dc4ebc0c4572`.
- aerodynamics: `equipment.iris-t.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `932ae45055ffbf808d6dbd941b984dafa086181c2c410dc8e93d6e36364bd426`.
- propulsion: `equipment.iris-t.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `ceea6e64bdce20cb3449b0e0ec109f61cd023203cc98aefe1c4e13d9e1176524`.
- fuze: `equipment.iris-t.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `b78916999d2fa3fb15f95572f91d26c62fc192a85134191f6954a1a6de06fb6f`.
- warhead: `equipment.iris-t.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `e06568f765da8fc98e01a63635d25df83678c75adff7e44f86ca30782fb23c4c`.

Source/uncertainty lineage: `native-vector-engineering`, `saab-iris-t`, `wp-irist`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## R.550 Magic 2 (`magic-2`)

R.550 Magic 2 is uncalibrated. Magic 2 uses the selected scalar reticle approximation despite historical descriptions of a multi-element receiver; no software sharing establishes optical hardware equivalence. Its 90 kg airframe and short two-second authored motor phase also differ from the Sidewinder family. Retained card anchors: mass 90.0 kg (published; source odin-magic2; uncertainty 1.0), length 2.75 m (published; source odin-magic2; uncertainty 0.02), diameter 0.157 m (published; source odin-magic2; uncertainty 0.001), warhead 12.7 kg (published; source wp-magic; uncertainty 0.5), explosive 6.0 kg (published; source wt-datamine; uncertainty 0.9). Current receiver thermal_conscan_reticle.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. The current single-detector spoke mask, scan frequency, noise and tracking behavior are explicit inference, not a measured Magic 2 optical assembly; warhead/charge reference classifications remain as authored. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`magic2.toml`](../../../aircraft/magic2/magic2.toml). Exact current component selections:

- seeker: `equipment.magic-2.seeker` -> `fsim.seeker.nonimaging_ir.v3`; parameter SHA-256 `a611327d8851e464ac42c8f2f50345f50fe37d4e5a7f2e5289469ee704029336`.
- guidance: `equipment.magic-2.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `e633631307113c387fd43eb1e99431e9eebd8bb1e93020a7c2f4ccf6e1c94353`.
- autopilot: `equipment.magic-2.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `581b88980e782effce32c661bd52fc3b5a5bdf239cf899742b687e1b3eb1b937`.
- allocator: `equipment.magic-2.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `e7c689b580572413b461f361f73873f4dd1dec2e49f9c864cefb661cfef7c352`.
- actuator: `equipment.magic-2.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `9047796a543a9a4665941e0cd896c5dcafeacf3f94d37a1bca7e27ac389e98f5`.
- aerodynamics: `equipment.magic-2.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `d463627f9ed01808d20108097b12773f9509d0c58ec2c593ec771eaa368a4283`.
- propulsion: `equipment.magic-2.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `d5488221a4fb5770bd32abe4d03ce88f7ec1c84f741bc85c607784d7074e040e`.
- fuze: `equipment.magic-2.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `a5ab904aaf400f4c90da5d1f3943bb2cc0e7c7b8edaf096b0f1bb5d5ed8b87b3`.
- warhead: `equipment.magic-2.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `fcd05a69e0d52c6d8ddaca13d5e82fc6cec6fc02ec610d00dd12bafa173d84c3`.

Source/uncertainty lineage: `native-conscan-engineering-v1`, `odin-magic2`, `wp-magic`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## Meteor (`meteor`)

Meteor is uncalibrated. Meteor executes the ducted-rocket V2 inlet/valve/combustor inventory model and ARH receiver on its 190 kg airframe. Booster and sustained air-breathing phases require altitude/Mach/inlet and starvation tests, rather than treating the authored sustained thrust as a solid-rocket motor. Retained card anchors: mass 190.0 kg (published; source mbda-meteor-identity; uncertainty 1.0), length 3.7 m (published; source mbda-meteor-identity; uncertainty 0.02), diameter 0.178 m (published; source mbda-meteor-identity; uncertainty 0.001), warhead 12.0 kg (published; source fr-meteor; uncertainty 2.0), explosive 4.43 kg (assumed; source unassigned; uncertainty 1.11). Current receiver rf_signal.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_ducted_rocket.v2; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. Manufacturer identity and published bulk mass do not calibrate inlet recovery, fuel metering, ram drag, combustor efficiency, seeker noise or the assumed explosive charge. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`meteor.toml`](../../../aircraft/meteor/meteor.toml). Exact current component selections:

- seeker: `equipment.meteor.seeker` -> `fsim.seeker.active_radar.v3`; parameter SHA-256 `b71c05fbe4fcc98582906922a08842e1e012e6253ee5e7ed52e57db52a377112`.
- guidance: `equipment.meteor.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `f37223aba2e7d5e557904e27b725c621e1e2ff7ae329997ac8c68fc21de86fe9`.
- autopilot: `equipment.meteor.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `717b612acbccf6cd711dac5114e92918f668c6ac20987a18960de19e1492c07d`.
- allocator: `equipment.meteor.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `29e47a7d9121f40dc400c8a6c519cc7f4b0c9e0568a13a853184c1ac746e85c2`.
- actuator: `equipment.meteor.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `b14f688ff5a2ccd76f3f0a04c93049ebeeee559a07a4a52fa6a34f348bc2a680`.
- aerodynamics: `equipment.meteor.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `6d69907f3f6b4a74aba7b5035c18007aafda37395f180ee332b2bc16cded82ba`.
- propulsion: `equipment.meteor.propulsion` -> `fsim.propulsion.jsbsim_ducted_rocket.v2`; parameter SHA-256 `a3fcb99980391f156b5f9dd3c56315e3b0badd1c2fdbbfe6cc6366e0b80c6edf`.
- fuze: `equipment.meteor.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `eacf6c946880d0f828a600ebe47a8b22c7fa94ee3697d17109b60c9dcdc91e88`.
- warhead: `equipment.meteor.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `fe94f07868fd02c2191815d173742c422f04ff3a334854ec6424dc8026eebef6`.

Source/uncertainty lineage: `fr-meteor`, `mbda-meteor-identity`, `nasa-ramjet-cycle-engineering`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## MICA EM (`mica-em`)

MICA EM is uncalibrated. MICA EM shares the 112 kg/160 mm geometry and solid-motor schedule with MICA IR but executes ARH RF exposure rather than dual-waveband thermal detection. Shared shape and 12 kg warhead do not provide cross-seeker detection/accuracy calibration. Retained card anchors: mass 112.0 kg (published; source mbda-mica-identity; uncertainty 1.0), length 3.1 m (published; source mbda-mica-identity; uncertainty 0.02), diameter 0.16 m (published; source mbda-mica-identity; uncertainty 0.001), warhead 12.0 kg (published; source wp-mica; uncertainty 1.0), explosive 3.7 kg (published; source wt-datamine; uncertainty 0.56). Current receiver rf_signal.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. The active EM aperture/waveform, support transport, actuator response and game-reference charge need their own measured response/uncertainty rather than an IR firing result. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`micaem.toml`](../../../aircraft/micaem/micaem.toml). Exact current component selections:

- seeker: `equipment.mica-em.seeker` -> `fsim.seeker.active_radar.v3`; parameter SHA-256 `53815f9175376a704166fdfa391d5f7844cc7db76f81a71bd9e9e3f1f75a021b`.
- guidance: `equipment.mica-em.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `ff390680932078d2c2e655243cc2562e79870f5d00579013f25ec9707a46f824`.
- autopilot: `equipment.mica-em.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `fdeaead3bbae25a4638984f3bc26c9ff465941ae5c8cdcf749c13787030e7bfc`.
- allocator: `equipment.mica-em.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `722e351284247886ca63e09063b6acf7cd08933166d38f5047de6bafde117bf9`.
- actuator: `equipment.mica-em.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `6828eae79c8bd43db3f6d16a3a22c48468e6abd33883ba45d9015808019605c9`.
- aerodynamics: `equipment.mica-em.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `0d427b1fe92742c210e386a083712b1a8ecb3532cf341b2a4c48d08ec90b1cd9`.
- propulsion: `equipment.mica-em.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `aecb8bd5420b166e15c9fe48b5efd01861b776acb5d4c21756d0de9d886450ed`.
- fuze: `equipment.mica-em.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `860eeebf25b54aee3a44eb6a81371c59eaaf991b1dd7d6fba65d5b8a7e1974b5`.
- warhead: `equipment.mica-em.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `6b9671b53578c67a316fbaaf669744efa33105152bc06a12d99d2969e515f0e9`.

Source/uncertainty lineage: `fr-mica`, `mbda-mica-identity`, `native-vector-engineering`, `wp-mica`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## MICA IR (`mica-ir`)

MICA IR is uncalibrated. MICA IR executes two separately rendered thermal bands with its own retained-state/association rules; its 112 kg body and motor match the EM design only at the declared structural inputs. Dual-band receiver state and spectral errors cannot be calibrated by an EM RF run. Retained card anchors: mass 112.0 kg (published; source mbda-mica-identity; uncertainty 1.0), length 3.1 m (published; source mbda-mica-identity; uncertainty 0.02), diameter 0.16 m (published; source mbda-mica-identity; uncertainty 0.001), warhead 12.0 kg (published; source wp-mica; uncertainty 1.0), explosive 3.7 kg (published; source wt-datamine; uncertainty 0.56). Current receiver thermal_dual_image.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. Band-specific optics/QE/integration and target spectra are engineering records; manufacturer identity and bulk mass do not establish exact dual-band discrimination or native flight/effects accuracy. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`micair.toml`](../../../aircraft/micair/micair.toml). Exact current component selections:

- seeker: `equipment.mica-ir.seeker` -> `fsim.seeker.imaging_ir.v3`; parameter SHA-256 `4a04a671660e9c01ec0bcd7d8212163543636dbeaf3d8548143f0dcb748f8f08`.
- guidance: `equipment.mica-ir.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `e6046050f4917cc0ffcbe6099a2f8242bfe22ee3740c723ccce6fdfe1f1a5fbd`.
- autopilot: `equipment.mica-ir.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `8f5dc5e63b5498b2c8764e99a9b5d9e8a3e361259674b266e4121f7717d2ae42`.
- allocator: `equipment.mica-ir.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `a0556ee4215c56c65fcf1a1fde516b289b7957d4e7a4fea1624649b9e7536223`.
- actuator: `equipment.mica-ir.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `85132a7075b1b8a38873fdbb0ab5438b03d19674131fc33455607a05acb0c51e`.
- aerodynamics: `equipment.mica-ir.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `40d6de3a769f7ddeb8d61db8ef62d640ed2317fc5e4fc46841cc79f4c98da4e2`.
- propulsion: `equipment.mica-ir.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `59a9c3769c9d064d59e4ad8608e5369770369efc2ecde284231f511e1d2e9d5f`.
- fuze: `equipment.mica-ir.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `7486de74a3b2872984b9d7074eda8a746923a799b91764a6d18d121be51feac5`.
- warhead: `equipment.mica-ir.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `be2ece63ca954c564bcd479cde7b251b13e3b9a424b939ace84944a34281d6a1`.

Source/uncertainty lineage: `fr-mica`, `mbda-mica-dual-waveband`, `mbda-mica-identity`, `native-vector-engineering`, `wp-mica`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## PL-10 (`pl-10`)

PL-10 is uncalibrated. PL-10 selects imaging IR, the actuator-aware cascade and finite TVC on its 89 kg design. Its measured-load directional-force/axial coupling, native actuator plant and inferred 11 kg warhead/4.06 kg charge differ from the generic cascade and from an OEM control law. Retained card anchors: mass 89.0 kg (published; source zh-pl10; uncertainty 2.0), length 3.0 m (published; source zh-pl10; uncertainty 0.05), diameter 0.16 m (assumed; source unassigned; uncertainty 0.03), warhead 11.0 kg (assumed; source unassigned; uncertainty 2.0), explosive 4.06 kg (assumed; source unassigned; uncertainty 1.01). Current receiver thermal_image.v1; control fsim.control.actuator_aware_cascade.v1; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. The current Chinese-language bulk-data lineage does not establish focal-plane noise, TVC transfer, full aerodynamic derivatives or a PL-10 detonation spectrum; real same-model accuracy remains separate from arithmetic regressions. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`pl10.toml`](../../../aircraft/pl10/pl10.toml). Exact current component selections:

- seeker: `equipment.pl-10.seeker` -> `fsim.seeker.imaging_ir.v3`; parameter SHA-256 `c4952df4b7e95a0d90f9d735f058512b9e9ada7483a2fa9506e4cd4af314c513`.
- guidance: `equipment.pl-10.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `e0c48a844510701246c677da88f5a7dd8d825dacb1d642b63c9e6fddab99b140`.
- autopilot: `equipment.pl-10.autopilot` -> `fsim.control.actuator_aware_cascade.v1`; parameter SHA-256 `26d6bbe3cbf65b82d7a5b357419a498e8e4fca587f53b710c882b71756de58b9`.
- allocator: `equipment.pl-10.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `0f1d2dab1a6f93acd9a74b7bb361c1f0e4ba5f9ea83dee75e50e8343e0528be8`.
- actuator: `equipment.pl-10.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `ebcdaa742b1533176c6fcc69099c3e9478a792df0a8d6a3f29104cf1c5df6a41`.
- aerodynamics: `equipment.pl-10.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `98480262fdc4247cedc6145b89b5bb7b7def81906471fe509d493fd95c0195fc`.
- propulsion: `equipment.pl-10.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `0904316fa78fc7aa7b700fb78d6ca1dea165b7807846c82070d0d51884617ae5`.
- fuze: `equipment.pl-10.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `e0bb8e8dde9579c267b2737d5e249a521bded8dbb7183a7a0bdbdb8dcbfa41ac`.
- warhead: `equipment.pl-10.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `52bf1236bb2e3f25fd285f31de77bf9439a6da42be96c2b30d42bce366689012`.

Source/uncertainty lineage: `native-vector-engineering`, `zh-pl10`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## PL-12 (`pl-12`)

PL-12 is uncalibrated. PL-12 retains its 180 kg/203 mm airframe, 3 s/6 s authored thrust phases and active RF terminal path. The 20 kg warhead is assumed while the 9.2 kg explosive entry is a game reference; those bases must not be merged into a sourced hardware case design. Retained card anchors: mass 180.0 kg (published; source wp-pl12; uncertainty 10.0), length 3.93 m (published; source zh-pl12; uncertainty 0.03), diameter 0.203 m (published; source zh-pl12; uncertainty 0.002), warhead 20.0 kg (assumed; source unassigned; uncertainty 3.0), explosive 9.2 kg (published; source wt-datamine; uncertainty 1.38). Current receiver rf_signal.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. The own fin geometry, impulse, aperture/noise and fragment energy distribution need held-out equipment observations; neither PL-15 nor another ARH result calibrates PL-12. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`pl12.toml`](../../../aircraft/pl12/pl12.toml). Exact current component selections:

- seeker: `equipment.pl-12.seeker` -> `fsim.seeker.active_radar.v3`; parameter SHA-256 `f648212b0dba9b419da1d546cf106c2038a804229003280c5cebc16ae0efec7b`.
- guidance: `equipment.pl-12.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `b15d02bd6b1258fee5eb7580d847ff675a6cb7b1d85e9886066aabe9683060bf`.
- autopilot: `equipment.pl-12.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `074cc7008127fa91d717cdf8b636a6dc85e65fba66bd20e7c694274d4b2d260b`.
- allocator: `equipment.pl-12.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `c01be530c24afee52ea948de75765515b56aaf6a8e5caac2103da9f96674fb83`.
- actuator: `equipment.pl-12.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `c71b3099e3301a88e0dd5063ffaa2ef22799913a827cfb1f472f6d048996a561`.
- aerodynamics: `equipment.pl-12.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `da0d8d7402b39574c7ae4a86573c6830fa712f7f9e0e7a56328b54ef46bd18d6`.
- propulsion: `equipment.pl-12.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `dccaac6b3db83c948865b6b6ee167bd1d707615cbc430b83ca829ed521f8eca0`.
- fuze: `equipment.pl-12.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `e98fc0f0da48327e5fce22b4928e145e22a499f77972da5e6214862e7c134773`.
- warhead: `equipment.pl-12.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `3a1393c8c0c97feae60ee0dc765b14429cb8ded8a986d2f5a29fe0f853e02e76`.

Source/uncertainty lineage: `wp-pl12`, `wt-datamine`, `zh-pl12`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## PL-15 (`pl-15`)

PL-15 is uncalibrated. PL-15 selects commanded dual-pulse propulsion on its 210 kg design, with a six-second authored inter-pulse interval and separately commanded restart. That physical scheduling and 22.5 kg assumed warhead are not the PL-12 dual-thrust motor or an exported variant. Retained card anchors: mass 210.0 kg (published; source zh-pl15; uncertainty 10.0), length 4.0 m (published; source odin-pl15; uncertainty 0.02), diameter 0.203 m (published; source wp-pl15; uncertainty 0.002), warhead 22.5 kg (assumed; source unassigned; uncertainty 2.5), explosive 8.3 kg (assumed; source unassigned; uncertainty 2.08). Current receiver rf_signal.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_commanded_dual_pulse.v1; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. Bulk Chinese-source values coexist with inferred pulse timing, grain/inertia, RF/INS parameters, charge and fragment geometry; a successful restart is engineering execution rather than an empirical range calibration. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`pl15.toml`](../../../aircraft/pl15/pl15.toml). Exact current component selections:

- seeker: `equipment.pl-15.seeker` -> `fsim.seeker.active_radar.v3`; parameter SHA-256 `d3fcf56320613bd9a987c079d86b726322eededf722620ae41331a5d4f7fb828`.
- guidance: `equipment.pl-15.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `be4dde56eef7d0cb371c6e7ec7ca76186dc3901f4c238e8a4ff32ad8a1589c81`.
- autopilot: `equipment.pl-15.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `fff468c7c1ec93109293e462d97daeb3f9fafee280e9b2a799c317bee30d33d7`.
- allocator: `equipment.pl-15.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `28ff310d65b2a1cf5945861442aa142dcd30dd450c358fc56aa8f9e58404ee43`.
- actuator: `equipment.pl-15.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `73b92fb1d697a36252abde1e25f370483e562d3405bc930d853abb360458563e`.
- aerodynamics: `equipment.pl-15.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `7f588467a4619715833588c5ef728323b6347cc3d6d2cb2c51d1588e640e34f1`.
- propulsion: `equipment.pl-15.propulsion` -> `fsim.propulsion.jsbsim_commanded_dual_pulse.v1`; parameter SHA-256 `02e86873f6b110cc680ccb469d5aa38026c8b7dad246f815211b3c18f2d3a1ca`.
- fuze: `equipment.pl-15.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `ae938cb68e0cdffb2cd3dc0ac173dba7e3d11b808dfa63d92175cf836bfe7a11`.
- warhead: `equipment.pl-15.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `be85e52fd52b8927b9576d532c1a1bd98e483fdf08ba1dbd0cb3230d3dde8034`.

Source/uncertainty lineage: `odin-pl15`, `wp-pl15`, `zh-pl15`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## PL-8 (`pl-8`)

PL-8 is uncalibrated. PL-8 retains its 115 kg/160 mm airframe, three-second authored motor phase and selected scalar reticle. Its identity relationship to the Python family does not supply an exact PL-8 mask, detector, control electronics or firing record. Retained card anchors: mass 115.0 kg (published; source wp-pl8; uncertainty 2.0), length 2.95 m (published; source wp-pl8; uncertainty 0.02), diameter 0.16 m (published; source wp-pl8; uncertainty 0.002), warhead 11.0 kg (published; source wp-pl8; uncertainty 1.0), explosive 5.0 kg (published; source wt-datamine; uncertainty 0.75). Current receiver thermal_conscan_reticle.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. Current mass/warhead and game-reference charge records constrain the parameterization only at their stated basis; reticle/noise, fin forces and fragment behavior remain individually uncalibrated. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`pl8.toml`](../../../aircraft/pl8/pl8.toml). Exact current component selections:

- seeker: `equipment.pl-8.seeker` -> `fsim.seeker.nonimaging_ir.v3`; parameter SHA-256 `99a57a0a41a4e9c994f60fe6c843004e03c24115302c237c4951feea510f9fce`.
- guidance: `equipment.pl-8.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `730c253252738ad187ab2278ca67c3b74049732c10e450569638c33fd30869e2`.
- autopilot: `equipment.pl-8.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `533606611479655ded4dfd7d32fc97a9a80100170428e013697c2cfdad73fad7`.
- allocator: `equipment.pl-8.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `0b4d22f19364f9f85dbfd19aca8a2fbcb36573ffb792484cd407775a0e3b17d8`.
- actuator: `equipment.pl-8.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `cc3189ac1a040bf9a354e48fd9d614c26e09e0fc3c4937ea9b8bd292fe12e371`.
- aerodynamics: `equipment.pl-8.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `9d1776c2f8cf8c6d405a64444ff7a7d6e633458929fb12270c76e50eedee8f11`.
- propulsion: `equipment.pl-8.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `26e5b47239f83fd005b085006dbf1e1c4482d345919d888367cb30704be7b29c`.
- fuze: `equipment.pl-8.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `fc38df9519ac0200049946a177d8e71c7da6dbe0e791dcb21420c21dc702325f`.
- warhead: `equipment.pl-8.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `98854317952a183567c341ded5a403b96ce7564baa77a1b0c53dc64e8aeff11d`.

Source/uncertainty lineage: `native-conscan-engineering-v1`, `wp-pl8`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## R-27ER (`r-27er`)

R-27ER is uncalibrated. R-27ER combines SARH illumination with the 350 kg extended 260 mm motor/body and connected-ring warhead topology. The 35 kN/9 kN authored phases and larger evolving inertia distinguish it from R-27R despite common rod/charge records. Retained card anchors: mass 350.0 kg (published; source ru-r27; uncertainty 5.0), length 4.78 m (derived; source ru-r27; uncertainty 0.08), diameter 0.26 m (published; source ru-r27; uncertainty 0.003), warhead 39.0 kg (published; source wp-r27; uncertainty 1.0), explosive 15.0 kg (published; source wt-datamine; uncertainty 2.25). Current receiver rf_signal.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.connected_ring.v1. The extended ER geometry/impulse, bistatic receiver and connected-ring deployment require their own measured response; R/ET/T sharing cannot establish ER hit probability or lethal radius. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`r27er.toml`](../../../aircraft/r27er/r27er.toml). Exact current component selections:

- seeker: `equipment.r-27er.seeker` -> `fsim.seeker.semi_active_radar.v3`; parameter SHA-256 `2bb75662658c764d026dbfb323d8f32fefc2e3b599b3bdd3a4900e63a25d1090`.
- guidance: `equipment.r-27er.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `6be11b0dea5733a35b4ab97d15262bdd528191df256641186ad43fc946713842`.
- autopilot: `equipment.r-27er.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `62538d3fae74c41f0c2dd904c53fdb3ce2484a908f14168f5a22d5984512a555`.
- allocator: `equipment.r-27er.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `a315f6e677c51379cf4ebf9c52c1fcc5cba5a64f7aa9a2f376657289d9620ff3`.
- actuator: `equipment.r-27er.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `04acee6a365c9021e6fd597ed12b1242853fae74766cda0544a639c9b5a3a3cf`.
- aerodynamics: `equipment.r-27er.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `a3ce11544fb6099035f2f77e2dc3c3df1e823be1fbdabc6346119873d1bf4bd4`.
- propulsion: `equipment.r-27er.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `8642507b9f21714b5992036b9bb662e728d32a8e31b4753232b10fca3686e5b6`.
- fuze: `equipment.r-27er.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `925f74a728e471de6192f7812eeda73bc96818e5cdb960a4b2a34601aad66fe9`.
- warhead: `equipment.r-27er.warhead` -> `native.warhead.connected_ring.v1`; parameter SHA-256 `11e9942ed61672a7780eba6cbfb987f5e117804fdded26844e32ea90ccb8b323`.

Source/uncertainty lineage: `ru-r27`, `wp-r27`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## R-27ET (`r-27et`)

R-27ET is uncalibrated. R-27ET combines the 348 kg extended-motor airframe with scalar thermal reticle sensing, not ER SARH. It retains a distinct body length/CG and a connected-ring warhead even where charge and motor-phase inputs are shared. Retained card anchors: mass 348.0 kg (published; source wp-r27; uncertainty 5.0), length 4.49 m (derived; source ru-r27; uncertainty 0.05), diameter 0.26 m (published; source ru-r27; uncertainty 0.003), warhead 39.0 kg (published; source wp-r27; uncertainty 1.0), explosive 15.0 kg (published; source wt-datamine; uncertainty 2.25). Current receiver thermal_conscan_reticle.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.connected_ring.v1. Exact ET thermal receiver/dome response, extended-motor impulse and connected-rod expansion remain inferred; ER radar performance and base T mass cannot serve as ET calibration. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`r27et.toml`](../../../aircraft/r27et/r27et.toml). Exact current component selections:

- seeker: `equipment.r-27et.seeker` -> `fsim.seeker.nonimaging_ir.v3`; parameter SHA-256 `7cdde5ea7dee28a792b26cc52c32eaf9d5f80f9b12f41c6bf2f412b0fa355265`.
- guidance: `equipment.r-27et.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `e9c8c8f4d860f8867f3709ea7b77e4741f4a0a75e106cf2ab1b8fefce10bcec8`.
- autopilot: `equipment.r-27et.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `c23092006787b76bba7c0b7eeca4eae4a076ae648fa5e743872e46cdb5f3cc21`.
- allocator: `equipment.r-27et.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `2557e04779dd782d9b41c7907e74d686cb94dfc38a9b46814539770ba2814a00`.
- actuator: `equipment.r-27et.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `ca32950bea8b5f50b68326bdab88b947edd4ef4782c5e03b6b89f601b88419ad`.
- aerodynamics: `equipment.r-27et.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `fa30dfcaaf2dea925263d8606b7a63df6039dfb367a6c9b2da941f27fafbb48e`.
- propulsion: `equipment.r-27et.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `0edf10af95ba266d4c111280a3627a868b12a1bb85fa8d64387f48b4a3f67a1f`.
- fuze: `equipment.r-27et.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `0b057fb60a807b7aa236ec5ea59497fe3670d3b59618ddc07003e36c5dcbb383`.
- warhead: `equipment.r-27et.warhead` -> `native.warhead.connected_ring.v1`; parameter SHA-256 `6a65f01012ba423c3fb71cf5fae1dfa6fea30665d7a71d486324f85ffcc9c187`.

Source/uncertainty lineage: `native-conscan-engineering-v1`, `ru-r27`, `wp-r27`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## R-27R (`r-27r`)

R-27R is uncalibrated. R-27R uses SARH on the 253 kg standard 230 mm body with a single six-second authored motor phase and connected-ring warhead. Its native geometry/inertia differs from the extended ER while the connected topology remains distinct from discrete rod fragments. Retained card anchors: mass 253.0 kg (published; source wp-r27; uncertainty 3.0), length 4.08 m (published; source wp-r27; uncertainty 0.02), diameter 0.23 m (published; source wp-r27; uncertainty 0.002), warhead 39.0 kg (published; source wp-r27; uncertainty 1.0), explosive 15.0 kg (published; source wt-datamine; uncertainty 2.25). Current receiver rf_signal.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.connected_ring.v1. Source-classified bulk figures do not establish this R receiver/illuminator response, native control coefficients, grain burn law or ring deployment errors. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`r27r.toml`](../../../aircraft/r27r/r27r.toml). Exact current component selections:

- seeker: `equipment.r-27r.seeker` -> `fsim.seeker.semi_active_radar.v3`; parameter SHA-256 `a6c652c709fc743e91964d63ed1fc3bbcd674958e72d9bf14129af030b939ceb`.
- guidance: `equipment.r-27r.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `ff2dcc9cdfe95d038c1405424c9d99a416da1152896e48c161d722e241bb717f`.
- autopilot: `equipment.r-27r.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `47e1fc45269f0acab7941568ec1a33ea695976d352aabe9aed836035f1782ebf`.
- allocator: `equipment.r-27r.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `660bdc968f995282cc12c6d871ccefb483c99dd2a1bbd5309c3df52bb49bc75a`.
- actuator: `equipment.r-27r.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `665dd98476039a4bc3da6ff12f53dc1c1855805a6c8ef08ce850c4b080178940`.
- aerodynamics: `equipment.r-27r.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `56adf6f8a9882a3ca13af82c7eceb1fa85c8be88733c191417aa2070cd5e474c`.
- propulsion: `equipment.r-27r.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `f068eaae80654e6f39d6be031d17a405ce92d42595855bdce0b26bee9fbd8d9f`.
- fuze: `equipment.r-27r.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `5052fabe66cb9f6885575f8763a81226967af416de5fc43ac2356ebfa0a25e1f`.
- warhead: `equipment.r-27r.warhead` -> `native.warhead.connected_ring.v1`; parameter SHA-256 `3f2892a997e0b210303b068cca39fd2e8f6effba3eaf9be2b78ca7006b1d651f`.

Source/uncertainty lineage: `wp-r27`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## R-27T (`r-27t`)

R-27T is uncalibrated. R-27T retains the explicitly derived 253 kg mass and shorter IR body, single-phase motor and connected-ring warhead. The selected scalar reticle is different from the R SARH receiver, and the mass derivation is not a direct T measurement. Retained card anchors: mass 253.0 kg (derived; source ru-r27; uncertainty 15.0), length 3.795 m (derived; source ru-r27; uncertainty 0.05), diameter 0.23 m (published; source ru-r27; uncertainty 0.002), warhead 39.0 kg (published; source wp-r27; uncertainty 1.0), explosive 15.0 kg (published; source wt-datamine; uncertainty 2.25). Current receiver thermal_conscan_reticle.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.connected_ring.v1. Exact T optical mask/noise and signature discrimination, CG/inertia, impulse and connected-rod behavior remain individual inference despite shared R-27 family dimensions/charge references. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`r27t.toml`](../../../aircraft/r27t/r27t.toml). Exact current component selections:

- seeker: `equipment.r-27t.seeker` -> `fsim.seeker.nonimaging_ir.v3`; parameter SHA-256 `dd36e416a24f61471aaa5945846e4aa2f59100240dd23965fcd50c5a39ebb5e5`.
- guidance: `equipment.r-27t.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `7b49b26e3c8dd9e7d7b054068c324e9380f8c5680a59470e7e66be9e8dda7b68`.
- autopilot: `equipment.r-27t.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `39fcf8cb341558cd951d21b527ebfa5815abcd02ef0c8d6b8d5e1ef2068fb09e`.
- allocator: `equipment.r-27t.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `785d12d0cef03ba11a222f62f9a95a3a4784b69d63591ecf748e09b036becb94`.
- actuator: `equipment.r-27t.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `925be4158d5482f8c36005f78470827f831f9dbef2687530917adb0012c7dfc5`.
- aerodynamics: `equipment.r-27t.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `c58db1e75f9c09aee4704e847fb4302e5298900de081a27873318f6ac8a3074e`.
- propulsion: `equipment.r-27t.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `0d5845389741ab78863ac4f56ca8a03aedf5bff36dfbe8c8e989244d1f32512d`.
- fuze: `equipment.r-27t.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `6e592128535664f9ce92b500bed91c321ad6d9f56bf7a17216e60c419475b4f6`.
- warhead: `equipment.r-27t.warhead` -> `native.warhead.connected_ring.v1`; parameter SHA-256 `5c4bd24f272137c1eb6030b86e6b9146a4f86d7dd51e9a4f564ec310cf068fc2`.

Source/uncertainty lineage: `native-conscan-engineering-v1`, `ru-r27`, `wp-r27`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## R-37M (`r-37m`)

R-37M is uncalibrated. R-37M retains its 510 kg/380 mm body, 60 kg warhead, assumed 22.14 kg charge and commanded dual-pulse propulsion with a ten-second authored interval. Its ARH aperture, large inertia and pulse schedule are not the smaller PL-15 model. Retained card anchors: mass 510.0 kg (published; source wp-r37; uncertainty 50.0), length 4.06 m (published; source wp-r37; uncertainty 0.02), diameter 0.38 m (published; source wp-r37; uncertainty 0.005), warhead 60.0 kg (published; source wp-r37; uncertainty 2.0), explosive 22.14 kg (assumed; source unassigned; uncertainty 5.54). Current receiver rf_signal.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_commanded_dual_pulse.v1; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. Family secondary bulk reports do not calibrate M-specific impulse, seeker/electronics, grain geometry or the inferred fragmentation spectrum; maximum-range labels are not held-out trajectory measurements. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`r37m.toml`](../../../aircraft/r37m/r37m.toml). Exact current component selections:

- seeker: `equipment.r-37m.seeker` -> `fsim.seeker.active_radar.v3`; parameter SHA-256 `09f554a3fb58e9892ab71c6b747f6c818abf4e43eced0828bad905a274ca4977`.
- guidance: `equipment.r-37m.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `145f53478e16f267cb51a98cd71b09ac433109f48d0db80913f5df30e647a33c`.
- autopilot: `equipment.r-37m.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `188a0788950d5740013623a43d6b172585c9470c98ee1d4a7d71286e4ef4f0e7`.
- allocator: `equipment.r-37m.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `94ff3f1fdc29bc70cfbe8603aec47f2762da4bc860068bfb486ae2e65ca4b7c6`.
- actuator: `equipment.r-37m.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `13ab7e7d43da56a1db99df81242e846b2474f9ac5c8740f1ee25ec4dbb699652`.
- aerodynamics: `equipment.r-37m.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `f385c50a5a9081b53f4e4067bcef2626125cfea6e8e0af5da3270419ddb9fdba`.
- propulsion: `equipment.r-37m.propulsion` -> `fsim.propulsion.jsbsim_commanded_dual_pulse.v1`; parameter SHA-256 `e1c81a2d14b20ae8081e4b3f1b392bf687620bbe7003aedc2e56e3aa2ff64d68`.
- fuze: `equipment.r-37m.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `344484540137f23afcefaf4baab3e9938a48ef5d32735e7e3c8e3baa25ae5bc0`.
- warhead: `equipment.r-37m.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `336eeea078aa75bcedcc28ceac066c417b83a6c83607d40d2d18bc8c529b0fb6`.

Source/uncertainty lineage: `wp-r37`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## R-60M (`r-60m`)

R-60M is uncalibrated. R-60M uses its 45 kg/120 mm geometry, scalar reticle and a discrete rod-fragment source under zoned_case, not connected-ring deployment. The card's depleted-uranium wording does not change the existing native steel material law. Retained card anchors: mass 45.0 kg (published; source wp-r60; uncertainty 1.0), length 2.132 m (derived; source wp-r60; uncertainty 0.01), diameter 0.12 m (published; source wp-r60; uncertainty 0.002), warhead 3.5 kg (published; source wp-r60; uncertainty 0.2), explosive 1.35 kg (published; source wt-datamine; uncertainty 0.2). Current receiver thermal_conscan_reticle.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. Exact M optical mask/noise, small motor impulse, control derivatives and rod metallurgy/ballistics remain uncalibrated; material fidelity must not be inferred from the designation or narrative alone. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`r60m.toml`](../../../aircraft/r60m/r60m.toml). Exact current component selections:

- seeker: `equipment.r-60m.seeker` -> `fsim.seeker.nonimaging_ir.v3`; parameter SHA-256 `b869f7c3072b2be2816221da7742b02ef37eadf93b3766990d987bbcedb7dd57`.
- guidance: `equipment.r-60m.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `9028b995f1940e9a78f50a39447701d5b09bd9c3af7ad98820b7a41733154535`.
- autopilot: `equipment.r-60m.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `5227d388bc9184e33dc0ca6fc0f7bd3586e14e0db201c430ea5731dfbd708b81`.
- allocator: `equipment.r-60m.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `b04441c47382de36ea798d019595690429eda1eb92ec1367bf7dcc74299dd4d8`.
- actuator: `equipment.r-60m.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `1429baff1db7736318f6543a09951361898afcfadaa50594109b1f31064b9649`.
- aerodynamics: `equipment.r-60m.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `7caec400449e496282011eec85b1a89c853b273f5ea0922ed3a326b593a37f1a`.
- propulsion: `equipment.r-60m.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `c34c16ecec6e123213306adde466cd42aeb53ca116bed2981c1053b09a89a37c`.
- fuze: `equipment.r-60m.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `83a22cbc1d50da2399e30936e655a148b55f3f4c54eec56d55119beecac0a21d`.
- warhead: `equipment.r-60m.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `4308f11295ae855004590be74303310362f3274574b5912725832c3c347b3efd`.

Source/uncertainty lineage: `native-conscan-engineering-v1`, `wp-r60`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## R-73 (`r-73`)

R-73 is uncalibrated. R-73 retains its 105 kg/170 mm airframe, scalar reticle, own vectoring-nozzle/fin geometry and discrete rod source under zoned_case. Manufacturer export-family bulk data are not an exact thermal/control model or connected-ring specification. Retained card anchors: mass 105.0 kg (published; source roe-r73e; uncertainty 0.5), length 2.93 m (published; source wp-r73; uncertainty 0.01), diameter 0.165 m (published; source wp-r73; uncertainty 0.002), warhead 8.0 kg (published; source roe-r73e; uncertainty 0.5), explosive 3.75 kg (published; source wt-datamine; uncertainty 0.56). Current receiver thermal_conscan_reticle.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. The R-73E source scope, inferred detector/TVC constants, native force moments and game-reference 3.75 kg charge retain their own uncertainty; no exact domestic/export equivalence is inferred. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`r73.toml`](../../../aircraft/r73/r73.toml). Exact current component selections:

- seeker: `equipment.r-73.seeker` -> `fsim.seeker.nonimaging_ir.v3`; parameter SHA-256 `3cdc049d662ab8946df83a42e38eeeebdee04157295a97a080374636216da04e`.
- guidance: `equipment.r-73.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `fc17ca9aa7da39726476521b9c96c11516225361ffb9beb25b82d1145fc23d8f`.
- autopilot: `equipment.r-73.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `e0ef7fca18e3717364387f8ac41ec48732c0d2517100a7f70865a6164c9004bd`.
- allocator: `equipment.r-73.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `f412fdcc62a3a7ab82f4c964edd9030b0a5a58bafe8f3998e58ab84211a50e43`.
- actuator: `equipment.r-73.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `6a4c9fb40242d33a40e2add453b266ca6d108b93724015f972c0604c8907d005`.
- aerodynamics: `equipment.r-73.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `9839976502c65583a6350449975270550cadb756da36b61cbb6dee6b53069b52`.
- propulsion: `equipment.r-73.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `adc083f4850080f7553f81dd63efac2fd05b38b2271be3fa3cf064fbcd1f84c7`.
- fuze: `equipment.r-73.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `4b39dd8f507af1313258876538d000c5e24c0a34166dcb5a4209947b62e17cd5`.
- warhead: `equipment.r-73.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `1f74174d9fca580f4d0d89dde11d16862607079f77654160b3dbe4b54a5f7767`.

Source/uncertainty lineage: `native-conscan-engineering-v1`, `native-vector-engineering`, `roe-r73e`, `wp-r73`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## R-77M (`r-77m`)

R-77M is uncalibrated. R-77M uses the bounded operating-point allocator and conventional-fin geometry with a declared three-segment generated-solid-motor schedule. Its 190 kg mass, 3.71 m length and 22.5 kg warhead are assumptions; neither R-77-1 dimensions nor a dual-pulse narrative supplies M calibration. Retained card anchors: mass 190.0 kg (assumed; source unassigned; uncertainty 25.0), length 3.71 m (assumed; source unassigned; uncertainty 0.25), diameter 0.2 m (published; source wp-r77; uncertainty 0.01), warhead 22.5 kg (assumed; source unassigned; uncertainty 2.0), explosive 9.725 kg (published; source wt-datamine; uncertainty 1.46). Current receiver rf_signal.v1; control fsim.control.operating_point_bounded.v1; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. FOI identity discussion and secondary/game references do not measure M fin curves, actuator states, actual motor scheduling or the discrete rod source. Correct one-step allocation is distinct from full-chain behavior and equipment accuracy. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`r77m.toml`](../../../aircraft/r77m/r77m.toml). Exact current component selections:

- seeker: `equipment.r-77m.seeker` -> `fsim.seeker.active_radar.v3`; parameter SHA-256 `79fb56394f6bfb4ca13cbd91775cba966eab767476abb1c7d56e672b81a5d5df`.
- guidance: `equipment.r-77m.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `a51a5dd91dacc55092705648a090910a7cfacb75adbef4beab297bb59e504ea7`.
- autopilot: `equipment.r-77m.autopilot` -> `fsim.control.operating_point_bounded.v1`; parameter SHA-256 `145cdc2165c7e89dc44c7caeadbd5595c0e2df0d4f9debcc2d547f72866e5282`.
- allocator: `equipment.r-77m.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `f4676b604f6526e6339009a7160e57aaeb566fc96e54ff2ed230648de511adac`.
- actuator: `equipment.r-77m.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `22353a03bc6eeb59f304d0e40675073be851f79c609016a758e3fb4fc82f5d43`.
- aerodynamics: `equipment.r-77m.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `afe703137aea1c1097fae1b31f5ba52021b3728fc386c7deb041f4d8c12076f3`.
- propulsion: `equipment.r-77m.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `d27774a6a1567e3580398a6c0402d174dc6eea801726078e9e6379eb4fb45cc1`.
- fuze: `equipment.r-77m.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `152fda5406066e22ebbe0ee3ff8442877b4c4b42076a62ed9d0d23aa8e6ccf5b`.
- warhead: `equipment.r-77m.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `2021298e53e0dd2ad34149d7aa8a3af2eca8d8cfc1d036ec74350b27b7a02886`.

Source/uncertainty lineage: `wp-r77`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.

## Super 530D (`super-530d`)

Super 530D is uncalibrated. Super 530D retains its 270 kg/263 mm airframe, SARH path, two-second boost/four-second sustain schedule and 30 kg zoned-case warhead. Neither MICA shared manufacturer history nor another SARH model establishes this receiver/motor/control response. Retained card anchors: mass 270.0 kg (published; source wp-super530; uncertainty 3.0), length 3.8 m (published; source wp-super530; uncertainty 0.02), diameter 0.263 m (published; source wp-super530; uncertainty 0.002), warhead 30.0 kg (published; source wp-super530; uncertainty 2.0), explosive 8.1 kg (published; source wt-datamine; uncertainty 1.21). Current receiver rf_signal.v1; control fsim.control.load_feedback_cascade; propulsion fsim.propulsion.jsbsim_generated_solid_rocket; fuze native.fuze.swept_path.v1; warhead native.warhead.zoned_case.v1. Its secondary bulk and game-reference 8.1 kg charge records remain distinct from inferred RF noise, finite actuator/airframe dynamics and fragment-spectrum assumptions. Source classes and uncertainty records remain unchanged; assumed, derived and game-reference values are not promoted to equipment measurements. Per-model theory, parameter deltas and numerical method routes are in docs/research/sim-m03/component-methods/per-model-effects-components.json and per-model-six-components.json; the separate docs/research/sim-m03/calibration-ledger.json identifies the remaining equipment comparisons. Arithmetic or native scenario verification does not establish held-out accuracy. No admitted same-model independent baseline-to-new held-out calibration receipt is claimed; engineering completion continues through its own gates.

Geometry/propulsion: [`super530d.toml`](../../../aircraft/super530d/super530d.toml). Exact current component selections:

- seeker: `equipment.super-530d.seeker` -> `fsim.seeker.semi_active_radar.v3`; parameter SHA-256 `f01a6f21bff7a2f15e08a54de3226bd2e5e092822a054dc899e624c9198ba860`.
- guidance: `equipment.super-530d.guidance` -> `fsim.guidance.measured_pn.v2`; parameter SHA-256 `beef5eab461a26e157cb837a0b62920596e4290d3d48d6029c254dd9d9702c59`.
- autopilot: `equipment.super-530d.autopilot` -> `fsim.control.load_feedback_cascade`; parameter SHA-256 `ceeb2f8d174fd1f6a9fa720226f8d3faf676012a04713137e17424dbf5a7a505`.
- allocator: `equipment.super-530d.allocator` -> `fsim.allocator.jsbsim_authored_fin_mixer`; parameter SHA-256 `f14816489bb552452af9328c9f3c81d6b651804483f83d02df0b4e80c985d821`.
- actuator: `equipment.super-530d.actuator` -> `fsim.actuator.jsbsim_lag_rate_stops`; parameter SHA-256 `e33ba0eda572c2369ec4faf39ddeb69f936a0a265adf85b94b0dcf4ad0029a62`.
- aerodynamics: `equipment.super-530d.aerodynamics` -> `fsim.aerodynamics.jsbsim_generated_store`; parameter SHA-256 `462d66f2a9ae7a3695c1cc9e3623caea7866df5112fde81e0aae910ba90f6b48`.
- propulsion: `equipment.super-530d.propulsion` -> `fsim.propulsion.jsbsim_generated_solid_rocket`; parameter SHA-256 `87d2e886b99da089a0b86aa08334f06a40a31655ef1520a3fba5049dbfc5d333`.
- fuze: `equipment.super-530d.fuze` -> `native.fuze.swept_path.v1`; parameter SHA-256 `af3c8a5f620b7f0d6f37fc5234a362be558e28d439f3662081185cde1ae2aa01`.
- warhead: `equipment.super-530d.warhead` -> `native.warhead.zoned_case.v1`; parameter SHA-256 `681f440b146a1e430c164b425b0e946a686f046c4fcc80a8f7a877a6cadb4290`.

Source/uncertainty lineage: `wp-super530`, `wt-datamine`. The JSON ledger retains the full referenced source records and bulk parameter uncertainties; unassigned source IDs remain explicitly unresolved.
