# Parameter and source matrix: 24 models x 9 components

This standalone projection has exactly 216 unique model/component keys. Documentary coverage is 216; current component acceptances are 0. It does not turn written theory, a shared constant, a historical numerical reference, or a source hash into a current native/installed/visual or equipment-calibration PASS.

The complete machine-readable companion is [parameter-source-matrix.json](parameter-source-matrix.json). Each row retains the exact source-file hash, canonical source-row hash and JSON pointer, mechanism, card identity and canonical descriptor/parameter hash. The JSON embeds each actual model plant and family state/I/O/discretization contract, so its derivations do not require a separate matrix to interpret.

## Source snapshot and reproduction

- `docs/research/sim-m03/component-methods/per-model-six-components.json`: SHA-256 `cfed99ab22820b2a0e3a502a5f54eb61e9c2111400ba5f27b8e92f56aebd7c86`.
- `docs/research/sim-m03/component-methods/per-model-effects-components.json`: SHA-256 `8b908b7687c8cb244136810d0d506d318f9d43c960b40a7fecf170acc70edb0e`.

Run `D:/ENV/python/3.13/python.exe -B D:/ENV/tmp/m03-project-three-216-matrices-20261103.py` using the recorded DENV environment. The script verifies 216-key uniqueness/completeness, exact current card/role hashes, role-record preservation, source-row identities and unchanged read inputs. It writes only these six matrix documents. The source-row documents remain frozen.

Source hashes are snapshots. The JSON explicitly records any source whose current working-tree hash differs from its original row snapshot. A later physical/source change requires an owning-table refresh and new actual verification; historical results retain their original epochs.

## Parameter interpretation

Every selected role contains its original parameter ID and complete value/unit/basis/uncertainty/note/source record. Explicit source identifiers are resolved against the same card metadata; absent identifiers remain absent. Published identity, generic physical-method literature, simulated aero values, assumed coefficients and game-reference fragment shares retain their separate stated bases. An identity citation is never promoted to evidence for a detector constant or proprietary control gain.

Allocator/aerodynamics rows with zero descriptor parameter roles still retain their actual model geometry, native reference dimensions, fin mixer, tables, mass/CG/inertia and serialized native resources. Screening uncertainty is not fitted accuracy, a probability interval, or an equipment calibration result.

## All 216 keyed parameter projections

| Model/component | Exact mechanism | Selected role count | Original basis counts |
|---|---|---:|---|
| aim-120c/seeker | fsim.seeker.active_radar.v3 | 87 | {"assumed": 87} |
| aim-120c/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| aim-120c/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| aim-120c/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| aim-120c/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| aim-120c/fuze | native.fuze.turning_cpa.v1 | 11 | {"assumed": 9, "published": 2} |
| aim-120c/warhead | native.warhead.finite_zoned_case.v2 | 17 | {"assumed": 14, "published": 3} |
| aim-120c/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 18 | {"assumed": 18} |
| aim-120c/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| aim-120d/seeker | fsim.seeker.active_radar.v3 | 87 | {"assumed": 87} |
| aim-120d/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| aim-120d/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| aim-120d/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| aim-120d/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| aim-120d/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| aim-120d/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 1, "published": 3} |
| aim-120d/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 18 | {"assumed": 18} |
| aim-120d/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| aim-7m/seeker | fsim.seeker.semi_active_radar.v3 | 87 | {"assumed": 87} |
| aim-7m/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| aim-7m/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| aim-7m/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| aim-7m/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| aim-7m/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| aim-7m/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 1, "published": 3} |
| aim-7m/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 18 | {"assumed": 18} |
| aim-7m/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| aim-9m/seeker | fsim.seeker.nonimaging_ir.v3 | 92 | {"assumed": 92} |
| aim-9m/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| aim-9m/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| aim-9m/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| aim-9m/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| aim-9m/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| aim-9m/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 1, "derived": 1, "published": 2} |
| aim-9m/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 16 | {"assumed": 16} |
| aim-9m/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| aim-9x/seeker | fsim.seeker.imaging_ir.v3 | 86 | {"assumed": 86} |
| aim-9x/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| aim-9x/autopilot | fsim.control.load_feedback_cascade | 7 | {"assumed": 7} |
| aim-9x/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| aim-9x/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| aim-9x/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| aim-9x/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 1, "published": 3} |
| aim-9x/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 16 | {"assumed": 16} |
| aim-9x/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| asraam/seeker | fsim.seeker.imaging_ir.v3 | 86 | {"assumed": 86} |
| asraam/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| asraam/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| asraam/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| asraam/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| asraam/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| asraam/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 2, "published": 2} |
| asraam/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 18 | {"assumed": 18} |
| asraam/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| iris-t/seeker | fsim.seeker.imaging_ir.v3 | 86 | {"assumed": 86} |
| iris-t/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| iris-t/autopilot | fsim.control.load_feedback_cascade | 7 | {"assumed": 7} |
| iris-t/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| iris-t/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| iris-t/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| iris-t/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 2, "published": 2} |
| iris-t/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 16 | {"assumed": 16} |
| iris-t/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| magic-2/seeker | fsim.seeker.nonimaging_ir.v3 | 92 | {"assumed": 92} |
| magic-2/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| magic-2/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| magic-2/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| magic-2/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| magic-2/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| magic-2/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 1, "published": 3} |
| magic-2/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 16 | {"assumed": 16} |
| magic-2/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| meteor/seeker | fsim.seeker.active_radar.v3 | 87 | {"assumed": 87} |
| meteor/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| meteor/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| meteor/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| meteor/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| meteor/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| meteor/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 2, "published": 2} |
| meteor/propulsion | fsim.propulsion.jsbsim_ducted_rocket.v2 | 34 | {"assumed": 34} |
| meteor/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| mica-em/seeker | fsim.seeker.active_radar.v3 | 87 | {"assumed": 87} |
| mica-em/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| mica-em/autopilot | fsim.control.load_feedback_cascade | 7 | {"assumed": 7} |
| mica-em/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| mica-em/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| mica-em/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| mica-em/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 1, "published": 3} |
| mica-em/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 16 | {"assumed": 16} |
| mica-em/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| mica-ir/seeker | fsim.seeker.imaging_ir.v3 | 111 | {"assumed": 111} |
| mica-ir/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| mica-ir/autopilot | fsim.control.load_feedback_cascade | 7 | {"assumed": 7} |
| mica-ir/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| mica-ir/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| mica-ir/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| mica-ir/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 1, "published": 3} |
| mica-ir/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 16 | {"assumed": 16} |
| mica-ir/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| pl-10/seeker | fsim.seeker.imaging_ir.v3 | 86 | {"assumed": 86} |
| pl-10/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| pl-10/autopilot | fsim.control.actuator_aware_cascade.v1 | 7 | {"assumed": 7} |
| pl-10/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| pl-10/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| pl-10/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| pl-10/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 4} |
| pl-10/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 16 | {"assumed": 16} |
| pl-10/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| pl-12/seeker | fsim.seeker.active_radar.v3 | 87 | {"assumed": 87} |
| pl-12/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| pl-12/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| pl-12/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| pl-12/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| pl-12/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| pl-12/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 2, "published": 2} |
| pl-12/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 18 | {"assumed": 18} |
| pl-12/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| pl-15/seeker | fsim.seeker.active_radar.v3 | 87 | {"assumed": 87} |
| pl-15/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| pl-15/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| pl-15/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| pl-15/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| pl-15/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| pl-15/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 3, "published": 1} |
| pl-15/propulsion | fsim.propulsion.jsbsim_commanded_dual_pulse.v1 | 23 | {"assumed": 23} |
| pl-15/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| pl-8/seeker | fsim.seeker.nonimaging_ir.v3 | 92 | {"assumed": 92} |
| pl-8/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| pl-8/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| pl-8/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| pl-8/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| pl-8/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| pl-8/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 1, "published": 3} |
| pl-8/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 16 | {"assumed": 16} |
| pl-8/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| r-27er/seeker | fsim.seeker.semi_active_radar.v3 | 87 | {"assumed": 87} |
| r-27er/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| r-27er/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| r-27er/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| r-27er/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| r-27er/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| r-27er/warhead | native.warhead.connected_ring.v1 | 4 | {"assumed": 1, "published": 3} |
| r-27er/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 18 | {"assumed": 18} |
| r-27er/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| r-27et/seeker | fsim.seeker.nonimaging_ir.v3 | 92 | {"assumed": 92} |
| r-27et/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| r-27et/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| r-27et/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| r-27et/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| r-27et/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| r-27et/warhead | native.warhead.connected_ring.v1 | 4 | {"assumed": 1, "published": 3} |
| r-27et/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 18 | {"assumed": 18} |
| r-27et/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| r-27r/seeker | fsim.seeker.semi_active_radar.v3 | 87 | {"assumed": 87} |
| r-27r/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| r-27r/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| r-27r/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| r-27r/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| r-27r/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| r-27r/warhead | native.warhead.connected_ring.v1 | 4 | {"assumed": 1, "published": 3} |
| r-27r/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 16 | {"assumed": 16} |
| r-27r/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| r-27t/seeker | fsim.seeker.nonimaging_ir.v3 | 92 | {"assumed": 92} |
| r-27t/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| r-27t/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| r-27t/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| r-27t/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| r-27t/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| r-27t/warhead | native.warhead.connected_ring.v1 | 4 | {"assumed": 1, "published": 3} |
| r-27t/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 16 | {"assumed": 16} |
| r-27t/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| r-37m/seeker | fsim.seeker.active_radar.v3 | 87 | {"assumed": 87} |
| r-37m/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| r-37m/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| r-37m/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| r-37m/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| r-37m/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| r-37m/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 2, "published": 2} |
| r-37m/propulsion | fsim.propulsion.jsbsim_commanded_dual_pulse.v1 | 23 | {"assumed": 23} |
| r-37m/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| r-60m/seeker | fsim.seeker.nonimaging_ir.v3 | 92 | {"assumed": 92} |
| r-60m/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| r-60m/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| r-60m/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| r-60m/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| r-60m/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| r-60m/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 1, "published": 3} |
| r-60m/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 16 | {"assumed": 16} |
| r-60m/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| r-73/seeker | fsim.seeker.nonimaging_ir.v3 | 92 | {"assumed": 92} |
| r-73/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| r-73/autopilot | fsim.control.load_feedback_cascade | 7 | {"assumed": 7} |
| r-73/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| r-73/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| r-73/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| r-73/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 1, "published": 3} |
| r-73/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 16 | {"assumed": 16} |
| r-73/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| r-77m/seeker | fsim.seeker.active_radar.v3 | 87 | {"assumed": 87} |
| r-77m/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| r-77m/autopilot | fsim.control.operating_point_bounded.v1 | 5 | {"assumed": 5} |
| r-77m/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| r-77m/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| r-77m/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| r-77m/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 2, "published": 2} |
| r-77m/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 20 | {"assumed": 20} |
| r-77m/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |
| super-530d/seeker | fsim.seeker.semi_active_radar.v3 | 87 | {"assumed": 87} |
| super-530d/guidance | fsim.guidance.measured_pn.v2 | 9 | {"assumed": 9} |
| super-530d/autopilot | fsim.control.load_feedback_cascade | 5 | {"assumed": 5} |
| super-530d/allocator | fsim.allocator.jsbsim_authored_fin_mixer | 0 | {} |
| super-530d/actuator | fsim.actuator.jsbsim_lag_rate_stops | 2 | {"assumed": 2} |
| super-530d/fuze | native.fuze.swept_path.v1 | 5 | {"assumed": 5} |
| super-530d/warhead | native.warhead.zoned_case.v1 | 4 | {"assumed": 1, "published": 3} |
| super-530d/propulsion | fsim.propulsion.jsbsim_generated_solid_rocket | 18 | {"assumed": 18} |
| super-530d/aerodynamics | fsim.aerodynamics.jsbsim_generated_store | 0 | {} |

Every key above has its exact card SHA, parameter SHA, source-row SHA/pointer, model plant, law-equivalence boundaries and source references in the matching JSON row. No current PASS is inferred from this document.
