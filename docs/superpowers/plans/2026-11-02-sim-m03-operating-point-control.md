# Operating-point bounded control

Main baseline: `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`.

The exclusive window owns the exact source paths communicated to the delivery
owner: new private weapon/OperatingPointControlV1.h/.cpp; new sibling
components/interfaces/OperatingPointControlV1.h; new autopilot implementation
OperatingPointAutopilotV1.h/.cpp; AutopilotProvidersV1.cpp; PlantAdapter.cpp; PlantProvidersV1.h/.cpp additive nonvirtual orchestration;
Hangar storecontrol.py/components.py; R-77M selector only; new
test_weapon_operating_point.cpp; test_storecontrol.py; test_components.py retained-provider fixture only;
test_payload_plant_adapters.cpp; World.h private cache and World.cpp private
binding/preStep/reset seams; WorldWeaponControls.cpp metadata-version condition only; this plan and the new method document. Root owns
CMake registration and serial native execution. Existing provider/input layouts,
vtable signatures, aerodynamic tables, physical scalars and controller gains are
retained.

The frozen R-77M trace enters non-monotone fin-control intervals under combined
rudder/aileron demand before seeker loss. Neutral-fin roll derivatives do not
describe that operating point. At physical stops the two directional derivatives
differ; substituting a signed scalar gain cannot solve this problem.

New explicit selection `fsim.control.operating_point_bounded.v1` consumes full
piecewise-linear physical-fin moment curves at current native incidence/Mach,
signed authored mixer, actual finite actuator states, stops and rates. The
controller predicts native lag/rate/mixer/fin-stop execution over one held World
native step and minimizes requested angular-acceleration error through bounded
piecewise allocation. Every singular/unreachable request remains a finite
physical command with a reported residual; no inverse divides by vanishing
effectiveness. Deterministic finite line searches and bounded multi-start
coordinate iterations do not claim a global nonlinear optimum. Existing
response times define the request, without retuning. Only the justified current
R-77M card selects this new version.

Tests first: exact frozen combined-fin differential and directional saturation,
coupled signed mixer, healthy neutral moment agreement, finite lag/rate/stops,
nonfinite input, singular/unreachable authority, and same frozen native encounter.
Root captures RED before production, then owns native/public/fleet GREEN.

Production numerical strategy: bounded sequential linearization with actual prior
input plus the ten retained seeds, four iterations each, directional box least
squares solved by pivoted Householder QR, scalar-native merit backtracking and
finite polls. The exhaustive tracer/PWL implementation remains a private reference;
its global branch sweep was an implementation choice, not a physical acceptance
requirement. Physical integration resolution, native curves, original cascade
response constants and frozen encounter predicates remain unchanged. Independent
linear/coupled/direction/native-endpoint/work-bound tests and measured whole-scene
1x execution are required alongside reference equivalence.
