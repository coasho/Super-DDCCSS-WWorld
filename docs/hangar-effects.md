# What each model gives the viewer's effects

Written by `python -m hangar effects --record docs/hangar-effects.md` from the designs, their models' nodes and
their manifests ([viewer.md](sdk/viewer.md#effects)): what the viewer has to draw each vehicle's and each payload's
moving parts, plumes and trails from. The simulation supplies the state; this audit checks model anchors and
manifest data, not rendered appearance or flight behavior. `tools/hangar/tests/test_effects.py` checks the
current rules' `missing` lists.

**Current audit exclusions.** Rotorcraft without exhaust anchors are reported in `what it gives`, not counted
as `missing`; rotorcraft also bypass the lifting-surface requirement. Rotorcraft exhaust, rotor wake and tip
vortices remain [B-59](backlog.md). Propeller heat haze and tip vortices, and a released store's continuing
wreck fire, are also outside these inventory checks. A zero `missing` count means zero failures under these
rules, not complete physical-effects coverage or a fresh visual acceptance run.

| design | category | engines | what it gives | missing |
| --- | --- | --- | --- | --- |
| `463lpallet` | store | - | a rigid body: nothing on it moves or burns | - |
| `a10c` | attack | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron, elevator, flaps, lef, rudder, speedbrake; gear; wing vapour and vortices | - |
| `a10tank600gal` | store | - | a rigid body: nothing on it moves or burns | - |
| `aim120c` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `aim120d` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `aim7m` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `aim9m` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+rudder; turned by the airstream: rolleron | - |
| `aim9x` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `anaaq28` | store | - | a rigid body: nothing on it moves or burns | - |
| `analq131` | store | - | a rigid body: nothing on it moves or burns | - |
| `analq184` | store | - | a rigid body: nothing on it moves or burns | - |
| `analq99` | store | - | a rigid body: nothing on it moves or burns | - |
| `anasq228` | store | - | a rigid body: nothing on it moves or burns | - |
| `asraam` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `b52h` | bomber | turbofan x8 | exhaust: 8 nozzle(s); moving surfaces: aileron, elevator, flaps, rudder, speedbrake; gear; weapon-bay doors; wing vapour and vortices | - |
| `c130j` | transport | turboprop x4 | exhaust: 0 nozzle(s), 4 placed by its manifest; 4 propeller(s) or rotor(s) turning at their engines' rpm; moving surfaces: aileron, elevator, flaps, rudder; gear; wing vapour and vortices | - |
| `c172` | light_ga | piston x1 | exhaust: 0 nozzle(s), 1 placed by its manifest; 1 propeller(s) or rotor(s) turning at their engines' rpm; moving surfaces: aileron, elevator, flaps, rudder; wing vapour and vortices | - |
| `c17a` | transport | turbofan x4 | exhaust: 4 nozzle(s); moving surfaces: aileron, elevator, flaps, rudder, speedbrake; gear; wing vapour and vortices | - |
| `cdsa22` | store | - | a rigid body: nothing on it moves or burns | - |
| `cf2` | multirotor | - | 4 propeller(s) or rotor(s) turning at their engines' rpm | - |
| `cj10a` | store | - | a rigid body: nothing on it moves or burns | - |
| `damocles` | store | - | a rigid body: nothing on it moves or burns | - |
| `e3g` | transport | turbofan x4 | exhaust: 4 nozzle(s); moving surfaces: aileron, elevator, flaps, rudder, speedbrake; gear; wing vapour and vortices | - |
| `e7a` | aew | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron, elevator, flaps, rudder, speedbrake; gear; wing vapour and vortices | - |
| `ea18g` | fighter | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron, elevator+aileron, flaps+speedbrake, lef, rudder+speedbrake; gear; wing vapour and vortices | - |
| `ec130h` | electronic warfare | turboprop x4 | exhaust: 0 nozzle(s), 4 placed by its manifest; 4 propeller(s) or rotor(s) turning at their engines' rpm; moving surfaces: aileron, elevator, flaps, rudder; gear; wing vapour and vortices | - |
| `f15c` | fighter | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron, elevator+aileron, flaps, rudder, speedbrake; gear; wing vapour and vortices | - |
| `f16c` | fighter | turbofan x1 | exhaust: 1 nozzle(s); moving surfaces: aileron+flaps, elevator+aileron, lef, rudder, speedbrake; gear; wing vapour and vortices | - |
| `f22a` | fighter | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron+flaps, aileron+speedbrake, elevator, elevator+aileron, lef, rudder+speedbrake; gear; weapon-bay doors; wing vapour and vortices | - |
| `f22tank600gal` | store | - | a rigid body: nothing on it moves or burns | - |
| `f35a` | fighter | turbofan x1 | exhaust: 1 nozzle(s); moving surfaces: aileron+flaps, elevator+aileron, lef, rudder+speedbrake; gear; weapon-bay doors; wing vapour and vortices | - |
| `fa18c` | fighter | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron, elevator+aileron, flaps, lef, rudder, speedbrake; gear; wing vapour and vortices | - |
| `fpu11` | store | - | a rigid body: nothing on it moves or burns | - |
| `fpu8` | store | - | a rigid body: nothing on it moves or burns | - |
| `gripen` | fighter | turbofan x1 | exhaust: 1 nozzle(s); moving surfaces: elevator, elevator+aileron, lef, rudder, speedbrake; gear; wing vapour and vortices | - |
| `gripentank1100l` | store | - | a rigid body: nothing on it moves or burns | - |
| `h6k` | bomber | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron, elevator, flaps, rudder; gear; wing vapour and vortices | - |
| `iris` | multirotor | - | 4 propeller(s) or rotor(s) turning at their engines' rpm | - |
| `irist` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `j10a` | fighter | turbofan x1 | exhaust: 1 nozzle(s); moving surfaces: elevator, elevator+aileron, lef, rudder, speedbrake; gear; wing vapour and vortices | - |
| `j10tank1600l` | store | - | a rigid body: nothing on it moves or burns | - |
| `j10tank800l` | store | - | a rigid body: nothing on it moves or burns | - |
| `j20a` | fighter | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron+flaps, elevator, elevator+aileron, lef, rudder; gear; weapon-bay doors; wing vapour and vortices | - |
| `kc135r` | transport | turbofan x4 | exhaust: 4 nozzle(s); moving surfaces: aileron, elevator, flaps, rudder, speedbrake; gear; wing vapour and vortices | - |
| `kc46a` | transport | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron, elevator, flaps, rudder, speedbrake; gear; wing vapour and vortices | - |
| `magic2` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+rudder; turned by the airstream: freeroll | - |
| `meteor` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `micaem` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `micair` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `mig29a` | fighter | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron, elevator+aileron, flaps, lef, rudder, speedbrake; gear; wing vapour and vortices | - |
| `mirage2000` | fighter | turbofan x1 | exhaust: 1 nozzle(s); moving surfaces: elevator+aileron, lef, rudder, speedbrake; gear; wing vapour and vortices | - |
| `mk82` | store | - | a rigid body: nothing on it moves or burns | - |
| `pl10` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `pl12` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `pl15` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `pl8` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: aileron, elevator+rudder | - |
| `ptb1150` | store | - | a rigid body: nothing on it moves or burns | - |
| `ptb1500` | store | - | a rigid body: nothing on it moves or burns | - |
| `ptb2000` | store | - | a rigid body: nothing on it moves or burns | - |
| `ptb800` | store | - | a rigid body: nothing on it moves or burns | - |
| `r27er` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: aileron, elevator+aileron | - |
| `r27et` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: aileron, elevator+aileron | - |
| `r27r` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: aileron, elevator+aileron | - |
| `r27t` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: aileron, elevator+aileron | - |
| `r37m` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `r60m` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+rudder | - |
| `r73` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: aileron, elevator+rudder | - |
| `r77m` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `rafale` | fighter | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: elevator+aileron, elevator+speedbrake, lef, rudder; gear; wing vapour and vortices | - |
| `rc135w` | transport | turbofan x4 | exhaust: 4 nozzle(s); moving surfaces: aileron, elevator, flaps, rudder, speedbrake; gear; wing vapour and vortices | - |
| `rpl522` | store | - | a rigid body: nothing on it moves or burns | - |
| `rpl541` | store | - | a rigid body: nothing on it moves or burns | - |
| `rpl701` | store | - | a rigid body: nothing on it moves or burns | - |
| `rpl751` | store | - | a rigid body: nothing on it moves or burns | - |
| `rq4b` | reconnaissance | turbofan x1 | exhaust: 1 nozzle(s); moving surfaces: aileron, elevator+rudder, flaps, speedbrake; gear; wing vapour and vortices | - |
| `skua` | uav | electric x1 | 1 propeller(s) or rotor(s) turning at their engines' rpm; moving surfaces: aileron, elevator, flaps, rudder; wing vapour and vortices | - |
| `su25` | attack | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron, elevator, flaps, lef, rudder, speedbrake; gear; wing vapour and vortices | - |
| `su27s` | fighter | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron+flaps, elevator+aileron, lef, rudder, speedbrake; gear; wing vapour and vortices | - |
| `su57` | fighter | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: aileron+flaps, aileron+speedbrake, elevator, elevator+aileron, lef, rudder+speedbrake; gear; weapon-bay doors; wing vapour and vortices | - |
| `super530d` | missile |  rocket motor | a rocket's plume and smoke trail (its nozzle, its `rocket` line); moving surfaces: elevator+aileron | - |
| `tank300gal` | store | - | a rigid body: nothing on it moves or burns | - |
| `tank370gal` | store | - | a rigid body: nothing on it moves or burns | - |
| `tank610gal` | store | - | a rigid body: nothing on it moves or burns | - |
| `typhoon` | fighter | turbofan x2 | exhaust: 2 nozzle(s); moving surfaces: elevator, elevator+aileron, lef, rudder, speedbrake; gear; wing vapour and vortices | - |
| `typhoontank1000l` | store | - | a rigid body: nothing on it moves or burns | - |
| `u2s` | reconnaissance | turbofan x1 | exhaust: 1 nozzle(s); moving surfaces: aileron, elevator, flaps, rudder, speedbrake; gear; wing vapour and vortices | - |
| `uh1h` | helicopter | turboshaft x1 | no exhaust anchor (a rotorcraft's own pipeline: docs/backlog.md, B-59); 2 propeller(s) or rotor(s) turning at their engines' rpm; moving surfaces: elevator | - |
| `uh60` | helicopter | turboshaft x1 | no exhaust anchor (a rotorcraft's own pipeline: docs/backlog.md, B-59); 2 propeller(s) or rotor(s) turning at their engines' rpm; moving surfaces: elevator | - |

88 designs; 0 with something missing.
