# Documentation index

Start with [current status](status.md) for implemented capabilities, milestone boundaries, recorded checks and known limits. This index identifies each document's role; it does not maintain a second status table. The [root README](../README.md) covers the shortest build, run and packaging paths.

## Current guides

| Guide | Use it for |
| --- | --- |
| [Development](development.md) | Environment, build variants, contribution rules and validation |
| [Resources](resources.md) | Authoritative inputs, generation steps, generated files and runtime consumers |
| [Hangar](hangar.md) | Aircraft and store design pipeline, mass accounting, calibration and rebuilds |
| [Testbench](../testbench/README.md) | Local scenario UI, headless suites and flight-stability checks |
| [Map assets](../assets/config/maps.README.md) | Offline map plans, tile downloads, coverage and package layout |
| [Internal type codes](type-codes.md) | Short names used in development and domain discussions |

## API references

The [SDK index](sdk/README.md) is the entrypoint for C++, C and Python consumers. It links world/vehicle lifecycle, controls, environment, agents, scenarios, mission systems, viewer, vision and batched training interfaces.

- [Engagement contract](sdk/engagement.md): native weapon lifecycle, seeker support, countermeasures, events, access and assumed model parameters.
- [SDK authority boundary](sdk/boundary.md): generated classification of exported interfaces; [boundary.toml](sdk/boundary.toml) is its classification input.
- Public headers in [`include/fsim/`](../include/fsim/) and the C ABI in [`fsim_c.h`](../include/fsim/fsim_c.h) are the code contracts behind the reference pages.

## Design decisions and planning

These documents explain rationale, intended contracts and domain plans. A roadmap entry or accepted design is not runtime verification; use [status](status.md) and the linked evidence to determine what has been demonstrated.

| Document | Role |
| --- | --- |
| [System architecture and design](FlightSim_System_Architecture_and_Design.md) | Overall design, requirements, module relationships, performance targets, milestone history and ADR register |
| [Control architecture (ADR-26)](control-architecture.md) | Capability contracts over the control runtime |
| [Rotorcraft (ADR-27)](rotorcraft.md) | Rotorcraft decisions and capability integration |
| [Vehicle Interface (ADR-28)](vehicle-interface.md) | Flight semantics and the public control interface |
| [Flight Autonomy (ADR-29)](flight-autonomy.md) | Native autonomy stages and their design/acceptance rationale |
| [Mission Systems (ADR-30)](mission-systems.md) | Payload, sensor and weapon design with domain milestone planning |
| [Agents and authority (ADR-31)](agent-boundary.md) | Mission autonomy, platform and range authority decisions |
| [Composite MA and training design](superpowers/specs/2026-10-12-ma-system-training-readiness-design.md) | Accepted upper-layer MA ownership, P2P/C2 boundaries, independent episode and training contracts |
| [Claude MA implementation plan](superpowers/plans/2026-10-12-ma-system-training-readiness.md) | Claude vertical implementation packages, required inputs, verification and integration gates |
| [Claude MA work allocation](superpowers/plans/2026-10-12-ma-system-training-work-allocation.md) | Complete Claude feature assignments, file allowlists, handoffs and first delivery package |
| [Backlog](backlog.md) | Measured findings, priorities and closure conditions |
| [SDK missile engagement design](superpowers/specs/2026-10-11-sdk-missile-engagement-design.md) | Dated implementation specification and its starting assumptions |
| [Architecture A1 maintenance](superpowers/specs/2026-10-11-architecture-maintenance-design.md) | Third-pass findings, initial boundary/domain refactor and prioritized remaining architecture work |
| [Architecture A2 maintenance](superpowers/specs/2026-10-12-architecture-lifecycle-design.md) | MA retirement with retained public history and C/Python native DMS/C2 ownership |
| [Architecture A3 maintenance](superpowers/specs/2026-10-12-architecture-adapters-design.md) | C control/VI and Python control/Mission ownership with unchanged public contracts |
| [Architecture A4 maintenance](superpowers/specs/2026-10-12-architecture-world-adapters-design.md) | World, Scenario and Environment adapters with retained runtime and registration ownership |
| [Architecture A5 maintenance](superpowers/specs/2026-10-12-architecture-batch-recording-design.md) | VecEnv and Recording adapters with retained C++ wrappers, types and zero-copy ownership |
| [Architecture A6 maintenance](superpowers/specs/2026-10-12-architecture-store-binding-design.md) | World store-to-flight-model binding state with retained lifecycle, weighing cadence and simulation order |
| [Native dependency acceptance](../tools/architecture/README.md) | Executable module policy, checker scope and regression fixtures |

## Recorded and generated evidence

Evidence applies to the date, build, scenario and scope it records. Source inspection, generated inventory, passing tests and an observed runtime session establish different things; none implies that every configuration or aircraft is validated.

| Evidence | What it records |
| --- | --- |
| [Engagement delivery validation](sdk/engagement-validation.md) | Dated local build/regression and native/browser encounter results, reproduction paths and exclusions |
| [Native viewer validation](sdk/viewer-validation.md) | Dated fullscreen native GPU checks, renderer lifecycle regressions, scene and performance evidence, and exclusions |
| [Stores build report](hangar-stores-build.md), [JSON record](hangar-stores-build.json) | Store pipeline outputs, sizes and timings from a recorded generation run |
| [Model effects inventory](hangar-effects.md) | Generated visual nodes and effect metadata available to consumers |
| [Flight-stability results](../testbench/results/stability/SUMMARY.md) | Per-aircraft recorded checks, with raw JSON beside the summary |

For new results, preserve the configuration and limitations with the evidence and update [status](status.md) to link it. The [development guide](development.md) defines validation expectations; the [resource guide](resources.md) defines when generated inventories need refreshing.
