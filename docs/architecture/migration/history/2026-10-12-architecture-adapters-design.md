# Architecture A3: control and Mission adapter ownership

## Intent and baseline

A2 is committed as `34e5516`. A3 continues from that baseline, splitting the C/Python control
and Mission adapters while preserving public API/ABI, principal permissions and simulation behavior. Files use
English. This slice changes adapter source ownership only.

The C Mission owner already exists as `src/sdk/fsim_mission_c.cpp`. Retain it and the A2 DMS/C2 owners unchanged.
Create a C control owner and matching Python control/Mission owners. Do not split public Python classes or alter
native World orchestration, models, assets or history retention.

## Findings and selected approach

The C world adapter has 4,388 lines. A contiguous control/VI section plus the existing hold and batch-command
entry points form a coherent owner: 233 exports and 55 local helper definitions. The remaining world owner keeps
65 exports for lifecycle, state, effects, transport, recording and related world services.

The Python native core has 5,798 lines. Control comprises 112 World handlers, 13 module handlers (including five
macro-generated names) and 46 local helpers. Mission comprises 37 World handlers, 12 module handlers and four
local helpers. Shared conversions and original type/module registration must remain centrally owned.

Three approaches were considered: literal domain extraction with minimal private support, public object-model
composition/mixins, and a general dispatch framework. Select literal extraction because source-body and runtime
comparisons can establish the requested compatibility. Defer the other approaches: they introduce identity,
serialization or execution changes beyond this slice.

## C control owner

Create `src/sdk/fsim_control_c.cpp`. Move the existing section from `fsim_vehicle_command_actuator` through
`fsim_command_field_count_full`, plus `fsim_hold` and `fsim_world_command_batch`. Move control-exclusive `absent`
and `command` helpers and the five command-layout assertions. Keep original linkage contexts, anonymous namespaces,
macros and local conversions; some helpers currently have C linkage and must not acquire an anonymous namespace.

`params` is used by both control and the remaining effect/transport adapters. Move its literal definition into
private `src/sdk/CParameterSupport.h`, included only by the two users. Keep `param`, vehicle initialization,
state/recording layout checks and related helpers in the original world file. Existing CAbiSupport, Handles,
last-error ownership, Mission and DMS/C2 owners remain unchanged.

All 157 first principal guards and 32 `FSIM_AGENT_AS` occurrences in the moved exports retain their positions.
Do not unify error clearing, result handling or validation. Control submissions record the handle's last command
detail; Mission results do not. Some direct successes retain LastError, while guarded successes clear it. Mission
accepts a null result and can reject a short result only after submission; control's null result is rejected before
submission. Characterize these differences before production edits.

## Python control and Mission owners

Create `python/src/_native_control.c` and `_native_mission.c`, compiled into the same `_native.pyd`. Move the
identified control/VI and Mission blocks, their local converters and module handlers literally. Keep terrain,
property access, state, lifecycle, VecEnv, Scenario, Recording and layout introspection in the core.

Keep WorldObject, WORLD_IDLE and shared support in `_native_world.h`. Move the existing `FSIM_PY_VALUES` definition
there with its value 40 unchanged. Expose the existing `as_int` and `get_array` bodies through private declarations;
keep `as_u64` and `as_doubles` in the core even though their definitions sit within extracted ranges. Existing
shared helpers, Error, BufferType, World/module/type tables, `add_type` and PyInit retain their original owners.

Only moved handler linkage changes from static to translation-unit-visible; their signatures and bodies do not
change. The control name macro keeps its body and invocation arguments, with that same linkage adjustment.
Forward declarations are provided by the private header. Both new translation units define
`FSIM_PY_NO_BUFFER_TYPE`, avoiding extra buffer registration. No public Python file or class/mixin changes.

Preserve FASTCALL, no-keyword behavior, tuple format/order/placeholders, borrowed data, memory ownership,
exception identity, buffer acquisition/release and every current busy/GIL check at its original position.
Mission handlers that currently lack WORLD_IDLE retain that behavior; this extraction does not introduce new
threading policy. The native payload tuple's placeholder and the public PayloadInfo conversion remain unchanged.

## Acceptance and protected scope

- Add meaningful C/Python behavior characterizations and run them against committed A2 before moving bodies.
- Compare every moved definition, macro body and original-file remainder with A2; retain registration tables,
  shared helper bodies, layouts and guard locations. Verify negative extraction checks before implementation.
- Compare all 229 World descriptors, 32 module functions and native Python export names with a captured runtime
  baseline; retain the sole `PyInit__native` export and all 1,338 SDK DLL exports.
- Run architecture/principal gates, C ABI, control/Mission/agent acceptance, Python isolation/thread/buffer
  acceptance, the full applicable headless suite and independent installed-package consumers.
- Refresh distribution and compare runtime manifest hashes and DLL bytes with the tested consumer.
- Hash-protect all public headers, Python object-model files, aircraft/store/generated and third-party content,
  native simulation/domain sources, existing Mission/DMS owners, Handles and CAbiSupport. The captured baseline
  covers 3,514 files. No public declaration, principal inventory or generated boundary changes.
- Update current architecture, development and status guidance. Record source comparison, review, failures,
  commands, counts and untested configurations explicitly.

## Environment and stopping point

Use existing Windows x64 UCRT64 GCC/Ninja tools under `D:/ENV`, C++17, C11, CMake 3.25+ and the CPython 3.11 stable
ABI. Compilation TMP, TEMP and TORCH_EXTENSIONS_DIR use `D:/ENV/tmp`; no dependencies are installed.

Stop after independent review and local adapter/functional/package acceptance. Leave A3 reviewable and uncommitted
until requested. Renderer/GPU, render SDK installation, CPack ZIP, remote CI and unchanged Hangar generation are
outside this slice. World state-owner decomposition, public-history retention, store bindings, geometry,
weapon observations, viewer/platform boundaries and export policy retain their own queue and acceptance.
