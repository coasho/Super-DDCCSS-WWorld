# Architecture A2: MA retirement and DMS/C2 adapter ownership

## Intent and baseline

Architecture A1 is committed as `6652584`. A2 continues on `codex/architecture-lifecycle`, focusing on MA lifecycle maintenance and C/Python interface separation. Preserve public C++/C/Python API and ABI, principal permissions, error semantics, borrowed lifetimes and simulation ordering. Written files use English; conversation uses Chinese.

## Findings and selected approach

Vehicle and C2-node removal notify DMS but not MaCore. MA retains private consent bookkeeping alongside publicly queryable roles, authority, overrides and tasks. Answered and automatic consent records accumulate despite no longer authorizing a real sender. The retained State map is also scanned after removal.

Removing the entire State is unsafe: existing APIs read records after removal, C returns borrowed role names, tasks retain their history, and controller IDs depend on the retained mapping. Separate live and historical maps would also alter future unordered-map traversal and potentially message ordering. The selected slice explicitly retires private maintenance while preserving those records in place. A finite bound on aggregate public history requires a separate contract decision and is deferred.

The remaining C adapter has a contiguous 659-line DMS/C2 block with 70 exports. The Python native adapter has the matching 818-line block with 47 World handlers and two local conversion helpers. Extract each into a separately compiled domain file, retaining its original method/SDK registration and shared handle/error ownership. Splitting Python classes or introducing mixins would affect identity and isolation serialization; keep `world.py` and its public classes intact.

## MA retirement

Introduce an internal consent ledger owning the existing request records. Preserve request IDs, requested approvers and first-answer authorization. Reclaim answered records; do not retain automatic-policy records that have no approvers. Keep unanswered records even after a strike ends: an authorized late reply must still produce the original activity-ended rejection rather than unauthorized. Preserve the DMS's separate public consent history.

Add `MaCore::retireEntity(std::uint32_t) noexcept`, called after successful removal from DMS in both World removal paths. It must find an existing State without insertion, release its private consent storage, and mark retirement. Keep RBAC strings, tasks, authority, overrides, counters and controller mappings at their current owners. Reset does not retire or clear MA state.

After retirement, retain the existing MA step transitions: direct C2 commands clear on the next step, overrides expire at the same `expiryS <= simTime` comparison, and lost communication changes at its existing threshold. Skip maintenance only once there are no remaining override/C2 transitions and communication is disabled or already lost. Keep NaN/infinite values on their existing path rather than inventing expiry rules. Retain map keys and iteration order. This still scans the retained map; it is not an O(live entities) iteration redesign.

## C DMS/C2 adapter

Create `src/sdk/fsim_links_c.cpp` from the contiguous AB-4/AB-5 section, including `netSpecOf`, `netSpecToC` and `taskToC`. Leave transport-fabric `fsim_comm_*` and its parameter helpers in `fsim_world_c.cpp`. Compile into the same SDK DLL with the existing private `CAbiSupport.h` and `Handles.h`.

Move signatures and bodies literally, retain the `extern "C"` context and every first principal guard. Preserve strict `struct_size` checks, return/error text, success error-clearing behavior and borrowed data. Do not add guard wrappers or new exported functions. An unused-helper annotation may be needed for the existing shared guard declaration; its body must remain unchanged.

## Python native DMS/C2 adapter

Create `python/src/_native_dms.c` and private `_native_world.h`. Share the exact WorldObject layout and WORLD_IDLE macro, existing argument conversions and a private bridge to the original error helper. Keep the Error object and World/module/type registration in `_native.c`. Preserve function bodies, tuple format strings, argument flags, buffer acquisition/release order and each existing busy check. No GIL-release phase is added.

The shared header must not instantiate a second buffer type registration. Isolate only the unused buffer slot/spec declarations for the DMS translation unit; buffer and parameter helper bodies remain unchanged. Keep CPython 3.11 stable ABI and C11. Keep `world.py`, public import paths, class/function identities, qualified names and pickle/isolation behavior unchanged.

## Acceptance

- Characterize removal/reset/history, delayed C2 completion, override/lost-comms timing, slot reuse, duplicate/multiple-approver answers and late unanswered consent replies before production edits.
- Demonstrate resolved consent storage does not accumulate and retirement releases its capacity; retain unanswered requests and all public history.
- Compare moved C/Python function bodies and Python registration tables with `6652584`; compare both SDK DLL and native Python export names.
- Run architecture checks, C ABI, agent/C2 acceptance, Python SDK/isolation/thread/buffer acceptance and native deterministic engagement tests.
- Build UCRT64 headless with warnings as errors; run the full applicable functional suite and installed-package gates, then refresh distribution and verify runtime hashes/DLL bytes.
- Public `include/`, public Python object-model files, generated models and third-party content remain unchanged. Record skipped Hangar generation, renderer/GPU, render packaging and remote CI separately.

## Environment and stopping point

Use existing tools under `D:/ENV`, C++17, C11, CMake 3.25+ and CPython 3.11+. Set TMP, TEMP and TORCH_EXTENSIONS_DIR to `D:/ENV/tmp` for compilation. No dependencies are installed.

A2 stops after the three independently reviewed tasks and integration acceptance. Whole-State history reclamation, broader VI/Python object-model decomposition, store bindings, geometry ownership and viewer/platform work retain the A1 design's queue. A2 does not advance a flight or weapon-model milestone.
