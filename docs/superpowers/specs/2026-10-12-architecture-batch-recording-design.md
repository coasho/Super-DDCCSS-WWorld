# Architecture A5: VecEnv and Recording adapters

## Intent and scope

A4 is committed as e708b87. The user requests another C/Python source split for VecEnv and Recording, preserving
public API/ABI, principal boundaries and simulation behavior. A5 uses codex/architecture-vecenv-recording and stays
uncommitted after acceptance. Conversation is Chinese; written files are English.

This is a source-ownership change. Public headers, Python object models, native simulation/domain sources, resource
bytes, prior adapter owners, first principal guards, defaults and observable error/lifetime behavior stay unchanged.

## C ownership

- Add src/sdk/fsim_vecenv_c.cpp for fsim_options_init and all 13 fsim_vecenv_* definitions. Move the original
  setError, void-return guard and toScenario helpers literally. Do not substitute CAbiSupport's int-return guard:
  the two helpers preserve different existing behavior. lastError remains owned by fsim.cpp.
- Share the unchanged opaque fsim_vecenv layout through private VecEnvHandles.h, included by fsim.cpp and the new
  owner. It retains env, last and the lazily cached unique_ptr<fsim_world>. Include env/VecEnv.h, sdk/Handles.h and
  memory so destruction sees the complete borrowed World handle.
- Keep the C++ VecEnv Impl/methods and toOptions/span/toResult in fsim.cpp, with ABI/version/log/registry functions
  and fsim::version. The fsim_vecenv_world definition retains its original namespace fsim and explicit C linkage.
- Add src/sdk/fsim_recording_c.cpp for the opaque recording/frame layout and nine fsim_recording_* definitions.
  Copy the original state/control layout assertions into this owner while retaining them in World. Event strings
  still point into the recording; sample/event arrays remain owned by it. Use the unchanged CAbiSupport helpers.
- Keep fsim_world_recording and record_path handling in World core. The recording query's first principal guard
  and World construction/destruction remain unchanged. Add only these owners and the private header to SDK CMake.

## Python ownership

- Add _native_vecenv.c for 11 callbacks: constructor/deallocator, step/reset, names, action ranges, vehicle steps,
  autoreset get/set, vehicle IDs and capsule. Move the original vecenv_fields table and local names_list helper.
- Add _native_recording.c for five callbacks: constructor/deallocator, info, frame_time and events.
- Add focused private _native_vecenv.h and _native_recording.h for literal object layouts and matching callback
  prototypes. They use existing private call/conversion support; _native_world.h and fsim_py.h remain unchanged.
- Retain vecenv_buffers, vecenv_world and recording_samples in core because they consume its static BufferType or
  world_borrowed constructor. They continue retaining their owner and exporting the same zero-copy data.
- Retain Error, every runtime type pointer, all 14 registration initializers, four type specs, add_type and PyInit
  in core. New owners define FSIM_PY_NO_BUFFER_TYPE. No new accessor, global runtime state or public interface is added.
- Preserve all moved bodies and field metadata literally, changing only required static linkage. Keep busy flags,
  GIL phases, scratch allocation, buffer release, capsule ref address and deallocator order at their existing places.

## Alternatives

Moving the entire mixed C/C++ VecEnv implementation would put public C++ ownership into a C adapter file. A private
literal handle header permits a narrower separation. Exporting Python type pointers or adding Buffer/World accessor
wrappers would move runtime state or modify body comparisons. Retaining the three coupled callbacks avoids either
change while separating the remaining domain behavior. General support/header redesign is a later task.

## Acceptance

1. Add small public C/Python characterization cases for borrowed World caching/lifetime, stable buffers, error
   retention, option/conversion/count contracts, Recording output clearing, event/sample borrowing and native identity.
   Run them against committed A4 before extraction; retain evidence of any corrected fixture assumptions.
2. Capture protected hashes, SDK exports, native descriptors/functions/layouts and CTest inventory. Compare all moved
   bodies, core remainders, C++ wrappers, private layouts, shared helpers, registrations and retained callbacks to A4.
   Observe missing-owner verifier failures before applying the extraction.
3. Build headless with warnings as errors, run ABI/Python/principal/architecture gates and the full applicable suite,
   including determinism, lifecycle, threads/buffers, fleet and both installed/package gates.
4. Refresh dist; compare SDK exports, protected files, installed-consumer DLL bytes and runtime-data hashes. Run all
   binding compatibility suites against distributed Python and independently review the complete adapter/test diff.
5. Update architecture/development/status/navigation and this round's plan. Verify English, exact file endings, links,
   PowerShell blocks and diff scope. A5 stays uncommitted. No dependency installation or new feature milestone.

## Environment and stopping point

Use existing UCRT64 GCC/Ninja, C++17, C11, CMake 3.25+ and CPython 3.11 stable ABI. Tools/caches remain D:/ENV;
compilation TMP/TEMP/TORCH_EXTENSIONS_DIR remain D:/ENV/tmp. Root owns configure/build/CTest/dist and integration;
parallel workers own disjoint adapter or new-test files. The user's explicit continuation authorizes this established
workflow in the clean existing checkout without another worktree or intermediate approval gate.

Unchanged Hangar generation, renderer/GPU, render SDK installation, CPack ZIP and remote CI are outside this slice.
World state decomposition, aggregate history retention, finer control/store ownership, geometry, weapon observations
and DLL export policy remain separate work.
