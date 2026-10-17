# Architecture A1 Implementation Plan

**Goal:** Enforce existing module boundaries and assign Mission Systems adapters and weapon launch rules to focused internal owners.

**Architecture:** Retain the existing native module layering, SDK DLL and session orchestration; declare the missing world IPC dependency. Add dependency acceptance, extract the existing MS C adapter and move stateless weapon eligibility out of World.

**Tech Stack:** C++17, C ABI, Python standard-library checks, CMake 3.25+, Windows UCRT64 GCC.

**Spec:** [Architecture maintenance design](2026-10-11-architecture-maintenance-design.md).

## Global constraints

- Preserve public API/ABI, principal guards, error strings, numerical expressions, lifetime and step/event ordering.
- Preserve aircraft/store/generated/third-party bytes; add no framework or feature expansion.
- Tools/caches and compilation scratch stay under `D:/ENV` and `D:/ENV/tmp`.
- Write English files.
- Start from committed `f8cfec9`; leave the new pass reviewable without an additional commit request.

## Tasks

### 1. Module boundaries

- [x] Add explicit target/source ownership and dependency acceptance with negative fixtures.
- [x] Demonstrate current undeclared `ms -> sim` and `world -> ipc` imports are detected.
- [x] Move unchanged quaternion implementations to core, preserve sim forwarding and add world IPC dependency.
- [x] Verify repository and fixture checks; register headless CTest acceptance without tying it to SDK binding availability.

### 2. Mission Systems adapter

- [x] Characterize guard coverage and missing-source failures before changing the scanner.
- [x] Move the existing MS block to `fsim_mission_c.cpp`; share existing helpers in private `CAbiSupport.h`.
- [x] Include every C adapter in principal guard checks; verify missing definitions cannot silently pass.
- [x] Compare moved bodies/export names and run C ABI/principal/agent acceptance.

### 3. Launch eligibility ownership

- [x] Add direct World characterization for current FOV/target/illumination refusals and unchanged rejection side effects.
- [x] Extract a pure `ms/weapon` eligibility function, preserving the existing expression and refusal ordering.
- [x] Test defaults, override, geometry and illumination mismatches; run native engagement/determinism and store acceptance.

### 4. Integration and delivery

- [x] Build the headless preset and run focused checks, then the complete applicable functional suite and installed consumer.
- [x] Compare public SDK export names and source/data scope to the pre-refactor baseline.
- [x] Perform independent diff review and resolve material findings.
- [x] Update architecture/development/current status with evidence and remaining debt.

## Review focus

Check classification of shared-directory targets and unconfigured renderer sources, transitive dependency closure, C ABI guard coverage after file splitting, shared thread-local error state, borrowed output lifetime, FOV/refusal short-circuit precedence and exact native event/worker determinism.

## Progress

- Second pass committed as `f8cfec9` with a clean tree.
- Read-only native, SDK and dependency audits selected three independent A1 tasks. Stateful lifecycle, Python public-class identity and GPU orchestration changes are deferred explicitly.
- A1 implementation and independent review are complete. The headless functional run passed 538 cases in 325.27 seconds; three architecture cases passed separately, including 33 module fixtures and 22 C guard test methods. All 1,338 DLL export names match the captured baseline.
- Architecture checks also passed with Python SDK bindings disabled. Public headers and model/source resources remain unchanged. Third-pass changes remain uncommitted for review.
- See [current status](../../../status.md#architecture-a1-verification-2026-10-11) for commands, raw logs, the initial shell-command failure and untested configurations.
