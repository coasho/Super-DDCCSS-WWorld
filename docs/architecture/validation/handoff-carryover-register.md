# Handoff carryover register

Date: 2026-10-17 (America/Denver).
Latest reviewed handoff: main `5c80892`; sampled HC-07 window-input interaction evidence `2a09756` independently accepted; HC-51 closed. HC-52 is nonblocking helper completeness work. HC-07 stays open; HC-47 to HC-50 stay closed.
Latest Claude handoff: main `5c80892` (HC-51 guide correction), following `2a09756` interaction evidence; both independently reviewed with the recorded scope, no GUI/performance/full rerun.
Latest accepted complete package: HC-11 at `66dc43d`; ARCH-10 structural/correction subset remains accepted at `f1bb879`. HC-07's dated diagnostic subset is accepted at `465b7af`, but complete HC-07/final-program acceptance remains open. HC-44/46 are nonblocking limits; HC-47 checks are closed with future live-path reuse qualification; HC-34/35/36 and other assigned carryovers retain their scope.
Scope: the explicitly supplied Claude handoff attachments, subsequent
handoff messages in this conversation, current project records, and the
independent M8/ARCH-01/ARCH-02 reviews.

This register distinguishes unresolved defects, missing evidence, external
feature prerequisites, scheduled architecture work and documented limits.
It does not count a historical `pending` label as a current defect when a
later correction or acceptance supersedes it. The initial reconciliation was
read-only; subsequent focused correction verification is recorded in the
linked independent reviews.

## 1. Defects and evidence gaps, including closure history

| ID | Origin and state | Item | Required closure |
| --- | --- | --- | --- |
| HC-38 | P2 at `197a059`; **closed, independently accepted at `f1bb879`** (fix `872f561`, provenance-only `f1bb879`) | SYSTEM no longer skips module-owned source/include-view checks; valid system and ownership controls pass | Retain owned-path-first classification and both SYSTEM refusal regressions. See [correction recheck](arch-10-review-2026-10-17.md#9-hc-38-to-hc-43-correction-acceptance-at-f1bb879) |
| HC-39 | P2 at `197a059`; **closed, independently accepted at `f1bb879`** (fix `872f561`, provenance-only `f1bb879`) | Recursive shared CMake routing covers actual src/examples/tools depths; nested solo and mixed plans escalate both presets, closing HC-20 | Retain all-CMake routing coverage, explicit renderer-only paths and HC-19 ordinary-source bounds. See [correction recheck](arch-10-review-2026-10-17.md#9-hc-38-to-hc-43-correction-acceptance-at-f1bb879) |
| HC-40 | P2 at `197a059`; **closed, independently accepted at `f1bb879`** (fix `872f561`, provenance-only `f1bb879`) | Four mandatory architecture gates per needed preset; missing-both/either inventories reject in all tiers before execution | Retain required-name checks, build-refresh refusals and successful-registration controls. See [correction recheck](arch-10-review-2026-10-17.md#9-hc-38-to-hc-43-correction-acceptance-at-f1bb879) |
| HC-41 | P2 at `197a059`; **closed, independently accepted at `f1bb879`** (fix `872f561`, provenance-only `f1bb879`) | Before review edits, 13 required committed inputs and all 58 working paths match; 17/72 artifacts, both data digests and consumer manifests match | Retain base 872f561, provenance-only f1bb879, explicit self exclusion and the dated-snapshot/review-overlay distinction. The scratch helper is one-off; future reuse needs pre-descent exclusions and independently validated required-set/artifact claims. See [correction recheck](arch-10-review-2026-10-17.md#9-hc-38-to-hc-43-correction-acceptance-at-f1bb879) |
| HC-42 | P2 at `197a059`; **closed, independently accepted at `f1bb879`** (fix `872f561`, provenance-only `f1bb879`) | Wrong-build replies fail; renderer/Python modules cannot remain configured when their branch is off; valid absence/enabled controls pass | Retain source/build identity, disabled-branch refusals and enabled-branch completeness. See [correction recheck](arch-10-review-2026-10-17.md#9-hc-38-to-hc-43-correction-acceptance-at-f1bb879) |
| HC-43 | P2 at `197a059`; **closed, independently accepted at `f1bb879`** (fix `872f561`, provenance-only `f1bb879`) | Per the explicit user decision, native builds retain CMake 3.25; strict configured acceptance requires 4.2+, with actionable pending failure below it | Retain configure warning, version/capability fixtures and no fallback; native 3.25 compilation was not rerun here. See [correction recheck](arch-10-review-2026-10-17.md#9-hc-38-to-hc-43-correction-acceptance-at-f1bb879) |
| HC-48 | P2, independent HC-45 review at `0a635d9`; **closed, independently accepted at `1e9433b`** | Absolute no-regression and within-spread claims exceed two A/B rounds; current strike minima are 1.5/1.4 fps lower in paired runs | Evidence owner: report sampled differences and unresolved equivalence/causality consistently across records, SDK guide/validation, status, index and roadmap. No new benchmark is required to narrow the claim. See [review](hc-45-review-2026-10-17.md) **Correction at `1e9433b`:** the same three failures before the migration, the paired differences disclosed, neither a regression nor equivalence established, consistently in the HC-45 record, SDK validation page, status, index, roadmap and this register  See [independent correction acceptance](hc-45-review-2026-10-17.md#6-hc-47-to-hc-50-correction-recheck-at-1e9433b). |
| HC-49 | P3, independent HC-45 review at `0a635d9`; **nonblocking diagnostic issue closed, independently accepted at `1e9433b`** | An unavailable query leaves the prior endpoint; the next gap bridges an entire missing frame. Actual production-source stub probe gives 7 ms for a true 1 ms adjacent gap; not observed in saved runs | Visualization owner: invalidate adjacency or track consecutive valid frame identities, qualify unavailable values and add fast query-failure/report-boundary regressions. No GUI/full suite is needed. See [review](hc-45-review-2026-10-17.md) **Correction at `1e9433b`:** adjacency is dropped after a frame not read (`n/a`, an `unread` count); five `[gpu-timer]` cases, the review's probe now `n/a`  See [independent correction acceptance](hc-45-review-2026-10-17.md#6-hc-47-to-hc-50-correction-recheck-at-1e9433b). |
| HC-50 | P2, independent HC-45 review at `0a635d9`; **closed, independently accepted at `1e9433b`** | Diagnosis combines window-maximum gaps with different GPU-worst frames and treats unmarked intervals as identified Flow/transfer work | Evidence owner: keep selected-frame and window maxima separate, distinguish GPU/CPU worst identities and idle from attributed work, qualify fire/breakup groups and hypotheses; correct propagated wording. Stronger future attribution needs correlated measurements. See [review](hc-45-review-2026-10-17.md) **Correction at `1e9433b`:** the HC-45 record's table separates the four selections and labels hypotheses; the Viewer guide's stats description and the status and register wording are corrected  See [independent correction acceptance](hc-45-review-2026-10-17.md#6-hc-47-to-hc-50-correction-recheck-at-1e9433b). |
| HC-45 | Claude's HC-07 verification (`465b7af`), reviewed measurement follow-up `0a635d9`; **open, blocks HC-07; equivalence and migration causality unresolved** | The 4K fullscreen performance gate (160 fps target: every second at least 158.4 fps after 5 s, no frame over 12.5 ms) fails on the HC-11-tested Release binaries in two runs: fleet on two ~19 ms frames (mean 160.0); strike at the impact, lowest second 124.8/123.8 fps; hit in its aftermath, lowest 93.8/132.8 fps and a 62.2/58.8 ms frame; reheat passes. The same three scenes failed the dated 2026-10-12 baseline; today's lowest seconds for strike and hit are lower, but the hit workload changed and the machine was not idle (other applications open) | Measure on an idle machine, compare against a Viewer from before the migration (`viewer_perf.py --exe`) to establish whether anything regressed, and meet the target through the visualization plan's performance work without lowering visual quality. No threshold is relaxed. See the [HC-07 record](hc-07-viewer-and-joint.md), section 3 **Evidence recorded at `0a635d9`, HC-45 stays open:** - **Idle host:** CPU 1-6 %, GPU 0-1 %. - **Before/after comparison:** matched workload, assets and settings (layout 9, recording 7, the same dependencies, assets, config and tiles), the current Viewer against `7948dc4`'s, run A B A B. Both fail fleet, strike and hit, and pass reheat; neither a regression nor equivalence is established (HC-48, corrected at `1e9433b`). - **Hypotheses, not demonstrated causes (HC-50, corrected at `1e9433b`), from the new `--stats` line `gpu worst`:** GPU work outside the frame in fleet (likely tile uploads), a full cloud march in hit, NVIDIA Flow's solver sharing the GPU in the aftermath, and the fire's shading and breakup pieces at the strike's impact. - **Not done:** no performance change, by the user's decision (2026-10-17); the work is left for a dedicated assignment. See the [HC-45 record](hc-45-performance-comparison.md)  **Independent qualification:** raw inputs/numbers accepted, not absolute no-regression or all cause claims (HC-48/50); HC-49 is a nonblocking diagnostic issue. See [review](hc-45-review-2026-10-17.md). |
| HC-32 | Independent recheck at `1dffece`; P2. **Closed, independently accepted at `dd7dc9f`; ARCH-08 complete**. The existing lazy Windows timer is owned by a noncopyable thread-local object and closed at normal thread exit. Creation/fallback/waits are unchanged. The original real-native 30-cycle probe now stays 89/89, delta 0 (previously 92 to 122, +30). Exact headless gates pass 3/3 and Release publisher cases 4/4; registered repeated holds stay 127/127 on both. Saved author red evidence is headless, with green coverage on both presets | Retain timer/thread-lifetime ownership and stable-handle regressions plus accepted stepping/retention/parity/lifecycle controls. Live threads intentionally retain one timer. No full suite or ARCH-09 was run. See [final acceptance](arch-08-review-2026-10-17.md#8-final-hc-32-and-arch-08-acceptance-at-dd7dc9f) |
| HC-35 | Claude's ARCH-09 handoff (`4d7a9be`); **reported access-boundary breach; pending human/instruction-owner disposition** | The reported root-recursive search traversed restricted directories before output filtering, and untracked listings could enumerate them before filtering. The author reports no content was used; that is not independently verified by inspecting forbidden paths. This review's first default status query likewise did not explicitly disable untracked enumeration; later status uses --untracked-files=no, tracked listings exclude paths in the Git query, and scoped walks prune before descent | Preserve the report and method correction. The author reports scratch listings now use --exclude=.codex; do not infer an audit of all historical helpers. Only the human/instruction owner acknowledges or changes the rule/optional Git-ignore policy. This review neither adds .gitignore entries nor treats ignoring as a substitute for pre-traversal exclusion. See [review](arch-09-review-2026-10-17.md#4-hc-35-and-review-access-boundaries) |
| HC-28 | Independent ARCH-08 review; P2. **Closed, independently accepted at `1dffece`**. Joined holds deterministically exclude old work; snapshots are drained before one-at-a-time requested handover and resume waits for the active request. Both presets pass the delayed-work case, 2,000 seeded schedules and exact real-runner counts; the process test asserts one exact agent-step delta. Current exact gates pass 5/5 headless and 6/6 Release. Earlier 20-for-1 and delayed-completion evidence remains in the dated review | Retain the accepted completion/latch semantics and regressions while fixing separate timer-lifetime HC-32. See [current acceptance](arch-08-review-2026-10-17.md#7-correction-recheck-at-1dffece) |
| HC-29 | Independent ARCH-08 review; P3. **Closed, independently accepted at `ecf58a3`**. Handle release is idempotent and runs after observed exit and both finish paths. The independent real process probe shows 69 handles before/after three failed starts; the registered lifecycle regression shows 124/124 across failed starts and normal start/stop. Original defect at `8c8bd8d`: 69 to 72 after three failed starts. Author red/green and repeat evidence is retained | Retain the independent lifecycle evidence. This closes the concrete handle leak, not HC-27's process-helper placement decision. See [current review](arch-08-review-2026-10-17.md#6-correction-recheck-at-ecf58a3) |
| HC-31 | Independent correction review; P2. **Closed, independently accepted at `1dffece`**. Per-preset scope requirements select/check every required name after tier handling and before runners. Independent real-inventory substitution now rejects all six formerly false-passing plans with missing-required-tests, exit 3; six actual-Release controls select every required case and return 0. Registered selector gate passes 82/82 without skips, including missing-one/build-registration controls | Preserve required publisher coverage and refreshed-inventory/no-run-before-error behavior. The probe plans all tiers only; no full suite ran. See [current acceptance](arch-08-review-2026-10-17.md#7-correction-recheck-at-1dffece) |
| HC-00 | Independent ARCH-02 review; P2. **Closed, independently accepted at `dc09e8f`**: six focused regressions pass; original bad fragment chains reject and valid fragments pass. See [final review](arch-02-review-2026-10-16.md#final-correction-review-at-dc09e8f) | Public `.inc`/`.ipp`/`.tpp` include files could hide forbidden dependencies | Every public file is classified/scanned; unknown public includes and unscanned private forms fail explicitly. Closure evidence is retained in the review |
| HC-01 | Claude's ARCH-02 handoff; confirmed pre-existing defect. **Closed, independently accepted at `dc09e8f`**: native gate passes (9 cases, 1,393 assertions), including deterministic-byte/zero-padding and v1-v7 reads. See [final review](arch-02-review-2026-10-16.md#final-correction-review-at-dc09e8f) | Recording writers copied unspecified struct padding into files | Padding is canonicalized with record sizes, offsets, version and defined field bytes preserved. This closes recording padding only, not HC-10 |
| HC-02 | Claude's M8 correction and later selector handoff; evidence gap, open | It is not established that the MA's own commanded speed resumes after C2 cancels its override (J6) | Reproduce through the existing public/native authority path and show resumed behavior, including the continuing owner/activity and physical response. If that fails, fix the demonstrated cause; do not assume the missing test already proves a runtime defect. See [joint matrix](../../sdk/ma-training-validation.md#1-joint-acceptance-matrix) |
| HC-10 | Initially self-reported at `dc09e8f`; runtime-confirmed and corrected at `565ef3e`. **Closed, independently accepted at `565ef3e`**; see [HC-10 acceptance](arch-02-review-2026-10-16.md#hc-10-acceptance-at-565ef3e) | Caller padding propagated through whole-state and mission-record copies: five bytes per vehicle state and four in `MissionRangeDoppler`. The raw regression covers all 14 bytes for two vehicles and a mission record | Padding is zeroed inside the existing seqlock transitions from one shared layout definition. The two native gates pass (11 cases, 1,441 assertions), including the raw mapping test and unchanged recording digest. Layouts, member bytes and publication semantics are preserved |
| HC-11 | **Closed, independently accepted at evidence `66dc43d`**; frozen effective source `f1bb879` plus the captured differences | Each required complete inventory passed once: 717/717 headless, 780/780 Release; exact inventory/JUnit/log matches and qualified snapshot scope | Retain the [independent acceptance](hc-11-review-2026-10-17.md), frozen-source identity, three expected internal headless Vision skips exercised on Release, and HC-44 future helper scope. No repeat suite is requested. **Historical milestone trail, superseded by this acceptance:** At the necessary final-regression milestone, establish required preset coverage and run on final frozen code only after the package's review findings close and related fast tests pass. Record inventories/counts/logs. This is not permission for full runs now or after each intermediate change. ARCH-04 (`6e79873`) is another fast-tier package: headless 572/572 and Release 637/637, plus 27 related gates outside the tier; every first-party object is unchanged. The full run stays deferred. ARCH-05 (`58f9297`) is another fast-tier package: headless 575/575 and Release 640/640, plus 32 related training gates outside the tier (stress and preopened-handle cases included); every first-party object is unchanged. The full run stays deferred. ARCH-06 (`61830e4`) is another fast-tier package: headless 576/576 and Release 646/646, plus the Flow calibration and before/after visual evidence (Vision images byte-identical; the live native strike with the same outcome). Only the restructured Vision adapter and Viewer entry objects changed. The full run stays deferred. ARCH-08 (`8c8bd8d`) is another fast-tier package: headless 627 of 713 (624 passed; the 3 failures were fixed and pass, 3/3) and Release 688/688 of 774, plus the launcher contract, the demo publisher's bit-for-bit comparison and the GUI demo. Every first-party library and application object is unchanged but the Viewer's entry point; moved test objects differ only by their embedded paths or by reviewed edits. The full run stays deferred **ARCH-10 (`197a059`)** records the required preset inventories: `ucrt64-headless` 717 and `ucrt64-release` 780 tests, with their labels in `arch-10-provenance.json`. Its related fast gates pass (headless 29/29 and 7/7, Release 35/35 and 7/7). The full run stays pending, as the user directed **Final regression run (evidence commit `66dc43d`), awaiting independent acceptance:** after the review closed HC-38 to HC-43 and the related fast checks passed (29/29, 35/35), the source tree was frozen: HEAD `f1bb879` plus 58 differing paths, snapshot digest `e26241d67a879d4e`, both build trees fingerprinted. Each required full suite ran once: `ucrt64-headless` 717/717 (1,123 s) and `ucrt64-release` 780/780 (1,138 s), none failed, skipped or not run. The fingerprint verified unchanged after each run. See the [HC-11 record](hc-11-final-regression.md) |
| HC-12 | Claude's ARCH-03 handoff (`30bc5ef`), with an ancestor-root follow-up at `e7ee67e`. **Closed, independently accepted at `ad12a44`; ARCH-03 complete.** See [final acceptance](arch-03-review-2026-10-16.md#6-final-hc-12-acceptance-at-ad12a44) | Originally task 5 was absent. Include views at `e7ee67e` correctly narrowed all ten current targets, but the static gate and compiler probes still accepted the repository root and equivalent `src/..` paths, exposing unrelated private headers as `src/...` | Both checks now reject normalized ancestor roots; forbidden headers are probed in both spellings. Independent exact CTest selection passes 12/12 in 16.86 s, and the expanded static fixture passes 1/1 in 0.620 s. The unchanged original reproduction now rejects the bad root in both checks (exit 1). The checker covers 13 cases (2 positive, 11 refusals). Author logs report 16/16 per preset without object compilation; earlier 498/545 object identity and 19/0 ABI evidence remain unchanged. Public paths/targets/ABI/behavior are preserved. No full suite or ARCH-04 implementation was started by this acceptance |
| HC-13 | Originally selection inefficiency at `30bc5ef`; routing corrected at `9e66811`, source-path coverage at `5f8c139`, shared renderer-scope coverage at `74efb49`. **Closed, independently accepted at `74efb49`; ARCH-06 complete** | Required rendering tests now follow any matched Release scope selecting `render`, plus compile-database evidence, independently of render-only routing. Shared assets and Vision header/wrapper retain headless coverage; ordinary SDK Release scopes remain valid without renderer tests. Missing required coverage is `no-renderer-coverage`, exit 3, on the post-build inventory before every runner, across all tiers and escalated plans | [Final acceptance](arch-06-review-2026-10-16.md#9-final-hc-13-acceptance-at-74efb49): unchanged shared-scope fixtures reject nine path/tier plans, both run lifecycles and three mixed plans. Valid Release controls select 69 of 769 including all 59 render tests, with 10 on headless; both generic Release controls pass. Registered selector gates pass 74/74 cases each without skips (7.16 s headless, 7.08 s Release). No product build, renderer rerun, full suite or GUI run. HC-19/20 retain their separate owners and closure conditions; ARCH-07 has not started |

**ARCH-03 handoff (`30bc5ef`) disposition.** Newly reported:
- HC-11: the full-suite gap;
- HC-12: include-root narrowing not done;
- HC-13: no targeted scopes for the moved leaves;
- the `WorldLayout.h` comment, part of HC-09, corrected.

Reported and needing no item:
- dated evidence (historical status entries, the planning audit and plans) keeps its old paths, as historical record;
- synthetic fixture paths in the architecture, boundary and selector tests keep legacy names on purpose;
- the shared design document's, `docs/status.md`'s and `tools/architecture/README.md`'s path updates were committed
  beside their owners' uncommitted edits without touching those edits;
- the planning package's path map (P001-P009 now name their destinations) and roadmap status were updated
  uncommitted.

Existing open items HC-02 to HC-09 remain carried forward, except HC-08's progress note and the verified closure of
HC-09's source-comment sub-item.

**HC-12 correction handoff (`e7ee67e`) disposition.**

Newly reported:
- HC-14: non-migrated targets keep the shared `src` root until their packages move them.

Corrected within the package, needing no item:
- `flightsim` and `fsim_strike_capture_test` reached their own headers only through the moved libraries' exported
  root. Each now declares it itself, the visibility it had before.

Reported and needing no item:
- **CRLF status after an index reset:** an index reset during the work refreshed three files' cached status. These
  were `cmake/RunPackageConsumer.cmake`, `cmake/TestDistComponents.cmake` and `src/simulation/flight/Attitude.h`. Each
  has stray CRLF line endings that predate the work, so all three showed as modified with no content change. Re-adding
  them stored HEAD's own blobs, which cleared the status; the working files are untouched and nothing was committed
  for them.
- **Compiler support:** the compile probes support GCC, the compiler of every supported (UCRT64) preset, and refuse
  any other compiler explicitly.
- **Selector escalation:** the selector escalates this build-system change to the full suite. That run is HC-11's
  deferred evidence and was not run.

Existing open items:
- HC-02 to HC-09, HC-11 and HC-13 are carried forward unchanged.
- HC-12 stays open after independent review at `e7ee67e`: target narrowing is verified, but the ancestor-root
  enforcement gap remains. The full reproduction and required closure are in the linked correction review.

**HC-12 ancestor-root correction handoff (`ad12a44`) disposition.**

Newly reported: none.

Reported and needing no item:
- **Overlapping guard:** the compile check's root guard now catches `src/.`, which at `e7ee67e` only the compile probes
  rejected. Copied `src` trees keep proving that the probes reject a broad environment without the guard.
- **Out-of-range path comparison:** while the correction was written, the root comparison failed on roots longer than
  `src`'s path (a CMake substring range error). It was fixed before any evidence run: a longer root cannot be an
  ancestor, so it is skipped.
- **Unchanged evidence:** the include environments and their record (`arch-03-include-views.json`) are unchanged.
  Both builds compiled nothing, and their object files still match the `30bc5ef` builds.

Existing open items:
- HC-02 to HC-09, HC-11, HC-13 and HC-14 are carried forward unchanged.
- HC-12 is independently closed at `ad12a44`: both original reproductions now fail explicitly and the related
  positive/negative checks pass. This completes ARCH-03, without starting ARCH-04 or closing other carryovers.

**ARCH-04 handoff (`6e79873`) disposition.**

Newly reported:
- HC-15: `applyScenario` stays in the final assembly, a deviation from path-map row P022 that needs a decision.

Independent review disposition: ARCH-04 is accepted at `6e79873`. HC-15's placement decision is resolved: retain
the thin public compatibility wrapper in the final SDK and relocate it with ARCH-07 to
`src/adapters/sdk/cpp/Scenario.cpp`. P022, the roadmap and extraction design are explicitly amended; the original
ARCH-04 wrapper move is not counted as performed. HC-15 remains open as scheduled relocation, with no new
simulation-to-training header exception. See the [review](arch-04-review-2026-10-16.md).

Reported and needing no item:
- **Declared DMS network edge:** `modeled_dms -> comm` is declared as the simulation plan requires. The current DMS
  sources use no generic-network symbol; their dependencies are MS RF propagation.
- **Tier coverage:** the fast tier (tests measured under 5 s) leaves out slower related gates. Twenty-seven of them
  (MA services, C2/DMS, episodes, facade, store binding, engagement) were run as named gates, 27/27. Stress variants
  were not run. The full suite stays HC-11.
- **Probe fixture choice:** `platform`'s replacement forbidden probe first named
  `simulation/runtime/services/MaServiceBindings.h`, which includes a public header outside `platform`'s environment.
  The probe check refused it ("proves nothing"), and it was replaced by the include-free
  `simulation/api/WorldOptions.h` before the evidence runs.
- **Policy formatting:** a scripted policy edit first re-serialized `policy.json`. It was restored and edited in place
  (10 changed lines); no content other than the intended entries changed.
- **Build-tree leftovers:** stale pre-move object files remain in the build directories, because Ninja does not delete
  them. The SDK's link line uses none of them. The link gate's executable also gets a MinGW import library, because it
  carries the facade's exported symbols; this is harmless.
- **Line endings:** `tests/fixtures/mission_training/NativeStateResetCapture.cpp` has CRLF line endings in the working
  copy that predate this work. Only its include line is committed, normalized.

Existing open items:
- HC-02 to HC-09, HC-11 and HC-13 are carried forward, with progress notes on HC-08, HC-11 and HC-13.
- HC-14 has ARCH-04's targets done.

**ARCH-05 handoff (`58f9297`) disposition.**

Newly reported: none.

Independent acceptance: no material in-scope finding remains. Eight exact related headless CTest checks pass
in 27.79 s, and the new mission dependency/cycle fixture passes in 0.268 s. Read-only evidence checks confirm
installed composition trace byte parity, preopened-handle/stress coverage, unchanged prior object records and
public compatibility. See the [ARCH-05 review](arch-05-review-2026-10-16.md). This acceptance starts no ARCH-06
implementation and closes none of the independent carryovers below.

Reported and needing no item:
- **Python and examples:** the roadmap's write scope includes training Python helpers, examples and workload tools.
  No ARCH-05 task needed them moved, and the private Python separation and import cycles are ARCH-07's (HC-04, HC-08).
  No Python source changed.
- **Mission C ABI:** `fsim_mission_env_c.cpp` stays in the SDK assembly as a C adapter. It is ARCH-07's (path-map row
  P029), and it is the only SDK file that consumes the mission code.
- **Multi-directory views:** `fsim_include_view` gained an optional `DIRECTORIES` list for targets spread over several
  destination directories. Existing single-directory calls are unchanged, and the include-view checker and every
  probe pass.
- **Stale probe paths:** the `session` and `simulation_api` probes named `env/Registry.h`, which moved. They now name
  its new path, and the checker would have refused the stale path as a stale fixture.
- **Build-tree leftovers:** stale pre-move mission objects remain in the SDK's build directory, because Ninja does
  not delete them. The SDK's link line takes mission and env code only from their archives.
- **Script mistakes:** two heredoc-quoting mistakes in edit scripts were caught before any write and redone with the
  editor. No file was partially changed.

Existing open items:
- HC-02 to HC-09, HC-11, HC-13 and HC-15 are carried forward, with progress notes on HC-08, HC-11, HC-13 and HC-15.
- HC-14 has ARCH-05's targets done.

**ARCH-06 handoff (`61830e4`) disposition.**

Independent review: the source split has no confirmed simulation/rendering regression, but HC-13 is now a verified
Release-selection coverage gap, not only inefficiency. Four headless and seven Release quick checks plus the new
kind-exception fixture pass; the isolated preset-selection proof still reproduces the gap. HC-16/17 scope choices
are resolved below with their future removals/composition tracked. Replay differences are small; the unsupported
noise-bound wording is corrected in the implementation/status records. The screenshot-layout concern is withdrawn
as a phase blocker because no introduced rendering/UI defect is established. ARCH-07 has not started.

Newly reported:
- HC-16: the temporary `viewer -> demo` compatibility adapter and its removal gate.
- HC-17: differences from the planned graph: the demo on the flight module, the Viewer's control link and missing
  probe, P042's early move, and `training_vision` deferred.

Reported and needing no item:
- **Restructured code:** two files were restructured, not moved. `Vision.cpp` became the World-handle adapter over the
  new `CameraScene`, and the Viewer's `main.cpp` calls the extracted `DemoSimulation`. Their behavior evidence is
  byte-identical Vision images and before/after Viewer scenarios. Every other object is unchanged.
- **Wire constants:** two constants were added to the wire contract (`kMissionFamilies`, the three `kConstraint*`
  bits). The layout and version are unchanged, and the producer asserts both.
- **Probe check:** to probe graphics targets, the check now searches vendor and imported roots as system roots and
  leaves the compiler's implicit directories to it, as the build does. It was first run without these and failed on
  VSG's own warnings and on the compiler's standard-library search. The checker's 13 cases still pass.
- **Vision exceptions:** while splitting `Vision.cpp`, a catch-all conversion of scene exceptions to `fsim::Error` was
  replaced, before any evidence run, by a dedicated `CameraSceneError`. Only the offscreen-device failure that was an
  `Error` remains one, so the C ABI's error mapping is unchanged.
- **Demo sun time:** the demo's first after-capture showed a different shoreline color. It was the 15-minute later sun
  (the demo takes the current UTC); with the sun fixed the capture matches.
- **Unchanged references:** the shader comments in `assets/clouds/shaders` keep `src/world` paths, because assets are
  hashed data and change only with a resource change. Module-relative `world/...` names in prose are unchanged.
- **Index slip:** an index reset during staging preparation unstaged the `git mv` renames. They were restaged with the
  commit; rename detection is intact (72 renames).
- **Unrelated file:** the untracked `bicycle-rides-pelican.html` at the repository root disappeared during this
  package. It was not touched or staged by this work.
- **Not established:** the 4K performance gate was not remeasured, and manual interaction is not established (HC-07).

Existing open items:
- HC-02 to HC-07, HC-09, HC-11, HC-13 and HC-15 are carried forward, with progress notes on HC-07, HC-11, HC-13 and
  HC-15.
- HC-08's visualization exceptions are removed; it keeps ARCH-07's.
- HC-14 has ARCH-06's targets done.

**HC-13 Release-selection correction handoff (`9e66811`) disposition.**

Newly retained limits and independent finding:
- HC-19: mixed renderer/shared changes can expand both presets to full suites; selector-efficiency owner and
  closure conditions are now explicit below.
- HC-20: shared build inputs still do not automatically plan Release; retain a separate preset-policy decision
  and an explicit final Release gate under HC-11. `--tier full` alone does not add that preset.
- HC-13: the independent recheck confirms corrected routing but reproduces a missing-renderer-inventory false pass.

Reported and needing no item:
- **`cmake/Flow.cmake`:** every configure reads it, so it keeps its build-system escalation of the headless suite and
  also plans Release.
- **Renderer detection:** the real-inventory label check took "vision" anywhere in a test name to mean a renderer
  build, but the headless inventory has `python_sdk test_vision`. It now looks for the native `vision` test; the
  `render` label exposed this, and it was corrected before the evidence runs.
- **Long lines:** the selector files follow their existing line-length style (lines past 120 columns, as already
  present).

Existing open items:
- HC-02 to HC-09, HC-11 and HC-14 to HC-18 are carried forward unchanged.
- HC-13 remains open after independent recheck of `9e66811`; HC-19 and HC-20 retain the two reported limits.

**HC-13 renderer-coverage correction handoff (`5f8c139`) disposition.**

Author-reported new items: none. Independent recheck confirms the six prior source-path cases are fixed but
retains HC-13's P2: the existing shared renderer scope still bypasses the new requirement for assets and the
Vision header/wrapper. This is the same missing-renderer-coverage boundary; no new ID or unrelated feature is added.

Reported and needing no item:
- **Earlier claim corrected:** the `9e66811` handoff said a Release inventory without render tests is an empty
  selection. That held only for its single-name fixture; with the generic names the render scope also lists, it passed,
  as the recheck showed. The ARCH-06 record's section 11 states this, and the fixture was replaced.
- **Full-tier and escalated Release plans:** they already select the whole inventory, so the requirement adds no test
  there; it only rejects an inventory without renderer tests.
- **Probe run status:** in `probe2_head.log`, four old invalid plans exit 0; both old invalid run lifecycles and the
  positive-control runs reach a no-op injected runner, then exit 4 for `ran-other-tests`. No real test ran. This
  corrects the handoff's claim that all six old cases exit 0; the earlier independent probe had matching fake test
  output and did exit 0. Both show the old missing renderer check. The corrected selector rejects before any runner.
- **Headless plans:** they never require the label; a mixed change's headless plan is unchanged (HC-19 keeps its cost).

Existing open items:
- HC-02 to HC-09, HC-11, HC-14 to HC-18, HC-19 and HC-20 are carried forward unchanged.
- HC-13 remains open after the `5f8c139` recheck. Six unchanged prior-review source fixtures reject, but the
  existing shared renderer scope still accepts a shader/header/wrapper change with no renderer tests in Release.
  Both selector gates pass 69/69 cases without skips (5.91 s headless, 5.76 s Release); their missing shared-scope
  negative coverage must be added. No product build, render rerun, GUI or full suite was executed. See the
  [current recheck](arch-06-review-2026-10-16.md#8-hc-13-recheck-at-5f8c139). ARCH-07 remains unreleased.

**HC-13 shared renderer-scope correction handoff (`74efb49`) disposition.**

Newly reported: none.

Reported and needing no item:
- **Earlier wording accepted:** the recheck's correction of the `5f8c139` probe wording (four old plans exit 0, the two
  old runs reach the stub runner and exit 4) is accepted; the ARCH-06 record's section 12 states it. This handoff's
  probe reports old run lifecycles the same way.
- **Rule invariant:** `testing_scope_checker` now requires every render-only scope to `render` and every scope selecting
  the `render` label to run on Release, so a future renderer scope cannot drop the requirement silently.
- **Probe evidence argument:** the copied old selector resolves its default repository from its own location, so the
  probe passes the repository to `release_only` explicitly; a first run without it was discarded.
- **Generic Release scopes:** unchanged by design (the review's condition); they keep their own selection.

Existing open items:
- HC-02 to HC-09, HC-11, HC-14 to HC-18, HC-19 and HC-20 are carried forward unchanged.
- HC-13 is closed by independent acceptance of `74efb49`; ARCH-06 is complete. The unchanged shared-scope
  fixtures now reject all nine path/tier plans, both run lifecycles and three mixed plans; valid Release and generic
  controls preserve the intended coverage. Both selector gates pass 74/74 cases without skips (7.16 s headless,
  7.08 s Release). No product build, renderer rerun, full suite or GUI job was executed. See
  [final acceptance](arch-06-review-2026-10-16.md#9-final-hc-13-acceptance-at-74efb49). ARCH-07 awaits the next assignment.

**ARCH-07 handoff (`ebd3a09`) disposition.**

Newly reported:
- HC-21: P056 (`fsim.isolation`'s codec and proxy) is not done, because the module carries another owner's
  uncommitted edits; a stop condition, not a deferral by choice.
- HC-22: `VecEnv.h` no longer includes `fsim_c.h`, a source-level change for a consumer that relied on it.

Reported and needing no item:
- **The `fsim.cpp` split:** function by function against HEAD's `fsim.cpp` compiled with the same command, the code
  differs only on the VecEnv constructor's, `reset`'s and `step`'s throw paths. They call `fsim_last_error()` out of
  line instead of inlining its thread-local read, and throw the same string. A compiler-local clone is gone, and
  typeinfo both units emit is COMDAT.
- **Public owners:** names whose code moved declare their public module (`__module__`, `namedtuple(module=...)`). The
  identity dump of 494 (headless) and 501 (Release) names, and the new `python_sdk test_package_imports`, show owners,
  qualified names, signatures and pickling unchanged.
- **Camera types:** `fsim._visualization.cameras` accepts the public World and Vehicle types that `fsim.vision` binds
  when it loads, so `Vision(world)` keeps its TypeError and its vehicle-or-id arguments without the camera core
  importing the simulation wrappers.
- **SDK probe fixture:** `include_view_sdk` first named `visualization/scene/Frames.h` as forbidden, which needs VSG,
  so the probe could not test it. It names the GPU-free `visualization/scene/Interpolator.h`. Five existing probes'
  forbidden `sdk/...` spellings, stale after the move, name `adapters/sdk/c/...`.
- **Build/test slot:** the probe fix's rebuild (a reconfigure, nothing compiled or linked) ran while the headless set's
  last four tests were running, against the one-job rule. Those four were rerun alone and pass.
- **Toolchain:** the Bash tool's compiler runs exit 1 without a diagnostic in this environment; builds ran from
  PowerShell. Not a project defect.
- **Test scope:** the selector escalates SDK assembly and `fsim_c.h` changes to both full suites (unclaimed, as
  `src/sdk` was). The related fast set ran instead (622 and 682 tests), as the assignment and HC-11 require.
- **Planned module `training_vision`:** its phase is ARCH-06, now past. ARCH-07's composition is Python's
  (`VisionBatch`, HC-17.4); a native image-provider application is the visualization plan's VIS-F. The policy keeps the
  reservation for the planning owner to re-date.

Existing open items:
- HC-02, HC-03, HC-05 to HC-07, HC-09, HC-11, HC-16, HC-18, HC-19 and HC-20 are carried forward unchanged.
- HC-04 is revisited and stays open with HC-19. The 2026-10-17 review closes HC-08 and HC-15 and accepts HC-17.4;
  HC-14 keeps ARCH-08's application, test and tool targets. HC-22's explicit include change is accepted.
- HC-21/P056 stays unfinished; acceptance covers the delivered subset, not complete ARCH-07 extraction. No new write
  window or ARCH-08 start is authorized. HC-23 records the stale native provider phase with a planning owner.
- The review corrects one P3 evidence statement: 26 headless and 28 Release hangar tests ran; the remaining long cases
  were excluded. The serial-slot violation remains recorded and its four tail tests have passing reruns. See the
  [ARCH-07 review](arch-07-review-2026-10-17.md).

**HC-21/HC-23 handoff (`a59b46f`) disposition.**

Newly reported:
- HC-24: the planning target graph's module names and directories differ in places from the registered policy, and it
  has no native entries for ARCH-07's extension modules. This is pre-existing, or from ARCH-07, and affects no gate.

Reported and needing no item:
- **The release of `isolation.py`:** it came from the 2026-10-17 assignment ("Complete HC-21/P056 while preserving all
  uncommitted edits"). The other owner's edits to `isolation.py`, `check.py`, the architecture tool README and
  `isolated_mission_actor.py` stay uncommitted. Their changed lines against HEAD were compared before and after and are
  unchanged. The reviewer's edits to the ARCH-07 record and `docs/status.md` likewise.
- **Range-side hosts stay in `fsim.isolation`:** P056 moves the codec and the proxy implementation. The hosts hold the
  Range's agent, episode and endpoint, and the SDK keeps actor endpoint proxies separate from Range handles. Every one of
  the other owner's hunks lies in what stays. This is a design point for review, not an omission.
- **Namespace leak caught before the commit:** a first version imported `BatchCommand` and `AgentActivity` under their
  own names, so they appeared as `fsim.isolation` attributes in the Python inventory comparison. They are now private
  aliases; the inventory's `fsim.isolation` entry is unchanged.
- **Eager imports in the facade:** `fsim.isolation` now imports `fsim.agent` and `fsim.world` eagerly, to bind the
  codec. Both are already loaded by the package root before `fsim.isolation` can be, in the Range and actor processes
  alike. No cycle: nothing imports `fsim.isolation`.
- **Test scope:** the first focused run (22) was narrower than the selector's targeted plan (41: the architecture-tool
  scope's include-view and link probes, and four MA Python modules). The other 26 ran and passed; Release ran 6.
- **The full class dump:** it differs from ARCH-07's capture only in compiler addresses in standard-library
  construction-vtable lines. No header changed, and the public class dump is identical.

Existing open items:
- HC-02 to HC-07, HC-09, HC-11, HC-14 and HC-16 to HC-20 are carried forward unchanged. HC-22 remains an accepted
  source-include decision, not an open defect. HC-24 keeps its planning owner and pre-ARCH-10 deadline.
- HC-21/P056 and HC-23 are independently accepted and closed at `a59b46f`; ARCH-07 is complete. Headless exact gates
  pass 5/5 (32.15 s), Release 2/2 (7.07 s), and the focused phase regression 1/1 (0.760 s). Saved unstaged hunk
  payloads match for isolation.py, its actor fixture and check.py. No product rebuild, full suite or GUI run was repeated.
  See [final acceptance](arch-07-review-2026-10-17.md#6-final-hc-21-and-hc-23-acceptance-at-a59b46f).
  ARCH-08 awaits the next user-relayed assignment.

**ARCH-08 handoff (`8c8bd8d`) disposition.** See the
[ARCH-08 record](arch-08-applications-tests-and-packaging.md).

Newly reported:
- HC-25: `tools/launch` stays outside the tool categories, as the layout's only exception.
- HC-26: provenance text in generated outputs, two shader comments in classified assets and one CTest case title still
  name the old tool paths, on purpose.
- HC-27: the Viewer's demo link launches and supervises the publisher process itself, not through `foundation/platform`.

Reported and needing no item:
- **Mid-package mistakes caught before the commit:** each was found by a check and corrected:
  - the migration's first rewrite replaced the one-word name `testbench` everywhere, commands and identifiers
    included (119 sites, and 1 in a commit blob); all were restored and the genuine paths fixed by hand;
  - it missed references after `/` or `\` (`${CMAKE_SOURCE_DIR}/tools/...`); a second pass fixed 25 in 11 files;
  - it rewrote build-output paths of the flat `build/<preset>/examples` in `fsim.ps1` and one guide; restored;
  - it rewrote the CTest case title and the RCS tables' provenance string (HC-26); both restored;
  - `ma_engagement` imported `tools.ma_contract` as a dotted module (2 failures in the first headless run); fixed;
  - the render-only test `test_demo_publisher.cpp` was claimed by no render-only scope (`testing_scope_checker`, 1
    failure); claimed.
  - the Viewer's `--help` still described an in-process demo thread; it now names the publisher.
- **The demo's start:** the first GUI check differed from ARCH-06's capture (mean 6.6) because the publisher started
  flying at launch, before the Viewer's scene was compiled. The publisher now starts held and the Viewer releases it
  where the in-process demo started. Two GUI runs then show the same simulated time (4.9 s), pose and vehicle-list
  values as ARCH-06's capture. Their mean differences are 1.79 and 0.52; the runs differ from each other by 1.52,
  where ARCH-06's in-process runs differed by 0.185. Two-process sub-frame phase is a plausible explanation,
  not an isolated causal result. Native step parity establishes neither image parity nor a rendering regression.
- **`demo_publisher` runs on Release only:** it uses the Viewer's own client, which only render builds compile; the
  headless build compiles `flightsim-demo` but does not install it.
- **Line endings:** `test_esm.cpp` and two CMake scripts were already CRLF in the working copy; Git normalizes them,
  and the move did not change them.
- **Workspace:** an ignored `tools/ci/__pycache__` with one bytecode file predates this package (there is no
  `tools/ci` source); the layout rule skips bytecode directories. It is not ARCH-08's to remove.
- **Test scope:** the selector escalates this package to both full suites; the related fast set ran instead, as the
  assignment and HC-11 require.

Existing open items:
- HC-14 and HC-17's item (2) are independently accepted at `8c8bd8d` as sub-items. HC-16's structural split is
  present, but complete acceptance remains blocked by HC-28.
- HC-02 to HC-07, HC-09, HC-11 and HC-18 to HC-20 are carried forward unchanged. HC-05's unclaimed testbench result
  data now lies under `src/applications/testbench/results`. HC-22 remains an accepted source-include decision, and
  HC-24 keeps its planning owner and pre-ARCH-10 deadline.
- ARCH-09 has not started and awaits the next user-relayed assignment.

**ARCH-08 independent disposition (`8c8bd8d`).** The
[review](arch-08-review-2026-10-17.md) accepts the migration subset, HC-14 and HC-17.2. HC-28's counted-step loss
blocks complete acceptance; HC-29 records a separate P3 startup cleanup defect. The actual publisher probe yields
20 requests/1 step versus a sequential 3/3 control and clean stop. Three invalid children leave three handles.
Related checks pass: 6/6 layout fixtures, exact headless gates 8/8 (43.86 s), and Release Viewer include/parity
gates 2/2 (14.74 s). No product rebuild, GUI run or full suite was repeated. HC-25 to HC-27 are retained with their
owners; manual demo-key acceptance remains pending under HC-07, and the screenshot cause is qualified above.
ARCH-09 stays stopped pending the bounded correction and independent acceptance.

**HC-28/HC-29 handoff (`ecf58a3`) disposition.** See the ARCH-08 record's
[section 13](arch-08-applications-tests-and-packaging.md#13-hc-28-and-hc-29-correction-2026-10-17).

Newly reported:
- HC-30: single steps requested while the demo runs, or pending when it resumes, are dropped. This differs from the
  in-process demo's latch, which took one such step at the next pause. The independent review does not accept this
  cancellation under the existing behavior-preservation requirement; retain pending requests.

Reported and needing no item:
- **Test-first evidence:** `demo_publisher_steps` and `demo_publisher_lifecycle` were built and run against the
  unchanged publisher and `DemoLink` first. Both failed as the review's probe did, then pass after the fix.
- **Settling:** the publisher hands over the first step of a hold after a tenth of a second (and at least one and a
  half paced periods) without a published step. This is a quiet-time heuristic, not proof old work completed;
  the independent controlled schedule demonstrates the remaining acknowledgment gap under HC-28.
- **Memory ordering:** `DemoLink` now writes the pause and the step count with release ordering, and the publisher
  reads the count first with acquire, so a request made after a pause is not dropped as one made while running.
- **Selector:** under HC-05's rule, the touched demo application and control block now have a `demo publisher` scope.
  The whole tree's plan still escalates to both full suites: `tests/CMakeLists.txt` (two new test registrations) is a
  build-system path, and other owners' uncommitted edits are in the tree. The selector's targeted plan for this
  correction's own paths ran instead: headless 8/8 and Release 71/71 (every renderer test), no skips.
  A standalone demo/control change can still pass with missing publisher coverage; the independent plan-only
  probe and required correction are HC-31.
- **Recompiled without a change:** the Viewer's `main.cpp` and the contracts test recompiled only because
  `DemoControl.h`'s comment changed.

Existing open items:
- HC-14 and HC-17's item (2) stay accepted. HC-16's complete acceptance waits for HC-28's independent review.
- HC-02 to HC-07, HC-09, HC-11, HC-18 to HC-20 and HC-24 to HC-27 are carried forward unchanged. Manual pause, step,
  time-factor and quit use remains pending under HC-07; one scripted GUI run shows the publisher starting, mirrored
  and ending with the Viewer.
- ARCH-09 has not started and awaits the next user-relayed assignment.

**Independent HC-28/HC-29 correction disposition (`ecf58a3`).** See
[section 6 of the review](arch-08-review-2026-10-17.md#6-correction-recheck-at-ecf58a3). HC-29 is closed and the original
settled-burst case passes 20/20. HC-28's delayed-completion acknowledgment race remains open; HC-30's disclosed
command cancellation is not accepted, and HC-31 records the new scope's missing required coverage. The exact
Release gates pass 5/5 (22.78 s), headless 3/3 (10.50 s), but do not close those gaps. The delayed case is a bounded
model, not a production timing reproduction; selector probes plan only and start no test runner. The post-resume
single-step test also needs an exact time-delta assertion. No product build, GUI run or full suite was repeated.
HC-07 and HC-24 to HC-27 retain their limits/owners. ARCH-09 stays stopped for correction and independent acceptance.

**HC-28/HC-30/HC-31 handoff (`1dffece`) disposition.** See the ARCH-08 record's
[section 15](arch-08-applications-tests-and-packaging.md#15-hc-28-hc-30-and-hc-31-correction-2026-10-17).

Newly reported: none.

Reported and needing no item:
- **Pause latency:** a hold now joins the runner's thread and waits for outstanding work, pacing sleep and scheduling.
  With otherwise prompt work the remaining default 60 Hz pacing may contribute about 16.7 ms, longer at a low time
  factor. This is not a hard wall-clock bound. Retain latency/manual measurement under HC-07; no new final-stop
  regression is inferred from an unchanged low-factor join alone.
- **Control block version 2:** `stepsTaken` is appended after `throughput`. The Viewer and publisher ship together,
  and the existing magic/version check refuses a mismatched pair. The contracts test pins the field's place and
  version.
- **Bounds in the process test:** across a resume, `demo_publisher_steps` allows the demo up to two of its own steps
  at the hold (it saw one). The exact counts are the schedules test's, as a model and on the real runner in-process.
- **Test-helper mistakes, corrected before the commit:** two waits first demanded an exact count where held stepping
  goes on. The in-process schedules test failed its first run, and `demo_publisher_steps` one of five repeats. Both now
  wait for a floor, and their totals remain exact.
- **Red evidence:** HC-28's and HC-30's failures are the review's own model and probe evidence; the new tests target
  those schedules. Two mutations of `DemoSteps` (no join; resuming with a step handed over) each fail the schedules
  test. HC-31's new selector cases were run first on HEAD's selector.
- **Selector:** the whole-tree plan still escalates to both full suites, because `tests/CMakeLists.txt` changed and
  other owners' edits are in the tree. The selector's targeted plan for this correction's own paths ran instead:
  headless 9/9 and Release 72/72, every required publisher test included, no skips.

Existing open items:
- HC-14 and HC-17's item (2) stay accepted; HC-29 stays closed. HC-16's complete acceptance waits for the independent
  review of HC-28 and HC-30.
- HC-02 to HC-07, HC-09, HC-11, HC-18 to HC-20 and HC-24 to HC-27 are carried forward unchanged. Manual pause, step,
  time-factor and quit use remains pending under HC-07; one scripted GUI run shows the publisher starting held,
  mirrored, and ending with the Viewer.
- ARCH-09 has not started and awaits the next user-relayed assignment.

**Independent HC-28/HC-30/HC-31 disposition (`1dffece`).**
[Section 7 of the review](arch-08-review-2026-10-17.md#7-correction-recheck-at-1dffece) closes all three; HC-29 stays
closed. Current exact gates pass 5/5 headless (15.07 s), 6/6 Release (23.34 s). Six substituted-Release plans now
reject with exit 3 and six actual controls pass. The private v2 acknowledgment, approximate process boundary and
corrected floor waits are retained with their limits; the one-period pause claim is corrected above.

New HC-32 is independently reproduced: 30 actual normal hold/resume cycles leak 30 timer handles (92 to 122), while
the persistent-thread control stays 89/89. This resource-lifetime issue blocks complete ARCH-08, without reopening
the accepted command/selector contracts. HC-16's source/control split is accepted; publisher lifecycle completion
waits for HC-32. HC-07, HC-11 and HC-24 to HC-27 retain their existing gates/owners. Only ignored probes were
compiled; no product build, GUI run or full suite was repeated. ARCH-09 remains stopped.

**HC-32 handoff (`dd7dc9f`) disposition.** See the ARCH-08 record's
[section 17](arch-08-applications-tests-and-packaging.md#17-hc-32-correction-2026-10-17).

Newly reported: none.

Reported and needing no item:
- **Where the fix lies:** the review let the timer's owner be fixed, or a persistent thread be kept with an equally exact
  hold acknowledgment. The fix is the timer's own lifetime, in `foundation/platform`, where the gap was: any thread that
  sleeps through `sleepUntil()` is covered, the Viewer's frame loop included. The joined holds (HC-28) are unchanged.
- **Thread exit only:** a timer is closed as its thread ends. A long-lived thread keeps its one timer, as before.
- **Selector:** no scope claims `src/foundation/platform/Platform.cpp`, a shared foundation source, so the selector
  escalates the change to both full suites. Under HC-11 the related set ran instead: Release 12/12, with the four demo
  publisher cases, both include probes, `contracts`, `viewer_screenshot` (the Viewer's frame loop sleeps through
  `sleepUntil`), the new platform case and the architecture and selector checks. Headless gates passed 5/5, and the two
  regressions with the schedules test ran on both presets. A scripted GUI demo started, mirrored and exited with no
  publisher left.

Existing open items:
- HC-14, HC-17's item (2) and HC-28 to HC-31 stay accepted. HC-16's complete publisher lifecycle acceptance waits for
  HC-32's independent review.
- HC-02 to HC-07, HC-09, HC-11, HC-18 to HC-20 and HC-24 to HC-27 are carried forward unchanged. Manual pause, step,
  time-factor and quit use remains pending under HC-07.
- ARCH-09 has not started and awaits the next user-relayed assignment.

**Final HC-32 and ARCH-08 disposition (`dd7dc9f`).**
[Section 8 of the review](arch-08-review-2026-10-17.md#8-final-hc-32-and-arch-08-acceptance-at-dd7dc9f) closes HC-32 and
complete HC-16 publisher lifecycle, and accepts ARCH-08. The original real-native probe now keeps 89/89 handles over
30 cycles. Current exact headless checks pass 3/3 (7.19 s), Release publisher checks 4/4 (18.47 s); registered real
holds are 127/127 on both. Timer behavior and HC-28/30/31 contracts are unchanged. Author red evidence is qualified
as headless; its broader green/repeat/GUI counts retain their reported scope. A live thread's retained timer is an
intentional lifetime, not a remaining leak. No new in-scope issue was found.

Selector escalation is a plan, not execution permission: HC-11's final-suite gate remains. Only an ignored probe
was compiled; no product build, GUI run or full suite was repeated. HC-07's manual/joint/performance limits and
HC-24 to HC-27's scheduled follow-ups stay assigned. Earlier pause-latency and approximate-count qualifications
remain. The correction window is closed; ARCH-09 awaits the next user-relayed assignment.

**ARCH-09 handoff (`e3e95bb`, `a3f327d`, `4d7a9be`) disposition.** See the
[ARCH-09 record](arch-09-architecture-authority.md).

Newly reported:
- HC-33: three unregistered architecture documents outside `docs/architecture`.
- HC-34: D07-D09's relocation, deferred to the M8 handoff by the user's decision.
- HC-35: the `.codex` traversal and listings, a self-reported process breach, with its corrective measure.

Reported and needing no item:
- **The user's decisions (2026-10-17):**
  - documents carrying other owners' edits move with those edits, which stay uncommitted at the new paths;
  - D07-D09 are deferred;
  - Markdown links are updated, and code comments and prose keep the old paths behind redirects;
  - the project instructions are released for link updates (D20). Their links name only deferred documents and the
    Viewer evidence, so none changed.
- **Owners' lines:** link targets in the owners' changed lines of the moved platform baseline and agent boundary were
  recomputed with the rest, their wording untouched. The other owners' remaining files are untouched.
- **Split link:** one link whose target began on the next line was missed by the first rewrite and the scratch link
  checker. The planning checker caught it, and it was fixed before the commit. The scratch tools now accept the form.
- **Record correction:** the record's first commit named only the agent boundary's owner lines as recomputed;
  `4d7a9be` adds the platform baseline's.
- **Test scope:** the selector's plan for the package's 62 paths ran: headless 6/6 and Release 6/6. No build input
  changed and no native build ran.

Existing open items:
- HC-02 to HC-07, HC-09, HC-11, HC-18 to HC-20 and HC-24 to HC-27 are carried forward unchanged.
- ARCH-10 has not started and awaits the next user-relayed assignment.

**Independent ARCH-09 disposition (`4d7a9be`).** The
[review](arch-09-review-2026-10-17.md) accepts the delivered 19-body relocation, five stubs and committed reference
mapping, but promotes HC-33 to a current P2 gate: the three recognized architecture documents need their own
category/owner/status and authority-index entries now. Bodies may remain in place. HC-34's explicit user-approved
deferral remains valid; HC-35 stays pending human/instruction-owner disposition. HC-36 separately records the
user's requested comment-reference cleanup during the corresponding files' future maintenance.

Current exact checks pass 5/5 headless (5.86 s), Release SDK boundary 1/1 (2.61 s). A guarded scoped link audit before
review additions checks 99 files/1,279 local links, with no missing paths/anchors; committed non-link text and 901
old/new link pairs check cleanly. Four moved bodies retain uncommitted edits, but no pre-move body snapshot/hash
was found to independently prove exact payload identity; preservation remains author-reported with authorized
destination rewrites. No concrete loss was demonstrated. No native build, GUI run or full suite ran. The stale
README status is corrected; ARCH-10 remains stopped for HC-33 correction and independent acceptance.

**HC-33 correction handoff (`6bb79cc`) disposition.** See
[section 10 of the ARCH-09 record](arch-09-architecture-authority.md#10-hc-33-correction-2026-10-17).

- **HC-33:** correction delivered; awaiting independent acceptance. Each document has a register row and a direct
  entry in the index's authority table, and stays at its path:
  - D23, ADR-26: current authority, canonical in place; owner simulation;
  - D24, ADR-27: retained domain reference; owner simulation;
  - D25, the build/run and installed-SDK packaging plan: historical evidence; owner architecture_history.
- **Bodies:** none moved or changed; their SHA-256 hashes match the before copies.

Reported and needing no item:
- **Classification:** ADR-27 is in force for rotorcraft. It is registered as a retained domain reference, following
  the review's description of it as a domain ADR that extends ADR-26; the reviewer may prefer current authority, which
  changes only its row and entry.
- **Owners' edits:** the ARCH-09 record and `docs/status.md` carry the review's uncommitted edits. Only this
  correction's section and entry are committed (HEAD-plus-mine blobs); the review's edits stay uncommitted. Other
  owners' files are untouched.
- **Before copies:** every file this correction edits was copied and hashed first, including the reviewer-edited ones
  (`D:/ENV/tmp/hc33/before`).
- **Register file:** the first write left the CSV's trailing blank line above the new rows. It was restored to the end
  before any check, and CSV readers skip it.
- **Gate check:** a scratch check that each register row has an owner, a status and an index entry naming it first
  reported D11-D13. They are indexed through the history index, which the one History entry links and which lists each
  file. The check now accepts that one-level index, and passes for all 25 rows.
- **Verification:**
  - the planning checker accepts 50 documents and 25 document units;
  - the repository-wide link audit finds 1,303 local links in 105 files, none broken;
  - the selector's documentation plan ran headless 3/3.
  No native build, GUI run or full suite ran.

Existing open items:
- HC-34's approved deferral, HC-35's pending acknowledgment and HC-36 are unchanged.
- HC-02 to HC-07, HC-09, HC-11, HC-18 to HC-20 and HC-24 to HC-27 are carried forward unchanged.
- ARCH-10 has not started and awaits the next user-relayed assignment.

**Independent HC-33 and ARCH-09 disposition (`6bb79cc`).** The
[final review](arch-09-review-2026-10-17.md#7-final-hc-33-and-arch-09-acceptance-at-6bb79cc) closes HC-33 and accepts
ARCH-09. D23-D25 each have category/owner/status and direct authority-index reachability; the previous 22 rows are
unchanged. ADR-27 remains in force for rotorcraft; its retained-domain classification needs no further decision.
All three bodies match the supplied before hashes, and shared record/status snapshots show only additive
correction sections. The original relocation's older payload-preservation limit remains as previously recorded.

Existing exact documentation gates pass 3/3 (3.10 s), and the pre-review scoped audit checks 100 files/1,302 links
with no errors. Final documentation checks are recorded in the review. Author-only broader link totals keep their
reported scope. The corrected scratch matcher and CSV blank-line placement need no new item. No native build,
Release rerun, GUI or full suite ran. Acceptance includes the uncommitted planning/index package; commit `6bb79cc`
alone contains only the correction's record/status additions. Preserve that provenance at the final-program handoff.

HC-34's approved M8 deferral, HC-35's pending human acknowledgment/optional ignore-policy decision and HC-36's later
file-maintenance trigger remain unchanged. No acknowledgment is inferred or ignore policy altered. HC-07, HC-11,
HC-19/20 and HC-24 to HC-27 retain their existing owners/gates. No files were staged, committed or pushed;
ARCH-10 has not started.

**ARCH-10 handoff (`197a059`) disposition.** See the [ARCH-10 record](arch-10-final-acceptance.md).

Resolved and awaiting independent acceptance (the user's decisions of 2026-10-17 for HC-19, HC-20 and HC-25):
- **HC-04:** retained by explicit decision.
- **HC-19:** bounded per preset.
- **HC-20:** both presets for shared build inputs.
- **HC-24:** planning graph aligned, and the alignment enforced.
- **HC-25:** a declared `launch` category.

Each row has its evidence.

Newly reported: HC-37, the tests that compile application sources (a documented arrangement).

Reported and needing no item:
- **Configured-graph gate:** `architecture_configured` and 27 negative fixtures were added, and both presets are accepted.
  Its assumptions are in the record:
  - a Python extension module is identified by a `python` directory;
  - an absent module needs its branch's option off;
  - a configure without Python is rejected.
- **Architecture scope:** the `architecture checker` scope now plans `ucrt64-release` too, at about 27 s.
- **Fixture development:** the new fixtures' first run errored in 2 cases, from a fixture-writer bug (a deleted
  target's dependents). It was fixed before the recorded runs.
- **Planning graph rewrite:** one inline list's formatting was restored, so the diff holds HC-24's changes alone.
- **Builds:** they compiled nothing, and the trees include other owners' uncommitted edits, as the provenance records.
- **Slot check:** an unrelated long-running process named `GCC` (from 2026-10-02) was left alone.
- **Before copies:** every edited file was copied and hashed first (`D:/ENV/tmp/arch10/before`).
- **Audit:** the planning source audit was refreshed, and its page carries a dated note.
- **Provenance capture:** the first capture dropped the leading dot of `.claude/CLAUDE.md`, from a stripped status
  line. It was fixed and recaptured before the commit.
- **Final documentation checks:** planning 51 documents; 106 files and 1,324 links, none broken; documentation tests
  3/3.

Existing open items:
- HC-02, HC-03, HC-05 to HC-07, HC-09, HC-18, HC-26, HC-27 and HC-34 to HC-36 are carried forward unchanged.
- HC-11's inventories are recorded, and its full run stays pending.
- No full suite or GUI scenario ran. ARCH-10 stops for independent acceptance.

**Independent ARCH-10 disposition (`197a059`).** The [review](arch-10-review-2026-10-17.md) accepts HC-19's bounded
escalation, HC-24's aligned planning graph, HC-25's launch category and HC-04's retained dependency rule. HC-20's
correction is incomplete (HC-39). New P2 items HC-38 to HC-43 block complete ARCH-10 acceptance: configured
include/preset validation, actual shared-CMake routing, required gate registration, source provenance and the
tooling-support contract need correction. Existing exact gates pass 5/5 headless (35.82 s), 2/2 Release (11.57 s),
but bounded reply/plan probes expose false passes. No complete inventory was run; 717/780 appear only in plans.

HC-37's application-source test arrangement is accepted and retained; library sources still require their owner.
The disclosed extra Release architecture coverage is appropriate. The fixture helper, leading-dot capture and
line-ending slips were corrected as reported, but source provenance still has the separately proven HC-41 gap.
Both inventories have 67 Python SDK tests, and the configured checker has 27 total cases including controls.
All 18 supplied before-copy hashes validate, original README/status owner payloads are preserved, and the 17/72
artifact hashes match. Planning inputs remain uncommitted and must be included in final provenance. The unrelated
`GCC` process was left alone. HC-35 stays pending human acknowledgment; no excluded content was inspected or
ignore policy changed. All existing functional/resource/maintenance carryovers and HC-07/HC-11 retain their gates.
No native configure/build, consumer rebuild, GUI or full suite was repeated; no files were staged, committed or
pushed. Stop for bounded corrections and independent recheck.

**HC-38 to HC-43 correction handoff (`872f561`, then its provenance commit) disposition.** See section 11 of the
[ARCH-10 record](arch-10-final-acceptance.md#11-hc-38-to-hc-43-correction-2026-10-17).

All six are corrected and await independent acceptance, as their rows say. The user decided HC-43 on 2026-10-17:
native CMake 3.25 support is kept; `architecture_configured` requires CMake 4.2+ and fails with an actionable
diagnostic below it, leaving acceptance pending; there is no fallback. HC-20 is complete through HC-39.

Reported and needing no item:
- **Red evidence:** on the pre-correction gate, 10 of 37 configured cases fail; on the pre-correction rules, 6 of the
  10 new selector cases fail. The controls pass on both.
- **Builds:** both presets reran CMake through the selector's `--build`; nothing compiled. The 17 and 72 artifact hashes
  equal those of the `197a059` record.
- **Development slips:**
  - the first gate edit shadowed a variable, and the real presets returned an input error until it was renamed;
  - a stray loop in the provenance script and a re-called `setUp` in a new test loop were removed before the recorded runs.
- **Listing:** `RulesTest` now also lists files with `--exclude=.codex` (HC-35's corrective measure).
- **Provenance verifier:** it is a scratch script (`D:/ENV/tmp/arch10-fix/provenance.py`, SHA-256 `c35a69cc33915f2e`),
  not a maintained tool. The record's `rule` field states what it checks.

Existing open items:
- HC-02, HC-03, HC-05 to HC-07, HC-09, HC-11, HC-18, HC-26, HC-27 and HC-34 to HC-36 are carried forward unchanged.
- No full suite, GUI scenario or package gate ran. The correction stops for independent acceptance.

**Independent HC-38 to HC-43 disposition (`f1bb879`).** The
[correction recheck](arch-10-review-2026-10-17.md#9-hc-38-to-hc-43-correction-acceptance-at-f1bb879) closes all six
and HC-20. Original SYSTEM, nested shared-CMake, missing-registration, wrong-build/disabled-branch and old-tooling
probes now behave correctly, with valid controls retained. Exact current gates pass 3/3 headless (17.50 s), 2/2
Release (11.63 s). Complete inventories appear only in plan probes; none ran.

Before any reviewer write, the manifest validates all 13 required base-commit inputs and exactly 58 working paths
(30 tracked/28 planning), plus 17/72 binary hashes, both 234-file data digests and both consumer manifests. All 17
supplied before-copy hashes validate, and shared owner payloads remain intact. `f1bb879` changes only the explicitly
excluded manifest after base `872f561`. Later seven-path review prose is a documented overlay; the manifest remains
dated evidence and is not silently recaptured after acceptance edits. Both commit IDs are now recorded here with
that distinction.

The scratch provenance helper is accepted for this dated capture, without claiming a maintained gate. Its full
SHA-256 is `c35a69cc33915f2e5c1cc35f2efef27d7917d8cc0a3c7b9ed9f5e49b77f56e8c`. It checks declared committed entries
and differing working paths, not required-set completeness or binaries/data; those were independently checked.
**Evidence owner, before future capture/reuse:** replace its post-filtered `rglob` with exclusions before descent
(HC-35 method), and validate the required package set for any completeness claim (HC-41 limit). No capture command
was executed here, and no actual excluded-directory access is inferred. These future-use limits do not reopen
the independently validated source snapshot or add a new mandatory code change to this package.

The shadowed variable, leftover loop and repeated fixture setup were corrected; the early CMake 4.1 claim was
superseded by the user's explicit 4.2 gate decision. The additional RulesTest listing exclusion is appropriate.
HC-35 still awaits human disposition, with no acknowledgment inferred. HC-04/19/24/25 and HC-37 remain accepted;
functional, resource and maintenance carryovers keep their owners. HC-07/HC-11 remain pending; complete ARCH-10,
full M8 and final three-domain acceptance are not claimed. No native configure/build, GUI, package gate or full
suite was repeated; nothing was staged, committed or pushed. Stop for the next user assignment; no pending run
is automatically authorized.

**HC-11 final regression handoff (`66dc43d`) disposition.** See the [HC-11 record](hc-11-final-regression.md).

HC-11's final regression has run and awaits independent acceptance:
- **Preconditions:** the review findings are closed (HC-38 to HC-43 and HC-20 at `f1bb879`), no register item is an
  open code defect, and the related fast checks passed.
- **Freeze:** the source tree and both build trees were fingerprinted.
- **Full suites:** `ucrt64-headless` passed 717/717 and `ucrt64-release` 780/780, each run once.
- **Snapshot:** unchanged after each run.

Reported and needing no item:
- **Fingerprint tool:** a new scratch tool (`D:/ENV/tmp/hc11/fingerprint.py`, SHA-256 `0f8f537cb3835456`), not the ARCH-10
  provenance helper. It prunes `.codex`, `.git` and `__pycache__` before descending, and its `verify` recomputes the
  whole file set; a trial rejected a changed digest, a dropped path and a changed artifact. This meets the review's
  condition for later captures.
- **Builds:** the incremental build compiled nothing, and no clean rebuild was part of the assignment.
- **Timeouts:** `--timeout 600` bounds the tests without their own timeout. Their slowest took 168.1 s (headless) and
  289.6 s (Release).
- **Draft slip:** the first draft of the record called that maximum 135 s, from older measurements. It was corrected
  from these runs before the commit.
- **Not run:** `ucrt64-debug`, which is not a required preset.
- **Overlay:** the evidence commit and the planning pages changed after the second verification, documentation only.
  Logs and JUnit files stay in `D:/ENV/tmp/hc11/`, with their hashes in the record.

Existing open items:
- HC-07 (Viewer, manual, performance and joint gates) and HC-06 stay pending.
- HC-02, HC-03 to HC-05, HC-09, HC-17, HC-18, HC-26, HC-27 and HC-34 to HC-37 keep their owners.
- HC-35 still awaits the user's acknowledgment.
- Final-program acceptance is not claimed.

**Independent HC-11 disposition (`66dc43d`).** The [review](hc-11-review-2026-10-17.md) closes HC-11. Saved
inventories, JUnit, raw starts/completions and time files agree on 717/717 headless and 780/780 Release CTest
entries, each unique and passed, with no skipped/unrun CTest entry. Headless has three expected internal optional
Vision skips (one package-import and two Vision unittest cases); Release runs and passes the counterparts. All
63 renderer entries passed, including actual 3840 x 2160 Viewer presented-frame checks. No suite was repeated.

Log/JUnit/helper/fingerprint hashes match, and both post-run verify logs report the covered snapshot unchanged.
Exactly six pre-review documentation overlays differ from the frozen 58-path record, with 52 hashes unchanged;
all six before-copy hashes validate and original owner payloads remain preserved. Commit `66dc43d` changes only
the declared four documentation/evidence files. Later reviewer edits are another explicit documentation overlay.

New **HC-44, nonblocking future evidence work**, qualifies the helper's scope. Its staged artifacts omit 30/36
test executables, example plugins and staged Python source; its inventory digest omits full commands/properties.
Current test DLLs match captured SDK DLLs, timestamps predate the freeze and no changed test input is established.
Before reuse, the evidence owner must extend any complete-test-input claim and add exclusions to tracked Git
queries before names return. Current capture shows no actual excluded-path access; HC-35 acknowledgment remains
the human's decision. This scope qualification does not reopen HC-11 or request a repeat full run.

The corrected 168.1/289.6-second default-timeout maxima, explicit longer timeouts, no-clean-build/Debug scope and
post-run documentation updates are reconciled. HC-07, HC-06, HC-02, HC-09 and other assigned carryovers remain;
full-program/complete ARCH-10 acceptance is not claimed. Only documentation fast checks run during review; no
native build, GUI, staging, commit or push is performed. Stop for the next user assignment.

**HC-07 verification handoff (`465b7af`) disposition.** See the [HC-07 record](hc-07-viewer-and-joint.md). HC-07 stays open.

Verified on the HC-11-tested Release binaries, at 3840 x 2160 fullscreen in one GUI slot:
- **Viewer scenarios:** the continuous live strike reproduces 2026-10-12's run. The replay frames, the demo and a world
  replaced under the same name pass.
- **Controls:** the demo's pause, step, time factor and quit pass through the window's key input (15/15).
- **Joint cases:** the generic ones passed in HC-11's suites.

Not met:
- **HC-45 (new):** the 4K performance gate fails for fleet, strike and hit.
- **Missing prerequisites (HC-06):** J5, J9, the combat parts of J3, J8 and J10, the combat Viewer run and the
  seed/aspect matrix.
- **Unverified:** a human-operated session, mouse, the timeline slider and resize.
- **Separate:** HC-02 (J6).

Newly reported:
- HC-45: the failing performance gate.
- HC-46: `--no-gui` still shows the Mission Systems windows, nonblocking.

Reported and needing no item:
- **First control check:** it passed 11 of 15, because it read a step counter the demo publisher does not maintain.
  It was corrected to the snapshot's simulation time and rerun: 15/15. Both logs are kept.
- **Data digest:** a quick digest of the staged data first disagreed with HC-11's because it ordered files
  differently. HC-11's own tool shows it unchanged.
- **Repeat run:** the performance gate was repeated once to tell a stable failure from load noise; both runs are
  reported.
- **Demo control column:** the demo's vehicle list leaves the control column empty, as in ARCH-08's capture.
- **Builds and suites:** no build, full suite or package gate ran.
- **Background load:** other applications stayed open during the runs and were left alone.

Existing open items:
- HC-02, HC-03 to HC-06, HC-09, HC-17, HC-18, HC-26, HC-27 and HC-34 to HC-37 keep their owners.
- HC-44 stays nonblocking.
- HC-35 awaits the user's acknowledgment.
- Final-program acceptance is not claimed.

**Independent HC-07 diagnostic disposition (`465b7af`).** The [review](hc-07-review-2026-10-17.md) accepts the
dated live native event/readback consistency, sampled replay/strike/demo images, 15 programmatic keyboard/setup/
lifecycle checks and observed same-process World replacement. All 20 evidence hashes, 17/72 HC-11 artifact
hashes, both 234-file data digests and eight before-copy hashes match. Owner payloads remain preserved and the
commit changes documentation/evidence only. Generic joint results retain HC-11's existing evidence.

All eight raw performance streams reproduce both reports: fleet/strike/hit fail twice, reheat passes twice.
HC-45 remains open; current load and changed hit workload establish neither regression nor its absence. Require
matched workloads/assets/settings and an idle host for a controlled comparison. Impact/fire timing is a profiling
lead, not a proven cause. HC-46's --no-gui panels are visible in current and historical samples, a pre-existing
nonblocking option-semantics issue. The demo's blank control column has the recorded lack-of-command explanation.

New HC-47 records diagnostic-helper limits: standalone-demo exit/remaining-publisher and staged-data-match summary
fields are assigned observations, not parsed measurements; data was independently checked here. The separate
control run measures exit/cleanup. Reconnect's script exit alone is insufficient without its actual log/image
evidence. Before automating these gates, require measured outcomes, replacement/frame/cleanup assertions and
negative controls. No new Viewer failure or full rerun is inferred.

The bad first clock probe, corrected reconnect keywords, initial crop placement, digest ordering and table-shape
slip are corrected evidence-tool history. No implementation defect is established by them. HC-06's combat
runtime/reduction inputs, HC-02 and mouse/replay/resize coverage remain pending; the keyboard probes do not
establish a human-operated session. HC-35 remains pending human disposition and HC-44 stays nonblocking.
HC-07/full-program acceptance remains open; HC-11 remains closed. Review edits are documentation only; no GUI,
benchmark, build or full suite was repeated, and nothing was staged, committed or pushed.

**HC-45 evidence handoff (`0a635d9`) disposition.** See the [HC-45 record](hc-45-performance-comparison.md). HC-45 stays open.

The review's measurements are done:
- **Idle host:** CPU 1-6 % and GPU 0-1 %.
- **Before/after comparison:** controlled, with matched inputs, against the Viewer of `7948dc4`, from before the
  migration. Both Viewers fail fleet, strike and hit; it establishes neither a regression nor equivalence (HC-48,
  corrected at `1e9433b`).

The user decided (2026-10-17) to record the evidence and stop rather than optimise. Meeting the gate is left to a
dedicated performance assignment, with hypotheses named (HC-50).

The new `--stats` line `gpu worst` (`render::GpuTimer`) passes the related Release selection, 68/68.

Reported and needing no item:
- **Old tree:** it was exported with `git archive` (not a worktree) and built in `D:/ENV/tmp/hc45/before-src`. Its
  `maps` junction was re-pointed to the repository's `assets/maps` for matched tiles, and its configure downloaded
  the project's sample model.
- **Escaped string:** the first diagnostics edit broke a string's escape through a shell heredoc. The build failed, and
  it was fixed before any run.
- **Binary changed:** the rebuilt Viewer (`2915dce58514c3f3`) is no longer the HC-11-tested binary; the change is
  diagnostic output only.
- **Background load:** the HC-07 runs' load is now known to explain most of their collapse.

Existing open items:
- HC-02, HC-03 to HC-07, HC-09, HC-17, HC-18, HC-26, HC-27, HC-34 to HC-37, HC-44, HC-46 and HC-47 keep their owners.
- HC-35 awaits the user's acknowledgment.

**Independent HC-45 disposition (`0a635d9`).** The [review](hc-45-review-2026-10-17.md) accepts the raw measurement
subset with qualifications, superseding the author handoff's absolute conclusions above. The four runs establish
matching pre-existing scene failures, not performance equivalence. CPU/GPU figures are pre-run samples; the
fraction of earlier slowdown caused by background load is not established. Window maxima and chosen-frame gaps
must remain separate, with Flow/transfer/fire-object attribution retained as profiling leads (HC-48/50).
HC-49's unavailable-query diagnostic path is reproduced with actual production source and a valid control, but
was not established in the saved runs and is nonblocking. The saved targeted tests are 68/68; four staged artifacts
now differ from HC-11, whose accepted full-run snapshot remains dated. HC-47 was not fixed. All handoff slips,
export/model/junction details and HC-35 acknowledgment remain recorded; no threshold/quality change or optimization
was made. No benchmark, Viewer or full suite was repeated. Only focused query/parser/document checks run.

**HC-47 to HC-50 correction handoff (`1e9433b`) disposition.** Awaiting independent acceptance. HC-45 stays open, its
optimization deferred by the user's decision; no performance scene was rerun.
- **HC-47:** fail-closed helpers in `D:/ENV/tmp/hc4750/helpers/`.
  - `demo_capture.py` asserts: the Viewer's exit code; its logged demo world `demo-<Viewer pid>` and the publisher
    process it mirrors; no publisher before or after; both segments released; the frame's size and log line.
  - `reconnect_check.py` asserts: each phase's published table (`old-one`, then `new-one`, distinct incarnations); the
    log's attach, one replacement, attach; frames 0-7 at 3840 x 2160 on both sides; exit 0; the segment released.
  - Negative controls: 14 and 13 doctored observations each fail, and the positive controls pass. One run of each
    passes on the current Release Viewer.
  - `summarize.py` reads the saved outcomes and recomputes the staged data's digest with HC-11's tree digest: both
    presets are still HC-11's. The original demo run's unrecorded exit and cleanup are now null. The
    [HC-07 summary](hc-07-evidence.json) is regenerated as `v2`; its other values are unchanged.
- **HC-48:** the A/B conclusion is narrowed in the [HC-45 record](hc-45-performance-comparison.md), the SDK validation
  page, status and this register. The same three scenes fail before the migration; paired differences are disclosed
  (strike -1.5/-1.4 fps); neither a regression nor equivalence is established.
- **HC-49:** `GpuTimer::addFrame`/`addMissing` measure a gap only between frames read one after the other; otherwise
  `n/a`, with an `unread` count; `largest gap` replaces `gaps max`. The review's probe gives `n/a` where it gave 7 ms;
  the control still gives 1 ms. Five `[gpu-timer]` cases in `fsim_render_tests` cover the probe, the control, a report
  boundary, backwards timestamps and an unread window. Related tests: 74/74 Release (121.86 s), 6/6 headless (8.62 s);
  `architecture_configured` passes on both presets.
- **HC-50:** the slow-frame table separates the GPU-slowest frame, the gap before it, the largest gap and the longest
  frame time. The pass groups are not single objects, and the leads are hypotheses. The
  [Viewer guide](../../sdk/viewer.md) now says what a gap can and cannot show.
- **Not run:** `tests/CMakeLists.txt` changed, so the selector plans both presets' full suites. As directed, they did
  not run and stay pending. The Release inventory is now 785 (780 plus 5); headless stays 717.
- **Binaries:** four staged Release artifacts differ from HC-11's, so HC-11 stays accepted for its frozen source:
  - the Viewer, `dd875197b212f60c`;
  - the Vision DLL in `bin` and `python/fsim/bin`, `855e87f78f4aa242`;
  - `_vision.pyd`, `412bd8343590f3e1`.
- **Self-reported:**
  - Each helper's first real run passed, but each was run again after a change to the helper (the demo's publisher
    identity check; the reconnect parser's world-name parameter), so that the recorded hashes are the ones that ran.
    Both attempts are kept.
  - A shell heredoc collapsed a backslash continuation in `summarize.py`, joining two lines. It was found on
    inspection and fixed before the script ran.
- **Preserved:** the reviewer's uncommitted edits stay in the working tree, outside the commit:
  - the HC-45 record's review paragraph;
  - the SDK validation page's HC-45 line;
  - the status page's HC-45 disposition;
  - the HC-07 record's status, demo row and section 7.

  Their descriptions of HC-47 to HC-50 as open are left for the reviewer to reconcile.
- **Unchanged:** existing open items keep their owners, and HC-35 awaits the user's acknowledgment.

**Independent HC-47 to HC-50 correction disposition (`1e9433b`).** The [recheck](hc-45-review-2026-10-17.md#6-hc-47-to-hc-50-correction-recheck-at-1e9433b)
closes HC-48/49/50. The exact five GPU-timer cases pass; no-regression and causal claims are properly qualified.
HC-47's measured outcomes, original-null correction, hashes and staged-data digests are accepted, but eight invalid
missing-data/order inputs still pass its helpers. Require complete typed measurements and reattach before new
frames; keep HC-47 open and add pure negatives, not new GUI/performance runs. Existing 15/14 helper self-tests
pass but miss those cases. All self-reported reruns, the corrected summary-script slip and owner overlaps are
reconciled in the review. HC-35 and other owners remain; HC-45 optimization stays deferred. Full-suite selection
is pending planning only; HC-11 remains dated accepted evidence. No product fix, staging, commit or push in review.

**HC-47 helper-check handoff (`d6bfb54`) disposition.** Awaiting independent acceptance. HC-47's measured runs, accepted
at the `1e9433b` recheck, are unchanged; only the checks changed. HC-45 optimization stays deferred.
- **Kept as they ran:** version 1 (`D:/ENV/tmp/hc4750/helpers/`), whose hashes the HC-07 summary lists, is not edited.
  Version 2 is in `D:/ENV/tmp/hc47b/helpers/`, with the same run path.
- **Missing, null and partial measurements:** `typed.need` makes each its own failure. Every demo and reconnect
  measurement is required and typed:
  - integer exit codes;
  - both segment results, as booleans;
  - publisher lists of process IDs;
  - both phases' vehicle, steps and table, with nonzero 32-bit, distinct incarnations;
  - each frame's size.
- **Probes:** `common.parse_tasklist` and `common.segment_state` give null when the query fails, or when Windows
  reports anything but not found. Under stubs, version 1 reads both as the passing answer.
- **Event order:** `reconnect_check.order_failures` holds the log to attach, frames, one replacement, reattach, frames.
- **Regressions:** `test_checks.py` gives 149 results, all as expected:
  - red/green on the review's eight records;
  - 47 demo and 54 reconnect negatives, every earlier negative kept, each rejected for its own reason
    (`negatives-explained.log`), with both valid controls;
  - saved-outcome and real-log valid controls;
  - captured and synthetic `tasklist` parsing, stubbed probes, and one missing-segment probe.
- **Summary:** the [HC-07 summary](hc-07-evidence.json) adds `helper_checks_v2` and its hashes; its other values are
  unchanged.
- **Checks:** The documentation checks pass: the planning checker (58 documents), the local-link audit (1,434 links in 113 files, none broken) and `ma_contract`, `architecture_boundary` and `sdk_guard_checker` (3/3).
- **Not run:** no GUI, Viewer, performance or full suite.
  - Version 2's run path has not run against a Viewer; before its reuse as an automated gate, one run of each
    helper remains.
  - The full suites selected at `1e9433b` stay pending.
- **Self-reported:**
  - A negative case was first written with a muddled conditional expression. It was replaced before the
    self-tests ran.
  - One red case first replicated version 1's parsing instead of calling it. It was replaced by version 1's own probes
    under stubs before the results were recorded.
- **Preserved:** the reviewer's uncommitted edits to status, the HC-07 record, this register, the index and the
  roadmap stay; the commit carries only this package's hunks.
- **Unchanged:** other open items keep their owners, and HC-35 awaits the user's acknowledgment.

**Independent HC-47 v2 acceptance (`d6bfb54`).** The [review](hc-45-review-2026-10-17.md#7-hc-47-version-2-correction-acceptance-at-d6bfb54)
closes the two remaining helper-check gaps. The actual regression script gives 149 expected results, including
the old/new eight-record contrast, strict missing/type checks, state-order checks and valid saved-run/real-log
controls. Unknown probe results are no longer passing absent/empty states. All 51 evidence hashes, 11 before-copy
hashes and shared owner payloads match. Both corrected self-reported fixture/red-test slips are reconciled.
V2's live path remains unexercised: record one run per helper before future automated reuse; it is not required
now and does not reopen the accepted check correction. HC-45, HC-07, HC-35 and other owners retain their limits;
HC-11 remains accepted for its dated frozen source. No GUI, performance, build or full suite ran in this review.

**HC-07 interaction handoff (`2a09756`) disposition.** Awaiting independent acceptance. HC-07 stays open on HC-45's
performance gate (optimization deferred), HC-06's combat cases, HC-02's J6 and a human-operated session. See the
[HC-07 record](hc-07-viewer-and-joint.md#mouse-replay-and-resize-2026-10-17).
- **Method:** keys, mouse buttons, drags and the wheel were posted to the Viewer's own window, and the window was
  resized, minimised, maximised and restored with the ordinary window calls. Input-driven, not a person.
- **Binaries:** the current Release build at 3840 x 2160 (Viewer `dd875197b212f60c`), not the HC-11-tested one.
- **Mouse** (21 frames, demo paused), passed:
  - left and middle drags rotate at the same distance;
  - the right button changes nothing;
  - the wheel moves 12 % a notch;
  - space + left drag does nothing locked to a vehicle and pans in the free camera;
  - the limits are 25 m (detached) and 50,000 km;
  - at the nearest, drags keep the view above the ground;
  - space pauses only without a drag.
- **Wheel, detached** (9 frames): zooms about the pointer, measured by registering the frames.
- **Replay** (16 and 5 frames), passed: space, `.`, `[`, `]`, home, the timeline slider, the pause box, the step
  button, and the loop at the end. The steps are exact from the first frame.
- **Resize** (7 frames, windowed), passed: every frame is its client area through resize, minimise and restore,
  maximise and restore, and 4:3; none is stretched or blank; the demo kept simulating.
- **Checks of the checks:** panel readings are taken by inspection and checked against each input's effect, with 18
  altered-reading controls rejected. Automatic measures: the pause flag, frame differences, sizes, simulation time,
  exit codes and cleanup.
- **Summary:** the [HC-07 summary](hc-07-evidence.json) is regenerated as v3. It adds `interaction`; its other values
  are unchanged.
- **Checks:** The documentation checks pass: the planning checker (58 documents), the local-link audit (1,446 links in 113 files, none broken) and `ma_contract`, `architecture_boundary` and `sdk_guard_checker` (3/3).
- **Found:** HC-51 (below), a nonblocking guide defect. It is not fixed here.
- **Self-reported:** four runs were repeated after faults in the checks, not the Viewer, and the first attempts are
  kept:
  - the mouse run's free camera began 40 m from the ground, and its wheel bursts were too slow for the frames;
  - the zoom run's undo was judged against a still frame's noise;
  - the resize run measured the demo's single-step count instead of its simulation time;
  - a probe's screenshot folder did not exist.

  Twice a shell heredoc halved a backslash, corrupting a string; both were found by inspection or by the parser and
  fixed before running.
- **Not run:** no full suite and no performance scene. HC-47 version 2's own run paths were not run; its probes were
  used for cleanup.
- **Preserved:** the reviewer's uncommitted edits stay, and the commit carries only this package's hunks.
- **Unchanged:** other open items keep their owners, and HC-35 awaits the user's acknowledgment.

**HC-51 correction handoff (`5c80892`) disposition.** Awaiting independent acceptance.
- **Basis:** no independent review of `2a09756` was found in the repository or its review evidence, so the correction
  follows HC-51's recorded closure condition.
- **Change:** documentation only. the [Viewer guide](../../sdk/viewer.md)'s camera bullet now says that space + left drag pans only when the camera is detached and does nothing locked to a vehicle; that the wheel zooms 12 % a notch, out to 50,000 km and in to 6 m from a followed vehicle (1.5 times a smaller one's radius, down to 5 cm) or 25 m from a detached focus; that locked it changes only the distance, while detached it zooms towards the ground under the pointer except beyond some 1,600 km; and that `--no-zoom-to-cursor`, or `camera.zoomToCursor: false` in `viewer.json`, turns that off.
- **Sources:** the camera code (`CameraController`: `kZoomPerNotch`, `kMaxDistance`, `kMinDistance`,
  `kMinDistanceRadii`, `kMinDistanceSmallest`, `kMinDistanceDetached`, `kFrameFromRadii`, `detached()`, and its
  space + drag and wheel handlers) and the Viewer's `--no-zoom-to-cursor` and `camera.zoomToCursor` parsing. The
  measured values are from `2a09756`'s checks.
- **Not measured:** the 6 m and 5 cm minimums, the 1,600 km framing range and the config key; these are from code.
- **Checks:** the selector plans the three documentation tests only. The documentation checks pass: the planned `architecture_boundary`, `ma_contract` and `sdk_guard_checker` (3/3),
the planning checker (58 documents) and the local-link audit (1,450 links in 113 files, none broken).
- **Records:** status, the HC-07 record, this register, the index and the roadmap. The reviewer's uncommitted edits
  stay; the commit carries only this package's hunks.
- **Unchanged:** HC-07 stays open on HC-45, HC-06, HC-02's J6 and a human-operated session. Other open items keep
  their owners, and HC-35 awaits the user's acknowledgment.

**Independent HC-07 interaction / HC-51 disposition (`2a09756`, `5c80892`).** The [review](hc-07-review-2026-10-17.md#6-interaction-subset-and-hc-51-acceptance-at-5c80892) accepts
the sampled programmatic interactions and closes HC-51. All 47 hashes, five frame aggregates and owner payloads
match; four reading sets and 18 negative controls pass. Current images and source support the corrected guide.
HC-52 retains a nonblocking resize-helper array-completeness gap, independently reproduced without a Viewer;
the actual saved data remain valid. Viewer differs from HC-11, while demo still matches. Earlier attempts, quoting
slips and the limited HC-47-v2 probe use are fully reconciled. Human/uncaptured interaction, HC-45, HC-06/HC-02
and HC-35 remain; no full run is authorized by this review. No GUI/performance/build/full suite repeated.

HC-00 and HC-01 were separate corrections and have separate closure evidence.
HC-10 was independently verified and closed later at `565ef3e`; it was not
silently closed by the earlier phase acceptance.

## 2. Documented limitations and temporary architecture arrangements

| ID | Origin and current disposition | Follow-up |
| --- | --- | --- |
| HC-47 | **Closed for helper-check correction, independently accepted at `d6bfb54`.** The dated v1 limits and intermediate `1e9433b` false accepts are historical; v2 rejects missing/null/partial/type errors and late reattach, while retaining valid saved-run controls. | 149 independent pure results as expected; all eight original false accepts reject on v2, and old/new probes preserve unknown results. Earlier helper/run hashes and evidence remain unchanged. Before future automated-gate reuse, evidence owner must exercise each v2 helper live and record exact hashes/outcomes; no live-path claim or GUI/full rerun now. See [acceptance](hc-45-review-2026-10-17.md#7-hc-47-version-2-correction-acceptance-at-d6bfb54). |
| HC-44 | Independent HC-11 review at `66dc43d`; **documented nonblocking fingerprint-scope limit**. Source and staged product artifacts are covered, but 30/36 native test executables, example plugins, staged Python and full CTest command/property metadata are not. Tracked Git listings also need exclusions before returning names. No changed test input or actual excluded-path access is established; HC-11 remains accepted | Evidence owner, before future helper reuse: cover actual executed inputs and full metadata for any complete identity claim, and prune/exclude before all directory/Git enumeration. Preserve the dated accepted snapshot and current limits; no automatic full rerun or current runtime fix is required. See [review](hc-11-review-2026-10-17.md#fingerprint-coverage-limit-hc-44-nonblocking) |
| HC-03 | M8's handoff and [pending/limits record](../../sdk/ma-training-validation.md#6-pending-and-limits): C2 commands and MA replies are sent once and are not retried; the lossy chain exercises reliable DMS messages rather than reliable C2 delivery | Retain as a modeled-delivery limit. Any reliability requirement needs a separately scoped design and acceptance; do not add retries as an incidental architecture change |
| HC-04 | **Retained tradeoff, independently accepted at `197a059`.** The root package eagerly imports its public API modules, so their import closure includes the Python tests importing `fsim`; this is real dependency coverage, as revisited in ARCH-07 | Retain the documented rule and import-closure regression. No selector false pass is inferred; later performance changes need their own dependency-preserving scope |
| HC-19 | **Closed, independently accepted at `197a059`.** Shared ordinary core changes now escalate only their own needed preset, preserving targeted Release renderer selection beside them | Keep mixed-change regressions and HC-11 execution controls. The real existing World/MonitorGui control plans headless 717/717 and Release 68/780. Shared build-system inputs are the separate HC-20/HC-39 correction |
| HC-20 | **Closed, independently accepted at `f1bb879` through HC-39.** Shared build inputs now plan and escalate both presets at actual depths; explicitly renderer-only paths retain Release-only routing | Retain nested solo/mixed and all-CMake regressions. Required inventories remain under HC-11; planning does not authorize execution |
| HC-05 | The selector handoff reported about 120 unclaimed tracked files, mostly metadata, standalone tools and testbench result data; the selector now refuses unclaimed targeted changes | Decide appropriate tests/scopes when those paths are changed. This is not an assertion that all such files require tests. The approximate historical count was not recomputed, and the fail-closed behavior is not an open selector defect |
| HC-16 | **Closed, independently accepted at `dd7dc9f`; complete ARCH-08 publisher lifecycle accepted**. The source/control split, removed Viewer/demo edge, exact joined holds, retained requests and timer lifetime now have independent evidence. Viewer links no demo/simulation/control code; the policy has no exception; native parity and startup/lifecycle controls pass | Retain read-only state, the forbidden-edge fixture and accepted native/command semantics. HC-29 to HC-32 remain closed. Manual/joint Viewer acceptance remains HC-07; package acceptance does not close those wider gates |
| HC-17 | ARCH-06 graph decisions retained: (1) legacy native demo implementation and (3) early Vision adapter move accepted; (2) **Viewer display-name decoupling accepted at `8c8bd8d`**; (4) **Python optional training/Vision composition accepted at ebd3a09** | fsim._visualization cameras import no VecEnv, and the outer fsim.vision VisionBatch composes both public APIs. Native BatchCameras remains the compatible adapter. For item (2), the neutral world-stream contract names the levels (`controlLevelName`), `World.cpp` asserts the counts agree, `test_control.cpp` checks every label against the control module, and the independent `include_view_viewer` gate passes. This does not close HC-28's new command-channel defect. The separate native image-provider reservation remains planned; its stale phase is HC-23 |
| HC-22 | **Explicitly accepted at ebd3a09 (2026-10-17)**: VecEnv.h retains its installed name, class signatures and opaque handle but no longer transitively exposes the C ABI | Mixed C/C++ source includes fsim/fsim_c.h explicitly, as vecenv.md documents. The independent old/current/explicit-include syntax probe gives pass/fail/pass. This is a documented source-include adjustment for the planned value split, not a claim of unchanged old-source compilation or a general compatibility waiver. ABI/signature/ownership requirements remain. See [decision](arch-07-review-2026-10-17.md#3-hc-22-explicit-compatibility-decision) |
| HC-23 | **Closed, independently accepted at `a59b46f` (2026-10-17)**: the stale ARCH-06 native training_vision reservation now waits for VIS-F | The committed policy keeps the provider planned; only planned-module phases accept SIM/TRN/VIS package names. Native/public exception expiry remains roadmap-only. The focused positive/refusal regression passes 1/1 (0.760 s). Workspace planning annotations retain planned-VIS-F and distinguish accepted Python VisionBatch from the future native provider. No provider was built. Policy/planning naming reconciliation is separately HC-24. See [final acceptance](arch-07-review-2026-10-17.md#6-final-hc-21-and-hc-23-acceptance-at-a59b46f) |
| HC-24 | **Closed, independently accepted at `197a059`.** The 30 planning native owners now match policy module names, targets, domains and destinations, including ipc, env/mission/sdk, Python adapters, app -> sim and the demo target | Retain the planning checker policy cross-check and drift refusals. Intentional future planning allowances remain distinct from actual configured edges |
| HC-25 | **Closed, independently accepted at `197a059`.** Per the user decision, launch is a declared tools category in policy/design; the temporary exception is removed, with no file move or launcher contract change | Retain the declared category, empty exception lists and FinalPolicy coverage |
| HC-26 | Claude's ARCH-08 handoff (`8c8bd8d`); text left naming the old tool paths on purpose. (1) Provenance in generated outputs: Hangar's output templates (`missiles.py`, `pieces.py`, `storebodies.py`, `storedesigns.py`, `storemodels.py`, `rotorcraft/heli.py`, `rotorcraft/multi.py`) and the aircraft and store files they wrote, and `flightsim-rcs`'s table method string `(tools/rcs)` in 36 committed `rcs.bin` tables. (2) Two hand-written comments in classified assets: `assets/clouds/shaders/fsim/coverage.glsl` and `assets/clouds/shaders/ocean/ocean.frag`. (3) One Catch2 case title, kept so its CTest name stays: `the propellant classes are NASA CEA's (tools/propellants/classes.json)`. Nothing reads these paths | Resource owners: update (1) with the next regeneration of those outputs, generator text and committed output together and checked by regeneration; (2) with the asset owners' next shader change; (3) only with an explicit CTest rename. A source move does not edit generated or classified data |
| HC-27 | Claude's ARCH-08 handoff (`8c8bd8d`); placement of operating-system process code. The Viewer's `DemoLink.cpp` starts, waits for and ends the demo publisher itself (`CreateProcessW` and `TerminateProcess` on Windows, `posix_spawn` elsewhere). Other operating-system services live in `foundation/platform`, and the publisher already checks its parent through `platform::processAlive` | Platform owner: move process launch and supervision into `foundation/platform` when a second application needs them, or accept application-local launching. No behavior change either way |
| HC-30 | Claude's original cancellation disclosure; P2. **Closed, independently accepted at `1dffece`**. Requests made while running and through resume remain pending, taken one each at a hold; quit may abandon them. Delayed-work and retention schedules, exact real-runner checks and current process test pass. Current process run takes all 27 pending at the hold, with one ordinary transition step; author run had 28 pending. That variable boundary is explicitly approximate | Retain cumulative requests and versioned stepsTaken acknowledgment while fixing HC-32. No request cancellation is authorized. Exact counts are supported by model/in-process tests, with the process snapshot boundary qualified. See [current acceptance](arch-08-review-2026-10-17.md#7-correction-recheck-at-1dffece) |
| HC-34 | Claude's ARCH-09 handoff (`4d7a9be`); a deferral the user decided on 2026-10-17. D07-D09, the MA training design, implementation plan and work allocation, stay canonical at their `docs/superpowers` paths. The register says not to move them during M8, and full M8 is pending (HC-06, HC-07). Their links to relocated documents were updated, and the architecture index lists them with their owner and status | Training feature owner, at the M8 feature handoff: relocate them per D07-D09, with redirects for `.claude/CLAUDE.md` and `AGENTS.md` or with those links updated (released for D20) |
| HC-36 | User's ARCH-09 reference decision (2026-10-17); **scheduled, nonblocking**. Source, test and public-header comments intentionally keep D02-D06's old document paths, answered by redirect-only stubs. The user requested that these retained references be recorded for later maintenance; this is distinct from HC-26's old tool provenance/comments | Owner of each affected file: update the document destination during that file's next related maintenance, preserving content and native behavior. Keep redirects while consumers still need them. No mass comment rewrite, extra native rebuild or full suite is required by this arrangement |
| HC-37 | **Documented arrangement, independently accepted at `197a059`.** fsim_demo_publisher_test compiles DemoSimulation.cpp and DemoLink.cpp; fsim_demo_steps_test compiles DemoSimulation.cpp. Applications expose no library target; the gate reports these test compilations and rejects library sources outside their owner | Application owners: preserve tests-of-application-source scope; link an application library if one is later extracted. Test include/link closure checks are outside this production-module gate; no further refactor is required here |
| HC-46 | Claude's HC-07 verification (`465b7af`); **nonblocking**; behavior since at least 2026-10-12, not a regression. `--no-gui` is documented as "just the rendered scene, no panels or labels". Yet the live strike's captures, made with it, show the Mission Systems windows, as the dated 2026-10-12 captures did | Viewer owner at the next scoped Viewer change: honor the documented no-panel behavior; a semantic change needs an explicit compatibility decision. Current/historical captures substantiate this pre-existing nonblocking issue |
| HC-52 | P3/nonblocking, independent HC-07 interaction review at `5c80892`: reusable resize helper accepts missing/empty/short size arrays and a one-element spread array; four pure false accepts reproduced. The actual saved seven frames and dimensions are independently valid | Evidence owner before automated reuse: require exactly seven correctly typed sizes/spreads, compare every client sample and reject missing/null/partial/malformed values; add pure negatives plus the valid saved control. No GUI/full/performance rerun needed. See [review](hc-07-review-2026-10-17.md#6-interaction-subset-and-hc-51-acceptance-at-5c80892) |
| HC-51 | Claude's HC-07 interaction verification (`2a09756`); **closed, independently accepted at `5c80892`**. The [Viewer guide](../../sdk/viewer.md)'s mouse description differs from the Viewer: the wheel zooms towards the pointer when the camera is detached (on unless `--no-zoom-to-cursor`), where the guide says it changes nothing but the distance and names a `--zoom-to-cursor` option that does not exist; locked to a vehicle, space + left drag does nothing; a detached camera's nearest distance is 25 m, not 6 m. Measured live (the zoom scales about the pointer: error 9.8, about the middle 15.9; nearest 25 m) and consistent with `CameraController` and the Viewer's option parsing ([HC-07 record](hc-07-viewer-and-joint.md#mouse-replay-and-resize-2026-10-17)) | Viewer documentation owner at the next guide change: describe the wheel per camera mode with the real option, the locked space + drag and the detached nearest distance. No behavior change is implied **Correction at `5c80892`:** the [Viewer guide](../../sdk/viewer.md)'s camera bullet now says that space + left drag pans only when the camera is detached and does nothing locked to a vehicle; that the wheel zooms 12 % a notch, out to 50,000 km and in to 6 m from a followed vehicle (1.5 times a smaller one's radius, down to 5 cm) or 25 m from a detached focus; that locked it changes only the distance, while detached it zooms towards the ground under the pointer except beyond some 1,600 km; and that `--no-zoom-to-cursor`, or `camera.zoomToCursor: false` in `viewer.json`, turns that off; from the camera code and the `2a09756` measurements  Source and sampled evidence match; no behavior change. See [acceptance](hc-07-review-2026-10-17.md#6-interaction-subset-and-hc-51-acceptance-at-5c80892). |

## 3. Feature prerequisites and acceptance still pending

| ID | Outstanding work | Source and closure boundary |
| --- | --- | --- |
| HC-06 | Full M5 runtime extension handlers and their confirmed registration/ownership seam; activation loading and gateway report integration; full M6 composition; versioned combat criteria, outcome reduction and rewards for full M7 | [Extension seam](../../sdk/mission-training.md#15-extension-seam-and-pending-gates) and [work allocation](../../superpowers/plans/2026-10-12-ma-system-training-work-allocation.md). Schemas and several native calls are already supplied; missing runtime integration and reduction contracts must not be described as missing everything. Noncombat/core acceptance does not close these gates **Checked again 2026-10-17 (HC-07, `465b7af`):** still not supplied - the engagement registry states `runtime_installed: false`, and no handler, registration seam or outcome reduction exists |
| HC-07 | **Open; diagnostic subset independently accepted at `465b7af`.** Full M8 J5/J9; combat portions of J3/J8/J10; complete installed-endpoint combat Viewer evidence and a published seed/aspect matrix | [Joint acceptance record](../../sdk/ma-training-validation.md#6-pending-and-limits). Generic chains use one deterministic seed/aspect each. Historical diagnostic Viewer evidence does not close joint acceptance. ARCH-06 (`61830e4`) re-ran the existing diagnostic Viewer scenarios before and after (live native strike, replay and demo, fullscreen 3840 x 2160); it does not remeasure the 4K performance gate and is not joint actor acceptance. The Vision split adds frame/pose collection allocations and reads, so image parity does not establish unchanged render cost. ARCH-08's scripted captures do not establish manual pause/step/time-factor/quit acceptance; native step parity also does not prove image parity. Retain these limits at the required visual/manual and performance milestones; this review authorizes no full performance run **Verified 2026-10-17 (evidence `465b7af`), HC-07 stays open:** on the HC-11-tested binaries at 3840 x 2160 fullscreen, the continuous live strike, the replay frames, the demo and live reconnection to a replaced world pass, as do the demo's pause, step, time factor and quit through the Viewer's window input (15/15). The 4K performance gate fails for fleet, strike and hit in two runs (HC-45). J5, J9, the combat parts of J3, J8 and J10, the combat Viewer run and the seed/aspect matrix lack their prerequisites (HC-06). A human-operated session, mouse, the timeline slider and resize remain unverified. See the [HC-07 record](hc-07-viewer-and-joint.md)  **Current review:** accept native/event consistency, visual samples, programmatic keyboard/setup/lifecycle checks and observed same-process World replacement. HC-45, HC-06/HC-02 and mouse/replay/resize remain open; no human-operated session is claimed. HC-47 qualifies standalone-demo observations and non-fail-closed reconnect output. See [review](hc-07-review-2026-10-17.md). **Interaction cases at `2a09756` (2026-10-17), awaiting independent acceptance:** mouse (rotate, the right button, the wheel, space + drag locked and free, the distance limits, the ground), replay (space, `.`, `[`, `]`, home, the timeline slider, the pause box, the step button, the loop) and window resize (resize, minimise, maximise, restore, 4:3) pass through input posted to the Viewer's window, with panel readings checked and controlled ([HC-07 record](hc-07-viewer-and-joint.md#mouse-replay-and-resize-2026-10-17)). Still open: HC-45, HC-06, HC-02's J6 and a human-operated session; HC-51 is new  **Current independent acceptance at `5c80892`:** sampled mouse/replay/resize subset accepted; earlier blanket unverified labels are historical. HC-51 closed; HC-52 qualifies future resize-helper reuse. HC-45, HC-06, HC-02, human operation and the stated untested panel/camera/option variants remain. See [review](hc-07-review-2026-10-17.md#6-interaction-subset-and-hc-51-acceptance-at-5c80892). |
| HC-08 | **Closed, independently accepted at `ebd3a09` (2026-10-17)**: the final public-header exception and both deferred Python cycles are removed | The common opaque handle lives in fsim_vecenv_handle.h; VecEnv's narrowed inclusion is explicitly accepted under HC-22. Training gets Error from its native owner; World/Agent values and registered factories live below both users. Policy lists no public/Python exceptions and the independent architecture gates pass. Earlier ipc/sim and visualization exceptions remain closed. See [ARCH-07 review](arch-07-review-2026-10-17.md) |
| HC-14 | **Closed, independently accepted at `8c8bd8d`**: the remaining headless application, RCS and strike-capture targets lost their broad roots. An independent current compile-database scan finds zero shared `src`/ancestor includes across 506 headless compilation entries (506 distinct sources) and 559 Release entries (557 distinct sources). The two repeated Release sources compile for application and test targets | Earlier per-module include acceptance is retained. Current `architecture_boundary` and `include_view_viewer` gates pass. This closes the broad-root item, not complete ARCH-08 acceptance or HC-28. See [review](arch-08-review-2026-10-17.md) |
| HC-15 | **Closed, independently accepted at `ebd3a09` (2026-10-17)**; the ARCH-04 thin-adapter placement decision is fulfilled | applyScenario moved unchanged to src/adapters/sdk/cpp/Scenario.cpp, retains a single definition, public signature and delegation to native session::applyScenario, and introduces no simulation-to-training exception. Native compatibility tests pass; author object, ABI and frozen-consumer evidence is retained. See [ARCH-07 review](arch-07-review-2026-10-17.md) |
| HC-21 | **Closed, independently accepted at `a59b46f` (2026-10-17); P056 complete**. The user released isolation.py subject to preserving uncommitted edits | Codec and actor-side proxies/loop live in fsim._adapters; Range hosts/checks remain in fsim.isolation. Public identities, typed serialization, error rebuilding, callable-method lists and episode generation/referee checks are retained. Independent saved-before/current unstaged payloads match for isolation.py (8 added/8 removed lines, 4 hunks), actor fixture and check.py. Exact headless gates pass 5/5, Release 2/2. Author identities, ABI and installed-consumer evidence remain qualified. See [final acceptance](arch-07-review-2026-10-17.md#6-final-hc-21-and-hc-23-acceptance-at-a59b46f). ARCH-07 is complete; ARCH-08 has not started |

The original physical-model and graphics goals remain separate programs of
work: [simulation plan](../modules/simulation/plan.md) and
[visualization plan](../modules/visualization/plan.md). Their retained MS
calibration, DLZ/WEZ, sensor/electronic-warfare and Viewer quality/performance
goals are planned scope, not newly discovered runtime regressions. In
particular, the dated Viewer performance and visual limitations have not been
remeasured during this reconciliation.

## 4. Record consistency

| ID | State | Required correction |
| --- | --- | --- |
| HC-09 | Older M5/M6/M7 status/contract summaries need reconciliation, and M8 generic lacks one final independent acceptance summary. The padding-comment sub-item is **closed at `30bc5ef`**, verified as an accurate comment-only change | Keep historical scope and remaining summary reconciliation visible. ARCH-01/02 and HC-10 summaries are aligned. Do not infer a runtime defect from stale labels or mark full M8 complete from generic corrections |
| HC-33 | Claude's ARCH-09 handoff, P2 at `4d7a9be`; **closed, independently accepted at `6bb79cc` with the inspected planning/index package; ARCH-09 complete**. D23-D25 now identify category/owner/status and direct index reachability for ADR-26, ADR-27 and the build/run packaging plan | Retain current-authority, retained-domain and historical classifications respectively; ADR-27 remains in force for rotorcraft. Bodies match before hashes and the prior 22 register rows are unchanged. No move is required. See [final review](arch-09-review-2026-10-17.md#7-final-hc-33-and-arch-09-acceptance-at-6bb79cc); preserve uncommitted-package provenance and existing evidence limits |

The authoritative current record must say which package and revision were
accepted. A successful narrow selector review does not establish full combat
or Viewer acceptance, and source relocation does not establish model quality.

### Workspace observation (nonblocking)

| ID | State | Follow-up |
| --- | --- | --- |
| HC-18 | The ARCH-06 handoff reports that untracked `bicycle-rides-pelican.html` disappeared. The named path is currently absent; the commit/index do not contain its removal. Attribution and whether the removal was intentional are unverified. This is a workspace event, not an established ARCH-06 code defect | Retain the report for the original artifact/workspace owner to confirm intentional deletion or arrange recovery from known source. Do not attribute deletion to another process solely from Git status, recreate an unrelated artifact, or inspect excluded directories/other worktrees. It does not block ARCH-06 |

## 5. Historical issues with later correction evidence

| Earlier self-report or review | Reconciliation |
| --- | --- |
| M5 reset serialization, Python child lifetime, parsed referee-document refusal, endpoint ownership and gateway-receipt checker | Corrected in `57a846d`, with later review/verification records; do not count the original reproductions as still-open defects |
| M6 due-north bearing displayed as 2 pi | Current [planner](../../../examples/training/ma_composition/planner.py) explicitly normalizes the near-2-pi result to zero. Source correction found; no runtime retest was performed for this audit |
| M6 consumer assertions failed for an incidental actor error rather than testing the intended refusal | Handoff records the corrected echo/control and specific refusal assertions plus mutation evidence |
| M7's five core findings and the preopened-child/batch transition hold | Corrected through `fc59959` and `0cf0150`; M7 core acceptance is recorded. Combat criteria/rewards remain HC-06, not a reopened core lifetime defect |
| M8 comparator false passes, stale/missing captures, installed-resource fallback masking and J8 reset preconditions | Corrections are recorded in `3802e7a` and `4a15aae`, with subsequent focused verification. These do not close HC-02 or HC-07 |
| Selector T1-T4, failed-run timing history and newly added Python tests missing CTest registration | Corrected through `4a15aae` and `7948dc4`; the missing-registration fix was independently accepted. HC-04/HC-05 are separate limits |
| ARCH-01's five findings, Windows path casing and wildcard import bindings | Independently accepted at `7625529`; see [review](arch-01-review-2026-10-16.md) |
| Intermediate quoting mistakes, temporary compilation failures and incorrect Python package argument | The same handoffs record their correction and later successful gates; they are not current backlog items |

## 6. Handoff reconciliation rule

User requirement: every Claude handoff must be read and reconciled in full.
Self-reported problems must be recorded for correction, including problems
described as pre-existing, cosmetic, out of scope or left for another package.

For every subsequent handoff:

1. Read the complete supplied message and each explicitly supplied attachment,
   including intermediate discoveries, the final summary, limits and pending
   sections. Split truncated output into further reads. Keyword extraction
   assists the review but does not replace reading the handoff.
2. Extract every reported defect, missing test/evidence item, limitation and
   deferred task. A passing suite or a successful commit does not dispose of
   an issue reported elsewhere in the handoff.
3. Give each unresolved item a stable ID here, or link it to an existing ID.
   Record its source handoff/revision, description, category, evidence,
   correction owner, intended package/priority and closure condition. If the
   current state is uncertain, record it as unverified and check it; do not
   silently omit it or label it fixed.
4. Reconcile the package's review findings, the implementer's self-reported
   issues/limits and this register's open carryovers before issuing a verdict.
   Report the disposition of newly mentioned items explicitly.
5. Close an item only with correction/acceptance evidence and its revision.
   Preserve its history. Deferral and an out-of-scope classification keep an
   item open; a documented design limit keeps its decision/status visible.
6. Carry open items into subsequent handoffs until their named correction
   package closes them. Phase acceptance does not erase independent defects
   or deferred functional gates. Work already assigned to another owner keeps
   that owner; recording it does not authorize concurrent changes.

Current execution destinations are:

| IDs | Correction or follow-up owner/package |
| --- | --- |
| HC-00 | Closed at `dc09e8f`; retain the independent acceptance evidence |
| HC-01 | Closed at `dc09e8f`; retain the independent acceptance evidence |
| HC-02 | Claude's J6 acceptance package; any demonstrated native defect stays with its existing implementation owner |
| HC-03 | Explicit C2 reliability requirement/design decision before any delivery-semantics change |
| HC-04 | Retained tradeoff independently accepted at `197a059`; preserve the documented import dependency rule |
| HC-05 | Owner of each affected file and its test/scope change when that path is touched |
| HC-06 | Existing extension-contract/native-input owners and Claude's generic integration package, per the accepted allocation |
| HC-07 | Open; diagnostic and sampled window-input interactions accepted. HC-45 performance owner, HC-06 combat owners, HC-02 J6 and human/uncaptured interaction gates remain; HC-51 closed, HC-52 nonblocking; HC-11 stays dated accepted |
| HC-08 | Closed at `ebd3a09`; retain public-value and Python-cycle acceptance evidence |
| HC-09 | Documentation owner at the next related handoff, reconciled against the dated acceptance evidence |
| HC-10 | Closed at `565ef3e`; retain raw shared-memory and recording compatibility evidence |
| HC-11 | Closed at evidence `66dc43d` for the frozen effective source at `f1bb879`; retain exact full-run evidence and scope, with HC-44 future helper work; no repeat run required |
| HC-12 | Closed at `ad12a44`; retain the independent include-boundary and ancestor-root acceptance evidence |
| HC-13 | Closed at `74efb49`; retain renderer-routing, source/shared-scope coverage and independent regression evidence |
| HC-14 | Closed at `8c8bd8d`; retain the independent compile-inventory and Viewer include-boundary evidence |
| HC-15 | Closed at `ebd3a09`; retain the registered applyScenario relocation, delegation and compatibility evidence |
| HC-16 | Closed at `dd7dc9f`; retain complete publisher lifecycle and removed-edge acceptance evidence |
| HC-17 | Item (2) accepted at `8c8bd8d`; Python optional training/Vision composition is accepted at `ebd3a09`; native reservation metadata is HC-23 |
| HC-18 | Original artifact/workspace owner: establish whether the reported removal was intentional; no reconstruction or cross-worktree investigation is authorized by this review |
| HC-19 | Closed at `197a059`; retain per-preset escalation and ordinary-core/render mixed controls |
| HC-20 | Closed at `f1bb879` through HC-39; retain real-depth shared CMake and render-only routing coverage |
| HC-21 | Closed at `a59b46f`; retain extraction, process/identity and preserved-unstaged-edit acceptance evidence |
| HC-22 | Accepted source-include decision at `ebd3a09`; retain the old/current/explicit-include syntax proof and public documentation |
| HC-23 | Closed at `a59b46f`; native provider remains planned VIS-F, with roadmap-only exception expiry retained |
| HC-24 | Closed at `197a059`; retain planning/policy alignment and drift checks |
| HC-25 | Closed at `197a059`; retain declared launch category and empty exception lists |
| HC-26 | Resource owners, with the next regeneration of the affected outputs or change of the assets; the CTest title only with an explicit rename |
| HC-27 | Platform owner, when a second application needs process launching |
| HC-28 | Closed at `1dffece`; retain deterministic completion/hold and delayed-work regressions |
| HC-29 | Closed at `ecf58a3`; retain independent handle-cleanup/lifecycle evidence |
| HC-30 | Closed at `1dffece`; retain request persistence across running/resume |
| HC-31 | Closed at `1dffece`; retain required-name coverage and fail-before-run regressions |
| HC-32 | Closed at `dd7dc9f`; retain timer/thread-lifetime ownership and independent stable-handle evidence |
| HC-33 | Closed at `6bb79cc` with the inspected planning/index package; retain D23-D25 metadata, direct index reachability, unchanged bodies and ADR-27's continuing force |
| HC-34 | Training feature owner at the M8 handoff |
| HC-35 | Human/instruction owner disposition remains pending; require pre-traversal exclusions and disable untracked enumeration for status queries; Git-ignore policy is not changed by this review |
| HC-36 | Corresponding source/test/public-header file owners during next related maintenance; preserve compatibility redirects, no mass rewrite |
| HC-37 | Arrangement independently accepted at `197a059`; application owners retain the two reported app-source tests and library-source ownership checks |
| HC-38 | Closed at `f1bb879`; Retain owned-path-first classification and both SYSTEM refusal regressions |
| HC-39 | Closed at `f1bb879`; Retain all-CMake routing coverage, explicit renderer-only paths and HC-19 ordinary-source bounds |
| HC-40 | Closed at `f1bb879`; Retain required-name checks, build-refresh refusals and successful-registration controls |
| HC-41 | Closed at `f1bb879`; Retain base 872f561, provenance-only f1bb879, explicit self exclusion and the dated-snapshot/review-overlay distinction. The scratch helper is one-off; future reuse needs pre-descent exclusions and independently validated required-set/artifact claims |
| HC-42 | Closed at `f1bb879`; Retain source/build identity, disabled-branch refusals and enabled-branch completeness |
| HC-43 | Closed at `f1bb879`; Retain configure warning, version/capability fixtures and no fallback; native 3.25 compilation was not rerun here |
| HC-44 | Evidence owner before next fingerprint-helper reuse: extend executed-input/inventory coverage and pre-query exclusions; nonblocking for accepted HC-11, no automatic rerun |
| HC-45 | Visualization/performance owner: matched measurement subset accepted; HC-48/49/50 independently closed at `1e9433b`; meet unchanged performance/quality gate in a dedicated assignment; optimization still deferred |
| HC-46 | Viewer owner at the next scoped Viewer change: honor documented no-panel behavior; any semantic change requires an explicit compatibility decision |
| HC-47 | Closed for check logic at `d6bfb54`; evidence owner retains future prerequisite: one measured live run per v2 helper before automated-gate reuse. No GUI/performance/full run requested in this assignment |
| HC-48 | Closed, independently accepted at `1e9433b`. Evidence owner: qualify sampled A/B comparison and causal confidence across current records; no extra benchmark requested |
| HC-49 | Closed, independently accepted at `1e9433b`. Visualization owner: nonblocking missing-query adjacency fix and fast production-source regression/control |
| HC-50 | Closed, independently accepted at `1e9433b`. Evidence owner: correct frame/max-gap associations and causal/idle interpretation before accepting diagnostic conclusions |
| HC-51 | Closed, independently accepted at `5c80892`; retain camera-mode-specific guide and actual option/settings names |
| HC-52 | Evidence owner before automated resize-check reuse: complete typed seven-entry arrays and negative/valid controls; nonblocking for accepted dated images, no GUI/performance/full rerun |

Use change-relevant fast tests for each correction. This register does not
authorize a new full suite, a conflicting build, an incidental model change or
the next implementation phase.

## SIM-M03 audit follow-up, 2026-10-20

These findings were observed at main `0d4f25a5df3fdd23696e696644cbd0ee32ab23af`
while implementing the independent SIM-M03 correctness continuation. Their
[evidence and acceptance boundary](../../sdk/weapon-fidelity-validation.md)
retain the separate SIM-M02 research and sensor/model dependencies.

| ID | Origin and state | Item | Owner and required closure |
| --- | --- | --- | --- |
| HC-53 | Existing baseline defect reproduced at the SIM-M03 handoff; **closed by the 2026-10-20 focused correction** | The external OpenVSP CMake script now has an exact-path scope with a required aero reference guard. Its own change requires no SDK build; the all-CMake invariant recognizes only this external exception. Missing-guard refusal and mixed shared-SDK escalation are covered. The earlier 100/101 result remains historical | Test-infrastructure owner: retain the bounded exception, fail-before-run requirement and ordinary/shared CMake controls. Current checks pass 2/2 on each Headless/Release inventory, with 105/105 selector and 150/150 reference methods in each. See [HC-53 evidence](hc-53-test-routing-2026-10-20.md); external geometry/solver acceptance remains separate |
| HC-54 | Existing model timing limit found in the independent native audit; open | Dispensers advance before the seeker's observation pass; fuze history reconstructs a previous decoy position from final velocity despite nonlinear drag/gravity integration | Codex SIM-M03 consumption, with the countermeasure model owner: freeze actual sample times and before/after decoy poses, verify observation/loss/capture and swept-fuze cases across native cadences, then integrate a separately reviewed correction |
| HC-55 | Existing native lethality geometry limit; open | Subsequent damage assessments receive the original complete Airframe after sections have separated; detached-body damage needs its own explicit contract | Codex SIM-M03 lethality: assess attached geometry consistently with native separation state and freeze supported detached-body behavior within the admitted entity scope; add repeat-burst/separation fixtures before changing production behavior |
| HC-56 | External physical-model and feature prerequisites; open, full SIM-M03 remains unaccepted | Calibrated seeker/IR/gimbal and fin/autopilot response, actual fuze/fragment/vulnerability inputs, modeled weapon feedback/assessment, accepted fused input and complete per-variant/fleet evidence are missing | Codex SIM-M03 owns complete native/public/model delivery; consume independently accepted SIM-M02 and sensor inputs. Claude retains the SIM-M01 fused-report feature and generic schema/gateway/installed integration seams. Freeze provenance, uncertainty and independent quantitative cases; complete native/public/installed/Viewer evidence without treating these software corrections as model calibration |
