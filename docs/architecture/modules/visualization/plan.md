# Visualization module plan

## 1. Responsibility and final boundary

Visualization owns presentation: scene construction, live/replay display, UI, cameras, offscreen images and visual effects. Its implementation root is src/visualization.

Simulation state is input. Visualization never advances the authoritative World, selects guidance outcomes, creates damage, replays actor decisions as physics or broadens an actor's knowledge.

A separate outer demo or operator application may submit explicitly authorized simulation commands. Such composition belongs in applications/adapters, not the scene/rendering library.

## 2. Current source owners

| Current source | Existing role | Destination |
| --- | --- | --- |
| src/render | VSG/Vulkan rendering backend | visualization/render |
| Earth, terrain, sky, clouds and VehicleVisuals in src/world | Scene composition | visualization/scene |
| Cameras/frames/presentation interpolation in src/world | Read-only view construction | visualization/scene |
| FlowSolver/FlowFires | Separate optional GPU solver target | visualization/vfx/flow |
| FlowVolumes, exhaust, airflow, trails and vapour | Visual effects | visualization/vfx/effects |
| src/ui | Presentation/UI | visualization/ui |
| src/vision offscreen rendering | Camera/image generation | visualization/vision |
| World-handle/C API part of src/vision | Compatible Range-side input adapter | adapters/vision |
| src/app/viewer_main.cpp live/replay | Passive Viewer composition | applications/viewer |
| viewer --demo and DemoAutopilot | Native simulation demo composition | applications/demo |

src/effects is not part of this module: it changes native/sensed physical state and belongs under simulation/environment.

## 3. Shared data and transport

IPC/recording value layouts belong below both producer and renderer. OS shared-memory and versioned recording mechanisms belong in foundation/transport, not inside the Viewer.

Preserve the current IPC layout/version, state array strides and old recording readers/fixtures. Directory migration cannot silently change an embedded native state representation.

The live source uses WorldMirror; replay reads recordings. Both produce a neutral presentation frame with simulation time/generation, poses, model/asset identity and event metadata. Renderer interpolation is distinct from native events and cannot be used to infer a hit.

## 4. Render/core separation

Keep Vulkan/VSG/Flow and GPU resource lifetimes entirely in visualization or its optional adapters. The generic foundation platform layer can provide OS loader primitives without including GPU headers or linking graphics libraries.

Scene code consumes immutable frames/contracts. Replace remaining private simulation includes with lower value definitions after contract extraction. Do not move simulation implementation into contracts to make a graphics import legal.

Keep asset decoding/resolution in named headless infrastructure where appropriate, but scene graph objects, textures, pipelines, materials and image readback stay optional.

## 5. Viewer applications

Split the current entrypoint by source/command ownership:

- Live viewer: attach to a selected published World and consume frames/events.
- Replay viewer: read a supported recording and consume its frames/events.
- Demo publisher: own the legacy flight scenario and scripted inputs through the existing native flight implementation. HC-17.1 preserves this behavior; ARCH-08 separates its publisher process and read-only stream without silently replacing it with a different World scenario.
- Operator control adapter: explicitly authorized commands, separate from display state.

Preserve existing CLI behavior during transition with a thin dispatch entrypoint. A pure viewer target must not link training or own FDM stepping. If the demo remains in the same executable temporarily, its dependency is declared as an outer composition target and cannot leak into the scene library.

No new network server or generic message framework is needed merely to separate these entrypoints.

## 6. Vision and training images

Vision renders offscreen from a read-only source or compatible Range handle. Sensors::render does not step the World.

Split the current Python vision wrapper's VecEnv-specific composition from camera implementation. An optional application/provider may consume both training and Vision APIs. Core training builds and imports without the graphics backend.

For image observations, define:

- Source service/platform, camera mounting and permitted view.
- Simulation generation/tick/time associated with the image.
- Image dimensions/format, validity, latency and buffer ownership.
- Stable lifetime across reset/close and refusal of stale requests.
- Which metadata is diagnostic/referee-only.
- Whether dropping a visual frame affects only presentation or an admitted observation cadence.

Do not automatically expose Range segmentation IDs, hidden entities or preflight truth to actors. The permitted image-report contract is a separate integration gate.

## 7. VFX and state authority

Flow, trails, plume, condensation and fire presentation read native state/events. Native simulation owns detonations, fires/damage sources, separation and object lifetime. A visual effect cannot schedule a destruction outcome to fit a recording.

Flow GPU stepping and render timing must preserve the established presentation behavior. Isolate the Win32 dynamic loader through a private visualization/platform adapter or generic opaque platform service; do not make headless core depend on Flow.

Keep physical plume/IR/sensor modeling separate from a visually convincing effect. A screenshot cannot calibrate a seeker or an aircraft's signature.

## 8. Work packages

| ID | Prerequisite | Output | Acceptance |
| --- | --- | --- | --- |
| VIS-A | ARCH-02 contracts | Neutral presentation/source interfaces and classified IPC/recording dependencies | No simulation-to-graphics edge; byte/layout checks |
| VIS-B | ARCH-03 support moves | Render, scene and physical/visual effects directory separation | Ownership checks; optional-target graph in headless checkout |
| VIS-C | VIS-A/B | Flow and visual-effect owners isolated, loader boundary explicit | Render build; calibration regressions; no headless GPU symbols |
| VIS-D | VIS-A/B | Offscreen rendering separated from World-handle and Python adapters | Vision lifecycle/readback/type/ABI tests; no World stepping |
| VIS-E | VIS-A | Viewer live/replay/demo source adapters separated | CLI compatibility, legacy replay, native output parity |
| VIS-F | Training provider contract + VIS-D | Optional training-image integration outside core domains | Permitted-view/metadata/lifetime tests; no truth in actor/masks |
| VIS-G | Completed moves and current native scenarios | Complete Viewer evidence set | Fullscreen run, native events, screenshots, lifecycle and performance |
| VIS-H | Separately scoped rendering backlog | Materials/shadows/performance fixes | Before/after measured visual acceptance without model retuning |

Structural and visual-quality work are separate packages. A passed render build does not close VIS-G/H.

## 9. Required acceptance matrix

### Headless

Build/test without rendering or Python SDK availability as applicable. Reject Vulkan/VSG/Flow imports and links in simulation/training/support targets. Scan optional visualization declarations even when rendering is off.

### Live and replay

Use the exact native World/recording being verified. Confirm release, flight, aftermath, entity replacement and reset/reconnect lifetime. Replay supported old recording versions and preserve event meaning.

A diagnostic range-driven scene is labeled diagnostic. It is not evidence that isolated actors completed the installed public endpoint chain.

### Images and resources

Verify image dimensions/formats, copied/borrowed buffer lifetime, camera transform, public state mapping, segmentation/truth classification and installed model/resource provenance. Inspect actual loaded paths.

### Fullscreen and performance

Use one GUI slot for a complete scenario. Retain exactly two event-relative strike captures when that is the scenario contract. Report event times, selected entities, build/GPU/driver/configuration, frame timing and capture cadence.

Measure the fixed scene/camera/resolution workload after any rendering or asset change. Do not reuse a historical frame-rate claim for changed code or a different scene.

### Output-only parity

Compare native trajectories/events with publication, recording, focus, observer count and capture rate changed. Camera/rendering work must not alter authoritative step timing or simulation RNG.

## 10. Existing evidence limits

The [dated Viewer record](../../../sdk/viewer-validation.md) remains scoped evidence. At that baseline, three of four scenes failed the 4K performance gate; replacement native-hit performance was not remeasured; aircraft cast shadows were not enabled; material detail remained unchanged; manual interaction and Vulkan validation-layer acceptance had pending scope.

These statements are not fresh GPU measurements. Keep them pending until new evidence closes the exact cases. Native engagement/model acceptance, training joint acceptance and Viewer acceptance are separate.

## 11. Exit

All visualization implementation is in its assigned root; stable legacy adapters are classified separately; Viewer/Vision consume read-only contracts; no graphics dependency reaches headless domains; required runtime/GPU/installed tests and complete visual scenarios have current evidence. The [acceptance matrix](../../validation/acceptance.md) governs final closure.
