# NVIDIA Flow 2 (vendored)

The viewer's fire and smoke solver (`src/visualization/vfx/flow/FlowSolver.cpp`, `cmake/Flow.cmake`).

- Source: https://github.com/NVIDIA-Omniverse/PhysX, its `flow/` folder, commit
  `6dc53585d7db07346f53d0598a5fd3bfdce47d8d` (Flow 2.2.0). BSD-3-Clause: `LICENSE.md`.
- `include/`: its public headers, unchanged.
- `bin/windows-x86_64/nvflow.dll`, `nvflowext.dll`: its release build, made by `tools/flow/build-flow.sh` (MSVC 14.44,
  2026-10-09). flightsim loads them at run time through Flow's own loader; nothing of them links.

To build them again, or another commit: `tools/flow/build-flow.sh <checkout dir>`, then either copy the two DLLs and
`flow/include` here or configure flightsim with `-DFSIM_FLOW_DIR=<checkout dir>/flow`.
