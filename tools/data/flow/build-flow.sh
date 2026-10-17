#!/usr/bin/env bash
# Builds NVIDIA Flow 2 (BSD-3-Clause), the viewer's fire and smoke solver (world::FlowVolumes, cmake/Flow.cmake), from
# NVIDIA-Omniverse/PhysX's flow/ folder at the pinned commit, then configure flightsim with
#   -DFSIM_FLOW_DIR=<checkout>/flow
#
# Usage (Git Bash): tools/data/flow/build-flow.sh <checkout dir> [MSBuild.exe]
#
# Flow builds with MSVC (its own premake, Slang and DXC come from NVIDIA's packman); flightsim only loads its two DLLs
# through their C interface, so the MinGW build is unaffected. What it took here (2026-10-09, MSVC 14.44 Build Tools):
# - build.bat's packman pull works, but its VsWhere does not see a Build Tools-only Visual Studio: premake and MSBuild
#   are run directly instead. (packman sets the user variable PM_PACKAGES_ROOT and caches its packages there.)
# - MSVC 14.44 warns C4756 on INFINITY and C4819 on a non-UTF-8 code page, both errors under Flow's /WX: CL adds
#   /wd4756 /utf-8.
set -euo pipefail

FLOW_COMMIT=6dc53585d7db07346f53d0598a5fd3bfdce47d8d # Flow 2.2.0
DIR=${1:?usage: build-flow.sh <checkout dir> [MSBuild.exe]}
MSBUILD=${2:-"/c/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/MSBuild.exe"}

if [ ! -d "$DIR/.git" ]; then
    git clone --filter=blob:none --no-checkout https://github.com/NVIDIA-Omniverse/PhysX.git "$DIR"
    git -C "$DIR" sparse-checkout set flow
fi
git -C "$DIR" fetch --filter=blob:none origin "$FLOW_COMMIT"
git -C "$DIR" checkout --detach "$FLOW_COMMIT"

cd "$DIR/flow"
# its dependencies (premake, Slang, DXC) from packman
cmd //c build.bat || true # (fails at its Visual Studio check once the packages are in)
./external/premake/windows-x86_64/premake5.exe --file=premake5.lua vs2022
MSYS_NO_PATHCONV=1 CL="/wd4756 /utf-8" "$MSBUILD" "_compiler\\vs2022\\nvflow.sln" /p:Configuration=release /m
ls -l _build/windows-x86_64/release/nvflow.dll _build/windows-x86_64/release/nvflowext.dll
echo "Flow built: configure flightsim with -DFSIM_FLOW_DIR=$DIR/flow"
