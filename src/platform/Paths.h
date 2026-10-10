#pragma once

#include <filesystem>

namespace fsim::platform {

/// Directory containing the running executable.
std::filesystem::path executableDir();

/// Directory containing the module this code is linked into: the executable
/// for the platform's own programs, but libfsim.dll's directory when the SDK
/// is loaded by a host that is not ours (Python, Rust, C#), whose
/// executableDir() says nothing about where the platform's files are.
std::filesystem::path moduleDir();

/// Where the program writes its logs and crash dumps: `logs/` beside it
/// (<module>/../logs - bin/../logs for the platform's programs, beside
/// libfsim.dll's folder for the SDK in a host), FSIM_LOGS_DIR overriding.
/// Created on first use. Nothing is written to the user's profile.
std::filesystem::path logsDir();

/// The local map set every reader of map tiles uses - the viewer, the
/// physics' terrain, the vision cameras - and tile_prefetch fills: `maps/`
/// beside the program (<module>/../maps, then <exe>/../maps: a package's own,
/// or the build tree's, which is a junction to the source tree's assets/maps),
/// else the source tree's assets/maps (the SDK loaded by a host from a staged
/// copy), FSIM_MAPS_DIR overriding all of them. Nothing is ever written
/// outside it.
std::filesystem::path mapsDir();

} // namespace fsim::platform
