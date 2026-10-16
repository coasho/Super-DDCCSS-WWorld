"""Executable negative fixtures for the native architecture acceptance gate."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


CHECK = Path(__file__).with_name("check.py")
SCRATCH = Path(os.environ.get("TMP", "D:/ENV/tmp" if os.name == "nt" else tempfile.gettempdir()))


class FixtureRepository(unittest.TestCase):
    """A small literal source tree and its policy; the test classes below plant violations in it."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="fsim-architecture-", dir=SCRATCH)
        self.addCleanup(self.temp.cleanup)
        self.repo = Path(self.temp.name)
        # Literal fixture graph: core -> platform; sim -> core; session -> sim/ms; ipc -> sim (a listed legacy
        # exception); env -> session; sdk (final assembly) -> env/session; world -> render/core/flow/ipc;
        # viewer (application) -> world/ipc.
        targets = {
            "platform": ("fsim_platform", [], True, "foundation", "src/foundation/platform"),
            "core": ("fsim_core", ["platform"], True, "foundation", "src/foundation/core"),
            "sim": ("fsim_sim", ["core"], True, "simulation", "src/simulation/flight"),
            "ms": ("fsim_ms", ["core"], True, "simulation", "src/simulation/mission_systems"),
            "ipc": ("fsim_ipc", ["sim"], True, "foundation", "src/foundation/transport"),
            "session": ("fsim_session", ["sim", "ms"], True, "simulation", "src/simulation/runtime"),
            "env": ("fsim_env", ["session"], True, "training", "src/training/native/vecenv"),
            "sdk": ("fsim", ["env", "session"], True, "adapters", "src/adapters/sdk/cpp"),
            "render": ("fsim_render", ["core"], False, "visualization", "src/visualization/render"),
            "world": ("fsim_world", ["render", "core", "flow", "ipc"], False, "visualization", "src/visualization/scene"),
            "flow": ("fsim_flow", ["core"], False, "visualization", "src/visualization/vfx/flow"),
            "viewer": ("flightsim-viewer", ["world", "ipc"], False, "applications", "src/applications/viewer"),
        }
        kinds = {"contracts": "contract", "foundation": "implementation", "simulation": "implementation",
                 "training": "implementation", "visualization": "implementation", "adapters": "assembly",
                 "applications": "application"}
        may = {"contracts": [], "foundation": ["contracts"], "simulation": ["foundation", "contracts"],
               "training": ["simulation", "foundation", "contracts"], "visualization": ["foundation", "contracts"],
               "adapters": list(kinds), "applications": list(kinds)}
        self.policy = {
            "schema": "fsim.architecture.source-policy.v2",
            "domains": {name: {"root": "src/" + name, "kind": kinds[name], "may_depend_on": may[name]} for name in kinds},
            "modules": {},
            "planned_modules": {
                "simulation_api": {"domain": "simulation", "kind": "implementation", "destinations": ["src/simulation/api"], "phase": "ARCH-04"},
                "mission": {"domain": "training", "kind": "implementation", "destinations": ["src/training/native/mission"], "phase": "ARCH-05"},
            },
            "exceptions": [{"from": "ipc", "to": "sim", "rule": "domain", "until": "ARCH-02", "reason": "fixture legacy edge"}],
            "public_headers": {},
            "public_exceptions": [],
            "external_targets": ["vsg::vsg", "Vulkan::Vulkan"],
            "python": {"package_root": "python/fsim", "package": "fsim", "extensions": ["fsim._native"],
                       "private_packages": {"fsim._contracts": [], "fsim._simulation": ["fsim._contracts", "fsim._native"],
                                            "fsim._training": ["fsim._contracts", "fsim._native"]},
                       "optional_modules": ["fsim.gym"], "optional_packages": ["gymnasium", "torch"],
                       "dynamic_imports": [], "exceptions": []},
        }
        for name, (target, dependencies, headless, domain, destination) in targets.items():
            module = {
                "target": target,
                "aliases": [] if name == "viewer" else ["fsim::" + name],
                "domain": domain,
                "kind": kinds[domain],
                "directories": ["src/" + name] if name not in {"flow", "viewer"} else [],
                "destinations": [destination],
                "files": [],
                "allowed_dependencies": dependencies,
                "headless": headless,
            }
            if name == "flow":
                module["files"] = ["src/world/FlowSolver.h", "src/world/FlowSolver.cpp"]
                cmake = "src/world/CMakeLists.txt"
                source = "FlowSolver.cpp"
                self.write("src/world/FlowSolver.h", "#pragma once\n")
                self.write("src/world/FlowSolver.cpp", '#include "core/Header.h"\n')
            elif name == "viewer":
                module["files"] = ["src/app/viewer_main.cpp"]
                cmake = "src/app/CMakeLists.txt"
                source = "viewer_main.cpp"
                self.write("src/app/viewer_main.cpp", '#include "world/Header.h"\n')
            else:
                cmake = f"src/{name}/CMakeLists.txt"
                source = "Header.h"
                self.write(f"src/{name}/Header.h", "#pragma once\n")
            self.policy["modules"][name] = module
            if name == "viewer":
                self.append(cmake, f"add_executable({target} {source})\n")
            else:
                self.append(cmake, f"add_library({target} STATIC {source})\n"
                            f"add_library(fsim::{name} ALIAS {target})\n")
            if dependencies:
                self.append(cmake, f"target_link_libraries({target} PUBLIC "
                            + " ".join("fsim::" + dependency for dependency in dependencies) + ")\n")
        self.write("include/fsim/Public.h", "#pragma once\n#include <cstdint>\n")
        self.write("python/fsim/__init__.py", "from . import _native\nfrom .world import World\n")
        self.write("python/fsim/world.py", "import math\n\n\nclass World:\n    pass\n")

    # a public header a fixture writes is classified unless the fixture classifies it itself
    PUBLIC_DEFAULTS = {"World.h": {"domain": "simulation", "kind": "facade"}}

    def write(self, path, text):
        if path.startswith("include/fsim/") and path not in self.policy["public_headers"]:
            self.policy["public_headers"][path] = dict(self.PUBLIC_DEFAULTS.get(path.rsplit("/", 1)[1],
                                                                              {"domain": "contracts", "kind": "value"}))
        destination = self.repo / path
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(text, encoding="utf-8")

    def append(self, path, text):
        destination = self.repo / path
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("a", encoding="utf-8") as stream:
            stream.write(text)

    def run_gate(self):
        self.write("policy.json", json.dumps(self.policy))
        result = subprocess.run([sys.executable, "-B", str(CHECK), "--repo", str(self.repo),
                                 "--policy", str(self.repo / "policy.json")],
                                capture_output=True, text=True, check=False)
        return result.returncode, result.stdout + result.stderr

    def rejects(self, code, detail):
        status, output = self.run_gate()
        self.assertEqual(status, 1, output)
        self.assertIn(code, output)
        self.assertIn(detail, output)


class ArchitectureFixtures(FixtureRepository):
    def test_valid_graph_retains_transitive_platform_includes(self):
        self.append("src/ms/Header.h", '#include "platform/Header.h"\n')
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_undeclared_ms_sim_include_is_rejected(self):
        self.append("src/ms/Header.h", '#include "sim/Header.h"\n')
        self.rejects("undeclared-include", "ms -> sim")

    def test_target_include_search_path_cannot_hide_undeclared_ms_sim_import(self):
        self.write("src/sim/OnlySim.h", "#pragma once\n")
        self.append("src/ms/CMakeLists.txt", "target_include_directories(fsim_ms PRIVATE ../sim)\n")
        self.append("src/ms/Header.h", '#include "OnlySim.h"\n')
        self.rejects("undeclared-include", "ms -> sim: OnlySim.h")

    def test_build_interface_search_path_cannot_hide_undeclared_import(self):
        self.write("src/sim/OnlySim.h", "#pragma once\n")
        self.append("src/ms/CMakeLists.txt", "target_include_directories(fsim_ms PRIVATE "
                    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/sim>)\n")
        self.append("src/ms/Header.h", '#include "OnlySim.h"\n')
        self.rejects("undeclared-include", "ms -> sim: OnlySim.h")

    def test_ambiguous_private_search_paths_do_not_hide_an_upward_import(self):
        # Angle imports do not prefer the source's own Header.h. The gate must
        # retain the other owner's candidate instead of silently choosing ours.
        self.append("src/ms/CMakeLists.txt", "target_include_directories(fsim_ms PRIVATE ../sim)\n")
        self.append("src/ms/Header.h", "#include <Header.h>\n")
        self.rejects("undeclared-include", "ms -> sim: Header.h")

    def test_declared_dependency_search_paths_and_repo_build_interfaces_are_accepted(self):
        self.write("src/core/OnlyCore.h", "#pragma once\n")
        self.append("src/core/CMakeLists.txt", "target_include_directories(fsim_core PUBLIC "
                    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> "
                    "$<INSTALL_INTERFACE:include>)\n")
        self.append("src/ms/CMakeLists.txt", "target_include_directories(fsim_ms PRIVATE "
                    "${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include)\n")
        self.append("src/ms/Header.h", '#include "OnlyCore.h"\n')
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_unresolved_include_directory_fails_closed(self):
        self.append("src/ms/CMakeLists.txt", "target_include_directories(fsim_ms PRIVATE ${HIDDEN_INCLUDE_PATH})\n")
        self.rejects("unsupported-cmake", "HIDDEN_INCLUDE_PATH")

    def test_directory_include_mutation_fails_closed(self):
        self.append("src/ms/CMakeLists.txt", "include_directories(../sim)\n")
        self.rejects("unsupported-cmake", "include_directories")

    def test_target_include_property_mutation_fails_closed(self):
        self.append("src/ms/CMakeLists.txt", "set_target_properties(fsim_ms PROPERTIES INCLUDE_DIRECTORIES ../sim)\n")
        self.rejects("unsupported-cmake", "INCLUDE_DIRECTORIES")

    def test_private_source_include_property_mutation_fails_closed(self):
        self.append("src/ms/CMakeLists.txt", "set_source_files_properties(Header.h PROPERTIES INCLUDE_DIRECTORIES ../sim)\n")
        self.rejects("unsupported-cmake", "source include directory")

    def test_vendor_search_directory_does_not_classify_sdk_or_std_headers_as_vendor(self):
        self.write("third_party/stb/stb_image.h", "#pragma once\n")
        self.append("src/ms/CMakeLists.txt", "target_include_directories(fsim_ms SYSTEM PRIVATE "
                    "${CMAKE_SOURCE_DIR}/third_party/stb)\n")
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_core_session_include_and_declared_upward_link_are_rejected(self):
        self.append("src/core/Header.h", '#include "session/Header.h"\n')
        self.append("src/core/CMakeLists.txt", "target_link_libraries(fsim_core PRIVATE fsim::session)\n")
        self.rejects("forbidden-link", "core -> session")

    def test_undeclared_core_session_include_is_rejected(self):
        self.append("src/core/Header.h", '#include "session/Header.h"\n')
        self.rejects("undeclared-include", "core -> session")

    def test_native_session_cannot_link_upward_to_env(self):
        self.append("src/session/CMakeLists.txt", "target_link_libraries(fsim_session PRIVATE fsim::env)\n")
        self.rejects("forbidden-link", "session -> env")

    def test_native_session_cannot_import_env_without_a_link(self):
        self.append("src/session/Header.h", '#include "env/Header.h"\n')
        self.rejects("undeclared-include", "session -> env")

    def test_env_cannot_link_to_render_through_a_native_target_name(self):
        self.append("src/env/CMakeLists.txt", "target_link_libraries(fsim_env PRIVATE fsim_render)\n")
        self.rejects("forbidden-link", "env -> render")

    def test_env_cannot_import_render_without_a_link(self):
        self.append("src/env/Header.h", '#include "render/Header.h"\n')
        self.rejects("undeclared-include", "env -> render")

    def test_viewer_cannot_link_to_env(self):
        self.append("src/app/CMakeLists.txt", "target_link_libraries(flightsim-viewer PRIVATE fsim::env)\n")
        self.rejects("forbidden-link", "viewer -> env")

    def test_viewer_cannot_import_env_without_a_link(self):
        self.append("src/app/viewer_main.cpp", '#include "env/Header.h"\n')
        self.rejects("undeclared-include", "viewer -> env")

    def test_rendered_world_ipc_dependency_is_required(self):
        cmake = self.repo / "src/world/CMakeLists.txt"
        cmake.write_text(cmake.read_text().replace(" fsim::ipc", ""))
        self.append("src/world/Header.h", '#include "ipc/Header.h"\n')
        self.rejects("undeclared-include", "world -> ipc")

    def test_public_header_cannot_import_private_source(self):
        self.append("include/fsim/Public.h", '#include "core/Header.h"\n')
        self.rejects("public-private", "core/Header.h")

    def test_public_relative_include_cannot_import_private_source(self):
        self.append("include/fsim/Public.h", '#include "../../src/core/Header.h"\n')
        self.rejects("public-private", "../../src/core/Header.h")

    def test_public_header_cannot_forward_declare_native_world(self):
        self.append("include/fsim/Public.h", "namespace fsim::session { class World; }\n")
        self.rejects("public-private-type", "session")

    def test_public_header_cannot_expose_a_qualified_native_type(self):
        self.append("include/fsim/Public.h", "fsim::session::World& native_world();\n")
        self.rejects("public-private-type", "session")

    def test_public_header_cannot_alias_a_native_namespace(self):
        self.append("include/fsim/Public.h", "namespace native = fsim::session;\n"
                    "native::World& native_world();\n")
        self.rejects("public-private-type", "session")

    def test_public_header_cannot_alias_an_unqualified_native_namespace(self):
        self.append("include/fsim/Public.h", "namespace fsim { namespace native = session; }\n")
        self.rejects("public-private-type", "session")

    def test_public_header_cannot_alias_an_episode_type(self):
        self.append("include/fsim/Public.h", "using Episode = fsim::env::MissionEpisode;\n")
        self.rejects("public-private-type", "env")

    def test_public_header_cannot_forward_declare_native_ipc(self):
        self.append("include/fsim/Public.h", "namespace fsim { namespace ipc { struct VehicleSample; } }\n")
        self.rejects("public-private-type", "ipc")

    def trusted_world_header(self):
        return """namespace fsim {
namespace session { class World; }
class FSIM_API World {
public:
    session::World& impl() noexcept { return *impl_; }
    static World* borrowed(session::World& w) { return new World(Borrow{}, w); }
private:
    struct Borrow {};
    World(Borrow, session::World& borrowed);
    std::shared_ptr<session::World> impl_;
};
}
"""

    def test_existing_trusted_world_seams_are_retained(self):
        self.write("include/fsim/World.h", self.trusted_world_header())
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_world_header_has_no_file_wide_native_type_exemption(self):
        self.write("include/fsim/World.h", self.trusted_world_header() +
                   "class MissionEndpoint { fsim::session::World& native_world(); };\n")
        self.rejects("public-private-type", "session")

    def test_trusted_world_method_copied_into_actor_is_rejected(self):
        self.write("include/fsim/World.h", self.trusted_world_header() +
                   "class MissionEndpoint { session::World& impl() noexcept { return *impl_; } };\n")
        self.rejects("public-private-type", "session")

    def test_trusted_world_method_copied_into_nested_actor_is_rejected(self):
        original = self.trusted_world_header()
        self.write("include/fsim/World.h", original.replace("public:\n", "public:\n"
                   "    class MissionEndpoint { session::World& impl() noexcept { return *impl_; } };\n"))
        self.rejects("public-private-type", "session")

    def test_new_native_world_method_is_not_a_legacy_seam(self):
        self.write("include/fsim/World.h", self.trusted_world_header().replace("public:\n", "public:\n"
                   "    session::World& new_escape();\n"))
        self.rejects("public-private-type", "session")

    def test_native_namespace_names_in_literals_and_comments_are_not_types(self):
        self.append("include/fsim/Public.h", 'const char* text = "fsim::session::World";\n'
                    '// namespace fsim::env { class MissionEpisode; }\n')
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_public_header_cannot_import_third_party_implementation(self):
        self.write("third_party/stb/stb_image.h", "#pragma once\n")
        self.append("include/fsim/Public.h", "#include <stb_image.h>\n")
        self.rejects("public-third-party", "stb_image.h")

    def test_headless_vsg_include_is_rejected(self):
        self.append("src/core/Header.h", "#include <vsg/all.h>\n")
        self.rejects("headless-gpu", "vsg/all.h")

    def test_headless_vulkan_link_is_rejected(self):
        self.append("src/ms/CMakeLists.txt", "target_link_libraries(fsim_ms PRIVATE Vulkan::Vulkan)\n")
        self.rejects("headless-gpu", "Vulkan::Vulkan")

    def test_declared_dependency_cycle_is_rejected(self):
        self.policy["modules"]["platform"]["allowed_dependencies"] = ["core"]
        self.append("src/platform/CMakeLists.txt", "target_link_libraries(fsim_platform PUBLIC fsim::core)\n")
        self.rejects("dependency-cycle", "core")

    def test_unknown_native_module_is_rejected(self):
        self.write("src/new_module/CMakeLists.txt", "add_library(fsim_new STATIC Header.h)\n")
        self.write("src/new_module/Header.h", "#pragma once\n")
        self.rejects("unknown-target", "fsim_new")

    def test_unknown_source_directory_is_rejected(self):
        self.write("src/new_module/Header.h", "#pragma once\n")
        self.rejects("unknown-owner", "src/new_module/Header.h")

    def test_flow_sources_do_not_inherit_world_dependencies(self):
        self.append("src/world/FlowSolver.cpp", '#include "render/Header.h"\n')
        self.rejects("undeclared-include", "flow -> render")

    def test_target_source_ownership_mismatch_is_rejected(self):
        self.append("src/core/CMakeLists.txt", "target_sources(fsim_core PRIVATE ../session/Header.h)\n")
        self.rejects("source-owner", "session owns")

    def test_public_sdk_header_wins_over_vendor_basename_collision(self):
        self.write("include/fsim/Export.h", "#pragma once\n")
        self.write("third_party/engine/Export.h", "#pragma once\n")
        self.append("include/fsim/Public.h", '#include "fsim/Export.h"\n')
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_comments_do_not_create_includes_or_cmake_targets(self):
        self.append("src/core/Header.h", '// #include "session/Header.h"\n'
                    '/*\n#include <vsg/all.h>\n*/\n'
                    'const char* example = R"example(\n#include "session/Header.h"\n)example";\n')
        self.append("src/core/CMakeLists.txt", "# add_library(fsim_unknown STATIC Fake.cpp)\n"
                    "#[=[\nadd_library(fsim_other STATIC Fake.cpp)\n]=]\n")
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_all_conditional_branches_are_scanned_without_configuration(self):
        self.append("src/core/CMakeLists.txt", "if(FSIM_WITH_RENDER)\n"
                    "  target_link_libraries(fsim_core PRIVATE vsg::vsg)\n"
                    "else()\n  target_link_libraries(fsim_core PRIVATE fsim::platform)\nendif()\n")
        self.rejects("headless-gpu", "vsg::vsg")

    def test_whole_archive_generator_expression_preserves_internal_graph(self):
        self.append("src/session/CMakeLists.txt", 'target_link_libraries(fsim_session PRIVATE '
                    '"$<LINK_LIBRARY:WHOLE_ARCHIVE,fsim::ms,fsim::sim>")\n')
        self.append("src/session/Header.h", '#include "platform/Header.h"\n')
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_unresolved_link_variable_fails_closed(self):
        self.append("src/core/CMakeLists.txt", "target_link_libraries(fsim_core PRIVATE ${HIDDEN_DEPENDENCIES})\n")
        self.rejects("unsupported-cmake", "HIDDEN_DEPENDENCIES")

    def test_link_property_mutation_fails_closed(self):
        self.append("src/core/CMakeLists.txt", "set_target_properties(fsim_core PROPERTIES LINK_LIBRARIES fsim::session)\n")
        self.rejects("unsupported-cmake", "LINK_LIBRARIES")

    def test_directory_link_mutation_fails_closed(self):
        self.append("src/core/CMakeLists.txt", "link_libraries(fsim::session)\n")
        self.rejects("unsupported-cmake", "link_libraries")

    def test_macro_include_fails_closed(self):
        self.append("src/core/Header.h", "#define DEPENDENCY <vsg/all.h>\n#include DEPENDENCY\n")
        self.rejects("unsupported-include", "DEPENDENCY")

    def test_private_import_from_public_uses_real_line_number(self):
        self.append("include/fsim/Public.h", '\n\n#include "session/Header.h"\n')
        self.rejects("public-private", "include/fsim/Public.h:5:")


class DomainFixtures(FixtureRepository):
    """ARCH-01: domains, kinds, future ownership and Python imports. Each violation is planted with its allowance
    already granted, so only the architecture rule under test can reject it."""

    def link(self, consumer, provider):
        self.policy["modules"][consumer]["allowed_dependencies"].append(provider)
        module = self.policy["modules"][provider]
        name = (module["aliases"] or [module["target"]])[0]
        cmake = "src/app/CMakeLists.txt" if consumer == "viewer" else f"src/{consumer}/CMakeLists.txt"
        self.append(cmake, f"target_link_libraries({self.policy['modules'][consumer]['target']} PRIVATE {name})\n")

    def input_error(self, detail):
        status, output = self.run_gate()
        self.assertEqual(status, 1, output)
        self.assertIn("architecture input error", output)
        self.assertIn(detail, output)

    # the current accepted shape, and the future nested one
    def test_the_fixture_tree_with_its_listed_legacy_exception_is_accepted(self):
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)
        self.assertIn("legacy exceptions in use: ipc -> sim (domain)", output)

    def test_a_source_moved_to_its_nested_destination_keeps_its_owner(self):
        self.write("src/training/native/vecenv/VecEnv.cpp", '#include "session/Header.h"\n')
        self.append("src/env/CMakeLists.txt", "target_sources(fsim_env PRIVATE ../training/native/vecenv/VecEnv.cpp)\n")
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_a_moved_module_keeps_no_legacy_directory(self):
        # ARCH-03's shape: a module moved whole to its destination - the legacy directory it still lists is stale
        (self.repo / "src/core/Header.h").unlink()
        (self.repo / "src/core/CMakeLists.txt").unlink()
        self.write("src/foundation/core/Header.h", "#pragma once\n")
        self.write("src/foundation/core/CMakeLists.txt", "add_library(fsim_core STATIC Header.h)\nadd_library(fsim::core ALIAS fsim_core)\n"
                   "target_link_libraries(fsim_core PUBLIC fsim::platform)\n")
        self.rejects("stale-legacy-directory", "core: src/core holds no source; the module has moved")
        self.policy["modules"]["core"]["directories"] = []
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_a_file_in_a_domain_root_outside_every_destination_has_no_owner(self):
        self.write("src/training/native/Stray.cpp", "int stray;\n")
        self.rejects("unknown-owner", "inside the training domain root")

    def test_a_file_in_a_planned_destination_needs_its_registered_target(self):
        self.write("src/training/native/mission/MissionGateway.cpp", "int gateway;\n")
        self.rejects("planned-owner", "reserved for planned module mission (ARCH-05)")

    # forbidden reverse edges
    def test_simulation_cannot_depend_on_training(self):
        self.link("session", "env")
        self.rejects("forbidden-domain", "session (simulation) -> env (training) by link")

    def test_simulation_cannot_depend_on_visualization(self):
        self.link("ms", "render")
        self.rejects("forbidden-domain", "ms (simulation) -> render (visualization) by link")

    def test_training_cannot_depend_on_rendering(self):
        self.link("env", "render")
        self.rejects("forbidden-domain", "env (training) -> render (visualization) by link")

    def test_training_cannot_depend_on_the_final_sdk_assembly(self):
        # the SDK extraction cycle: training -> final SDK -> training
        self.link("env", "sdk")
        status, output = self.run_gate()
        self.assertEqual(status, 1, output)
        self.assertIn("forbidden-kind: CMake graph: env (training) -> sdk (adapters) by link: a domain implementation", output)
        self.assertIn("dependency-cycle", output)
        self.assertIn("env -> sdk -> env", output)

    def test_visualization_core_cannot_depend_on_training(self):
        self.link("world", "env")
        self.rejects("forbidden-domain", "world (visualization) -> env (training) by link")

    def test_visualization_core_cannot_depend_on_the_mutable_world_runtime(self):
        self.link("world", "session")
        self.rejects("forbidden-domain", "world (visualization) -> session (simulation) by link")

    def test_a_transitive_path_does_not_launder_a_reverse_include(self):
        # world reaches sim through ipc's listed exception; the exception is ipc's, not world's
        self.append("src/world/Header.h", '#include "sim/Header.h"\n')
        self.rejects("forbidden-domain", "world (visualization) -> sim (simulation) by include")

    def test_foundation_cannot_depend_on_a_domain(self):
        self.link("core", "sim")
        self.rejects("forbidden-domain", "core (foundation) -> sim (simulation) by link")

    def test_a_contract_cannot_depend_on_an_implementation(self):
        self.policy["modules"]["contracts"] = {
            "target": "fsim_contracts", "aliases": ["fsim::contracts"], "domain": "contracts", "kind": "contract",
            "directories": [], "destinations": ["src/contracts"], "files": [], "headless": True, "allowed_dependencies": []}
        self.write("src/contracts/Values.h", "#pragma once\n")
        self.write("src/contracts/CMakeLists.txt", "add_library(fsim_contracts STATIC Values.h)\n"
                   "add_library(fsim::contracts ALIAS fsim_contracts)\n")
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)  # (the contract alone, in its destination root)
        self.link("contracts", "core")
        self.rejects("forbidden-kind", "contracts (contracts) -> core (foundation) by link: a contract depends on an implementation")

    def test_nothing_depends_on_an_application(self):
        self.link("world", "viewer")
        self.rejects("forbidden-kind", "world (visualization) -> viewer (applications) by link: nothing depends on an application")

    # graphics leakage
    def test_training_cannot_include_graphics_headers(self):
        self.append("src/env/Header.h", "#include <vsg/all.h>\n")
        self.rejects("headless-gpu", "vsg/all.h")

    def test_a_graphics_module_outside_visualization_is_an_input_error(self):
        self.policy["modules"]["ms"]["headless"] = False
        self.input_error("graphics (headless false) in the simulation domain")

    # duplicate ownership
    def test_two_modules_claiming_one_directory_is_an_input_error(self):
        self.policy["modules"]["ms"]["destinations"].append("src/simulation/flight")
        self.input_error("duplicate-owner: src/simulation/flight is claimed by sim and ms")

    def test_a_planned_module_cannot_claim_an_active_destination(self):
        self.policy["planned_modules"]["mission"]["destinations"].append("src/training/native/vecenv")
        self.input_error("duplicate-owner: src/training/native/vecenv")

    def test_one_source_in_two_target_source_lists_is_rejected(self):
        self.append("src/ms/CMakeLists.txt", "target_sources(fsim_ms PRIVATE ../sim/Header.h)\n")
        status, output = self.run_gate()
        self.assertEqual(status, 1, output)
        self.assertIn("duplicate-source: src/sim/Header.h: listed by targets fsim_ms, fsim_sim", output)

    # policy versus the actual graph, and exceptions that expire
    def test_an_allowance_without_a_link_is_rejected(self):
        self.policy["modules"]["env"]["allowed_dependencies"].append("ms")
        self.rejects("unused-allowance", "env -> ms is allowed but not linked")

    def test_an_exception_for_an_edge_that_is_gone_is_stale(self):
        self.policy["exceptions"].append({"from": "render", "to": "sim", "rule": "domain", "until": "ARCH-06", "reason": "gone"})
        self.rejects("stale-exception", "render -> sim (domain, until ARCH-06): the edge no longer exists")

    def test_an_exception_for_a_legal_edge_is_stale(self):
        self.policy["exceptions"].append({"from": "session", "to": "sim", "rule": "domain", "until": "ARCH-04", "reason": "legal"})
        self.rejects("stale-exception", "session -> sim (domain, until ARCH-04): the edge breaks no rule")

    def test_an_exception_does_not_cover_another_rule(self):
        self.link("env", "sdk")
        self.policy["exceptions"].append({"from": "env", "to": "sdk", "rule": "domain", "until": "ARCH-05", "reason": "wrong rule"})
        self.rejects("forbidden-kind", "env (training) -> sdk (adapters)")

    # unsupported declarations fail explicitly
    def test_an_unknown_policy_key_is_an_input_error(self):
        self.policy["modules"]["ms"]["owner_hint"] = "x"
        self.input_error("module ms: keys must be")

    def test_a_destination_outside_its_domain_root_is_an_input_error(self):
        self.policy["modules"]["env"]["destinations"] = ["src/simulation/vecenv"]
        self.input_error("destination src/simulation/vecenv is outside its domain root src/training")

    def test_a_legacy_directory_inside_a_domain_root_is_an_input_error(self):
        self.policy["modules"]["env"]["directories"].append("src/training/old")
        self.input_error("legacy directory src/training/old lies in a domain root")

    def test_an_exception_without_a_later_phase_is_an_input_error(self):
        self.policy["exceptions"][0]["until"] = "someday"
        self.input_error("unknown module or phase someday")

    def test_a_kind_foreign_to_its_domain_is_an_input_error(self):
        self.policy["modules"]["env"]["kind"] = "assembly"
        self.input_error("kind assembly does not belong to domain training")

    # Python imports
    def test_a_new_deferred_facade_backedge_is_a_cycle(self):
        self.write("python/fsim/world.py", "class World:\n    def agent(self):\n        from fsim import World as W\n        return W\n")
        self.rejects("python-cycle", "fsim.world -> fsim (deferred)")

    def test_a_private_implementation_cannot_import_the_facade_even_locally(self):
        self.write("python/fsim/_training/__init__.py", "")
        self.write("python/fsim/_training/batch.py", "def error():\n    import fsim\n    return fsim\n")
        self.rejects("python-facade-backedge", "fsim._training.batch -> fsim (deferred)")

    def test_a_private_implementation_cannot_import_another_domain(self):
        self.write("python/fsim/_simulation/__init__.py", "")
        self.write("python/fsim/_training/__init__.py", "")
        self.write("python/fsim/_training/batch.py", "from fsim._simulation import values\n")
        self.write("python/fsim/_simulation/values.py", "")
        self.rejects("python-forbidden-import", "fsim._training.batch -> fsim._simulation.values (eager)")

    def test_a_listed_private_dependency_is_accepted(self):
        self.write("python/fsim/_contracts/__init__.py", "")
        self.write("python/fsim/_contracts/errors.py", "")
        self.write("python/fsim/_training/__init__.py", "")
        self.write("python/fsim/_training/batch.py", "from fsim._contracts import errors\nfrom fsim import _native\n")
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_an_undeclared_subpackage_has_no_owner(self):
        self.write("python/fsim/helpers/__init__.py", "")
        self.rejects("python-unknown-owner", "subpackage fsim.helpers is not a declared private package")

    def test_importing_the_package_cannot_load_an_optional_module(self):
        self.write("python/fsim/gym.py", "import gymnasium\n")
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)  # (optional on its own)
        self.append("python/fsim/__init__.py", "from . import gym\n")
        self.rejects("python-optional-eager", "importing fsim eagerly imports optional fsim.gym")

    def test_importing_the_package_cannot_load_an_optional_external_package(self):
        self.write("python/fsim/world.py", "def threads():\n    import torch\n    return torch\n")
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)  # (deferred: loaded only when used)
        self.write("python/fsim/world.py", "import torch\n")
        self.rejects("python-optional-eager", "eagerly imports optional package torch (through fsim.world)")

    def test_an_unregistered_dynamic_import_is_rejected(self):
        self.write("python/fsim/world.py", "import importlib\n\n\ndef load(name):\n    return importlib.import_module(name)\n")
        self.rejects("python-dynamic-import", "unregistered dynamic import in load")
        self.policy["python"]["dynamic_imports"].append({"module": "fsim.world", "function": "load", "reason": "fixture"})
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_a_legacy_python_backedge_is_exact_deferred_and_expires(self):
        self.write("python/fsim/world.py", "def agent():\n    import fsim\n    return fsim\n")
        self.policy["python"]["exceptions"].append({"from": "fsim.world", "to": "fsim", "until": "ARCH-07", "reason": "fixture"})
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)
        self.write("python/fsim/world.py", "import fsim\n")
        self.rejects("python-exception", "legacy backedge fsim.world -> fsim is now eager")
        self.write("python/fsim/world.py", "")
        self.rejects("python-exception", "stale legacy backedge fsim.world -> fsim: no such import")

    # review finding 1: an exact file claim never overrides a reserved or foreign destination
    def test_an_exact_file_claim_cannot_take_a_reserved_destination(self):
        path = "src/training/native/mission/Takeover.cpp"
        self.write(path, "int takeover;\n")
        self.append("src/session/CMakeLists.txt", "target_sources(fsim_session PRIVATE ../training/native/mission/Takeover.cpp)\n")
        self.rejects("planned-owner", "reserved for planned module mission (ARCH-05)")
        self.policy["modules"]["session"]["files"].append(path)
        self.input_error(f"file-claim: session (simulation) claims {path}, reserved for planned module mission (ARCH-05)")

    def test_an_exact_file_claim_cannot_take_another_modules_destination(self):
        self.policy["modules"]["session"]["files"].append("src/training/native/vecenv/Take.cpp")
        self.input_error("file-claim: session (simulation) claims src/training/native/vecenv/Take.cpp in the destination of env (training)")

    def test_an_exact_file_claim_cannot_take_another_domains_legacy_directory(self):
        self.policy["modules"]["session"]["files"].append("src/env/Take.cpp")
        self.input_error("file-claim: session (simulation) claims src/env/Take.cpp in the legacy directory of env (training)")

    def test_an_exact_file_claim_inside_a_domain_root_needs_its_own_destination(self):
        self.policy["modules"]["session"]["files"].append("src/training/Loose.cpp")
        self.input_error("claims src/training/Loose.cpp inside the training domain root, outside its own destinations")

    def test_exact_file_claims_in_a_same_domain_legacy_directory_or_own_destination_are_kept(self):
        # (Flow's files in src/world are the fixture's legacy shared directory; a claim in Flow's own destination too)
        self.write("src/visualization/vfx/flow/FlowFires.cpp", '#include "core/Header.h"\n')
        self.policy["modules"]["flow"]["files"].append("src/visualization/vfx/flow/FlowFires.cpp")
        self.append("src/world/CMakeLists.txt", "target_sources(fsim_flow PRIVATE ../visualization/vfx/flow/FlowFires.cpp)\n")
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    # review finding 2: imported import functions and their aliases are dynamic-import sites
    def test_an_imported_import_function_is_a_dynamic_import_site(self):
        for binding, name in (("from importlib import import_module", "import_module"),
                              ("from importlib import import_module as load_module", "load_module"),
                              ("from builtins import __import__ as raw_import", "raw_import")):
            with self.subTest(binding=binding):
                self.policy["python"]["dynamic_imports"] = []
                self.write("python/fsim/world.py", f"{binding}\n\n\ndef load(module):\n    return {name}(module)\n")
                self.rejects("python-dynamic-import", "unregistered dynamic import in load")
                self.policy["python"]["dynamic_imports"] = [{"module": "fsim.world", "function": "load", "reason": "fixture"}]
                status, output = self.run_gate()
                self.assertEqual(status, 0, output)

    def test_an_import_function_used_other_than_as_a_direct_call_is_unsupported(self):
        self.policy["python"]["dynamic_imports"] = [{"module": "fsim.world", "function": "load", "reason": "fixture"}]
        for text in ("import importlib\n\n\ndef load(module):\n    loader = importlib.import_module\n    return loader(module)\n",
                     "from importlib import import_module as load_module\n\n\ndef load(module):\n    return map(load_module, [module])\n",
                     "import importlib\n\n\ndef load(module):\n    return getattr(importlib, 'import_module')(module)\n"):
            with self.subTest(text=text):
                self.write("python/fsim/world.py", text)
                self.rejects("python-dynamic-import", "unsupported dynamic-import form in load")

    # ARCH-02: transport below the domains, the contracts it depends on, and public headers by owner
    def test_transport_cannot_depend_on_simulation_once_its_exception_is_gone(self):
        self.policy["exceptions"] = []
        self.rejects("forbidden-domain", "ipc (foundation) -> sim (simulation) by link")

    def contracts_below_transport(self):
        """The fixture as ARCH-02 leaves the tree: ipc on the contracts, not on sim; no exception."""
        self.policy["modules"]["contracts"] = {
            "target": "fsim_contracts", "aliases": ["fsim::contracts"], "domain": "contracts", "kind": "contract",
            "directories": [], "destinations": ["src/contracts"], "files": [], "headless": True, "allowed_dependencies": []}
        self.write("src/contracts/world_stream/WorldLayout.h", "#pragma once\n#include <cstdint>\n")
        self.write("src/contracts/CMakeLists.txt", "add_library(fsim_contracts INTERFACE world_stream/WorldLayout.h)\n"
                   "add_library(fsim::contracts ALIAS fsim_contracts)\n")
        self.policy["exceptions"] = []
        self.policy["modules"]["ipc"]["allowed_dependencies"] = ["contracts"]
        self.write("src/ipc/CMakeLists.txt", "add_library(fsim_ipc STATIC Header.h)\nadd_library(fsim::ipc ALIAS fsim_ipc)\n"
                   "target_link_libraries(fsim_ipc PUBLIC fsim::contracts)\n")
        self.append("src/ipc/Header.h", '#include "contracts/world_stream/WorldLayout.h"\n')

    def test_transport_on_the_contracts_is_accepted_without_an_exception(self):
        self.contracts_below_transport()
        self.append("src/world/Header.h", '#include "contracts/world_stream/WorldLayout.h"\n')  # (a reader, through ipc)
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)
        self.assertIn("legacy exceptions in use: none", output)

    def test_transport_on_the_contracts_cannot_reach_simulation_headers(self):
        self.contracts_below_transport()
        self.append("src/ipc/Header.h", '#include "sim/Header.h"\n')
        self.rejects("undeclared-include", "ipc -> sim: sim/Header.h")

    def test_a_contract_header_cannot_include_a_simulation_header(self):
        self.contracts_below_transport()
        self.policy["modules"]["contracts"]["allowed_dependencies"] = ["sim"]
        self.append("src/contracts/CMakeLists.txt", "target_link_libraries(fsim_contracts INTERFACE fsim::sim)\n")
        self.append("src/contracts/world_stream/WorldLayout.h", '#include "sim/Header.h"\n')
        self.rejects("forbidden-kind", "contracts (contracts) -> sim (simulation) by link: a contract depends on an implementation")

    def test_an_unclassified_public_header_is_rejected(self):
        self.write("include/fsim/New.h", "#pragma once\n")
        del self.policy["public_headers"]["include/fsim/New.h"]
        self.rejects("unclassified-public-header", "include/fsim/New.h")

    def test_a_classified_public_header_that_is_gone_is_stale(self):
        self.policy["public_headers"]["include/fsim/Gone.h"] = {"domain": "contracts", "kind": "value"}
        self.rejects("stale-public-header", "include/fsim/Gone.h is classified but does not exist")

    def training_headers(self):
        self.policy["public_headers"]["include/fsim/VecEnv.h"] = {"domain": "training", "kind": "facade"}
        self.write("include/fsim/VecEnv.h", "#pragma once\n")

    def test_simulation_cannot_include_a_training_public_header(self):
        # the A-03 shape: the runtime reading scenario values through the training aggregate
        self.training_headers()
        self.append("src/session/Header.h", '#include "fsim/VecEnv.h"\n')
        self.rejects("forbidden-public", "session (simulation) includes include/fsim/VecEnv.h (training facade): simulation may depend on")

    def test_a_simulation_public_header_cannot_include_a_training_one(self):
        self.training_headers()
        self.write("include/fsim/ScenarioDocument.h", '#pragma once\n#include "fsim/VecEnv.h"\n')
        self.policy["public_headers"]["include/fsim/ScenarioDocument.h"] = {"domain": "simulation", "kind": "interface"}
        self.rejects("forbidden-public", "include/fsim/ScenarioDocument.h (simulation) includes include/fsim/VecEnv.h (training facade)")
        # the split: the document without the batch environment; the training aggregate includes both
        self.write("include/fsim/ScenarioDocument.h", "#pragma once\n")
        self.write("include/fsim/Scenario.h", '#pragma once\n#include "fsim/ScenarioDocument.h"\n#include "fsim/VecEnv.h"\n')
        self.policy["public_headers"]["include/fsim/Scenario.h"] = {"domain": "training", "kind": "facade"}
        self.append("src/session/Header.h", '#include "fsim/ScenarioDocument.h"\n')
        self.append("src/env/Header.h", '#include "fsim/Scenario.h"\n')
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    def test_a_contract_public_header_cannot_include_a_domain_api(self):
        self.write("include/fsim/World.h", "#pragma once\n")
        self.append("include/fsim/Public.h", '#include "fsim/World.h"\n')
        self.rejects("forbidden-public", "include/fsim/Public.h (contracts) includes include/fsim/World.h (simulation facade): a contract includes")

    def test_an_implementation_cannot_include_the_c_abi_but_an_assembly_or_application_can(self):
        self.policy["public_headers"]["include/fsim/fsim_c.h"] = {"domain": "adapters", "kind": "c_abi"}
        self.write("include/fsim/fsim_c.h", "#pragma once\n")
        self.append("src/sdk/Header.h", '#include "fsim/fsim_c.h"\n')
        self.append("src/app/viewer_main.cpp", '#include "fsim/fsim_c.h"\n')
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)
        self.append("src/env/Header.h", '#include "fsim/fsim_c.h"\n')
        self.rejects("forbidden-public", "env (training) includes include/fsim/fsim_c.h (adapters c_abi): the C ABI or an assembly's API")

    def test_a_public_exception_is_exact_and_expires(self):
        self.training_headers()
        self.append("src/session/Header.h", '#include "fsim/VecEnv.h"\n')
        self.policy["public_exceptions"] = [{"from": "session", "to": "include/fsim/VecEnv.h", "until": "ARCH-04", "reason": "fixture"}]
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)
        self.write("src/session/Header.h", "#pragma once\n")
        self.rejects("stale-exception", "public session -> include/fsim/VecEnv.h (until ARCH-04): the include no longer exists")
        self.policy["public_exceptions"] = [{"from": "env", "to": "include/fsim/VecEnv.h", "until": "ARCH-05", "reason": "legal"}]
        self.append("src/env/Header.h", '#include "fsim/VecEnv.h"\n')
        self.rejects("stale-exception", "public env -> include/fsim/VecEnv.h (until ARCH-05): the include breaks no rule")

    # HC-00: a public include file of any suffix is classified and scanned; nothing it includes disappears
    def test_an_unclassified_public_include_fragment_is_rejected_where_it_is_included(self):
        # the review's chain: session -> fsim/Bridge.inc -> fsim/VecEnv.h (training), Bridge.inc unclassified
        self.training_headers()
        self.write("include/fsim/Bridge.inc", '#include "fsim/VecEnv.h"\n')
        del self.policy["public_headers"]["include/fsim/Bridge.inc"]
        self.append("src/session/Header.h", '#include "fsim/Bridge.inc"\n')
        status, output = self.run_gate()
        self.assertEqual(status, 1, output)
        self.assertIn("unclassified-public-header: include/fsim/Bridge.inc", output)
        self.assertIn("unclassified-public-include: src/session/Header.h:2: session includes include/fsim/Bridge.inc", output)

    def test_an_indirect_simulation_to_training_include_through_a_fragment_is_rejected(self):
        self.training_headers()
        self.write("include/fsim/Bridge.inc", '#include "fsim/VecEnv.h"\n')
        self.policy["public_headers"]["include/fsim/Bridge.inc"] = {"domain": "simulation", "kind": "interface"}
        self.append("src/session/Header.h", '#include "fsim/Bridge.inc"\n')
        self.rejects("forbidden-public", "include/fsim/Bridge.inc (simulation) includes include/fsim/VecEnv.h (training facade)")

    def test_an_indirect_contract_to_domain_include_through_a_fragment_is_rejected(self):
        self.write("include/fsim/World.h", "#pragma once\n")
        self.write("include/fsim/Values.ipp", '#include "fsim/World.h"\n')
        self.policy["public_headers"]["include/fsim/Values.ipp"] = {"domain": "contracts", "kind": "value"}
        self.append("include/fsim/Public.h", '#include "fsim/Values.ipp"\n')
        self.rejects("forbidden-public", "include/fsim/Values.ipp (contracts) includes include/fsim/World.h (simulation facade)")

    def test_a_valid_include_fragment_chain_is_accepted(self):
        self.write("include/fsim/World.h", "#pragma once\n")
        self.write("include/fsim/Detail.tpp", '#include "fsim/World.h"\n#include "fsim/Public.h"\n')
        self.policy["public_headers"]["include/fsim/Detail.tpp"] = {"domain": "simulation", "kind": "interface"}
        self.append("src/session/Header.h", '#include "fsim/Detail.tpp"\n')
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)
        self.assertIn("Public headers: 3 classified", output)  # (Public.h, World.h and Detail.tpp)

    def test_a_private_include_of_an_unscanned_file_is_refused(self):
        self.write("src/session/Shader.glsl", '#include "env/Header.h"\n')
        self.append("src/session/Header.h", '#include "session/Shader.glsl"\n')
        self.rejects("unscanned-include", "resolves to src/session/Shader.glsl, whose suffix the gate does not scan")

    def test_a_public_file_that_is_not_text_is_refused(self):
        self.write("include/fsim/Public.h", "#pragma once\n")
        (self.repo / "include/fsim/blob.bin").write_bytes(b"\xff\xfe\x00\x80")
        self.policy["public_headers"]["include/fsim/blob.bin"] = {"domain": "contracts", "kind": "value"}
        self.rejects("unsupported-public-file", "include/fsim/blob.bin: not UTF-8 text")

    def test_public_header_classification_errors_are_input_errors(self):
        self.policy["public_headers"]["include/fsim/Public.h"] = {"domain": "contracts", "kind": "facade"}
        self.input_error("a contracts header is a value or an interface, not a facade")
        self.policy["public_headers"]["include/fsim/Public.h"] = {"domain": "simulation", "kind": "c_abi"}
        self.input_error("the C ABI belongs to the adapters domain")
        self.policy["public_headers"]["include/fsim/Public.h"] = {"domain": "contracts", "kind": "value"}
        self.policy["public_exceptions"] = [{"from": "nobody", "to": "include/fsim/Public.h", "until": "ARCH-06", "reason": "x"}]
        self.input_error("public exception from unknown module or header nobody")

    # recheck item 1: Windows case variants meet the same ownership rules, and only the canonical spelling passes
    def test_a_case_variant_file_claim_cannot_take_a_reserved_destination(self):
        path = "src/Training/native/mission/Takeover.cpp"
        self.write(path, "int takeover;\n")
        self.append("src/session/CMakeLists.txt", "target_sources(fsim_session PRIVATE ../Training/native/mission/Takeover.cpp)\n")
        self.rejects("planned-owner", "reserved for planned module mission (ARCH-05)")
        self.policy["modules"]["session"]["files"].append(path)
        self.input_error(f"file-claim: session (simulation) claims {path}, reserved for planned module mission (ARCH-05)")

    def test_case_variant_policy_claims_meet_the_same_rules(self):
        for path, detail in (("src/TRAINING/native/vecenv/Take.cpp", "in the destination of env (training)"),
                             ("src/Env/Take.cpp", "in the legacy directory of env (training)"),
                             ("src/Training/Loose.cpp", "inside the training domain root, outside its own destinations")):
            with self.subTest(path=path):
                self.policy["modules"]["session"]["files"] = [path]
                self.input_error(f"file-claim: session (simulation) claims {path}")
                self.input_error(detail)

    def test_case_variant_policy_directories_are_one_owner(self):
        self.policy["modules"]["ms"]["destinations"].append("src/simulation/FLIGHT")
        self.input_error("duplicate-owner: src/simulation/FLIGHT is claimed by sim and ms")

    def test_a_noncanonical_policy_spelling_is_an_input_error(self):
        self.policy["modules"]["flow"]["files"].append("src/World/FlowFires.cpp")
        self.input_error("file-claim: flow (visualization) claims src/World/FlowFires.cpp, spelled differently from src/world")
        self.policy["modules"]["flow"]["files"].pop()
        self.policy["modules"]["env"]["destinations"] = ["src/Training/native/vecenv"]
        self.input_error("destination src/Training/native/vecenv is spelled differently from its domain root src/training")

    def test_a_source_spelled_differently_on_disk_than_its_owner_is_rejected(self):
        self.write("src/Training/native/vecenv/VecEnv.cpp", '#include "session/Header.h"\n')
        self.append("src/env/CMakeLists.txt", "target_sources(fsim_env PRIVATE ../Training/native/vecenv/VecEnv.cpp)\n")
        self.rejects("noncanonical-path", "spelled differently from env's directory src/training/native/vecenv")

    def test_legacy_shared_claims_keep_their_canonical_spelling(self):
        # Flow's files in src/world and the Viewer's in src/app, exactly as the policy spells them
        self.assertIn("src/world/FlowSolver.cpp", self.policy["modules"]["flow"]["files"])
        self.assertIn("src/app/viewer_main.cpp", self.policy["modules"]["viewer"]["files"])
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    # recheck item 2: a wildcard import of an import function cannot hide a dynamic import
    def test_a_wildcard_import_function_binding_is_unsupported_even_when_registered(self):
        self.policy["python"]["dynamic_imports"] = [{"module": "fsim.world", "function": "load", "reason": "fixture"}]
        for module, call in (("importlib", "import_module"), ("builtins", "__import__")):
            with self.subTest(module=module):
                self.write("python/fsim/world.py", f"from {module} import *\n\n\ndef load(name):\n    return {call}(name)\n")
                self.rejects("python-dynamic-import", "unsupported dynamic-import form in <module>")
                self.policy["python"]["dynamic_imports"] = []
                self.rejects("python-dynamic-import", "unsupported dynamic-import form in <module>")
                self.policy["python"]["dynamic_imports"] = [{"module": "fsim.world", "function": "load", "reason": "fixture"}]

    def test_a_wildcard_import_from_another_module_is_not_an_import_function(self):
        self.write("python/fsim/world.py", "from math import *\n\n\ndef area(r):\n    return pi * r * r\n")
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)

    # review finding 3: a package initializer on the way to an imported module runs too
    def test_an_intermediate_package_initializer_cannot_load_an_optional_package(self):
        self.write("python/fsim/_training/__init__.py", "import torch\n")
        self.write("python/fsim/_training/batch.py", "def run():\n    pass\n")
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)  # (nothing imports the subpackage yet)
        for importer in ("from ._training.batch import run\n", "import fsim._training.batch\n"):
            with self.subTest(importer=importer):
                self.write("python/fsim/__init__.py", "from . import _native\nfrom .world import World\n" + importer)
                self.rejects("python-optional-eager", "eagerly imports optional package torch (through fsim._training)")

    def test_initialization_edges_create_no_facade_cycle_and_follow_deferral(self):
        self.write("python/fsim/_training/__init__.py", "import torch\n")
        self.write("python/fsim/_training/batch.py", "from fsim._training import helpers\n")
        self.write("python/fsim/_training/helpers.py", "")
        self.write("python/fsim/__init__.py", "from . import _native\nfrom .world import World\n\n\n"
                   "def batch():\n    from ._training.batch import run\n    return run\n")
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)  # (deferred: the initializer runs only when used; no root backedge invented)

    # review finding 4: an exception must close its own cycle
    def test_an_exception_between_two_separate_cycles_is_stale(self):
        self.write("python/fsim/a.py", "def f():\n    import fsim.c\n    import fsim.b\n")
        self.write("python/fsim/c.py", "def g():\n    import fsim.a\n")
        self.write("python/fsim/b.py", "def f():\n    import fsim.d\n")
        self.write("python/fsim/d.py", "def g():\n    import fsim.b\n")
        backedges = [("fsim.c", "fsim.a"), ("fsim.d", "fsim.b")]
        self.policy["python"]["exceptions"] = [{"from": f, "to": t, "until": "ARCH-07", "reason": "fixture"} for f, t in backedges]
        status, output = self.run_gate()
        self.assertEqual(status, 0, output)
        self.policy["python"]["exceptions"].append({"from": "fsim.a", "to": "fsim.b", "until": "ARCH-07", "reason": "between cycles"})
        self.rejects("python-exception", "legacy backedge fsim.a -> fsim.b no longer closes a cycle")


class BaselineFixtures(FixtureRepository):
    """The baseline record (baseline.py): the actual graphs and hashes, reproducible, dated by its caller."""

    def baseline(self, out):
        result = subprocess.run([sys.executable, "-B", str(CHECK.with_name("baseline.py")), "--repo", str(self.repo),
                                 "--policy", str(self.repo / "policy.json"), "--date", "2026-10-16", "--out", str(out)],
                                capture_output=True, text=True, check=False)
        return result.returncode, result.stdout + result.stderr

    def test_the_record_holds_the_actual_graph_and_is_reproducible(self):
        self.write("policy.json", json.dumps(self.policy))
        status, output = self.baseline(self.repo / "one.json")
        self.assertEqual(status, 0, output)
        status, output = self.baseline(self.repo / "two.json")
        self.assertEqual(status, 0, output)
        one = (self.repo / "one.json").read_text(encoding="utf-8")
        self.assertEqual(one, (self.repo / "two.json").read_text(encoding="utf-8"))
        record = json.loads(one)
        self.assertEqual(record["native"]["links"]["world"], ["core", "flow", "ipc", "render"])
        self.assertEqual(record["native"]["modules"]["sdk"]["kind"], "assembly")
        self.assertIn("src/sim/Header.h", record["native"]["modules"]["sim"]["files"])
        self.assertEqual(record["checker"]["issues"], [])
        self.assertEqual(record["python"]["modules"]["fsim.world"]["owner"], "fsim")

    # review finding 5: every declared extension input and link is in the record
    EXTENSIONS = """function(fsim_python_module target name source library)
    add_library(${target} MODULE ${source} src/fsim_py.h)
    set_target_properties(${target} PROPERTIES PREFIX "" OUTPUT_NAME ${name} SUFFIX ".pyd")
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(${target} PRIVATE ${library} "${FSIM_PYTHON3_DLL}")
    target_compile_definitions(${target} PRIVATE Py_LIMITED_API=0x030B0000)
endfunction()
fsim_python_module(fsim_python _native src/_native.c fsim::sdk)
target_sources(fsim_python PRIVATE src/_native_world.c)
if(TARGET fsim_vision)
    fsim_python_module(fsim_python_vision _vision src/_vision.c fsim::vision)
endif()
"""

    def extensions(self, name):
        self.write("policy.json", json.dumps(self.policy))
        status, output = self.baseline(self.repo / name)
        self.assertEqual(status, 0, output)
        record = json.loads((self.repo / name).read_text(encoding="utf-8"))
        return {e["target"]: e for e in record["package_outputs"]["python_extensions"]}

    def test_helper_inputs_and_links_are_recorded_and_their_changes_are_visible(self):
        self.write("python/CMakeLists.txt", self.EXTENSIONS)
        for source in ("_native.c", "_native_world.c", "_vision.c", "fsim_py.h"):
            self.write("python/src/" + source, "/* " + source + " */\n")
        first = self.extensions("first.json")
        native = first["fsim_python"]
        self.assertEqual(native["module"], "fsim._native")
        self.assertEqual([s["declared"] for s in native["sources"]], ["src/_native.c", "src/_native_world.c", "src/fsim_py.h"])
        self.assertTrue(all("sha256" in s for s in native["sources"]))
        self.assertEqual([(l["declared"], l["resolved"], l["by"]) for l in native["links"]],
                         [("fsim::sdk", True, "fsim_python_module"), ("${FSIM_PYTHON3_DLL}", False, "fsim_python_module")])
        self.assertEqual(native["compile_definitions"], ["Py_LIMITED_API=0x030B0000"])
        self.assertEqual(first["fsim_python_vision"]["conditions"], ["if(TARGET fsim_vision)"])
        # the shared helper header changes: both extensions' recorded input changes
        self.write("python/src/fsim_py.h", "/* changed */\n")
        second = self.extensions("second.json")
        for target in ("fsim_python", "fsim_python_vision"):
            before = {s["declared"]: s.get("sha256") for s in first[target]["sources"]}
            after = {s["declared"]: s.get("sha256") for s in second[target]["sources"]}
            self.assertNotEqual(before["src/fsim_py.h"], after["src/fsim_py.h"])
        # a direct link and a helper link change are both visible
        self.append("python/CMakeLists.txt", "target_link_libraries(fsim_python PRIVATE winmm)\n")
        self.write("python/CMakeLists.txt", (self.repo / "python/CMakeLists.txt").read_text(encoding="utf-8")
                   .replace('${library} "${FSIM_PYTHON3_DLL}"', '${library} "${FSIM_PYTHON3_DLL}" ws2_32'))
        third = self.extensions("third.json")
        links = [(l["declared"], l["by"]) for l in third["fsim_python"]["links"]]
        self.assertIn(("winmm", "direct"), links)
        self.assertIn(("ws2_32", "fsim_python_module"), links)
        self.assertIn(("ws2_32", "fsim_python_module"), [(l["declared"], l["by"]) for l in third["fsim_python_vision"]["links"]])

    def test_a_missing_declared_source_is_recorded_as_missing(self):
        self.write("python/CMakeLists.txt", self.EXTENSIONS)
        self.write("python/src/_native.c", "")
        native = self.extensions("missing.json")["fsim_python"]
        self.assertTrue(next(s for s in native["sources"] if s["declared"] == "src/fsim_py.h")["missing"])

    def test_a_rejected_tree_is_recorded_as_rejected(self):
        self.append("src/ms/Header.h", '#include "sim/Header.h"\n')
        self.write("policy.json", json.dumps(self.policy))
        status, output = self.baseline(self.repo / "rejected.json")
        self.assertEqual(status, 1, output)
        record = json.loads((self.repo / "rejected.json").read_text(encoding="utf-8"))
        self.assertTrue(any("undeclared-include" in issue for issue in record["checker"]["issues"]))


if __name__ == "__main__":
    unittest.main()
