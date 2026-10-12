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


class ArchitectureFixtures(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="fsim-architecture-", dir=SCRATCH)
        self.addCleanup(self.temp.cleanup)
        self.repo = Path(self.temp.name)
        # Literal fixture graph: core -> platform; sim -> core; session -> sim/ms;
        # world -> render/core/flow/ipc. Flow shares world/, but owns four files.
        targets = {
            "platform": ("fsim_platform", [], True),
            "core": ("fsim_core", ["platform"], True),
            "sim": ("fsim_sim", ["core"], True),
            "ms": ("fsim_ms", ["core"], True),
            "ipc": ("fsim_ipc", ["sim"], True),
            "session": ("fsim_session", ["sim", "ms"], True),
            "render": ("fsim_render", ["core"], False),
            "world": ("fsim_world", ["render", "core", "flow", "ipc"], False),
            "flow": ("fsim_flow", ["core"], False),
        }
        self.policy = {"modules": {}, "external_targets": ["vsg::vsg", "Vulkan::Vulkan"]}
        for name, (target, dependencies, headless) in targets.items():
            module = {
                "target": target,
                "aliases": ["fsim::" + name],
                "directories": ["src/" + name] if name != "flow" else [],
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
            else:
                cmake = f"src/{name}/CMakeLists.txt"
                source = "Header.h"
                self.write(f"src/{name}/Header.h", "#pragma once\n")
            self.policy["modules"][name] = module
            self.append(cmake, f"add_library({target} STATIC {source})\n"
                        f"add_library(fsim::{name} ALIAS {target})\n")
            if dependencies:
                self.append(cmake, f"target_link_libraries({target} PUBLIC "
                            + " ".join("fsim::" + dependency for dependency in dependencies) + ")\n")
        self.write("include/fsim/Public.h", "#pragma once\n#include <cstdint>\n")

    def write(self, path, text):
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


if __name__ == "__main__":
    unittest.main()
