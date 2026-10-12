# Native architecture acceptance

Run with Python 3.11 or newer; only the standard library is used:

```powershell
python -B tools/architecture/check.py --repo .
python -B tools/architecture/test_check.py
```

The CMake architecture acceptance tests run independently of Python SDK binding
availability. Fixture scratch follows `TMP` (on Windows, the fallback is
`D:/ENV/tmp`). No compiler, renderer, installed SDK, or third-party Python
package is needed for this gate.

## Policy and evidence

`policy.json` explicitly registers each native module's target, aliases, source
ownership, headless status, and permitted direct dependencies. CMake supplies
the actual edges: permitted dependencies alone do not make an import valid.
Imports must reach their owning module through the declared graph. This retains
legitimate transitive imports, such as platform services through core.

All native `src/**/CMakeLists.txt` files and all C/C++ sources and headers under
`src` are inspected, including optional renderer branches in a headless checkout.
The checker rejects unknown targets or source owners, forbidden edges, dependency
cycles, mismatched target sources, imports outside the dependency closure, and
VSG/Vulkan dependencies in headless modules. Public `include/fsim` headers cannot
import private sources or third-party implementation headers. Vendor headers are
identified from the checked-in third-party tree and GPU include names; an existing
public SDK header takes precedence over a vendor basename collision.

Directory ownership includes private helper headers that CMake does not list as
build inputs. Explicit file ownership takes precedence in two shared directories:

| Module | Shared directory ownership |
| --- | --- |
| `flow` / `fsim_flow` | `src/world/FlowSolver.{h,cpp}` and `src/world/FlowFires.{h,cpp}` |
| `viewer` / `flightsim-viewer` | `src/app/viewer_main.cpp` and `src/app/DemoAutopilot.h` |

Flow has its own core dependency closure and does not inherit world dependencies.
It is an optional GPU module, including the existing Win32 Flow loader; isolating
that loader behind platform is deferred to the renderer maintenance pass. The
gate does not introduce an OS-header restriction or claim a renderer/GPU build
has passed. JSBSim and Flow's supplied header target are registered external
dependencies, not native modules whose source is rewritten or scanned.

## Deliberately limited CMake parser

The parser reads source declarations rather than a configured File API codemodel,
so disabling rendering cannot hide a boundary violation. It takes the union of
all conditional branches instead of evaluating platform and option variables.
Comments, quoted arguments, bracket comments, multiline commands and native
aliases are handled. The accepted dependency subset is:

- Literal `add_library`, `add_executable`, and `target_sources` source paths.
- Literal `target_link_libraries` targets, including visibility/configuration
  keywords and literal semicolon-separated link lists.
- The existing `$<LINK_LIBRARY:WHOLE_ARCHIVE,...>` expression with literal targets.
- Literal `target_include_directories` paths, including the source/current-source
  root variables and the existing build/install interface forms. Build paths
  participate in header lookup; install paths do not belong to a source checkout.

Unresolved variables and other generator expressions in these declarations fail
closed. Directory-wide links/includes, graph-property mutation, included CMake scripts,
functions/macros and dynamic CMake evaluation under `src` also fail closed; they
could otherwise conceal graph changes. This is not a general CMake interpreter.
Root dependency setup and non-native example/tool/test targets are outside its
scope. Introduce a tested parser extension and update policy when changing how
native targets are declared, rather than hiding a target behind a variable.

Include scanning handles literal quote/angle imports, removes C/C++ comments and
raw-string examples, and inspects every preprocessor branch. Macro includes fail
closed. Target include paths from the declared dependency closure also participate
in lookup: adding `../sim` cannot hide an undeclared import behind `OnlySim.h`.
All matching private candidates are checked conservatively; ambiguous search
paths cannot select an owner that hides an out-of-closure header. Public headers
are checked against all registered target search paths. The gate checks declared
module reachability, not exact compiler search precedence or CMake PUBLIC/PRIVATE
include propagation. Source-specific include-directory properties are accepted
only for vendor paths (the existing NTIA wrappers); other forms fail closed.
Existing broad `src` include directories
are retained; this gate detects undeclared imports despite that visibility.

The executable fixtures cover the original `ms -> sim` and `world -> ipc` leaks,
upward links/imports, cycles, public/private and vendor leakage, headless GPU
headers/links, unknown modules, shared Flow ownership, transitive platform access,
comments, conditional branches, whole-archive links and unsupported declarations.
