# Native architecture acceptance

Run with Python 3.11 or newer; only the standard library is used:

```powershell
python -B tools/architecture/check.py --repo .
python -B tools/architecture/test_check.py
python -B tools/architecture/baseline.py --date <YYYY-MM-DD>   # the dated baseline record (ARCH-01)
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

The graph also keeps native simulation/session dependencies below `env`, keeps
rendering out of `env`, and keeps the Viewer independent of mission episodes.
Focused negative fixtures exercise `session -> env`, `env -> render` and
`viewer -> env` as both links and imports, including native target names and
aliases. These are checks of the existing policy, not additional runtime layers.

Public headers also reject direct references, forward declarations and aliases
for the private `session`, `env` and `ipc` namespaces. The existing trusted World
facade retains its exact native forward declaration, `impl`, `borrowed`, borrowed
constructor and storage declarations in `include/fsim/World.h`. Only the existing
World class's direct members are exempt: another class in that header, including
a nested actor endpoint, cannot reuse the exemption. Comments and string literals
do not create type references. This is a lexical declaration guard, not a C++ type
resolver or a security sandbox; M5 must still verify that public actor endpoints
cannot return a public World/Range/referee handle or bypass Agent permissions.

Source walks are limited to the explicit `src`, `include/fsim` and `third_party`
roots and prune excluded private directories before descending into them.

Directory ownership includes private helper headers that CMake does not list as
build inputs. Explicit file ownership could take precedence in a shared legacy
directory. Until ARCH-06 two modules held such claims: Flow
(`src/world/FlowSolver.{h,cpp}`, `src/world/FlowFires.{h,cpp}`) and the Viewer
(`src/app/viewer_main.cpp`, `DemoAutopilot.h`, `StrikeCapture.h`). Both now own
their destinations, and no module holds one.

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
They also cover mission/Viewer reverse dependencies and private native namespace
exposure without an include, retaining only the exact legacy World facade seams.

## Domains, kinds and the staged migration (ARCH-01)

The policy (`fsim.architecture.source-policy.v2`) gives every module a domain and
a kind, following the [architecture plan](../../docs/architecture/README.md):

| Domain | Root | Kind | May depend on (implementations) |
| --- | --- | --- | --- |
| contracts | `src/contracts` | contract | nothing |
| foundation | `src/foundation` | implementation | contracts |
| simulation | `src/simulation` | implementation | foundation, contracts |
| training | `src/training` | implementation | simulation, foundation, contracts |
| visualization | `src/visualization` | implementation | foundation, contracts |
| adapters | `src/adapters` | final assembly | any domain, other assemblies |
| applications | `src/applications` | application (external consumer) | any domain, assemblies |

Kinds add rules of their own: nothing depends on an application; a domain
implementation or contract never depends on a final assembly (the SDK extraction
cycle, training -> final SDK -> training); a contract depends only on contracts.
Implementation domains hold only headless modules; graphics belongs to
visualization, assemblies and applications. The rules apply to every direct link
and to every include of another module's header, so a transitive link cannot
launder a reverse include.

The current tree has three reverse edges, listed as exact exceptions that name
the package removing them: `ipc -> sim` (ARCH-02 moves the wire values below both
sides), `world -> sim` and `ui -> sim` (ARCH-06 makes visualization consume
read-only frames and contracts). An exception covers one module pair and one rule.
One that names an edge that has gone, or an edge that breaks no rule, fails as
stale, so exceptions are removed with their cause.

The policy is held to the actual graph in both directions. A link needs its
allowance, and an allowance without a link fails as `unused-allowance`: a
permitted edge is not evidence that a dependency exists. A source listed by two
targets fails as `duplicate-source`.

### Ownership of the destination directories

Each module lists its current (legacy, flat) `directories` and its future
`destinations` inside its domain root. Planned modules that have no target yet
(`contracts`, `simulation_api`, `modeled_dms`, `mission`, `vision_core`, the
Python and Vision extension adapters, `oms`, `demo`, `training_vision`,
`testbench`) reserve their destinations. Ownership resolves as follows:

- an exact file entry wins first, then the longest claimed directory prefix. An
  exact entry keeps a legacy shared directory (as Flow's in `src/world` and the
  Viewer's in `src/app` did until ARCH-06), but never overrides another owner. Inside a domain
  root it must lie in the module's own destination; in a legacy directory, the
  directory's owner must be in the same domain. A claim on a reserved, foreign or
  unassigned file is an input error (`file-claim`);
- paths are identified case-insensitively, as Windows names them, so a case
  variant such as `src/Training/...` meets the same owner rules. Only the
  policy's own spelling passes:
  - a policy path spelled differently from its domain root or covering
    directory is an input error;
  - two policy paths differing only in case are one claim (`duplicate-owner`);
  - a source whose on-disk or declared spelling differs from its owner's
    directory or claim fails as `noncanonical-path`;
- two claims of the same length fail as `duplicate-owner`; the same path claimed
  twice in the policy is an input error;
- a file in a planned owner's destination fails as `planned-owner` until that
  module's target is registered;
- a file inside a domain root but outside every destination (for example
  `src/training/native/Stray.cpp`) has no owner.

So the nested layout can be adopted directory by directory without admitting
unowned code. The policy itself is validated before any source is read. Each of
the following is an input error, never ignored:
- an unknown key;
- a kind foreign to its domain;
- a destination outside its domain root, or a legacy directory inside one;
- graphics in a non-visualization implementation;
- an exception without a later roadmap phase, or a planned module whose phase
  names neither a later roadmap package nor a module plan's package (`SIM-x`,
  `TRN-x`, `VIS-x`; the native training-image provider waits for VIS-F, HC-23).

Do not run the destination layout against unmigrated directories as if it were
the present tree: a legacy directory stays valid until its package moves it.

### Python imports

`python_imports.py` scans `python/fsim` (literal imports; function-local
imports are deferred edges, everything else eager). It requires:

- an acyclic module graph, deferred edges included. Since ARCH-07 the policy
  lists no legacy backedge: `fsim.training`'s implementation takes `Error` from
  `fsim._native`, and `fsim.world` builds agents and referees through
  `fsim._simulation`'s factories, which `fsim.agent` registers. A listed
  exception that becomes eager or disappears fails.
- private implementation packages (`fsim._contracts`, `_simulation`,
  `_training` and `_visualization`, which ARCH-07 created, and `_adapters`)
  import only what the policy lists for them. They never import the package root or a
  public facade module, not even function-locally.
- no optional module (`fsim.gym`, `fsim.sb3`, `fsim.vision`) or optional
  package (`gymnasium`, `stable_baselines3`, `torch`) in the package's eager
  import closure. Importing `a.b.c` runs the initializers of the packages on the
  way, so that closure follows initialization edges to each intermediate
  package. These are recorded apart from explicit imports, and the always
  initialized root is not one, so they invent no facade cycle and no facade
  backedge.
- an exception that still closes its own cycle: its target reaches its source.
  Lying in some other cycle is not enough.
- dynamic imports only at registered sites (today `fsim._adapters.actor.serve`,
  the isolated actor process's loop, which `fsim.isolation` runs). A
  site is a direct call of `importlib.import_module` (through any module alias),
  `__import__`, or a name bound by `from importlib import import_module [as x]` or
  `from builtins import __import__ [as y]`. An import function used other than
  as a direct call (stored, passed, or fetched with `getattr`) cannot be followed,
  and fails as an unsupported form, registered or not. So does a wildcard import
  from `importlib` or `builtins`, which binds the import functions invisibly.
- every subpackage a declared private package.

Compiled extensions are owner-level leaves; their native links are checked as
native targets.

### Baseline record

`baseline.py --date <date>` writes `docs/architecture/validation/arch-01-baseline.json`
from the gate's own analysis. The record holds:
- HEAD and the scoped working-tree changes;
- each module's target, library type, domain, kind, directories and owned
  sources with SHA-256;
- the actual link, external-link and include graphs, and the exceptions in use;
- the public headers with hashes and including modules;
- the Python modules with owners and hashes, every import with its line, the
  cycles, the eager closure and the dynamic sites;
- the build's declared package outputs: target types, install rules as written,
  and each CPython extension module with every input and link it is declared
  with. Helper functions are expanded with their call's arguments, so the shared
  `src/fsim_py.h` and the stable-ABI `${FSIM_PYTHON3_DLL}` link are the
  extension's. Direct `target_sources` and `target_link_libraries` are added, as
  are include directories, definitions, properties and the enclosing `if()`
  branches. A file input carries its hash; a variable or generator expression is
  recorded as written and marked unresolved, never guessed.

The record includes the gate's verdict; a rejected tree is recorded and exits 1.
It is a dated record, not a gate: source edits change its hashes, so refresh it
at each migration handoff.

### Reconciling with the configured build

The source parser stays the ownership authority: it reads every branch, so no
preset can hide a violation. The configured graph is checked against it per
required preset (`ucrt64-headless`, `ucrt64-release`) once the moves begin. The
target name is the module identity, and ARCH-03 keeps target names and aliases
while moving directories, so both graphs key by target. ARCH-01 does not
configure:

1. Before configuring, write the CMake File API query
   `build/<preset>/.cmake/api/v1/query/codemodel-v2`; read the reply's targets,
   sources, dependencies, link fragments and include directories.
2. Every configured target built from `src` is a registered module target, and
   every configured source has that module as its owner (the same owner rule).
3. Every configured target-to-target dependency, whole-archive fragments
   included, is an edge of the source graph. A configured edge the parser did
   not see means a hidden declaration, and fails.
4. A source edge missing from a preset is accepted only when its branch is
   disabled there (render-only targets in `ucrt64-headless`); the record names
   the branch.
5. Configured include directories resolve within the module's dependency
   closure. Broad `src` roots are reported until each target narrows them.
6. The reconciliation runs in the exclusive build slot, after the preset is
   configured, as a test of its own. A preset that cannot be configured is
   pending, not passed.

## Neutral contracts and public headers (ARCH-02)

`contracts` (`fsim_contracts`, `src/contracts`) is a header-only interface target:
the wire and recording layouts (`world_stream/WorldLayout.h`,
`vision_stream/VisionLayout.h`, `recording/RecordingFormat.h`), the stream batch
(`world_stream/SnapshotBatch.h`) and the native construction values
(`simulation/WorldOptions.h`). These definitions keep the names, namespaces,
fields and order they had in `src/ipc`, `src/sim` and `src/session`, so no type,
layout or byte changed with the owner. Contracts include only the standard
library and public contract headers, and link nothing.

The transport (`ipc`) links the contracts, core and platform, and no
simulation module: the `ipc -> sim` exception is gone. `tests/test_contracts.cpp`
builds against `fsim::contracts` alone. `tests/test_state_layout.cpp` holds every
frozen offset, stride and old-recording read, plus the writer's framing and
fields at literal offsets.

Every `include/fsim` header is classified in the policy's `public_headers` by
owner domain and kind:
- `value`: plain data;
- `interface`: values with their functions, or an extension point;
- `facade`: an execution API;
- `c_abi`: the C ABI.

Contracts headers are values or interfaces, and the C ABI belongs to the
adapters. A module's include of a public header, and a public header's include of
another, follow the same direction rules as module links: a simulation module or
header cannot include a training one, a contract cannot include a domain API, and
only assemblies and applications include the C ABI or an assembly's API. An
unclassified header, or a classified one that is gone, fails.

No public exception remains: ARCH-06 removed the `ui` ones, and ARCH-07
removed `VecEnv.h -> fsim_c.h` by giving the C handle the facade shares with
the C ABI its own contracts header, `fsim/fsim_vecenv_handle.h`.

The scenario document's values and reading/writing are in
`fsim/ScenarioDocument.h` (simulation). `fsim/Scenario.h` (training) includes
it and adds `applyScenario` and `vecEnvOptions`, so its includers see the same
declarations. The runtime applies its own `session::ScenarioRecipe`, and the
VecEnv mapping is training's (`src/env/ScenarioVecEnv.cpp`).

Public include files are classified and scanned whatever their suffix (HC-00).
Every regular file under `include/fsim` is public, because the install copies
the directory whole: an `.inc`, `.ipp` or `.tpp` file meets the same
classification and direction rules as a header. Source scanning includes those
fragment suffixes and `.hxx`. Explicit failures:
- an include of an unclassified public file is reported where it is included
  (`unclassified-public-include`), beside the unclassified file itself;
- a resolved private include whose suffix is not scanned fails as
  `unscanned-include`;
- a public file that is not UTF-8 text fails as `unsupported-public-file`.

## Foundation and simulation leaves in their destinations (ARCH-03)

`platform`, `core`, `io` and the transport (`ipc`) are in `src/foundation`.
`sim`, `control`, `effects`, `comm` and `ms` are in `src/simulation` (`flight`,
`flight_control`, `environment`, `communications/network`, `mission_systems`).
Their target names, aliases and namespaces are unchanged, includes name the
destination paths (`foundation/core/Log.h`), and the policy no longer lists
their legacy directories. A legacy directory the policy still lists must hold
sources: once a module has moved, its old claim fails as
`stale-legacy-directory` instead of lingering.

A moved module exports its headers through an include view
(`cmake/IncludeViews.cmake`), not the shared `src` root. The view is a
build-tree mirror of the module's own headers, so a target compiles with its
own headers and its linked providers' only. A moved module fails as
`broad-include-root` when it declares an include directory under `src` outside
its destinations, or one holding the whole src tree: `src` itself or any
ancestor, such as the repository root. The comparison uses resolved paths, so
`src/..` is the repository root. The `include_view_<module>` tests compile
allowed and forbidden headers in each target's own environment
(`cmake/CheckIncludeView.cmake`):
- a root that resolves to `src` or an ancestor of it is refused before any
  probe;
- forbidden headers are probed both from src (`simulation/...`) and from the
  repository root (`src/simulation/...`).

## Simulation runtime, modeled DMS and the lower facade (ARCH-04)

The runtime (`fsim_session`) is in `src/simulation/runtime`, split by state
owner into `world/`, `services/` and `stores/`. Modeled DMS is its own module,
`modeled_dms` (`src/simulation/communications/dms`), over MS RF propagation and
beside the generic network. The lower facade, `simulation_api`
(`src/simulation/api`), holds the public World, Agent and recording
implementations and the options mapping. Training and the SDK assembly use it;
it uses neither.

Both new modules are registered (no longer planned) with allowed dependencies
equal to their links. Checks:
- `test_the_lower_simulation_facade_stays_below_training_and_the_sdk_assembly`
  rejects a facade link to, or include of, training or the final assembly;
- the `simulation_api_link` test links every facade object with only its
  declared closure;
- the three modules have `include_view_<module>` probes.

## Training extraction (ARCH-05)

`fsim_env` is in `src/training/native`: the legacy VecEnv in `vecenv/` and the
native mission episode owner in `episodes/runtime/`. The mission
implementation is the `mission` module (`fsim_training_mission`): `mission/`,
`batch/`, and the public facade in `episodes/api/`. It is over the native
episode owner, `simulation_api` and the runtime. The final SDK links it; it
never links the SDK.

Checks:
- `test_mission_training_stays_below_the_sdk_assembly_that_links_it` rejects
  the reverse link (a forbidden kind and a cycle) and an include of the
  assembly's headers;
- `training_link` links both training targets whole with only their closure;
- `include_view_env` and `include_view_mission` probe their environments.

`fsim_include_view` accepts `DIRECTORIES` for a target spread over several
destination directories.

## Visualization, Vision adapter and the Viewer's demo (ARCH-06)

Visualization is in `src/visualization`:
- `render`;
- `scene` (`fsim_world`), which also holds the visual effects in `vfx/effects`;
- the optional Flow solver in `vfx/flow`;
- `ui`;
- `vision_core` (offscreen camera scenes).

The scene and UI read contracts and read-only frames: the `world -> sim` and
`ui -> sim` exceptions and the UI's two public-header exceptions are gone. The
Vision World-handle API and C ABI are the `vision` adapter (`src/adapters/vision`).

The Viewer (`src/applications/viewer`) links the demo application
(`src/applications/demo`, `demo`) only to keep its `--demo` command line. That is
the exact exception `viewer -> demo` (rule `kind`, until ARCH-08), which
`test_a_temporary_application_composition_needs_its_exact_kind_exception` holds to
its rule.

The include-view probe check searches vendor and imported roots as system roots
and leaves the compiler's implicit directories to it, as the build does.
