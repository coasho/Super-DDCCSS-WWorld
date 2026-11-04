# Per-model component closure

The mandatory denominator is the current **24 authored AAMs × 9 components =
216 cells**. Each cell requires separate theory, independent method, native,
public/installed and visual evidence. Availability and a neutral equipment
selector establish neither executed behavior nor equipment calibration.

[component-matrix.json](component-matrix.json) is the machine-readable current
projection. [parameter-deltas.json](parameter-deltas.json) retains the exact
descriptor, referenced parameter records and source/design differences from the
recorded main baseline. Both are produced by
[component_matrix.py](../../../tools/validation/weapon_quality/component_matrix.py).
These files must be regenerated after their inputs change. A projection without
evidence deliberately has zero closed cells and a false engineering flag.

The final report requires `--component-evidence` and independently rechecks all
216 current cells, all five gates and the separate 24-model calibration ledger.
An aggregate campaign pass cannot replace a missing component proof. The report
retains the complete admitted cell records and exact evidence-manifest hash.

## Identity and actual inputs

The collector independently reads the maintained store cards and equipment
catalogue, and reconciles them with the coverage roster. It requires exactly
24 AAMs and all nine selected components. The catalogue supplies the actual
selection-to-mechanism relationship; it supplies no invented hardware part name.
The card's availability, basis, validity, parameter source, units and uncertainty
are retained unchanged. An assumed parameter never becomes sourced because its
generic equation was verified.

Every cell binds the actual registration/include implementation files, its
card, maintained design TOML and generated primary native XML. The native XML
and its extracted flight-control, actuator, propulsion or aerodynamic parameters
are current products, explicitly mutable until forced production. An allocator
or aerodynamic descriptor with empty `parameter_refs` still has real design,
mixer and native model inputs; those inputs are not replaced by a blank record.
Scalar/configuration XML values are inline. Oversized generated numeric text
remains in its unchanged primary XML node; the projection records its exact
text SHA256, byte and line counts alongside the whole-file binding. This avoids
duplicating the existing aerodynamic grids in both maintained projections.

The canonical `parameter_sha256` is exactly:

```python
descriptor = card['component_models']['components'][component]
payload = {
    'descriptor': descriptor,
    'parameters': {
        role: card['parameters'][ref]
        for role, ref in descriptor.get('parameter_refs', {}).items()
    },
}
sha256(json.dumps(payload, sort_keys=True, separators=(',', ':'),
                  ensure_ascii=False, allow_nan=False).encode('utf-8'))
```

The complete parameter records are hashed, including basis, uncertainty, source
and explanatory metadata. Native XML and design hashes are additional inputs;
they are not silently inserted into this shared canonical parameter hash.

## Proof interchange

A manifest uses schema `fsim.sim-m03.component-evidence.v1`, a `cells` array and
a separate `calibration` array. A cell has `model` (the store-card ID),
`component`, and `evidence`, whose known gate names are `theory`, `method`,
`native`, `public_installed` and `visual`. Every supplied gate is an exact
`{artifact: {path, sha256}, row: <integer>}` reference. A partial manifest can
publish individual verified gates immediately; it cannot set a final flag.
Malformed, stale, contradictory and foreign proofs fail explicitly.

Proof documents contain `schema`, `passed`, `sources`, `raw` and `models`.
`sources` and `raw` are nonempty lists of current path/SHA256 bindings. Model
rows are unique by model/component. Every selected row must match the cell's
`model`, `component`, `implementation_id`, `mechanism_id`, `card_sha256` and
`parameter_sha256`, and have `passed: true`. All required cell source bindings
must be included. The raw artifacts must actually exist and retain their bytes.
The matrix consumes the independent collectors' proofs; it does not execute a
simulation or turn unvalidated counters into physical measurements.

| Gate | Proof schema | Required selected-row content |
| --- | --- | --- |
| theory | `fsim.sim-m03.component-reference.v1` | `theory`: equations, applicability/limits, theory delta and primary sources with title, URL, locator and exact scope |
| method | same reference schema | `method`: actual case, positive integer test/assertion counts, named actual/expected residuals and original tolerances; explicit `shared_equivalence` |
| native | `fsim.sim-m03.component-native.v1` | `native`: case/counts, exact native model, actual per-model execution, all nine coupled components and both existing UCRT64 presets |
| public_installed | `fsim.sim-m03.component-public-installed.v1` | `public_installed`: case/counts, exact model execution, C++/C99/Python routes and verified loaded identity |
| visual | `fsim.sim-m03.component-visual.v1` | `visual`: reviewed model, case/counts, actual fullscreen 1x, image binding and the complete enabled scene |

Execution records refuse failures, errors, skips and expected failures. A method
residual must be finite and satisfy its original nonnegative tolerance. A
shared-method equivalence has an explicit ID, exact model membership, mechanism,
scope and limitations; it cannot include another selected mechanism. Reusing a
method does not replace each model's native, installed or visual execution.

The visual scene list is Earth imagery, terrain, clouds, sunlight, shadows,
exhaust, contrails and native effects. Source option values alone do not supply
the required actual review proof. Native misses and failed coupled chains remain
failed; a method/source pass cannot rename them successful.

## Separate calibration ledger

There are exactly **24 calibration records**, independently reconciled with the
AAM roster. `uncalibrated` requires an explicit reason. `calibrated` requires a
hash-bound `fsim.sim-m03.equipment-calibration.v1` receipt for that exact model
and card epoch, actual equipment-specific observations, unchanged holdout,
parameter difference, baseline-to-new error reduction, retained raw/source
artifacts and positive uncertainty. Generic method checks and model names cannot
admit calibration. An engineering-complete cell retains its uncalibrated label
when no such separate evidence has been admitted.

## Reproduction and refusal

```powershell
D:/ENV/python/3.13/python.exe -B tools/validation/weapon_quality/component_matrix.py `
  --coverage build/architecture-reviews/sim-m03-production-20261103/current-coverage.json `
  --baseline e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f `
  --output docs/research/sim-m03/component-matrix.json `
  --parameter-deltas docs/research/sim-m03/parameter-deltas.json
```

Supply `--evidence <manifest>` to join current actual proofs. Exit code2 and
false final flags are the required result while any cell lacks a gate. A
clean source export supplies `--git-source-root` naming the original main
repository for immutable baseline reads; current inputs are still read from
the export's `--root`. No branch or worktree is created.
A
successful component matrix remains one mandatory input to the complete M03
campaign; it does not admit the remaining models, package, effects, mission
outcomes or equipment accuracy on its own. No accepted M02 rotor flight scope
is changed by this tool.
