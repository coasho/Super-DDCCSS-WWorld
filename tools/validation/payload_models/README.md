# Payload configuration and evidence report

Read-only, standard-library reporting over the immediate `assets/stores/*.json`
catalogue. It reuses Hangar's component metadata resolver. It does not execute or
validate flight, sensing, guidance, fuzes or damage.

From the repository root:

```powershell
D:/ENV/python/3.13/python.exe -B -m unittest discover -s tools/validation/payload_models -p test_report.py
D:/ENV/python/3.13/python.exe -B tools/validation/payload_models/report.py --output-dir build/architecture-reviews/sim-m03-model-report
```

Open `report.html` locally; `report.json` preserves the complete resolved input
provenance. Both are offline files. Search by card, role, concrete class or
source text. Expand a component to inspect its declaration, referenced parameter
provenance, limitation note, validity and indexed source paths.

Optionally import one or more CTest JUnit artifacts using `--native-junit PATH`.
Their outcomes remain a separate table; passing a program never marks each
equipment variant physically accepted. The report hashes each artifact and
retains failed/skipped cases. It does not establish that artifact's binary
provenance, freshness or applicability to the current source snapshot.

Exit 0 means report generation found no unmapped available IDs or imported
failed/skipped cases. Exit 1 still produces the diagnostic reports and means
one of those conditions remains. Neither exit value establishes model fidelity.
Malformed card/profile/provenance, missing indexed source or malformed/empty
JUnit fails generation explicitly.

The ID-to-class table is a maintained source index. Generation verifies the
referenced declarations and registration IDs, and hashes the selected class,
registration, report, resolver and card files. It is not C++ dispatch analysis,
RTTI, release-instance evidence or a guarantee that a declared profile passes
every provider's execution validation. Update the index when factories change.
Unknown providers stay unmapped; no generic fallback is invented.

Counts distinguish cards, implementation IDs and concrete classes. Parameter
configuration hashes include resolved slot values and units, excluding identity,
paths and citations. They are not full model/asset equivalence or behavioral
comparisons. An empty parameter set therefore has one configuration, even when
generated assets differ. Published parameters do not promote an assumed
component, and an interface version is not an evidence or maturity level.
Runtime evidence and physical acceptance remain `not_assessed` per component;
use the linked package validation records for those separate claims.
