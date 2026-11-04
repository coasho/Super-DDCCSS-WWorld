# Component parameter and source differences

[parameter-deltas.json](parameter-deltas.json) records all216 selected AAM
component cells against the immutable main baseline named in the artifact.
It retains the complete before/current descriptor, every role's complete
referenced parameter record, and the real design values used by that component.
Native model XML values and literal source-file hashes are additional current
inputs. The source list distinguishes substantive text changes from checkout
line endings while preserving the literal digests.

An unchanged parameter is reported unchanged. A newly explicit assumption is
reported as a new input; it is not a measured improvement. Original units,
sources, assumed/derived/sourced bases, uncertainty and validity remain intact.
The baseline is resolved to a commit hash rather than a moving branch label.
Generated XML inputs remain mutable until the required normal production and
runtime evidence are refreshed.

The independent family proofs provide each cell's actual equations, theory
delta, method residuals, domain and limitations through the
[component matrix contract](component-matrix.md). The separate calibration
ledger requires equipment-specific held-out improvement and positive
uncertainty. Neither a source difference nor an engineering equation by itself
supplies that evidence. This artifact edits no equipment card, accepted M02
model, controller, tolerance, scene or runtime implementation.
