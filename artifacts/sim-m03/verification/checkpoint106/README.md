# Archived SIM-M03 verification evidence

This directory preserves four original logs outside the disposable build tree.
The [manifest](manifest.json) records their original paths, byte counts and
SHA-256 hashes. Archiving did not rerun a build, a test or a visual scenario.

| Evidence | Observed result | Limit |
|---|---|---|
| [Checkpoint 106 SDK build](checkpoint106-full-build.log) | Build log reaches its final link step without a reported error | No reconstructed source-input manifest or full regression |
| [Checkpoint 106 focused checks](checkpoint106-focused-tests.log) | 9 of 10 CTest entries passed; `weapon_navigation_integration` failed | The failing native World case reports unavailable navigation diagnostics; its cause is not established by this archive |
| [Checkpoint 104 Viewer build](checkpoint104-viewer-build.log) | Viewer executable linked | Earlier checkpoint; not proof of matching checkpoint 106 interfaces |
| [Checkpoint 104 Viewer checks](checkpoint104-viewer-tests.log) | 2 of 2 CTest entries passed | Transport and GPU-free UI checks, not actual full-screen visual acceptance |

These populations overlap and must not be added into a fabricated total.
The manifest's checkout HEAD describes the checkout when the archive was made;
it does not identify all uncommitted inputs used by an earlier execution.

The archive does not establish full SIM-M03 completion, all-model acceptance,
installed-consumer acceptance, equipment calibration or a pushed delivery.
Original failures remain intact.

Return to the [persistent project research index](../../../../docs/research/sim-m03/README.md)
or the [read-only diagnostic presentation guide](../../../../docs/research/sim-m03/runtime-observation-guide.md).
