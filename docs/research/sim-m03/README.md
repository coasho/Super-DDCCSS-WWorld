# SIM-M03 research, implementation and products

This is the persistent project entry point for SIM-M03 work across sessions.
Research and implementation records belong here, not only in a chat or a build
directory. Maintained artifacts are English.

## Current implementation assignment

The user requires working production mechanisms and model fidelity. Describing a
missing function or labeling it uncalibrated does not implement it. The active
continuation implements radiometric IR/IIR image formation and tracking, coupled
strapdown inertial/GNSS navigation, RF antenna/clutter detection processing, and
source-backed weapon/carrier and actuation completeness. The preceding bounded
engineering implementation and its audits are a baseline, not final acceptance.

The SDK remains the simulation authority. Existing public layouts, vtables,
numeric identifiers and retained provider behavior remain compatible through
additive private/versioned interfaces. Work stays on main.

## Research and baseline products

- [Sensing implementation and primary references](baseline/sensing-reference-map.md)
- [Sensing values, source versions and numerical comparisons](baseline/sensing-reference-map.json)
- [Warhead, damage and component implementation references](baseline/effects-reference-map.md)
- [Effects source/evidence inventory](baseline/effects-reference-map.json)
- [24-model capability and recorded-flight atlas](baseline/model-capability-atlas.md)
- [Model atlas machine-readable data](baseline/model-capability-atlas.json)
- [Model atlas CSV](baseline/model-capability-atlas.csv)
- [Model atlas figure](baseline/model-capability-atlas.png)

These baseline records preserve their original source and runtime hashes. They
identify concrete model-order gaps being implemented; they do not certify newer
source bytes, hardware accuracy, or complete SIM-M03 acceptance.

Original source extracts, licenses and reference scripts are retained under
`artifacts/sim-m03/research/`. Exact native campaign logs and recordings initially
reside in `build/architecture-reviews/sim-m03-complete-20261102/`; the delivery
index will bind the final retained products under `artifacts/sim-m03/` before
handoff. Interrupted source exports A-G and failures are diagnostic evidence,
never counted as completed acceptance.

## Required production completion

See [the implementation continuation](../../superpowers/plans/2026-11-02-sim-m03-physical-completion.md).
Each completed mechanism must link its production symbols, physical method,
per-model parameters, reproducible normal entry point and actual observations.
Verification accompanies the implementation; it does not replace it.


## Physical components and runtime explanation

- [Reading native observations, units, availability and replay](runtime-observation-guide.md)
- [Archived build and focused-check evidence](../../../artifacts/sim-m03/verification/checkpoint106/README.md)
- [Actual IR/IIR scene, detector and image extraction](../../sdk/weapon-thermal-image.md)
- [Actual RF aperture, range-Doppler and detection processing](../../sdk/weapon-radio-receiver.md)
- [Body-IMU strapdown and coupled GNSS estimation](../../sdk/weapon-navigation-strapdown-v2.md)
- [Independent navigation equations and statistical oracle](strapdown-navigation-method.md)
- [Native component diagnostics and companion recording](../../sdk/weapon-diagnostics-v1.md)
- [Finite fuel-transit sources and reduced equations](fuel-transit-method.md)
- [Capacity-conserving liquid occupancy and geometric uncertainty](liquid-occupancy-method.md)
- [Chronological liquid scene point/rod methods](liquid-scene-method.md)
- [Actual native contents, detached ownership and binding gates](native-liquid-binding.md)

The Viewer component window consumes native data. It must expose actual image
pixels and detector units, antenna/CFAR cells, navigation covariance and bias,
finite control/propulsion state, causal fuze/fragment/damage events and sample
ages. Presentation code never computes substitute simulation outcomes.

Focused native observations and exact source hashes: [checkpoint89](../../../artifacts/sim-m03/physical-components/checkpoint89/execution.json). The retained earlier failure shows the original two-source detector predicate; its correction is in the actual image processor. This record does not replace final normal-entrypoint integration evidence.
