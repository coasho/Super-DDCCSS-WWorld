"""The signal-level radar's cross-check against FERS (ADR-30 MS-6; docs/sdk/mission-systems.md#signal-level-ms-6).

FERS (the Flexible Extensible Radar Simulator, GPL-2.0: https://github.com/davidbits/FERS) is run here as a separate
program - nothing of it is linked - on one coherent interval of a pulsed radar at 10 GHz: 10 kHz PRF, 32 pulses, a
point target of 1 m^2 moving along the line of sight, receiver noise at 290 K. Two pulses: a plain 1 us pulse, and a
Barker-13 phase code of 1 us chips (13 us, compressed to 1 us). FERS's raw samples (10 MHz) are put through what a
receiver does with them - blanked while it transmits, matched filtered (at the target's Doppler), sampled a chip apart (the range gates),
windowed (4-term Blackman-Harris) and Fourier transformed across the pulses (the Doppler filters) - and the cells about
the target written out, each over the noise a cell has (kTB through the same processing; FERS's noise alone is run
too, and its cells' mean and correlation recorded).

tests/simulation/test_signal.cpp ("... against FERS") gives the platform's chain (src/simulation/mission_systems/radar/Signal, which never forms the
samples: it writes each filter's spectrum in closed form, a compressed pulse's range response a triangle) the same
target, its SNR from the same radar equation, and compares the cells: the range straddle, the Doppler scalloping, both
folded several times over, the eclipse where the echo meets the next transmission. The plain pulse's cells are what the
chain's assumptions give exactly; the Barker code's differ by its range sidelobes (1/13 in amplitude at every gate the
code's correlation reaches), which the chain leaves out - recorded, and compared within them. A phase code's compression
not matched to the Doppler loses more (sinc^2 of the cycles the Doppler turns in the pulse) and its sidelobes rise: the
chain takes a radar's compression as tolerant of Doppler, and both are recorded too (`uncompensated`).

    py -3.13 tools/validation/fers/crosscheck.py <fers-cli.exe> [--keep <dir>]     (needs numpy and h5py; FERS's DLLs on PATH)

writes tests/simulation/data/signal/fers_crosscheck.json. FERS gives each pulse's echo to the receive window open when the pulse
goes out, so a radar's own windows would lose its echoes from beyond the first unambiguous range: the scene has a
transmitter and, beside it, a receiver whose one window spans every pulse, cut into pulse repetition intervals here.
"""
import glob
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

import h5py
import numpy as np

C = 299792458.0
FC = 10e9
LAMBDA = C / FC
PRF = 10e3
PRI = 1.0 / PRF
PULSES = 32                       # in the interval (its Doppler filters)
CHIP = 1e-6
RATE = 10e6                       # FERS's samples
SPC = int(round(RATE * CHIP))     # samples a chip (10)
SPP = int(round(RATE * PRI))      # samples a pulse repetition interval (1000)
GATE_M = C * CHIP / 2.0           # a gate: a chip's range
RA = C / (2.0 * PRF)              # unambiguous range (100 gates)
GATES = SPP // SPC                # 100
LEAD = 7                          # pulses before the interval (echoes from as far as 6 unambiguous ranges back fill it)
NOISE_INTERVALS = 8               # the noise alone: its statistics over as many intervals
PAD = 4                           # zero samples after the pulse in its file: FERS renders a pulse in whole samples from its
                                  # fractional start and, for some pulses, drops its last - a zero, padded
ENERGY_J = 1.3e7                  # a pulse's (not a radar's: the cells 5 to 40 dB over the noise, where the chain's
                                  # average is sure)
RCS_M2 = 1.0
NOISE_K = 290.0
BOLTZMANN = 1.3806503e-23         # (FERS's value)
WINDOW = (0.35875, 0.48829, 0.14128, 0.01168)

CODES = {"plain": [1.0], "barker13": [1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1]}

# the cases: folded range in gates and Doppler in filters (nu = filter / PULSES of the PRF, positive closing), and how
# many unambiguous ranges and Doppler bands each is from the first
CASES = {
    "plain": [
        dict(name="centred", folds=2, gate=40.0, filter=10.0, wraps=2),
        dict(name="straddled", folds=2, gate=40.5, filter=10.5, wraps=2),
        dict(name="opening", folds=5, gate=70.25, filter=-3.3, wraps=-1),
        dict(name="end eclipsed", folds=3, gate=99.4, filter=20.0, wraps=1),
        dict(name="deep fold", folds=6, gate=22.7, filter=25.6, wraps=4),
    ],
    "barker13": [
        dict(name="centred", folds=2, gate=40.0, filter=10.0, wraps=2),
        dict(name="straddled", folds=2, gate=40.5, filter=10.5, wraps=2),
        dict(name="end eclipsed", folds=3, gate=95.0, filter=20.0, wraps=1),
    ],
}


def window(n):
    k = np.arange(n)
    return (WINDOW[0] - WINDOW[1] * np.cos(2 * np.pi * k / n) + WINDOW[2] * np.cos(4 * np.pi * k / n)
            - WINDOW[3] * np.cos(6 * np.pi * k / n))


def kinematics(case):
    """The case's range at the interval's middle, and its range rate (positive opening)."""
    nu = case["filter"] / PULSES + case["wraps"]          # cycles a pulse, unfolded: nu = -2 rate / (lambda PRF)
    return case["folds"] * RA + case["gate"] * GATE_M, -nu * LAMBDA * PRF / 2.0


def scenario(path, code, case, noise_k, pulses):
    """FERS's XML for a case (None: no target)."""
    span = pulses * PRI          # the receiver's one window
    end = span + 0.5 * PRI
    power = ENERGY_J / (len(code) * CHIP)  # (the same energy for either pulse)
    target = ""
    if case is not None:
        r_mid, rate = kinematics(case)
        t_mid = (LEAD + PULSES / 2.0) * PRI
        r0, r1 = r_mid - rate * t_mid, r_mid + rate * (end - t_mid)
        target = f"""
    <platform name="TargetPlatform">
        <motionpath interpolation="linear">
            <positionwaypoint><x>{r0:.9f}</x><y>0</y><altitude>0</altitude><time>0</time></positionwaypoint>
            <positionwaypoint><x>{r1:.9f}</x><y>0</y><altitude>0</altitude><time>{end:.9f}</time></positionwaypoint>
        </motionpath>
        <fixedrotation><startazimuth>0</startazimuth><startelevation>0</startelevation><azimuthrate>0</azimuthrate><elevationrate>0</elevationrate></fixedrotation>
        <target name="Target"><rcs type="isotropic"><value>{RCS_M2}</value></rcs></target>
    </platform>"""
    xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<simulation name="crosscheck">
    <parameters>
        <starttime>0.0</starttime>
        <endtime>{end:.9f}</endtime>
        <rate>{RATE}</rate>
        <simSamplingRate>1e6</simSamplingRate>
        <randomseed>12345</randomseed>
    </parameters>
    <waveform name="Pulse">
        <power>{power!r}</power>
        <carrier_frequency>{FC}</carrier_frequency>
        <pulsed_from_file filename="pulse.h5"/>
    </waveform>
    <timing name="Clock"><frequency>{FC}</frequency></timing>
    <antenna name="Iso" pattern="isotropic"><efficiency>1.0</efficiency></antenna>
    <platform name="RadarPlatform">
        <motionpath interpolation="static">
            <positionwaypoint><x>0</x><y>0</y><altitude>0</altitude><time>0</time></positionwaypoint>
        </motionpath>
        <fixedrotation><startazimuth>0</startazimuth><startelevation>0</startelevation><azimuthrate>0</azimuthrate><elevationrate>0</elevationrate></fixedrotation>
        <transmitter name="Tx" waveform="Pulse" antenna="Iso" timing="Clock">
            <pulsed_mode><prf>{PRF}</prf></pulsed_mode>
        </transmitter>
        <receiver name="Radar" antenna="Iso" timing="Clock" nodirect="true">
            <pulsed_mode>
                <prf>{0.5 / span:.9f}</prf>
                <window_skip>0</window_skip>
                <window_length>{span:.9f}</window_length>
            </pulsed_mode>
            <noise_temp>{noise_k}</noise_temp>
        </receiver>
    </platform>{target}
</simulation>
"""
    with open(path, "w") as f:
        f.write(xml)
    return power


def run(fers, code, case, noise_k, work, intervals=1):
    for f in glob.glob(os.path.join(work, "*.h5")):
        if os.path.basename(f) != "pulse.h5":
            os.remove(f)
    with h5py.File(os.path.join(work, "pulse.h5"), "w") as f:
        chips = np.concatenate([np.repeat(np.asarray(code, dtype=float), SPC), np.zeros(PAD)])
        f.create_group("I").create_dataset("value", data=chips)
        f.create_group("Q").create_dataset("value", data=np.zeros_like(chips))
    pulses = LEAD + PULSES * intervals + 1
    power = scenario(os.path.join(work, "scenario.fersxml"), code, case, noise_k, pulses)
    done = subprocess.run([fers, "scenario.fersxml", "--log-level=WARNING"], cwd=work, capture_output=True, text=True)
    outputs = [f for f in glob.glob(os.path.join(work, "*.h5")) if os.path.basename(f) != "pulse.h5"]
    if done.returncode != 0 or len(outputs) != 1:
        raise SystemExit("FERS failed (%s):\n%s%s" % (outputs, done.stdout, done.stderr))
    with h5py.File(outputs[0], "r") as h:
        chunks = sorted(k[:-2] for k in h.keys() if k.startswith("chunk_") and k.endswith("_I"))
        i, q = h[chunks[0] + "_I"], h[chunks[0] + "_Q"]
        samples = (np.asarray(i, dtype=float) + 1j * np.asarray(q, dtype=float)) * float(i.attrs["fullscale"])
    if len(chunks) != 1 or len(samples) < pulses * SPP:
        raise SystemExit("FERS's windows: %d, %d samples" % (len(chunks), len(samples)))
    return samples[:pulses * SPP], power


def process(samples, code, doppler_hz=0.0, interval=0):
    """A receiver's processing: blanked while it transmits, matched filtered (every sample: aligned to the echo's
    start; the reference turned at `doppler_hz`, a compression matched to the target's Doppler), a sample a gate, the
    window and the DFT across the interval's pulses. [gate, filter]."""
    x = samples.copy()
    on = np.zeros(SPP, dtype=bool)
    on[:SPC * len(code)] = True
    x[np.tile(on, len(x) // SPP)] = 0.0
    ref = np.repeat(np.asarray(code, dtype=float), SPC) * np.exp(-2j * np.pi * doppler_hz * np.arange(SPC * len(code)) / RATE)
    y = np.zeros(len(x) - len(ref), dtype=complex)
    for j, c in enumerate(ref):
        y += c * x[j:j + len(y)]
    first = (LEAD + interval * PULSES) * SPP
    cube = np.array([[y[first + p * SPP + g * SPC] for g in range(GATES)] for p in range(PULSES)])  # [pulse, gate]
    return np.fft.fft(cube * window(PULSES)[:, None], axis=0).T  # [gate, filter]


def noise_cell_w(code):
    """kTB through the processing: the matched filter's samples' sum, the window's squares."""
    return BOLTZMANN * NOISE_K * RATE * SPC * len(code) * float(np.sum(window(PULSES) ** 2))


def echo_power(samples, range_m, code):
    """The mean power of the interval's middle pulse's echo, inside it (its edges' band-limited ripple left out)."""
    start = (LEAD + PULSES // 2) * SPP + int(round(2.0 * range_m / C * RATE))
    return float(np.mean(np.abs(samples[start + 3:start + SPC * len(code) - 3]) ** 2))


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    fers = os.path.abspath(sys.argv[1])
    keep = sys.argv[sys.argv.index("--keep") + 1] if "--keep" in sys.argv else None
    work = keep or tempfile.mkdtemp(prefix="fers_")
    os.makedirs(work, exist_ok=True)
    result = dict(
        source="FERS 0.1.0 (github.com/davidbits/FERS, 90a0cb2), run by tools/validation/fers/crosscheck.py",
        carrier_hz=FC, prf_hz=PRF, pulses=PULSES, chip_s=CHIP, sample_rate_hz=RATE, pulse_energy_j=ENERGY_J,
        rcs_m2=RCS_M2, noise_k=NOISE_K, boltzmann=BOLTZMANN, gate_m=GATE_M, gates=GATES, codes=[])
    for name, code in CODES.items():
        unit = noise_cell_w(code)
        raw, _ = run(fers, code, None, NOISE_K, work, NOISE_INTERVALS)
        # the receive gates whose matched filter ends before the next transmission (the last len(code) - 1 reach into
        # it, blanked: less noise - and less echo, the eclipse - there)
        specs = [process(raw, code, 0.0, i) for i in range(NOISE_INTERVALS)]
        clear = [sp[len(code):GATES - len(code) + 1, :] for sp in specs]
        noise = dict(raw_sample_w=float(np.mean(np.abs(raw) ** 2)), ktb_w=BOLTZMANN * NOISE_K * RATE, intervals=NOISE_INTERVALS,
                     cell_mean=float(np.mean([np.mean(np.abs(sp) ** 2) for sp in clear])) / unit,
                     adjacent_filter_correlation=float(np.mean([np.real(np.mean(sp[:, :-1] * np.conj(sp[:, 1:]))) for sp in clear])) / unit,
                     last_gate_mean=float(np.mean([np.mean(np.abs(sp[GATES - 1, :]) ** 2) for sp in specs])) / unit)
        entry = dict(code=name, chips=len(code), pulse_s=len(code) * CHIP, blanked_gates=len(code), noise=noise, cases=[])
        print("%s: noise a raw sample %.4e W (kTB %.4e W), a cell %.3f of kTB's (the last gate %.3f), adjacent filters' "
              "correlation %.3f" % (name, noise["raw_sample_w"], noise["ktb_w"], noise["cell_mean"], noise["last_gate_mean"],
                                    noise["adjacent_filter_correlation"]))
        for case in CASES[name]:
            sig, power = run(fers, code, case, 0.0, work)
            r_mid, rate = kinematics(case)
            doppler = -2.0 * rate / LAMBDA
            plain = np.abs(process(sig, code)) ** 2 / unit  # (the compression not turned: the code's Doppler loss)
            cells = np.abs(process(sig, code, doppler)) ** 2 / unit
            g0 = int(math.floor(case["gate"]))
            k0 = int(math.floor(case["filter"])) % PULSES
            gates = [g for g in range(g0 - 1, g0 + 3) if len(code) <= g < GATES]
            mask = np.ones_like(cells, dtype=bool)
            mask[:len(code), :] = False
            elsewhere = cells.copy()
            elsewhere[g0:g0 + 2, :] = 0.0
            entry["cases"].append(dict(
                name=case["name"], range_m=r_mid, range_rate_ms=rate, gates=gates, first_filter=(k0 - 4) % PULSES,
                cells=[[float(cells[g, (k0 + dk) % PULSES]) for dk in range(-4, 5)] for g in gates],
                peak=float(cells[mask].max()), peak_at=[int(v) for v in np.unravel_index(np.argmax(np.where(mask, cells, 0)), cells.shape)],
                elsewhere_max=float(elsewhere[mask].max()),
                uncompensated=dict(peak=float(plain[mask].max()), adjacent_gates=float(max(plain[g, k0] for g in (g0 - 1, g0 + 2) if len(code) <= g < GATES))),
                echo_w=echo_power(sig, r_mid, code),
                radar_equation_w=power * RCS_M2 * LAMBDA ** 2 / ((4 * math.pi) ** 3 * r_mid ** 4)))
            c = entry["cases"][-1]
            print("  %-13s peak %5.1f dB at gate %d filter %d; beyond its two gates %5.1f dB; uncompensated: peak %5.1f dB, the "
                  "gates either side %5.1f dB; echo %.4e W, the radar equation's %.4e W" % (
                      case["name"], 10 * math.log10(c["peak"]), *c["peak_at"], 10 * math.log10(max(c["elsewhere_max"], 1e-30)),
                      10 * math.log10(c["uncompensated"]["peak"]), 10 * math.log10(max(c["uncompensated"]["adjacent_gates"], 1e-30)),
                      c["echo_w"], c["radar_equation_w"]))
        result["codes"].append(entry)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "tests", "simulation", "data", "signal", "fers_crosscheck.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="\n") as f:
        json.dump(result, f, indent=1)
        f.write("\n")
    print("wrote", os.path.normpath(out))
    if not keep:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    main()
