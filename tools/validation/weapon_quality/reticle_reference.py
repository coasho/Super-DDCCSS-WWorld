"""Analytic con-scan reference, independent of native rendering and tracking.

The100Hz/12-period primary bench describes the published SA-14 model, not
the hardware constants of any maintained AAM. Production design assumptions
are explicit, separately derived from the admitted optical/electronic policy.
"""
import argparse
import json
import math
import csv
import hashlib
from pathlib import Path
from acceptance import allowed


def positive(value):
    if not math.isfinite(value) or value <= 0:
        raise ValueError('Positive finite reticle quantity required')
    return value


def carrier_hz(frequency, periods):
    positive(frequency)
    if not isinstance(periods, int) or not 1 <= periods <= 128:
        raise ValueError('Bounded positive integer reticle period count required')
    return frequency*periods


def angular_speed(point, radius, phase, frequency):
    """Exact derivative of arg(a+r exp(i phi)); no finite differencing."""
    positive(radius)
    positive(frequency)
    x, y = point
    if not all(map(math.isfinite, (x, y, phase))):
        raise ValueError('Finite focal-plane position and scan phase required')
    projection = x*math.cos(phase)+y*math.sin(phase)
    denominator = x*x+y*y+2*radius*projection+radius*radius
    if denominator <= 0:
        raise ValueError('Reticle angle is undefined at its centre')
    return 2*math.pi*frequency*radius*(radius+projection)/denominator


def mask(point, phase, radius, stop_radius, periods):
    carrier_hz(1, periods)
    positive(radius)
    positive(stop_radius)
    x = point[0]+radius*math.cos(phase)
    y = point[1]+radius*math.sin(phase)
    if not all(map(math.isfinite, (x, y))):
        raise ValueError('Finite mask position required')
    if x*x+y*y > stop_radius*stop_radius:
        return 0.
    return float(math.cos(periods*math.atan2(y, x)) >= 0)


def electrons(power_w, exposure_s, efficiency, wavelength_m):
    positive(exposure_s)
    positive(wavelength_m)
    if not math.isfinite(power_w) or power_w < 0 or not 0 < efficiency <= 1:
        raise ValueError('Finite nonnegative radiant power and efficiency required')
    return power_w*exposure_s*efficiency*wavelength_m/(6.62607015e-34*299792458.)


def response(previous, incident, elapsed, time_constant):
    positive(elapsed)
    positive(time_constant)
    return previous+(incident-previous)*(-math.expm1(-elapsed/time_constant))


def design(width, height, pitch, sigma, response_s):
    positive(pitch)
    positive(sigma)
    positive(response_s)
    if min(width, height) < 8:
        raise ValueError('Finite optical field required')
    stop = min(width, height)*pitch/2
    radius = stop/2
    periods = max(1, math.floor(math.pi*radius/(8*sigma)))
    if periods > 128:
        raise ValueError('Reticle resolution exceeds numerical domain')
    return {'basis': 'assumed', 'stop_radius_rad': stop, 'scan_radius_rad': radius,
            'periods': periods, 'frequency_hz': 1/(2*math.pi*periods*response_s)}


def scalar_electronics(width, height, pixel_well, pixel_dark, pixel_read_noise, correlation=0):
    """Area/capacitance inference; correlation is sensitivity, not calibration."""
    area = width*height
    if area <= 0 or not 0 <= correlation <= 1:
        raise ValueError('Finite positive detector area and correlation domain required')
    positive(pixel_well)
    if not all(map(math.isfinite, (pixel_dark, pixel_read_noise))) or min(pixel_dark, pixel_read_noise) < 0:
        raise ValueError('Finite nonnegative reference charge required')
    return dict(full_well=area*pixel_well, dark_current=area*pixel_dark,
                read_noise=pixel_read_noise*math.sqrt(area*(1-correlation)+area*area*correlation))


def primary_bench():
    samples = []
    for point in ((0., 0.), (.002, -.003), (-.004, .001)):
        values = [angular_speed(point, .024, 2*math.pi*j/4096, 100) for j in range(4096)]
        samples.append({'point_rad': list(point), 'mean_angular_speed_rad_s': math.fsum(values)/len(values),
                        'mean_chopping_hz': 12*math.fsum(values)/(2*math.pi*len(values))})
    return {'schema': 'fsim.thermal-reticle-analytic-reference.v1',
            'primary': {'title': 'Modelling an infrared Man Portable Air Defence System',
                        'url': 'https://doi.org/10.1016/j.infrared.2010.07.001',
                        'locator': 'Tracking loop and pre-amplifier sections; SA-14 model only'},
            'reference_frequency_hz': 100, 'reference_periods': 12,
            'expected_carrier_hz': 1200, 'samples': samples,
            'independent_area_noise': scalar_electronics(64, 64, 1e6, 1, 12),
            'fully_correlated_noise_sensitivity': scalar_electronics(64, 64, 1e6, 1, 12, 1),
            'equipment_calibration': False}


def compare_native_waveform(rows):
    """Numeric comparison only; native execution/provenance is a separate gate.

    The actual native fixture freezes100Hz,12 periods,128 samples,100us pole
    and the declared equivalent-area ADC. No waveform is manufactured here.
    """
    if len(rows) != 128:
        raise ValueError('Complete128-sample actual waveform required')
    phases, incident, expected, measured = [], [], [], []
    for index, row in enumerate(rows):
        values = [float(row[key]) for key in ('phase_rad', 'incident_electrons', 'expected_electrons', 'measured_electrons')]
        if not all(map(math.isfinite, values)) or min(values[1:]) < 0:
            raise ValueError('Finite physical charge required')
        if abs(values[0]-2*math.pi*(index+.5)/128) > 1e-14:
            raise ValueError('Foreign scan phase chronology')
        for column, value in zip((phases, incident, expected, measured), values):
            column.append(value)
    amplitudes = {harmonic: abs(sum(value*complex(math.cos(harmonic*phase), math.sin(harmonic*phase))
                                  for value, phase in zip(incident, phases))) for harmonic in (11, 12, 13)}
    if not amplitudes[12] > max(amplitudes[11], amplitudes[13]):
        raise ValueError('Primary1200Hz carrier absent from actual photons')
    prior = 0.
    prior_time = 0.
    maximum_error = 0.
    adc = 4096*1e6/65535
    for index, value in enumerate(incident):
        time = .01+(index+.5)/(100*128)
        prior = response(prior, value, time-prior_time, .0001)
        error = abs(expected[index]-prior)
        if error > 1e-11*max(1., abs(prior)):
            raise ValueError('Native charge response differs from independent exact pole')
        maximum_error = max(maximum_error, error)
        code = measured[index]/adc
        if not 0 <= measured[index] <= 4096*1e6 or abs(code-round(code)) > 1e-9:
            raise ValueError('Native scalar charge is not the declared bounded ADC output')
        prior_time = time
    return {'passed': True, 'samples': 128, 'reference_carrier_hz': 1200,
            'carrier_amplitudes': amplitudes, 'maximum_response_error_electrons': maximum_error,
            'response_relative_tolerance': 1e-11, 'execution_admission': 'separate_current_native_run_receipt_required',
            'equipment_calibration': False}


MODEL_SCOPES = {'aim-9m': {'base_parameters_sha256': '11ace01ba8d71bdb00a177c58ec31fa7b3eeec79113eb2a7f61fa6eaf760eea3', 'reference_scope': {'url': 'https://www.af.mil/About-Us/Fact-Sheets/Display/Article/104557/aim-9-sidewinder/', 'locator': 'Background, L/M paragraphs', 'fact': 'AIM-9L conical scan and AIM-9M L heritage/background discrimination; exact M mask/electronics not disclosed.'}}, 'magic-2': {'base_parameters_sha256': 'e8dadfbb8e5d17cbd120d9238be2f1602157ef4af757ed1ba8ab78a5bedf3ab5', 'reference_scope': {'url': 'https://www.irsem.fr/storage/file_manager_files/2025/03/missiles-tactiques-comaero.pdf', 'locator': 'Printed147, Magic2', 'fact': 'Historical multi-element IR description. A scalar reticle is an explicit approximation, not evidence of the actual detector arrangement.'}}, 'r-60m': {'base_parameters_sha256': 'f3364d7ab499c73d5cb1522abfc539e727c30c72dd7a107008a796f5edeab2bd', 'reference_scope': {'url': 'https://djvu.online/file/fUvs0bhsAlgza', 'locator': 'Polish R-60MK manual Lot2821/89,1990,1.3 and3.3', 'fact': 'Related MK thermal head; exact M scan/reticle cannot be transferred silently.'}}, 'r-73': {'base_parameters_sha256': '4572557b6b5347dcde6e65e1083f23fba5068819796c34cfa0b075df5ca397fd', 'reference_scope': {'url': 'https://audm.org.ua/en/news/artem-pochne-remontuvati-ta-modernizuvati-raketi-r-73/', 'locator': '2021-04-08, MK-80 manufacturer statement', 'fact': 'Arsenal MK-80 IR seeker identity. Internal detector/mask and variant constants are not established.'}}, 'r-27t': {'base_parameters_sha256': 'b21885732074cfb23dee30cb2222a0f284040420d4095460feb0a635967dba8c', 'reference_scope': {'url': 'https://roe.ru/pdfs/pdf_4714.pdf', 'locator': 'R-27T1/R-27ET1 export family description', 'fact': 'IR guidance for related export family; no exact base-T reticle or scan constant established.'}}, 'r-27et': {'base_parameters_sha256': 'd70e519e00935a1cfab03aa671bd0170ea1e62bdc2551704739d759a25e07f57', 'reference_scope': {'url': 'https://roe.ru/pdfs/pdf_4714.pdf', 'locator': 'R-27T1/R-27ET1 export family description', 'fact': 'IR guidance for related export family; no exact base-ET reticle or scan constant established.'}}, 'pl-8': {'base_parameters_sha256': 'b070ac0a33e78a92cc75595734aaf2320be4a17fa9d27a272e6997f270acf2f4', 'reference_scope': {'url': 'https://military.cctv.com/2025/03/23/ARTIhBJ4g3KjFFJg4PnYWwpg250323.shtml', 'locator': 'Public designation report', 'fact': 'Identity evidence only. Exact optical topology, scan and detector constants remain engineering inference.'}}}


def describe_models(root):
    root = allowed(root)
    rows = []
    for model, scope in MODEL_SCOPES.items():
        path = allowed(root/"assets/stores"/(model+".json"))
        data = path.read_bytes()
        card = json.loads(data)
        descriptor = card["component_models"]["components"]["seeker"]
        original = {key: value for key, value in card["parameters"].items() if not key.startswith("receiver_reticle_")}
        digest = lambda value: hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode()).hexdigest()
        if digest(original) != scope["base_parameters_sha256"]:
            raise ValueError("Original parameter record changed: "+model)
        if descriptor["bindings"]["receiver_model"] != "thermal_conscan_reticle.v1" or card["seeker"] != "infrared":
            raise ValueError("Actual reticle mechanism identity differs")
        parameters = {role: card["parameters"][reference] for role, reference in descriptor["parameter_refs"].items()}
        reticle = {key: value for key, value in card["parameters"].items() if key.startswith("receiver_reticle_")}
        if len(reticle) != 6 or any(value.get("source") not in card["sources"] or value.get("basis") != "assumed" or value.get("uncertainty", 0) <= 0 for value in reticle.values()):
            raise ValueError("Complete explicit inferred reticle parameter provenance required")
        rows.append(dict(model=model, card_sha256=hashlib.sha256(data).hexdigest(), parameter_sha256=digest(dict(descriptor=descriptor, parameters=parameters)), implementation_id=descriptor["implementation_id"], receiver_model=descriptor["bindings"]["receiver_model"], prior_receiver_model="thermal_quadrant.v1", original_parameters_sha256=scope["base_parameters_sha256"], all_original_parameter_records_unchanged=True, reference_scope=scope["reference_scope"], reticle_parameters=reticle, theory_delta="Actual photon-mask nutation and scalar waveform fitting replace four-charge Gaussian inversion. Exact OEM topology and calibration remain unverified.", native_verification="pending", equipment_calibration=False))
    return dict(schema="fsim.thermal-reticle-models.v1", state="implementation_authored_native_verification_pending", shared_policy="thermal_conscan_reticle.v1", shared_equivalence="Identical declared scalar receiver algorithm; not OEM hardware equivalence or empirical calibration.", models=rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--native-waveform', type=Path)
    parser.add_argument('--describe-models', action='store_true')
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[3])
    args = parser.parse_args()
    output = allowed(args.output)
    document = describe_models(allowed(args.root)) if args.describe_models else primary_bench()
    if args.native_waveform:
        native = allowed(args.native_waveform)
        with native.open(newline='', encoding='utf-8') as stream:
            document['native_numeric_comparison'] = compare_native_waveform(list(csv.DictReader(stream)))
        document['native_waveform'] = {'path': str(native), 'sha256': hashlib.sha256(native.read_bytes()).hexdigest()}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2, allow_nan=False)+'\n', encoding='utf-8')
