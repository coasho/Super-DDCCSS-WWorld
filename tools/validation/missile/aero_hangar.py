"""Read generated Hangar static expressions for an offline diagnostic comparison.

This bounded expression reader does not run a flight model or generate tables.
Its full static basis includes empirical crossflow and axial viscous estimates;
comparison with Euler/panel coefficients does not admit any physical channel.
"""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def _finite(value):
    try:
        number = float(value)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError('expected finite numeric static-model input') from error
    if isinstance(value, bool) or not math.isfinite(number):
        raise ValueError('expected finite numeric static-model input')
    return number


def _table(node, properties):
    if [child.tag for child in node] != ['independentVar', 'tableData']:
        raise ValueError('only one-dimensional generated Mach tables are supported')
    variable, data = list(node)
    if (variable.text or '').strip() != 'velocities/mach' or variable.attrib not in ({}, {'lookup': 'row'}):
        raise ValueError('unsupported static table coordinate')
    rows = [line.split() for line in (data.text or '').splitlines() if line.strip()]
    if not 2 <= len(rows) <= 1024 or any(len(row) != 2 for row in rows):
        raise ValueError('invalid generated Mach table shape')
    rows = [tuple(_finite(value) for value in row) for row in rows]
    if any(mach < 0 or (i and mach <= rows[i-1][0]) for i, (mach, _) in enumerate(rows)):
        raise ValueError('Mach coordinates must increase strictly')
    mach = properties['velocities/mach']
    if not rows[0][0] <= mach <= rows[-1][0]:
        raise ValueError('comparison cannot extrapolate a generated Mach table')
    for (left, a), (right, b) in zip(rows, rows[1:]):
        if left <= mach <= right:
            fraction = _finite(_finite(mach-left)/_finite(right-left))
            return _finite(a + _finite(_finite(b-a)*fraction))
    raise ValueError('missing table interval')


def _expression(node, properties, depth=0):
    if depth > 32:
        raise ValueError('static expression nesting limit exceeded')
    children = list(node)
    if node.tag == 'value' and not children and not node.attrib:
        return _finite(node.text)
    if node.tag == 'property' and not children and not node.attrib:
        key = (node.text or '').strip()
        if key not in properties:
            raise ValueError('unsupported static-model property: ' + key)
        return properties[key]
    if node.tag == 'table' and not node.attrib:
        return _table(node, properties)
    if node.attrib or node.tag not in ('product', 'sum', 'sin', 'cos', 'abs'):
        raise ValueError('unsupported generated static expression: ' + node.tag)
    if not children or (node.tag in ('sin', 'cos', 'abs') and len(children) != 1):
        raise ValueError('invalid generated expression arity')
    values = [_expression(child, properties, depth+1) for child in children]
    try:
        if node.tag == 'product':
            value = math.prod(values)
        elif node.tag == 'sum':
            value = math.fsum(values)
        else:
            value = {'sin': math.sin, 'cos': math.cos, 'abs': abs}[node.tag](values[0])
    except (ValueError, OverflowError) as error:
        raise ValueError('static expression overflow') from error
    return _finite(value)


def _unique(parent, query):
    nodes = parent.findall(query)
    if len(nodes) != 1:
        raise ValueError('missing or ambiguous generated field: ' + query)
    return nodes[0]


def assess_initial(xml_path, rows, reference):
    """Compare exact emitted full static coefficients at each supplied condition.

    References must match generated metrics within their emitted rounding.
    The generated-origin coefficients remain available separately; pitch is
    transported from its rounded AERORP to the requested solver moment origin.
    This intentionally does not convert full empirical drag/crossflow into an
    inviscid validation target. Every difference is diagnostic and unaccepted.
    """
    path = Path(xml_path)
    if path.stat().st_size > 2 * 1024 * 1024:
        raise ValueError('generated static model exceeds the 2 MiB limit')
    raw = path.read_bytes()
    if not raw or len(raw) > 2 * 1024 * 1024:
        raise ValueError('unsupported empty/oversized generated XML')
    try:
        text = raw.decode('utf-8-sig')
    except UnicodeError as error:
        raise ValueError('generated static XML must be UTF-8') from error
    # NUL is forbidden XML text; rejecting it also prevents the XML parser
    # from reinterpreting decoded UTF-16-without-BOM bytes as another encoding.
    if '\x00' in text:
        raise ValueError('generated static XML must be UTF-8 without NUL characters')
    # Screen decoded text so another encoding cannot hide a declaration.
    # Comments are ordinary generated provenance; declarations/entities are not.
    if '<!DOCTYPE' in text or '<!ENTITY' in text:
        raise ValueError('declaration-bearing generated XML is unsupported')
    try:
        tree = ET.fromstring(text)
    except ET.ParseError as error:
        raise ValueError('malformed generated static XML') from error
    if tree.tag != 'fdm_config' or sum(1 for _ in tree.iter()) > 8192:
        raise ValueError('unsupported or oversized generated static expression tree')
    metrics = _unique(tree, 'metrics')
    actual = {}
    for key, tag, unit, scale in (('area_m2', 'wingarea', 'FT2', .3048**2),
                                  ('length_m', 'chord', 'FT', .3048)):
        node = _unique(metrics, tag)
        if node.attrib != {'unit': unit}:
            raise ValueError('unsupported generated metric units')
        actual[key] = _finite(_finite(node.text) * scale)
        expected = _finite(reference[key])
        # The standard generator emits these metrics with five decimal places.
        if expected <= 0 or abs(_finite(actual[key]-expected)) > .500001e-5*scale:
            raise ValueError('generated reference differs from the solver: ' + key)
    origin = _unique(metrics, "location[@name='AERORP']")
    if origin.attrib != {'name': 'AERORP', 'unit': 'IN'}:
        raise ValueError('unsupported generated moment-reference units')
    xyz = [_finite(_finite(_unique(origin, axis).text)*.0254) for axis in ('x', 'y', 'z')]
    requested_xyz = [_finite(reference['moment_x_m']), 0., 0.]
    delta = [_finite(target-generated) for target, generated in zip(requested_xyz, xyz)]
    if any(abs(value) > .500001e-3*.0254 for value in delta):
        raise ValueError('generated launch-CG moment reference differs from the solver')
    area_ratio = _finite(actual['area_m2'] / _finite(reference['area_m2']))
    reference_length = _finite(reference['length_m'])
    length_ratio = _finite(actual['length_m'] / reference_length)
    expressions = {}
    for key, axis, name in (('ca', 'AXIAL', 'CA'), ('cn', 'NORMAL', 'CN'), ('cm', 'PITCH', 'Cm')):
        function = _unique(tree, ".//function[@name='aero/coefficient/%s']" % name)
        owner = _unique(tree, "aerodynamics/axis[@name='%s']" % axis)
        if function not in list(owner):
            raise ValueError('generated coefficient has the wrong axis')
        children = [child for child in function if child.tag != 'description']
        if len(children) != 1 or len(list(function.iter())) > 512:
            raise ValueError('missing or oversized generated static function')
        normalizers = ['aero/qbar-psf', 'metrics/Sw-sqft'] + (['metrics/cbarw-ft'] if key == 'cm' else [])
        outer = children[0]
        if (outer.tag != 'product' or len(outer) <= len(normalizers)
                or any(child.tag != 'property' or (child.text or '').strip() != name
                       for child, name in zip(outer, normalizers))):
            raise ValueError('generated force/moment normalizers are missing or unsupported')
        expressions[key] = children[0]
    if not isinstance(rows, list) or not 1 <= len(rows) <= 1024:
        raise ValueError('comparison requires a bounded nonempty case list')
    cases = []
    for row in rows:
        if not isinstance(row, dict) or 'mach' not in row or 'alpha_deg' not in row:
            raise ValueError('comparison requires explicit Mach and incidence')
        mach, alpha, beta = (_finite(row.get(key, 0)) for key in ('mach', 'alpha_deg', 'beta_deg'))
        if mach < 0 or beta != 0 or abs(alpha) > 90:
            raise ValueError('static comparison requires supported Mach/alpha and zero sideslip')
        properties = {'aero/qbar-psf': 1., 'metrics/Sw-sqft': 1., 'metrics/cbarw-ft': 1.,
                      'aero/alpha-rad': math.radians(alpha), 'velocities/mach': mach}
        emitted = {key: _expression(node, properties) for key, node in expressions.items()}
        generated = {key: _finite(_finite(value*area_ratio)*(length_ratio if key == 'cm' else 1))
                     for key, value in emitted.items()}
        # Design axes are x aft and z up. For delta=requested-generated,
        # My(requested)=My(generated)+dx*N-dz*A, all normalized to solver refs.
        transport = _finite(_finite(_finite(delta[0]*generated['cn']) -
                                    _finite(delta[2]*generated['ca'])) / reference_length)
        initial = {**generated, 'cm': _finite(generated['cm']+transport)}
        coefficients = row.get('coefficients')
        if not isinstance(coefficients, dict) or any(key not in coefficients for key in initial):
            raise ValueError('comparison requires explicit axial, normal and pitch coefficients')
        selected = {key: _finite(coefficients[key]) for key in initial}
        difference = {}
        for key in initial:
            error = abs(_finite(selected[key]-initial[key]))
            difference[key] = dict(absolute=error, relative=_finite(error/abs(initial[key])) if initial[key] else None)
        cases.append(dict(mach=mach, alpha_deg=alpha, beta_deg=beta, initial_full_static=initial,
                          initial_at_generated_origin=generated, moment_transport_cm=transport,
                          selected_solver=selected, difference=difference))
    return dict(raw_file=dict(path=str(path.resolve()), sha256=hashlib.sha256(raw).hexdigest()),
                reference=actual, moment_reference_xyz_m=xyz, cases=cases,
                generated_moment_reference_xyz_m=xyz, requested_moment_reference_xyz_m=requested_xyz,
                moment_transport=dict(delta_requested_minus_generated_m=delta,
                    reference_length_m=reference_length, formula='(dx*CN - dz*CA)/Cref; design x aft, z up'),
                basis='Emitted Hangar full static terms: axial viscous estimates and empirical crossflow included; rates and controls excluded',
                like_for_like_inviscid=False, model_accepted=False)
