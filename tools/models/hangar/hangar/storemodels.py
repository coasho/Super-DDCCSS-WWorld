"""Physical mass, contact and propulsion shells. Aerodynamics use the shared aircraft VLM writer."""
import json
import math
import os
LB, FT, IN = (0.45359237, 0.3048, 0.0254)
SLUGFT2 = 1.35581795
RING = 6

def _card(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)

def _value(card, name, fallback=None):
    p = card.get('parameters', {}).get(name)
    return p['value'] if p else fallback

def model_name(card):
    """The flight model's name: its card's model without "jsbsim:" (None: it has none)."""
    m = card.get('model', '')
    return m[7:] if m.startswith('jsbsim:') else None

def store_xml(card):
    """The tank/pod mass, contact and fuel shell; aerodynamic XML is shared."""
    if card.get('class') not in ('tank','pod') or not model_name(card):return None
    m=_value(card,'mass_kg')
    length,diameter=_value(card,'length_m'),_value(card,'diameter_m')
    if not (m and length and diameter):return None
    r=.5*diameter;frontal=math.pi*r*r;cg=.5*length;cp=cg;k=1/IN
    ixx=m*r*r;iyy=m*(.5*r*r+length*length/12.)
    full=m+(_value(card,'fuel_kg',0.) or 0.)
    spring,damping=20.*full/LB,2.*full/LB
    def loc(name,x):
        return '<location name="%s" unit="IN"><x>%.17g</x><y>0</y><z>0</z></location>'%(name,x*k)
    def contacts(name,x):
        rows=[]
        for i in range(RING):
            a=2.*math.pi*(i+.5)/RING
            rows.append('<contact type="STRUCTURE" name="%s %d"><location unit="IN"><x>%.17g</x><y>%.17g</y><z>%.17g</z></location><static_friction>0.8</static_friction><dynamic_friction>0.5</dynamic_friction><spring_coeff unit="LBS/FT">%.17g</spring_coeff><damping_coeff unit="LBS/FT/SEC">%.17g</damping_coeff></contact>'%(name,i+1,x*k,r*math.cos(a)*k,r*math.sin(a)*k,spring/RING,damping/RING))
        return '\n'.join(rows)
    return _template(card,locals())


def _template(card, v):
    frontal, diameter, length, cp, cg, ixx, iyy, m, r, k = (v[n] for n in ('frontal', 'diameter', 'length', 'cp', 'cg', 'ixx', 'iyy', 'm', 'r', 'k'))
    loc,contacts=v['loc'],v['contacts']
    return '<?xml version="1.0"?>\n<!-- Written by hangar (tools/hangar/hangar/storemodels.py) from the store card %s: the store flying on its own once it\n     is released or jettisoned (MS-7g). Its aerodynamics use the shared aircraft VLM tables and writer. Do not edit. -->\n<fdm_config name="%s" version="2.0" release="ALPHA">\n    <fileheader>\n        <author> hangar </author>\n        <description> %s, released </description>\n    </fileheader>\n    <metrics>\n        <wingarea unit="FT2"> %.4f </wingarea>\n        <wingspan unit="FT"> %.4f </wingspan>\n        <chord unit="FT"> %.4f </chord>\n        <htailarea unit="FT2"> 0 </htailarea>\n        <htailarm unit="FT"> 0 </htailarm>\n        <vtailarea unit="FT2"> 0 </vtailarea>\n        <vtailarm unit="FT"> 0 </vtailarm>\n%s\n%s\n%s\n    </metrics>\n    <mass_balance negated_crossproduct_inertia="false">\n        <ixx unit="SLUG*FT2"> %.4f </ixx>\n        <iyy unit="SLUG*FT2"> %.4f </iyy>\n        <izz unit="SLUG*FT2"> %.4f </izz>\n        <emptywt unit="LBS"> %.17g </emptywt>\n%s\n        <pointmass name="fuel">\n            <form shape="cylinder">\n                <radius unit="IN"> %.3f </radius>\n                <length unit="IN"> %.3f </length>\n            </form>\n            <weight unit="LBS"> 0 </weight>\n%s\n        </pointmass>\n    </mass_balance>\n    <ground_reactions>\n%s\n%s\n    </ground_reactions>\n    <propulsion/>\n    <flight_control name="none"/>\n\n</fdm_config>\n' % (card['id'], model_name(card), card.get('designation', card['id']), frontal / FT ** 2, diameter / FT, length / FT, loc('AERORP', cp), loc('EYEPOINT', cg), loc('VRP', cg), ixx / SLUGFT2, iyy / SLUGFT2, iyy / SLUGFT2, m / LB, loc('CG', cg), r * 0.95 * k, length * 0.8 * k, '            <location unit="IN"> <x> %.3f </x> <y> 0 </y> <z> 0 </z> </location>' % (cg * k), contacts('nose', 0.1 * length), contacts('tail', 0.9 * length))
def cargo_xml(card):
    """The JSBSim definition of a cargo store on its own (None: not one, or its card lacks its size): a uniform box."""
    if card.get('class') != 'cargo' or not model_name(card):
        return None
    m = _value(card, 'mass_kg')
    L, W, H = (_value(card, 'length_m'), _value(card, 'width_m'), _value(card, 'height_m'))
    if not (m and L and W and H):
        return None
    k = 1.0 / IN
    ixx, iyy, izz = (m * (W * W + H * H) / 12.0, m * (L * L + H * H) / 12.0, m * (L * L + W * W) / 12.0)
    cg = 0.5 * L
    front, side, top = (W * H, L * H, L * W)
    spring, damping = (20.0 * m / LB, 2.0 * m / LB)

    def loc(name):
        return '        <location name="%s" unit="IN"> <x> %.3f </x> <y> 0 </y> <z> 0 </z> </location>' % (name, cg * k)
    corners = []
    for i, x in enumerate((0.0, L)):
        for j, y in enumerate((-0.5 * W, 0.5 * W)):
            for n, z in enumerate((-0.5 * H, 0.5 * H)):
                corners.append('        <contact type="STRUCTURE" name="corner %d">\n            <location unit="IN"> <x> %.3f </x> <y> %.3f </y> <z> %.3f </z> </location>\n            <static_friction> 0.8 </static_friction>\n            <dynamic_friction> 0.6 </dynamic_friction>\n            <spring_coeff unit="LBS/FT"> %.0f </spring_coeff>\n            <damping_coeff unit="LBS/FT/SEC"> %.0f </damping_coeff>\n        </contact>' % (4 * i + 2 * j + n + 1, x * k, y * k, z * k, spring / 4.0, damping / 4.0))
    return '<?xml version="1.0"?>\n<!-- Written by hangar (tools/hangar/hangar/storemodels.py) from the store card %s: the cargo on its own once it has\n     left its aircraft\'s hold. Its aerodynamics use the shared aircraft VLM tables and writer. Do not edit. -->\n<fdm_config name="%s" version="2.0" release="ALPHA">\n    <fileheader>\n        <author> hangar </author>\n        <description> %s, released </description>\n    </fileheader>\n    <metrics>\n        <wingarea unit="FT2"> %.4f </wingarea>\n        <wingspan unit="FT"> %.4f </wingspan>\n        <chord unit="FT"> %.4f </chord>\n        <htailarea unit="FT2"> 0 </htailarea>\n        <htailarm unit="FT"> 0 </htailarm>\n        <vtailarea unit="FT2"> 0 </vtailarea>\n        <vtailarm unit="FT"> 0 </vtailarm>\n%s\n%s\n%s\n    </metrics>\n    <mass_balance negated_crossproduct_inertia="false">\n        <ixx unit="SLUG*FT2"> %.4f </ixx>\n        <iyy unit="SLUG*FT2"> %.4f </iyy>\n        <izz unit="SLUG*FT2"> %.4f </izz>\n        <emptywt unit="LBS"> %.17g </emptywt>\n%s\n    </mass_balance>\n    <ground_reactions>\n%s\n    </ground_reactions>\n    <propulsion/>\n    <flight_control name="none"/>\n\n</fdm_config>\n' % (card['id'], model_name(card), card.get('designation', card['id']), front / FT ** 2, W / FT, L / FT, loc('AERORP'), loc('EYEPOINT'), loc('VRP'), ixx / SLUGFT2, iyy / SLUGFT2, izz / SLUGFT2, m / LB, loc('CG'), '\n'.join(corners))

def files(card):
    """The store's flight model as its files, {path under aircraft/stores/<model>/: text}: a tank's, a pod's and a cargo
    store's definition from its card; a missile's and a bomb's from its design, a missile's with its motor's and its
    nozzle's (storebodies.py). None: its card lacks what it is written from."""
    text = store_xml(card) or cargo_xml(card)
    if text is not None:
        from . import components
        selected = components.resolve(card, 'aerodynamics')
        if selected is not None:
            components._available(selected, components.AERODYNAMICS, (), ('model', 'design', 'source'), card)
            if selected['bindings']['source'] != 'vlm':
                raise ValueError('components: native store aerodynamic source mismatch')
        from . import pipeline, airframe
        from pathlib import Path
        path = Path(pipeline.repo_root()) / 'aircraft' / model_name(card) / (model_name(card) + '.toml')
        text = airframe.install(text, path, card=card)
        return {model_name(card) + '.xml': text}
    from . import storebodies
    return storebodies.files(card)

def write(cards_dir, aircraft_dir):
    """Compatibility entrypoint; publication always uses the normal build stage."""
    from pathlib import Path
    from . import pipeline,storestages
    root=Path(pipeline.repo_root())
    if Path(cards_dir).resolve()!=(root/'assets/stores').resolve() or Path(aircraft_dir).resolve()!=(root/'aircraft/stores').resolve():
        raise ValueError('Store publication uses project-owned normal build paths')
    written=[]
    for name,source in storestages.designs().items():
        d=pipeline.Design(source)
        stage=storestages.build(d)
        written.append(stage['path'])
    return written


def main():
    from . import pipeline
    root = pipeline.repo_root()
    for p in write(os.path.join(root, 'assets', 'stores'), os.path.join(root, 'aircraft', 'stores')):
        print(p)
if __name__ == '__main__':
    main()
