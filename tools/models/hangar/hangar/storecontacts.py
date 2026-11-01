"""All store terrain support uses the common aircraft geometry/contact method.

Use the actual serialized dry tensor for the lightest-state contact modes;
an aerodynamic reference area/chord is never physical collision geometry.
"""
import math
import numpy as np
import xml.etree.ElementTree as ET


def install(xml,aircraft):
    from . import jsbsim
    root=ET.fromstring(xml);balance=root.find('mass_balance')
    def number(node,units):
        if node is None or node.get('unit') not in units:raise ValueError('Missing supported native contact mass unit')
        value=float(node.text)*units[node.get('unit')]
        if not math.isfinite(value):raise ValueError('Nonfinite contact mass property')
        return value
    weights={'KG':1.,'LBS':.45359237};lengths={'M':1.,'IN':.0254};moments={'KG*M2':1.,'SLUG*FT2':1.35581795}
    cg=balance.find("location[@name='CG']");scale=lengths[cg.get('unit')]
    empty=dict(mass=number(balance.find('emptywt'),weights),cg=np.array([float(cg.findtext(a))*scale for a in 'xyz']))
    for axis in ('ixx','iyy','izz','ixy','ixz','iyz'):
        node=balance.find(axis);empty[axis]=number(node,moments) if node is not None else 0.
    convention=balance.get('negated_crossproduct_inertia','true')
    if convention not in ('true','false'):raise ValueError('Unknown native inertia product convention')
    if convention=='true':
        for axis in ('ixy','ixz','iyz'):empty[axis]*=-1
    if empty['mass']<=0 or not np.isfinite(empty['cg']).all():raise ValueError('Invalid native dry contact state')
    total=empty['mass'];weighted=total*empty['cg']
    for tank in root.findall('propulsion/tank'):
        fuel=number(tank.find('contents'),weights);location=tank.find('location');scale=lengths[location.get('unit')]
        position=np.array([float(location.findtext(a))*scale for a in 'xyz'])
        total+=fuel;weighted+=fuel*position
    class Mass:
        def empty(self):return empty
        def loaded(self):return total,weighted/total
    original=root.find('ground_reactions');index=list(root).index(original);root.remove(original)
    root.insert(index,ET.fromstring(jsbsim.ground_reactions_xml(aircraft,Mass())))
    ET.indent(root,space='    ')
    return ET.tostring(root,encoding='unicode')
