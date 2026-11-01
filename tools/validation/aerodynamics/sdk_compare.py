"""Pure comparison safeguards retained for validation-tool compatibility.

These functions never generate flight forces or label numerical differences
as equipment accuracy. CG/reference inputs are structural coordinates (metres);
forces and moments are body SI, as in the JSBSim frame convention documented at
https://jsbsim-team.github.io/jsbsim-reference-manual/user/concepts/frames-of-reference/.
"""
import re
import xml.etree.ElementTree as ET

import numpy as np


def _pairs(left,right):
    a,b=np.asarray(left,float),np.asarray(right,float)
    if a.ndim!=1 or not a.size or a.shape!=b.shape or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('Comparison requires complete finite one-dimensional pairs')
    return a,b


def relative_change(previous,current):
    a,b=_pairs(previous,current)
    denominator=float(np.linalg.norm(a))
    return 100.*float(np.linalg.norm(b-a))/denominator if denominator else None


def reference_error(actual,reference):
    a,b=_pairs(actual,reference)
    rmse=float(np.sqrt(np.mean((a-b)**2)))
    scale=float(np.sqrt(np.mean(b*b)))
    return dict(rmse=rmse,normalized_rmse_percent=100.*rmse/scale if scale else None)


def reference_coefficients(row):
    force=np.asarray(row['force_body_N'],float)
    moment=np.asarray(row['moment_body_Nm'],float)
    cg=np.asarray(row['cg_m'],float)
    arp=np.asarray(row['aerodynamic_reference_m'],float)
    if any(x.shape!=(3,) or not np.isfinite(x).all() for x in (force,moment,cg,arp)):
        raise ValueError('Reference transfer requires finite three-dimensional snapshots')
    qbar=float(row['qbar_psf'])*4.4482216152605/.3048**2
    # Unit reference geometry is the explicit default for synthetic safeguard
    # fixtures; equipment comparisons must supply their actual SI dimensions.
    area=float(row.get('area_m2',1.));span=float(row.get('span_m',1.));chord=float(row.get('chord_m',1.))
    if not all(np.isfinite(x) and x>0 for x in (qbar,area,span,chord)):
        raise ValueError('Reference normalization requires finite positive pressure and geometry')
    arm=(cg-arp)*np.array([-1.,1.,-1.])
    reference_moment=moment+np.cross(arm,force)
    return np.concatenate((force/(qbar*area),reference_moment/(qbar*area*np.array([span,chord,span])))).tolist()


def marker(source,digest):
    if not isinstance(digest,str) or not re.fullmatch('[0-9a-f]{64}',digest):
        raise ValueError('Generator marker requires a SHA256 digest')
    root=ET.fromstring(source)
    controls=root.findall('flight_control')
    if not controls:controls=[ET.SubElement(root,'flight_control',name='Neutral validation fixture')]
    for control in controls:
        outputs=set()
        for channel in control.findall('channel'):
            for node in channel.iter():
                name=node.get('name','')
                if name.startswith('fcs/'):outputs.add(name)
                if node.tag=='output' and (node.text or '').strip():outputs.add(node.text.strip())
            control.remove(channel)
        declared={(node.text or '').strip() for node in control.findall('property')}
        for name in sorted(outputs-declared):ET.SubElement(control,'property',value='0').text=name
    name='fsim/validation/generator-'+digest
    ET.SubElement(controls[0],'property',value='1').text=name
    return ET.tostring(root,encoding='unicode'),name
