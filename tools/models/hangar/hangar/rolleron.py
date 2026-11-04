"""Passive gyro-actuated physical fin patches (NACA RM SL55C22 method).

All numerical mechanical inputs below are explicit generic engineering bounds,
not measured AIM-9M hardware constants. No command channel drives these patches.
"""
import copy
import math
import xml.etree.ElementTree as ET
import numpy as np
from . import finregistry

PREFIX='fsim/passive-rolleron-v1/'

def hinge_stiffness(surface,control):
    """NACA938 plain-flap pressure integral, with declared finite-AR closure.

    Moment derivative is q * integral(c^2 * Ch_delta ds). Normalize by full
    airfoil chord, not by flap chord twice. No equipment coefficient is fit.
    """
    if control.all_moving:raise ValueError('rolleron method1 requires a plain trailing-edge flap')
    z,w=np.polynomial.legendre.leggauss(64)
    def section(cf):
        theta_h=math.acos(2*cf-1)
        theta=theta_h+(z+1)*.5*(math.pi-theta_h)
        weights=w*.5*(math.pi-theta_h)
        pressure=4*((math.pi-theta_h)/math.pi*(1+np.cos(theta))/np.sin(theta)+
            np.log(np.sin((theta+theta_h)/2)/np.sin((theta-theta_h)/2))/math.pi)
        chord=.5*(1-np.cos(theta));hinge=1-cf
        return float(np.sum(weights*(chord-hinge)*pressure*.5*np.sin(theta)))
    eta=control.eta0+(z+1)*.5*(control.eta1-control.eta0)
    weights=w*.5*(control.eta1-control.eta0)*surface.half_arc
    raw=sum(float(weight)*surface.station(float(e))[1]**2*section(control.chord_fraction(float(e))) for e,weight in zip(eta,weights))
    full_eta=(z+1)*.5
    area=sum(float(weight)*.5*surface.half_arc*surface.station(float(e))[1] for e,weight in zip(full_eta,w))
    aspect=surface.half_arc**2/area
    finite=aspect/(aspect+2) # Declared elliptic lifting-line engineering closure.
    if not all(math.isfinite(v) and v>0 for v in (raw,aspect,finite)):
        raise ValueError('rolleron hinge geometry must produce finite positive pressure moment')
    return {'hinge-section-stiffness-m3':raw,'hinge-finite-aspect-ratio':aspect,
            'hinge-finite-aspect-factor':finite,'hinge-aerodynamic-stiffness-m3':raw*finite}


def geometry(aircraft):
    result={}
    origin=np.asarray(aircraft.spec['mass']['empty_cg'],float)
    for surface in aircraft.surfaces:
        config=surface.spec.get('rolleron',{})
        if config.get('version')!=1:continue
        for control in surface.controls:
            if not control.name.startswith('passive rolleron '):continue
            if config.get('hinge-aerodynamic-method')!=1:raise ValueError('rolleron needs explicit geometry-derived hinge method1')
            eta=.5*(control.eta0+control.eta1)
            le,chord,_,_=surface.station(eta)
            centre=le+np.array([chord*(1-.5*control.chord_fraction(eta)),0,0])
            axis=surface.span_direction(eta)
            for side in ((1,-1) if surface.mirror else (1,)):
                key=finregistry.identity(surface,control,side)
                row={k:float(v) for k,v in config.items() if k not in ('version','radius')}
                row.update(hinge_stiffness(surface,control))
                row['wheel-radius-m']=surface.rolleron
                row['centre-from-origin-body-m']=[-float(centre[0]-origin[0]),side*float(centre[1])-float(origin[1]),-float(centre[2]-origin[2])]
                row['hinge-axis-body']=[0.,side*float(axis[1]),-float(axis[2])]
                row['limit-rad']=math.radians(min(-control.min_deg,control.max_deg))
                if not all(math.isfinite(v) and v>0 for k,v in row.items() if isinstance(v,float)):
                    raise ValueError('rolleron requires finite positive mechanical inputs')
                if key in result:raise ValueError('duplicate passive fin identity')
                result[key]=row
    return result

def without_authority(aircraft,tables):
    passive=geometry(aircraft)
    if not passive:return tables
    result=dict(tables)
    result['native_controls']=[dict(row,gains={ch:0. for ch in row['gains']}) if row['id'] in passive else row
                              for row in tables.get('native_controls',[])]
    return result

def install(fcs,channel,aircraft,physical):
    selected=geometry(aircraft)
    if not selected:return {}
    rows={}
    def metadata(name,value):
        ET.SubElement(fcs,'property',value='%.17g'%value).text=PREFIX+name
    metadata('version',1);metadata('count',len(selected))
    metadata('relative-input-uncertainty',.5)
    for index,fin in enumerate(physical):
        if fin['id'] not in selected:continue
        row=dict(selected[fin['id']]);number=len(rows)
        # Normalize the physical hinge/wheel handedness using the actual
        # native Cl slope: negative physical angle must oppose positive roll.
        if 'values' in fin:
            a=int(np.argmin(np.abs(fin['_alpha'])))
            knots=np.asarray(fin['deflection']);j=int(np.searchsorted(knots,0.))
            slope=(fin['values']['Cl'][a,j+1]-fin['values']['Cl'][a,j-1])/(knots[j+1]-knots[j-1])
            if not math.isfinite(slope) or slope==0:raise ValueError('passive rolleron has no physical roll authority')
            sign=math.copysign(1.,slope)
        else:sign=1. # Metadata-only fixture; production always supplies loads.
        row.update({'fin-id':fin['id'],'fin-index':index,'native-deflection-sign':sign})
        for key,value in row.items():
            if isinstance(value,list):
                for axis,v in zip('xyz',value):metadata('fin-%d/%s-%s'%(number,key,axis),v)
            else:metadata('fin-%d/%s'%(number,key),value)
        metadata('fin-%d/angle-rad'%number,0)
        node=ET.SubElement(channel,'fcs_function',name=PREFIX+'fin-%d/native-angle'%number)
        ET.SubElement(ET.SubElement(node,'function'),'property').text=PREFIX+'fin-%d/angle-rad'%number
        ET.SubElement(node,'output').text='fcs/fin-%d-delta-rad'%index
        rows[index]=row
    if len(rows)!=len(selected):raise ValueError('passive rolleron geometry lacks its native aerodynamic patch')
    return rows
