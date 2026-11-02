"""Reconcile every maintained selected fit and bundled JSBSim model explicitly.

Applicability is a contract for the authored selected configuration. It never
claims that an aircraft type cannot carry equipment in another configuration.
"""
import argparse
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]

# Reviewed configuration-level evidence. Family/function distinctions are not
# equipment-accuracy measurements. Authored cards retain parameter uncertainties.
CONFIGURATIONS={
 'a10c':('attack electro-optical / weapons fit','https://www.af.mil/About-Us/Fact-Sheets/Display/Article/104490/a-10c-thunderbolt-ii/'),
 'b52h':('legacy APQ-166 ground/navigation radar; not the APQ-188 modernization','https://www.esd.whs.mil/Portals/54/Documents/FOID/Reading%20Room/Selected_Acquisition_Reports/June_2025_MSARs/B-52_RMP_MSAR_Jun_2025_Amended.pdf'),
 'c130j':('APN-241 weather/navigation radar','https://www.lockheedmartin.com/content/dam/lockheed-martin/aero/documents/sustainment/csc/service-news/sn-mag-v21-v30/V26N01.pdf'),
 'c172':('selected civil trainer with no authored mission payload','https://cessna.txtav.com/en/piston/cessna-skyhawk'),
 'c17a':('authored weather-radar / transport fit','https://www.amc.af.mil/About-Us/Fact-Sheets/Display/Article/977489/c-17-globemaster-iii/'),
 'cf2':('selected Crazyflie research quadrotor, no mission payload','https://www.bitcraze.io/products/old-products/crazyflie-2-0/'),
 'ec130h':('Compass Call electronic-attack fit','https://www.af.mil/About-Us/Fact-Sheets/Display/article/104550/ec-130h-compass-call/'),
 'h6k':('authored ground-map / electro-optical / weapons fit','aircraft/h6k/payloads.json'),
 'iris':('selected research quadrotor, no mission payload','aircraft/iris/payloads.json'),
 'kc135r':('authored tanker weather-radar fit','https://www.af.mil/About-Us/Fact-Sheets/Display/Article/1529736/kc-135-stratotanker/'),
 'kc46a':('authored tanker self-protection / navigation fit; no claim of physical weather-radar absence','https://www.boeing.com/defense/tankers-and-transports/kc-46-pegasus'),
 'rc135w':('Rivet Joint signals-intelligence fit','https://www.raf.mod.uk/aircraft/airseeker-rc-135w-rivet-joint/'),
 'rq4b':('authored EISS SAR/EOIR configuration; not a Block40 MP-RTIP fit','https://www.rtx.com/raytheon/what-we-do/air/globalhawk-iss'),
 'skua':('project design explicitly selects no mission payload','aircraft/skua/payloads.json'),
 'su25':('authored laser / warning-receiver / weapons fit; not an upgraded radar configuration','aircraft/su25/payloads.json'),
 'u2s':('ASARS-2 SAR / electro-optical / SIGINT fit','https://www.beale.af.mil/News/Article-Display/Article/3825542/the-eyes-and-ears-of-the-u-2/'),
 'uh1h':('authored utility-helicopter / door-gun fit','aircraft/uh1h/payloads.json'),
 'uh60':('authored utility-helicopter fit; not an MH-60R maritime-radar configuration','aircraft/uh60/payloads.json'),
}


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def assess(inventory):
    fits=[];cards={}
    for path in sorted((ROOT/'assets/payloads').glob('*.json')):cards[path.stem]=json.loads(path.read_text(encoding='utf-8'))
    unused=set(CONFIGURATIONS)
    for fit in inventory['fits']:
        name=fit['design'];path=ROOT/'aircraft'/name/'payloads.json';data=json.loads(path.read_text(encoding='utf-8'))
        families=sorted({cards[p['card']]['family'] for p in data.get('payloads',[])})
        if fit['air_radars']:
            disposition='applicable_complete_native_air_radar'
            evidence=[dict(card=p['card'],sources=cards[p['card']].get('sources',{})) for p in fit['air_radars']]
            selected='authored '+', '.join(p['card'] for p in fit['air_radars'])
        else:
            selected,source=CONFIGURATIONS[name];unused.remove(name)
            disposition='selected_configuration_uses_other_families_or_no_mission_payload'
            evidence=[dict(source=source,claim='Selected role/equipment context; not an exhaustive physical-absence proof')]
        fits.append(dict(design=name,disposition=disposition,selected_configuration=selected,families=families,
            fit_sha256=sha(path),evidence=evidence,engineering_assignment_closed=True,
            verified_physical_equipment_absence=False))
    if unused:raise ValueError('Stale configuration assessments: '+str(unused))
    stock=[]
    for path in sorted((ROOT/'third_party/jsbsim/aircraft').glob('*/*.xml')):
        if path.stem!=path.parent.name:continue
        adjacent=path.parent/'payloads.json'
        stock.append(dict(model=path.stem,relative_xml=path.relative_to(ROOT).as_posix(),xml_sha256=sha(path),
            adjacent_fit=adjacent.is_file(),fit_sha256=sha(adjacent) if adjacent.is_file() else None,
            disposition='adjacent_fit_only_no_inferred_airframe_alias'))
    if not stock:raise ValueError('Bundled model inventory missing')
    return dict(schema='fsim.sim-m01.applicability.v1',passed=len(fits)==inventory['airframes'],fits=fits,
        source_rule='Air-radar M01 applies to selected authored air_radar cards. Ground-map, weather, EW, EOIR and utility fits are distinct contracts, not absent implementations of M01 air radar.',
        stock_models=stock,stock_resolution='Runtime resolves an exact JSBSim model and reads its adjacent payloads.json; neither a matching prefix nor a real-world type name aliases another fit.',
        custom_models='External/custom JSBSim model names are open-ended inputs, not a finite claimed equipment roster. The same explicit adjacent-fit contract applies.',
        physical_calibration=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--inventory',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists() or any(x.casefold()=='.codex' for path in (a.inventory,a.output) for x in path.parts):raise ValueError('Fresh allowed output required')
    result=assess(json.loads(a.inventory.read_text(encoding='utf-8')));a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(passed=result['passed'],fits=len(result['fits']),stock_models=len(result['stock_models']))))
