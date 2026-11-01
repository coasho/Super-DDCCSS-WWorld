"""Native nozzle/cycle diagnostics through the public Python Range interface.

Bare-plant mechanism checks, not actor isolation, equipment calibration or a
replacement for legal Agent/C2 release encounters. Run via run.ps1 -Mode python.
"""
import argparse
import json
import math
from pathlib import Path
import fsim

DT=1/120
LBF=4.4482216152605
LBFT=1.3558179483314


def create(world,model,fuel=None):
    v=world.create_vehicle(model,'jsbsim:'+model,latitude_deg=37.6,longitude_deg=-122.4,
        altitude_msl_m=6000,heading_deg=90,pitch_deg=0,roll_deg=0,airspeed_ms=800,fuel_fraction=fuel)
    v.set_protection('off')
    return v


def vector(model):
    with fsim.World(publish=False,workers=1,dt=DT,frame_skip=1,seed=17) as w:
        v=create(w,model);get=v.get_property
        limit=get('fsim/thrust-vector-control/limit_rad');rate=get('fsim/thrust-vector-control/rate_rad_s')
        previous=(0.,0.);peak=0.;force_error=0.;moment_error=0.;stuck=None
        for i in range(36):
            if i==18:
                v.set_property('fsim/thrust-vector-control/pitch-actuator/malfunction/fail_stuck',1)
                stuck=get('propulsion/engine/pitch-angle-rad')
            v.command_actuator(elevator=.03 if i<18 else -.03,rudder=-.02,throttle=1,gear_down=0)
            w.step()
            angles=tuple(get('propulsion/engine/'+axis+'-angle-rad') for axis in ('pitch','yaw'))
            assert all(abs(a)<=limit+1e-10 and abs(a-p)<=rate*DT+1e-9 for a,p in zip(angles,previous))
            if stuck is not None:assert abs(angles[0]-stuck)<1e-12
            p,y=angles;thrust=v.state.thrust_n[0]
            expected=(thrust*math.cos(p)*math.cos(y),thrust*math.cos(p)*math.sin(y),-thrust*math.sin(p))
            actual=tuple(get('forces/fb'+axis+'-prop-lbs')*LBF for axis in 'xyz')
            cg=tuple(get('inertia/cg-'+axis+'-in') for axis in 'xyz')
            exit=tuple(get('propulsion/engine/'+axis+'-position') for axis in 'xyz')
            arm=tuple((n-c)*.0254*sign for n,c,sign in zip(exit,cg,(-1,1,-1)))
            torque=(arm[1]*actual[2]-arm[2]*actual[1],arm[2]*actual[0]-arm[0]*actual[2],arm[0]*actual[1]-arm[1]*actual[0])
            force_error=max(force_error,*(abs(a-b) for a,b in zip(actual,expected)))
            moment_error=max(moment_error,*(abs(get('moments/'+axis+'-prop-lbsft')*LBFT-b) for axis,b in zip('lmn',torque)))
            assert force_error<1e-6 and moment_error<1e-6
            peak=max(peak,*map(abs,angles));previous=angles
        assert peak>1e-6
        return dict(model=model,samples=36,peak_vector_rad=peak,max_force_error_n=force_error,
                    max_moment_error_nm=moment_error,stuck_pitch_holds=True)


def ducted(fuel):
    with fsim.World(publish=False,workers=1,dt=DT,frame_skip=1,seed=17) as w:
        v=create(w,'meteor',fuel);get=v.get_property;prefix='fsim/ducted-rocket/engine[1]/'
        initial=get('propulsion/tank[1]/contents-lbs');peak=0.;samples=0;max_error=0.;air_peak=0.
        previous=0.
        for i in range(420):
            v.command_actuator(throttle=1,gear_down=0);w.step()
            valve=get('fcs/throttle-pos-norm[1]')
            assert 0<=valve<=1 and abs(valve-previous)<=10*DT+1e-9
            previous=valve
            native=v.state.thrust_n[1];net=get(prefix+'net-thrust-n')
            air=get(prefix+'air-kg-s');flow=get(prefix+'fuel-kg-s');exit=get(prefix+'exit-speed-m-s')
            # An independent momentum balance checks the evaluated native
            # cycle. Native pre-functions precede state integration by one dt.
            balance=(air+flow)*exit-air*get('velocities/vt-fps')*.3048
            error=abs(net-balance);max_error=max(max_error,error)
            assert error<max(2.,.01*abs(net))
            assert abs(native-max(0.,net))<max(2.,.01*abs(net))
            if native>1: samples+=1
            peak=max(peak,native);air_peak=max(air_peak,air)
            assert v.state.engine_count==2
        consumed=(initial-get('propulsion/tank[1]/contents-lbs'))*.45359237
        if fuel==0:assert peak==0 and consumed==0 and air_peak==0
        else:assert samples>0 and peak>1 and consumed>0 and air_peak>0
        return dict(model='meteor',fuel_fraction=fuel,samples=420,sustainer_positive_samples=samples,
                    peak_sustainer_thrust_n=peak,peak_captured_air_kg_s=air_peak,
                    fuel_consumed_kg=consumed,max_momentum_error_n=max_error)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    rows=[vector(m) for m in ('aim9x','irist','micaem','micair')]+[ducted(1),ducted(0)]
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(dict(scope='Native engineering mechanisms; no equipment calibration',checks=rows),indent=2)+'\n')
    for row in rows:print(json.dumps(row),flush=True)


if __name__=='__main__':main()
