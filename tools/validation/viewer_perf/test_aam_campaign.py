import unittest
import subprocess
import copy
import json
from unittest import mock
from pathlib import Path

import aam_campaign as campaign


class FullscreenCampaign(unittest.TestCase):
    def test_every_current_named_model_reports_selection_and_mechanism_separately(self):
        models=campaign.equipment_models.catalogue()
        count=0
        for model in models:
            path=campaign.ROOT/model['card_path'];card=json.loads(path.read_text(encoding='utf-8'))
            if card['class']!='aam':continue
            common=dict(model=card['model'].removeprefix('jsbsim:'),card=card['id'],card_sha256=campaign.sha(path),
                providers=card['component_models']['components'],
                card_description={key:card.get(key) for key in ['designation','seeker','guidance','motor']},
                native=dict(max_alpha_deg=10.,terminal_kind='detonated'))
            head=dict(common,scenario='head-on');tail=dict(common,scenario='tail-turn')
            result=campaign.assess([head,tail])
            self.assertEqual(result['seeker_provider'],model['components']['seeker']['selector'])
            self.assertEqual(result['seeker_mechanism'],model['components']['seeker']['mechanism_id'])
            self.assertNotIn('Retained ideal',result['guidance'])
            bad=copy.deepcopy(head);bad['providers']['seeker']['implementation_id']='equipment.invented.seeker'
            with self.assertRaises(ValueError):campaign.assess([bad,tail])
            count+=1
        self.assertEqual(count,24)

    def test_flight_summary_filters_world_lifetime_instead_of_reused_local_age(self):
        import fsim
        import numpy as np
        from fsim._state import recorded_sample_dtype
        frames=[]
        for world_time,age,mach in ((1.,1.,99.),(14.,1.,1.),(20.,0.,99.)):
            samples=np.zeros(1,dtype=recorded_sample_dtype);samples['slot']=2
            samples['state']['sim_time']=age;samples['state']['mach']=mach
            samples['state']['alpha_rad']=.1 if mach==1 else 3.
            samples['state']['engine_count']=1
            frames.append((world_time,samples))
        class Recording:
            frame_count=3
            def __iter__(self):return iter(frames)
            def events(self,index):return []
        weapon=dict(id=3,weapon_slot=2,launch_time_s=13.,end_time_s=17.,terminal_kind='detonated',
            miss_m=3.,acquired=True,activated=False,supported=False,seeker='imaging_infrared')
        with mock.patch.object(fsim,'Recording',return_value=Recording()):
            result=campaign.summarize('synthetic.fsrec',dict(weapons=[weapon],target_slot=1,engagements=[]))
        self.assertEqual(result['flight_samples'],1)
        self.assertEqual(result['max_mach'],1.)
        self.assertIsNone(result['first_reverse_flow'])

    def test_every_visual_uses_fullscreen_and_complete_scene_effects(self):
        command=campaign.viewer_command(Path('build'),'native world',Path('fullscreen.png'))
        self.assertIn('--fullscreen',command)
        self.assertEqual(command[command.index('--width')+1],'3840')
        self.assertEqual(command[command.index('--height')+1],'2160')
        self.assertEqual(command[command.index('--imagery')+1],'satellite')
        self.assertEqual(command[command.index('--elevation')+1],'terrarium')
        self.assertEqual(command[command.index('--clouds')+1],'scattered')
        self.assertNotIn('--no-effects',command)
        self.assertNotIn('--no-gui',command)
        self.assertNotIn('none',command)
        self.assertNotIn('off',command)
        self.assertNotIn('--replay',command)
        self.assertNotIn('--time-factor',command)

    def test_current_successes_are_not_reported_as_hard_coded_departures(self):
        for name,card_id in [('micaem','mica-em'),('micair','mica-ir'),
                             ('pl10','pl-10'),('r77m','r-77m'),('aim9m','aim-9m')]:
            with self.subTest(model=name):
                path=campaign.ROOT/'assets/stores'/(card_id+'.json')
                card=json.loads(path.read_text(encoding='utf-8'))
                common=dict(model=name,card=card_id,card_sha256=campaign.sha(path),
                    providers=card['component_models']['components'],
                    card_description={key:card.get(key) for key in ['designation','seeker','guidance','motor']},
                    native=dict(max_alpha_deg=10.,terminal_kind='detonated'))
                head=copy.deepcopy(common);head['scenario']='head-on'
                tail=copy.deepcopy(common);tail['scenario']='tail-turn'
                result=campaign.assess([head,tail])
                self.assertNotIn('Both cases depart',result['card_gap'])
                self.assertNotIn('head-on case remains in flight',result['card_gap'])
                self.assertIn('Head-on: detonated; tail: detonated.',result['card_gap'])
                self.assertIn('No large incidence departure',result['control'])


class MandatoryViewerPresentation(unittest.TestCase):
    def test_disabled_scene_and_acceleration_are_rejected_before_opening_window(self):
        build=campaign.ROOT/'build/ucrt64-release'
        viewer=build/'bin/flightsim-viewer.exe'
        self.assertTrue(viewer.is_file())
        for options in [('--no-sun',),('--no-effects',),('--no-gui',),
                        ('--imagery','none'),('--elevation','none'),
                        ('--clouds','off'),('--time-factor','4')]:
            with self.subTest(options=options):
                result=subprocess.run([str(viewer),'--help',*options],
                    env=campaign.environment(build),capture_output=True,text=True,timeout=15)
                self.assertEqual(result.returncode,2,result.stdout+result.stderr)


if __name__=='__main__':unittest.main()
