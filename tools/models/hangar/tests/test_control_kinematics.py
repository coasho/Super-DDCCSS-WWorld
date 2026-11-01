"""JSBSim control signs are geometry, independent of aerodynamic generators."""
from types import SimpleNamespace
import unittest


class ControlKinematics(unittest.TestCase):
    def test_channel_signs_follow_surface_normal_and_aircraft_side(self):
        from hangar.geometry.controls import channel_gain
        control=SimpleNamespace(channels=dict(elevator=2.,flap=.5,aileron=1.,rudder=3.))
        for channel,normal,position,expected in (
            ('elevator',[0.,0.,1.],[0.,0.,0.],2.),('elevator',[0.,0.,-1.],[0.,0.,0.],-2.),
            ('flap',[0.,0.,1.],[0.,1.,0.],.5),('aileron',[0.,0.,1.],[0.,-1.,0.],1.),
            ('aileron',[0.,0.,1.],[0.,1.,0.],-1.),('rudder',[0.,-1.,0.],[0.,0.,0.],-3.)):
            with self.subTest(channel=channel,normal=normal,position=position):
                self.assertEqual(channel_gain(channel,control,normal,position),expected)
        self.assertEqual(channel_gain('elevator',control,[0.,1.,0.],[0.,0.,0.]),0.)
        self.assertEqual(channel_gain('missing',control,[0.,0.,1.],[0.,0.,0.]),0.)
        self.assertEqual(channel_gain('elevator',None,[0.,0.,1.],[0.,0.,0.]),0.)

    def test_required_shared_vlm_handles_lifting_and_zero_panel_geometry(self):
        # SIM-M02 now requires the common VLM; the former absence check was
        # superseded by that requirement, not by a numerical tolerance change.
        from pathlib import Path
        import numpy as np
        from hangar.pipeline import Design,repo_root
        from hangar.engineering import roster
        from hangar.aero.model import AeroModel
        from hangar.aero.vlm import VLM
        lifting=Design(str(Path(repo_root())/'aircraft/c172/c172.toml'))
        body=next(d for r in roster() if r['category']=='store' for d in [Design(r['path'])] if not d.aircraft.surfaces)
        for d in (lifting,body):
            with self.subTest(category=d.aircraft.category):
                model=d.aero_model()
                self.assertIsInstance(model,AeroModel)
                if d.aircraft.surfaces:self.assertIsInstance(model.vlm,VLM)
                else:self.assertIsNone(model.lat)
                values=model.evaluate(.05,.02)
                self.assertTrue(np.isfinite([values[k] for k in ('CD','CY','CL','Cl','Cm','Cn')]).all())

    def test_components_route_only_to_the_shared_airframe_provider(self):
        from unittest import mock
        from hangar import components,airframe
        card={'model':'jsbsim:fixture','component_models':{'schema_version':1,'components':{'aerodynamics':{
            'availability':'available','implementation_id':components.AERODYNAMICS,'configuration_version':1,
            'parameter_refs':{},'bindings':{'model':'jsbsim:fixture','design':'aircraft/fixture/fixture.toml','source':'vlm'},
            'basis':'assumed','note':'Synthetic routing fixture','validity':'Software routing only'}}}}
        source='aircraft/fixture/fixture.toml';sentinel=object()
        with mock.patch.object(airframe,'coefficients',return_value=sentinel) as provider:
            self.assertIs(components.coefficients(card,{'design_path':source}),sentinel)
            provider.assert_called_once_with(source)
        card['component_models']['components']['aerodynamics']['bindings']['source']='alternate'
        with self.assertRaisesRegex(ValueError,'VLM source'):
            components.coefficients(card,{'design_path':source})


if __name__=='__main__':unittest.main()
