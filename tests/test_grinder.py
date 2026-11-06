"""Internal rotating mass must affect the sensor without inventing external force."""
from pathlib import Path
import tempfile
import unittest
import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from fr3imp.experiment import default_plant_parameters, run_experiment
from fr3imp.grinding import GrindingContact
from fr3imp.model import build_model, home_data
from fr3imp.sensors import Sensors, SensorParameters


@unittest.skipUnless(os.environ.get('FR3IMP_ENABLE_GRINDING_TESTS')=='1',
                     'Experiment E is deferred by user instruction in AGENTS.md')
class GrinderTests(unittest.TestCase):
    def test_fixed_tool_matches_centrifugal_force_and_tcp_moment(self):
        p=default_plant_parameters('E')
        plant=build_model(p,visuals=False)
        # Clamp the seven arm joints by removing their DOFs in an independent
        # fixture; retain the exact rotor, drive and F/T sensor from production.
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fixture.xml'
            mujoco.mj_saveLastXML(str(path),plant)
            root=ET.parse(path).getroot()
            for body in root.findall('.//worldbody//body'):
                for joint in list(body.findall('joint')):
                    if joint.get('name','').startswith('fr3_joint'):body.remove(joint)
            actuator=root.find('actuator')
            for item in list(actuator):
                if item.get('name')!='rotor_drive':actuator.remove(item)
            root.remove(root.find('keyframe'))
            root.find('option').set('gravity','0 0 0')
            fixture=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        data=mujoco.MjData(fixture)
        omega=2*np.pi*p.rotor_frequency_hz
        sensor=Sensors(SensorParameters(q_std=0,v_std=0,force_std=0,moment_std=0,tool_calibration_mass=0))
        self.assertEqual(fixture.nv,1)
        for index,phase in enumerate(np.linspace(0,2*np.pi,9)):
            data.qpos[:]=phase;data.qvel[:]=omega;data.ctrl[:]=omega
            mujoco.mj_forward(fixture,data)
            rotation=data.site_xmat[fixture.site('wrist').id].reshape(3,3)
            measured=sensor.wrench(index,data.sensordata[:6],rotation)
            force=p.rotor_mass*p.rotor_eccentricity*omega**2*np.array([np.cos(phase),0,-np.sin(phase)])
            expected=np.r_[rotation@force,rotation@np.cross([0,0,-.04],force)]
            np.testing.assert_allclose(measured,expected,atol=2e-8)
            self.assertEqual(data.ncon,0)

    def test_rotor_is_hidden_from_controller_and_logged_for_replay(self):
        p=default_plant_parameters('E')
        self.assertEqual(build_model(p,visuals=False).nv,8)
        self.assertEqual(build_model(p,nominal=True,visuals=False).nv,7)
        with tempfile.TemporaryDirectory() as directory:
            summary=run_experiment('E',duration=.03,output=directory,seed=100)
            self.assertEqual(summary['status'],'completed')
            self.assertEqual(summary['force_samples'],30)
            self.assertEqual(summary['sensors']['tool_calibration_mass'],p.tool_mass)
            with np.load(next(Path(directory).glob('*/trajectory.npz'))) as data:
                self.assertEqual(data['q_measured'].shape,(30,7))
                self.assertEqual(data['plant_q'].shape,(30,8))
                np.testing.assert_array_equal(data['force_true'],0.)
                self.assertGreater(np.std(data['force_measured'][:,0]),1.)
                self.assertLess(np.max(np.abs(data['rotor_motor_torque'])),.5)

    def test_spinning_contact_is_dissipative_and_coulomb_bounded(self):
        p=default_plant_parameters('E')
        base=build_model(p,visuals=False);initial=home_data(base)
        tcp=initial.site_xpos[base.site('tcp').id]
        model=build_model(p,visuals=False,environment=[dict(name='workpiece',
            pos=f'{tcp[0]} {tcp[1]} {tcp[2]-.0199}',size='.2 .2 .01')])
        data=home_data(model)
        contact=GrindingContact(model,p);contact.update(data)
        self.assertGreater(contact.normal_load,0.)
        self.assertLess(contact.power,0.)
        self.assertLessEqual(np.linalg.norm(contact.wrench[:3]),p.contact_friction*contact.normal_load)
        self.assertTrue(all(data.contact[i].dim==1 for i in range(data.ncon)))
        # Generalized power must equal contact-point power, including the
        # hidden rotor DOF; applying friction to the arm TCP would fail this.
        generalized=np.zeros(model.nv)
        wrench=data.xfrc_applied[contact.body]
        mujoco.mj_applyFT(model,data,wrench[:3],wrench[3:],data.xipos[contact.body],contact.body,generalized)
        self.assertAlmostEqual(float(generalized@data.qvel),contact.power,places=8)


if __name__=='__main__':unittest.main()
