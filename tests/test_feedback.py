import unittest
import mujoco
import numpy as np
from fr3imp.sensors import Sensors,SensorParameters
from fr3imp.controllers import EnergyTank,HInfinity
from fr3imp.dynamics import NominalDynamics
from fr3imp.model import build_model,home_data,HOME
from fr3imp.scenarios import Scenario


class FeedbackTests(unittest.TestCase):
    def test_physical_wrist_sensor_sign_origin_and_gravity(self):
        m=build_model();d=home_data(m)
        tcp=m.site('tcp').id;wrist=m.site('wrist').id;body=m.body('tool').id
        rotation=d.site_xmat[wrist].reshape(3,3)
        force=np.array([1.,2.,3.]);moment=np.array([.1,.2,.3])
        jp=np.zeros((3,7));jr=jp.copy()
        mujoco.mj_jacSite(m,d,jp,jr,tcp)
        d.xfrc_applied[body,:3]=force
        d.xfrc_applied[body,3:]=moment+np.cross(d.site_xpos[tcp]-d.xipos[body],force)
        d.ctrl[:]=d.qfrc_bias-jp.T@force-jr.T@moment
        mujoco.mj_forward(m,d)
        sensor=Sensors(SensorParameters(q_std=0,v_std=0,force_std=0,moment_std=0))
        measured=sensor.wrench(0,d.sensordata[:6],rotation)
        np.testing.assert_allclose(measured,np.r_[force,moment],atol=1e-10)

    def test_sampling_and_reproducibility(self):
        p=SensorParameters(tool_calibration_mass=0)
        a,b=Sensors(p,4),Sensors(p,4)
        records=[]
        for k in range(12):
            fa=a.wrench(k,np.zeros(6),np.eye(3))
            fb=b.wrench(k,np.zeros(6),np.eye(3))
            np.testing.assert_array_equal(fa,fb)
            records.append(fa)
        for k in [1,2,3,5,6,7,9,10,11]:np.testing.assert_array_equal(records[k],records[k//4*4])
        self.assertFalse(np.array_equal(records[0],records[4]))
        self.assertEqual(a.force_sample_count,3)

    def test_measurement_noise_changes_control(self):
        m=build_model(nominal=True);d=home_data(m)
        origin=d.site_xmat[m.site('tcp').id].reshape(3,3)
        dyn1,dyn2=NominalDynamics(m,origin),NominalDynamics(m,origin)
        s1=dyn1.evaluate(HOME,np.zeros(7));s2=dyn2.evaluate(HOME,np.zeros(7))
        scenario=Scenario('B',s1.p)
        c1,c2=HInfinity(dyn1,scenario),HInfinity(dyn2,scenario)
        t1=c1.step(0,s1,np.zeros(6)).torque
        t2=c2.step(0,s2,np.array([.5,0,0,0,0,0])).torque
        self.assertGreater(np.linalg.norm(t1-t2),1e-3)

    def test_modulation_branches_and_tank_accounting(self):
        # Source eq. (19): positive generation ratio, saturating at one;
        # non-positive compensator power -> zero, depleted tank -> one.
        for generation,compensation,expected in [(1,2,.5),(3,2,1),(-1,2,0),(1,-2,0)]:
            tank=EnergyTank()
            self.assertEqual(tank.update(generation,compensation,0,0,0,0),expected)
        tank=EnergyTank(initial=.01)
        self.assertEqual(tank.update(-1,2,0,0,0,0),1)
        tank=EnergyTank(initial=.5)
        tank.update(1,2,.2,0,0,0)
        self.assertAlmostEqual(tank.energy,.4)


if __name__=='__main__':unittest.main()
