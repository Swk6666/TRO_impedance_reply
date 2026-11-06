import unittest
import numpy as np

from fr3imp.dynamics import euler_rate_map
from fr3imp.scenarios import Scenario


class ScenarioTests(unittest.TestCase):
    def test_paper_reference_derivatives_and_f_repeats_b(self):
        p0=np.array([.4,0,.4,0,0,0.])
        for name in 'ABCF':
            scene=Scenario(name,p0,variant='paper_figure')
            for t in (.7,3.,9.,14.,23.):
                h=1e-4;before=scene.reference(t-h);ref=scene.reference(t);after=scene.reference(t+h)
                np.testing.assert_allclose((after.p-before.p)/(2*h),ref.v,atol=1e-8)
                np.testing.assert_allclose((after.v-before.v)/(2*h),ref.a,atol=1e-8)
        b=Scenario('B',p0,variant='paper_figure');f=Scenario('F',p0,variant='paper_figure')
        self.assertEqual(b.environment(),f.environment())
        for t in np.arange(0,30,.1):np.testing.assert_array_equal(b.reference(t).p,f.reference(t).p)

    def test_virtual_operator_releases_and_is_power_dual(self):
        scene=Scenario('D',np.zeros(6),variant='paper_figure')
        pose=np.array([0,0,0,np.pi/2,.1,.2]);velocity=np.array([0,0,0,.3,-.2,.1])
        np.testing.assert_array_equal(scene.applied_wrench(34,pose,velocity),np.zeros(6))
        force=scene.applied_wrench(8.5,pose,velocity)
        E=euler_rate_map(pose[3:]);generalized=E.T@force[3:]
        self.assertAlmostEqual(generalized[0],0)
        self.assertAlmostEqual(generalized[2],0)
        self.assertAlmostEqual(force[3:]@(E@velocity[3:]),generalized@velocity[3:])

    def test_scaled_operator_and_passive_roll_support(self):
        scene=Scenario('D',np.zeros(6),variant='paper_figure',
                       operator_torque_scale=[1.,.25,.25],operator_roll_hold=True)
        p=np.array([0,0,0,np.pi/2,.1,.2]);v=np.zeros(6)
        # At the guide target it applies no roll moment. Away from the target,
        # the moment restores roll; it does not overwrite the robot state.
        np.testing.assert_allclose(scene.applied_wrench(34,p,v),0,atol=1e-12)
        p[3]-=.01
        f=scene.applied_wrench(34,p,v)
        generalized=euler_rate_map(p[3:]).T@f[3:]
        self.assertGreater(generalized[0],0)
        np.testing.assert_allclose(generalized[1:],0,atol=1e-12)
        f=scene.applied_wrench(8.5,p,v)
        self.assertLessEqual(abs((euler_rate_map(p[3:]).T@f[3:])[1]),3.)


if __name__=='__main__':unittest.main()
