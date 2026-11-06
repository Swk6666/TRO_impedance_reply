import unittest
import mujoco
import numpy as np

from fr3imp.model import build_model, home_data, HOME, PlantParameters
from fr3imp.dynamics import NominalDynamics


class DynamicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = build_model(nominal=True)
        d = home_data(cls.model)
        cls.origin = d.site_xmat[cls.model.site('tcp').id].reshape(3,3).copy()

    def setUp(self):
        self.dyn = NominalDynamics(self.model, self.origin)
        self.q = HOME + np.array([.05, -.08, .03, .06, -.04, .07, .02])
        self.v = np.array([.1, -.2, .3, -.15, .2, .05, -.1])

    def test_analytic_jacobian_and_derivative(self):
        s = self.dyn.evaluate(self.q, self.v)
        h = 1e-6
        plus = self.dyn.evaluate(self.q+h*self.v, self.v)
        minus = self.dyn.evaluate(self.q-h*self.v, self.v)
        np.testing.assert_allclose(s.Jdot, (plus.J-minus.J)/(2*h), atol=2e-8)
        np.testing.assert_allclose(s.pdot, (plus.p-minus.p)/(2*h), atol=2e-8)

    def test_coriolis_product_against_christoffel(self):
        h = 1e-6
        derivatives = np.empty((7,7,7))
        for k in range(7):
            dq = np.eye(7)[k] * h
            derivatives[:,:,k] = (self.dyn.evaluate(self.q+dq,self.v).M-
                                  self.dyn.evaluate(self.q-dq,self.v).M)/(2*h)
        C = np.zeros((7,7))
        for i in range(7):
            for j in range(7):
                for k in range(7):
                    C[i,j] += .5*(derivatives[i,j,k]+derivatives[i,k,j]-derivatives[j,k,i])*self.v[k]
        s = self.dyn.evaluate(self.q, self.v)
        u = np.array([.2, .1, -.1, .3, .05, -.2, .15])
        np.testing.assert_allclose(self.dyn.coriolis_product(u), C@u, atol=2e-8)
        np.testing.assert_allclose(C@self.v, s.bias-s.gravity, atol=2e-8)
        Mdot = np.einsum('ijk,k->ij', derivatives, self.v)
        np.testing.assert_allclose(Mdot, C+C.T, atol=2e-8)

    def test_redundancy_and_power_duality(self):
        s = self.dyn.evaluate(self.q, self.v)
        np.testing.assert_allclose(s.J@s.Jplus, np.eye(6), atol=1e-12)
        np.testing.assert_allclose(s.J@np.linalg.solve(s.M,s.null.T), np.zeros((6,7)),atol=1e-12)
        wrench = np.arange(6.)+.1
        f = s.generalized_wrench(wrench)
        physical_velocity = np.r_[s.pdot[:3],s.E@s.pdot[3:]]
        self.assertAlmostEqual(f@s.pdot, wrench@physical_velocity, places=12)

    def test_task_space_energy_identity(self):
        h=1e-6
        plus=self.dyn.evaluate(self.q+h*self.v,self.v)
        minus=self.dyn.evaluate(self.q-h*self.v,self.v)
        derivative=(plus.inertia-minus.inertia)/(2*h)
        self.dyn.evaluate(self.q,self.v)
        gamma=np.column_stack([self.dyn.task_coriolis_product(e) for e in np.eye(6)])
        np.testing.assert_allclose(derivative,gamma+gamma.T,atol=2e-7)

    def test_plant_nominal_separation(self):
        plant = build_model()
        nominal = self.model
        self.assertFalse(np.array_equal(plant.body_mass,nominal.body_mass))
        self.assertTrue(np.all(plant.dof_damping>0))
        self.assertTrue(np.all(nominal.dof_damping==0))
        self.assertTrue(np.all(nominal.dof_frictionloss==0))
        self.assertEqual(plant.opt.timestep,.00025)
        np.testing.assert_allclose(plant.actuator_gainprm[:,0],1)

    def test_headless_mesh_removal_preserves_physics(self):
        full=build_model();compact=build_model(visuals=False)
        a,b=home_data(full),home_data(compact)
        a.qpos[:]=self.q;b.qpos[:]=self.q
        a.qvel[:]=self.v;b.qvel[:]=self.v
        for m,d in [(full,a),(compact,b)]:mujoco.mj_forward(m,d)
        ma,mb=np.zeros((7,7)),np.zeros((7,7))
        mujoco.mj_fullM(full,a,ma);mujoco.mj_fullM(compact,b,mb)
        np.testing.assert_allclose(ma,mb,atol=1e-12,rtol=0)
        np.testing.assert_allclose(a.qfrc_bias,b.qfrc_bias,atol=1e-12,rtol=0)
        np.testing.assert_allclose(a.qacc,b.qacc,atol=1e-10,rtol=0)
        self.assertLess(compact.nmesh,full.nmesh)


if __name__ == '__main__':
    unittest.main()
