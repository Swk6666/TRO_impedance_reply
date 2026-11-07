import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'scripts'))

from check_indy7_inertia import base_from_full,experiment_pose,inertia_matrix,regrouped_links,rot_x,tcp_pose
from fr3imp.controllers import Admittance
from fr3imp.dynamics import NominalDynamics
from fr3imp.experiment import default_plant_parameters
from fr3imp.model import HOME,build_model
from fr3imp.scenarios import Scenario
from report_d_admittance import AXES,L,STEP,identify,response
from validate_d_admittance import DiagnosticAdmittance,cases


class DiagnosticAdmittanceTests(unittest.TestCase):
    def setUp(self):
        model=build_model(default_plant_parameters('D','fr3v2'),nominal=True)
        self.dynamics=NominalDynamics(model,np.eye(3))
        state=self.dynamics.evaluate(HOME,np.zeros(7))
        self.scene=Scenario('D',state.p)
        self.wrench=np.array([1.,-2.,3.,.4,-.5,.6])

    def controller(self,cls,**attributes):
        controller=cls(self.dynamics,self.scene)
        for name,value in attributes.items():setattr(controller,name,value)
        return controller

    def test_switched_off_terms_equal_the_paper_controller(self):
        plain=self.controller(Admittance).nominal_acceleration(0.,HOME,np.zeros(7),self.wrench)
        same=self.controller(DiagnosticAdmittance).nominal_acceleration(0.,HOME,np.zeros(7),self.wrench)
        np.testing.assert_allclose(same[0],plain[0],atol=1e-12)
        np.testing.assert_allclose(same[1],plain[1],atol=1e-12)

    def test_feedthrough_adds_task_acceleration_mx_inverse_f(self):
        plain=self.controller(DiagnosticAdmittance).nominal_acceleration(0.,HOME,np.zeros(7),self.wrench)
        fed=self.controller(DiagnosticAdmittance,feedthrough=1.).nominal_acceleration(0.,HOME,np.zeros(7),self.wrench)
        n=plain[2]
        np.testing.assert_allclose(n.J@(fed[0]-plain[0]),np.linalg.solve(n.inertia,n.generalized_wrench(self.wrench)),atol=1e-9)
        # The -J^T F in the torque and the new acceleration cancel, so the torque keeps only the impedance part.
        np.testing.assert_allclose(fed[1]-plain[1],n.J.T@n.generalized_wrench(self.wrench),atol=1e-9)


class Indy7ModelTests(unittest.TestCase):
    def test_table_a1_combinations_agree_with_the_dh_table(self):
        rng=np.random.default_rng(3)
        for _ in range(10):
            links=[]
            for _ in range(6):
                A=rng.normal(size=(3,3));links.append(dict(m=rng.uniform(.5,8),h=rng.normal(size=3),I=A@A.T+np.eye(3)))
            rotor=rng.uniform(0,2,6);q=rng.uniform(-3,3,6)
            b=base_from_full(links,rotor)
            regrouped=inertia_matrix(q,regrouped_links(b),[0,0,b['b22'],b['b33'],b['b44'],b['b55']])
            # The table rounds its geometric constants, so the agreement is not exact.
            np.testing.assert_allclose(regrouped,inertia_matrix(q,links,rotor),atol=2e-3)

    def test_inverse_kinematics_holds_the_point_through_the_roll(self):
        point=np.array([.55,-.187,.45])
        first,second=experiment_pose(point,.12,-np.pi/2)
        np.testing.assert_allclose(tcp_pose(first,.12)[0],point,atol=1e-6)
        np.testing.assert_allclose(tcp_pose(second,.12)[0],point,atol=1e-6)
        np.testing.assert_allclose(tcp_pose(second,.12)[1],rot_x(-np.pi/2)@tcp_pose(first,.12)[1],atol=1e-6)


class IdentificationTests(unittest.TestCase):
    def test_recovers_known_diagonal_gain_and_zero(self):
        time=np.arange(0,20,STEP)
        m=np.column_stack([np.sin(.7*time)**2,np.sin(1.1*time)*(time>2),np.cos(.9*time)*(time>1)])
        truth=np.array([3.,4.,2.])
        x=np.column_stack([response(i,(1+truth[i])*m[:,i]) for i in range(3)])
        fit=identify(time,x,m,(0.,20.))
        np.testing.assert_allclose(np.diag(fit['W']),truth,atol=.02)
        ideal=np.column_stack([response(i,m[:,i]) for i in range(3)])
        np.testing.assert_allclose(np.diag(identify(time,ideal,m,(0.,20.))['W']),np.zeros(3),atol=1e-6)
        self.assertEqual(len(AXES),len(L))

    def test_matrix_has_every_candidate(self):
        names=set(cases())
        for name in ('feedthrough/D_hinf','feedthrough_own/D_admittance','servo_g0.03/D_admittance',
                     'ft_delay20/D_admittance','force_gain4/D_admittance','feedthrough/C_admittance','feedthrough/B_admittance','feedthrough/A_impedance'):
            self.assertIn(name,names)


if __name__=='__main__':
    unittest.main()
