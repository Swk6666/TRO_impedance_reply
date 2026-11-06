"""Check ideal closed-loop equations rather than reproducing torque code."""
import unittest
import numpy as np

from fr3imp.controllers import HInfinity,Impedance,Admittance
from fr3imp.dynamics import NominalDynamics,angle_error
from fr3imp.model import build_model,home_data,HOME
from fr3imp.scenarios import Scenario


class ClosedLoopTests(unittest.TestCase):
    def setUp(self):
        self.model=build_model(nominal=True,visuals=False)
        home=home_data(self.model);site=self.model.site('tcp').id
        origin=home.site_xmat[site].reshape(3,3)
        self.dyn=NominalDynamics(self.model,origin)
        self.scenario=Scenario('B',self.dyn.evaluate(HOME,np.zeros(7)).p)
        self.q=HOME+np.array([.02,-.03,.01,.04,-.02,.03,.02])
        preliminary=self.dyn.evaluate(self.q,np.zeros(7))
        # Eliminate null velocity to isolate the paper's task diagonal block.
        self.v=preliminary.Jplus@np.array([.01,-.02,.03,-.02,.01,.03])
        self.state=self.dyn.evaluate(self.q,self.v)
        self.force=np.array([2.,-1.,3.,.1,.2,-.1])

    def task_acceleration(self,command):
        s=self.state
        qdd=np.linalg.solve(s.M,command+s.J.T@s.generalized_wrench(self.force)-s.bias)
        return s.J@qdd+s.Jdot@s.v

    def test_impedance_realizes_desired_differential_equation(self):
        c=Impedance(self.dyn,self.scenario);ref=self.scenario.reference(9.)
        out=c.step(9.,self.state,self.force)
        e=angle_error(ref.p,self.state.p);edot=ref.v-self.state.pdot
        residual=(c.L*(ref.a-self.task_acceleration(out.torque))+c.D*edot+c.K*e
                  +self.state.generalized_wrench(self.force))
        np.testing.assert_allclose(residual,0,atol=1e-9)

    def test_hinf_reference_error_obeys_closed_loop_decay(self):
        c=HInfinity(self.dyn,self.scenario)
        c.started=True;c.integral=np.array([.01,-.02,.03,.01,-.03,.02])
        ref=self.scenario.reference(9.);s=self.state
        aref=ref.a+(c.D*(ref.v-s.pdot)+c.K*angle_error(ref.p,s.p)+s.generalized_wrench(self.force))/c.L
        out=c.step(9.,s,self.force)
        derivative=aref-self.task_acceleration(out.torque)
        feedback=np.array([1100,1100,1100,150,150,150])*out.reference_error
        residual=s.inertia@derivative+self.dyn.task_coriolis_product(out.reference_error)+feedback
        np.testing.assert_allclose(residual,0,atol=1e-9)

    def test_admittance_proxy_realizes_task_impedance(self):
        c=Admittance(self.dyn,self.scenario);ref=self.scenario.reference(9.)
        qdd,_,n=c.nominal_acceleration(9.,self.q,self.v,self.force)
        pdd=n.J@qdd+n.Jdot@n.v
        residual=c.L*(ref.a-pdd)+c.D*(ref.v-n.pdot)+c.K*angle_error(ref.p,n.p)+n.generalized_wrench(self.force)
        np.testing.assert_allclose(residual,0,atol=1e-9)


if __name__=='__main__':unittest.main()
