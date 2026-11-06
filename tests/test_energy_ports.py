"""Independent energy balance for the source's hybrid force-command extension."""
import unittest
import tempfile
from pathlib import Path
import numpy as np
from fr3imp.controllers import EnergyTank,Bidirectional
from fr3imp.actuation import ActuatorParameters,TorqueInterface
from fr3imp.experiment import run_experiment
from fr3imp.dynamics import NominalDynamics,angle_error
from fr3imp.model import HOME,build_model,home_data
from fr3imp.scenarios import Scenario


class EnergyPortTests(unittest.TestCase):
    def test_force_command_is_a_separate_power_supply(self):
        # One collocated axis suffices to distinguish physical environment
        # power from a commanded force. This identity is derived from the
        # mechanical equations, not from the controller implementation.
        m,mn,K,D=3.,4.,50.,8.
        p,pn,pd=.12,.08,.2
        v,vn,vd=.2,-.1,.05
        c,F,Fd,beta=1.7,4.,2.5,.4
        er,en=pd-p,pd-pn
        ar=(K*er-D*v-Fd+(1-beta)*c+F)/m
        an=(K*en-D*vn-Fd-beta*c+F)/mn
        sr_dot=m*v*ar+K*er*(vd-v)
        sn_dot=mn*vn*an+K*en*(vd-vn)
        environment_port=v*F
        reference_port=vd*K*((1-beta)*en+beta*er)
        command_port=-((1-beta)*vn+beta*v)*Fd
        dissipation=(1-beta)*D*vn**2+beta*D*v**2
        internal=(1-beta)*((vn-v)*F-beta*(vn-v)*c)
        self.assertAlmostEqual((1-beta)*sn_dot+beta*sr_dot,
            environment_port+reference_port+command_port-dissipation+internal)
        self.assertGreater(abs((vn-v)*F-(vn-v)*(F-Fd)),.1)

    def test_source_rule_suppresses_cancellable_power(self):
        for generation,compensation in [(2.,4.),(4.,2.),(-2.,4.)]:
            beta=EnergyTank().update(generation,compensation,0,0,0,0)
            residual=(1-beta)*(generation-beta*compensation)
            self.assertLessEqual(residual,1e-12)

    def test_command_filter_matches_analytic_step(self):
        dt=.001;fc=100.;a=dt/(dt+1/(2*np.pi*fc))
        interface=TorqueInterface(ActuatorParameters(cutoff_hz=fc),dt,np.zeros(7))
        for k in range(1,11):
            np.testing.assert_allclose(interface.step(np.ones(7)),1-(1-a)**k,atol=1e-14)
        interface=TorqueInterface(ActuatorParameters(),dt,np.zeros(7))
        np.testing.assert_array_equal(interface.step(np.arange(7)),np.arange(7))

    def test_beta_one_removes_the_compensator_from_real_torque(self):
        model=build_model(nominal=True,visuals=False);home=home_data(model)
        dyn=NominalDynamics(model,home.site_xmat[model.site('tcp').id].reshape(3,3))
        initial=dyn.evaluate(HOME,np.zeros(7))
        scenario=Scenario('C',initial.p,variant='paper_figure')
        state=dyn.evaluate(HOME+np.array([.001,0,0,0,0,0,0]),np.full(7,.001))
        ctrl=Bidirectional(dyn,scenario,beta_override=1.,energy_port='physical',
                           coupling_update='stages',virtual_substeps=4)
        ref=scenario.reference(.1)
        expected=state.gravity+state.J.T@(scenario.stiffness*angle_error(ref.p,state.p)
                   -scenario.damping*state.pdot-state.generalized_wrench(ref.force))
        expected+=state.null.T@(12*(HOME-state.q)-4*state.v)
        out=ctrl.step(.1,state,np.array([2.,-1.,9.,0.,0.,0.]))
        np.testing.assert_allclose(out.torque,expected,atol=1e-12)
        self.assertEqual(out.beta,1.)

    def test_virtual_substeps_do_not_resample_physical_feedback(self):
        with tempfile.TemporaryDirectory() as root:
            result=run_experiment('C','bidirectional',duration=.05,seed=100,output=root,
                scenario_variant='paper_figure',control_overrides=dict(alpha=1.,nric_gain_scale=1.,
                beta_override=1.,energy_port='physical',coupling_update='stages',virtual_substeps=4),
                actuator_parameters=ActuatorParameters(cutoff_hz=100))
            self.assertEqual(result['status'],'completed',result['termination_reason'])
            self.assertEqual(result['completed_control_steps'],50)
            self.assertEqual(result['force_samples'],50)
            with np.load(next(Path(root).glob('*/trajectory.npz'))) as a:
                np.testing.assert_allclose(a['time'],np.arange(50)*.001)
                self.assertTrue(np.isfinite(a['proxy_q']).all())


if __name__=='__main__':unittest.main()
