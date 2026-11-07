import unittest
import numpy as np
from fr3imp.evaluation import ideal_impedance_error,impedance_metrics
from fr3imp.scenarios import Scenario


class ImpedanceEvaluationTests(unittest.TestCase):
    def test_critically_damped_step_matches_analytic_solution(self):
        t=np.arange(5000)*.001
        # e''+4e'+4e=8 => e=2[1-(1+2t)exp(-2t)].
        result=ideal_impedance_error(t,np.full((len(t),6),8.),np.ones(6),
                                    np.full(6,4.),np.full(6,4.),np.zeros(6),np.zeros(6))
        expected=2*(1-(1+2*t)*np.exp(-2*t))
        np.testing.assert_allclose(result[:,0],expected,atol=1e-12)

    def test_zero_stiffness_force_axis_has_no_artificial_spring(self):
        t=np.arange(2000)*.001
        result=ideal_impedance_error(t,np.full((len(t),6),2.),np.ones(6),
                                    np.full(6,2.),np.zeros(6),np.zeros(6),np.zeros(6))
        np.testing.assert_allclose(result[:,0],t-.5*(1-np.exp(-2*t)),atol=1e-12)

    def test_stiff_axis_metrics_exclude_the_force_axis(self):
        scenario=Scenario('C',np.r_[.4,0,.3,0,0,0],variant='paper_figure')
        self.assertEqual(scenario.stiffness[2],0)
        n=200;desired=np.tile(np.r_[.4,0,.3,0,0,0],(n,1))
        pose=desired.copy();pose[:,2]+=np.linspace(0,.05,n)  # Drift along z only: free to do so.
        log=dict(time=np.arange(n)*.001,pose=pose,desired=desired,velocity=np.zeros((n,6)),
                 force_true=np.zeros((n,6)),force_desired=np.zeros((n,6)),force_measured=np.zeros((n,6)),
                 force_contact=np.zeros((n,6)),reference_error=np.zeros((n,6)),board_contact=np.zeros(n))
        original=scenario.reference
        scenario.reference=lambda t:type(original(t))(desired[0],np.zeros(6),np.zeros(6),np.zeros(6))
        metrics=impedance_metrics(log,scenario)[0]
        self.assertGreater(metrics['dynamic_impedance_position_rmse_m'],.01)
        self.assertLess(metrics['dynamic_impedance_position_rmse_stiff_axes_m'],1e-9)
        self.assertLess(metrics['equilibrium_position_error_measured_mean_m'],1e-9)


if __name__=='__main__':unittest.main()
