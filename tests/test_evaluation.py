import unittest
import numpy as np
from fr3imp.evaluation import ideal_impedance_error


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


if __name__=='__main__':unittest.main()
