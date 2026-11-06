"""Configuration provenance: task equality and effective F feedback gains."""
import tempfile
import unittest
import numpy as np

from fr3imp.experiment import run_experiment
from fr3imp.profiles import profile_parameters


class ProfileTests(unittest.TestCase):
    def test_f_uses_same_task_as_b(self):
        for profile in ('fr3','paper','legacy'):
            self.assertEqual(profile_parameters('B',profile),profile_parameters('F',profile))

    def test_effective_gain_and_task_are_recorded(self):
        with tempfile.TemporaryDirectory() as root:
            records=[run_experiment('F',name,duration=.01,seed=100,output=root)
                     for name in ('hinf_0.25','hinf_0.5','hinf_1')]
        for r in records:
            self.assertEqual(r['status'],'completed')
            self.assertEqual(r['profile'],'fr3')
            self.assertEqual(r['impedance'],records[0]['impedance'])
            self.assertEqual(r['plant'],records[0]['plant'])
        effective=np.array([r['controller_parameters']['alpha'] for r in records])
        np.testing.assert_allclose(effective/effective[-1],[.25,.5,1.])
        self.assertEqual(effective[-1],records[-1]['control_settings']['ref_gain_scale'])


if __name__=='__main__':unittest.main()
