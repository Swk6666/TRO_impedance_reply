"""Configuration provenance: task equality and effective F feedback gains."""
import tempfile
import unittest
import numpy as np

from fr3imp.experiment import default_plant_parameters,run_experiment
from fr3imp.profiles import profile_parameters
from fr3imp.scenarios import PARAMETERS


class ProfileTests(unittest.TestCase):
    def test_f_uses_same_task_as_b(self):
        for profile in ('fr3','fr3v2','paper','legacy'):
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

    def test_fr3_adaptation_is_unchanged_by_fr3v2(self):
        b=profile_parameters('B','fr3')
        np.testing.assert_allclose(b['impedance']['inertia'],np.asarray(PARAMETERS['B'][0])*.125)
        d=profile_parameters('D','fr3')
        self.assertTrue(d['operator_roll_hold'])
        self.assertEqual(d['control_settings']['nric_gain_scale'],.25)
        self.assertEqual(default_plant_parameters('B','fr3').friction_scale,1.)


class Fr3v2ProfileTests(unittest.TestCase):
    def test_restores_paper_task_and_operator(self):
        for experiment in 'ABCDF':
            configuration=profile_parameters(experiment,'fr3v2')
            paper=profile_parameters(experiment,'paper')
            self.assertEqual(configuration['impedance'],paper['impedance'])
            for key in ('operator_stiffness_scale','operator_torque_scale','operator_roll_hold'):
                self.assertEqual(configuration[key],paper[key])
            self.assertEqual(configuration['control_settings'],{**paper['control_settings'],'ref_gain_scale':.25})

    def test_platform_assumptions_are_declared(self):
        for experiment in 'ABCDF':
            self.assertEqual(default_plant_parameters(experiment,'fr3v2').friction_scale,3.)
        b=default_plant_parameters('B','fr3v2')
        self.assertEqual((b.contact_timeconstant,b.contact_dampratio,b.contact_impedance),(.02,.4,(.95,.99,.001)))
        self.assertEqual(b,default_plant_parameters('F','fr3v2'))
        c=default_plant_parameters('C','fr3v2')
        self.assertEqual((c.contact_timeconstant,c.contact_dampratio,c.contact_impedance),(.04,.3,(.9,.95,.001)))

    def test_grinding_stays_deferred(self):
        with self.assertRaises(ValueError):profile_parameters('E','fr3v2')

    def test_comparison_shares_platform_and_records_effective_gain(self):
        with tempfile.TemporaryDirectory() as root:
            records=[run_experiment('B',name,duration=.01,seed=100,output=root,profile='fr3v2')
                     for name in ('impedance','admittance','bidirectional','smc','hinf')]
            sweep=[run_experiment('F',name,duration=.01,seed=100,output=root,profile='fr3v2')
                   for name in ('hinf_0.25','hinf_0.5','hinf_1')]
        for r in records:
            self.assertEqual(r['status'],'completed')
            for key in ('plant','sensors','impedance','initial_q','actuator'):
                self.assertEqual(r[key],records[0][key])
            self.assertEqual(r['sensors']['delay_samples'],3)
        np.testing.assert_allclose([r['controller_parameters']['alpha'] for r in sweep],[.0625,.125,.25])
        np.testing.assert_allclose(records[1]['controller_parameters']['G'],[1200,1200,600,400,400,400,400])

    def test_nric_gain_vector_overrides_paper_gains(self):
        gain=[80.,90.,140.,380.,40.,43.,30.]
        with tempfile.TemporaryDirectory() as root:
            record=run_experiment('A','admittance',duration=.01,seed=100,output=root,profile='fr3v2',
                                  control_overrides=dict(nric_gain_vector=gain))
        self.assertEqual(record['controller_parameters']['G'],gain)


if __name__=='__main__':unittest.main()
