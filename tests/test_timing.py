"""The FR3 clock must not follow the internal physics integration rate."""
import tempfile
import unittest
from pathlib import Path
import numpy as np
from fr3imp.experiment import run_experiment


class TimingTests(unittest.TestCase):
    def test_sensor_and_control_counts_with_independent_physics(self):
        with tempfile.TemporaryDirectory() as root:
            for dt, substeps in [(0.001, 1), (0.001, 4), (0.00025, 1)]:
                folder = Path(root) / f'{dt}_{substeps}'
                result = run_experiment('C', duration=.012, output=folder,
                                        control_period=dt, physics_substeps=substeps)
                self.assertEqual(result['completed_control_steps'], round(.012 / dt))
                self.assertEqual(result['force_samples'], 12)
                self.assertAlmostEqual(result['plant']['timestep'], dt / substeps)
                with np.load(next(folder.glob('*/trajectory.npz'))) as a:
                    np.testing.assert_allclose(a['time'], np.arange(12) * .001)

    def test_hybrid_contact_regression_at_one_khz(self):
        with tempfile.TemporaryDirectory() as root:
            result = run_experiment('C', seed=100, output=root,profile='legacy')
            self.assertEqual(result['status'], 'completed')
            self.assertEqual(result['completed_control_steps'], 30000)
            self.assertEqual(result['force_samples'], 30000)
            self.assertEqual(result['metrics']['saturation_fraction'], 0)
            self.assertLess(result['metrics']['force_tracking_rmse_N'], .7)
            with np.load(next(Path(root).glob('*/trajectory.npz'))) as a:
                self.assertTrue(np.all(a['board_contact'][a['time'] > .2]))
                self.assertGreater(np.count_nonzero(a['threshold_contact']), 1000)


if __name__ == '__main__':
    unittest.main()
