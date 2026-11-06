"""Sampled noisy feedback. Plant truth is unavailable to controller code."""
from dataclasses import dataclass
import numpy as np

from .model import DT


@dataclass(frozen=True)
class SensorParameters:
    q_std: float = 1e-5
    v_std: float = 5e-4
    force_std: float = .1
    moment_std: float = .005
    force_period: float = .001
    delay_samples: int = 0
    force_bias: tuple = (0., 0., 0., 0., 0., 0.)
    tool_calibration_mass: float = .4
    # Calibration removes static tool weight, not its dynamic inertial wrench.


class Sensors:
    def __init__(self, parameters=SensorParameters(), seed=0):
        self.parameters = parameters
        seq = np.random.SeedSequence(seed).spawn(2)
        self.joint_rng, self.force_rng = (np.random.default_rng(s) for s in seq)
        self.held_local = np.zeros(6)
        self.history = []
        self.force_sample_count = 0

    def joints(self, q, v):
        p = self.parameters
        return (q + self.joint_rng.normal(0,p.q_std,7),
                v + self.joint_rng.normal(0,p.v_std,7))

    def wrench(self, step, raw_local, measured_rotation, extra_noise=0.):
        p = self.parameters
        period = round(p.force_period / DT)
        if not np.isclose(period*DT, p.force_period):
            raise ValueError("Force period must be an integer number of control periods")
        if step % period == 0:
            # MuJoCo's transmitted force is the parent-on-child wrench.
            # Negate to obtain forces on the parent from the tool/environment.
            # Static tool-weight calibration is measured separately from the
            # nominal robot's (deliberately uncertain) dynamic model.
            gravity_local = measured_rotation.T @ np.array([0.,0.,-9.81*p.tool_calibration_mass])
            measured = -np.asarray(raw_local).copy()
            measured[:3] -= gravity_local
            measured[3:] -= np.cross([0.,0.,.06], gravity_local)
            measured[3:] -= np.cross([0.,0.,.12], measured[:3])
            std = np.r_[np.full(3,np.hypot(p.force_std,extra_noise)),np.full(3,p.moment_std)]
            measured += self.force_rng.normal(size=6)*std + p.force_bias
            self.history.append(measured)
            index = max(0,len(self.history)-1-p.delay_samples)
            self.held_local = self.history[index].copy()
            if len(self.history) > p.delay_samples+2:
                self.history.pop(0)
            self.force_sample_count += 1
        return np.r_[measured_rotation @ self.held_local[:3],
                     measured_rotation @ self.held_local[3:]]
