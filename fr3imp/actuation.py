"""Optional command-side dynamics; independent of 1 kHz sensor sampling.

The one-pole update matches libfranka 0.21.1 lowpassFilter. The MuJoCo
controller outputs total joint torque, including nominal gravity; this adapter
filters that total command. It is NOT a complete model of FCI's built-in
load/gravity/friction compensation or the real drive electronics.
https://github.com/frankarobotics/libfranka/blob/0.21.1/src/lowpass_filter.cpp
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class ActuatorParameters:
    cutoff_hz: float = 0.  # zero means bypass, retaining the historical model
    torque_rate_limit: float = 0.  # Nm/s, zero means disabled


class TorqueInterface:
    def __init__(self, parameters, dt, initial):
        if not np.isfinite(parameters.cutoff_hz) or parameters.cutoff_hz < 0:
            raise ValueError('cutoff_hz must be finite and nonnegative')
        if not np.isfinite(parameters.torque_rate_limit) or parameters.torque_rate_limit < 0:
            raise ValueError('torque_rate_limit must be finite and nonnegative')
        self.parameters=parameters;self.dt=dt
        self.previous=np.array(initial, dtype=float)

    def step(self, command):
        p=self.parameters
        if 0 < p.cutoff_hz < 1000:
            gain=self.dt/(self.dt+1/(2*np.pi*p.cutoff_hz))
            result=gain*np.asarray(command)+(1-gain)*self.previous
        else:
            result=np.asarray(command).copy()
        if p.torque_rate_limit:
            bound=p.torque_rate_limit*self.dt
            result=self.previous+np.clip(result-self.previous,-bound,bound)
        self.previous=result.copy()
        return result
