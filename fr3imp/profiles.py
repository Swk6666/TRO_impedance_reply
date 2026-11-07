"""Declared task/controller adaptations; shared by every comparison in a task.

`paper` keeps the paper's impedance matrices and feedback gains. `fr3` is a
separate simulation adaptation, not a claim of identified FR3 hardware values.
`fr3v2` restores the paper task impedances and NRIC gains; its departures are
the base H-infinity gain here and the platform assumptions in experiment.py.
"""
import numpy as np

from .scenarios import PARAMETERS


def profile_parameters(experiment, profile='fr3', c_profile='fr3'):
    if profile not in ('fr3', 'fr3v2', 'paper', 'legacy'):
        raise ValueError('profile must be fr3, fr3v2, paper or legacy')
    if profile in ('fr3', 'fr3v2') and experiment == 'E':
        raise ValueError('Experiment E is deferred by AGENTS.md; no FR3 adaptation is defined')
    L, D, K = np.asarray(PARAMETERS[experiment], dtype=float).copy()
    controls = dict(alpha=.25 if experiment == 'C' and c_profile == 'fr3' else 1.,
                    nric_gain_scale=.1 if experiment == 'C' and c_profile == 'fr3' else 1.)
    operator = dict(operator_stiffness_scale=1., operator_torque_scale=[1., 1., 1.],
                    operator_roll_hold=False)
    if profile in ('paper', 'fr3', 'fr3v2'):
        controls.update(alpha=1., nric_gain_scale=1., energy_port='physical',
                        coupling_update='stages', virtual_substeps=4)
    if profile == 'fr3v2':
        # F's suffix is relative to this base; effective alpha is saved too.
        controls['ref_gain_scale'] = .25
    if profile == 'fr3':
        # F's suffix is relative to this base; effective alpha is saved too.
        controls['ref_gain_scale'] = .25
        if experiment in ('B', 'F'):
            L *= .125
            D *= np.sqrt(.125)
        elif experiment == 'C':
            # The force axis and orientation keep their paper impedances.
            L[:2] *= .25
            D[:2] *= .5
        elif experiment == 'D':
            # Scale rotational impedance and yaw/pitch interaction together.
            # The passive roll guide remains engaged during the second phase.
            L[3:] *= .25
            D[3:] *= .25
            K[3:] *= .25
            controls['nric_gain_scale'] = .25
            operator.update(operator_stiffness_scale=4.,
                            operator_torque_scale=[1., .25, .25], operator_roll_hold=True)
    return dict(impedance=dict(inertia=L.tolist(), damping=D.tolist(), stiffness=K.tolist()),
                control_settings=controls, **operator)
