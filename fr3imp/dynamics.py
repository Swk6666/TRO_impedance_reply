"""Nominal Euler-coordinate task dynamics, including reference-velocity C.

Paper (50)-(53), with W=M_hat, hence Pi_p=(J M^-1 J.T)^-1.
R is represented relative to the fixed initial world orientation to avoid an
Euler branch cut at the FR3's upside-down home tool. Angular velocity and
physical moments are transformed as power-dual quantities.
"""
from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


def euler_rate_map(rpy):
    _, pitch, yaw = rpy
    cp, sp, cy, sy = np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cy*cp, -sy, 0.], [sy*cp, cy, 0.], [-sp, 0., 1.]])


def euler_rate_map_dot(rpy, rates):
    _, pitch, yaw = rpy
    _, dp, dy = rates
    cp, sp, cy, sy = np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[-sy*dy*cp-cy*sp*dp, -cy*dy, 0.],
                     [cy*dy*cp-sy*sp*dp, -sy*dy, 0.], [-cp*dp, 0., 0.]])


def angle_error(desired, actual):
    error = np.asarray(desired) - actual
    error[3:] = (error[3:] + np.pi) % (2*np.pi) - np.pi
    return error


@dataclass
class TaskState:
    q: np.ndarray
    v: np.ndarray
    p: np.ndarray
    pdot: np.ndarray
    J: np.ndarray
    Jdot: np.ndarray
    E: np.ndarray
    M: np.ndarray
    gravity: np.ndarray
    bias: np.ndarray
    Jplus: np.ndarray
    inertia: np.ndarray
    null: np.ndarray
    sigma_min: float

    def generalized_wrench(self, physical):
        return np.r_[physical[:3], self.E.T @ physical[3:]]


class NominalDynamics:
    """Owns only nominal MjModel/MjData; never receives the plant MjData."""
    def __init__(self, model, orientation_origin):
        self.model = model
        self.data = mujoco.MjData(model)
        self.origin = np.asarray(orientation_origin).copy()
        self.site = model.site("tcp").id
        self.body = model.site_bodyid[self.site]
        self.scratch = np.zeros(model.nv)
        self.last = None

    def _bias_at_velocity(self, velocity):
        self.data.qvel[:] = velocity
        mujoco.mj_comVel(self.model, self.data)
        mujoco.mj_rne(self.model, self.data, 0, self.scratch)
        return self.scratch.copy()

    def evaluate(self, q, v):
        m, d = self.model, self.data
        d.qpos[:] = q
        d.qvel[:] = v
        mujoco.mj_forward(m, d)
        R = d.site_xmat[self.site].reshape(3, 3)
        rpy = Rotation.from_matrix(R @ self.origin.T).as_euler("xyz")
        if abs(np.cos(rpy[1])) < .08:
            raise FloatingPointError("Euler chart approaches pitch singularity")
        E = euler_rate_map(rpy)
        Einv = np.linalg.inv(E)
        jp, jr, jdp, jdr = [np.zeros((3, m.nv)) for _ in range(4)]
        mujoco.mj_jacSite(m, d, jp, jr, self.site)
        mujoco.mj_jacDot(m, d, jdp, jdr, d.site_xpos[self.site], self.body)
        J = np.vstack((jp, Einv @ jr))
        rates = (J @ v)[3:]
        Jdot = np.vstack((jdp, Einv @ (jdr - euler_rate_map_dot(rpy, rates) @ Einv @ jr)))
        M = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, d, M)
        bias = d.qfrc_bias.copy()
        gravity = self._bias_at_velocity(np.zeros(m.nv))
        self._bias_at_velocity(v)
        sigma_min = np.linalg.svd(J, compute_uv=False)[-1]
        if sigma_min < .015:
            raise FloatingPointError(f"Task Jacobian near singularity: {sigma_min:.5g}")
        MinvJT = np.linalg.solve(M, J.T)
        inertia = np.linalg.inv(J @ MinvJT)
        Jplus = MinvJT @ inertia
        self.last = TaskState(np.array(q), np.array(v), np.r_[d.site_xpos[self.site], rpy],
                              J @ v, J, Jdot, E, M, gravity, bias, Jplus, inertia,
                              np.eye(m.nv)-Jplus @ J, float(sigma_min))
        return self.last

    def coriolis_product(self, reference_velocity):
        """C(q,v)u via polarization of Christoffel's quadratic form.

        2 C(v)u = b(v+u)-b(v)-b(u)+g. Joint armature is constant.
        The test suite independently checks this against mass derivatives.
        """
        s = self.last
        combined = self._bias_at_velocity(s.v + reference_velocity)
        reference = self._bias_at_velocity(reference_velocity)
        self._bias_at_velocity(s.v)
        return .5 * (combined - s.bias - reference + s.gravity)

    def task_coriolis_product(self, reference_task_velocity):
        s = self.last
        u = s.Jplus @ reference_task_velocity
        return s.Jplus.T @ self.coriolis_product(u) - s.inertia @ s.Jdot @ u

    def null_torque(self, posture, stiffness=12., damping=4.):
        s = self.last
        return s.null.T @ (stiffness*(posture-s.q) - damping*s.v + s.gravity)
