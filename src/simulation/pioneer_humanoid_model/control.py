"""Motor control with no overshoot.

Each motor is a torque motor (model.py) driven by a PD computed here and capped at the
motor's torque. Two things keep it from overshooting, which a fixed PD with a jumping
target does (a 0 -> -90 deg drag of the shoulder overshot by 24 deg and rang for 0.8 s):

  * a trapezoidal reference: it moves towards the requested target at no more than the
    joint's speed limit, accelerates and brakes no harder than ACCEL_SHARE of the motor's
    torque can for the inertia it moves, nor faster than the motor's loop can track (full
    speed in at least ACCEL_BW of its time constants), and brakes more gently than it
    accelerates (BRAKE_SHARE), so it lands on the target without the joint carrying past it
  * critical damping (times DAMPING_MARGIN) from the inertia the joint actually moves now:
    its diagonal of the mass matrix (everything outboard of it plus the rotor's armature),
    refreshed every INERTIA_EVERY steps as the pose changes
  * gains the simulation step can follow. A joint feels two inertias: "locked" (the
    mass-matrix diagonal, the other joints held still) for slow moves and "free"
    (1 / diag of the inverse mass matrix, the other joints giving way) for fast ones, which
    can be much smaller (the elbow: its upper arm gives way). The damping must stay under
    KD_DT_MAX x free inertia / timestep (sized only on the locked inertia it over-corrected
    each 2 ms step and the elbow chattered at its torque cap), and still be critical for the
    locked inertia, so the stiffness is lowered until both hold (and the natural frequency
    on the free inertia stays under WN_DT_MAX / timestep). Gravity is fed forward, so a
    softer joint still holds its angle. (A real motor
    driver's loop runs ~1000x faster than this step and needs neither cap.)

torque = clip(kp (ref - q) + kd (ref_vel - qd) + [M ref_acc + bias]_joint, -cap, cap): the PD
plus inverse-dynamics feedforward: the torque every joint needs for all the planned
accelerations together (the full mass matrix, so a swinging shoulder doesn't shove the
elbow off its angle and the elbow doesn't throw the shoulder past its target) and against
gravity and velocity effects (MuJoCo's bias force), so each motor follows its plan instead
of lagging and catching up past it. The damping is part of the
capped torque, as on a real motor driver: a position actuator's damping is integrated
implicitly by MuJoCo outside the force cap, which let the hip's damping cancel most of its
222 Nm and stall the leg.
"""

from __future__ import annotations

import mujoco
import numpy as np

from model import Robot

DAMPING_MARGIN = 1.1  # a little over critical damping
ACCEL_SHARE = 0.5  # share of the torque cap the reference may use to accelerate (the rest holds against gravity)
INERTIA_EVERY = 10  # physics steps between inertia updates (100 Hz at a 1 ms step)
WN_DT_MAX = 0.25  # natural frequency x timestep the discrete PD stays well-behaved at
KD_DT_MAX = 0.8  # damping x timestep / inertia: a discrete damper is stable below 2, smooth below ~1
ACCEL_BW = 3.0  # the reference takes at least this many of the motor's time constants (1 / w_n) to reach full speed
BRAKE_SHARE = 0.5  # braking deceleration as a share of the acceleration limit


class Controller:
  def __init__(self, robot: Robot, data: mujoco.MjData):
    self.robot, self.m, self.d = robot, robot.model, data
    n = len(robot.motors)
    self.dof = np.array([robot.dofadr(j) for j in robot.motors])
    self.qadr = np.array([robot.qadr(j) for j in robot.motors])
    self.kp = robot.kp.copy()
    self.cap = self.m.actuator_forcerange[:n, 1].copy()
    self.lo, self.hi = robot.lower.copy(), robot.upper.copy()
    self.vmax = robot.speed.copy()
    self._M = np.zeros((self.m.nv, self.m.nv))
    # a motor whose joint leads mimic joints (finger PIP -> DIP) also moves their inertia
    self.followers = [(robot.motors.index(lead), robot.dofadr(f), mult) for f, (lead, mult) in robot.mimics.items()]
    self.steps = 0
    mujoco.mj_forward(self.m, self.d)  # the mass matrix must exist before the first inertia update
    self.reset()

  def reset(self):
    """Hold the current pose (targets = references = where the joints are)."""
    q = self.d.qpos[self.qadr]
    self.target = np.clip(q, self.lo, self.hi)
    self.ref = self.target.copy()
    self.ref_vel = np.zeros_like(self.ref)
    self.ref_acc = np.zeros_like(self.ref)
    self._update_gains()

  def set_target(self, i: int, value: float):
    self.target[i] = float(np.clip(value, self.lo[i], self.hi[i]))

  def inertia(self) -> np.ndarray:
    """Per motor: the inertia it moves now [kg m^2] (mass-matrix diagonal, with armature)."""
    mujoco.mj_fullM(self.m, self.d, self._M)
    return self._M[self.dof, self.dof]

  def free_inertia(self) -> np.ndarray:
    """Per motor: the inertia it moves when every other joint gives way (1 / diag of M^-1)."""
    n = len(self.dof)
    unit = np.zeros((n, self.m.nv))
    unit[np.arange(n), self.dof] = 1.0
    sol = np.zeros_like(unit)
    mujoco.mj_solveM(self.m, self.d, sol, unit)  # each row: M^-1 times a unit vector
    return 1.0 / sol[np.arange(n), self.dof]

  def _update_gains(self):
    dt = self.m.opt.timestep
    locked, free = self.inertia(), self.free_inertia()
    diag = self._M.diagonal()
    for k, fdof, mult in self.followers:
      locked[k] += mult**2 * diag[fdof]
      free[k] += mult**2 * diag[fdof]
    self.I = locked
    kd_max = KD_DT_MAX * free / dt
    kp_fits_damping = kd_max**2 / (4.0 * DAMPING_MARGIN**2 * locked)  # critical damping on `locked` within kd_max
    self.kp_eff = np.minimum.reduce([self.kp, free * (WN_DT_MAX / dt) ** 2, kp_fits_damping])
    self.kd = DAMPING_MARGIN * 2.0 * np.sqrt(self.kp_eff * locked)
    wn = np.sqrt(self.kp_eff / locked)
    self.amax = np.minimum(ACCEL_SHARE * self.cap / locked, self.vmax * wn / ACCEL_BW)

  def tick(self):
    """Call once per physics step, before mj_step: advance the references, set the torques."""
    if self.steps % INERTIA_EVERY == 0:
      self._update_gains()
    self.steps += 1
    dt = self.m.opt.timestep
    # trapezoidal reference: the fastest speed that can still brake to a stop on the target
    gap = self.target - self.ref
    v_stop = np.sqrt(2.0 * BRAKE_SHARE * self.amax * np.abs(gap))
    v_want = np.sign(gap) * np.minimum(self.vmax, v_stop)
    v_prev = self.ref_vel.copy()
    self.ref_vel += np.clip(v_want - self.ref_vel, -self.amax * dt, self.amax * dt)
    step = self.ref_vel * dt
    done = np.abs(step) >= np.abs(gap)  # would pass the target this step: land on it
    self.ref = np.where(done, self.target, self.ref + step)
    self.ref_vel = np.where(done, 0.0, self.ref_vel)
    self.ref_acc = (self.ref_vel - v_prev) / dt
    q, qd = self.d.qpos[self.qadr], self.d.qvel[self.dof]
    acc = np.zeros(self.m.nv)
    acc[self.dof] = self.ref_acc
    for k, fdof, mult in self.followers:
      acc[fdof] = mult * self.ref_acc[k]
    inv_dyn = np.zeros(self.m.nv)
    mujoco.mj_mulM(self.m, self.d, inv_dyn, acc)  # M(q) x planned accelerations
    feedforward = inv_dyn[self.dof] + self.d.qfrc_bias[self.dof]
    torque = self.kp_eff * (self.ref - q) + self.kd * (self.ref_vel - qd) + feedforward
    self.d.ctrl[: len(torque)] = np.clip(torque, -self.cap, self.cap)
