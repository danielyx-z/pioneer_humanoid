"""Move each simulated motor of the Pioneer humanoid and see what it does.

Browser app (viser): one slider per motor, grouped by body part. Each motor is a
position-controlled joint in MuJoCo physics; the panel shows its target, its actual
angle and its torque, and flags a motor that can't reach its target or sits at its
torque limit. Buttons sweep one motor, or every motor in turn, across its range.

  uv sync                                    # once
  uv run python motor_test.py                # pelvis held in mid-air
  uv run python motor_test.py --base stand   # free, on the floor
  # then open http://localhost:8080 (forward the port from a remote machine)

Headless test of every motor, written to a report:

  uv run python motor_test.py --sweep --report motor_report.md
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from model import GROUPS, Robot, build_model, visual_meshes

SWEEP_S = 3.0  # one motor's sweep: zero -> min -> max -> zero, in seconds
REACH_TOL = math.radians(3.0)  # a motor "reaches" a target within this
SAT_SHARE = 0.98  # "at its torque limit" from this share of the cap


# --------------------------------------------------------------------------- #
# Physics side, shared by the app and the headless sweep
# --------------------------------------------------------------------------- #


@dataclass
class Sim:
  robot: Robot
  data: mujoco.MjData = field(init=False)
  lock: threading.Lock = field(default_factory=threading.Lock)

  def __post_init__(self):
    self.data = mujoco.MjData(self.robot.model)
    self.reset()

  @property
  def m(self) -> mujoco.MjModel:
    return self.robot.model

  def reset(self):
    with self.lock:
      mujoco.mj_resetData(self.m, self.data)
      self.data.ctrl[:] = np.clip(0.0, self.m.actuator_ctrlrange[:, 0], self.m.actuator_ctrlrange[:, 1])
      mujoco.mj_forward(self.m, self.data)

  def step(self, seconds: float):
    with self.lock:
      for _ in range(max(1, int(seconds / self.m.opt.timestep))):
        mujoco.mj_step(self.m, self.data)

  def status(self, i: int) -> dict:
    """Motor i: target, actual angle, torque, and whether it is stuck or saturated."""
    j = self.robot.motors[i]
    q = float(self.data.qpos[self.robot.qadr(j)])
    tau = float(self.data.actuator_force[i])
    cap = float(self.m.actuator_forcerange[i, 1])
    target = float(self.data.ctrl[i])
    return dict(joint=j, target=target, q=q, err=target - q, tau=tau, cap=cap, sat=abs(tau) >= SAT_SHARE * cap)


def sweep_profile(lo: float, hi: float, t: float) -> float:
  """Target at time t of a sweep: zero (clamped into range) -> lo -> hi -> zero."""
  z = min(max(0.0, lo), hi)
  pts = [(0.0, z), (0.2, lo), (0.65, hi), (1.0, z)]
  s = min(max(t / SWEEP_S, 0.0), 1.0)
  for (s0, v0), (s1, v1) in zip(pts, pts[1:]):
    if s <= s1:
      w = (s - s0) / (s1 - s0)
      w = w * w * (3 - 2 * w)  # smooth
      return v0 + (v1 - v0) * w
  return z


def test_motor(sim: Sim, i: int) -> dict:
  """Sweep motor i alone (others held at zero) and measure how well it follows."""
  sim.reset()
  lo, hi = sim.m.actuator_ctrlrange[i]
  worst_err, peak_tau, reached_lo, reached_hi, sat_steps, n = 0.0, 0.0, False, False, 0, 0
  dt = 0.01
  for k in range(int(SWEEP_S / dt) + int(0.5 / dt)):
    sim.data.ctrl[i] = sweep_profile(lo, hi, k * dt)
    sim.step(dt)
    s = sim.status(i)
    worst_err = max(worst_err, abs(s["err"]))
    peak_tau = max(peak_tau, abs(s["tau"]))
    reached_lo |= abs(s["q"] - lo) < REACH_TOL
    reached_hi |= abs(s["q"] - hi) < REACH_TOL
    sat_steps += s["sat"]
    n += 1
  return dict(
    joint=sim.robot.motors[i], group=sim.robot.group_of(sim.robot.motors[i]), lo=lo, hi=hi,
    reached_lo=reached_lo, reached_hi=reached_hi, worst_err=worst_err, peak_tau=peak_tau,
    cap=float(sim.m.actuator_forcerange[i, 1]), sat_share=sat_steps / n,
  )  # fmt: skip


def write_report(results: list[dict], base: str, path: Path) -> str:
  ok = [r for r in results if r["reached_lo"] and r["reached_hi"] and r["sat_share"] < 0.05]
  lines = [
    "# Pioneer humanoid: motor test",
    "",
    f"Each motor alone, in MuJoCo physics, base `{base}`: commanded from zero to its lower limit, to its upper limit "
    f"and back over {SWEEP_S:.0f} s, all other motors held at zero. A motor **passes** when it reaches both limits "
    f"(within {math.degrees(REACH_TOL):.0f} deg) and spends under 5% of the sweep at its torque cap.",
    "",
    "Masses, inertias and torque caps are the URDF's placeholders (see `assets/pioneer_humanoid/README.md`), so a "
    "failure means *with these values*: check it again once the real motor specs are in.",
    "",
    f"**{len(ok)} of {len(results)} motors pass.**",
    "",
    "| Group | Motor | Range (deg) | Reaches min | Reaches max | Worst tracking error (deg) | Peak torque / cap (Nm) | Time at cap | Result |",
    "|---|---|---|---|---|---|---|---|---|",
  ]
  for r in results:
    passed = r in ok
    lines.append(
      f"| {r['group']} | `{r['joint']}` | {math.degrees(r['lo']):.0f} .. {math.degrees(r['hi']):.0f} | "
      f"{'yes' if r['reached_lo'] else '**no**'} | {'yes' if r['reached_hi'] else '**no**'} | "
      f"{math.degrees(r['worst_err']):.1f} | {r['peak_tau']:.2f} / {r['cap']:.0f} | {100 * r['sat_share']:.0f}% | "
      f"{'PASS' if passed else '**FAIL**'} |"
    )
  text = "\n".join(lines) + "\n"
  path.write_text(text)
  return f"{len(ok)} of {len(results)} motors pass"


# --------------------------------------------------------------------------- #
# Browser app
# --------------------------------------------------------------------------- #


def run_app(robot: Robot, port: int):
  import viser

  sim = Sim(robot)
  m = robot.model
  server = viser.ViserServer(port=port, label="Pioneer humanoid motor test")
  server.scene.set_up_direction("+z")
  server.scene.add_grid("/floor", width=3, height=3, cell_size=0.25, plane="xy")

  handles = {}
  for b, (v, f, rgb) in visual_meshes(robot).items():
    handles[b] = server.scene.add_mesh_simple(f"/robot/{m.body(b).name}", vertices=v, faces=f, color=rgb, flat_shading=False)

  # GUI: one slider per motor, by group
  sliders, readouts = [], []
  state = {"sweep": None}  # (motor indices queue, start time)
  with server.gui.add_folder("Test"):
    status_md = server.gui.add_markdown("Move a slider, or sweep.")
    pick = server.gui.add_dropdown("Motor to sweep", options=tuple(robot.motors), initial_value=robot.motors[0])
    b_one = server.gui.add_button("Sweep this motor")
    b_all = server.gui.add_button("Sweep every motor in turn")
    b_stop = server.gui.add_button("Stop sweep")
    b_zero = server.gui.add_button("All to zero")
  by_group: dict[str, list[int]] = {}
  for i, j in enumerate(robot.motors):
    by_group.setdefault(robot.group_of(j), []).append(i)
  for g in list(GROUPS) + ["Other"]:
    if g not in by_group:
      continue
    with server.gui.add_folder(g, expand_by_default=g in ("Torso & head", "Left arm", "Left leg")):
      for i in by_group[g]:
        lo, hi = np.degrees(m.actuator_ctrlrange[i])
        init = float(np.clip(0.0, lo, hi))
        s = server.gui.add_slider(robot.motors[i], min=float(lo), max=float(hi), step=0.5, initial_value=init)
        while len(sliders) <= i:
          sliders.append(None)
          readouts.append(None)
        sliders[i] = s
        readouts[i] = server.gui.add_markdown("")

        @s.on_update
        def _(_, i=i, s=s):
          if state["sweep"] is None:
            sim.data.ctrl[i] = math.radians(s.value)

  def zero_all():
    state["sweep"] = None
    for i, s in enumerate(sliders):
      lo, hi = m.actuator_ctrlrange[i]
      s.value = math.degrees(float(np.clip(0.0, lo, hi)))
      sim.data.ctrl[i] = math.radians(s.value)

  b_zero.on_click(lambda _: zero_all())
  b_stop.on_click(lambda _: state.update(sweep=None))
  b_one.on_click(lambda _: (zero_all(), state.update(sweep=([robot.motors.index(pick.value)], time.time()))))
  b_all.on_click(lambda _: (zero_all(), state.update(sweep=(list(range(len(robot.motors))), time.time()))))

  print(f"Open http://localhost:{port}  ({len(robot.motors)} motors, base '{robot.base}')", flush=True)
  frame, last_text = 0, 0.0
  while True:
    t0 = time.time()
    sw = state["sweep"]
    if sw is not None:  # drive the current motor of the sweep
      queue, start = sw
      i = queue[0]
      lo, hi = m.actuator_ctrlrange[i]
      t = time.time() - start
      sim.data.ctrl[i] = sweep_profile(lo, hi, t)
      sliders[i].value = round(math.degrees(sim.data.ctrl[i]) * 2) / 2
      if t > SWEEP_S:
        queue.pop(0)
        state["sweep"] = (queue, time.time()) if queue else None
    sim.step(1 / 60)
    with sim.lock:
      for b, h in handles.items():
        h.position = sim.data.xpos[b]
        h.wxyz = sim.data.xquat[b]
    if time.time() - last_text > 0.25:  # readouts at 4 Hz
      last_text = time.time()
      flagged = []
      for i in range(len(robot.motors)):
        s = sim.status(i)
        stuck = abs(s["err"]) > REACH_TOL
        mark = " ⚠️ can't reach" if stuck else ""
        mark += " 🔴 at torque cap" if s["sat"] else ""
        readouts[i].content = f"actual **{math.degrees(s['q']):+.1f}°**, torque {s['tau']:+.2f} / {s['cap']:.0f} Nm{mark}"
        if mark:
          flagged.append(f"`{s['joint']}`{mark}")
      sweeping = f"Sweeping **{robot.motors[state['sweep'][0][0]]}**. " if state["sweep"] else ""
      status_md.content = sweeping + ("All motors on target." if not flagged else "Flagged: " + ", ".join(flagged[:8]))
    frame += 1
    time.sleep(max(0.0, 1 / 60 - (time.time() - t0)))


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--base", choices=("hang", "stand"), default="hang", help="hang: pelvis fixed in mid-air; stand: free on the floor")
  ap.add_argument("--port", type=int, default=8080)
  ap.add_argument("--sweep", action="store_true", help="test every motor headless instead of opening the app")
  ap.add_argument("--report", default="motor_report.md", help="where --sweep writes its report")
  args = ap.parse_args()
  robot = build_model(args.base)
  if args.sweep:
    sim = Sim(robot)
    results = []
    for i, j in enumerate(robot.motors):
      r = test_motor(sim, i)
      results.append(r)
      flag = "" if r["reached_lo"] and r["reached_hi"] and r["sat_share"] < 0.05 else "  <-- FAIL"
      print(f"{j:18s} reached min {r['reached_lo']!s:5s} max {r['reached_hi']!s:5s} worst err {math.degrees(r['worst_err']):5.1f} deg  peak torque {r['peak_tau']:5.2f}/{r['cap']:.0f} Nm{flag}", flush=True)
    print(write_report(results, args.base, Path(args.report)), "->", args.report)
  else:
    run_app(robot, args.port)


if __name__ == "__main__":
  main()
