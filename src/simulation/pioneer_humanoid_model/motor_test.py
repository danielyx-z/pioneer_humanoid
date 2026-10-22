"""Move each simulated motor of the Pioneer humanoid and see what it does.

Browser app (viser): one slider per motor, grouped by body part. Each motor is driven
by control.py (no overshoot, its speed and torque limits from joint_specs.yaml) in MuJoCo physics; the panel shows its target, its actual
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

from control import Controller
from model import GROUPS, Robot, build_model, visual_meshes

SWEEP_S = 4.0  # one motor's sweep target: zero -> min -> max -> zero, in seconds (the motor follows at its speed limit)
STEP_HOLD_S = 3.0  # how long a step test runs
REACH_TOL = math.radians(3.0)  # a motor "reaches" a target within this
OVERSHOOT_OK = math.radians(0.5)  # pass if it never goes past its target by more than this
SAT_SHARE = 0.98  # "at its torque limit" from this share of the cap


def source_tag(robot: Robot, joint: str) -> str:
  src = robot.specs["joints"][joint]["source"]
  return "real" if "real" in src else "assumed" if "ASSUMED" in src else "placeholder"


def moved_mass(robot: Robot, joint: str) -> float:
  """Mass the motor moves: everything outboard of its joint [kg]."""
  m = robot.model
  return float(m.body_subtreemass[m.jnt_bodyid[m.joint(joint).id]])


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
    self.ctl = Controller(self.robot, self.data)
    self.reset()

  @property
  def m(self) -> mujoco.MjModel:
    return self.robot.model

  def reset(self):
    with self.lock:
      mujoco.mj_resetData(self.m, self.data)
      mujoco.mj_forward(self.m, self.data)
      self.ctl.reset()

  def step(self, seconds: float):
    with self.lock:
      for _ in range(max(1, int(round(seconds / self.m.opt.timestep)))):
        self.ctl.tick()
        mujoco.mj_step(self.m, self.data)

  def status(self, i: int) -> dict:
    """Motor i: requested target, ramped reference, actual angle and speed, torque, flags."""
    j = self.robot.motors[i]
    q = float(self.data.qpos[self.robot.qadr(j)])
    tau = float(self.data.actuator_force[i])
    cap = float(self.m.actuator_forcerange[i, 1])
    target, ref = float(self.ctl.target[i]), float(self.ctl.ref[i])
    arrived = abs(ref - target) < 1e-9  # the ramp has finished
    return dict(joint=j, target=target, ref=ref, q=q, err=target - q, qd=float(self.data.qvel[self.robot.dofadr(j)]),
                tau=tau, cap=cap, sat=abs(tau) >= SAT_SHARE * cap, stuck=arrived and abs(target - q) > REACH_TOL)  # fmt: skip


def sweep_profile(lo: float, hi: float, t: float) -> float:
  """Requested target at time t of a sweep: it jumps, like fast slider drags, from zero
  (clamped into range) to the lower limit, then the upper limit, then back to zero."""
  z = min(max(0.0, lo), hi)
  s = t / SWEEP_S
  return z if s < 0.05 else lo if s < 0.4 else hi if s < 0.8 else z


def test_motor(sim: Sim, i: int) -> dict:
  """Motor i alone (the others hold zero): a sweep across its range, then a step to half of
  each side of its range. Measures reach, overshoot, speed against its limit and torque."""
  m, j = sim.m, sim.robot.motors[i]
  lo, hi = sim.robot.lower[i], sim.robot.upper[i]
  z = min(max(0.0, lo), hi)
  res = dict(joint=j, group=sim.robot.group_of(j), lo=lo, hi=hi, cap=float(m.actuator_forcerange[i, 1]),
             speed_limit=float(sim.robot.speed[i]), mass=moved_mass(sim.robot, j), source=source_tag(sim.robot, j),
             reached_lo=False, reached_hi=False, overshoot=0.0, peak_speed=0.0, peak_tau=0.0, settle=0.0)  # fmt: skip
  sim.reset()
  dt, sat, n = 0.01, 0, 0
  for k in range(int(SWEEP_S / dt) + int(1.0 / dt)):
    sim.ctl.set_target(i, sweep_profile(lo, hi, k * dt))
    sim.step(dt)
    s = sim.status(i)
    res["reached_lo"] |= abs(s["q"] - lo) < REACH_TOL
    res["reached_hi"] |= abs(s["q"] - hi) < REACH_TOL
    res["peak_speed"] = max(res["peak_speed"], abs(s["qd"]))
    res["peak_tau"] = max(res["peak_tau"], abs(s["tau"]))
    sat += s["sat"]
    n += 1
  res["sat_share"] = sat / n
  for end in (lo, hi):  # step tests
    if abs(end - z) < 1e-6:
      continue
    goal = z + 0.5 * (end - z)
    sign = 1.0 if goal > z else -1.0
    sim.reset()
    sim.ctl.set_target(i, goal)
    last_out = 0.0
    for k in range(int(STEP_HOLD_S / dt)):
      sim.step(dt)
      s = sim.status(i)
      res["overshoot"] = max(res["overshoot"], sign * (s["q"] - goal))
      res["peak_speed"] = max(res["peak_speed"], abs(s["qd"]))
      if abs(s["q"] - goal) > math.radians(1.0):
        last_out = (k + 1) * dt
    res["settle"] = max(res["settle"], last_out)
  res["passed"] = (res["reached_lo"] and res["reached_hi"] and res["overshoot"] <= OVERSHOOT_OK
                   and res["sat_share"] < 0.05 and res["peak_speed"] <= 1.05 * res["speed_limit"])  # fmt: skip
  return res


def write_limits(robot: Robot, path: Path) -> str:
  """JOINT_LIMITS.md: every joint's limits, torque, speed and where they come from (from joint_specs.yaml)."""
  specs = robot.specs["joints"]
  lines = [
    "# Pioneer humanoid: joint limits",
    "",
    "Generated from `joint_specs.yaml` by `uv run python motor_test.py --limits JOINT_LIMITS.md`: edit the YAML, "
    "not this table. *Mass moved* is everything outboard of the joint, with the model's current masses.",
    "",
    "| Group | Joint | Lower (°) | Upper (°) | Torque cap (Nm) | Speed limit (rad/s) | Rotor inertia (kg·m²) | Mass moved (kg) | Specs source | Limits source |",
    "|---|---|---|---|---|---|---|---|---|---|",
  ]
  for j in robot.motors:
    s = specs[j]
    lines.append(f"| {robot.group_of(j)} | `{j}` | {s['lower']:g} | {s['upper']:g} | {s['torque']:g} | {s['speed']:g} | "
                 f"{s['armature']:g} | {moved_mass(robot, j):.2f} | {s['source']} | {s['limits_source']} |")  # fmt: skip
  if robot.mimics:
    lines += ["", "**Finger DIP joints** (no motor; each follows its PIP joint, URDF `<mimic>`): "
              + ", ".join(f"`{f}` = {mult:g} × `{lead}`" for f, (lead, mult) in sorted(robot.mimics.items())) + "."]  # fmt: skip
  masses = robot.specs.get("links", {})
  if masses:
    lines += ["", "**Link masses set in `joint_specs.yaml`** (the rest are the URDF's placeholders): "
              + ", ".join(f"`{k}` {v['mass']:g} kg" for k, v in masses.items()) + "."]  # fmt: skip
  lines += ["", f"Total robot mass: {robot.model.body_subtreemass[0]:.1f} kg."]
  path.write_text("\n".join(lines) + "\n")
  return f"{len(robot.motors)} joints"


def write_report(results: list[dict], base: str, path: Path) -> str:
  ok = [r for r in results if r["passed"]]
  lines = [
    "# Pioneer humanoid: motor test",
    "",
    f"Each motor alone, in MuJoCo physics, base `{base}`, all other motors holding zero:",
    f"- **sweep**: its target jumps zero → lower limit → upper limit → zero over {SWEEP_S:.0f} s (like fast slider drags);",
    f"- **step**: its target jumps from zero to half of each side of its range, held {STEP_HOLD_S:.0f} s.",
    "",
    f"A motor **passes** when it reaches both limits (within {math.degrees(REACH_TOL):.0f}°), never goes past a target "
    f"by more than {math.degrees(OVERSHOOT_OK):.1f}°, stays within its speed limit (+5%), and spends under 5% of the "
    "sweep at its torque cap. Limits, torque caps, speed limits and the leg masses come from `joint_specs.yaml`; "
    "*Source* says whether a joint's motor values are real, assumed or placeholders.",
    "",
    f"**{len(ok)} of {len(results)} motors pass.**",
    "",
    "| Group | Motor | Range (°) | Torque cap (Nm) | Speed limit (rad/s) | Mass moved (kg) | Reaches min / max | "
    "Overshoot (°) | Peak speed (rad/s) | Settles in (s) | Peak torque (Nm) | Time at cap | Source | Result |",
    "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
  ]
  for r in results:
    lines.append(
      f"| {r['group']} | `{r['joint']}` | {math.degrees(r['lo']):.0f} .. {math.degrees(r['hi']):.0f} | {r['cap']:g} | "
      f"{r['speed_limit']:g} | {r['mass']:.2f} | {'yes' if r['reached_lo'] else '**no**'} / {'yes' if r['reached_hi'] else '**no**'} | "
      f"{math.degrees(r['overshoot']):.2f} | {r['peak_speed']:.2f} | {r['settle']:.2f} | {r['peak_tau']:.1f} | "
      f"{100 * r['sat_share']:.0f}% | {r['source']} | {'PASS' if r['passed'] else '**FAIL**'} |"
    )
  path.write_text("\n".join(lines) + "\n")
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
        lo, hi = np.degrees(robot.lower[i]), np.degrees(robot.upper[i])
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
            sim.ctl.set_target(i, math.radians(s.value))

  def zero_all():
    state["sweep"] = None
    for i, s in enumerate(sliders):
      lo, hi = robot.lower[i], robot.upper[i]
      s.value = math.degrees(float(np.clip(0.0, lo, hi)))
      sim.ctl.set_target(i, math.radians(s.value))

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
      lo, hi = robot.lower[i], robot.upper[i]
      t = time.time() - start
      sim.ctl.set_target(i, sweep_profile(lo, hi, t))
      sliders[i].value = round(math.degrees(sim.ctl.target[i]) * 2) / 2
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
        mark = " ⚠️ can't reach" if s["stuck"] else ""
        mark += " 🔴 at torque cap" if s["sat"] else ""
        readouts[i].content = (
          f"actual **{math.degrees(s['q']):+.1f}°** · {s['qd']:+.1f} / {robot.speed[i]:g} rad/s · "
          f"torque {s['tau']:+.1f} / {s['cap']:g} Nm{mark}  \n"
          f"<small>moves {moved_mass(robot, s['joint']):.2f} kg · specs: {source_tag(robot, s['joint'])}</small>"
        )
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
  ap.add_argument("--limits", metavar="FILE", help="write the joint limits table (e.g. JOINT_LIMITS.md) and exit")
  args = ap.parse_args()
  robot = build_model(args.base)
  if args.limits:
    print(write_limits(robot, Path(args.limits)), "->", args.limits)
    return
  if args.sweep:
    sim = Sim(robot)
    results = []
    for i, j in enumerate(robot.motors):
      r = test_motor(sim, i)
      results.append(r)
      print(f"{j:17s} reach {r['reached_lo']!s:5s}/{r['reached_hi']!s:5s} overshoot {math.degrees(r['overshoot']):5.2f} deg  "
            f"peak speed {r['peak_speed']:5.2f}/{r['speed_limit']:g}  torque {r['peak_tau']:6.1f}/{r['cap']:g}  "
            f"at cap {100 * r['sat_share']:.0f}%  {'PASS' if r['passed'] else 'FAIL'}", flush=True)
    print(write_report(results, args.base, Path(args.report)), "->", args.report)
  else:
    run_app(robot, args.port)


if __name__ == "__main__":
  main()
