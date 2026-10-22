# Pioneer humanoid: physics model and motor test

The final Pioneer humanoid design (`assets/pioneer_humanoid/`) as a MuJoCo physics model, with a tool to move
each simulated motor and see what it does.

```bash
cd src/simulation/pioneer_humanoid_model
uv sync                                                  # once: mujoco, viser, numpy
uv run python motor_test.py                              # pelvis held in mid-air: every limb moves freely
uv run python motor_test.py --base stand                 # free on the floor
uv run python motor_test.py --sweep --report motor_report.md   # test every motor, no viewer
```

Open **http://localhost:8080** (on a remote machine: `ssh -N -L 8080:localhost:8080 <you>@<host>`).

## The motor test app

- **One slider per motor, grouped by body part:** torso & head (3), each arm (7, including the wrist), each
  hand (16), each leg (6). That makes 61 motors. The 8 finger DIP joints follow their PIP joint (the URDF's
  `<mimic>`) and have no motor of their own.
- **Under each slider:** the motor's actual angle and its torque against its cap. "⚠️ can't reach" means the
  joint is more than 3° from its target; "🔴 at torque cap" means the motor is at its torque limit.
- **Sweep this motor:** zero → lower limit → upper limit → zero over 3 s.
- **Sweep every motor in turn:** the same, for all 61 motors, one after another.

## How the model is built (`model.py`)

| What | How |
|---|---|
| Base | `hang`: the pelvis is fixed 0.35 m higher than standing, so the feet clear the floor. `stand`: a free joint on the floor |
| Motors | A position (PD) motor on each joint: gain 20 Nm/rad per Nm of torque cap, critically damped, torque capped at the URDF's effort limit, target limited to the joint's range |
| Finger DIP joints | An equality constraint to their PIP joint (×1) |
| Contacts | The robot's collision boxes touch only the floor. They are each link's bounding box, and neighbours overlap, so self-collision is off |
| Viewer meshes | Each link's meshes merged and simplified to a 3 mm grid: 2.76 M → 0.47 M triangles |

## Results (`motor_report.md`)

**61 of 61 motors pass** in `hang` mode: each reaches both of its limits and spends under 5% of its sweep at the
torque cap. Shoulder pitch lags the most (up to 27°, near its 10 Nm cap), because its sweep swings the whole arm
through ±180° in 3 s.

## Caveats

- **Masses, inertias, and torque and speed limits are the URDF's placeholders:** 8 kg in total, 10 Nm and
  6 rad/s on every joint. So a pass or fail means "with these values". Re-run the sweep once real motor specs
  are in the URDF.
- **Self-collision isn't simulated** (bounding boxes only). Use the viewer to see whether a pose makes parts pass
  through each other.
