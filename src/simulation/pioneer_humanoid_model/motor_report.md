# Pioneer humanoid: motor test

Each motor alone, in MuJoCo physics, base `hang`, all other motors holding zero:
- **sweep**: its target jumps zero → lower limit → upper limit → zero over 4 s (like fast slider drags);
- **step**: its target jumps from zero to half of each side of its range, held 3 s.

A motor **passes** when it reaches both limits (within 3°), never goes past a target by more than 0.5°, stays within its speed limit (+5%), and spends under 5% of the sweep at its torque cap. Limits, torque caps, speed limits and the leg masses come from `joint_specs.yaml`; *Source* says whether a joint's motor values are real, assumed or placeholders.

**61 of 61 motors pass.**

| Group | Motor | Range (°) | Torque cap (Nm) | Speed limit (rad/s) | Mass moved (kg) | Reaches min / max | Overshoot (°) | Peak speed (rad/s) | Settles in (s) | Peak torque (Nm) | Time at cap | Source | Result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Torso & head | `waist_yaw` | -90 .. 90 | 10 | 6 | 3.89 | yes / yes | 0.00 | 6.01 | 0.38 | 5.3 | 0% | real | PASS |
| Torso & head | `neck_yaw` | -70 .. 70 | 10 | 6 | 0.37 | yes / yes | 0.00 | 6.00 | 0.12 | 2.6 | 0% | real | PASS |
| Torso & head | `neck_pitch` | -20 .. 30 | 10 | 6 | 0.26 | yes / yes | 0.00 | 6.00 | 0.07 | 3.1 | 0% | real | PASS |
| Right arm | `r_shoulder_pitch` | -180 .. 180 | 53 | 6 | 1.15 | yes / yes | 0.30 | 6.13 | 0.32 | 29.3 | 0% | assumed | PASS |
| Right arm | `r_shoulder_roll` | -90 .. 90 | 53 | 6 | 0.81 | yes / yes | 0.35 | 6.16 | 0.18 | 19.1 | 0% | assumed | PASS |
| Right arm | `r_upper_arm_yaw` | -90 .. 90 | 18 | 6 | 0.63 | yes / yes | 0.00 | 6.00 | 0.13 | 2.4 | 0% | assumed | PASS |
| Right arm | `r_elbow` | -90 .. 0 | 18 | 6 | 0.42 | yes / yes | 0.23 | 6.15 | 0.19 | 5.2 | 0% | assumed | PASS |
| Right arm | `r_forearm_roll` | -90 .. 90 | 18 | 6 | 0.34 | yes / yes | 0.00 | 6.00 | 0.13 | 2.3 | 0% | assumed | PASS |
| Right arm | `r_wrist_dev` | -20 .. 30 | 0.73 | 6 | 0.26 | yes / yes | 0.00 | 5.11 | 0.15 | 0.6 | 0% | assumed | PASS |
| Right arm | `r_wrist_flex` | -60 .. 60 | 0.73 | 6 | 0.25 | yes / yes | 0.00 | 5.99 | 0.19 | 0.7 | 0% | assumed | PASS |
| Right hand | `r_abd` | -15 .. 15 | 10 | 6 | 0.04 | yes / yes | 0.00 | 6.07 | 0.04 | 2.3 | 0% | real | PASS |
| Right hand | `r_mcp` | 0 .. 90 | 10 | 6 | 0.03 | yes / yes | 0.00 | 6.07 | 0.15 | 2.3 | 0% | real | PASS |
| Right hand | `r_pip` | 0 .. 90 | 10 | 6 | 0.02 | yes / yes | 0.26 | 6.16 | 0.16 | 3.6 | 0% | real | PASS |
| Right hand | `r_i_abd` | -15 .. 15 | 10 | 6 | 0.04 | yes / yes | 0.00 | 6.07 | 0.04 | 2.3 | 0% | real | PASS |
| Right hand | `r_i_mcp` | 0 .. 90 | 10 | 6 | 0.03 | yes / yes | 0.00 | 6.07 | 0.15 | 2.3 | 0% | real | PASS |
| Right hand | `r_i_pip` | 0 .. 90 | 10 | 6 | 0.02 | yes / yes | 0.26 | 6.16 | 0.16 | 3.6 | 0% | real | PASS |
| Right hand | `r_r_abd` | -15 .. 15 | 10 | 6 | 0.04 | yes / yes | 0.00 | 6.07 | 0.04 | 2.3 | 0% | real | PASS |
| Right hand | `r_r_mcp` | 0 .. 90 | 10 | 6 | 0.03 | yes / yes | 0.00 | 6.07 | 0.15 | 2.3 | 0% | real | PASS |
| Right hand | `r_r_pip` | 0 .. 90 | 10 | 6 | 0.02 | yes / yes | 0.26 | 6.16 | 0.16 | 3.6 | 0% | real | PASS |
| Right hand | `r_l_abd` | -15 .. 15 | 10 | 6 | 0.04 | yes / yes | 0.00 | 6.07 | 0.04 | 2.3 | 0% | real | PASS |
| Right hand | `r_l_mcp` | 0 .. 90 | 10 | 6 | 0.03 | yes / yes | 0.00 | 6.07 | 0.15 | 2.3 | 0% | real | PASS |
| Right hand | `r_l_pip` | 0 .. 90 | 10 | 6 | 0.02 | yes / yes | 0.26 | 6.16 | 0.16 | 3.6 | 0% | real | PASS |
| Right hand | `r_th_c` | 0 .. 50 | 10 | 6 | 0.04 | yes / yes | 0.00 | 6.07 | 0.09 | 2.4 | 0% | real | PASS |
| Right hand | `r_th_f` | 0 .. 60 | 10 | 6 | 0.03 | yes / yes | 0.00 | 6.07 | 0.11 | 2.4 | 0% | real | PASS |
| Right hand | `r_th_mcp` | 0 .. 80 | 10 | 6 | 0.02 | yes / yes | 0.00 | 6.07 | 0.13 | 2.3 | 0% | real | PASS |
| Right hand | `r_th_ip` | 0 .. 80 | 10 | 6 | 0.01 | yes / yes | 0.00 | 6.06 | 0.13 | 2.3 | 0% | real | PASS |
| Left arm | `l_shoulder_pitch` | -180 .. 180 | 53 | 6 | 1.14 | yes / yes | 0.30 | 6.13 | 0.32 | 29.3 | 0% | assumed | PASS |
| Left arm | `l_shoulder_roll` | -90 .. 90 | 53 | 6 | 0.81 | yes / yes | 0.35 | 6.16 | 0.18 | 19.1 | 0% | assumed | PASS |
| Left arm | `l_upper_arm_yaw` | -90 .. 90 | 18 | 6 | 0.63 | yes / yes | 0.00 | 6.00 | 0.13 | 2.4 | 0% | assumed | PASS |
| Left arm | `l_elbow` | -90 .. 0 | 18 | 6 | 0.42 | yes / yes | 0.23 | 6.15 | 0.19 | 5.2 | 0% | assumed | PASS |
| Left arm | `l_forearm_roll` | -90 .. 90 | 18 | 6 | 0.34 | yes / yes | 0.00 | 6.00 | 0.13 | 2.3 | 0% | assumed | PASS |
| Left arm | `l_wrist_dev` | -20 .. 30 | 0.73 | 6 | 0.26 | yes / yes | 0.00 | 5.11 | 0.15 | 0.6 | 0% | assumed | PASS |
| Left arm | `l_wrist_flex` | -60 .. 60 | 0.73 | 6 | 0.25 | yes / yes | 0.00 | 5.99 | 0.19 | 0.7 | 0% | assumed | PASS |
| Left hand | `l_abd` | -15 .. 15 | 10 | 6 | 0.04 | yes / yes | 0.00 | 6.07 | 0.04 | 2.3 | 0% | real | PASS |
| Left hand | `l_mcp` | 0 .. 90 | 10 | 6 | 0.03 | yes / yes | 0.00 | 6.07 | 0.15 | 2.3 | 0% | real | PASS |
| Left hand | `l_pip` | 0 .. 90 | 10 | 6 | 0.02 | yes / yes | 0.26 | 6.16 | 0.16 | 3.6 | 0% | real | PASS |
| Left hand | `l_i_abd` | -15 .. 15 | 10 | 6 | 0.04 | yes / yes | 0.00 | 6.07 | 0.04 | 2.3 | 0% | real | PASS |
| Left hand | `l_i_mcp` | 0 .. 90 | 10 | 6 | 0.03 | yes / yes | 0.00 | 6.07 | 0.15 | 2.3 | 0% | real | PASS |
| Left hand | `l_i_pip` | 0 .. 90 | 10 | 6 | 0.02 | yes / yes | 0.26 | 6.16 | 0.16 | 3.6 | 0% | real | PASS |
| Left hand | `l_r_abd` | -15 .. 15 | 10 | 6 | 0.04 | yes / yes | 0.00 | 6.07 | 0.04 | 2.3 | 0% | real | PASS |
| Left hand | `l_r_mcp` | 0 .. 90 | 10 | 6 | 0.03 | yes / yes | 0.00 | 6.07 | 0.15 | 2.3 | 0% | real | PASS |
| Left hand | `l_r_pip` | 0 .. 90 | 10 | 6 | 0.02 | yes / yes | 0.26 | 6.16 | 0.16 | 3.6 | 0% | real | PASS |
| Left hand | `l_l_abd` | -15 .. 15 | 10 | 6 | 0.04 | yes / yes | 0.00 | 6.07 | 0.04 | 2.3 | 0% | real | PASS |
| Left hand | `l_l_mcp` | 0 .. 90 | 10 | 6 | 0.03 | yes / yes | 0.00 | 6.07 | 0.15 | 2.3 | 0% | real | PASS |
| Left hand | `l_l_pip` | 0 .. 90 | 10 | 6 | 0.02 | yes / yes | 0.26 | 6.16 | 0.16 | 3.6 | 0% | real | PASS |
| Left hand | `l_th_c` | 0 .. 50 | 10 | 6 | 0.04 | yes / yes | 0.00 | 6.07 | 0.09 | 2.4 | 0% | real | PASS |
| Left hand | `l_th_f` | 0 .. 60 | 10 | 6 | 0.03 | yes / yes | 0.00 | 6.07 | 0.11 | 2.4 | 0% | real | PASS |
| Left hand | `l_th_mcp` | 0 .. 80 | 10 | 6 | 0.02 | yes / yes | 0.00 | 6.07 | 0.13 | 2.3 | 0% | real | PASS |
| Left hand | `l_th_ip` | 0 .. 80 | 10 | 6 | 0.01 | yes / yes | 0.00 | 6.06 | 0.13 | 2.3 | 0% | real | PASS |
| Left leg | `Hip_F_L` | -20 .. 90 | 222 | 3.67 | 17.15 | yes / yes | 0.01 | 3.69 | 0.34 | 161.0 | 0% | real | PASS |
| Left leg | `Hip_A_L` | -10 .. 60 | 120 | 20.94 | 15.29 | yes / yes | 0.01 | 3.73 | 0.36 | 73.0 | 0% | real | PASS |
| Left leg | `Hip_R_L` | -30 .. 90 | 60 | 20.42 | 13.11 | yes / yes | 0.00 | 20.55 | 0.05 | 37.3 | 0% | real | PASS |
| Left leg | `Knee_L` | 0 .. 65 | 222 | 3.67 | 6.72 | yes / yes | 0.00 | 3.69 | 0.19 | 48.4 | 0% | real | PASS |
| Left leg | `Ankle_P_L` | -30 .. 50 | 120 | 20.42 | 2.15 | yes / yes | 0.00 | 20.42 | 0.03 | 43.2 | 0% | real | PASS |
| Left leg | `Ankle_R_L` | -15 .. 15 | 120 | 20.42 | 1.89 | yes / yes | 0.00 | 20.05 | 0.01 | 39.7 | 0% | real | PASS |
| Right leg | `Hip_F_R` | -20 .. 90 | 222 | 3.67 | 17.15 | yes / yes | 0.01 | 3.69 | 0.34 | 161.0 | 0% | real | PASS |
| Right leg | `Hip_A_R` | -10 .. 60 | 120 | 20.94 | 15.29 | yes / yes | 0.01 | 3.73 | 0.36 | 73.0 | 0% | real | PASS |
| Right leg | `Hip_R_R` | -30 .. 90 | 60 | 20.42 | 13.11 | yes / yes | 0.00 | 20.55 | 0.05 | 37.3 | 0% | real | PASS |
| Right leg | `Knee_R` | 0 .. 65 | 222 | 3.67 | 6.72 | yes / yes | 0.00 | 3.69 | 0.19 | 48.4 | 0% | real | PASS |
| Right leg | `Ankle_P_R` | -30 .. 50 | 120 | 20.42 | 2.15 | yes / yes | 0.00 | 20.42 | 0.03 | 43.2 | 0% | real | PASS |
| Right leg | `Ankle_R_R` | -15 .. 15 | 120 | 20.42 | 1.89 | yes / yes | 0.00 | 20.05 | 0.01 | 39.7 | 0% | real | PASS |
