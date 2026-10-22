# Pioneer humanoid: motor test

Each motor alone, in MuJoCo physics, base `hang`: commanded from zero to its lower limit, to its upper limit and back over 3 s, all other motors held at zero. A motor **passes** when it reaches both limits (within 3 deg) and spends under 5% of the sweep at its torque cap.

Masses, inertias and torque caps are the URDF's placeholders (see `assets/pioneer_humanoid/README.md`), so a failure means *with these values*: check it again once the real motor specs are in.

**61 of 61 motors pass.**

| Group | Motor | Range (deg) | Reaches min | Reaches max | Worst tracking error (deg) | Peak torque / cap (Nm) | Time at cap | Result |
|---|---|---|---|---|---|---|---|---|
| Torso & head | `waist_yaw` | -90 .. 90 | yes | yes | 10.0 | 5.00 / 10 | 0% | PASS |
| Torso & head | `neck_yaw` | -70 .. 70 | yes | yes | 1.7 | 2.14 / 10 | 0% | PASS |
| Torso & head | `neck_pitch` | -20 .. 30 | yes | yes | 0.6 | 0.69 / 10 | 0% | PASS |
| Right arm | `r_shoulder_pitch` | -180 .. 180 | yes | yes | 13.1 | 10.00 / 10 | 3% | PASS |
| Right arm | `r_shoulder_roll` | -90 .. 90 | yes | yes | 12.2 | 4.00 / 10 | 0% | PASS |
| Right arm | `r_upper_arm_yaw` | -90 .. 90 | yes | yes | 2.1 | 2.74 / 10 | 0% | PASS |
| Right arm | `r_elbow` | -90 .. 0 | yes | yes | 3.4 | 2.37 / 10 | 0% | PASS |
| Right arm | `r_forearm_roll` | -90 .. 90 | yes | yes | 2.0 | 2.73 / 10 | 0% | PASS |
| Right arm | `r_wrist_dev` | -20 .. 30 | yes | yes | 0.6 | 0.65 / 10 | 0% | PASS |
| Right arm | `r_wrist_flex` | -60 .. 60 | yes | yes | 1.5 | 1.67 / 10 | 0% | PASS |
| Right hand | `r_abd` | -15 .. 15 | yes | yes | 0.3 | 0.45 / 10 | 0% | PASS |
| Right hand | `r_mcp` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Right hand | `r_pip` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Right hand | `r_i_abd` | -15 .. 15 | yes | yes | 0.3 | 0.45 / 10 | 0% | PASS |
| Right hand | `r_i_mcp` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Right hand | `r_i_pip` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Right hand | `r_r_abd` | -15 .. 15 | yes | yes | 0.3 | 0.45 / 10 | 0% | PASS |
| Right hand | `r_r_mcp` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Right hand | `r_r_pip` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Right hand | `r_l_abd` | -15 .. 15 | yes | yes | 0.3 | 0.46 / 10 | 0% | PASS |
| Right hand | `r_l_mcp` | 0 .. 90 | yes | yes | 1.1 | 1.56 / 10 | 0% | PASS |
| Right hand | `r_l_pip` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Right hand | `r_th_c` | 0 .. 50 | yes | yes | 0.6 | 0.87 / 10 | 0% | PASS |
| Right hand | `r_th_f` | 0 .. 60 | yes | yes | 0.7 | 1.05 / 10 | 0% | PASS |
| Right hand | `r_th_mcp` | 0 .. 80 | yes | yes | 1.0 | 1.39 / 10 | 0% | PASS |
| Right hand | `r_th_ip` | 0 .. 80 | yes | yes | 1.0 | 1.38 / 10 | 0% | PASS |
| Left arm | `l_shoulder_pitch` | -180 .. 180 | yes | yes | 13.1 | 10.00 / 10 | 3% | PASS |
| Left arm | `l_shoulder_roll` | -90 .. 90 | yes | yes | 12.2 | 4.00 / 10 | 0% | PASS |
| Left arm | `l_upper_arm_yaw` | -90 .. 90 | yes | yes | 2.1 | 2.74 / 10 | 0% | PASS |
| Left arm | `l_elbow` | -90 .. 0 | yes | yes | 3.4 | 2.37 / 10 | 0% | PASS |
| Left arm | `l_forearm_roll` | -90 .. 90 | yes | yes | 2.0 | 2.73 / 10 | 0% | PASS |
| Left arm | `l_wrist_dev` | -20 .. 30 | yes | yes | 0.6 | 0.65 / 10 | 0% | PASS |
| Left arm | `l_wrist_flex` | -60 .. 60 | yes | yes | 1.5 | 1.67 / 10 | 0% | PASS |
| Left hand | `l_abd` | -15 .. 15 | yes | yes | 0.3 | 0.45 / 10 | 0% | PASS |
| Left hand | `l_mcp` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Left hand | `l_pip` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Left hand | `l_i_abd` | -15 .. 15 | yes | yes | 0.3 | 0.45 / 10 | 0% | PASS |
| Left hand | `l_i_mcp` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Left hand | `l_i_pip` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Left hand | `l_r_abd` | -15 .. 15 | yes | yes | 0.3 | 0.45 / 10 | 0% | PASS |
| Left hand | `l_r_mcp` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Left hand | `l_r_pip` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Left hand | `l_l_abd` | -15 .. 15 | yes | yes | 0.3 | 0.46 / 10 | 0% | PASS |
| Left hand | `l_l_mcp` | 0 .. 90 | yes | yes | 1.1 | 1.56 / 10 | 0% | PASS |
| Left hand | `l_l_pip` | 0 .. 90 | yes | yes | 1.1 | 1.57 / 10 | 0% | PASS |
| Left hand | `l_th_c` | 0 .. 50 | yes | yes | 0.6 | 0.87 / 10 | 0% | PASS |
| Left hand | `l_th_f` | 0 .. 60 | yes | yes | 0.7 | 1.05 / 10 | 0% | PASS |
| Left hand | `l_th_mcp` | 0 .. 80 | yes | yes | 1.0 | 1.39 / 10 | 0% | PASS |
| Left hand | `l_th_ip` | 0 .. 80 | yes | yes | 1.0 | 1.38 / 10 | 0% | PASS |
| Left leg | `Hip_F_L` | -20 .. 90 | yes | yes | 7.4 | 6.60 / 10 | 0% | PASS |
| Left leg | `Hip_A_L` | -10 .. 60 | yes | yes | 6.9 | 5.03 / 10 | 0% | PASS |
| Left leg | `Hip_R_L` | -30 .. 90 | yes | yes | 1.6 | 1.65 / 10 | 0% | PASS |
| Left leg | `Knee_L` | 0 .. 65 | yes | yes | 1.6 | 2.10 / 10 | 0% | PASS |
| Left leg | `Ankle_P_L` | -30 .. 50 | yes | yes | 0.8 | 1.13 / 10 | 0% | PASS |
| Left leg | `Ankle_R_L` | -15 .. 15 | yes | yes | 0.3 | 0.45 / 10 | 0% | PASS |
| Right leg | `Hip_F_R` | -20 .. 90 | yes | yes | 7.4 | 6.60 / 10 | 0% | PASS |
| Right leg | `Hip_A_R` | -10 .. 60 | yes | yes | 6.9 | 5.03 / 10 | 0% | PASS |
| Right leg | `Hip_R_R` | -30 .. 90 | yes | yes | 1.6 | 1.65 / 10 | 0% | PASS |
| Right leg | `Knee_R` | 0 .. 65 | yes | yes | 1.6 | 2.10 / 10 | 0% | PASS |
| Right leg | `Ankle_P_R` | -30 .. 50 | yes | yes | 0.8 | 1.13 / 10 | 0% | PASS |
| Right leg | `Ankle_R_R` | -15 .. 15 | yes | yes | 0.3 | 0.45 / 10 | 0% | PASS |
