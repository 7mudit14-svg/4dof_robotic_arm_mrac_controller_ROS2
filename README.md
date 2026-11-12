<div align="center">

# 4-DOF Robotic Arm — Robust Direct MRAC in ROS 2

**Adaptive control of a 4-DOF manipulator using Model Reference Adaptive Control (MRAC), simulated end-to-end in ROS 2 Jazzy + Gazebo Harmonic, with MoveIt 2 for trajectory planning.**

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![MoveIt2](https://img.shields.io/badge/MoveIt-2-blue)
![ros2_control](https://img.shields.io/badge/ros2__control-controller%20plugin-blueviolet)
![License](https://img.shields.io/badge/license-MIT-green)
![Language](https://img.shields.io/badge/C%2B%2B-Controller-00599C?logo=c%2B%2B)
![Language](https://img.shields.io/badge/Python-Commander%20%26%20Monitor-3776AB?logo=python&logoColor=white)

<img src="docs/assets/arm_render.png" width="600" alt="4-DOF robotic arm render">

</div>

---

## Overview

Fixed-gain PID controllers degrade on robotic manipulators once payload, friction, or gravitational loading change — no single set of gains handles all operating conditions. This project uses an adaptive controller built from two cooperating parts:

- **Direct MRAC** adjusts feedback and feedforward gains online so that each joint's response tracks a chosen second-order reference model.
- A **robust (uncertainty) term** learns the torques the gain adaptation cannot represent — gravity, Coulomb friction and constant offsets — as weights on a small physics-based basis.

**MoveIt 2** plans the path. The planned waypoints are interpolated with cubic Hermite segments into a smooth desired position, velocity and acceleration for each joint, and the **MRAC controller** turns those into joint torques at **1 kHz**. The controller is a `ros2_control` controller plugin, so it runs inside the controller manager in step with the simulation.

Built as a Bachelor's Thesis Project (BTP) at LNMIIT, ECE Department.

---

## Demo

Clips recorded with the first version of the controller (see [What changed](#what-changed-from-the-first-version)):

| 1st Training Loop | 4th Training Loop |
|:---:|:---:|
| ![1st training loop](docs/assets/loop_1.gif) | ![4th training loop](docs/assets/loop_4.gif) |

---

## Controller design

### Reference model

Each joint should behave like

```
G_ref(s) = ω_n² / (s² + 2ζω_n·s + ω_n²)
```

with state `x_m = [q_m; q̇_m]`, `A_m = [[0, 1], [−ω_n², −2ζω_n]]`, `B_m = [0; ω_n²]`. The matrix `P` solves the Lyapunov equation `P·A_m + A_mᵀ·P = −I` and is computed analytically from `ω_n` and `ζ`.

### Direct MRAC (multi-state form)

A torque-driven joint has two states, position and velocity, so the plant state is `x = [q; q̇]` (8 values for 4 joints) and the gains are matrices:

```
u    = K̂_x · x + K̂_r · r              K̂_x: 4×8 = [Kp | Kd],   K̂_r: 4×4
e    = x − x_m
s    = sign(b) · Bᵀ P e                 per joint: p12·e + p22·ė
K̂̇_x = −Γ_x · s · xᵀ − σ (K̂_x − K̂_x0)
K̂̇_r = −Γ_r · s · rᵀ − σ (K̂_r − K̂_r0)
```

This is the multi-state extension of the scalar law `k̇_x = −Γ·e·sign(b)·x`, obtained from the Lyapunov function `V = eᵀPe + tr(K̃_xᵀΓ_x⁻¹K̃_x) + tr(K̃_rᵀΓ_r⁻¹K̃_r)`.

- With `decentralized: true` (default) each joint adapts only its own position gain `Kp`, velocity gain `Kd` and reference gain `Kr`. With `false` the full matrices adapt, including cross-joint terms.
- `K̂_r` adapts only while a trajectory is executing; during a hold `r` is constant.
- Leakage (`σ`) pulls the estimates toward their initial or loaded values, not toward zero. It is off by default.

### Robust term

```
τ   = u − Ŵᵀ·α(q, q̇)
Ŵ̇  = Γ_W · α(q, q̇) · sᵀ − σ_W (Ŵ − Ŵ_0)
```

`α` is a 7-term basis taken from the arm's physics. Each joint uses only the terms that act on it:

| Basis term | Physical effect | Used by |
|---|---|---|
| `1` | constant offset | all joints |
| `sin(q2)` | gravity on the upper arm | joint 2 |
| `sin(q2+q3)` | gravity on the forearm (joints 2 and 3 have parallel axes) | joints 2 and 3 |
| `tanh(q̇_i)` | Coulomb friction, smooth approximation | joint *i* |

Viscous friction is proportional to `q̇`, which the velocity gain `Kd` already covers, so it is not repeated in the basis. A payload change alters the gravity *weights*, not the basis.

### Reference pre-compensation

The reference model is a low-pass filter, so feeding it the planned position directly would make it lag the plan. The planned trajectory is therefore pre-compensated with the inverse of the model:

```
r = q_d + (2ζ/ω_n)·q̇_d + (1/ω_n²)·q̈_d
```

`q_d`, `q̇_d`, `q̈_d` come from the Hermite sampler. With this, `q_m` follows `q_d` without lag.

### Bounds and safeguards

- **Per-joint projection bounds** `Kp_max`, `Kd_max`, `W_max`. A sampled control loop is only stable while roughly `|Kp|·T²/J + |Kd|·T/J < 1` (`T` = control period, `J` = joint inertia). The waist (arm upright) and the wrist are very light, so their bounds are small.
- **Torque limits** match the URDF (`150, 150, 100, 50` Nm). Adaptation of a joint pauses while its torque is saturated.
- **Control rate is 1 kHz.** At 100 Hz the velocity feedback destabilises the two light joints.

### Block diagram

<img src="docs/assets/robust_mrac_controller.png" width="800" alt="Robust direct MRAC controller block diagram">

The structure is unchanged from the first version (reference model, gain adaptation, weight adaptation, `u_ad` added to the control law); the signals feeding the adaptation laws are now `s = BᵀPe` and the full state `[q; q̇]`.

---

## System architecture

```
test_moveit.py / mrac_trainer.py ──MoveGroup action──► move_group (MoveIt 2, OMPL)
                                                           │ FollowJointTrajectory
                                                           ▼
                              mrac_arm_controller  (ros2_control plugin, 1 kHz)
                                Hermite sampler → q_d, q̇_d, q̈_d → pre-compensation → r
                                reference model → e → adaptation of K̂x, K̂r, Ŵ
                                τ = K̂x·[q; q̇] + K̂r·r − Ŵᵀα
                                                           │ effort command interfaces
                                                           ▼
                                        gz_ros2_control → Gazebo Harmonic
                                                           │ position / velocity state interfaces
                                                           └────► back to the controller,
                                                                  joint_state_broadcaster → MoveIt, RViz
```

| Layer | Responsibility |
|---|---|
| **Motion planning** (`/move_group`) | MoveIt 2 / OMPL — trajectory planning, KDL kinematics |
| **Control** (`mrac_arm_controller`) | C++ `ros2_control` controller plugin, 1 kHz — Direct MRAC + robust term |
| **Simulation** (Gazebo) | Physics, joint state feedback through `gz_ros2_control` |
| **Visualization** (RViz) | Live robot state and planning scene |

`docs/assets/system_architecture.png` and `docs/assets/rqt_graph-10.png` show the first version, in which the controller was a separate node publishing to a forwarding effort controller.

### Hardware model

<img src="docs/assets/joint_layout.png" width="500" alt="4-DOF joint layout">

| Joint | Function | Axis | Effort limit |
|---|---|---|---|
| J1 — Waist | Base rotation | vertical | 150 Nm |
| J2 — Shoulder | Proximal lift | horizontal | 150 Nm |
| J3 — Elbow | Mid-link flex | horizontal, parallel to J2 | 100 Nm |
| J4 — Wrist | Roll about the forearm | along the link | 50 Nm |

Full URDF/Xacro specification — inertia tensors, mass distribution, joint limits, damping and friction — in `my_robot_description/urdf/`.

---

## Results

Evaluation: `ros2 run my_robot_commander_py test_moveit` with its defaults — four Home → Pose 1 → Pose 2 → Home loops, 10 s hold at each pose, MoveIt velocity/acceleration scaling 0.05, weights starting from their initial values, no payload, Gazebo headless. All 16 moves completed. Error is plant position minus reference model, in rad; the error against the planned trajectory is the same to four decimals.

| Metric | Joint 1 | Joint 2 | Joint 3 | Joint 4 |
|---|---|---|---|---|
| Loop 1 — max \|e\| | 0.0077 | 0.0484 | 0.1113 | 0.0046 |
| Loop 1 — mean \|e\| | 0.0029 | 0.0049 | 0.0069 | 0.0017 |
| Loop 4 — max \|e\| | 0.0106 | 0.0134 | 0.0080 | 0.0063 |
| Loop 4 — mean \|e\| | 0.0044 | 0.0019 | 0.0029 | 0.0023 |
| Final e after the last hold | 0.0002 | −0.0003 | −0.0002 | 0.0001 |

- Joints 2 and 3 carry the gravity load and improve from loop to loop as the gravity weights are learned. Joint 3 showed a brief 0.147 rad peak in loop 2 before settling below 0.01 rad in loops 3 and 4.
- Joints 1 and 4 carry no gravity. Their mean error stays below 0.005 rad and does not improve over the loops.

**Comparison with the first version** (same pose loop, values from the earlier README):

| Metric | Joint 1 | Joint 2 | Joint 3 | Joint 4 |
|---|---|---|---|---|
| First version — final e | 0.0100 | 0.0339 | 0.0055 | 0.0151 |
| Current — final e | 0.0002 | 0.0003 | 0.0002 | 0.0001 |
| First version — mean \|e\| | 0.0075 | 0.0342 | 0.0158 | 0.0049 |
| Current — mean \|e\|, loop 4 | 0.0044 | 0.0019 | 0.0029 | 0.0023 |

**Saved weights.** After 8 random training poses (`mrac_trainer`), the weights were saved, the simulation restarted and the weights loaded. In the first evaluation loop joint 3 improved (max 0.111 → 0.019, mean 0.0069 → 0.0033) and so did joint 2's mean (0.0049 → 0.0035); joint 2's max was slightly worse (0.048 → 0.057). This is a single run.

**Not yet measured:** payload variation, and plots for the current controller. The plot images in `docs/assets/` (`mrac_e_mrac.png`, `mrac_Kx.png`, `mrac_Kr.png`, `mrac_W_hat.png`, `mrac_lyapunov.png`) were produced by the first version. Regenerate them with `mrac_monitor` (press `s` to save).

---

## What changed from the first version

| Area | First version | Current version |
|---|---|---|
| Control law | `τ = K̂x·q + K̂r·r`, one scalar gain per joint, position only | `τ = K̂x·[q; q̇] + K̂r·r` with matrix gains; each joint has `Kp`, `Kd`, `Kr` |
| Gain adaptation | driven by the position error `e` | driven by `s = BᵀPe = p12·e + p22·ė`, as the Lyapunov derivation for a two-state plant requires |
| Basis `α` | 22 terms shared by all joints (`q`, `sin`, `cos`, `q_i·q_j`, `q̇`, `tanh`) | 7 physics terms with a per-joint mask, including `sin(q2+q3)` |
| Reference | planned position fed straight into the reference model (lagging the plan) | Hermite `q_d, q̇_d, q̈_d` pre-compensated so the model follows the plan |
| Execution | standalone node on a 1 ms wall timer, publishing to a `ForwardCommandController` at 100 Hz | `ros2_control` controller plugin at 1 kHz, using the measured period and simulation time |
| Startup | controller started by a 35 s timer; arm unpowered until then | spawned right after the joint state broadcaster; arm held within seconds |
| Leakage | toward zero (left a steady error) | toward the initial / loaded values, off by default |
| Bounds | one global bound on gains, torque clamp above the URDF limits | per-joint `Kp_max`, `Kd_max`, `W_max`; torque limits equal to the URDF; adaptation pauses in saturation |
| Trajectory action | always reported success 0.5 s after the planned end; cancel and preemption not handled; no goal validation | success only inside `goal_tolerance`, abort after `goal_time_tolerance`; cancel holds position; new goal preempts; malformed goals rejected |
| End of trajectory | one control tick with `r = 0` (torque spike) | sampler returns the final point after the end |
| Weights | lost on restart | `save_weights` / `load_weights` services and `load_weights_on_activate` |
| Training | fixed three-pose loop only | plus `mrac_trainer`: random reachable poses at random speeds |
| Evaluation script | settings hard-coded in the file | ROS parameters; prints max / mean / final error per joint per loop |
| Diagnostics | `/mrac/lyapunov_V` = `eᵀPe + ‖Ŵ‖²/Γ` | `/mrac/lyapunov_V` = `eᵀPe`; added `/mrac/tau`; `/mrac/traj_error` is `q − q_d` |
| Tests | none | 12 unit tests of the control maths (`test/test_mrac_core.cpp`) |

The first implementation is kept in `mrac_controller/src/mrac_controller.cpp` for reference. It still builds as the `mrac_controller` executable but is not launched.

---

## Repository structure

```
.
├── mrac_controller/                 # MRAC controller (ros2_control plugin, 1 kHz)
│   ├── include/mrac_controller/
│   │   ├── mrac_core.hpp                # control law, adaptation, reference model, basis (no ROS)
│   │   ├── trajectory_sampler.hpp       # Hermite sampling → q_d, q̇_d, q̈_d
│   │   ├── weights_io.hpp               # save / load of the adaptive weights (YAML)
│   │   └── mrac_controller_plugin.hpp
│   ├── src/mrac_controller_plugin.cpp   # ROS wiring: interfaces, action server, services, topics
│   ├── src/robust_direct_mrac_params.yaml   # every tunable, commented
│   ├── src/mrac_controller.cpp          # first version (standalone node), not launched
│   ├── mrac_plugin.xml                  # pluginlib description
│   ├── test/test_mrac_core.cpp          # unit tests
│   └── MRAC_TUNING_GUIDE.md             # written for the first version
│
├── my_robot_description/            # URDF/Xacro robot model
│   ├── urdf/                            # arm, gripper, ros2_control, common properties
│   ├── launch/display.launch.xml        # RViz-only viewer (no Gazebo)
│   └── DAMPING_FRICTION_GUIDE.md
│
├── my_robot_moveit_config/          # MoveIt 2 configuration
│   ├── config/                          # SRDF, kinematics, joint limits, controllers
│   └── launch/
│
├── my_robot_bringup/                # Full-system launch
│   ├── launch/my_robot_gz.launch.py     # entry point
│   ├── config/ros2_controllers.yaml     # controller manager: 1 kHz, controller types
│   └── scripts/                         # check_mrac_system.py, debug_system.py
│
├── my_robot_commander_py/           # Python tools
│   └── my_robot_commander_py/
│       ├── test_moveit.py               # evaluation loop (Home / Pose 1 / Pose 2)
│       ├── mrac_trainer.py              # random-pose training, then saves the weights
│       ├── mrac_monitor.py              # matplotlib plots of errors, gains, weights
│       ├── mrac_system_test.py / mrac_integration_test.py
│       └── diagnose_mrac.py
│
├── scripts/mrac_tuning_workflow.sh  # tuning notes for the first version
└── docs/assets/                     # images used in this README
```

---

## Quick start

Requirements: ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2, `ros2_control`, `ros2_controllers`, `gz_ros2_control`.

```bash
# 1. Build (from the repository root, or from a workspace whose src/ contains it)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# 2. Source the build — needed in EVERY new terminal
source install/setup.bash

# 3. Launch the full stack (Gazebo + controllers + MoveIt + RViz)
ros2 launch my_robot_bringup my_robot_gz.launch.py

# 4. In a second terminal (source install/setup.bash first) — evaluation loop
ros2 run my_robot_commander_py test_moveit

# 5. In a third terminal — live plots of error, gains and weights
ros2 run my_robot_commander_py mrac_monitor
#    'p' shows the plots, 's' saves them to /tmp/mrac_*.png
```

`Package 'my_robot_bringup' not found` means step 2 was skipped in that terminal.

### Launch options

| Argument | Default | Effect |
|---|---|---|
| `headless:=true` | `false` | run Gazebo without its GUI |
| `use_rviz:=false` | `true` | do not start RViz |
| `payload_mass:=0.5` | `0.0` | payload mass at the tool, in kg |
| `moveit_delay:=20.0` | `20.0` | seconds before `move_group` starts |

### Evaluation loop settings

`test_moveit` takes ROS parameters:

```bash
ros2 run my_robot_commander_py test_moveit --ros-args -p n_loops:=2 -p dwell_sec:=30.0 -p vel_scale:=0.1 -p acc_scale:=0.1
```

| Parameter | Default | Meaning |
|---|---|---|
| `n_loops` | 4 | Home → Pose 1 → Pose 2 → Home cycles |
| `dwell_sec` | 10.0 | hold at each pose, in seconds |
| `vel_scale`, `acc_scale` | 0.05 | MoveIt velocity / acceleration scaling |
| `adaptive_mode` | true | `false` switches the robust term off |
| `result_timeout` | 120.0 | per-move timeout, in seconds |

### Training and saved weights

```bash
# Random reachable poses at random speeds, then save K̂x, K̂r, Ŵ
ros2 run my_robot_commander_py mrac_trainer --ros-args -p n_poses:=20

# Save / load by hand
ros2 service call /mrac_arm_controller/save_weights std_srvs/srv/Trigger
ros2 service call /mrac_arm_controller/load_weights std_srvs/srv/Trigger
```

The file is `~/.ros/mrac_weights.yaml` unless `weights_file` is set. Set `load_weights_on_activate: true` in `robust_direct_mrac_params.yaml` to start every run from the saved weights.

### Tuning

All parameters are in `mrac_controller/src/robust_direct_mrac_params.yaml`, with comments. The main ones:

| Parameter | Meaning |
|---|---|
| `wn`, `zeta` | reference model natural frequency and damping |
| `gamma_Kx`, `gamma_Kr`, `gamma_W` | adaptation rates, one value per joint |
| `Kp_init`, `Kd_init`, `Kr_init` | initial gains, one value per joint |
| `Kp_max`, `Kd_max`, `W_max` | per-joint projection bounds |
| `decentralized` | per-joint gains (`true`) or full matrices (`false`) |
| `sigma`, `sigma_W` | leakage toward the initial / loaded values |
| `goal_tolerance`, `goal_time_tolerance` | when a trajectory goal succeeds or is aborted |

After editing the YAML, run `colcon build --packages-select mrac_controller` and restart the launch.

### Diagnostics

| Task | Command |
|---|---|
| Controller state | `ros2 control list_controllers` (expect `mrac_arm_controller active`) |
| Unit tests | `colcon test --packages-select mrac_controller && colcon test-result --verbose` |
| RViz only, no Gazebo | `ros2 launch my_robot_description display.launch.xml` |
| Toggle the robust term | `ros2 topic pub --once /mrac/robust_enable std_msgs/msg/Bool "{data: false}"` |
| Live topic inspection | `ros2 topic echo /mrac/error` |

### Topics

| Topic | Content |
|---|---|
| `/mrac/error` | `[e, ė]` per joint — plant minus reference model |
| `/mrac/traj_error` | `q − q_d` per joint — plant minus planned trajectory |
| `/mrac/K_hat` | `[Kp, Kd, Kr]` per joint (each joint's own adaptive gains) |
| `/mrac/W_hat` | robust weights `Ŵ`, 7 values per joint |
| `/mrac/tau`, `/mrac/u_ad` | commanded torque and its robust part `−Ŵᵀα` |
| `/mrac/ref_model` | `[q_m, q̇_m]` per joint |
| `/mrac/lyapunov_V` | tracking-error energy `eᵀPe` per joint |
| `/mrac/mode` | 0 idle, 1 executing, 2 hold |

Diagnostics are published at 100 Hz (`diag_decimation: 10`).

### Troubleshooting

- **Launch hangs on `Waiting messages on topic [robot_description]`** (and RViz reports it never received `robot_description`). The ROS nodes are not discovering each other. This happens on WSL 2 with `networkingMode=mirrored`, where the UDP discovery Fast DDS uses by default is unreliable. Make Fast DDS use shared memory only, in **every** terminal before launching or running anything:
  ```bash
  export FASTDDS_BUILTIN_TRANSPORTS=SHM
  ```
  Add that line to `~/.bashrc` to make it permanent. It only works while all nodes run on the same machine. If nodes still do not connect after a crashed run, stop all ROS processes and run `fastdds shm clean`.
- **Joints 1 or 4 buzz or hit their torque limits.** The control rate is too low for the velocity gains. Keep `update_rate: 1000` in `ros2_controllers.yaml`, or lower `Kd_max` for those joints.
- **`gripper_controller` fails to activate.** Known: `gripper_right_finger_joint` has no command interface in the `ros2_control` description. The arm is not affected.

---

## Known limitations

- The constant basis term can absorb part of the gravity load on joints 2 and 3. That part is specific to the poses visited, which limits how well saved weights transfer to new poses.
- Coriolis and configuration-dependent inertia are not in the basis. At the low speeds used here gravity and friction dominate.
- MoveIt's planning scene has no ground plane; `mrac_trainer` keeps its random targets above the ground with a simple height check.
- Tested in simulation only.

## Future work

- Regenerate the result plots and run the payload experiments with the current controller.
- Compare against a fixed-gain baseline (PD with gravity compensation, or `JointTrajectoryController`) on the same pose loop.
- Move from simulation to hardware through the same `ros2_control` interfaces.
- Pick-and-place with varying payloads.

---

## Acknowledgements

This project was developed as a Bachelor's Thesis Project (BTP) at **The LNM Institute of Information Technology (LNMIIT), Jaipur**, ECE Department.

Special thanks to **Dr. Bharat Verma** (Assistant Professor, ECE, LNMIIT) for his mentorship, guidance, and foundational research in adaptive control — particularly his work on fast-adaptation MRAC techniques — which directly inspired and informed the design of this controller.

---

## References

1. B. Verma, R. Dey, S. K. Jain, and P. K. Padhy, "Proportional learning rate model reference adaptive control for fast adaptation," *2017 IEEE International Conference on Power, Control, Signals and Instrumentation Engineering (ICPCSI)*, Chennai, India, 2017. DOI: [10.1109/ICPCSI.2017.8279170](https://ieeexplore.ieee.org/document/8279170)
2. K. J. Åström and B. Wittenmark, *Adaptive Control*, 2nd ed., Addison-Wesley, 1995.
3. M. W. Spong, S. Hutchinson, M. Vidyasagar, *Robot Modeling and Control*, Wiley, 2006.
4. J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3rd ed., Pearson, 2005.
5. H. K. Khalil, *Nonlinear Systems*, 3rd ed., Prentice Hall, 2002.
6. I. D. Landau, R. Lozano, M. M'Saad, A. Karimi, *Adaptive Control: Algorithms, Analysis and Applications*, 2nd ed., Springer, 2011.
7. P. A. Ioannou and J. Sun, *Robust Adaptive Control*, Prentice Hall, 1996.
8. B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009.
9. J.-J. E. Slotine and W. Li, *Applied Nonlinear Control*, Prentice Hall, 1991.
10. K. S. Narendra and A. M. Annaswamy, *Stable Adaptive Systems*, Prentice Hall, 1989.
11. G. Tao, *Adaptive Control Design and Analysis*, Wiley, 2003.
12. [ROS 2 Documentation](https://docs.ros.org)
13. [Gazebo Simulator Documentation](https://gazebosim.org)
14. [MoveIt Motion Planning Framework](https://moveit.ros.org)

---

## Authors

- **Mudit Todi**

## License

This project is licensed under the [MIT License](LICENSE).
