# Changes and fixes

This file lists what changed between the first version of the project (commit `3a76eae` on `main`) and the `mrac-ros2-control-plugin` branch. The first version ran the controller as a standalone node; the current version runs it as a `ros2_control` controller plugin with a corrected control law.

For how to build and run the project, see [README.md](README.md).

## Summary

| | First version | Current version |
|---|---|---|
| Controller | standalone node, wall timer | `ros2_control` plugin `mrac_arm_controller`, 1 kHz |
| Control law | `τ = K̂x·q + K̂r·r` (position only, scalar gains) | `τ = K̂x·[q; q̇] + K̂r·r − Ŵᵀα` (matrix gains) |
| Basis | 22 ad-hoc terms | 7 physics terms |
| Final error on the pose loop (joint 2) | 0.034 rad | 0.0003 rad |
| Weights after restart | lost | can be saved and loaded |
| Unit tests | none | 12 |

---

## 1. Control law

All of this is in `mrac_controller/include/mrac_controller/mrac_core.hpp`.

| # | Problem in the first version | Change |
|---|---|---|
| 1.1 | Feedback used position only. A torque-driven joint has two states (`q`, `q̇`), so the reference model could never be matched and the only damping came from the URDF. | `K̂x` is now a 4×8 matrix acting on `[q; q̇]`, giving each joint a position gain `Kp` and a velocity gain `Kd`. `K̂r` is a 4×4 matrix. |
| 1.2 | `K̂x` and `K̂r` were adapted with the position error `e` alone, which is the first-order (scalar) law. The Lyapunov proof for a two-state plant does not cover it. | All three adaptation laws now use `s = sign(b)·BᵀPe = p12·e + p22·ė`. |
| 1.3 | The matrices were not adjustable in structure. | Parameter `decentralized`: `true` (default) adapts each joint's own `Kp`, `Kd`, `Kr`; `false` adapts the full matrices including cross-joint terms. |
| 1.4 | Leakage (σ-modification) pulled gains toward zero, which left a permanent steady-state error. | Leakage pulls toward the initial or loaded values. It is off by default (`sigma: 0.0`, `sigma_W: 0.0`). |
| 1.5 | The reference model was integrated with forward Euler at a fixed 1 ms step. | RK4 integration using the measured control period. |

## 2. Basis functions

| # | Problem in the first version | Change |
|---|---|---|
| 2.1 | The 22-term basis could not represent this arm's gravity: joints 2 and 3 have parallel axes, so gravity contains `sin(q2+q3)`, which was missing. | Basis is now `[1, sin(q2), sin(q2+q3), tanh(q̇1), tanh(q̇2), tanh(q̇3), tanh(q̇4)]`. |
| 2.2 | Every joint used every term, including terms with no physical effect on it (`q_i·q_j` products, gravity terms on the waist and wrist). | A per-joint mask: joint 1 and 4 use the constant and their own friction term; joint 2 adds both gravity terms; joint 3 adds `sin(q2+q3)`. |
| 2.3 | `tanh(q̇)` existed only for joints 1 and 2. | One Coulomb-friction term per joint. |
| 2.4 | Linear `q` and `q̇` terms duplicated what the `K̂x` gains already do. | Removed. Viscous friction is handled by `Kd`. |

## 3. Reference and trajectory handling

| # | Problem in the first version | Change |
|---|---|---|
| 3.1 | The planned position was fed straight into the reference model, which lagged the plan by about 0.5 s. The sampled velocity was computed and discarded. | The sampler returns `q_d`, `q̇_d`, `q̈_d`. The reference is pre-compensated, `r = q_d + (2ζ/ωn)·q̇_d + (1/ωn²)·q̈_d`, so the model follows the plan. |
| 3.2 | On the last tick of every trajectory the sampler returned zeros, giving one tick with `r = 0` and a torque spike. | The sampler returns the final point for any time at or after the end. |
| 3.3 | Trajectory time used the wall clock, so the reference ran ahead when Gazebo ran slower than real time. | Trajectory time advances by the controller's own period (simulation time). |

Files: `mrac_controller/include/mrac_controller/trajectory_sampler.hpp`, `mrac_core.hpp`.

## 4. Execution: port to a ros2_control plugin

| # | Problem in the first version | Change |
|---|---|---|
| 4.1 | The controller was a separate node publishing torques to a `ForwardCommandController`. The node ran at 1 kHz but the controller manager ran at 100 Hz, so it integrated ten times on each measurement. | The controller is the plugin `mrac_controller/MracController`. It reads position/velocity state interfaces and writes effort command interfaces directly, once per control cycle. |
| 4.2 | Controller manager rate was 100 Hz. | 1000 Hz (`my_robot_bringup/config/ros2_controllers.yaml`). |
| 4.3 | The controller node was started by a 35 s timer; the arm had no torque until then. | Spawned right after the joint state broadcaster. The arm is held within seconds of startup. |
| 4.4 | `arm_effort_controller` (forwarding controller) was needed. | Removed. |

New files: `mrac_controller/src/mrac_controller_plugin.cpp`, `include/mrac_controller/mrac_controller_plugin.hpp`, `mrac_plugin.xml`.
The first implementation, `mrac_controller/src/mrac_controller.cpp`, is kept and still builds, but is not launched.

## 5. Bounds and saturation

| # | Problem in the first version | Change |
|---|---|---|
| 5.1 | The torque clamp (250/250/250/80 Nm) was above the URDF limits (150/150/100/50 Nm), so Gazebo clipped the torque without the controller knowing. | Parameter `tau_limit`, set to the URDF values. |
| 5.2 | Adaptation kept integrating while the torque was saturated. | Adaptation of a joint pauses while that joint is saturated. |
| 5.3 | One global bound for all gains. | Per-joint bounds `Kp_max`, `Kd_max`, `W_max`, sized so the light joints (waist, wrist) stay stable at 1 kHz. |

## 6. Trajectory action server

| # | Problem in the first version | Change |
|---|---|---|
| 6.1 | Every goal reported success 0.5 s after the planned end, whatever the error. | Success only when every joint is within `goal_tolerance` (0.05 rad); abort with `GOAL_TOLERANCE_VIOLATED` after `goal_time_tolerance` (5 s). |
| 6.2 | Cancel never finished the goal. | Cancel stops the motion, holds the current position and reports `CANCELED`. |
| 6.3 | A new goal overwrote the old one without ending it. | The old goal is aborted, then the new one starts. |
| 6.4 | No goal validation; a malformed goal could crash the node. | Goals are rejected if a joint is missing, a point has the wrong size, a value is not finite, or times decrease. |
| 6.5 | Parameter arrays were indexed without checking their length. | Every per-joint parameter must have exactly 4 values, or the controller refuses to configure. |

## 7. New features

| # | Feature | Where |
|---|---|---|
| 7.1 | Save and load the adaptive weights: services `~/save_weights`, `~/load_weights`; parameters `weights_file`, `load_weights_on_activate`. Default file `~/.ros/mrac_weights.yaml`. | `weights_io.hpp`, plugin |
| 7.2 | Random-pose training node: random reachable joint targets at random speeds through MoveIt, then saves the weights. | `my_robot_commander_py/.../mrac_trainer.py` |
| 7.3 | Unit tests of the maths: Lyapunov equation, reference model, sampler, basis and masks, saturation, a closed-loop arm model, save/load. | `mrac_controller/test/test_mrac_core.cpp` |
| 7.4 | Launch arguments `headless:=true` (Gazebo without GUI) and `use_rviz:=false`. | `my_robot_gz.launch.py` |
| 7.5 | New topic `/mrac/tau` (commanded torque). `/mrac/traj_error` now reports `q − q_d`. | plugin |
| 7.6 | Evaluation checkpoints during training: `mrac_trainer -p eval_at:="[0, 50, 200]"` runs the evaluation loop before training and after 50 and 200 random poses. | `mrac_trainer.py` |
| 7.7 | Session recorder: records the `/mrac/*` topics, splits the session into segments (training pose n, evaluation n) and keeps the exact mean and max error of each segment. | `my_robot_commander_py/.../mrac_record.py` |
| 7.8 | Plot script: tracking error, gains and weights over the session; mean and max error per training pose; evaluation error before/after training; summary tables. | `my_robot_commander_py/.../mrac_plot.py` |
| 7.9 | Topic `/mrac/segment` (`train:17`, `eval:50`, `eval_loop:2`, `idle:0`), published by `test_moveit` and `mrac_trainer` so a recording can be split. | `test_moveit.py` |
| 7.10 | Launch argument `record_camera:=true`: world with a fixed camera whose images are bridged to `/rec_cam`; the recorder saves the frames and the plot script can turn them into GIFs. | `my_robot_bringup/worlds/recording.sdf`, launch file |

## 8. Scripts and configuration

| # | File | Change |
|---|---|---|
| 8.1 | `test_moveit.py` | Settings are ROS parameters (`n_loops`, `dwell_sec`, `vel_scale`, `acc_scale`, `adaptive_mode`, `result_timeout`) instead of constants in the file. Prints max, mean and final error per joint for each loop. Defaults are unchanged (4 loops, 10 s hold, scaling 0.05). |
| 8.1b | `test_moveit.py` | New parameter `pose_tolerance` (0.01 rad; the goal tolerance was a fixed 0.1 rad), so every loop goes to the same poses. The tolerance window is clipped at the physical joint limits (see 9.4). |
| 8.2 | `mrac_monitor.py` | Updated for the 7-term basis and the new `/mrac/K_hat` layout `[Kp, Kd, Kr]` per joint; plots `Kd` alongside `Kp`. |
| 8.3 | `robust_direct_mrac_params.yaml` | Rewritten for the plugin (top-level key `mrac_arm_controller`), with new parameters and corrected comments. |
| 8.4 | `my_robot_gz.launch.py` | Spawns the plugin with its parameter file; removed the 35 s timer and the unused `gamma`, `kp`, `kd` arguments. |
| 8.5 | `check_mrac_system.py`, `debug_system.py`, `diagnose_mrac.py`, `mrac_integration_test.py`, `mrac_system_test.py` | Old node and topic names replaced with the new ones. Not otherwise changed and not re-run. |
| 8.6 | `mrac_controller/CMakeLists.txt`, `package.xml` | Plugin library, pluginlib export, new dependencies, unit test target. |
| 8.7 | `my_robot_commander_py/package.xml`, `setup.py` | Removed unused dependencies (`moveit_py`, `tf_transformations`), added the ones actually used; added the `mrac_trainer` entry point. |
| 8.8 | `/mrac/lyapunov_V` | Now publishes `eᵀPe` only. The earlier formula added `‖Ŵ‖²/Γ`, which is not part of a Lyapunov function. |
| 8.9 | `README.md` | Rewritten for the current controller. |

## 9. Problems found and fixed during the rework

These appeared while testing the new controller in Gazebo.

| # | Symptom | Cause | Fix |
|---|---|---|---|
| 9.1 | Waist and wrist torque flipped between ± limit every cycle; MoveIt rejected trajectories with "start point deviates from current robot state". | At 100 Hz, velocity feedback is unstable on joints with very small inertia. | Controller rate raised to 1 kHz; per-joint `Kd_max` (5.2, 5.3). |
| 9.2 | The wrist went unstable after some minutes of adaptation. | The viscous basis term `q̇` and the velocity gain `Kd` both adapted on the same signal, and their sum grew past the stable limit. | Removed the `q̇` terms from the basis (2.4); per-joint `W_max`. |
| 9.3 | Joints 1 and 4 held a small constant offset at rest. | With no gravity terms and zero velocity, nothing in their adaptation acted on a steady error. | Added the constant term `1` to the basis. |

| 9.4 | A 0.15 rad error spike on joint 3 when leaving home, appearing in some loops only. | Home is exactly on the lower stop of joints 2 and 3. With a ±0.1 rad goal tolerance and MoveIt's limits widened to −0.05, MoveIt sometimes chose a home goal of about −0.04 rad, beyond the stop. The controller pushed against the stop for the whole hold, the offset weight wound up, and the arm overshot when it left. | `test_moveit` clips its goal tolerance at the physical joint limits and uses a tight tolerance (8.1b). MoveIt's widened limit is kept, because a start state resting a hair below the stop must not be rejected; setting it to exactly 0 made MoveIt refuse most plans (`START_STATE_INVALID`). |

## 10. Environment fix (WSL 2)

| Symptom | Cause | Fix |
|---|---|---|
| Launch hangs on `Waiting messages on topic [robot_description]`; RViz never receives the robot model. | WSL 2 with `networkingMode=mirrored`: the UDP discovery that Fast DDS uses by default is unreliable there, so nodes do not find each other. | `export FASTDDS_BUILTIN_TRANSPORTS=SHM` in every terminal (added to `~/.bashrc`). Works while all nodes run on the same machine. |
| `Package 'my_robot_bringup' not found` | Workspace not sourced in that terminal. | `source install/setup.bash` in every new terminal. |

## 11. Measured results

One recorded session, starting from zero weights and the initial gains, no payload, Gazebo headless: evaluation loop → 50 random training poses → evaluation loop → 150 more training poses → evaluation loop. The evaluation loop is Home → Pose 1 → Pose 2 → Home with 10 s holds and scaling 0.05. 199 of 200 training moves and 12 of 12 evaluation moves succeeded. Errors in rad. Graphs are in the README and in `docs/assets/`; the tables are also in `docs/assets/results_summary.md`.

| Mean \|e\| on the evaluation loop | Joint 1 | Joint 2 | Joint 3 | Joint 4 |
|---|---|---|---|---|
| First version (earlier README) | 0.0075 | 0.0342 | 0.0158 | 0.0049 |
| Current, before training | 0.0018 | 0.0032 | 0.0020 | 0.0009 |
| Current, after 50 training poses | 0.0023 | 0.0015 | 0.0020 | 0.0005 |
| Current, after 200 training poses | 0.0017 | 0.0014 | 0.0015 | 0.0012 |

| Max \|e\| on the evaluation loop | Joint 1 | Joint 2 | Joint 3 | Joint 4 |
|---|---|---|---|---|
| First version (earlier README) | 0.0322 | 0.1994 | 0.1544 | 0.0245 |
| Current, before training | 0.0091 | 0.0178 | 0.0094 | 0.0028 |
| Current, after 50 training poses | 0.0079 | 0.0055 | 0.0089 | 0.0059 |
| Current, after 200 training poses | 0.0094 | 0.0063 | 0.0057 | 0.0043 |

What the session shows:

- Training helps the gravity-loaded joints on the evaluation loop: joint 2's mean error halves and its peak drops to about a third; joint 3's peak drops from 0.0094 to 0.0057.
- Joint 1 does not change. Joint 4 is slightly worse after training (around 0.001 rad).
- On the random training poses themselves the error stays roughly level over the 200 poses; only joint 2 shows a clear drop towards the end.
- The gains do not settle to fixed values within 200 poses. The wrist's velocity gain sits at its bound for most of the session.

Also checked:

- Unit tests: 12 of 12 pass.
- Action server: normal goal, cancel mid-motion, preemption, and three kinds of malformed goal all behave as described in section 6.
- An earlier 500-pose session (before fix 9.4) completed 500 of 500 training moves. Its last evaluation was spoiled by the hard-stop problem, and joint 1's error rose over its last 150 poses with that joint's velocity gain at its bound, so longer training is not simply better.

## 12. Known open issues

| Issue | Status |
|---|---|
| `gripper_controller` fails to activate: `gripper_right_finger_joint` has no command interface. | Not fixed. The arm is unaffected. |
| The demo GIFs in `docs/assets/` are from the first version. | A recording of the current controller is to be added. The plots were regenerated. |
| Joint 1 does not benefit from training, and in a 500-pose session its gains drifted to their bounds. | Open. Leakage (`sigma`) toward the initial gains is the next thing to try; it is off by default. |
| In one 90-minute session the ROS `/clock` topic stopped being delivered after 25 minutes (WSL 2). The run itself continued normally. | `mrac_plot` rebuilds the time axis from the sample count when this happens. Cause not investigated. |
| No run with a payload has been done on the current controller. | Open. |
| The constant basis term can absorb part of the gravity load on joints 2 and 3. This was clear after training on only 8 poses; after 200 varied poses the constant weights ended near zero and the load sat on the gravity terms. | Open; removing the constant from joints 2 and 3 is still worth testing. |
| `MRAC_TUNING_GUIDE.md` and `scripts/mrac_tuning_workflow.sh` describe the first version. | Not updated. |
| `pose_2` joint 3 is 2.0133 in `test_moveit.py` and 2.1133 in the SRDF. | Not changed. |
| `my_robot_moveit_config/launch/move_group.launch.py` uses an attribute that does not exist on Jazzy. It is not used by the main launch. | Not fixed. |
| Only tested headless; the Gazebo GUI, RViz and the monitor's plots were not checked on screen. | Open. |

## 13. File list

New:

```
CHANGES.md
mrac_controller/include/mrac_controller/mrac_core.hpp
mrac_controller/include/mrac_controller/trajectory_sampler.hpp
mrac_controller/include/mrac_controller/weights_io.hpp
mrac_controller/include/mrac_controller/mrac_controller_plugin.hpp
mrac_controller/src/mrac_controller_plugin.cpp
mrac_controller/mrac_plugin.xml
mrac_controller/test/test_mrac_core.cpp
my_robot_commander_py/my_robot_commander_py/mrac_trainer.py
my_robot_commander_py/my_robot_commander_py/mrac_record.py
my_robot_commander_py/my_robot_commander_py/mrac_plot.py
my_robot_bringup/worlds/recording.sdf
docs/assets/  (new plots: eval_*.png, train_*.png, mrac_W_gravity.png, mrac_tau.png, results_summary.md;
               regenerated: mrac_e_mrac.png, mrac_Kx.png, mrac_Kr.png, mrac_W_hat.png, mrac_lyapunov.png)
```

Modified:

```
README.md
mrac_controller/CMakeLists.txt
mrac_controller/package.xml
mrac_controller/src/robust_direct_mrac_params.yaml
my_robot_bringup/CMakeLists.txt
my_robot_bringup/config/ros2_controllers.yaml
my_robot_bringup/launch/my_robot_gz.launch.py
my_robot_bringup/scripts/check_mrac_system.py
my_robot_bringup/scripts/debug_system.py
my_robot_commander_py/package.xml
my_robot_commander_py/setup.py
my_robot_commander_py/my_robot_commander_py/test_moveit.py
my_robot_commander_py/my_robot_commander_py/mrac_monitor.py
my_robot_commander_py/my_robot_commander_py/diagnose_mrac.py
my_robot_commander_py/my_robot_commander_py/mrac_integration_test.py
my_robot_commander_py/my_robot_commander_py/mrac_system_test.py
```

Unchanged apart from one comment in `my_robot_moveit_config/config/joint_limits.yaml`: the URDF/Xacro, the SRDF and MoveIt configuration, and `mrac_controller/src/mrac_controller.cpp`.
