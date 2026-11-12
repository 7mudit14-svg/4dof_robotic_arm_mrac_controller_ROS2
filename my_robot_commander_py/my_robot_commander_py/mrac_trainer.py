#!/usr/bin/env python3
"""
mrac_trainer.py — random-pose training for the MRAC weights  (ROS2 Jazzy)
════════════════════════════════════════════════════════════════════════
Sends the arm to random reachable joint targets at random speeds through
MoveIt, so the controller's weights are excited over the whole workspace
instead of along one path. When finished it asks the controller to save
its weights (K̂x, K̂r, Ŵ) to file.

It can also run the evaluation loop (Home → Pose 1 → Pose 2 → Home) at
chosen points during training, to measure how accuracy on the evaluation
path changes with the amount of training:

  ros2 run my_robot_commander_py mrac_trainer --ros-args \\
      -p n_poses:=500 -p eval_at:="[0, 50, 500]"

runs one evaluation loop before any training, one after 50 random poses
and one after 500. Record the session with mrac_record and plot it with
mrac_plot.

Parameters (defaults in brackets):
  n_poses         [20]     number of random targets
  dwell_sec       [3.0]    hold at each random target [s]
  vel_min/vel_max [0.05 / 0.3]   range of the MoveIt velocity scaling
  seed            [0]      random seed (0 = different every run)
  save_weights    [true]   call ~/save_weights at the end
  min_height      [0.20]   keep elbow and tool at least this high [m]
  eval_at         [[-1]]   training-pose counts at which to run an evaluation loop
  eval_dwell_sec  [10.0]   hold at each pose of an evaluation loop [s]
  eval_vel_scale  [0.05]   velocity/acceleration scaling of an evaluation loop

Reload the saved weights later with
  ros2 service call /mrac_arm_controller/load_weights std_srvs/srv/Trigger
or set  load_weights_on_activate: true  in robust_direct_mrac_params.yaml.
════════════════════════════════════════════════════════════════════════
"""

import math
import random

import rclpy
from std_srvs.srv import Trigger

from my_robot_commander_py.test_moveit import MRACLoopTrainer, JOINT_NAMES, LOOP_SEQUENCE

# Sampling ranges [rad], inside the URDF joint limits with a margin.
RANGES = {
    "joint1": (-2.5, 2.5),
    "joint2": (0.05, 1.5),
    "joint3": (0.05, 2.3),
    "joint4": (-2.5, 2.5),
}

# Link geometry from arm.xacro, used only to keep targets above the ground
# (MoveIt's planning scene has no ground plane).
SHOULDER_Z = 0.35     # world → joint2 axis
UPPER_ARM  = 0.60     # joint2 → joint3
FOREARM    = 0.50     # joint3 → fingertips (0.40 link + hand + gripper)


def heights(q2, q3):
    elbow = SHOULDER_Z + UPPER_ARM * math.cos(q2)
    tool  = elbow + FOREARM * math.cos(q2 + q3)
    return elbow, tool


def sample_target(rng, min_height):
    for _ in range(1000):
        t = {j: rng.uniform(*RANGES[j]) for j in JOINT_NAMES}
        elbow, tool = heights(t["joint2"], t["joint3"])
        if elbow >= min_height and tool >= min_height:
            return t
    raise RuntimeError("could not sample a target above min_height")


def main(args=None):
    rclpy.init(args=args)
    # Shorter default hold than the evaluation loop (override with -p dwell_sec:=…).
    node = MRACLoopTrainer(node_name="mrac_trainer", default_dwell=3.0)
    n_poses    = node.declare_parameter("n_poses", 20).value
    vel_min    = node.declare_parameter("vel_min", 0.05).value
    vel_max    = node.declare_parameter("vel_max", 0.3).value
    seed       = node.declare_parameter("seed", 0).value
    do_save    = node.declare_parameter("save_weights", True).value
    min_height = node.declare_parameter("min_height", 0.20).value
    eval_at    = {int(n) for n in node.declare_parameter("eval_at", [-1]).value if n >= 0}
    eval_dwell = node.declare_parameter("eval_dwell_sec", 10.0).value
    eval_scale = node.declare_parameter("eval_vel_scale", 0.05).value
    dwell = node.dwell_sec

    rng = random.Random(seed if seed else None)

    print("\n" + "=" * 65)
    print("  MRAC random-pose training")
    print("=" * 65)
    print(f"  Targets  : {n_poses}")
    print(f"  Dwell    : {dwell:.1f}s per target")
    print(f"  Speed    : velocity scaling {vel_min} – {vel_max}")
    if eval_at:
        print(f"  Eval at  : {sorted(eval_at)} training poses")
    print("=" * 65 + "\n")

    def evaluate(after):
        node.announce("eval", after)
        node.reset_stats()
        ok = node.run_eval_loop(dwell_sec=eval_dwell, vel_scale=eval_scale, acc_scale=eval_scale)
        node.print_stats(
            f"Evaluation after {after} training poses — {ok}/{len(LOOP_SEQUENCE)} moves OK   [rad]"
        )

    ok_count = 0
    for k in range(n_poses + 1):
        if k in eval_at:
            evaluate(k)
        if k == n_poses:
            break
        target = sample_target(rng, min_height)
        scale = rng.uniform(vel_min, vel_max)
        label = f"random {k+1}/{n_poses}"
        node.get_logger().info(
            "[trainer] target "
            + ", ".join(f"{j}={target[j]:+.2f}" for j in JOINT_NAMES)
            + f"  speed={scale:.2f}"
        )
        node.announce("train", k + 1)
        if node.move_to(target, label=label, vel_scale=scale, acc_scale=scale):
            ok_count += 1
            node.dwell(dwell)
        else:
            node.dwell(1.0)

    print(f"\n  Training — {ok_count}/{n_poses} random moves OK")

    if do_save:
        client = node.create_client(Trigger, "/mrac_arm_controller/save_weights")
        if client.wait_for_service(timeout_sec=5.0):
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                print(f"  {future.result().message}")
            else:
                print("  save_weights did not answer")
        else:
            print("  save_weights service not available — weights were NOT saved")

    node.announce("idle")
    node.dwell(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
