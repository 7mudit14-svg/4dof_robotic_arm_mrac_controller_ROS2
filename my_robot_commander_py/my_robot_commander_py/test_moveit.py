#!/usr/bin/env python3
"""
test_moveit.py — MRAC evaluation loop  (ROS2 Jazzy)
════════════════════════════════════════════════════════════════════════
Drives the arm through  home → pose_1 → pose_2 → home  with a hold at each
pose, N times, and reports the tracking error per joint.

Every move is planned from the ACTUAL current robot state and executed
immediately (plan_only=False), so there is never a start-state mismatch.

Settings are ROS parameters (defaults in brackets):
  adaptive_mode   [true]   true = robust term Ŵᵀα on, false = K̂x/K̂r only
  n_loops         [4]      number of full cycles
  dwell_sec       [10.0]   hold at each pose [s]
  vel_scale       [0.05]   MoveIt velocity scaling
  acc_scale       [0.05]   MoveIt acceleration scaling
  result_timeout  [120.0]  per-move timeout [s] (plan + execute)

Example:
  ros2 run my_robot_commander_py test_moveit --ros-args -p n_loops:=2 -p dwell_sec:=30.0
════════════════════════════════════════════════════════════════════════
"""

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from std_msgs.msg import Bool, Float64MultiArray

POSES = {
    "home":   {"joint1":  0.0,    "joint2": 0.0,    "joint3": 0.0,    "joint4": 0.0},
    "pose_1": {"joint1": -0.5031, "joint2": 0.3591, "joint3": 1.7818, "joint4": 0.2256},
    "pose_2": {"joint1":  1.3358, "joint2": 0.7459, "joint3": 2.0133, "joint4": 1.267},
}

LOOP_SEQUENCE = ["home", "pose_1", "pose_2", "home"]
JOINT_NAMES   = ["joint1", "joint2", "joint3", "joint4"]


class ErrorStats:
    """Running max / mean of |x| per joint."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.n = 0
        self.max = [0.0] * 4
        self.sum = [0.0] * 4
        self.last = [0.0] * 4

    def add(self, values):
        self.n += 1
        for i in range(4):
            a = abs(values[i])
            self.max[i] = max(self.max[i], a)
            self.sum[i] += a
            self.last[i] = values[i]

    def mean(self):
        return [s / self.n if self.n else 0.0 for s in self.sum]


class MRACLoopTrainer(Node):
    def __init__(self, node_name="mrac_test_commander", default_dwell=10.0):
        super().__init__(
            node_name,
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.adaptive_mode  = self.declare_parameter("adaptive_mode", True).value
        self.n_loops        = self.declare_parameter("n_loops", 4).value
        self.dwell_sec      = self.declare_parameter("dwell_sec", float(default_dwell)).value
        self.vel_scale      = self.declare_parameter("vel_scale", 0.05).value
        self.acc_scale      = self.declare_parameter("acc_scale", 0.05).value
        self.result_timeout = self.declare_parameter("result_timeout", 120.0).value

        # Single action client — plan + execute
        self._move_client = ActionClient(self, MoveGroup, "/move_action")
        self._robust_pub  = self.create_publisher(Bool, "/mrac/robust_enable", 10)

        # e_model = q − q_m (reference model),  e_traj = q − q_d (planned path)
        self._lock = threading.Lock()
        self.model_stats = ErrorStats()
        self.traj_stats  = ErrorStats()
        self.create_subscription(Float64MultiArray, "/mrac/error", self._error_cb, 50)
        self.create_subscription(Float64MultiArray, "/mrac/traj_error", self._traj_error_cb, 50)

        time.sleep(0.5)
        self._set_robust_mode(self.adaptive_mode)

    # ── Error bookkeeping ────────────────────────────────────────────────
    def _error_cb(self, msg):
        if len(msg.data) >= 8:
            with self._lock:
                self.model_stats.add([msg.data[2 * i] for i in range(4)])

    def _traj_error_cb(self, msg):
        if len(msg.data) >= 4:
            with self._lock:
                self.traj_stats.add(list(msg.data[:4]))

    def _get_errors(self):
        with self._lock:
            return list(self.model_stats.last)

    def reset_stats(self):
        with self._lock:
            self.model_stats.reset()
            self.traj_stats.reset()

    def stats_snapshot(self):
        with self._lock:
            return {
                "model_max": list(self.model_stats.max),
                "model_mean": self.model_stats.mean(),
                "model_last": list(self.model_stats.last),
                "traj_max": list(self.traj_stats.max),
                "traj_mean": self.traj_stats.mean(),
                "traj_last": list(self.traj_stats.last),
            }

    def _set_robust_mode(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        for _ in range(5):
            self._robust_pub.publish(msg)
            time.sleep(0.1)
        self.get_logger().info(
            f"[trainer] Robust term: {'ON' if enabled else 'OFF (K gains only)'}"
        )

    # ── Motion ───────────────────────────────────────────────────────────
    def _build_goal(self, target, vel_scale=None, acc_scale=None):
        """MoveGroup goal for a joint target {joint_name: angle}: plan from the current state AND execute."""
        constraints = Constraints()
        for jname in JOINT_NAMES:
            jc = JointConstraint()
            jc.joint_name       = jname
            jc.position         = float(target[jname])
            jc.tolerance_above  = 0.1
            jc.tolerance_below  = 0.1
            jc.weight           = 1.0
            constraints.joint_constraints.append(jc)

        request = MotionPlanRequest()
        request.group_name                      = "arm"
        request.goal_constraints.append(constraints)
        request.num_planning_attempts           = 10
        request.allowed_planning_time           = 10.0
        request.max_velocity_scaling_factor     = float(vel_scale if vel_scale is not None else self.vel_scale)
        request.max_acceleration_scaling_factor = float(acc_scale if acc_scale is not None else self.acc_scale)
        # start_state intentionally empty — MoveIt reads actual current state

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only       = False  # plan + execute
        goal_msg.planning_options.replan          = True
        goal_msg.planning_options.replan_attempts = 3
        return goal_msg

    def move_to(self, target, label=None, vel_scale=None, acc_scale=None):
        """Move to a named pose or a joint dict. Returns True on SUCCESS."""
        if isinstance(target, str):
            label = label or target
            target = POSES[target]
        label = label or "target"

        self.get_logger().info(f"[trainer]   → {label} ...")
        self._move_client.wait_for_server()

        future = self._move_client.send_goal_async(self._build_goal(target, vel_scale, acc_scale))
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done() or not future.result().accepted:
            self.get_logger().error(f"[trainer] Goal not accepted for {label}")
            return False

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.result_timeout)

        if not result_future.done():
            self.get_logger().error(
                f"[trainer] ✗ Timed out after {self.result_timeout:.0f}s for {label}"
            )
            return False

        result = result_future.result().result
        if result.error_code.val == 1:
            e = self._get_errors()
            self.get_logger().info(
                f"[trainer] ✓ {label:8s}  "
                f"e=[{e[0]:+.3f}, {e[1]:+.3f}, {e[2]:+.3f}, {e[3]:+.3f}] rad"
            )
            return True
        self.get_logger().error(
            f"[trainer] ✗ {label} failed (MoveIt code={result.error_code.val})"
        )
        return False

    def dwell(self, seconds):
        """Spin for `seconds`. MRAC holds and adapts at the current pose."""
        if seconds <= 0.0:
            return
        self.get_logger().info(f"[trainer]   Dwelling {seconds:.1f}s ...")
        end_time = time.time() + seconds
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def print_stats(self, title):
        s = self.stats_snapshot()

        def row(values):
            return "  ".join(f"{x:+.4f}" for x in values)

        print(f"\n  {title}")
        print("                                joint1   joint2   joint3   joint4")
        print(f"    max  |q − q_m|  (model) : {row(s['model_max'])}")
        print(f"    mean |q − q_m|  (model) : {row(s['model_mean'])}")
        print(f"    max  |q − q_d|  (plan)  : {row(s['traj_max'])}")
        print(f"    mean |q − q_d|  (plan)  : {row(s['traj_mean'])}")
        print(f"    final q − q_m           : {row(s['model_last'])}")
        print(f"    final q − q_d           : {row(s['traj_last'])}")


def main(args=None):
    rclpy.init(args=args)
    trainer = MRACLoopTrainer()

    print("\n" + "=" * 65)
    print("  MRAC evaluation loop")
    print("=" * 65)
    print(f"  Robust   : {'ON' if trainer.adaptive_mode else 'OFF'}")
    print(f"  Sequence : {' → '.join(LOOP_SEQUENCE)}")
    print(f"  Loops    : {trainer.n_loops}")
    print(f"  Dwell    : {trainer.dwell_sec:.1f}s per pose")
    print(f"  Vel/Acc  : {trainer.vel_scale}/{trainer.acc_scale}")
    print(f"  Timeout  : {trainer.result_timeout:.0f}s per move")
    print("=" * 65 + "\n")

    total_moves = 0
    total_ok    = 0

    for loop_idx in range(trainer.n_loops):
        print(f"\n{'─'*55}\n  Loop {loop_idx+1} / {trainer.n_loops}\n{'─'*55}")
        loop_ok = 0
        trainer.reset_stats()

        for pose_name in LOOP_SEQUENCE:
            ok = trainer.move_to(pose_name)
            total_moves += 1
            if ok:
                total_ok += 1
                loop_ok  += 1
                trainer.dwell(trainer.dwell_sec)
            else:
                trainer.dwell(2.0)  # short recovery dwell

        trainer.print_stats(
            f"Loop {loop_idx+1}/{trainer.n_loops} — {loop_ok}/{len(LOOP_SEQUENCE)} moves OK   [rad]"
        )

    print("\n" + "=" * 65)
    print(f"  Done — {total_ok}/{total_moves} moves successful")
    print("=" * 65)
    print("\nMonitor: ros2 run my_robot_commander_py mrac_monitor")
    trainer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
