#!/usr/bin/env python3
"""
test_moveit.py — MRAC Continuous Loop Trainer  (ROS2 Jazzy)
════════════════════════════════════════════════════════════════════════
STRATEGY: plan_only=False for every move.

MoveIt plans from the ACTUAL current robot state and immediately
executes — no start-state mismatch, no cached stale trajectories.
This is the only correct approach when the robot physically moves
between planning calls.

WHY THE PREVIOUS APPROACH FAILED:
  Pre-planning with plan_only=True kept the robot at home for ALL plans.
  So every cached trajectory started from home.  When pose_2 was
  executed from pose_1, MoveIt's execution monitor detected the
  start-state mismatch → CONTROL_FAILED (code=-4) immediately.

  Additionally, traj_start_ in the C++ controller used sim-time (now())
  but trajectory time_from_start values are in wall-clock seconds.  In
  Gazebo/VirtualBox, sim runs slower than real time, so the elapsed
  t (sim) never reached t_end (wall) → succeed() was never called →
  Python timed out at RESULT_TIMEOUT.  FIXED in mrac_controller.cpp by
  switching traj_start_ / elapsed-t to std::chrono::steady_clock.

USER SETTINGS:
  ADAPTIVE_MODE   — True = §17.3 + §19.2 robust,  False = pure §17.3
  N_LOOPS         — full home→p1→p2→home cycles
  DWELL_SEC       — seconds to hold at each pose (0 = off)
  VEL_SCALE / ACC_SCALE — MoveIt scaling (0.1 – 1.0)
  RESULT_TIMEOUT  — per-move timeout [s] (plan + execute combined)
════════════════════════════════════════════════════════════════════════
"""

# ─── USER SETTINGS ────────────────────────────────────────────────────────────
ADAPTIVE_MODE  = True    # True = §17.3 + §19.2,  False = pure §17.3
N_LOOPS        = 4       # number of full cycles
DWELL_SEC      = 10.0    # hold at each pose [s] for MRAC adaptation (0 = off)
VEL_SCALE      = 0.05     # MoveIt velocity scaling
ACC_SCALE      = 0.05     # MoveIt acceleration scaling
RESULT_TIMEOUT = 60.0   # per-move timeout [s] — covers plan + execution time
# ─────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from std_msgs.msg import Bool, Float64MultiArray
import time
import threading

POSES = {
    "home":   {"joint1":  0.0,    "joint2": 0.0,    "joint3": 0.0,    "joint4": 0.0},
    "pose_1": {"joint1": -0.5031, "joint2": 0.3591, "joint3": 1.7818, "joint4": 0.2256},
    "pose_2": {"joint1":  1.3358, "joint2": 0.7459, "joint3": 2.0133, "joint4": 1.267},
}

LOOP_SEQUENCE = ["home", "pose_1", "pose_2", "home"]  # sequence of poses to cycle through
JOINT_NAMES   = ["joint1", "joint2", "joint3", "joint4"]


class MRACLoopTrainer(Node):
    def __init__(self):
        super().__init__(
            "mrac_test_commander",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        # Single action client — plan + execute
        self._move_client = ActionClient(self, MoveGroup, "/move_action")
        self._robust_pub  = self.create_publisher(Bool, "/mrac/robust_enable", 10)

        self._e_mrac = [0.0] * 4
        self._e_lock = threading.Lock()
        self.create_subscription(Float64MultiArray, "/mrac/error", self._error_cb, 10)

        time.sleep(0.5)
        self._set_robust_mode(ADAPTIVE_MODE)

    def _error_cb(self, msg):
        with self._e_lock:
            for i in range(4):
                if i * 2 < len(msg.data):
                    self._e_mrac[i] = msg.data[2 * i]

    def _get_errors(self):
        with self._e_lock:
            return list(self._e_mrac)

    def _set_robust_mode(self, enabled):
        msg = Bool()
        msg.data = enabled
        for _ in range(5):
            self._robust_pub.publish(msg)
            time.sleep(0.1)
        self.get_logger().info(
            f"[trainer] Robust mode: {'ON (§19.2 robust)' if enabled else 'OFF (pure §17.3)'}"
        )

    def _build_goal(self, pose_name):
        """Build a MoveGroup goal: plan from current state AND execute."""
        constraints = Constraints()
        for jname in JOINT_NAMES:
            jc = JointConstraint()
            jc.joint_name       = jname
            jc.position         = POSES[pose_name][jname]
            jc.tolerance_above  = 0.1
            jc.tolerance_below  = 0.1
            jc.weight           = 1.0
            constraints.joint_constraints.append(jc)

        request = MotionPlanRequest()
        request.group_name                      = "arm"
        request.goal_constraints.append(constraints)
        request.num_planning_attempts           = 10
        request.allowed_planning_time           = 10.0
        request.max_velocity_scaling_factor     = VEL_SCALE
        request.max_acceleration_scaling_factor = ACC_SCALE
        # start_state intentionally empty — MoveIt reads actual current state

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only       = False  # plan + execute
        goal_msg.planning_options.replan          = True
        goal_msg.planning_options.replan_attempts = 3
        return goal_msg

    def move_to(self, pose_name):
        """Plan from actual current state and execute. Returns True on SUCCESS."""
        self.get_logger().info(f"[trainer]   → {pose_name} ...")
        self._move_client.wait_for_server()

        future = self._move_client.send_goal_async(self._build_goal(pose_name))
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done() or not future.result().accepted:
            self.get_logger().error(f"[trainer] Goal not accepted for {pose_name}")
            return False

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=RESULT_TIMEOUT)

        if not result_future.done():
            self.get_logger().error(
                f"[trainer] ✗ Timed out after {RESULT_TIMEOUT:.0f}s for {pose_name}\n"
                "  → Check mrac_controller is calling succeed(). "
                "Rebuild after the steady_clock fix."
            )
            return False

        result = result_future.result().result
        if result.error_code.val == 1:
            e = self._get_errors()
            self.get_logger().info(
                f"[trainer] ✓ {pose_name:8s}  "
                f"e=[{e[0]:+.3f}, {e[1]:+.3f}, {e[2]:+.3f}, {e[3]:+.3f}] rad"
            )
            return True
        else:
            self.get_logger().error(
                f"[trainer] ✗ {pose_name} failed (MoveIt code={result.error_code.val})"
            )
            return False

    def dwell(self, seconds):
        """Spin for `seconds`. MRAC holds and adapts at current position."""
        if seconds <= 0.0:
            return
        self.get_logger().info(f"[trainer]   Dwelling {seconds:.1f}s ...")
        end_time = time.time() + seconds
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def print_loop_summary(self, loop_idx, successes, total):
        e = self._get_errors()
        print(
            f"\n  Loop {loop_idx+1}/{N_LOOPS} — {successes}/{total} moves OK  │  "
            f"e_mrac: j1={e[0]:+.4f}  j2={e[1]:+.4f}  j3={e[2]:+.4f}  j4={e[3]:+.4f} rad"
        )
        if   abs(e[1]) > 0.3:  print(f"  ⚠  joint2 still large ({e[1]:+.4f} rad) — keep looping")
        elif abs(e[1]) < 0.05: print(f"  ✅ joint2 converged ({e[1]:+.4f} rad)")


def main(args=None):
    rclpy.init(args=args)
    trainer = MRACLoopTrainer()

    mode_str = "ADAPTIVE (§17.3 + §19.2)" if ADAPTIVE_MODE else "DIRECT MRAC ONLY (§17.3)"
    print("\n" + "=" * 65)
    print("  MRAC Continuous Loop Trainer")
    print("=" * 65)
    print(f"  Mode     : {mode_str}")
    print(f"  Sequence : {' → '.join(LOOP_SEQUENCE)}")
    print(f"  Loops    : {N_LOOPS}")
    print(f"  Dwell    : {DWELL_SEC:.1f}s per pose")
    print(f"  Vel/Acc  : {VEL_SCALE}/{ACC_SCALE}")
    print(f"  Timeout  : {RESULT_TIMEOUT:.0f}s per move")
    print("=" * 65 + "\n")

    total_moves = 0
    total_ok    = 0

    for loop_idx in range(N_LOOPS):
        print(f"\n{'─'*55}\n  Loop {loop_idx+1} / {N_LOOPS}\n{'─'*55}")
        loop_ok = 0

        for pose_name in LOOP_SEQUENCE:
            ok = trainer.move_to(pose_name)
            total_moves += 1
            if ok:
                total_ok += 1
                loop_ok  += 1
                trainer.dwell(DWELL_SEC)
            else:
                trainer.dwell(2.0)  # short recovery dwell

        trainer.print_loop_summary(loop_idx, loop_ok, len(LOOP_SEQUENCE))

    print("\n" + "=" * 65)
    print(f"  Done — {total_ok}/{total_moves} moves successful")
    print(f"  Final e_mrac: {trainer._get_errors()}")
    print("=" * 65)
    print("\nMonitor: ros2 run my_robot_commander_py mrac_monitor")
    trainer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()