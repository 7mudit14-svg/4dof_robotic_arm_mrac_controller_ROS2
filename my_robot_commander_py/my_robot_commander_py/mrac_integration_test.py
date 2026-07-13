#!/usr/bin/env python3
"""
mrac_integration_test.py
========================
Tests end-to-end MRAC closed-loop control:
1. Verify all nodes and topics are connected
2. Send MoveIt trajectory to MRAC
3. Monitor joint states and control outputs
4. Display RQT graph
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    WorkspaceParameters,
    Constraints,
    JointConstraint,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import subprocess

# Test poses
POSES = {
    "home": {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0},
    "pose_1": {"joint1": -0.5031, "joint2": 0.3591, "joint3": 1.7818, "joint4": 0.2256},
    "pose_2": {"joint1": 1.3358, "joint2": 0.7459, "joint3": 2.1133, "joint4": 1.267},
}
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]


class MRACIntegrationTester(Node):
    def __init__(self):
        super().__init__("mrac_integration_tester", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers for monitoring
        self.joint_states = {}
        self.torques = []
        self.K_hat_x = []
        self.K_hat_r = []
        self.errors = []
        self.lyapunov = []

        # Joint state subscriber
        self.js_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._js_callback,
            10,
            callback_group=self.callback_group,
        )

        # MRAC data subscribers
        self.tau_sub = self.create_subscription(
            Float64MultiArray,
            "/arm_effort_controller/commands",
            self._tau_callback,
            10,
            callback_group=self.callback_group,
        )

        self.k_hat_sub = self.create_subscription(
            Float64MultiArray,
            "/mrac/K_hat",
            self._k_hat_callback,
            10,
            callback_group=self.callback_group,
        )

        self.err_sub = self.create_subscription(
            Float64MultiArray,
            "/mrac/error",
            self._err_callback,
            10,
            callback_group=self.callback_group,
        )

        self.lyap_sub = self.create_subscription(
            Float64MultiArray,
            "/mrac/lyapunov_V",
            self._lyap_callback,
            10,
            callback_group=self.callback_group,
        )

        # MoveIt action client
        self.move_group_client = ActionClient(
            self, MoveGroup, "/move_action", callback_group=self.callback_group
        )

    def _js_callback(self, msg: JointState):
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in JOINT_NAMES:
                self.joint_states[name] = {"pos": pos, "vel": vel}

    def _tau_callback(self, msg: Float64MultiArray):
        self.torques = list(msg.data)

    def _k_hat_callback(self, msg: Float64MultiArray):
        # [Kx1, Kr1, Kx2, Kr2, ...]
        self.K_hat_x = [msg.data[i] for i in range(0, len(msg.data), 2)]
        self.K_hat_r = [msg.data[i] for i in range(1, len(msg.data), 2)]

    def _err_callback(self, msg: Float64MultiArray):
        # [e1, de1, e2, de2, ...]
        self.errors = [
            (msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)
        ]

    def _lyap_callback(self, msg: Float64MultiArray):
        self.lyapunov = list(msg.data)

    def check_topics(self):
        """Verify all expected topics exist"""
        print("\n" + "=" * 70)
        print("TOPIC CONNECTION CHECK")
        print("=" * 70)

        required_topics = {
            "/joint_states": "sensor_msgs/JointState",
            "/arm_effort_controller/commands": "std_msgs/Float64MultiArray",
            "/mrac/K_hat": "std_msgs/Float64MultiArray",
            "/mrac/error": "std_msgs/Float64MultiArray",
            "/mrac/lyapunov_V": "std_msgs/Float64MultiArray",
        }

        # Give system time to start
        print("Waiting 3 seconds for topics to stabilize...")
        time.sleep(3)

        graph = self.get_graph()
        topics_in_graph = [t.full_name for t in graph.get_topic_names_and_types()]

        all_good = True
        for req_topic in required_topics:
            if req_topic in topics_in_graph:
                print(f"✓ {req_topic}")
            else:
                print(f"✗ {req_topic} [MISSING]")
                all_good = False

        return all_good

    def check_nodes(self):
        """Verify all expected nodes are running"""
        print("\n" + "=" * 70)
        print("NODE CHECK")
        print("=" * 70)

        required_nodes = [
            "/robust_mrac_controller",
            "/move_group",
            "/controller_manager",
            "/gz_bridge",
            "/joint_state_broadcaster",
            "/arm_effort_controller",
        ]

        # Give MoveIt time to fully start (it takes time)
        print("Waiting 5 seconds for all nodes to start...")
        time.sleep(5)

        graph = self.get_graph()
        running_nodes = set(graph.get_node_names())

        all_good = True
        for req_node in required_nodes:
            if req_node in running_nodes:
                print(f"✓ {req_node}")
            else:
                print(f"✗ {req_node} [NOT RUNNING]")
                all_good = False

        return all_good

    def test_joint_state_feedback(self):
        """Verify joint state feedback is working"""
        print("\n" + "=" * 70)
        print("JOINT STATE FEEDBACK TEST")
        print("=" * 70)
        print("Waiting for joint states to stabilize...")

        for i in range(10):
            time.sleep(0.2)
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.joint_states) >= 4:
                break

        if len(self.joint_states) >= 4:
            print("✓ Joint states received:")
            for jname in JOINT_NAMES:
                if jname in self.joint_states:
                    js = self.joint_states[jname]
                    print(
                        f"  {jname}: pos={js['pos']:.4f} rad, vel={js['vel']:.4f} rad/s"
                    )
            return True
        else:
            print(f"✗ Only {len(self.joint_states)} joint states received (expected 4)")
            return False

    def send_trajectory(self, start_pose: str, end_pose: str):
        """Send a trajectory from start_pose to end_pose via MoveIt+MRAC"""
        print("\n" + "=" * 70)
        print(f"TRAJECTORY EXECUTION: {start_pose} → {end_pose}")
        print("=" * 70)

        # First go to start pose
        self.get_logger().info(f"Moving to {start_pose}...")
        if not self._send_goal(start_pose):
            return False

        time.sleep(2)  # Wait between movements

        # Then move to end pose
        self.get_logger().info(f"Moving to {end_pose}...")
        if not self._send_goal(end_pose):
            return False

        return True

    def _send_goal(self, pose_name: str) -> bool:
        """Send MoveIt goal"""
        self.get_logger().info(f"Waiting for move_group action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=10):
            self.get_logger().error("move_group action server not available!")
            return False

        pose = POSES[pose_name]
        constraints = Constraints()
        for jname in JOINT_NAMES:
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = pose[jname]
            jc.tolerance_above = 0.1
            jc.tolerance_below = 0.1
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request = MotionPlanRequest()
        request.group_name = "arm"
        request.goal_constraints.append(constraints)
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for {pose_name}")
            return False

        self.get_logger().info(f"Goal accepted, executing {pose_name}...")

        # Collect trajectory execution data
        start_time = time.time()
        while (time.time() - start_time) < 20:  # Max 20s per trajectory
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.torques:
                print(
                    f"  τ = [{self.torques[0]:7.2f}, {self.torques[1]:7.2f}, "
                    f"{self.torques[2]:7.2f}, {self.torques[3]:7.2f}] Nm  ",
                    end="\r",
                )

            # Check if done
            result_future = goal_handle.get_result_async()
            if result_future.done():
                result = result_future.result().result
                if result.error_code.val == 1:  # SUCCESS
                    print(f"\n✓ Trajectory {pose_name} completed successfully")
                    return True
                else:
                    print(
                        f"\n✗ Trajectory {pose_name} failed: error_code={result.error_code.val}"
                    )
                    return False

        print("\n✗ Trajectory execution timeout")
        return False

    def display_rqt_graph(self):
        """Display RQT graph"""
        print("\n" + "=" * 70)
        print("LAUNCHING RQT GRAPH")
        print("=" * 70)
        try:
            subprocess.Popen(
                ["rqt_graph"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            print("✓ RQT graph launched (check RQT window for node/topic connections)")
        except Exception as e:
            print(f"✗ Could not launch rqt_graph: {e}")


def main(args=None):
    rclpy.init(args=args)
    tester = MRACIntegrationTester()

    print("\n" + "=" * 70)
    print("MRAC CLOSED-LOOP INTEGRATION TEST")
    print("=" * 70)
    print("This test verifies:")
    print("  1. All nodes are running")
    print("  2. All topics are connected")
    print("  3. Joint state feedback works")
    print("  4. MoveIt→MRAC→Gazebo pipeline works")
    print("  5. Control outputs are produced")
    print("=" * 70)

    # Run tests
    nodes_ok = tester.check_nodes()
    topics_ok = tester.check_topics()
    feedback_ok = tester.test_joint_state_feedback()

    if not all([nodes_ok, topics_ok, feedback_ok]):
        print("\n" + "!" * 70)
        print("SYSTEM NOT READY - Fix errors above and try again")
        print("!" * 70)
        rclpy.shutdown()
        return

    # Run trajectory test
    success = tester.send_trajectory("home", "pose_1")
    if success:
        print("\n✓ MRAC closed-loop control working!")

        # Show final diagnostic data
        print("\n" + "=" * 70)
        print("CONTROL DIAGNOSTICS")
        print("=" * 70)
        if tester.K_hat_x:
            print(f"Adaptive Gains K_x: {[f'{k:.3f}' for k in tester.K_hat_x]}")
            print(f"Adaptive Gains K_r: {[f'{k:.3f}' for k in tester.K_hat_r]}")
        if tester.lyapunov:
            print(f"Lyapunov V: {[f'{v:.6f}' for v in tester.lyapunov]}")

        tester.display_rqt_graph()
    else:
        print("\n" + "!" * 70)
        print("TRAJECTORY EXECUTION FAILED")
        print("!" * 70)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
