#!/usr/bin/env python3
"""
Diagnose MRAC→MoveIt integration issues
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from rclpy.parameter import Parameter
import time
import sys

class MRACDiagnostic(Node):
    def __init__(self):
        super().__init__("mrac_diagnostic", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])
        self.callback_group = ReentrantCallbackGroup()

        # Try to connect to MRAC action server
        self.mrac_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/mrac_arm_controller/follow_joint_trajectory",
            callback_group=self.callback_group
        )

    def diagnose(self):
        """Run diagnostics"""
        print("\n" + "="*70)
        print("MRAC → MoveIt INTEGRATION DIAGNOSTIC")
        print("="*70 + "\n")

        # 1. Check if action server exists
        print("[1] Checking for MRAC action server...")
        print("    Action: /mrac_arm_controller/follow_joint_trajectory")

        if self.mrac_client.wait_for_server(timeout_sec=3):
            print("    ✓ MRAC action server is RUNNING\n")
        else:
            print("    ✗ MRAC action server NOT FOUND or TIMEOUT\n")
            print("    DIAGNOSIS: The mrac_arm_controller controller may not be running.")
            print("    Try: ros2 node list | grep mrac")
            print("    Try: ros2 topic list | grep mrac\n")
            return False

        # 2. Check move_group
        print("[2] Checking MoveIt move_group...")
        graph = self.get_graph()
        nodes = set(graph.get_node_names())

        if "/move_group" in nodes:
            print("    ✓ /move_group is RUNNING\n")
        else:
            print("    ✗ /move_group NOT RUNNING\n")
            print("    DIAGNOSIS: MoveIt move_group is not running.")
            return False

        # 3. Check topics
        print("[3] Checking communication topics...")
        topics = [t.full_name for t in graph.get_topic_names_and_types()]

        required = {
            "/joint_states": "Joint state feedback",
            "/mrac/tau": "Torque commands",
            "/mrac/error": "MRAC diagnostics",
        }

        for topic, desc in required.items():
            if topic in topics:
                print(f"    ✓ {topic:40} ({desc})")
            else:
                print(f"    ✗ {topic:40} [MISSING]")
        print()

        return True

def main():
    rclpy.init()
    diag = MRACDiagnostic()

    try:
        success = diag.diagnose()

        if success:
            print("[4] System appears to be functioning.")
            print("\nNext steps:")
            print("  - Check MoveIt's move_group logs for execution errors")
            print("  - Check MRAC controller logs")
            print("  - Verify MoveIt controller_manager plugin config")
            print("  - Try: ros2 topic echo /mrac_arm_controller/follow_joint_trajectory/status")

        sys.exit(0 if success else 1)
    finally:
        diag.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
