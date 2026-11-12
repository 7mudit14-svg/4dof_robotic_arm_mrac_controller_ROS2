#!/usr/bin/env python3
"""
check_mrac_system.sh equivalent - Python version
Quick diagnostic to verify MRAC setup
"""

import subprocess
import sys
import time


def run_cmd(cmd):
    """Run shell command and return output"""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=5
        )
        return result.stdout + result.stderr
    except:
        return "[Command timeout or failed]"


def main():
    print("\n" + "=" * 70)
    print("MRAC SYSTEM DIAGNOSTIC")
    print("=" * 70)

    print("\n[1] Checking ROS nodes...")
    output = run_cmd("ros2 node list")
    for line in output.split("\n"):
        if (
            "mrac" in line.lower()
            or "move_group" in line.lower()
            or "controller" in line.lower()
        ):
            print(f"  ✓ {line}")

    print("\n[2] Checking ROS topics...")
    required_topics = [
        "/joint_states",
        "/mrac/tau",
        "/mrac/K_hat",
        "/mrac/error",
        "/mrac/lyapunov_V",
    ]
    output = run_cmd("ros2 topic list")
    found_topics = output.split("\n")
    for topic in required_topics:
        if topic in found_topics:
            print(f"  ✓ {topic}")
        else:
            print(f"  ✗ {topic} [MISSING]")

    print("\n[3] Checking ROS services...")
    output = run_cmd("ros2 service list | grep -E 'controller|plan'")
    if output:
        for line in output.split("\n"):
            if line:
                print(f"  ✓ {line}")

    print("\n[4] Checking ROS actions...")
    output = run_cmd("ros2 action list | grep -E 'trajectory|mrac'")
    if output:
        for line in output.split("\n"):
            if line:
                print(f"  ✓ {line}")

    print("\n[5] Checking /joint_states content...")
    output = run_cmd("ros2 topic echo /joint_states --once")
    if "joint1" in output and "joint2" in output:
        print("  ✓ Joint states contain arm joints")
    else:
        print("  ✗ Joint states might be malformed")

    print("\n" + "=" * 70)
    print("Run 'rqt_graph' to visualize node connections")
    print(
        "Run 'ros2 launch my_robot_bringup my_robot_gz.launch.py payload_mass:=1.0' to start"
    )
    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()
