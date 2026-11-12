#!/usr/bin/env python3
"""
Debug script to check the state of Gazebo + ros2_control + MoveIt integration
Run while the system is running
"""
import subprocess
import time
import sys

def run_cmd(cmd):
    """Run command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        return result.stdout + result.stderr
    except Exception as e:
        return f"ERROR: {e}"

def check_section(title, cmd):
    """Print a section with command output"""
    print(f"\n{'='*70}")
    print(f"  {title}")
    print(f"{'='*70}")
    output = run_cmd(cmd)
    print(output if output else "  [No output]")

print("\n" + "="*70)
print("  GAZEBO + ROS2_CONTROL + MOVEIT DEBUG REPORT")
print("="*70)

# 1. Check nodes
check_section(
    "1. RUNNING NODES",
    "ros2 node list 2>&1 | sort"
)

# 2. Check controller_manager
check_section(
    "2. CONTROLLER MANAGER STATUS",
    "ros2 service list | grep controller_manager"
)

# 3. Check available controllers
check_section(
    "3. AVAILABLE CONTROLLERS (from controller_manager)",
    "ros2 service call /controller_manager/list_controllers rcl_interfaces/srv/ListControllers '{}' 2>&1 | head -50"
)

# 4. Check action servers
check_section(
    "4. ACTION SERVERS",
    "ros2 action list | grep -E 'mrac|follow_joint|gripper'"
)

# 5. Check topics
check_section(
    "5. KEY TOPICS",
    "ros2 topic list | grep -E 'joint_states|effort|mrac|clock'"
)

# 6. Check joint_states
check_section(
    "6. JOINT STATES (sample)",
    "ros2 topic echo /joint_states --max-msgs 1 2>&1 | head -30"
)

# 7. Check move_group parameters
check_section(
    "7. MOVE_GROUP CONTROLLER CONFIG",
    "ros2 param list /move_group 2>&1 | grep -i 'controller' | head -20"
)

# 8. Check gazebo
check_section(
    "8. GAZEBO STATUS",
    "ps aux | grep -E 'gzserver|gzclient' | grep -v grep"
)

# 9. Check MRAC controller
check_section(
    "9. MRAC CONTROLLER STATUS",
    "ros2 node info /mrac_arm_controller 2>&1 | head -50"
)

# 10. Check for errors in logs
check_section(
    "10. RECENT ERRORS (last 10 lines of stderr)",
    "dmesg | tail -10"
)

print("\n" + "="*70)
print("  ANALYSIS")
print("="*70)

# Check critical components
nodes_output = run_cmd("ros2 node list 2>&1")
controllers_output = run_cmd("ros2 service call /controller_manager/list_controllers rcl_interfaces/srv/ListControllers '{}' 2>&1")
mrac_action = run_cmd("ros2 action list | grep 'mrac_arm_controller'")
joint_states = run_cmd("ros2 topic echo /joint_states --max-msgs 1 2>&1")

issues = []

if "/controller_manager" not in nodes_output:
    issues.append("❌ controller_manager NOT RUNNING - Gazebo plugin failed to start it")
else:
    print("✓ controller_manager is running")

if "mrac_arm_controller" not in controllers_output:
    issues.append("❌ mrac_arm_controller NOT LOADED - Check controller_manager config")
else:
    print("✓ mrac_arm_controller loaded")

if "joint_state_broadcaster" not in controllers_output:
    issues.append("❌ joint_state_broadcaster NOT LOADED")
else:
    print("✓ joint_state_broadcaster loaded")

if "/mrac_arm_controller" not in nodes_output:
    issues.append("⚠ MRAC controller NOT RUNNING - may not be started yet or failed")
else:
    print("✓ MRAC controller running")

if not mrac_action:
    issues.append("❌ MRAC action server NOT RESPONDING - Check MRAC node")
else:
    print("✓ MRAC action server available")

if "joint_names" not in joint_states:
    issues.append("❌ /joint_states NOT PUBLISHING - Check joint_state_broadcaster")
else:
    print("✓ /joint_states publishing")

if issues:
    print("\n🔴 CRITICAL ISSUES FOUND:")
    for issue in issues:
        print(f"   {issue}")
else:
    print("\n🟢 ALL SYSTEMS APPEAR FUNCTIONAL")

print("\n" + "="*70)
