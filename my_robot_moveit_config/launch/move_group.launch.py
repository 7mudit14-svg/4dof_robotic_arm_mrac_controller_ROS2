"""
move_group.launch.py  (my_robot_moveit_config/launch/)  —  FIXED

FIX: Added use_sim_time=True so MoveIt uses Gazebo /clock.
Without this, MoveIt uses wall clock while Gazebo publishes /clock,
causing: "No state update received within 100ms of system clock"
and: "latest received state has time 0.000000"
which makes MoveIt refuse to plan because it thinks joint_states are stale.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo simulation time"
    )

    moveit_config = MoveItConfigsBuilder(
        "my_robot", package_name="my_robot_moveit_config"
    ).to_moveit_configs()

    # Add controller connection timeout and execution parameters
    moveit_config.move_group_configuration.update({
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 5.0,
        "trajectory_execution.execution_duration_monitoring": False,  # key line
        "move_group/controller_connection_timeout": 30.0,  # wait up to 30s for MRAC
    })

    ld = generate_move_group_launch(moveit_config)
    ld.add_action(use_sim_time_arg)
    return ld
