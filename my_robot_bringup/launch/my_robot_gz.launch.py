"""
my_robot_gz.launch.py  —  COMPLETE (Jazzy + Gazebo Harmonic + MRAC)

Key features:
  1. Gazebo plugin loads ros2_controllers.yaml via <parameters> tag
  2. Controller spawners chained with RegisterEventHandler for reliability
  3. All nodes get use_sim_time=True
  4. MRAC action server: /mrac_arm_controller/follow_joint_trajectory
  5. MoveIt sends trajectories to MRAC node via MoveItSimpleControllerManager
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ── Package share directories ──────────────────────────────────────────
    pkg_desc = get_package_share_directory("my_robot_description")
    pkg_bringup = get_package_share_directory("my_robot_bringup")
    pkg_moveit = get_package_share_directory("my_robot_moveit_config")
    pkg_mrac = get_package_share_directory("mrac_controller")

    urdf_file = os.path.join(pkg_desc, "urdf", "my_robot.urdf.xacro")
    initial_pos_file = os.path.join(pkg_desc, "urdf", "initial_positions.yaml")
    rviz_config = os.path.join(pkg_bringup, "config", "my_robot_moveit.rviz")
    mrac_params_file = os.path.join(pkg_mrac, "robust_direct_mrac_params.yaml")

    # ── Launch arguments ───────────────────────────────────────────────────
    payload_mass_arg = DeclareLaunchArgument(
        "payload_mass",
        default_value="0.0",
        description="Payload mass kg: 0.0 / 0.5 / 1.0 / 2.0",
    )
    gamma_arg = DeclareLaunchArgument(
        "gamma", default_value="1.0", description="MRAC adaptation gain"
    )
    kp_arg = DeclareLaunchArgument("kp", default_value="20.0")
    kd_arg = DeclareLaunchArgument("kd", default_value="10.0")

    moveit_delay_arg = DeclareLaunchArgument(
        "moveit_delay",
        default_value="20.0",
        description="Seconds to wait before starting move_group",
    )
    mrac_delay_arg = DeclareLaunchArgument(
        "mrac_delay",
        default_value="35.0",
        description="Seconds to wait before starting MRAC controller",
    )
    rviz_delay_arg = DeclareLaunchArgument(
        "rviz_delay",
        default_value="22.0",
        description="Seconds to wait before starting RViz",
    )

    # ── Robot description ──────────────────────────────────────────────────
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            urdf_file,
            " payload_mass:=",
            LaunchConfiguration("payload_mass"),
            " initial_positions_file:=",
            initial_pos_file,
            " use_sim:=true",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            value=robot_description_content, value_type=str
        )
    }

    # ── MoveIt config ──────────────────────────────────────────────────────
    moveit_config = MoveItConfigsBuilder(
        "my_robot", package_name="my_robot_moveit_config"
    ).to_moveit_configs()

    # ── 1. Robot State Publisher ───────────────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # ── 2. Gazebo Harmonic ─────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── 3. Clock bridge gz -> ROS ──────────────────────────────────────────
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ── 4. Spawn robot into Gazebo ─────────────────────────────────────────
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_my_robot",
        arguments=["-name", "my_robot", "-topic", "robot_description", "-z", "0.05"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ── 5. Controller spawners ─────────────────────────────────────────────
    # Chained with RegisterEventHandler: each starts only AFTER previous exits
    # This is Automatic Addison's proven pattern — no guess timers
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    load_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_effort_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    load_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Chain: spawn_robot exits → jsb loads → arm loads → gripper loads
    after_spawn_load_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_jsb],
        )
    )

    after_jsb_load_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_jsb,
            on_exit=[load_arm],
        )
    )

    after_arm_load_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm,
            on_exit=[load_gripper],
        )
    )

    # ── 6. MoveIt move_group ───────────────────────────────────────────────
    move_group_node = TimerAction(
        period=LaunchConfiguration("moveit_delay"),
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {"use_sim_time": True},
                ],
            )
        ],
    )

    # ── 7. MRAC controller node ────────────────────────────────────────────
    mrac_node = TimerAction(
        period=LaunchConfiguration("mrac_delay"),
        actions=[
            Node(
                package="mrac_controller",
                executable="mrac_controller",
                name="robust_mrac_controller",
                output="screen",
                parameters=[
                    mrac_params_file,
                    {"use_sim_time": True},
                ],
            )
        ],
    )

    # ── 8. RViz ────────────────────────────────────────────────────────────
    rviz_node = TimerAction(
        period=LaunchConfiguration("rviz_delay"),
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            payload_mass_arg,
            gamma_arg,
            kp_arg,
            kd_arg,
            moveit_delay_arg,
            mrac_delay_arg,
            rviz_delay_arg,
            rsp_node,
            gz_sim,
            gz_bridge,
            spawn_robot,
            after_spawn_load_jsb,
            after_jsb_load_arm,
            after_arm_load_gripper,
            move_group_node,
            mrac_node,
            rviz_node,
        ]
    )
