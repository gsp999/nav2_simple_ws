"""Launch a local closed-loop Nav2 + MPPI simulation.

This uses the real Nav2 stack and a lightweight fake robot:
  Nav2 /cmd_vel -> ramp_zone_manager -> /cmd_vel_adjusted
                -> fake robot pose/odom + cmd_vel_bridge -> /t0x0111_action
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_map(context, pkg_dir):
    map_override = LaunchConfiguration("map").perform(context)
    if map_override:
        return map_override
    team = LaunchConfiguration("team").perform(context)
    return os.path.join(pkg_dir, "maps", f"field_{team}.yaml")


def generate_launch_description():
    pkg_dir = get_package_share_directory("nav2_pose_navigator")
    nav2_params = os.path.join(pkg_dir, "config", "nav2_params.yaml")
    rviz_config = os.path.join(
        pkg_dir, "rviz", "mppi_nav2_visualization.rviz")

    team_arg = DeclareLaunchArgument("team", default_value="red")
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Custom map YAML path, overrides team",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="Open RViz with the included MPPI visualization config",
    )
    initial_x_arg = DeclareLaunchArgument("initial_x", default_value="0.0")
    initial_y_arg = DeclareLaunchArgument(
        "initial_y",
        default_value="0.0",
        description="Initial Y in map/nav_map world coordinates",
    )
    initial_yaw_arg = DeclareLaunchArgument("initial_yaw", default_value="0.0")

    odom_to_tf = Node(
        package="nav2_pose_navigator",
        executable="odom_to_tf_node",
        name="odom_to_tf_node",
        output="screen",
        parameters=[{"pose_topic": "/odin1/relocation"}],
    )

    ramp_zone_manager = Node(
        package="nav2_pose_navigator",
        executable="ramp_zone_manager",
        name="ramp_zone_manager",
        output="screen",
        parameters=[{"team": LaunchConfiguration("team")}],
    )

    cmd_vel_bridge = Node(
        package="nav2_pose_navigator",
        executable="cmd_vel_bridge",
        name="cmd_vel_bridge",
        output="screen",
        remappings=[("/cmd_vel", "/cmd_vel_adjusted")],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params],
    )

    lifecycle_bringup = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="nav2_pose_navigator",
                executable="lifecycle_bringup",
                name="lifecycle_bringup",
                output="screen",
                parameters=[{
                    "node_names": [
                        "map_server",
                        "planner_server",
                        "controller_server",
                        "behavior_server",
                        "waypoint_follower",
                        "bt_navigator",
                    ],
                    "sleep_configure": 0.2,
                    "sleep_activate": 0.2,
                    "service_timeout": 30.0,
                    "service_poll_period": 0.25,
                    "transition_timeout": 12.0,
                    "transition_retries": 2,
                    "retry_delay": 0.5,
                }],
            )
        ],
    )

    goto_pose_server = Node(
        package="nav2_pose_navigator",
        executable="goto_pose_server",
        name="goto_pose_server",
        output="screen",
        parameters=[{"team": LaunchConfiguration("team")}],
    )

    mppi_rviz_visualizer = Node(
        package="nav2_pose_navigator",
        executable="mppi_rviz_visualizer",
        name="mppi_rviz_visualizer",
        output="screen",
        parameters=[{
            "frame_id": "nav_map",
            "pose_topic": "/odin1/relocation",
            "raw_cmd_topic": "/cmd_vel",
            "adjusted_cmd_topic": "/cmd_vel_adjusted",
            "plan_topic": "/plan",
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        arguments=["-d", rviz_config],
    )

    def _launch_map_server(context):
        yaml_path = _resolve_map(context, pkg_dir)
        return [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[nav2_params, {"yaml_filename": yaml_path}],
            )
        ]

    def _launch_sim_robot(context):
        return [
            Node(
                package="nav2_pose_navigator",
                executable="nav2_sim_robot",
                name="nav2_sim_robot",
                output="screen",
                parameters=[{
                    "initial_x": LaunchConfiguration("initial_x"),
                    "initial_y": LaunchConfiguration("initial_y"),
                    "initial_yaw": LaunchConfiguration("initial_yaw"),
                }],
            )
        ]

    return LaunchDescription([
        team_arg,
        map_arg,
        start_rviz_arg,
        initial_x_arg,
        initial_y_arg,
        initial_yaw_arg,
        OpaqueFunction(function=_launch_sim_robot),
        odom_to_tf,
        ramp_zone_manager,
        cmd_vel_bridge,
        controller_server,
        planner_server,
        behavior_server,
        waypoint_follower,
        bt_navigator,
        lifecycle_bringup,
        goto_pose_server,
        mppi_rviz_visualizer,
        rviz,
        OpaqueFunction(function=_launch_map_server),
    ])
