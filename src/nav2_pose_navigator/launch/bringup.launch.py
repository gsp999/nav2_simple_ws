"""Launch everything needed for GoToPose navigation.

Usage:
    ros2 launch nav2_pose_navigator bringup.launch.py team:=red
    ros2 launch nav2_pose_navigator bringup.launch.py team:=blue
    ros2 launch nav2_pose_navigator bringup.launch.py map:=/path/to/custom.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_map(context, pkg_dir):
    """Pick map YAML: custom --map path takes priority, else field_{team}.yaml."""
    map_override = LaunchConfiguration("map").perform(context)
    if map_override:
        return map_override
    team = LaunchConfiguration("team").perform(context)
    return os.path.join(pkg_dir, "maps", f"field_{team}.yaml")


def generate_launch_description():
    pkg_dir = get_package_share_directory("nav2_pose_navigator")

    team_arg = DeclareLaunchArgument(
        "team", default_value="red",
        description="Team color: red or blue",
    )
    map_arg = DeclareLaunchArgument(
        "map", default_value="",
        description="Custom map YAML path (overrides team)",
    )
    enable_rviz_viz_arg = DeclareLaunchArgument(
        "enable_rviz_viz",
        default_value="true",
        description="Start RViz Marker/Path visualization helper",
    )

    nav2_params = os.path.join(pkg_dir, "config", "nav2_params.yaml")

    # Static nodes — no conditional logic
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
        package="nav2_controller", executable="controller_server",
        name="controller_server", output="screen",
        parameters=[nav2_params],
    )

    planner_server = Node(
        package="nav2_planner", executable="planner_server",
        name="planner_server", output="screen",
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package="nav2_behaviors", executable="behavior_server",
        name="behavior_server", output="screen",
        parameters=[nav2_params],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower", executable="waypoint_follower",
        name="waypoint_follower", output="screen",
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        name="bt_navigator", output="screen",
        parameters=[nav2_params],
    )

    # global_costmap / local_costmap are NOT independent lifecycle nodes
    # in Nav2 Jazzy — they are sub-nodes created internally by
    # planner_server / controller_server and managed by them.
    # Only the parent servers need lifecycle transitions.
    _LIFECYCLE_NODES = [
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "waypoint_follower",
        "bt_navigator",
    ]

    # Custom manual bringup — replaces nav2_lifecycle_manager.
    # On ARM boards (Radxa Airbox Q900) the DDS layer can't keep up with
    # lifecycle_manager's rapid-fire configure → get_state → activate cycle
    # and times out server-side (rmw_response.cpp:153).
    # This node calls change_state directly with retries. It starts early and
    # polls for lifecycle services, so fast boots don't pay a fixed long delay.
    lifecycle_bringup = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="nav2_pose_navigator",
                executable="lifecycle_bringup",
                name="lifecycle_bringup",
                output="screen",
                parameters=[{
                    "node_names": _LIFECYCLE_NODES,
                    "sleep_configure": 0.2,
                    "sleep_activate": 0.2,
                    "service_timeout": 90.0,
                    "service_poll_period": 0.25,
                    "transition_timeout": 20.0,
                    "transition_retries": 2,
                    "retry_delay": 0.5,
                }],
            )
        ],
    )

    # goto_pose_server starts immediately — no need to wait for Nav2.
    # If a goal arrives before Nav2 is ready, it returns failure gracefully.
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
        condition=IfCondition(LaunchConfiguration("enable_rviz_viz")),
        parameters=[{
            "frame_id": "nav_map",
            "pose_topic": "/odin1/relocation",
            "raw_cmd_topic": "/cmd_vel",
            "adjusted_cmd_topic": "/cmd_vel_adjusted",
            "plan_topic": "/plan",
        }],
    )

    # map_server needs the resolved YAML path, so we build it via OpaqueFunction
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

    return LaunchDescription([
        team_arg,
        map_arg,
        enable_rviz_viz_arg,
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
        OpaqueFunction(function=_launch_map_server),
    ])
