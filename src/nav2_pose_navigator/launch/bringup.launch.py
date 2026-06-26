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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    # LIFECYCLE_NODES order: costmaps before their parent servers so
    # subscriptions to /map are set up before the servers need them.
    _LIFECYCLE_NODES = [
        "map_server",
        "global_costmap",
        "planner_server",
        "local_costmap",
        "controller_server",
        "behavior_server",
        "waypoint_follower",
        "bt_navigator",
    ]

    # Custom manual bringup — replaces nav2_lifecycle_manager.
    # On ARM boards (Radxa Airbox Q900) the DDS layer can't keep up with
    # lifecycle_manager's rapid-fire configure → get_state → activate cycle
    # and times out server-side (rmw_response.cpp:153).
    # This node calls change_state directly with generous inter-step sleeps.
    # 8s delay gives costmap nodes (inside planner/controller_server) time
    # to finish constructing before we start looking for their services.
    lifecycle_bringup = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="nav2_pose_navigator",
                executable="lifecycle_bringup",
                name="lifecycle_bringup",
                output="screen",
                parameters=[{
                    "node_names": _LIFECYCLE_NODES,
                    "sleep_configure": 2.0,
                    "sleep_activate": 1.0,
                    "service_timeout": 10.0,
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
        OpaqueFunction(function=_launch_map_server),
    ])
