import os
from glob import glob
from setuptools import setup

package_name = "nav2_pose_navigator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "odom_to_tf_node = nav2_pose_navigator.odom_to_tf_node:main",
            "cmd_vel_bridge = nav2_pose_navigator.cmd_vel_bridge:main",
            "goto_pose_server = nav2_pose_navigator.goto_pose_server:main",
            "ramp_zone_manager = nav2_pose_navigator.ramp_zone_manager:main",
            "map_viz_tool = nav2_pose_navigator.map_viz_tool:main",
            "nav2_goal = nav2_pose_navigator.nav2_goal:main",
            "lifecycle_bringup = nav2_pose_navigator.lifecycle_bringup:main",
        ],
    },
)
