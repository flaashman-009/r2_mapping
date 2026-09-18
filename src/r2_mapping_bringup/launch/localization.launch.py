#!/usr/bin/env python3
"""R2 定位复测：地图加载 + AMCL，不含任何运动节点。

用法：
    ros2 launch r2_mapping_bringup localization.launch.py map:=/abs/path/map.yaml
    ros2 launch r2_mapping_bringup localization.launch.py \
        map:=/home/jetson/r2_mapping/maps/room_01.yaml start_vendor:=false

本 launch 自己起 map_server + amcl + lifecycle_manager，
不依赖 nav2_bringup 的整套参数文件（避免被无关的 planner/controller 参数干扰）。

⚠️ 定位与建图不能同时运行 —— map->odom 只允许一个发布者。
   启动前请确认 slam_toolbox 已退出。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _flag(context, name):
    value = LaunchConfiguration(name).perform(context)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *_args, **_kwargs):
    bringup_share = get_package_share_directory("r2_mapping_bringup")
    amcl_params = os.path.join(bringup_share, "config", "amcl.yaml")
    map_server_params = os.path.join(bringup_share, "config", "map_server.yaml")
    rviz_config = os.path.join(bringup_share, "rviz", "rviz_localization.rviz")

    map_yaml = LaunchConfiguration("map").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic").perform(context).strip() or "scan"

    actions = []

    if not os.path.isfile(map_yaml):
        actions.append(
            LogInfo(
                msg=(
                    "[r2_mapping][错误] 找不到地图文件：" + map_yaml + "\n"
                    "[r2_mapping] 用 map:=<绝对路径>/<name>.yaml 指定，"
                    '例如 map:=$HOME/r2_mapping/maps/room_01.yaml'
                )
            )
        )

    actions.append(
        LogInfo(
            msg="[r2_mapping] 定位配置： map={} scan={}".format(map_yaml, scan_topic)
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "sensors.launch.py")
            ),
            launch_arguments={
                "start_vendor": LaunchConfiguration("start_vendor"),
                "vendor_launch": LaunchConfiguration("vendor_launch"),
                "robot_type": LaunchConfiguration("robot_type"),
                "lidar_type": LaunchConfiguration("lidar_type"),
                "use_sim_time": use_sim_time,
                "publish_lidar_tf": LaunchConfiguration("publish_lidar_tf"),
                "lidar_parent_frame": LaunchConfiguration("lidar_parent_frame"),
                "lidar_frame": LaunchConfiguration("lidar_frame"),
                "lidar_x": LaunchConfiguration("lidar_x"),
                "lidar_y": LaunchConfiguration("lidar_y"),
                "lidar_z": LaunchConfiguration("lidar_z"),
                "lidar_roll": LaunchConfiguration("lidar_roll"),
                "lidar_pitch": LaunchConfiguration("lidar_pitch"),
                "lidar_yaw": LaunchConfiguration("lidar_yaw"),
            }.items(),
        )
    )

    actions.append(
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                map_server_params,
                {"yaml_filename": map_yaml, "use_sim_time": use_sim_time},
            ],
        )
    )

    actions.append(
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                amcl_params,
                {
                    "use_sim_time": use_sim_time,
                    "base_frame_id": LaunchConfiguration("base_frame"),
                    "odom_frame_id": LaunchConfiguration("odom_frame"),
                    "global_frame_id": LaunchConfiguration("map_frame"),
                },
            ],
            remappings=[("scan", scan_topic)],
        )
    )

    actions.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "node_names": ["map_server", "amcl"],
                }
            ],
        )
    )

    if _flag(context, "rviz"):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_localization",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    return actions


def generate_launch_description():
    default_map = os.path.join(
        os.path.expanduser("~"), "r2_mapping", "maps", "room_01.yaml"
    )
    args = [
        DeclareLaunchArgument(
            "map", default_value=default_map, description="地图 YAML 的绝对路径"
        ),
        DeclareLaunchArgument("start_vendor", default_value="true"),
        DeclareLaunchArgument(
            "vendor_launch",
            default_value="/home/jetson/closed_loop/laser_bringup_tg_launch.py",
        ),
        DeclareLaunchArgument("robot_type", default_value="r2"),
        DeclareLaunchArgument("lidar_type", default_value="4ROS"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="scan"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("publish_lidar_tf", default_value="false"),
        DeclareLaunchArgument("lidar_parent_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_frame", default_value="laser"),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.185"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])

