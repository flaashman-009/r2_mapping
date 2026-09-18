#!/usr/bin/env python3
"""R2 2D 在线建图：硬件 bringup + (可选)扫描滤波 + slam_toolbox online_async。

用法：
    ros2 launch r2_mapping_bringup mapping.launch.py
    ros2 launch r2_mapping_bringup mapping.launch.py start_vendor:=false
    ros2 launch r2_mapping_bringup mapping.launch.py use_scan_filter:=true
    ros2 launch r2_mapping_bringup mapping.launch.py map_frame:=odom

⚠️ 建图与定位（AMCL）不能同时运行 —— map->odom 只允许一个发布者。
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
    slam_params = os.path.join(
        bringup_share, "config", "slam_toolbox_mapping.yaml"
    )
    rviz_config = os.path.join(bringup_share, "rviz", "rviz_mapping.rviz")

    use_scan_filter = _flag(context, "use_scan_filter")

    # scan_topic 留空表示自动：开滤波用 /scan_filtered，否则用 /scan
    scan_topic = LaunchConfiguration("scan_topic").perform(context).strip()
    if not scan_topic:
        scan_topic = "/scan_filtered" if use_scan_filter else "/scan"

    map_frame = LaunchConfiguration("map_frame").perform(context)
    odom_frame = LaunchConfiguration("odom_frame").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")

    actions = [
        LogInfo(
            msg="[r2_mapping] 建图配置： scan={} map={} odom={} base={}".format(
                scan_topic, map_frame, odom_frame, base_frame
            )
        ),
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
        ),
    ]

    if use_scan_filter:
        actions.append(
            LogInfo(
                msg="[r2_mapping] 启用扫描滤波：/scan -> {}（低速建图建议先关掉，"
                "只有车体自遮挡严重时才开）".format(scan_topic)
            )
        )
        actions.append(
            Node(
                package="r2_mapping_perception",
                executable="scan_filter_node",
                name="scan_filter",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/scan",
                        "output_topic": scan_topic,
                        "min_range": 0.15,
                        "max_range": 12.0,
                        "use_sim_time": use_sim_time,
                    },
                    {"blind_sectors": LaunchConfiguration("blind_sectors")},
                ],
            )
        )

    # slam_toolbox：显式用 async 节点 + 本项目的参数文件。
    # 注意不要拿 mapper_params_online_sync.yaml 喂给 async 节点（见 docs/05）。
    actions.append(
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                slam_params,
                {
                    "use_sim_time": use_sim_time,
                    "scan_topic": scan_topic,
                    "map_frame": map_frame,
                    "odom_frame": odom_frame,
                    "base_frame": base_frame,
                },
            ],
        )
    )

    if _flag(context, "rviz"):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_mapping",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument("start_vendor", default_value="true"),
        DeclareLaunchArgument(
            "vendor_launch",
            default_value="/home/jetson/closed_loop/laser_bringup_tg_launch.py",
        ),
        DeclareLaunchArgument("robot_type", default_value="r2"),
        DeclareLaunchArgument("lidar_type", default_value="4ROS"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "use_scan_filter",
            default_value="false",
            description="是否启用 /scan_filtered 扫描滤波",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="",
            description="SLAM 使用的激光话题；留空则按 use_scan_filter 自动选择",
        ),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument(
            "blind_sectors",
            default_value="[]",
            description='车体自遮挡扇区，如 ["150:210"]（度）',
        ),
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

