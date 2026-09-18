#!/usr/bin/env python3
"""R2 建图（slam_toolbox 版）：硬件 + 扫描滤波 + slam_toolbox online async。

和 gmapping 版的区别：
  - 用 slam_toolbox（图优化 + 显式回环检测）替代 gmapping（粒子滤波）
  - 大面积场地（已实测到 28 x 33 m）表现明显更好
  - **必须用 async_slam_toolbox_node**，参数文件要用本项目这份，
    不能拿 slam_toolbox 自带的 mapper_params_online_sync.yaml 喂给 async 节点
    （那样会静默错配，节点能起但行为不对）

用法：
    ros2 launch /home/jetson/r2_mapping/src/r2_mapping_bringup/launch/mapping_slam_toolbox.launch.py
    ... use_scan_filter:=false          # 直接用 /scan
    ... slam_params:=<其它参数文件>
    ... start_hardware:=false           # 硬件已在跑，只挂 SLAM

注意：
  - 本包没有 colcon build 过，所以用文件路径启动，
    不能写 `ros2 launch r2_mapping_bringup ...`
  - 同一时间只能有一套底盘驱动
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser("~")
R2_MAPPING = os.path.join(HOME, "r2_mapping")

DEFAULT_SLAM_PARAMS = os.path.join(
    R2_MAPPING, "src", "r2_mapping_bringup", "config", "slam_toolbox_mapping.yaml"
)
DEFAULT_SCAN_FILTER = os.path.join(
    R2_MAPPING, "src", "r2_mapping_perception",
    "r2_mapping_perception", "scan_filter_node.py"
)


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "slam_params", default_value=DEFAULT_SLAM_PARAMS,
            description="slam_toolbox 参数文件",
        ),
        DeclareLaunchArgument(
            "scan_filter_script", default_value=DEFAULT_SCAN_FILTER,
            description="scan_filter_node.py 的绝对路径",
        ),
        DeclareLaunchArgument(
            "use_scan_filter", default_value="true",
            description="是否屏蔽车体自遮挡扇区（slam 订阅 /scan_filtered）",
        ),
        DeclareLaunchArgument(
            "blind_sectors", default_value="120:150,-150:-120",
            description="要屏蔽的扇区（度）",
        ),
        DeclareLaunchArgument("start_hardware", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="/scan_filtered"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "mode", default_value="mapping",
            description="mapping 或 localization",
        ),
    ]

    # 硬件：底盘 + 雷达 + TF
    yahboom_nav_share = get_package_share_directory("yahboomcar_nav")
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboom_nav_share, "launch", "laser_bringup_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_hardware")),
    )

    # 扫描滤波
    scan_filter = ExecuteProcess(
        cmd=[
            "python3",
            LaunchConfiguration("scan_filter_script"),
            "--ros-args",
            "-p", "input_topic:=/scan",
            "-p", "output_topic:=/scan_filtered",
            "-p", "min_range:=0.0",
            "-p", "max_range:=12.0",
            "-p", ["blind_sectors:=", LaunchConfiguration("blind_sectors")],
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_scan_filter")),
    )

    # slam_toolbox：async 节点 + 我们的参数文件
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_params"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "base_frame": LaunchConfiguration("base_frame"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "map_frame": LaunchConfiguration("map_frame"),
                "mode": LaunchConfiguration("mode"),
            },
        ],
    )

    return LaunchDescription(
        args
        + [
            LogInfo(msg=[
                "[r2_mapping] slam_toolbox 建图：",
                " 参数=", LaunchConfiguration("slam_params"),
                "  扫描=", LaunchConfiguration("scan_topic"),
                "  滤波=", LaunchConfiguration("use_scan_filter"),
            ]),
            hardware,
            scan_filter,
            slam,
        ]
    )

