#!/usr/bin/env python3
"""R2 建图（gmapping 版）：硬件 + 扫描滤波 + 调过参的 gmapping。

和原厂 map_gmapping_4ros_launch.py 的区别：
  1. 用我们调过的 gmapping 参数（config/slam_gmapping_r2.yaml），
     重点修"地图飘"：关键帧加密、量程放开、降低对里程计旋转的信任；
  2. 中间加一层 scan_filter_node，**屏蔽车体自遮挡扇区**
     （实测车尾左右角在 0.15-0.35 m 处有固定回波，占有效点 9.3%），
     gmapping 改为订阅 /scan_filtered。

用法：
    ros2 launch r2_mapping_bringup mapping_gmapping.launch.py
    ros2 launch r2_mapping_bringup mapping_gmapping.launch.py use_scan_filter:=false
    ros2 launch r2_mapping_bringup mapping_gmapping.launch.py \
        gmapping_params:=$HOME/r2_mapping/config/slam_gmapping_r2.yaml

⚠️ 前提：
  - 环境里要能 import rclpy / sensor_msgs（source 过 ROS）
  - scan_filter_node 直接用源码路径跑，**不需要 colcon build**
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

DEFAULT_GMPPING_PARAMS = os.path.join(
    HOME, "r2_mapping", "config", "slam_gmapping_r2.yaml"
)
DEFAULT_SCAN_FILTER = os.path.join(
    HOME, "r2_mapping", "src", "r2_mapping_perception",
    "r2_mapping_perception", "scan_filter_node.py"
)


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "gmapping_params", default_value=DEFAULT_GMPPING_PARAMS,
            description="gmapping 参数文件（我们调过的那份）",
        ),
        DeclareLaunchArgument(
            "use_scan_filter", default_value="true",
            description="是否屏蔽车体自遮挡扇区",
        ),
        DeclareLaunchArgument(
            "blind_sectors", default_value="120:150,-150:-120",
            description="要屏蔽的扇区（度），实测车尾左右角的回波就在这两个区间",
        ),
        DeclareLaunchArgument(
            "scan_filter_script", default_value=DEFAULT_SCAN_FILTER,
            description="scan_filter_node.py 的绝对路径",
        ),
        DeclareLaunchArgument("start_hardware", default_value="true"),
    ]

    # 硬件：底盘 + 雷达 + TF（复用原厂 bringup）
    yahboom_nav_share = get_package_share_directory("yahboomcar_nav")
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboom_nav_share, "launch", "laser_bringup_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_hardware")),
    )

    # 扫描滤波：屏蔽车体自遮挡
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

    # gmapping：订阅 /scan_filtered，用我们的参数
    gmapping = Node(
        package="slam_gmapping",
        executable="slam_gmapping",
        name="slam_gmapping",
        output="screen",
        parameters=[LaunchConfiguration("gmapping_params")],
        remappings=[("scan", "/scan_filtered")],
    )

    return LaunchDescription(
        args
        + [
            LogInfo(msg=[
                "[r2_mapping] gmapping 建图：",
                " 参数=", LaunchConfiguration("gmapping_params"),
                "  滤波=", LaunchConfiguration("use_scan_filter"),
                "  屏蔽扇区=", LaunchConfiguration("blind_sectors"),
            ]),
            hardware,
            scan_filter,
            gmapping,
        ]
    )
