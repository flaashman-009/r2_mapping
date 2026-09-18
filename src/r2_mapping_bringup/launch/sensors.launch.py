#!/usr/bin/env python3
"""R2 硬件 bringup 包装层：底盘 + LiDAR + base_node + TF。

本文件**不重新实现**底盘驱动，而是把已有的原厂 bringup 包一层，
这样将来原厂 launch 换位置/换名字时只需改一个参数。

用法一：由本 launch 拉起原厂 bringup（默认）
    ros2 launch r2_mapping_bringup sensors.launch.py

用法二：原厂 stack 已经在跑，只挂接检查（避免串口二次占用）
    ros2 launch r2_mapping_bringup sensors.launch.py start_vendor:=false

用法三：指定其它 bringup 文件
    ros2 launch r2_mapping_bringup sensors.launch.py \
        vendor_launch:=/path/to/other_bringup.launch.py

⚠️ /dev/myserial 同一时间只能被一个进程占用。start_vendor:=true 之前
   请先执行 scripts/serial_guard.sh 确认没有残留驱动进程。
"""

import os

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

# 目前已知可用（实测过）的原厂硬件 bringup 入口。
# 它是历史部署目录里的文件；若后续替换成正式 package，用 vendor_launch 覆盖即可。
DEFAULT_VENDOR_LAUNCH = "/home/jetson/closed_loop/laser_bringup_tg_launch.py"


def _flag(context, name):
    value = LaunchConfiguration(name).perform(context)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *_args, **_kwargs):
    actions = []

    if not _flag(context, "start_vendor"):
        actions.append(
            LogInfo(
                msg=(
                    "[r2_mapping] start_vendor:=false —— 假定底盘/LiDAR/base_node "
                    "已在运行。本 launch 只负责挂接 TF 与后续检查。"
                )
            )
        )
    else:
        vendor_launch = LaunchConfiguration("vendor_launch").perform(context)
        if not os.path.isfile(vendor_launch):
            actions.append(
                LogInfo(
                    msg=(
                        "[r2_mapping][错误] 找不到原厂 bringup 文件："
                        + vendor_launch
                        + "\n[r2_mapping] 处理方式：\n"
                        "  1) vendor_launch:=<实际路径> 指定正确路径；或\n"
                        "  2) start_vendor:=false 直接挂到已运行的 stack 上。\n"
                        "[r2_mapping] 注意：本机是 Windows 的话该路径必然不存在，"
                        "请在 Jetson 上运行。"
                    )
                )
            )
        else:
            actions.append(
                LogInfo(msg="[r2_mapping] 启动原厂 bringup: " + vendor_launch)
            )
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(vendor_launch),
                    launch_arguments={
                        "robot_type": LaunchConfiguration("robot_type"),
                        # 原厂 bringup 用的参数名是 rplidar_type
                        "rplidar_type": LaunchConfiguration("lidar_type"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }.items(),
                )
            )

    if _flag(context, "publish_lidar_tf"):
        actions.append(
            LogInfo(
                msg=(
                    "[r2_mapping] publish_lidar_tf:=true —— 正在用 "
                    "static_transform_publisher 发布 base_link -> laser。\n"
                    "[r2_mapping] ⚠️ URDF 里同名的 base_link -> laser 必须先注释掉，"
                    "否则同一对父子会有两个发布者。"
                )
            )
        )
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_extrinsic_tf",
                output="screen",
                arguments=[
                    "--x", LaunchConfiguration("lidar_x"),
                    "--y", LaunchConfiguration("lidar_y"),
                    "--z", LaunchConfiguration("lidar_z"),
                    "--roll", LaunchConfiguration("lidar_roll"),
                    "--pitch", LaunchConfiguration("lidar_pitch"),
                    "--yaw", LaunchConfiguration("lidar_yaw"),
                    "--frame-id", LaunchConfiguration("lidar_parent_frame"),
                    "--child-frame-id", LaunchConfiguration("lidar_frame"),
                ],
            )
        )

    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "start_vendor",
            default_value="true",
            description="是否由本 launch 启动原厂底盘/LiDAR bringup",
        ),
        DeclareLaunchArgument(
            "vendor_launch",
            default_value=os.environ.get("R2_VENDOR_BRINGUP", DEFAULT_VENDOR_LAUNCH),
            description="原厂硬件 bringup 的 launch 文件路径",
        ),
        DeclareLaunchArgument(
            "robot_type", default_value="r2", description="车型标识，传给原厂 bringup"
        ),
        DeclareLaunchArgument(
            "lidar_type", default_value="4ROS", description="LiDAR 型号，传给原厂 bringup"
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="是否使用仿真时间"
        ),
        # ---- LiDAR 外参（仅在 publish_lidar_tf:=true 时生效）----
        DeclareLaunchArgument(
            "publish_lidar_tf",
            default_value="false",
            description="是否用 static_transform_publisher 覆盖 LiDAR 外参 TF",
        ),
        DeclareLaunchArgument("lidar_parent_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_frame", default_value="laser"),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.185"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
    ]

    return LaunchDescription(
        args
        + [
            OpaqueFunction(function=_launch_setup),
        ]
    )

