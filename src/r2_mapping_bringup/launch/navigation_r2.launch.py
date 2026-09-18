#!/usr/bin/env python3
"""R2 导航 launch（本项目专用，2026-09-16 大改）。

## 为什么不能用 Nav2 自带的 navigation_launch.py

三个问题，每一个都会让这台车出故障：

1. **velocity_smoother 会把转向吃掉**
   Nav2 的 velocity_smoother 按"转向 = angular.z"设计，而我们最终要走的
   是 linear.y（底盘的直接转向角通道）。实测发 0.02 进去、出来是 0.0。

2. **转向语义不匹配**（本次修改的核心）
   底盘对 angular.z 的处理有两个坑（2026-09-16 实测）：
     - 车静止时 angular.z 完全不起作用
     - **行驶中发 0 时"保持上一次角度"，不回正**
       实测运动 567 帧里有 67 帧 angular.z=0，那 67 帧前轮全部卡在 -25°，
       车在"以为直行"的时候其实在画弧 —— 点云持续偏、误差累积、AMCL 迷失。
   所以本 launch 在 Nav2 和底盘之间插一个 cmd_vel_ackermann 适配节点：
   自己按阿克曼模型算转向角，走 linear.y，并做限幅 + 转速率限制。

3. **扫描输入必须和建图一致**
   地图是用 /scan_filtered 建的（屏蔽了车尾 ±120–150° 的车体自遮挡），
   但 AMCL 和代价地图默认读原始 /scan，带着 144 个车身近点。车一转，
   这些点就扫过整个场景，持续污染匹配。本 launch 统一走 scan_filter_node。

## 数据流

    controller_server ─┐
                       ├─ /cmd_vel_nav ─→ cmd_vel_ackermann ─→ /cmd_vel ─→ 底盘
    behavior_server  ──┘                      (算出转向角，走 linear.y)

    /scan ─→ scan_filter_node ─→ /scan_filtered ─→ amcl + 代价地图

## 用法

    ros2 launch ~/r2_mapping/src/r2_mapping_bringup/launch/navigation_r2.launch.py \
        map:=/home/jetson/r2_mapping/maps/room_04.yaml

    硬件要另外起（本 launch 不含硬件）：
        ros2 launch yahboomcar_nav laser_bringup_launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    R2_MAPPING = os.path.expanduser("~/r2_mapping")
    default_map = os.path.join(R2_MAPPING, "maps", "room_04.yaml")
    default_params = os.path.join(R2_MAPPING, "config", "nav_r2.yaml")
    ack_script = os.path.join(R2_MAPPING, "scripts", "cmd_vel_ackermann.py")
    watchdog_script = os.path.join(R2_MAPPING, "scripts", "lost_watchdog.py")
    guard_script = os.path.join(R2_MAPPING, "scripts", "localization_guard.py")
    filter_script = os.path.join(
        R2_MAPPING, "src", "r2_mapping_perception",
        "r2_mapping_perception", "scan_filter_node.py")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")
    # 转向限幅不在这里，全部在 nav_r2.yaml 的 cmd_vel_ackermann 段

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    # Nav2 的转向输出先送到 /cmd_vel_nav，由适配节点接管
    nav_out = remappings + [("cmd_vel", "/cmd_vel_nav")]

    # ---------------- 扫描滤波（和建图保持一致）----------------
    scan_filter = ExecuteProcess(
        cmd=["python3", filter_script,
             "--ros-args",
             "-p", "input_topic:=/scan",
             "-p", "output_topic:=/scan_filtered",
             "-p", "min_range:=0.0",
             "-p", "max_range:=12.0",
             "-p", "blind_sectors:=120:150,-150:-120"],
        output="screen",
    )

    # ---------------- 转向适配（本 launch 的核心）----------------
    # 2026-09-18：转向限幅全部挪到 nav_r2.yaml 的 cmd_vel_ackermann 段，
    # 这里只给参数文件路径 —— 以后所有限幅只改 nav_r2.yaml 一个文件。
    ack_adapter = ExecuteProcess(
        cmd=["python3", ack_script, "--ros-args",
             "--params-file", params_file],
        output="screen",
    )

    # ---------------- 定位护卫 ----------------
    # 协方差过大 → 停车 + 提示重新给位姿；恢复后自动解除。
    # 依据：9-18 记录里 4 次 30~40 米的瞬移，前兆都是协方差 0.1 → 85 → 224。
    loc_guard = ExecuteProcess(
        cmd=["python3", guard_script, "--ros-args",
             "-p", "halt_topic:=/cmd_vel_halt"],
        output="screen",
    )

    # ---------------- 迷失看门狗（2026-09-17 暂时摘掉，按需再开）----------------
    # 现象：车停着不动时 AMCL 会失去约束、自发散（实测残差爬到 0.76、
    # 协方差 155、位姿瞬移 12 m；9-16 更严重的一次跳 90 m / 177°）。
    # 脚本保留在 scripts/lost_watchdog.py，需要时把下面这段放回即可：
    #
    #   lost_watchdog = ExecuteProcess(
    #       cmd=["python3", watchdog_script, "--ros-args",
    #            "-p", "scan_topic:=/scan_filtered",
    #            "-p", "halt_topic:=/cmd_vel_halt"],
    #       output="screen")
    #   然后把它加进下面的节点列表。

    # ---------------- 定位 ----------------
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"yaml_filename": map_yaml,
                                  "use_sim_time": use_sim_time}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    lifecycle_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time,
                     "autostart": autostart,
                     "node_names": ["map_server", "amcl"]}],
    )

    # ---------------- 导航 ----------------
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=nav_out,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=nav_out,
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    lifecycle_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time,
                     "autostart": autostart,
                     "node_names": ["controller_server",
                                    "smoother_server",
                                    "planner_server",
                                    "behavior_server",
                                    "bt_navigator"]}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map,
                              description="地图 YAML 路径"),
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="Nav2 参数文件"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        # 转向限幅相关的启动参数已移除 —— 现在全在 nav_r2.yaml 的
        # cmd_vel_ackermann 段里改（见该文件的注释）。

        LogInfo(msg=["[r2_mapping] R2 导航：地图=", map_yaml,
                     " 参数=", params_file]),
        LogInfo(msg="[r2_mapping] 转向由 cmd_vel_ackermann 适配："
                    "ω -> δ=atan(ωL/v) -> linear.y"),

        scan_filter,
        ack_adapter,
        loc_guard,
        map_server,
        amcl,
        lifecycle_localization,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_navigation,
    ])
