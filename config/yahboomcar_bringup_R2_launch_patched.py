from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

# 本项目自己的脚本目录（里程计静止门等节点放在这里）
R2_MAPPING_SCRIPTS = os.path.expanduser('~/r2_mapping/scripts')


def generate_launch_description():
    if not os.environ.get("PRINTED"):
        os.environ["PRINTED"] = "1"
        print("---------------------robot_type = r2---------------------")
    urdf_tutorial_path = get_package_share_path('yahboomcar_description')
    default_model_path = urdf_tutorial_path / 'urdf/yahboomcar_R2.urdf.xacro'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/yahboomcar.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='false',
                                            description='Whether to publish the tf from the original odom to the base_footprint')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    imu_filter_config = os.path.join(              
        get_package_share_directory('yahboomcar_bringup'),
        'param',
        'imu_filter_param.yaml'
    ) 

    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Ackman_driver_R2',
    )

    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_R2',
        # 当使用ekf融合时，该tf有ekf发布
        parameters=[{
            'pub_odom_tf': LaunchConfiguration('pub_odom_tf'),
            'linear_scale_x': 1.0,
            'linear_scale_y': 1.0,
            # ⚠️ 必须设！base_node_R2.cpp 里 wheelbase 的默认值是 0.25 m，
            # 而 R2 实测轴距是 0.2681 m，差了 7.24%。
            #
            # 源码里的航向计算：
            #     R = wheelbase / tan(steer_angle)
            #     ω = v / R = v * tan(steer_angle) / wheelbase
            # wheelbase 偏小 7.24% -> ω 偏大 7.24% -> **每个转弯都多转 7.24%**
            #
            # 走一圈转 4 个 90° 直角 = 360°，累积航向误差约 26°，
            # 地图被"拧"的主因之一。
            'wheelbase': 0.2681,
        }]
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config]
    )
    
    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_localization'), 'launch'),
            '/ekf_x1_x3_launch.py'])
    )

    # ---- 里程计静止门 odom_gate（2026-09-19 新增）----
    # 为什么必须有：
    #   车停住时底盘固件不更新转向（前轮卡在最后一次的角度，实测 +19°），
    #   而 base_node_R2 用 ω = vx·tan(δ)/L 算航向 —— 它拿着这个卡住的转角
    #   加上停车时轮子的微小往复（vx 有 ±0.1 m/s 级噪声），
    #   算出持续非零角速度，6 分钟里航向累积了 6.4 圈、位置画了个圆。
    #   EKF 融合它的 vx，就把这条"螺旋"积分出来 → 漂 31 米。
    #   结果就是"车停下来，点云突然对不上地图"。
    #
    # 这个节点把"车速低于 0.03 m/s"时的 twist 置零，让 EKF 不积分幻影运动。
    # 它必须和 EKF 一起启动 —— 因为 EKF 的 odom0 已经指向 /odom_gated，
    # 这个节点不跑，EKF 就收不到里程计，/odom 会完全没有。
    odom_gate_node = ExecuteProcess(
        cmd=['python3', os.path.join(R2_MAPPING_SCRIPTS, 'odom_gate.py'),
             '--ros-args',
             '-p', 'input_topic:=/odom_raw',
             '-p', 'output_topic:=/odom_gated'],
        output='screen'
    )

    # ---- 原厂手柄节点 yahboom_joy_R2：默认关掉 ----
    #
    # 为什么要关（2026-09-14 实测，踩了三次）：
    #
    # 1. 它的解锁键是 buttons[9]，而这台手柄**不产生索引 9**，
    #    所以正常情况下它永远解锁不了、不发指令。
    #
    # 2. 但一旦不知怎么被解锁（buttons[9] 偶然触发），它的转向增益是
    #       linear.y = axes[2] * yspeed_limit(5.0)
    #    即 ±5000°，而打满前轮只需要 0.045 —— **推一下摇杆就打到满舵**。
    #    实测前轮被它卡在 -45° 回不来。
    #
    # 3. 它还把 buttons[11] 当蜂鸣器开关，而 11 正好是 ps2_teleop 的解锁键，
    #    一按就通过 /Buzzer 让小车一直响。
    #
    # 结论：这个节点在这台车上没有任何用处，只会制造麻烦。
    # 手柄控制统一走 r2_mapping 的 ps2_teleop（有死区、限幅、看门狗）。
    #
    # 想临时启用：启动时加 use_factory_joy:=true
    use_factory_joy_arg = DeclareLaunchArgument(
        'use_factory_joy', default_value='false',
        description='是否启动原厂手柄节点（默认关，见上方注释）')
    yahboom_joy_node = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy_R2',
        condition=IfCondition(LaunchConfiguration('use_factory_joy')),
        remappings=[('Buzzer', 'Buzzer_disabled_by_r2_mapping')],
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        pub_odom_tf_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        #rviz_node,
        driver_node,
        base_node,
        imu_filter_node,
    ekf_node,
    odom_gate_node,
        use_factory_joy_arg,
        yahboom_joy_node,
        joy_node
    ])
