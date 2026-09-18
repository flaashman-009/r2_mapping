# 硬件与话题基线

数据来源：2026-08 在 R2 上的实测记录（见 `docs/06_test_log.md` 与历史项目
`car_drive_mode_switch/docs/yahboom_r2_car_info.md`）。

## 1. 平台

| 项目 | 值 |
|---|---|
| 车型 | Yahboom ROSMaster R2（Ackermann，前轮转向 + 后轮驱动） |
| 主控 | Jetson Orin NX SUPER 16GB，Ubuntu 22.04.5，内核 5.15-tegra |
| ROS | ROS2 Humble |
| `ROS_DOMAIN_ID` | 28 |
| 主机名 / 用户 | `yahboom` / `jetson`（密码见本地保管记录，**不要写进仓库**） |
| 常用 IP | `192.168.43.10` |
| 车体标识 | `my_robot_type=r2`、`my_lidar=4ROS` |

所有脚本和 launch 都**显式导出 `ROS_DOMAIN_ID=28`**，否则会看不到车里的话题。

## 2. 设备与串口

| 设备 | 型号 | 串口 | 备注 |
|---|---|---|---|
| LiDAR | YDLIDAR（驱动识别为 TG30 / 4ROS 配置） | `/dev/ydlidar` → `/dev/ttyUSB0`，512000 | `/scan`，约 10 Hz |
| 底盘控制板 | STM32 via CH340（`1a86:7523`） | `/dev/myserial` | 115200，**唯一占用** |
| 语音模块 | CH340（`1a86:7522`） | `/dev/myspeech` | 与底盘同芯片不同 PID，别搞混 |
| 相机 | Orbbec Astra Pro Plus | `/dev/video0/1` | 本期不使用 |
| 手柄 | DragonRise / PS2 | — | `/joy` → `/cmd_vel` |

udev 规则应保证 `7522 → myspeech`、`7523 → myserial`。若两者都映射错，
底盘驱动会打不开串口，见 `docs/05_troubleshooting.md`。

## 3. 关键节点

| 节点 | 作用 |
|---|---|
| `Ackman_driver_R2`（`driver_node`） | 底盘串口驱动，订阅 `/cmd_vel`，发布 `/vel_raw` |
| `r2_driver_monitor` | 底盘状态监控（20 Hz 采样版本） |
| `base_node_R2` | 里程计，发布 `/odom_raw` 与相关 TF |
| `imu_filter_madgwick` | IMU 姿态滤波 |
| `robot_state_publisher` | 依据 URDF 发布静态/动态 TF |
| `yahboom_joy_R2` + `joy_node` | PS2 遥控 |
| `slam_toolbox` | 建图（本项目用 `async_slam_toolbox_node`） |
| `nav2_map_server` / `nav2_amcl` | 地图加载与定位复测 |

## 4. 关键话题

| 话题 | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | ~10 Hz | LiDAR 原始扫描 |
| `/scan_filtered` | `sensor_msgs/LaserScan` | ~10 Hz | 本项目可选滤波输出 |
| `/odom_raw` | `nav_msgs/Odometry` | ~10 Hz | 轮速死推算里程计 |
| `/odom` | `nav_msgs/Odometry` | ~10–20 Hz | EKF 融合里程计 |
| `/imu/data_raw` | `sensor_msgs/Imu` | ~10 Hz | 原始 IMU |
| `/imu/yaw_deg` | `std_msgs/Float64` | ~10 Hz | 板载融合 yaw（单位：度） |
| `/cmd_vel` | `geometry_msgs/Twist` | — | `linear.x` = 速度 m/s；`linear.y` = 转向角/1000 |
| `/vel_raw` | `geometry_msgs/Twist` | ~10 Hz | 底盘反馈，`linear.y` 为实际转角（度） |
| `/joint_states` | `sensor_msgs/JointState` | ~10 Hz | 关节角（弧度） |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | — | TF 树 |
| `/map` | `nav_msgs/OccupancyGrid` | ~0.5 Hz | `slam_toolbox` 输出 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | — | AMCL 定位结果 |

### 转向指令的坑

`/cmd_vel.linear.y = 0` **不会回正**，回中需要发一个非零小角度。
遥控建图时不涉及，但如果用脚本发 `/cmd_vel`，务必显式回正。

## 5. 车辆几何（实测 2026-08-27）

| 参数 | 值 |
|---|---|
| 轴距 L | 0.2681 m |
| 轮距 d | 0.1646 m |
| 最大转向角 | 45°（0.7854 rad），正负均可 |
| 转向比 | 1.000 |
| 零位偏置 | 0° |
| 转向微调 `steer_trim_deg` | −2.9° |

### 动力学（后轮架起实测）

| 参数 | 值 |
|---|---|
| 首次速度响应延迟 | 0.11 s |
| 90% 上升时间 | 约 0.60 s |
| 稳态速度（指令 0.5 m/s） | 0.50–0.51 m/s |
| 最大加速度估算 | 约 0.7 m/s² |

**建图建议速度：0.2–0.4 m/s**。阿克曼车在低速下转向响应慢，
速度过高会让 slam_toolbox 的 scan matching 直接失配。

## 6. 环境依赖

本项目的 launch 默认复用小车上的原厂 ROS2 工作区：

```bash
source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
```

以及历史部署目录 `/home/jetson/closed_loop/`（其中的
`laser_bringup_tg_launch.py` 是目前唯一已知可用的硬件 bringup 入口）。

`r2_mapping_bringup` 通过 launch 参数 `vendor_launch` 引用它，
路径可覆盖，方便后续替换成正式的原厂 package。
