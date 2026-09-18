# 项目说明（Project Charter）

> 这份文档是项目的输入契约。任何实现上的取舍，先回到这里对一下边界。

## 1. 项目目标

在 Yahboom R2 阿克曼小车上完成 **ROS2 2D 建图**，并把地图交付到可被
Nav2 `map_server` / `amcl` 直接加载的程度。

## 2. 硬件

| 项 | 说明 |
|---|---|
| 主控 | Jetson Orin NX |
| 系统 / ROS | Ubuntu 22.04 / ROS2 Humble |
| ROS_DOMAIN_ID | 28 |
| LiDAR | YDLIDAR 4ROS，话题 `/scan`，约 10 Hz |
| 底盘通信 | CH340 USB 串口 `/dev/myserial` → STM32（**不是 CAN**） |
| 原厂模式 | Ackermann |
| 驱动 | `Ackman_driver_R2` / `r2_driver_monitor` |
| 已有节点/话题 | `base_node_R2`、`/odom_raw`、`/imu/data_raw`、`/imu/yaw_deg`、`/tf` |

## 3. 关键约束

1. 建图先使用**原厂 Ackermann 模式**，不使用软件差速模式 —— 差速模式没有完整的 odom/TF。
2. `/dev/myserial` 同一时间只能被一个驱动进程占用。
3. 需要检查 LiDAR、底盘、IMU 的**时间戳和 TF**。
4. 地图要能保存为 **PGM + YAML**，并能被 Nav2 `map_server` / `amcl` 加载。
5. 需要关注**里程计漂移、雷达安装外参、时间同步**。

## 4. 优先完成顺序

1. 启动底盘、LiDAR、`base_node`。
2. 确认 `/scan`、`/odom_raw`、`/imu/data_raw`、`/tf` 正常。
3. RViz 显示 LaserScan、TF、Odometry、RobotModel。
4. 用 `slam_toolbox` **online_async** 建图。
5. 遥控低速走闭合路径，保存地图。
6. 用保存的地图启动 AMCL 做定位复测。

## 5. 技术检查清单

| 项目 | 检查内容 |
|---|---|
| 传感器 | `/scan` 是否稳定 10 Hz |
| 里程计 | `/odom_raw` 是否连续、方向正确 |
| IMU | `/imu/yaw_deg` 是否稳定、无跳变 |
| TF | `map→odom→base_link→laser` 是否完整 |
| 时间 | LiDAR、odom、IMU 时间戳是否接近 |
| 外参 | LiDAR 相对 `base_link` 的 x/y/z/yaw |
| 地图 | 分辨率、occupied/free/unknown 是否合理 |
| 闭环 | 回到起点时地图是否重合 |

## 6. 明确不做（本期范围外）

- 不实现自主导航 / 路径规划 / 避障（Nav2 只用来验收定位）。
- 不做软件差速底盘驱动（属于另一个项目 `car_drive_mode_switch`）。
- 不改动原厂 `Ackman_driver_R2` 与底盘固件。
- 不做 3D 建图 / 点云地图在线构建。

## 7. 显式风险与前置条件

- 底盘控制板 USB 存在历史硬件隐患（枚举失败、`/dev/myserial` 缺失）。
  若串口起不来，建图任务直接阻塞，属于硬件问题，不在软件范围内解决。
- `/odom_raw` 的 yaw 由轮速死推算，长距离不可信。建图链路中
  **航向以 IMU 为准**，`/odom_raw` 只提供平移分量。
- LiDAR 外参尚未正式标定，属于本项目必须补齐的一项。

