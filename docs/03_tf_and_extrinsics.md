# TF 树与 LiDAR 外参标定

## 1. 目标 TF 树

```
map                      <- slam_toolbox（建图时）/ amcl（定位时）发布
 └─ odom                 <- 里程计源（base_node_R2 / EKF）
     └─ base_footprint   <- 地面投影
         └─ base_link    <- 车体几何中心
             ├─ laser    <- LiDAR 安装位置（静态）
             └─ imu_link <- IMU 安装位置（静态）
```

实际 URDF 里 `base_footprint` 与 `base_link` 的父子方向可能相反，
只要端点连通即可。自检工具按「能不能查到变换」判定，不假定方向。

### 各段的发布者

| 变换 | 发布者 | 频率 |
|---|---|---|
| `map → odom` | 建图：`slam_toolbox`；定位：`amcl` | 建图 ~50 Hz，定位 ~10 Hz |
| `odom → base_footprint` | `base_node_R2`（或 EKF） | ~10–30 Hz |
| `base_footprint → base_link` | `robot_state_publisher` | 静态 |
| `base_link → laser` | `robot_state_publisher`（URDF） | 静态 |
| `base_link → imu_link` | `robot_state_publisher`（URDF） | 静态 |

**`map → odom` 只能有一个发布者。** 建图和定位不能同时跑，
否则 TF 会来回打架，RViz 里能看到地图跳动。

## 2. 排查命令

```bash
# 一次性看整条链路
ros2 run tf2_ros tf2_echo map laser
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser

# 看谁在发 TF
ros2 run tf2_tools view_frames        # 生成 frames.pdf
ros2 topic echo /tf_static --once

# 看时间戳与延迟
ros2 run tf2_ros tf2_monitor odom base_link
```

## 3. LiDAR 外参标定流程

外参 = `base_link → laser` 的 `x / y / z / roll / pitch / yaw`。

当前状态：**未标定（占位值）**。占位值在
`config/lidar_extrinsics.yaml`，标定完再写回 URDF 或该文件。

### 3.1 测量初始值（卷尺）

以 `base_link` 原点（车体几何中心/后轴中心，按 URDF 定义）为基准：

| 量 | 方法 |
|---|---|
| `x` | 车体中心向前到 LiDAR 光心的水平距离，前为正 |
| `y` | 车体中心线到 LiDAR 光心的横向偏移，左为正 |
| `z` | 地面到 LiDAR 光心的竖直高度（相对 `base_link` 的 z） |
| `yaw` | LiDAR 的 0° 方向与车头方向的夹角，逆时针为正 |
| `roll/pitch` | 用水平尺确认，一般设 0 |

### 3.2 RViz 精调

1. 把车推到一面平墙前，**车头正对墙**，距离约 1.0 m。
2. RViz 固定坐标系设 `base_link`，加载 LaserScan（`/scan`）和 RobotModel。
3. 调 `yaw` 让墙面在扫描里是**一条水平直线**（垂直于车头方向）。
4. 调 `x` 让墙的距离与卷尺一致；调 `y` 让左右墙距离对称。
5. 调 `z` 只影响 RViz 中的高度显示，对 2D SLAM 影响小，但仍建议量准。

### 3.3 交叉验证（必做）

1. 车原地转 90°，再对同一面墙重复步骤 2–4，墙面偏差应 < 0.05 m。
2. 再转 90°（共 180°），确认墙面距离读数一致。
3. 侧对墙慢速平移 1 m，扫描中的墙线不应用肉眼可见的倾斜。

只要两次朝向给出的外参差超过 2 cm / 2°，说明 `yaw` 没标好。

### 3.4 写回

标定结果写回 `config/lidar_extrinsics.yaml`，然后二选一：

1. 改 URDF 中 `base_link → laser` 的 `<origin xyz rpy>`（推荐，长期方案）；
2. 用 `sensors.launch.py publish_lidar_tf:=true`
   临时发一条 `static_transform_publisher`。

> ⚠️ 方案 2 必须先把 URDF 里同名的 `base_link → laser` 注释掉，
> 否则同一对父子有两个发布者，TF 值会随机抖动。

## 4. 常见 TF 问题

| 现象 | 可能原因 |
|---|---|
| `map → odom` 查不到 | 只起了硬件没起 SLAM/AMCL |
| TF 有，但 RViz 里扫描旋转 90° | 外参 `yaw` 错，或 URDF 里 LiDAR 朝上没有转正 |
| 建图中地图突然整体跳一段 | `map → odom` 有两个发布者，或 scan matching 失配后重定位 |
| `TF_OLD_DATA` / extrapolation 报错 | 时间戳不同步（见 `docs/05_troubleshooting.md`） |
| 扫描贴着一圈小点跟着车走 | 车体自遮挡，需要开 `scan_filter_node` 屏蔽车体扇区 |

