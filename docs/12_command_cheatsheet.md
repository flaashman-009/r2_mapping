# 命令速查手册

所有常用命令 + 典型问题的排查步骤。**每次先看这里，不用再问。**

---

# 第一部分：所有命令

## 0. 环境（每个新终端第一句）

```bash
source ~/r2_mapping/scripts/env.sh
```

这一句干了四件事：`ROS_DOMAIN_ID=28`、source ROS、source 原厂两个工作区、
source 本项目。**不 source 就看不到车上的话题。**

---

## 1. 启动

### 1.1 只起硬件（底盘 + 雷达 + IMU + EKF + TF）

```bash
ros2 launch yahboomcar_nav laser_bringup_launch.py
```

起了：`Ackman_driver_R2`、`base_node_R2`、`imu_filter_madgwick`、`ekf_filter_node`、
`robot_state_publisher`、`ros2 launch yahboomcar_nav laser_bringup_launch.py`

### 1.2 建图（含硬件，一条命令）

```bash
ros2 launch ~/r2_mapping/src/r2_mapping_bringup/launch/mapping_slam_toolbox.launch.py
```

> ⚠️ 用**文件路径**不是包名——`r2_mapping` 没 colcon build 过。
> ⚠️ 起之前先停掉已经跑着的硬件，否则两个驱动抢串口。

### 1.3 导航（两个终端）

```bash
# 终端 1
ros2 launch yahboomcar_nav laser_bringup_launch.py

# 终端 2
ros2 launch yahboomcar_nav navigation_teb_launch.py \
  map:=/home/jetson/r2_mapping/maps/room_04.yaml \
  params_file:=/home/jetson/r2_mapping/config/nav_r2.yaml
```

> 这个 launch **不含 RViz**，要另开。

### 1.4 RViz

```bash
# 建图（轻量配置，省 CPU）
rviz2 -d ~/r2_mapping/src/r2_mapping_bringup/rviz/rviz_mapping_lite.rviz

# 导航
rviz2 -d ~/r2_mapping/src/r2_mapping_bringup/rviz/rviz_localization.rviz
```

### 1.5 手柄遥控

```bash
python3 ~/r2_mapping/src/r2_mapping_tools/r2_mapping_tools/ps2_teleop.py
```

### 1.6 相机

```bash
# 启动
ros2 launch astra_camera astro_pro_plus.launch.xml \
  enable_color:=true enable_depth:=true enable_ir:=false \
  enable_point_cloud:=false depth_registration:=true

# 看图（浏览器打开 http://localhost:8080）
ros2 run web_video_server web_video_server
```

---

## 2. 停止

```bash
# 1) 各终端 Ctrl-C

# 2) 一键清硬件/建图相关
bash ~/r2_mapping/scripts/restart_hardware.sh --no-start

# 3) 确认
ros2 node list
fuser -v /dev/myserial
```

> Nav2 那套要**先在它自己的终端 Ctrl-C**，脚本目前不清它。

---

## 3. 干净重启

```bash
bash ~/r2_mapping/scripts/restart_hardware.sh
```

它会：停所有硬件进程 → 清 fastrtps 共享内存 → 停 ros2 daemon → 重启硬件 → 自检。

**什么时候用**：
- 网络换了（IP 变了）之后，新进程看不到 ROS 图
- 驱动卡死（进程活着但 `ros2 node list` 里没有）
- 报 `RTPS_TRANSPORT_SHM Error: Failed init_port`

---

## 4. 状态检查

```bash
# 有哪些节点
ros2 node list

# 有哪些话题
ros2 topic list

# 某个话题的频率
ros2 topic hz /scan
ros2 topic hz /odom

# 谁在发 /cmd_vel（排查必用）
ros2 topic info /cmd_vel

# 串口被谁占
fuser -v /dev/myserial

# 关键进程
ps -eo pid,etime,pcpu,cmd --no-headers | grep -e Ackman -e ydlidar -e ekf -e base_node

# TF 链路
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map laser
```

---

## 5. 地图

```bash
# 保存
ros2 launch yahboomcar_nav save_map_launch.py \
  map_path:=/home/jetson/r2_mapping/maps/room_05

# 修 free_thresh（不修的话 unknown 会被当成 free）
sed -i 's/free_thresh: 0.25/free_thresh: 0.196/' \
  /home/jetson/r2_mapping/maps/room_05.yaml

# 评估质量
python3 ~/r2_mapping/src/r2_mapping_tools/r2_mapping_tools/map_eval.py \
  /home/jetson/r2_mapping/maps/room_05.yaml
```

---

## 6. 诊断工具

```bash
# 航向漂移率（静止测 IMU；行驶时对比 odom vs IMU）
python3 -u ~/r2_mapping/scripts/imu_drift.py --duration 60

# 雷达有没有扫到自己（车体自遮挡）
python3 ~/r2_mapping/scripts/scan_stats.py --near 0.4

# 手柄原始数据（绕过 ROS，判断是手柄坏还是驱动卡）
python3 ~/r2_mapping/scripts/js_raw.py

# 前轮回正
bash ~/r2_mapping/scripts/steer_center_cmd.sh

# 里程计尺度标定（推车模式）
python3 ~/r2_mapping/src/r2_mapping_tools/r2_mapping_tools/odom_calibrate.py \
  --ros-args -p push_mode:=true

# 校验 EKF 参数是否改对了
bash ~/r2_mapping/scripts/check_ekf_config.sh

# 串口占用检查
bash ~/r2_mapping/scripts/serial_guard.sh
```

---

## 7. 导航操作

```bash
# 设初始位姿（不用 RViz 时）
~/r2_mapping/scripts/set_initial_pose.sh X Y YAW_DEG
```

或者在 RViz 里点 **2D Pose Estimate**。

---

## 8. 急停

```bash
# 1) 断电
# 2) 导航终端 Ctrl-C
# 3) 杀串口占用者
fuser -k /dev/myserial
```

---

---

# 第二部分：三个典型问题的排查

## 问题 1：舵机一直在调方向（像 F1 暖胎）

**排查顺序**：从最可能到最不可能，每一步都有明确判据。

### 步骤 1 —— 几个节点在发 `/cmd_vel`？（最常见）

```bash
ros2 topic info /cmd_vel
```

| 结果 | 结论 | 处理 |
|---|---|---|
| `Publisher count: 2` | **就是它**：两个节点抢 `/cmd_vel`，舵机在两边来回跳 | 找到第二个是谁（见下），关掉 |
| `Publisher count: 1` | 正常，往下走 | |

**找出第二个发布者是谁**：

```bash
ros2 node list | grep -E "ps2_teleop|joy_ctrl"
```

- 有 `ps2_teleop` → **按一次解锁键上锁**（上锁后它不再发），或者 Ctrl-C 它
- 有 `joy_ctrl` → 原厂手柄节点，我们的 launch 已经禁掉了，如果出现了说明 launch 版本不对

### 步骤 2 —— 指令值本身在抖吗？

```bash
ros2 topic echo /cmd_vel
```

盯住 `linear.y`（转向指令）：

| 现象 | 结论 |
|---|---|
| 数值在小幅来回跳（如 0.001 ↔ 0.012） | **TEB 本身在抖**（见下方说明） |
| 数值稳定不动，但舵机还在动 | 不是指令问题，是舵机/机械 |

**TEB 抖动是它的特性**：TEB 每个控制周期（20 Hz）重新优化轨迹，
转向输出天然比 DWB 抖。阿克曼车更明显。

**缓解办法**：
- 换 DWB：`ros2 launch yahboomcar_nav navigation_dwa_launch.py ...`
- 或者调 TEB 参数（`min_vel_x`、轨迹优化权重）

### 步骤 3 —— 定位在抖吗？

```bash
ros2 topic echo /amcl_pose --field pose.pose.position
```

| 现象 | 结论 |
|---|---|
| 位置稳定（车没动时不变） | 定位没问题 |
| 位置在小幅跳 | 定位抖 → 路径抖 → 舵机抖 |

### 步骤 4 —— 局部路径在抖吗？

RViz 里加 `/local_plan`（见问题 3），看它是不是在抖。

### 补充：舵机没有死区

底盘固件对每条指令都执行，**1° 的变化它也会动**。20 Hz 下发时，
任何微小抖动都会表现为舵机持续"找方向"。

**这条可以通过降低控制频率或加死区缓解，但优先级最低**——
先确认是不是前三条。

---

## 问题 2：有没有避障？

### 结论

**有避障，但是"反应式"的**（基于代价地图），不是智能避障。

| 能力 | 有没有 | 靠什么 |
|---|---|---|
| 地图里的墙 | ✅ | 全局代价地图 static layer |
| **实时扫到的新障碍** | ✅ | obstacle layer，订阅 `/scan` |
| 绕开障碍 | ✅ | 全局重规划 + 局部避让 |
| 障碍膨胀（留安全距离） | ✅ | `inflation_radius` |
| 动态障碍预测 | ❌ | 不预测人往哪走 |
| 独立急停模块 | ❌ | `collision_monitor` 默认没启用 |
| 语义理解 | ❌ | — |

### 怎么验证（直接测，最直观）

**1. 导航跑起来，发个目标让车走**

**2. 在路径上放个箱子**

**3. 在 RViz 里看三件事**

| 看什么 | 期望 |
|---|---|
| `/local_costmap/costmap` | 箱子位置**立刻出现障碍块** |
| `/plan`（绿线） | **绕开**箱子重新规划 |
| 车 | 停下来等、或者绕过去 |

三条都对 = 避障正常工作。

### 如果没反应，查这两条

```bash
# 代价地图有没有订阅激光
ros2 topic info /scan

# 障碍层的参数
ros2 param get /local_costmap/local_costmap obstacle_layer.scan.obstacle_range
ros2 param get /local_costmap/local_costmap obstacle_layer.scan.topic
```

### 已知的坑：阿克曼车 + TEB 倾向停下

遇到障碍时，TEB 在阿克曼车上常表现为**直接停下**而不是绕
（我们之前在 `room_02` 上见过 `Collision Ahead - Exiting DriveOnHeading`）。

**想改善**：
- 换 DWB（`navigation_dwa_launch.py`）
- 调小 `inflation_radius`（0.2 → 0.15），让车敢走窄缝

---

## 问题 3：RViz 里怎么看全局/局部路径

### 加图层（一次配置，以后自动保存）

**1.** 左侧 **Displays** 面板 → 底部 **Add**

**2.** 切到 **By topic** 标签页

**3.** 展开 `/plan` → 双击 **Path**

**4.** 同样再加 `/local_plan`

**5.** 在左侧选中它们，改 `Color` 区分：

| 话题 | 含义 | 建议颜色 |
|---|---|---|
| `/plan` | **全局路径**（规划器算的完整路线） | 绿色 |
| `/local_plan` | **局部路径**（控制器实际在追的） | 红色 |

### 顺便加代价地图（看避障效果）

同样 Add → By topic：

| 话题 | 类型 | 含义 |
|---|---|---|
| `/global_costmap/costmap` | Map | 全局代价地图 |
| `/local_costmap/costmap` | Map | 车周围实时更新的局部代价地图 |

加的时候 `Color Scheme` 选 **`costmap`**。

**代价地图怎么读**：

| 颜色 | 含义 |
|---|---|
| 黑/深色 | 障碍 |
| 外围渐变圈 | 膨胀区（车不能进） |
| 浅色 | 可通行 |
| 灰蓝 | 未知 |

### 怎么读这三条线

| 现象 | 说明 |
|---|---|
| 绿线和红线**基本重合** | 车在正常跟踪路径 ✅ |
| 红线**大幅偏离**绿线 | 车走偏了，或被障碍挤开了 |
| 红线**消失** | 控制器没在算——可能卡住、没目标、或生命周期没激活 |
| 绿线**没有** | 规划器没出路径——目标不可达，或代价地图有问题 |
| 两条线都在**抖动** | 定位抖（见问题 1 步骤 3） |

### ⚠️ CPU 提醒

代价地图图层很吃 CPU（全局的尺寸大，尤其费）。
**排查完就取消勾选**，别一直开着。

---

# 第三部分：三句话总结

| 问题 | 一句话答案 |
|---|---|
| **舵机一直调方向** | 先跑 `ros2 topic info /cmd_vel`，**2 个发布者就是根因**；1 个的话是 TEB 抖动 |
| **有没有避障** | **有**（基于代价地图的反应式避障），但没有动态预测和独立急停 |
| **怎么看路径** | RViz → Add → By topic → `/plan`（绿，全局）+ `/local_plan`（红，局部） |
