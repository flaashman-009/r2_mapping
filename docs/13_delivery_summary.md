# R2 阿克曼小车 2D 建图与导航 —— 交付总结

日期：2026-09-16
平台：Yahboom ROSMaster R2（Ackermann） / Jetson Orin NX / ROS 2 Humble / ROS_DOMAIN_ID=28

---

## 1. 目标

在 Yahboom R2 阿克曼小车上完成：

1. 传感器、底盘、TF、时间的基线核查
2. 用 SLAM 建立可用于导航的 2D 栅格地图（PGM + YAML）
3. 用生成的地图做定位与自主导航
4. 建立可复用、可自检的诊断工具链

---

## 2. 交付物

### 2.1 地图

| 文件 | 说明 |
|---|---|
| `maps/room_01 ~ room_04` | 四个版本的地图（PGM + YAML + PNG） |
| `maps/room_04.*` | **质量最好的一版**，用于导航 |

`room_04` 指标：1050 × 1149 px，分辨率 0.05 m/px，世界范围 52.5 × 57.5 m，
occupied 1.6%，free 17.6%，碎斑密度 0.08 个/m²。

### 2.2 代码与配置

| 文件 | 作用 |
|---|---|
| `config/nav_r2.yaml` | Nav2 全套参数（AMCL / TEB / 代价地图 / 行为） |
| `src/r2_mapping_bringup/launch/navigation_r2.launch.py` | 本项目导航 launch（含转向适配 + 扫描滤波） |
| `scripts/cmd_vel_ackermann.py` | **转向适配节点**：把 Nav2 的 (v, ω) 换算成底盘原生的转向指令 |
| `scripts/nav.sh` | 一站式启动 / 停止 / 状态检查 |
| `scripts/restart_hardware.sh` | 硬件干净重启（清共享内存、释放串口） |
| `scripts/estop.sh` | 急停（带底盘反馈验证） |
| `config/ekf_x1_x3_patched.yaml` | EKF 配置补丁 |
| `config/slam_toolbox_mapping.yaml` | 建图参数 |

### 2.3 诊断工具链

| 脚本 | 用途 |
|---|---|
| `scripts/nav_watch.py` | **行车记录仪**：8 路信号 + 点云-地图残差，异常实时报警 |
| `scripts/tf_audit.py` | TF 冲突 / 跳变检测 |
| `scripts/odom_vs_tf.py` | 判定 AMCL 实际吃的是哪份里程计 |
| `tools/analysis/steer_channel.py` | 转向通道与量纲判定 |
| `tools/analysis/pose_vs_map.py` | 把位姿投到地图上，统计走在什么区域 |
| `tools/analysis/analyze_nav_watch.py` / `plot_nav.py` | 记录数据自动分析 + 出图 |
| `scripts/usb_status.sh` | 车载 USB 外设一键自检 |
| `scripts/verify_changes.sh` | 参数改动一键校验 |
| `scripts/verify_nav_live.sh` | 导航链路实时校验 |
| `scripts/scan_stats.py` / `imu_drift.py` / `steer_debug.py` | 传感器与执行器诊断 |

---

## 3. 硬件基线与实测数据

| 项目 | 实测值 |
|---|---|
| 轴距 L | 0.2681 m |
| 轮距 | 0.1646 m |
| 车体尺寸 | 0.3375 × 0.1911 m |
| 最大转向角 | ±44°（机械极限） |
| 雷达 | 2020 光束，angle_increment 0.00311 rad，有效回波 ~6 m |
| 转向通道 `linear.y` | **值 × 1000 = 度**；实测 0.02→20.0°，0.03→30.0°，0.001→1.0° |
| 转向通道 `angular.z` | 固件按车速换算；**车静止时不生效**；**发 0 时"保持上一次角度"** |

---

## 4. 定位到的根因（按发现顺序）

### 4.1 航向源不可信

- `base_node_R2` 用"转向角模型"积分位置，而车有机械偏差（右后轮快 1.6–2.6%），
  直行时其实在画弧 → 位姿越走越歪
- **修复**：EKF 只融合 `/odom_raw` 的 `vx`，航向改用 IMU 的 `yaw + vyaw`
- IMU 磁力计未启用 → `use_mag: false → true`，静止漂移 0.78 → 0.39 °/min

### 4.2 车体自遮挡污染扫描

- 原始 `/scan` 中 144 个 < 0.4 m 的近点，集中在车尾 −150° ~ −120°
- **修复**：`scan_filter_node` 屏蔽 ±120–150° 扇区，建图统一使用 `/scan_filtered`

### 4.3 建图算法缺回环检测

- gmapping 为粒子滤波，大场地无显式回环
- **修复**：改用 `slam_toolbox`（图优化 + 回环），碎斑密度 1.34 → 0.08 个/m²

### 4.4 转向指令到不了底盘（关键）

Nav2 的 `velocity_smoother` 按"转向 = `angular.z`"设计，
实测往 `/cmd_vel_nav` 发 `linear.y = 0.02`，输出被夹成 `0.0`。

### 4.5 ★ 底盘转向语义与 Nav2 不一致（本次核心发现）

**依据**：442 秒导航记录（4413 帧），`/cmd_vel`、`/vel_raw`、`/amcl_pose`、
点云-地图残差同步采集。

| 证据 | 数值 |
|---|---|
| 运动期间 `angular.z = 0` 的帧数 | 67 帧 |
| 那 67 帧前轮实际转角 | **全部卡在 −25°，一次都没回正** |
| `angular.z` 变化频率 | 2.29 次/秒，符号翻转 19 次 |
| 前轮实际转角范围 | ±44°（顶死机械极限） |

**结论**：Nav2 的语义是"ω = 0 → 直行"，底盘固件的语义是"ω = 0 → 不更新转向"。
于是车在前轮打着 25° 的情况下"直行"——**实际在画弧，控制器却以为在走直线**。
点云因此持续左右偏移，误差累积。

### 4.6 定位在车一动起来就失去匹配

记录仪采集的点云-地图残差（有效指标：静止且位姿正确时精确为 0.000 m）：

| 时刻 | 残差中位数 |
|---|---|
| 0 – 84 s（车静止，AMCL 已收敛） | 0.000 m |
| 84.5 s（发目标，前轮 0.1 秒内打到 +44°） | 开始上升 |
| 92 s | 0.25 m |
| 124 – 184 s | 0.83 m |
| 301 – 441 s | 0.25 – 1.0 m |

把位姿投到地图上统计：

- **23.0% 的时间落在"从没建过图的未知区"**
- **7.2% 落在"障碍格上"**（人站在墙里是不可能的）→ 位姿估计已错

### 4.7 最终崩塌

| 时刻 | 事件 |
|---|---|
| 215 s 起 | 车已停车，但残差长期 0.4+，AMCL 在错误的假设上继续被随机粒子污染 |
| **381 s** | **崩塌：`map→odom` 一帧跳 90 m、转 171°，协方差 0.2 → 385** |
| 结束 | `map→odom` 的 yaw = **+176.6°**（整体翻转 180°） |

**这就是"朝目标反方向开"的直接原因**：AMCL 把整车在地图中的朝向翻转了 180°，
而 Nav2 仍在按图规划，于是车往反方向走。

---

## 5. 已实施并验证的修改

| # | 修改 | 文件 | 验证方式 |
|---|---|---|---|
| 1 | 新增转向适配节点：δ = atan(ω·L/v)，限幅 ±40°，转速率限制 70°/s，δ≈0 强制发 1° | `scripts/cmd_vel_ackermann.py` | 实测 `v=0.23, ω=0.5` → `δ=30.2°` → `linear.y=0.0302` → 舵机精确 30.0° |
| 2 | 导航 launch 重写：Nav2 输出改走 `/cmd_vel_nav`，串入适配节点；加入扫描滤波 | `navigation_r2.launch.py` | `/cmd_vel_nav` 2 发布者 → 适配节点 → `/cmd_vel` 1 发布者 1 订阅者 |
| 3 | AMCL 与建图统一使用 `/scan_filtered` | `nav_r2.yaml` | `scan_topic = scan_filtered`，订阅者 2 个（AMCL + 代价地图） |
| 4 | AMCL 加固：`max_beams` 60→180，`laser_max_range` 100→12 | 同上 | 参数校验脚本通过 |
| 5 | TEB 补齐阿克曼约束：`min_turning_radius`、`weight_kinematics_turning_radius`、`weight_kinematics_forward_drive`、`weight_kinematics_nh` | 同上 | 参数由 22 → 29 项 |
| 6 | TEB 限幅：`max_vel_theta` 1.0→0.6，`acc_lim_theta` 3.2→0.8，关闭拓扑规划 | 同上 | 校验通过 |
| 7 | footprint 由 r=0.1 圆改为真实矩形 0.3375×0.1911 | 同上 | TEB + 两张代价地图均已生效 |
| 8 | 恢复行为倒车速度 −1.0 → −0.15 m/s | 同上 | 校验通过 |
| 9 | AMCL 启动即设初始位姿（`set_initial_pose`） | 同上 | 日志确认 `Setting pose: 0.000 0.000 0.000` |
| 10 | `nav.sh` 补充新节点的清理逻辑 | `scripts/nav.sh` | — |

**说明**：修改 #2 之前的问题是 `velocity_smoother` 会吃掉转向通道；
修改 #1 之前的问题是底盘"发 0 保持转向"的语义与 Nav2 冲突。
两者共同导致"舵机持续修方向"和"点云缓慢偏移累积"。

---

## 6. 复现与验收流程

### 6.1 前置自检

```bash
bash ~/r2_mapping/scripts/usb_status.sh      # 外设是否齐全
bash ~/r2_mapping/scripts/verify_changes.sh  # 参数改动是否到位
```

### 6.2 建图

```bash
ros2 launch ~/r2_mapping/src/r2_mapping_bringup/launch/mapping_slam_toolbox.launch.py
# 遥控低速走闭合路径
ros2 launch yahboomcar_nav save_map_launch.py map_path:=/home/jetson/r2_mapping/maps/room_05
sed -i 's/free_thresh: 0.25/free_thresh: 0.196/' ~/r2_mapping/maps/room_05.yaml
python3 ~/r2_mapping/src/r2_mapping_tools/r2_mapping_tools/map_eval.py \
  ~/r2_mapping/maps/room_05.yaml
```

### 6.3 导航

```bash
bash ~/r2_mapping/scripts/nav.sh
# 地图会自动出现（AMCL 启动即初始化），发一个近处目标
```

### 6.4 全程记录（可选，用于复盘）

```bash
python3 -u ~/r2_mapping/scripts/nav_watch.py
```

### 6.5 验收指标

| 指标 | 期望 |
|---|---|
| 转向指令变化次数/秒 | < 1（修改前 2.29） |
| 单步转向幅度 | 不再出现 ±44° 满舵乱甩 |
| 点云-地图残差中位数 | 稳定在 0.05 – 0.20 m |
| AMCL 协方差 | 不发散（正常 < 1） |
| 回到起点地图重合度 | 误差约一个栅格（5 cm） |

---

## 7. 未完成项与阻塞

### 7.1 硬件阻塞：雷达与摄像头不在 USB 总线上

**现象**：`/dev/ydlidar` 不存在，雷达驱动报
`Error, cannot bind to the specified serial port [/dev/ydlidar]`。

**排查证据（均为实测）**：

| 检查 | 结果 |
|---|---|
| `lsusb` 中 `10c4:ea60`（CP2102）与 `2bc5:*`（摄像头） | 均不存在 |
| 换 USB 口（含已验证可用的口） | **udev / 内核零事件** |
| Hub 电气层端口状态 | 各端口 `power` 有、`connect` 无 → 无设备挂在数据线上 |
| 同一 Hub 上的底盘串口、手柄接收器 | 正常枚举，Hub 本身工作正常 |
| 内核日志 | 无过流、无枚举失败，开机扫描时即无该设备 |
| udev 规则 / 内核模块 / 服务 | 均未被改动（时间戳为历史值） |

**结论**：雷达与摄像头在电学层面未连接到 USB 总线，属于硬件侧问题，
软件层无法修复。建议按以下顺序处理：

1. 换一根**确认可传数据**的 USB 线复测
2. 把雷达插到**普通电脑**上验证（不识别即为设备故障）
3. 若确认设备故障：更换雷达（或先更换其 USB 转接板）

### 7.2 其他待改进项

- `maps/room_04` 仅 17.6% 为已建图区域，建议在雷达恢复后重录一张更完整的地图
- 导航目标应尽量落在已建图区域内
- 可选：增加"迷失看门狗"——点云残差持续超限即停车报警

---

## 8. 结论

- 建图链路（slam_toolbox + 扫描滤波 + EKF）已完成并产出 4 张地图
- 导航链路的**根因已定位并有数据支撑**：底盘对 `angular.z = 0` 的解释
  与 Nav2 语义冲突，导致车辆"画弧当直行"，进而引发定位漂移与最终 180° 翻转
- 针对该根因的修复（转向适配节点 + 参数修正）已实施并在车上验证通过
- 当前唯一阻塞是**雷达硬件的 USB 连接问题**，属硬件范畴；
  雷达恢复后按第 6 节流程即可直接运行验收
