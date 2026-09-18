# 工程架构（先看这一份）

> 这份文档回答三个问题：**数据怎么流**、**文件在哪**、**要改什么改哪儿**。
> 想快速上手看 `15_vscode_guide.md`，想查命令看 `12_command_cheatsheet.md`。

---

## 1. 一分钟看懂整个系统

```
【硬件层】                                    【ROS 节点】                        【话题】
  STM32 底盘 ──串口(/dev/myserial)──→ Ackman_driver_R2 ──→ /vel_raw
                                                          ──→ /imu/data_raw ──→ imu_filter_madgwick ──→ /imu/data
  YDLIDAR 雷达 ──串口(/dev/ydlidar)──→ ydlidar_ros2_driver ──→ /scan
                                                                    │
                                                          scan_filter_node
                                                                    ↓
                                                            /scan_filtered

【定位层】
  /odom_raw（原厂里程计，航向不可信）
  /imu/data（航向）
        └──→ ekf_filter_node（robot_localization）──→ /odom  +  发布 odom→base_footprint
                                                          │
  /scan_filtered + /map（已知地图）                       │
        └──→ amcl（Nav2）──────────────────────────────→ 发布 map→odom

【规划控制层】
  planner_server（全局路径）──→ controller_server（TEB，局部轨迹）
                                      │
                                      发布 /cmd_vel_nav
                                      ↓
                            cmd_vel_ackermann（本项目自写）
                                      │  自己算 δ = atan(ω·L/v)
                                      发布 /cmd_vel（走 linear.y）
                                      ↓
                                底盘驱动 → STM32 → 舵机/电机

【保护层】
  localization_guard：盯 /amcl_pose 协方差 → 太大就通过 /cmd_vel_halt 叫停
```

**一句话**：里程计和 IMU 融合出位姿 → AMCL 把位姿对齐到地图 → Nav2 规划路径 →
TEB 出速度指令 → **本项目自写的适配节点把"角速度"翻译成"前轮转向角"** → 底盘执行。

---

## 2. 为什么需要自写的两个节点

这两个是整个工程里最"非标准"的部分，也是最容易出问题的部分。

### `scripts/cmd_vel_ackermann.py` —— 转向适配

**背景**（2026-09-16 实测）：这台车的底盘有两条转向通道，行为完全不同。

| 通道 | 行为 |
|---|---|
| `linear.y` | 直接给角度，**值×1000 = 度数**。任何车速都立即生效 |
| `angular.z` | 固件自己按车速换算，但有两个坑：<br>① 车静止时**完全不生效**<br>② 行驶中发 0 时会**保持上一次角度**，不回正 |

坑② 是致命的：**Nav2 的语义是"ω=0 → 直行"，底盘理解成"不更新转向"**。
于是车在前轮打着 25° 的情况下"直行"——实际在画弧，控制器却以为在走直线。
点云因此持续偏移、误差累积，最终导致定位迷失。

**本节点的做法**：不依赖固件的换算，自己按阿克曼模型算，走 `linear.y`：

```
δ = atan(ω · L / v)          L = 轴距 0.2681 m
δ 限幅 ±40°（倒车时 ±22°）
再做转速率限制 45°/s + 死区 1.5°
```

### `scripts/localization_guard.py` —— 定位护卫

**背景**：AMCL 在"似然场平坦/多解"的位置（长走廊、相似结构）没有约束，
位姿会滑走甚至瞬移几十米。实测最严重的一次：map→odom 跳 90 米、航向翻转 177°，
表现为"车朝目标反方向开"。

**本节点的做法**：只订阅 `/amcl_pose`（**不用地图、不用激光，代价接近零**）：

```
协方差 > 20 持续 2 秒  →  通过 /cmd_vel_halt 停车 + 提示重新给位姿
协方差 < 1.0 持续 5 秒  →  自动解除
```

---

## 3. 目录结构与职责

```
r2_mapping/
├── README.md                 工程总览（入口）
├── config/                   各种 YAML 参数（含原厂 vs 补丁版对照）
│   ├── nav_r2.yaml           ★ 导航全套参数（改参数主要看这个）
│   ├── ekf_x1_x3_patched.yaml    EKF 补丁
│   ├── r2_vehicle.yaml       车辆实测几何（轴距/轮距/最大转角…）
│   └── lidar_extrinsics.yaml 雷达外参（待标定）
├── docs/                     文档（00~15）
├── maps/                     生成的地图（room_01 ~ room_05）
├── bags/                     录制的 rosbag
├── logs/                     记录仪 CSV + 诊断图
├── scripts/                  ★ 运维入口 + 运行时节点 + 诊断脚本
├── tools/analysis/           纯电脑端分析工具（不依赖 ROS）
└── src/                      ROS2 包
    ├── r2_mapping_bringup/   launch + rviz 配置
    ├── r2_mapping_perception/scan_filter_node
    ├── r2_mapping_tools/     地图评估、里程计标定、手柄遥控等
    └── r2_mapping_msgs/      自定义消息（未使用）
```

### 关键文件速查

| 我想…… | 打开这个 |
|---|---|
| 改转向限幅 / 死区 / 倒车限幅 | `config/nav_r2.yaml` → `cmd_vel_ackermann:` 段 |
| 改 TEB 控制器参数 | `config/nav_r2.yaml` → `controller_server.FollowPath` |
| 改 AMCL 定位参数 | `config/nav_r2.yaml` → `amcl:` 段 |
| 改代价地图 / 车体尺寸 | `config/nav_r2.yaml` → `local_costmap` / `global_costmap` |
| 改导航启动流程 | `src/r2_mapping_bringup/launch/navigation_r2.launch.py` |
| 改建图参数 | `src/r2_mapping_bringup/config/slam_toolbox_mapping.yaml` |
| 改转向换算逻辑 | `scripts/cmd_vel_ackermann.py` |
| 改迷失保护策略 | `scripts/localization_guard.py` |
| 改一键启动脚本 | `scripts/nav.sh` |

---

## 4. 三个使用场景的完整流程

### 场景 A：建图

```
① 停掉导航（避免抢 /cmd_vel）
   bash ~/r2_mapping/scripts/nav.sh stop

② 起建图（硬件 + 扫描滤波 + slam_toolbox）
   ros2 launch ~/r2_mapping/src/r2_mapping_bringup/launch/mapping_slam_toolbox.launch.py

③ 另一个终端起手柄遥控
   python3 ~/r2_mapping/src/r2_mapping_tools/r2_mapping_tools/ps2_teleop.py
   （解锁键 = 按键 11；默认上锁，不解锁不发任何指令）

④ 手柄开一圈，慢速、贴墙、最后回到起点闭合

⑤ 存图 + 评估
   ros2 launch yahboomcar_nav save_map_launch.py map_path:=~/r2_mapping/maps/room_06
   sed -i 's/free_thresh: 0.25/free_thresh: 0.196/' ~/r2_mapping/maps/room_06.yaml
   python3 ~/r2_mapping/src/r2_mapping_tools/r2_mapping_tools/map_eval.py ~/r2_mapping/maps/room_06.yaml
```

### 场景 B：导航

```
① 一键启动（硬件 + Nav2 + 转向适配 + 定位护卫）
   bash ~/r2_mapping/scripts/nav.sh
   （默认用 maps/room_05.yaml；换图：bash nav.sh room_04）

② 另一个终端：记录仪（强烈建议）
   python3 -u ~/r2_mapping/scripts/nav_watch.py

③ 另一个终端：RViz
   rviz2 -d ~/r2_mapping/src/r2_mapping_bringup/rviz/rviz_localization.rviz

④ RViz 里发 2D Goal Pose（地图会自动出现，不用先点 2D Pose Estimate）

⑤ 收工：Ctrl-C 各终端，或
   bash ~/r2_mapping/scripts/nav.sh stop
```

### 场景 C：出问题了

```
急停           bash ~/r2_mapping/scripts/estop.sh
（真出事）      直接拍电源开关 —— 软件急停只能保证"不再发新指令"

看链路是否正常  bash ~/r2_mapping/scripts/verify_nav_live.sh
看有没有重复节点 bash ~/r2_mapping/scripts/check_no_dup.sh
看外设在不在    bash ~/r2_mapping/scripts/usb_status.sh
看参数对不对    bash ~/r2_mapping/scripts/verify_amcl_tuning.sh
干净重启硬件    bash ~/r2_mapping/scripts/restart_hardware.sh
```

---

## 5. 排查问题的思路（按数据流找）

定位问题永远按这条链**从后往前**查：

```
地图 → 定位(AMCL) → 里程计(EKF) → 传感器 → 硬件
```

| 症状 | 先怀疑 | 用哪个工具查 |
|---|---|---|
| 点云与地图对不上 | 定位 / 地图 | `nav_watch.py`（看"点云残差"那列） |
| 位姿瞬移几十米 | AMCL 随机粒子跳走 | 同上（看 `map→odom` 跳变） |
| 车朝反方向走 | 定位已崩但你还发目标 | `localization_guard` 的终端报警 |
| 舵机左右来回打 | 控制器抖动 | `nav_watch.py`（看"符号翻转"次数） |
| 车不动 | 底盘驱动 / 串口 | `usb_status.sh`、`fuser -v /dev/myserial` |
| 没有 /scan | 雷达 USB | `usb_status.sh` |
| 没有 /odom | EKF / 原厂里程计 | `ros2 topic hz /odom` |
| RViz 一片空白 | `map` 坐标系不存在 | `why_no_map.sh` |

---

## 6. 已知的坑（踩过的，别再踩）

| 坑 | 表现 | 结论 |
|---|---|---|
| 底盘 `angular.z=0` 不回正 | 车"画弧"却以为在直行 | 必须走 `linear.y` 通道 |
| Nav2 自带 `velocity_smoother` | 把 `linear.y`（转向）丢掉 | **不要启用它** |
| 车速越低转向角越被放大 | δ=atan(ωL/\|v\|) 在低速时爆掉 | 倒车单独限幅 ±22° |
| **似然场"平坦"** | 长走廊里位姿自己滑 | ①调大 `update_min_d` ②加 `max_beams` |
| AMCL 只在收到激光时才发 TF | 不设初始位姿 → RViz 全黑 | 已设 `set_initial_pose: true` |
| `nav.sh stop` 杀不掉 launch 父进程 | 攒出两个 controller_server 抢指令 | 文件名是 `navigation_r2.launch.py`（点号），模式别写错 |
| PowerShell 吃引号 | 带消息体的 `ros2 topic pub` 静默失败 | **含引号的命令一律写成脚本文件再执行** |
| `pkill -f` 自匹配 | 杀命令把自己也杀了 | 模式里加方括号，如 `lost_[w]atch` |

---

## 7. 待办 / 未完成

| 项 | 状态 |
|---|---|
| 雷达外参标定 | 未标定（`config/lidar_extrinsics.yaml` 里还是 null） |
| 里程计尺度标定 | 有工具 `odom_calibrate.py`，未正式标 |
| 地图覆盖率 | room_05 仅 18.5%（bbox 内），主要因为没进去的房间 |
| 长走廊定位稳定性 | 已调 `update_min_*` + `max_beams`，**待实车验证** |
| `r2_mapping` 的 colcon 编译 | 目前 launch 全部用**文件绝对路径**，未 build |
