# r2_mapping — Yahboom R2 ROS2 2D 建图

在 Yahboom ROSMaster R2（阿克曼）小车上完成 **ROS2 Humble 2D 建图 + 定位复测**。

这个仓库存在的意义，是把「建图」这件事从散落的脚本收敛成一条可复现的链路：

```
传感器稳定 -> 时间戳同步 -> TF 正确 -> 里程计可靠 -> SLAM 建图 -> 保存地图 -> AMCL 复测
```

`slam_toolbox` 只是链路的第 5 步。前四项不稳，地图一定漂。

## 0. 从哪开始看

| 你想…… | 看这个 |
|---|---|
| **把项目交给另一个 AI 接手** | [`docs/17_ai_handoff.md`](docs/17_ai_handoff.md) |
| **一分钟搞懂整个系统怎么跑** | [`docs/14_architecture.md`](docs/14_architecture.md) |
| **在 VS Code 里看代码** | [`docs/15_vscode_guide.md`](docs/15_vscode_guide.md) |
| 查命令 | [`docs/12_command_cheatsheet.md`](docs/12_command_cheatsheet.md) |
| 看改过什么、为什么改 | [`docs/10_before_after.md`](docs/10_before_after.md) |
| 交接 / 交付 | [`docs/13_delivery_summary.md`](docs/13_delivery_summary.md) |
| 找某个脚本 | [`scripts/README.md`](scripts/README.md)（按用途分组） |
| 分析记录数据 | [`tools/analysis/README.md`](tools/analysis/README.md) |

### 最常用的三条命令

```bash
bash ~/r2_mapping/scripts/nav.sh          # 导航一键启动（含硬件 + Nav2 + 保护）
bash ~/r2_mapping/scripts/estop.sh        # 急停
bash ~/r2_mapping/scripts/nav.sh stop     # 停止
```

> 真出事请**直接拍电源开关**——软件急停只能保证"不再发新指令"。

## 1. 关键约束（先读这段）

1. 建图使用**原厂 Ackermann 模式**，不使用软件差速模式 —— 差速模式没有完整 odom/TF。
2. `/dev/myserial`（CH340 底盘串口）**同一时间只能被一个进程占用**。
   冲突进程清单见 `docs/05_troubleshooting.md`。
3. `/odom_raw` 的 yaw 不可信（轮速死推算）。实测结论：EKF 若融合 `/odom_raw` 的 yaw，
   会与 LiDAR/AMCL 打架，导致定位漂移、导航任务 ABORTED。航向只用 IMU yaw。
4. 地图必须能存成 **PGM + YAML**，并能被 Nav2 `map_server` / `amcl` 加载。
5. LiDAR 安装外参、里程计标尺、时间同步三项必须先标定再建图。

## 2. 硬件与软件基线

| 项目 | 值 |
|---|---|
| 主控 | Jetson Orin NX SUPER 16GB，Ubuntu 22.04 |
| ROS | ROS2 Humble |
| ROS_DOMAIN_ID | 28 |
| LiDAR | YDLIDAR 4ROS，`/scan`，约 10 Hz，串口 `/dev/ydlidar` |
| 底盘 | CH340 USB 串口 `/dev/myserial`，STM32，**非 CAN** |
| 驱动 | `Ackman_driver_R2` + `r2_driver_monitor`（原厂 Ackermann） |
| 里程计 | `base_node_R2` → `/odom_raw`，约 10 Hz |
| IMU | `/imu/data_raw`、`/imu/yaw_deg` |
| 轴距 L / 轮距 d | 0.2681 m / 0.1646 m（实测） |
| 最大转向角 | ±45°（0.7854 rad） |

细节见 `docs/02_hardware_topics.md`。

## 3. 目录结构

```
r2_mapping/
├── src/
│   ├── r2_mapping_bringup/      启动脚本、launch、参数、RViz 配置
│   ├── r2_mapping_perception/   点云/扫描滤波
│   ├── r2_mapping_tools/        自检、地图评估、里程计标定
│   └── r2_mapping_msgs/         可选自定义消息（健康状态上报）
├── maps/                        生成的 PGM/YAML（含 posegraph）
├── bags/                        录制数据
├── config/                     车辆实测参数、外参占位配置
├── scripts/                    一键脚本 + PCD→PGM 工具
└── docs/                       建图流程、验收标准、问题记录
```

文档索引：

| 文件 | 内容 |
|---|---|
| `docs/00_project_charter.md` | 项目目标、约束、范围 |
| `docs/01_acceptance_criteria.md` | 验收标准（Gate A/B） |
| `docs/02_hardware_topics.md` | 硬件、串口、话题基线 |
| `docs/03_tf_and_extrinsics.md` | TF 树与 LiDAR 外参标定 |
| `docs/04_mapping_workflow.md` | 建图标准作业顺序 |
| `docs/05_troubleshooting.md` | 问题排查 |
| `docs/06_test_log.md` | 上车记录 |
| `docs/07_ps2_teleop.md` | **PS2 手柄遥控操作手册** |
| `docs/08_map_drift_analysis.md` | **地图漂移根因分析与修复记录** |

## 4. 编译

在 Jetson（或任意装有 ROS2 Humble 的 Linux 机器）上：

```bash
cd ~/r2_mapping
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
chmod +x scripts/*.sh        # 从 Windows 传过来时补一次可执行位
```

## 5. 快速开始

```bash
# 0) 确认底盘串口没被别的进程占用
./scripts/serial_guard.sh

# 1) 建图前置条件自检（先跑这一步，别急着建图）
./scripts/check_all.sh

# 2) 建图：底盘 + LiDAR + base_node + slam_toolbox + RViz
./scripts/start_mapping.sh

# 3) 遥控低速走闭合路径，回到起点后保存地图
./scripts/save_map.sh room_01

# 4) 用保存的地图做定位复测
./scripts/start_localization.sh maps/room_01.yaml
./scripts/check_all.sh --expect-map
```

## 6. 验收标准

完整版见 `docs/01_acceptance_criteria.md`，摘要：

| 维度 | 通过标准 |
|---|---|
| `/scan` | 稳定 8–12 Hz，无断流，量程内无成片 NaN |
| `/odom_raw` | 约 10 Hz，连续无跳变，前进时 x 单调递增，方向符号正确 |
| `/imu/data_raw` | 约 10 Hz，静止时角速度接近 0，yaw 无 ±180° 跳变 |
| `/imu/yaw_deg` | 稳定，静止漂移小，跨 ±180° 已 unwrap |
| TF | `map→odom→base_footprint→base_link→laser` 完整可查 |
| 时间 | LiDAR/odom/IMU 时间戳互差 < 50 ms（警告线），< 200 ms 视为失败 |
| 外参 | LiDAR 相对 `base_link` 的 x/y/z/yaw 已标定并写入 URDF/配置 |
| 地图 | 分辨率 0.05 m，occupied/free/unknown 比例合理，墙线厚度 1–2 格 |
| 闭环 | 回到起点时地图重合，重影 < 2 格（0.1 m） |
| 定位复测 | AMCL 在保存地图上收敛，`/amcl_pose` 与实车位置一致 |

## 7. 已知风险

- 底盘控制板 USB 曾有硬件枚举失败（`/dev/myserial` 不存在）的历史，
  见 `docs/05_troubleshooting.md`。
- 建图时若同时跑软件差速驱动 `r2_diff_driver`，会抢占 `/dev/myserial`，必须二选一。
- Jetson 磁盘曾接近 93% 占用，录 bag 前先 `df -h`。
