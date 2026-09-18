# scripts/ 索引

按用途分成五组。**日常只会用到第一组和第二组**，第三、五组是排查和验证工具。

---

## 组 1：入口与环境（每天都用）

| 脚本 | 用途 | 怎么跑 |
|---|---|---|
| `env.sh` | 环境变量 + source 两个原厂工作区。**其它脚本都 source 它** | `source ~/r2_mapping/scripts/env.sh` |
| `nav.sh` | **导航一键启动**（硬件 + Nav2 + 转向适配 + 定位护卫） | `bash nav.sh` / `bash nav.sh room_04` / `bash nav.sh stop` / `bash nav.sh status` |
| `estop.sh` | 急停：停发布者 → 灌零速 → 读反馈验证 | `bash estop.sh`（`--hard` 连驱动一起杀） |
| `restart_hardware.sh` | 干净重启硬件（杀进程 + 清共享内存 + 停 daemon） | `bash restart_hardware.sh`（`--no-start` 只停不启） |
| `r2.sh` | 全功能菜单（建图/定位/遥控/检查） | `bash r2.sh` |
| `serial_guard.sh` | 串口占用检查 | `bash serial_guard.sh` |
| `check_all.sh` | 综合自检 | `bash check_all.sh` |

---

## 组 2：运行时节点（**launch 会调用，别改名别乱移**）

| 脚本 | 用途 | 谁在用 |
|---|---|---|
| `cmd_vel_ackermann.py` | **转向适配**：ω → 前轮转向角，走 `linear.y` | `navigation_r2.launch.py` |
| `localization_guard.py` | **定位护卫**：协方差太大就停车 + 提示 | `navigation_r2.launch.py` |
| `lost_watchdog.py` | 迷失看门狗（**已从 launch 摘掉**，备用） | 手动跑 |
| `steer_center.py` | 前轮回正（发小非零角度） | 手动跑 |

参数全部在 `config/nav_r2.yaml` 的 `cmd_vel_ackermann:` 段里改。

---

## 组 3：跑车时的记录与诊断

| 脚本 | 用途 |
|---|---|
| `nav_watch.py` | **行车记录仪**（排查第一工具）：8 路信号 + 点云残差 + 实时报警 |
| `tf_audit.py` | TF 体检：谁在发哪条 TF、有没有冲突/跳变 |
| `odom_vs_tf.py` | 判定 AMCL 实际吃的是哪份里程计 |
| `why_no_map.sh` | RViz 里没有地图时排查（最常见：`map` 坐标系不存在） |
| `verify_nav_live.sh` | 导航链路实时校验（话题接对了没） |
| `verify_amcl_tuning.sh` | 确认 AMCL 调参在运行节点上生效 |
| `verify_changes.sh` | 参数改动一键校验 |
| `check_no_dup.sh` | 重复节点检查（重复会抢 `/cmd_vel`） |
| `check_ekf_config.sh` | EKF 配置校验 |
| `ekf_running_params.sh` | 看**运行中**的 EKF 参数（不是文件里的） |
| `scan_frame_check.sh` | `/scan` 的 frame_id 落在 TF 树哪里 |
| `who_pub_tf.sh` | 谁在发 `/tf`、`/odom` |
| `imu_drift.py` | IMU 航向漂移率 |
| `scan_stats.py` | 扫描统计（有效点/近点/角度分布） |
| `js_raw.py` | 手柄原始数据（绕过 ROS，判断是手柄坏还是驱动卡） |
| `measure_cpu.sh` | 某个进程的稳态 CPU/内存 |

### USB 排查四件套

| 脚本 | 用途 |
|---|---|
| `usb_status.sh` | 一键看所有外设在不在（最常用） |
| `usb_deep_check.sh` | 深度检查：udev 规则、Hub 拓扑、授权状态 |
| `usb_timeline.sh` | 从内核日志还原插拔时间线 |
| `usb_watch.sh` | 实时监听 USB 事件（边插边看） |

---

## 组 4：建图与地图

| 脚本 | 用途 |
|---|---|
| `start_mapping.sh` | 建图（硬件 + slam_toolbox） |
| `start_localization.sh` | 定位复测 |
| `save_map.sh` | 存图 |
| `map_eval.sh` | 地图质量评估 |
| `set_initial_pose.sh` | 命令行设初始位姿 |
| `record_bag.sh` | 录 rosbag |
| `pcd_to_pgm.py` | PCD 点云 → PGM 地图（离线处理） |
| `start_odom_experimental.sh` | 实验性的自定义里程计 |

---

## 组 5：验证脚本（验证某个功能/参数，用完可删）

| 脚本 | 验证什么 |
|---|---|
| `test_adapter_isolated.sh` | 转向适配节点的换算/限幅/死区（**完全隔离，车不动**） |
| `test_watchdog_halt.sh` | 看门狗叫停链路 |
| `steer_probe.sh` / `steer_probe2.sh` | 底盘转向通道判定（**需架起车**） |
| `steer_step.sh` | 舵机阶跃响应（能不能达到指令角度） |
| `steer_center_cmd.sh` | 前轮回正（单发指令版） |
| `ack_quick.sh` / `debug_ack.sh` | 转向适配节点快速验证 |
| `bench_watchdog.sh` | 看门狗资源占用测量 |
| `test_smoother.sh` / `test_smoother2.sh` / `fix_smoother.sh` | 验证 Nav2 自带 smoother 会不会吃掉 `linear.y` |

---

## 两个通用约定

**1. 所有脚本第一句都是 source env.sh**

```bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1
```

`env.sh` 负责：`ROS_DOMAIN_ID=28`、source ROS、source 两个原厂工作区。
**不 source 就看不到车上的话题。**

**2. 带引号的 ROS 命令一律写成脚本文件**

从 PowerShell/ssh 直接拼带消息体的 `ros2 topic pub`，引号很容易被吃掉，
命令会**静默失败**（2026-09-16 急停就栽在这上面）。写成 `.sh` 文件再执行。
