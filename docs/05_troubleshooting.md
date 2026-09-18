# 问题排查

## 1. `/dev/myserial` 打不开 / 串口被占用

**这是本项目最高频的阻塞点。**

### 1.1 确认设备存在

```bash
ls -l /dev/myserial
ls -l /dev/serial/by-id/
udevadm info -q property -n /dev/myserial | grep -E 'DEVPATH|ID_VENDOR_ID|ID_MODEL_ID'
```

期望 `ID_VENDOR_ID=1a86`、`ID_MODEL_ID=7523`。

若 `/dev/myserial` 不存在：

- 检查 udev 规则 `7523 → myserial`、`7522 → myspeech` 是否还在。
- `dmesg | tail -50` 看有没有 `Device not responding to setup address` /
  `device not accepting address ... error -71`。
- 换 USB 线、直插 Jetson 而不要经过 Hub。

历史结论（2026-08-29）：底盘板 `1a86:7523` 在 Windows 上也枚举不到，
倾向硬件（USB 芯片/线/供电）问题，不是 Jetson 或 ROS 软件问题。

### 1.2 确认没有别的进程占用

以下进程**都会打开同一个串口**，同一时间只允许一个：

```text
Ackman_driver_R2                        原厂 Ackermann 驱动
r2_driver_monitor.py                    底盘状态监控
r2_center_steer.py                      转向居中脚本
Rosmaster/rosmaster/rosmaster_main.py   原厂 Rosmaster
Rosmaster/rosmaster/wifi_rosmaster.py   原厂 WiFi 桥
r2_diff_driver（r2_diff_driver_node）    软件差速驱动（本项目禁用）
```

检查命令：

```bash
./scripts/serial_guard.sh
# 或
fuser -v /dev/myserial
pgrep -af 'Ackman_driver_R2|r2_driver_monitor|rosmaster|r2_diff_driver'
```

报错特征：

```text
SerialException: device reports readiness to read but returned no data
(device disconnected or multiple access on port?)
--- Rosmaster Serial Opened! Baudrate=115200
--- set_car_motion error! ---
```

处理：杀掉多余进程，**只保留一个**底盘驱动。

### 1.3 顺序问题

驱动进程必须在串口就绪后启动。若先起驱动再插 USB，驱动不会自动重连，
需要重启驱动节点。

---

## 2. `/scan` 没有数据或频率不对

```bash
ros2 topic hz /scan
ros2 topic info /scan --verbose
ls -l /dev/ydlidar
```

| 现象 | 原因 | 处理 |
|---|---|---|
| 完全没数据 | LiDAR 串口没打开 | 检查 `/dev/ydlidar`、波特率 512000 |
| 频率 5 Hz 左右 | USB 带宽/CPU 抢占 | 关掉相机、YOLO、rviz 的其它订阅 |
| 频率 15+ Hz | 参数被改过 | 统一回 10 Hz |
| 数据里成片 `inf` | 场地太空旷，超出量程 | 正常，SLAM 会用有效量程 |
| 数据里成片 `0.0` | 近距离遮挡 / 车体自遮挡 | 开 `scan_filter_node` 屏蔽扇区 |

---

## 3. TF 报错

| 报错 | 原因 | 处理 |
|---|---|---|
| `Could not find transform map → odom` | 没起 SLAM/AMCL | 起对应节点 |
| `Lookup would require extrapolation into the future` | 时间戳超前于 TF 缓存 | 检查时间同步 |
| `Lookup would require extrapolation into the past` | TF 发布频率太低 | 调高 TF 发布频率或 `transform_tolerance` |
| `TF_OLD_DATA` | 时间戳过旧 | 检查节点是否卡死、时钟是否跳变 |
| 同一父子有两个发布者 | URDF + static_transform_publisher 重复 | 只留一个 |

诊断：

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_monitor map laser
```

---

## 4. 时间戳不同步

症状：RViz 中扫描拖影、SLAM 报 extrapolation、地图接缝错位。

检查（本项目自检工具会直接给出互差）：

```bash
./scripts/check_all.sh
```

常见原因：

1. LiDAR / 底盘 / IMU 使用各自独立的时钟源（USB 时间 vs 系统时间）。
2. 某个节点用了 `use_sim_time:=true`，其它用 false。
3. Jetson 上 NTP 校正导致系统时间跳变。

处理顺序：先统一 `use_sim_time` → 再看驱动是否用 `now()` 打时间戳 →
必要时在 bag 里事后对齐。

---

## 5. 建图漂 / 重影 / 闭环不重合

按这个顺序查（**不要直接调 SLAM 参数**）：

1. **里程计**：推行 1 m 看 RViz 是否走 1 m；偏太多先标定
   （`r2_mapping_tools odom_calibrate`）。
2. **外参**：按 `docs/03_tf_and_extrinsics.md` 对墙标定。
3. **时间戳**：`check_all.sh` 看互差。
4. **速度**：降到 0.2 m/s 重走。阿克曼低速转弯 + 高速 = 必定失配。
5. **路线**：转弯过急、贴墙走、原地打方向，都会破坏 scan matching。
6. 以上都过了再调 SLAM 参数：先动 `minimum_travel_distance` /
   `minimum_travel_heading`，再动 `correlation_search_space_*`。

### `/odom_raw` 的 yaw 陷阱

历史实测（2026-08-29）：EKF 融合了 `/odom_raw` 的 yaw（轮速死推算），
与 LiDAR/AMCL 不一致，导致 AMCL 漂移、Nav2 任务 ABORTED。

**规则：航向信 IMU，`/odom_raw` 只信平移。**
若使用 EKF（`robot_localization`），把 odom 的 yaw/vyaw 权重置零：

```yaml
odom0_config: [true, true, false,
               false, false, false,
               true, false, false,
               false, false, false,
               false, false, false]
```

---

## 6. `slam_toolbox` 起不来 / 不更新 `/map`

| 现象 | 原因 |
|---|---|
| 节点起了但 `/map` 一直没有 | `scan_topic` 参数与实际话题不一致 |
| 参数没生效 | **参数文件用错了**：`async_slam_toolbox_node` 必须配 `mapper_params_online_async` 那一套，直接用 `mapper_params_online_sync.yaml` 会静默错配 |
| 一直 warning 说找不到 TF | `base_frame` 写错（`base_link` vs `base_footprint`） |
| `/map` 有但不动 | 车没动，或 `minimum_travel_distance` 太大 |

检查：

```bash
ros2 param dump /slam_toolbox | grep -E 'scan_topic|base_frame|odom_frame|mode'
ros2 topic info /map --verbose
```

---

## 7. AMCL 不收敛 / 定位漂移

1. 确认初始位姿正确：`./scripts/set_initial_pose.sh X Y YAW_DEG`，
   且与建图时的起点朝向一致。
2. 确认地图是**建图时同一环境**，且地图 YAML 的 `origin` 没被改坏。
3. 确认 `amcl.yaml` 中 `set_initial_pose: false`（否则每次启动都覆盖
   你手动设置的位姿）。
4. `transform_tolerance` 从 0.25 起调；太小会在 TF 抖动时丢定位。
5. 还是漂 → 回到第 5 节排查里程计和外参，AMCL 治不了里程计的病。

---

## 8. 磁盘满 / 录 bag 失败

```bash
df -h /
du -sh ~/.ros/log/* | tail -20
rm -rf ~/.ros/log/<旧日期目录>
```

录 bag 前先清 `~/.ros/log`；`bags/` 里的历史数据建议转存后再删。

---

## 9. 地图 unknown 区域"消失"

症状：`map_eval.py` 报 unknown 0%，但明明有大片没走过的区域；
RViz 里地图看起来"到处都是可通行"。

原因：`free_thresh > 0.196`。unknown 格的灰度是 205，
占据概率 `(255-205)/255 = 0.196`，`free_thresh` 一旦大于它，
unknown 就被判成 free。

处理：

1. 地图 YAML 里把 `free_thresh` 改成 `0.196`（`save_map.sh` 已经这么做了）；
2. 用 `map_eval.py` 复核：它会同时打印"按阈值分类"和"按灰度 205"两个比例；
3. AMCL 本身对 unknown/free 不敏感（它主要用 occupied 做似然场），
   但导航栈里 unknown 被当 free 会让 costmap 把没探索的地方当通道，
   属于隐患，建议改掉。

---

## 10. 建图和定位互相打架

症状：RViz 里地图整体跳一下，或 `/amcl_pose` 忽然飞走。

原因：`map → odom` 有两个发布者（`slam_toolbox` 和 `amcl` 同时在跑）。

处理：

```bash
ros2 node list | grep -E 'slam_toolbox|amcl'
```

同一时间只留一个。切换时先 `Ctrl-C` 停掉建图，确认
`ros2 node list` 里没有 `slam_toolbox` 再启动定位。

