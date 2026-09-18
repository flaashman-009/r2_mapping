# 建图流程（标准作业顺序）

顺序不能跳。每一步都有明确的"通过"信号，不通过就不要进入下一步。

---

## 步骤 0 — 上车准备

```bash
# 确认底盘串口空着
./scripts/serial_guard.sh

# 确认磁盘
df -h /
```

通过信号：`/dev/myserial` 存在，且没有驱动进程占用它；根分区剩余 > 2 GB。

---

## 步骤 1 — 启动底盘 + LiDAR + base_node

```bash
./scripts/start_mapping.sh --rviz false
```

或者分步，先只起硬件：

```bash
ros2 launch r2_mapping_bringup sensors.launch.py
```

通过信号：日志里依次出现 LiDAR 打开成功、串口打开成功、
`base_node_R2` 开始发布 `/odom_raw`。

---

## 步骤 2 — 确认四个基础话题与 TF

```bash
./scripts/check_all.sh
```

等价于手工检查：

```bash
ros2 topic hz /scan
ros2 topic hz /odom_raw
ros2 topic hz /imu/data_raw
ros2 run tf2_ros tf2_echo odom base_link
```

通过信号（对应 Gate A）：

- `/scan` 8–12 Hz；`/odom_raw` ≈10 Hz；`/imu/data_raw` ≈10 Hz。
- 时间戳互差 < 50 ms。
- `odom→base_link→laser` 可查。

---

## 步骤 3 — RViz 可视化确认

```bash
ros2 launch r2_mapping_bringup mapping.launch.py rviz:=true
```

RViz 里必须有（配置见 `src/r2_mapping_bringup/rviz/rviz_mapping.rviz`）：

| 显示项 | 话题 | 期望 |
|---|---|---|
| LaserScan | `/scan` | 墙是直线，不随车"抖动" |
| TF | — | 坐标系齐，无红色断裂 |
| Odometry | `/odom_raw` | 箭头随车移动，前进时指向前方 |
| RobotModel | `/robot_description` | 车模与 LiDAR 位置对得上 |

手动推行 1 m，RViz 里车应该移动约 1 m。偏差大先做里程计标定：

```bash
ros2 run r2_mapping_tools odom_calibrate --ros-args -p target_m:=1.0 -p speed:=0.3
```

---

## 步骤 4 — 启动 slam_toolbox 建图

```bash
ros2 launch r2_mapping_bringup mapping.launch.py \
  start_vendor:=false          # 硬件已经在跑时
```

用的是 `async_slam_toolbox_node` + 本项目参数
`src/r2_mapping_bringup/config/slam_toolbox_mapping.yaml`。

通过信号：

```bash
ros2 topic hz /map        # 应该能看到 ~0.5 Hz 的更新
ros2 topic echo /map --once | head -20
```

RViz 中 Map 图层出现，unknown 区域随行进变成 free / occupied。

### 关键参数说明

| 参数 | 值 | 说明 |
|---|---|---|
| `resolution` | 0.05 | 与验收标准一致，改了两边都要改 |
| `max_laser_range` | 12.0 | 室内有效距离；TG30 标称更远，但室内拉满会引入噪声 |
| `minimum_travel_distance` | 0.2 | 走 0.2 m 才处理一帧，阿克曼低速下够用 |
| `minimum_travel_heading` | 0.2 rad | 转向约 11.5° 才处理一帧 |
| `minimum_time_interval` | 0.2 | 最快 5 Hz 处理，10 Hz 激光不会压垮 CPU |
| `do_loop_closing` | true | 闭环靠它 |
| `base_frame` | `base_footprint` | 若车上只有 `base_link`，改这里 |

---

## 步骤 5 — 遥控低速走闭合路径

安全前提：人手放在遥控器/急停上，场地内无人。

规则：

1. **速度 0.2–0.4 m/s**，转弯前减速到 0.2 m/s 以下。
2. 路线尽量**闭合**：绕一圈回到起点，且回到起点时朝向与出发一致。
3. 转弯要**平缓**，阿克曼车原地打方向只会让 scan matching 失配。
4. 走廊里走**中线**，不要贴墙；贴墙会让 LiDAR 丢一半视野。
5. 同一个地方**不要反复来回擦**，容易出现重影。
6. 感觉到 RViz 中地图跳动 → 停下，等 SLAM 稳定，再继续。

建议路线：

```
出发 → 沿走廊直行 → 左转 → 绕房间一圈 → 回到走廊 → 回到出发点（朝向一致）
```

边走边观察：`/map` 更新正常、RViz 中激光与已建墙线重合。

---

## 步骤 6 — 保存地图

回到起点、朝向一致后，先**停稳 3 s** 让 SLAM 收敛，再保存：

```bash
./scripts/save_map.sh room_01
```

产物：

```
maps/room_01.pgm
maps/room_01.yaml
maps/room_01_posegraph.posegraph     # slam_toolbox 序列化结果，便于以后续建
maps/room_01_posegraph.data
```

立即评估：

```bash
./scripts/map_eval.sh maps/room_01.yaml
```

检查 occupied / free / unknown 比例与墙线厚度，异常就重走一遍。

---

## 步骤 7 — 用地图启动 AMCL 定位复测

先停掉建图（避免两个 `map→odom` 发布者）：

```bash
# Ctrl-C 结束 mapping.launch.py，或
pkill -f async_slam_toolbox_node
```

然后：

```bash
./scripts/start_localization.sh maps/room_01.yaml
```

设置初始位姿（把车摆到建图时的起点，朝向一致）：

```bash
./scripts/set_initial_pose.sh 0.0 0.0 0.0
```

验证：

```bash
./scripts/check_all.sh --expect-map
ros2 topic echo /amcl_pose --once
```

通过信号：

- AMCL 在 10 s 内收敛（协方差变小）。
- RViz 中 `/scan` 与地图墙线重合。
- 手动慢推小车移动 1–2 m，`/amcl_pose` 跟随且回到原位置后不漂。

---

## 步骤 8 — 留档

```bash
./scripts/record_bag.sh room_01_run1      # 建图过程必需
```

并在 `docs/06_test_log.md` 追加一条记录：日期、地图名、参数、
自检结果、问题现象。

---

## 离线补充：PCD → PGM

如果需要把点云数据离线转成地图（例如相机/深度点云，或事后重投影）：

```bash
python3 scripts/pcd_to_pgm.py input.pcd maps/room_01_offline \
  --resolution 0.05 \
  --z-min -0.1 --z-max 1.5 \
  --voxel-size 0.03 \
  --outlier-radius 0.08 --min-neighbors 3 \
  --inflate-cells 2
```

处理链：时间戳筛选 → Z 高度过滤 → 体素降采样 → 离群点过滤 →
2D 投影 → 障碍膨胀 → PGM/YAML 输出，并打印各阶段点数与耗时。
