# 参数速查 & 代码大纲

给"想自己改参数"用的参考。分两部分：

1. **整个建图栈的结构**——有哪些节点、各干什么、参数在哪
2. **EKF 的可调参数**——哪些能改、改了有什么用

---

## 第一部分：建图栈大纲

### 1.1 四层结构

```
[硬件层] 原厂 yahboomcar_bringup，一般不用动
  Ackman_driver_R2       串口 /dev/myserial 与 STM32 底盘板通信
    订阅 /cmd_vel        下发速度 + 转向角
    发布 /vel_raw        速度 + 实际转角
    发布 /imu/data_raw   原始 IMU
    发布 /imu/mag        磁力计
    发布 /voltage        电池电压
    发布 /joint_states   关节角（给 RViz 显示车模）

  ydlidar_ros2_driver    发布 /scan（10 Hz，2020 束）
  imu_filter_madgwick    /imu/data_raw + /imu/mag → /imu/data
  base_node_R2           /vel_raw → /odom_raw（轮式里程计）
  robot_state_publisher  URDF → base_link 与各部件之间的 TF
  static_transform_pub   base_link → laser（LiDAR 外参）

                        ↓

[融合层]
  ekf_filter_node        /odom_raw + /imu/data → /odom
                         并发布 TF：odom → base_footprint

                        ↓

[SLAM 层] 本项目 r2_mapping
  scan_filter            /scan → /scan_filtered（屏蔽车尾扇区）
  slam_toolbox           /scan_filtered + TF → /map
                         并发布 TF：map → odom

                        ↓

[工具层]
  ps2_teleop             /joy → /cmd_vel（带看门狗、限幅、死区）
  rviz2                  可视化
```

### 1.2 TF 链（谁发布哪一段）

```
map --(slam_toolbox / amcl)--> odom --(EKF)--> base_footprint
                                                   |
                                                   v  (URDF)
                                               base_link
                                                   |
                                                   v  (静态)
                                                 laser
```

| 变换 | 发布者 |
|---|---|
| `map → odom` | 建图时 `slam_toolbox`；定位时 `amcl` |
| `odom → base_footprint` | `ekf_filter_node` |
| `base_footprint → base_link` | `robot_state_publisher`（来自 URDF） |
| `base_link → laser` | `static_transform_publisher`（写死在 launch 里） |

**规则**：`map → odom` 同一时间只能有一个发布者。建图和定位不能同时跑。

### 1.3 每个节点的参数文件在哪

| 节点 | 参数文件 |
|---|---|
| `ekf_filter_node` | `software/library_ws/install/robot_localization/share/robot_localization/params/ekf_x1_x3.yaml` |
| `imu_filter_madgwick` | `yahboomcar_ws/install/yahboomcar_bringup/share/yahboomcar_bringup/param/imu_filter_param.yaml` |
| `base_node_R2` | `yahboomcar_ws/install/yahboomcar_bringup/share/yahboomcar_bringup/launch/yahboomcar_bringup_R2_launch.py`（写在 launch 里） |
| `slam_toolbox` | `r2_mapping/src/r2_mapping_bringup/config/slam_toolbox_mapping.yaml` |
| `scan_filter` | launch 参数 `blind_sectors`，在 `r2_mapping/src/r2_mapping_bringup/launch/mapping_slam_toolbox.launch.py` |
| `ps2_teleop` | 运行时参数，见 `docs/07_ps2_teleop.md` |

> ⚠️ 改 install 目录里的文件能立刻生效，但 **src 目录里有一份同名副本**，
> 两处都改才不会被下一次 build 覆盖回去。

---

## 第二部分：EKF 可调参数

### 2.1 输入与融合配置（最常动的）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `odom0` | `/odom_raw` | 轮式里程计话题 |
| `imu0` | `/imu/data` | IMU 话题 |
| **`odom0_config`** | 只留 `vx` | **融合哪些量**，15 位见 2.2 |
| **`imu0_config`** | `yaw` + `vyaw` | 同上 |
| `odom0_differential` | `false` | true = 把位置差分成速度再用（能消掉累积漂移） |
| `odom0_relative` | `false` | true = 以启动瞬间为原点 |
| `imu0_relative` | `true` | IMU 的 yaw 以启动朝向为零点 |
| `odom0_queue_size` / `imu0_queue_size` | `10` | 订阅队列长度 |

### 2.2 config 数组的 15 位顺序（必须记牢）

```
[  x,     y,     z,
   roll,  pitch, yaw,
   vx,    vy,    vz,
   vroll, vpitch, vyaw,
   ax,    ay,    az  ]

  位置(3)   姿态(3)    线速度(3)    角速度(3)      线加速度(3)
```

**当前配置**：

| 来源 | 融合的量 | 理由 |
|---|---|---|
| `/odom_raw` | `vx` | 编码器直接测的速度，可信 |
| `/imu/data` | `yaw`, `vyaw` | 航向的唯一来源 |

**不该融合的**：

- odom 的 x/y —— 是积分值，且用了错误的航向模型
- odom 的 yaw/vyaw —— 转向角推算的，有系统性偏差
- vy —— 非完整约束车辆不能侧向平移，恒为 0

### 2.3 坐标系

| 参数 | 当前值 | 说明 |
|---|---|---|
| `odom_frame` | `odom` | |
| `base_link_frame` | `base_footprint` | 车体根坐标系 |
| `world_frame` | `odom` | EKF 输出 odom→base_footprint（不是 map→odom） |
| `publish_tf` | `true` | 是否广播 TF |

> `world_frame` 保持 `odom`：EKF 负责短时连续，绝对位置由 SLAM 用 `map→odom` 纠正。

### 2.4 滤波行为

| 参数 | 当前值 | 能改什么 | 什么时候动 |
|---|---|---|---|
| `frequency` | `30.0` | 输出频率 | 一般不动 |
| `sensor_timeout` | `0.1` | 多久没数据就只预测不校正 | 一般不动 |
| `two_d_mode` | `true` | 强制 2D | **保持 true** |
| `transform_time_offset` | `0.0` | TF 时间戳前移 | 出现"预测未来"报错时加 0.02~0.05 |
| `transform_timeout` | `0.0` | 等 TF 的时间 | 一般不动 |
| `reset_on_time_jump` | `true` | 时间倒退时重置 | 不动 |
| `print_diagnostics` | `false` | **打开后能在 /diagnostics 看每个传感器融合情况** | **排查时开** |

### 2.5 离群值拒绝（马氏距离阈值）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `odom0_pose_rejection_threshold` | `20.0` | 位置测量偏离预测太远就丢弃 |
| `odom0_twist_rejection_threshold` | `1.542` | 速度测量同上 |
| `imu0_pose_rejection_threshold` | `20.0` | |
| `imu0_twist_rejection_threshold` | `1.542` | |
| `imu0_linear_acceleration_rejection_threshold` | `10.0` | |

- **调小** = 更严格地丢离群值（测量有毛刺时有用）
- **调大** = 更宽松（TF 抖动导致频繁丢数据时有用）

### 2.6 过程噪声协方差（调参核心）

`process_noise_covariance` 是 15×15 矩阵（当前只填了对角线）。

**直觉**：

```
过程噪声调大  ->  滤波觉得"我的预测不可靠"  ->  更信测量
过程噪声调小  ->  滤波觉得"我的预测很准"    ->  更信模型
```

**当前对角线值**（顺序同 2.2）：

| 量 | 值 | 说明 |
|---|---|---|
| x, y | 0.05 | 位置 |
| z | 0.06 | |
| roll, pitch | 0.03 | |
| **yaw** | **0.06** | 航向：调大 = 更信 IMU 的 yaw |
| vx, vy | 0.025 | |
| vz | 0.04 | |
| vroll, vpitch | 0.01 | |
| **vyaw** | **0.02** | 角速度 |
| ax, ay | 0.01 | |
| az | 0.015 | |

**实用场景**：

- IMU 的 yaw 漂移明显 -> 把 `yaw` 那一项**调小**（更信模型）
- 反过来，模型本身不准、想更依赖 IMU -> **调大**

### 2.7 初始协方差

`initial_estimate_covariance` 当前对角线全是 `1e-9`（极度自信的初值）。

调大某个量 = 该量收敛更快（启动时更快追上真实值）。一般不用动。

### 2.8 控制输入（当前关闭）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `use_control` | `false` | **打开后可以用 /cmd_vel 当控制输入** |
| `control_config` | `[true, false, false, false, false, true]` | vx + vyaw |
| `control_timeout` | `0.2` | 指令有效期 |
| `acceleration_limits` | `[1.3, 0, 0, 0, 0, 3.4]` | 加速度上限 |
| `deceleration_limits` | `[1.3, 0, 0, 0, 0, 4.5]` | 减速度上限 |

**打开 `use_control` 的好处**：传感器数据短暂丢失时，
滤波能靠"我知道刚发了什么指令"撑住预测，不会突然失准。

**代价**：要把 `/cmd_vel` 正确接进来，参数也要匹配实际车辆动力学。

### 2.9 不建议动的

| 参数 | 原因 |
|---|---|
| `debug` / `debug_out_file` | 打开会大幅拖慢节点，只在深挖问题时临时开 |
| `publish_acceleration` | 用不到 |
| `sensor_timeout` | 改动会引发连锁反应 |

---

## 第三部分：调参优先级

想改 EKF 的时候按这个顺序想：

```
1. 我怀疑哪个传感器？            -> 改对应的 *_config
2. 我怀疑融合权重不对？           -> 改 process_noise_covariance 对应项
3. 我怀疑测量有毛刺被采纳了？      -> 调小 *_rejection_threshold
4. 我怀疑 TF 时间对不上？         -> 改 transform_time_offset
5. 我想知道到底哪个传感器出问题？  -> 打开 print_diagnostics
```

**一次只改一个，改完立刻验证。**

---

## 第四部分：验证改动是否生效

```bash
# EKF 参数文件是否改对了
bash ~/r2_mapping/scripts/check_ekf_config.sh

# 运行时实际生效的值（最权威）
ros2 param get /ekf_filter_node odom0_config

# 看 /odom 和 /odom_raw 是否还"完全相同"
ros2 topic echo /odom_raw --once --field pose.pose.position
ros2 topic echo /odom     --once --field pose.pose.position
```

**注意**：改完参数文件**必须重启硬件**才生效：

```bash
bash ~/r2_mapping/scripts/restart_hardware.sh
```

