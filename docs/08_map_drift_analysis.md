# 地图漂移（"飘"）根因分析与修复记录

日期：2026-09-14
问题：之前用 gmapping 建的图（`room_01`）整体被"拧"，走完一圈起点终点对不上。

本文记录完整的排查过程和实测数据，不是推测。

---

## 1. 症状与初步怀疑

症状：地图局部能对上，但整体沿轨迹方向被剪切/旋转，越到后面越歪。

最初的怀疑（**后来被数据推翻**）：

> 轮式里程计有约 1.7°/s 的假偏航——落地直行时右后轮比左后轮快 1.6%–2.6%，
> 差速模型 `ω = (v_r − v_l) / d` 会把它算成持续转弯。

推算：`0.016 × 0.3 / 0.1646 = 0.029 rad/s ≈ 1.7°/s`。

---

## 2. 第一步：查 EKF 到底融合了什么

文件：`~/yahboomcar_ros2_ws/software/library_ws/install/robot_localization/
share/robot_localization/params/ekf_x1_x3.yaml`

```yaml
odom0: /odom_raw
odom0_config: [true,  true,  false,     # x, y, z
               false, false, false,     # roll, pitch, yaw   ← yaw 已经是 false
               true,  true,  false,     # vx, vy, vz
               false, false, false,     # ... vyaw 也是 false
               false, false, false]

imu0: /imu/data
imu0_config:  [false, false, false,
               false, false, true,      # yaw ← 来自 IMU
               false, false, false,
               false, false, true,      # vyaw ← 来自 IMU
               false, false, false]
```

**结论：原厂配置本来就正确地关掉了 odom 的 yaw/vyaw。**
`/odom` 的航向来自 IMU，不来自轮式里程计。

→ **"里程计假偏航"这条假设不成立**，排查方向转向 IMU。

> 顺带发现一个非标准项：`odom0_config` 里 `y: true` 和 `vy: true`。
> 对非完整约束的阿克曼车，横向位置/速度恒为 0，让 EKF 去信它没有好处，
> 理论上应该改成 `false`。属于待优化项，不是本次主因。

---

## 3. 第二步：查到真凶

文件：`~/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_bringup/
share/yahboomcar_bringup/param/imu_filter_param.yaml`

```yaml
imu_filter_madgwick:
    ros__parameters:
        fixed_frame: "base_link"
        use_mag: false        # ← 磁力计被关掉了
        publish_tf: false
        world_frame: "enu"
        orientation_stddev: 0.1
```

**Madgwick 不用磁力计时，yaw 没有绝对参考**：roll/pitch 靠重力（加速度计）
能稳住，但 yaw 只能靠陀螺仪积分，陀螺零偏会持续累积。

链路：

```
陀螺仪积分漂移
  → /imu/data 的 yaw（madgwick 输出）
    → /odom 的 yaw（EKF 的 imu0 输入）
      → gmapping 的运动先验
        → 地图越来越歪
```

---

## 4. 第三步：量化测量（A/B 对比）

工具：`scripts/imu_drift.py`（本次新写）
方法：车静止、水平放置，采样 180 秒，对解卷绕后的 yaw 序列做最小二乘拟合。

### 4.1 基线（原厂配置 `use_mag: false`）

```
来源         帧数     起止变化      漂移率        判断
odom        1794     +2.18°      +0.78°/min    偏大
imu_data    1794     +2.20°      +0.78°/min    偏大
```

- 静止 180 秒，航向自己转了 **2.18°**
- `/odom` 与 `/imu/data` 的漂移率**完全一致** → 证实 `/odom` 的 yaw 就是照搬 madgwick
- 外推：**建图 20 分钟累计偏 15.6°**。若场地 10 m 宽，末端错位
  `10 × sin(15.6°) ≈ 2.7 m` —— 足以解释肉眼可见的"地图被拧"

### 4.2 打开磁力计（`use_mag: true`）

```
来源         帧数     起止变化      漂移率        判断
odom        1801     +0.29°      +0.39°/min    良好
imu_data    1800     +0.46°      +0.39°/min    良好
```

| 配置 | 漂移率 | 180 s 累计 | 20 min 外推 |
|---|---|---|---|
| `use_mag: false` | +0.78°/min | +2.18° | 15.6° |
| **`use_mag: true`** | **+0.39°/min** | **+0.29°** | **7.8°** |

**漂移率降一半，绝对累计量降 7.5 倍。**

> ⚠️ 注意这个测试是**静止**状态下做的，电机没转。实车行驶时电机、电流
> 产生的磁干扰更强，实际效果需要**行驶中复测**。如果发现路测反而更差，
> 把 `use_mag` 改回 `false` 即可（原文件已备份为 `.bak`）。

---

## 5. 已应用的修改

| # | 文件 | 改动 | 状态 |
|---|---|---|---|
| 1 | `imu_filter_param.yaml`（install + src 两处） | `use_mag: false` → `true` | ✅ 已应用，已备份 |
| 2 | `config/slam_gmapping_r2.yaml`（新文件） | 关键帧加密、量程放开、**`str`/`stt` 调大** | ✅ 已应用 |
| 3 | `src/r2_mapping_bringup/launch/mapping_gmapping.launch.py`（新文件） | 硬件 + 扫描滤波 + 调参 gmapping | ✅ 已验证可跑 |
| 4 | `scripts/restart_hardware.sh`（新文件） | 干净重启，清 fastrtps 共享内存 | ✅ 已验证 |
| 5 | `scripts/imu_drift.py`（新文件） | 漂移率测量工具 | ✅ |
| 6 | `scripts/scan_stats.py`（新文件） | 车体自遮挡诊断工具 | ✅ |

### gmapping 参数改动明细

| 参数 | 原厂 | 改成 | 原因 |
|---|---|---|---|
| `linearUpdate` | 1.0 m | 0.3 | 走 1 米才处理一帧，中间转弯就丢位姿 |
| `angularUpdate` | 0.5 rad | 0.2 | 28.6° 才更新，转弯处特征不够 |
| `temporalUpdate` | 1.0 s | 0.5 | 静止时也保持更新 |
| `maxUrange` | 4.0 m | 12.0 | 只用 4 m 内特征，大房间不够 |
| `maxRange` | 6.0 m | 12.0 | 同上 |
| `particles` | 30 | 60 | 粒子少收敛慢 |
| `iterations` | 5 | 10 | 匹配迭代更充分 |
| **`str`** | 0.1 | **0.35** | 平移→旋转噪声。调大 = "直行时别信里程计的旋转" |
| `stt` | 0.2 | 0.4 | 旋转→旋转噪声 |
| `map_update_interval` | 5.0 s | 2.0 | 出图快，便于实时观察 |
| 地图范围 | ±10 m | ±30 m | 原厂 19.2 m 上限 |

---

## 6. 车体自遮挡（独立于漂移的第二个问题）

雷达能看到自己的车身：**车尾左右两个角**在 0.15–0.35 m 处产生固定回波，
落在 `120°~150°` 和 `-150°~-120°` 两个扇区（与几何推算的 ±147° 吻合）。

原始扫描实测：近点 (<0.4 m) 108 个，占有效点 7.9%，其中

```
-150° ~ -120° :   60
 120° ~  150° :   34
```

**危害**：这些点会被写进地图形成"鬼影"，并让代价地图在车周围标出假障碍。
2026-09-11 导航中止的日志链就是它造成的：

```
[planner_server]  GridBased: failed to create plan      ← 车自己站的位置被判为致命格
[controller_server] TebLocalPlannerROS: trajectory is not feasible
[behavior_server] Collision Ahead - Exiting DriveOnHeading  ← 倒车时车后也有假障碍
[bt_navigator]    Goal failed
```

**处理**：`mapping_gmapping.launch.py` 启动 `scan_filter_node`，屏蔽上述扇区。

验证结果：

| | `/scan` | `/scan_filtered` |
|---|---|---|
| 近点 (<0.4 m) | 108 个 (7.9%) | **19 个 (1.6%)** |
| `-150°~-120°` | 60 | **0** |
| `120°~150°` | 34 | **0** |
| `90°~120°`（真实障碍） | 13 | 13（保留）|

---

## 7. 后续待办

| 项 | 说明 | 前置条件 |
|---|---|---|
| **里程计尺度标定** | 实测直行 3 m 实际只走 2.81 m（差 6.3%）。`base_node_R2` 的 `linear_scale_x/y` 默认都是 1.0，从未标定 | 需要人看车，用 `odom_calibrate.py` |
| **行驶中复测漂移** | 电机转起来后磁干扰更强，需确认 `use_mag: true` 在路上仍然更好 | 需要人看车 |
| EKF 的 `odom0_config` 关闭 `y`/`vy` | 非完整约束车辆不应融合横向位置/速度 | 随时可改 |
| 重跑一次建图验证 | 用新参数 + 滤波，对比地图质量 | 需要人看车 |

---

## 8. 复现命令

```bash
source ~/r2_mapping/scripts/env.sh

# 干净重启硬件
bash ~/r2_mapping/scripts/restart_hardware.sh

# 测静止漂移率
python3 ~/r2_mapping/scripts/imu_drift.py --duration 180

# 看雷达有没有扫到自己
python3 ~/r2_mapping/scripts/scan_stats.py --near 0.4

# 建图（硬件 + 滤波 + 调参 gmapping）
ros2 launch ~/r2_mapping/src/r2_mapping_bringup/launch/mapping_gmapping.launch.py
```

---

## 9. 修复效果验证（room_02，2026-09-14）

用新配置（`use_mag: true` + 调参 gmapping + 车体自遮挡滤波）重跑了一次建图。

### 9.1 两次建图对比

| 指标 | room_01（旧配置） | room_02（新配置） | 判断 |
|---|---|---|---|
| 已探索 bbox | 9.0 × 13.7 m | **28.3 × 33.4 m** | 走了长 7.7 倍的路 |
| 已探索面积 | 123 m² | **945 m²** | |
| bbox 内覆盖率 | 55.1% | 36.5% | 大场地只走了一部分，合理 |
| occupied | 1809 格 | 6632 格 | |
| occupied 密度 | 14.7 格/m² | 7.0 格/m² | 新场地开阔空间多，单位面积墙少 |
| **平均墙厚** | 2.16 格 | **2.09 格** | ⭐ **持平 = 无 ghosting** |
| 真墙数（≥50 格连通块）| 8 | 22 | 场地大，墙多 |
| 碎块数 | 174 | 1267 | |
| **碎块密度** | **1.42 个/m²** | **1.34 个/m²** | ⭐ **持平 = 无噪声恶化** |

### 9.2 结论

**两项修复都验证有效：**

1. **墙厚保持 2 格**（2.16 → 2.09）。这是判断有没有 ghosting 的关键指标：
   如果 IMU 漂移仍在，长距离行走会把墙"摊开"或留下平行双线，墙厚估计必然变大。
   这次在**面积大 7.7 倍、路径长得多**的情况下墙厚反而保持住，说明漂移被压制了。

2. **碎块密度保持 1.34 个/m²**，且已确认 `scan_filter_node` 在建图时确实在运行
   （`ps` 里能看到带 `blind_sectors:=120:150,-150:-120` 的进程）。
   既然滤波生效，这些碎块就是**环境里的真实小障碍**（桌椅腿等），不是雷达噪声。

### 9.3 一个踩过的指标坑

`map_eval.py` 最初报"碎斑占 occupied 的 35%"（room_01 是 23%），看着像变差了。

**这个指标是不可比的**：它 = 碎块格子数 ÷ occupied 总格子数。面积变大时
occupied 的增长远慢于碎块数的增长，这个比值天然会升高。

已改为按**密度**（个/m²）统计，并把告警阈值调到 4 个/m²
（1-2 个/m² 属于有家具的室内场地的正常量级）。

> 教训：跨地图、跨面积比较时，**任何"占比"指标都要先想清楚分母会不会变**。

### 9.4 仍待确认

- **闭环一致性**：走完一圈回到起点时，起点和终点区域的墙线是否重合
  （需要把 PGM 拉下来做平行双线检测，或用 `map_eval --compare` 比对重叠区）
- **里程计尺度**：仍有 6.3% 的系统误差（`linear_scale_x/y` 从未标定），
  属于下一个待修项

---

## 10. room_02 的漂移定量分析（2026-09-14）

对保存下来的 `room_02.pgm` 做了三重检测，**确认仍有漂移**：

### 10.1 平行双线（最直接的证据）

对每个长墙段沿其法向做投影直方图。连通块 #3（5.55 m 长）：

```
  -0.02 m     90  ##############################################
  +0.28 m     17  #########
```

**同一堵墙出现了两条平行线，相距 0.30 m。**

### 10.2 墙间距分布

统计水平/垂直方向上相邻 occupied 段之间的空隙：

```
空隙落在 0.10-0.60 m (2-12 格):  28.9%
```

这个区间正是"鬼影/双线"的典型尺度（不是房间尺寸，也不是墙厚）。

### 10.3 长墙的直线度

对 ≥50 格的 22 段长墙做 PCA，测量沿法向的 RMS 偏离：

| # | 格数 | 长度 | RMS | 判断 |
|---|---|---|---|---|
| 1 | 590 | 7.96 m | 7.44 | 明显弯/双线 |
| 3 | 227 | 5.55 m | 3.66 | 明显弯/双线 |
| 4 | 219 | 5.33 m | 3.40 | 明显弯（实为 L 形拐角，误报）|
| 5 | 207 | 4.86 m | 1.11 | 直 |

> 注意：PCA 会把**带拐角的 L 形墙段**也判成"弯"，所以 #4 是误报。
> 判断漂移要看 10.1 的平行双线检测，那个才是决定性的。

### 10.4 结论与剩余原因

IMU 漂移和车体自遮挡的修复**有效但不充分**。剩余三个原因，按影响排序：

| # | 原因 | 说明 | 修法 |
|---|---|---|---|
| 1 | **里程计尺度 6.3% 未标定** | `base_node_R2` 的 `linear_scale_x/y` 仍是出厂 1.0。走 30 m 闭环会累积 1.9 m 误差 | 用 `odom_calibrate.py` 标定 |
| 2 | **gmapping 不适合 28×33 m** | 粒子滤波，**没有显式回环检测**，长走廊特征重复就认不出来 | 换 slam_toolbox |
| 3 | `minimum_score: 0.0` | 接受任何匹配结果，包括明显错的 | 提到 50–150 |

---

## 11. 切换到 slam_toolbox（2026-09-14）

### 11.1 为什么换

gmapping 是 Rao-Blackwellized 粒子滤波，靠"走回旧区域时匹配分数够高"
**偶然**闭合回环。场地到 28×33 m、且有长走廊（特征高度重复）后彻底不够用——
表现在地图上就是同一堵墙被建了两遍（见 10.1）。

slam_toolbox 是**位姿图优化 + 显式回环检测**（Ceres 求解），
这个尺寸才合适。

### 11.2 新增文件

| 文件 | 说明 |
|---|---|
| `config/slam_toolbox_mapping.yaml`（更新） | scan_topic 改 `/scan_filtered`；`tf_buffer_duration` 30→60；`loop_search_maximum_distance` 3→5 |
| `src/r2_mapping_bringup/launch/mapping_slam_toolbox.launch.py`（新） | 硬件 + 扫描滤波 + slam_toolbox async |

### 11.3 启动（本包未 colcon build，必须用文件路径）

```bash
source ~/r2_mapping/scripts/env.sh
ros2 launch ~/r2_mapping/src/r2_mapping_bringup/launch/mapping_slam_toolbox.launch.py
```

> ⚠️ 必须是 `async_slam_toolbox_node` 配**本文件**的参数。
> 拿 slam_toolbox 自带的 `mapper_params_online_sync.yaml` 喂给 async 节点
> 会静默错配——节点能起来但行为不对（这个坑以前踩过）。

### 11.4 实测验证

```
[slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner
[slam_toolbox]: Registering sensor: [Custom Described Lidar]

TF map -> base_footprint      ✓
/scan            10.15 Hz     ✓
/scan_filtered   10.39 Hz     ✓
/odom            10.00 Hz     ✓
/map              0.50 Hz     ✓（map_update_interval 2.0 对应）
```

---

## 12. 蜂鸣器一直响的问题（2026-09-14）

**症状**：按解锁键后小车蜂鸣器一直响，停不下来。

**原因**：原厂 `yahboom_joy_R2` 把 `buttons[11]` 当蜂鸣器开关，
而这个键正好是我们 `ps2_teleop` 的解锁键。按一次两个节点都收到：
原厂节点往 `/Buzzer` 发 `True` → 驱动 `car.set_beep(1)` → 一直响。

**修法（两层）**：

1. **改道原厂节点的 Buzzer 话题**
   `yahboomcar_bringup_R2_launch.py` 里给 `yahboom_joy_R2` 加：
   ```python
   remappings=[('Buzzer', 'Buzzer_disabled_by_r2_mapping')],
   ```
   （install 和 src 两处都改，原文件备份为 `.bak`）
   验证：`ros2 topic info /Buzzer` → **Publisher count: 0**

2. **解锁提示音改由 `ps2_teleop` 自己发**
   新参数 `beep_on_arm`（默认 true）、`beep_count`（默认 3）、
   `beep_period_ms`（默认 150）。解锁时发 `True→False` 交替 3 组，
   **最后一帧一定是 `False`**，保证不会停在响的状态。

```bash
# 想改成响两声
python3 .../ps2_teleop.py --ros-args -p beep_count:=2
# 想静音
python3 .../ps2_teleop.py --ros-args -p beep_on_arm:=false
```

---

## 13. 真正的根因：EKF 融合了错误的里程计位置（2026-09-14）

### 13.1 现象

建图时**一开始激光点和地图墙线能重合，走一段后就逐渐分开**。

这个现象本身就是定位：**开始时误差还没累积 → 后面累积到位姿明显偏离**。

### 13.2 决定性证据

同时读两个话题的位置：

```
/odom_raw 位置: x = -16.25394609066308   y = 22.776052846703728
/odom     位置: x = -16.25394609066308   y = 22.776052846703728
                                            ↑ 完全相同，到小数点后 14 位
```

**EKF 的位置输出就是 `/odom_raw` 的位置。** 原因：`base_node_R2` 把协方差设成
`0.001`（σ = 3 cm，宣称极度精确），EKF 没有任何其他位置来源，于是完全采信。

### 13.3 数据流分析

```
base_node_R2                          EKF                      slam_toolbox
    │                                  │                            │
    │ omega = v·tan(δ)/L   ← 只看转向角 │                            │
    │ x += v·cos(heading)·dt           │                            │
    │ y += v·sin(heading)·dt           │                            │
    └── /odom_raw (x,y,vx,vy,cov=0.001)┤                            │
                                       │ 融合 x,y,vx,vy ← 位置来自这里 │
        /imu/data ─────────────────────┤ 融合 yaw,vyaw  ← 航向来自这里 │
                                       └── /odom ────────────────────→ 运动先验
```

| | 来源 | 对不对 |
|---|---|---|
| 位置 **x, y** | `base_node` 用**转向角模型**积分 | ❌ 错 |
| 航向 **yaw** | IMU 真实测量 | ✅ 对 |

**R2 有机械偏差（右后轮比左轮快 1.6–2.6%），转向角为 0 时车其实在弯，
但 `base_node` 认为在走直线 —— 它算出的路径是错的（弯的）。**

EKF 输出的位姿 = **弯的路径 + 正确的航向**，自相矛盾。

### 13.4 为什么"前面准后面偏"

```
出发：地图还是空的，误差没累积
      → scan matching 轻松对齐 → 红黑点重合 ✅

走一段：base_node 的航向误差持续累积（约 1°/s 量级）
      → 它报告的路径越来越弯
      → 运动先验越来越歪
      → scan matching 拽不回来
      → 红点和黑墙分开 ❌
```

误差是**累加**的，所以表现为"越走越偏"。

### 13.5 修改

文件（两份都要改）：

```
~/yahboomcar_ros2_ws/software/library_ws/install/robot_localization/
    share/robot_localization/params/ekf_x1_x3.yaml
~/yahboomcar_ros2_ws/software/library_ws/src/robot_localization/
    params/ekf_x1_x3.yaml
```

| | 改前 | 改后 |
|---|---|---|
| `odom0_config` | `[true, true, ..., true, true, ...]`（融合 x,y,vx,vy）| `[false×6, true(vx), false×8]` |
| `imu0_config` | `[..., yaw=true, ..., vyaw=true, ...]` | 不变 |

改后 EKF 用 **IMU 的航向** 去积分 **编码器的速度**：

```
x = ∫ vx·cos(yaw_imu) dt
y = ∫ vx·sin(yaw_imu) dt
```

位置和航向自洽。

### 13.6 预期效果

| | 航向误差率 | 20 分钟累积 |
|---|---|---|
| 改前（base_node 转向角模型） | ~1°/s ≈ 60°/min | ~1200° |
| 改后（IMU 航向） | 0.39°/min | 7.8° |

### 13.7 校验与回滚

校验：

```bash
bash ~/r2_mapping/scripts/check_ekf_config.sh
ros2 param get /ekf_filter_node odom0_config
```

回滚：把 `odom0_config` 恢复成
`[true, true, false, false, false, false, true, true, false, ...]`，
原文件备份在同目录的 `.orig_bak`。

### 13.8 与其他修复的关系

| 修复 | 解决的问题 | 是否已做 |
|---|---|---|
| `use_mag: true` | IMU yaw 的绝对参考 | ✅ |
| 扫描滤波屏蔽车尾扇区 | 车体自遮挡的假障碍 | ✅ |
| `wheelbase` 0.25 → 0.2681 | 转弯时模型多转 7.24% | ✅ |
| gmapping → slam_toolbox | 大场地缺回环检测 | ✅ |
| **EKF 不融合 odom 位置** | **位置与航向来自不一致的模型** | ✅ 本节 |

**最后一条是前面四条都没能解决"仍在漂"的原因** —— 它不在传感器侧，
而在融合层。
