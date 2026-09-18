# 第一次建图 → 现在：到底改了什么

时间跨度：2026-09-11（room_01）→ 2026-09-14（room_04）

一句话：**改了 5 处关键配置，跨越 4 个层次；另外建了 8 个诊断工具，
才把"地图飘"从一个模糊的现象追到一个具体的错误参数。**

---

## 一、五处关键改动

### ① 传感器层 —— IMU 打开磁力计

| 项 | 改前 | 改后 |
|---|---|---|
| 文件 | `yahboomcar_bringup/share/yahboomcar_bringup/param/imu_filter_param.yaml` | 同 |
| 参数 | `use_mag: false` | `use_mag: true` |

**为什么**：Madgwick 不用磁力计时，yaw 没有绝对参考，只能靠陀螺仪积分，
必然漂移。而 EKF 的航向正是取自它。

**实测效果**：静止漂移 **0.78 → 0.39 度/分钟**（建图 20 分钟累计 15.6 度 → 7.8 度）

---

### ② 传感器层 —— 屏蔽车体自遮挡

| 项 | 改前 | 改后 |
|---|---|---|
| 新增节点 | 无 | `scan_filter_node`，屏蔽 ±(120°~150°) 扇区 |
| SLAM 输入 | `/scan` | `/scan_filtered` |

**为什么**：LiDAR 装在车体前部，会扫到**车尾左右两个角**（几何推算 ±147°，
实测吻合）。这些点在 0.15–0.35 m 处，占有效点 9.3%，会被当成障碍写进地图。

**实测效果**：近点 **108 → 19 个**；车尾两扇区归零，其它方向的真实近障碍保留。

---

### ③ 底盘模型层 —— 修正轴距

| 项 | 改前 | 改后 |
|---|---|---|
| 文件 | `yahboomcar_bringup/.../launch/yahboomcar_bringup_R2_launch.py` | 同 |
| 参数 | 未设置（用源码默认 `0.25`） | `'wheelbase': 0.2681` |

**为什么**：`base_node_R2.cpp` 用阿克曼模型算航向：
`R = wheelbase / tan(steer_angle)`，`omega = v / R`。

出厂默认 `wheelbase = 0.25`，而 R2 **实测轴距 0.2681**，差 **7.24%**
→ **每次转弯都多转 7.24%**。走一圈 4 个 90° 直角，累积航向误差约 26°。

而且**任何 launch 都没设置过这个参数**，一直是出厂值。

---

### ④ SLAM 层 —— gmapping 换成 slam_toolbox

| 项 | 改前 | 改后 |
|---|---|---|
| 算法 | `slam_gmapping`（粒子滤波） | `slam_toolbox`（图优化 + 显式回环） |
| 启动 | `map_gmapping_4ros_launch.py` | `mapping_slam_toolbox.launch.py` |
| 参数 | 厂默认 | 本项目 `config/slam_toolbox_mapping.yaml` |

**为什么**：场地到 28×33 m、有长走廊（两面平行墙特征高度重复）。
gmapping 靠"走回旧区域时匹配分数够高"**偶然**闭合，长走廊里认不出来——
实测地图上出现了**相距 0.30 m 的平行双线**（同一堵墙建了两遍）。

slam_toolbox 是位姿图优化 + 显式回环检测，这个尺寸才合适。

> 顺带记录了 gmapping 的原厂参数问题（以后若要用）：
> `linearUpdate: 1.0 m`（走 1 米才处理一帧）、`maxUrange: 4.0 m`
> （只用 4 米内特征）、`particles: 30`、地图范围仅 ±10 m。

---

### ⑤ 融合层 —— EKF 不再采信里程计的位置 ⭐ 最关键

| 项 | 改前 | 改后 |
|---|---|---|
| 文件 | `robot_localization/share/robot_localization/params/ekf_x1_x3.yaml`（install + src 两处） | 同 |
| `odom0_config` | 融合 **x, y**, vx, vy | 只融合 **vx** |

**为什么**：这是前面四条都没能解决"仍在漂"的原因——**它不在传感器侧，在融合层。**

数据流：

```
base_node_R2                                  EKF                    slam_toolbox
    │                                          │                          │
    │ omega = v*tan(steer)/L   <- 只看转向角    │                          │
    │ x += v*cos(heading)*dt                   │                          │
    │ y += v*sin(heading)*dt                   │                          │
    └── /odom_raw (x,y,vx,vy, cov=0.001) ──────┤                          │
                                               │ 融合 x,y,vx,vy <- 位置来自这里
        /imu/data ─────────────────────────────┤ 融合 yaw,vyaw   <- 航向来自这里
                                               └── /odom ────────────────> 运动先验
```

问题：**位置来自错误模型，航向来自正确传感器，两者自相矛盾。**

而且 `base_node` 把协方差设成 `0.001`（σ = 3cm，宣称极度精确），EKF 完全采信。

**决定性证据**：

```
/odom_raw 位置: x = -16.25394609066308   y = 22.776052846703728
/odom     位置: x = -16.25394609066308   y = 22.776052846703728
                                              ↑ 完全相同，到小数点后 14 位
```

改后 EKF 用 **IMU 的航向** 积分 **编码器的速度**：
`x = ∫ vx*cos(yaw_imu) dt`，`y = ∫ vx*sin(yaw_imu) dt`。位置和航向自洽。

**这条直接对应你观察到的现象**："一开始红黑点能重合，走一段就分开"——
因为误差是**累加**的。改完之后，**这个现象消失了**。

---

## 二、附带的改动

| 改动 | 原因 |
|---|---|
| **禁用原厂手柄节点** `yahboom_joy_R2` | 解锁键是 `buttons[9]`（这手柄不产生）；一旦被解锁，转向增益是 5.0（=5000°）而打满前轮只需 0.045——**推一下摇杆就打满舵**；还把 `buttons[11]` 当蜂鸣器开关，和我们的解锁键冲突 |
| 原厂节点的 `Buzzer` 话题改道 | 同上，按解锁键会让小车一直响 |
| RViz 轻量配置 | Frame Rate 30→10，只留 Grid/TF/LaserScan/Map。实测 RViz 曾占 72% CPU，把 planner 挤到 1.16 Hz |

---

## 三、效果对比

| 指标 | room_01（第一次） | room_04（现在） | 变化 |
|---|---|---|---|
| SLAM 算法 | gmapping | slam_toolbox | — |
| **已探索面积** | 123 m² | **3000 m²** | **↑ 24 倍** |
| **碎斑密度** | 1.42 个/m² | **0.08 个/m²** | **↓ 17 倍** |
| **鬼影区**（0.10–0.60 m 间距） | — | 14.6%（room_02 是 28.9%） | **↓ 一半** |
| 真墙段（≥50 格连通块） | 8 | 24 | ↑ |
| **红黑点是否分离** | **会** | **不会** | ✅ |

**在面积大 24 倍的场地上，地图反而干净 17 倍。**

---

## 四、每条改动对应哪个症状

| 症状 | 对应的改动 |
|---|---|
| 地图整体被"拧" | ③ wheelbase + ⑤ EKF 位置 |
| 地图上出现平行双线（同一堵墙建两遍） | ④ 换 slam_toolbox |
| 地图上有跟着车走的一圈假点 | ② 扫描滤波 |
| 走长距离后累积偏移 | ① IMU 磁力计 + ⑤ EKF 位置 |
| **"一开始重合，走一段就分开"** | **⑤ EKF 位置（最关键）** |

---

## 五、为了找到这些，建了什么工具

这些工具本身不是"改动"，但**没有它们就找不到根因**：

| 工具 | 用途 | 关键时刻 |
|---|---|---|
| `scripts/imu_drift.py` | 测静止航向漂移率 + 对比 `/odom_raw` vs `/imu/data` | 测出 0.78°/min，并证明 EKF 航向来自 IMU |
| `scripts/scan_stats.py` | 分析激光近点角度分布 | 发现车尾 ±147° 自遮挡，与几何推算吻合 |
| `scripts/js_raw.py` | 绕开 joy_node 直读 `/dev/input/js0` | 证明"手柄没反应"其实是驱动卡死，不是手柄坏 |
| `scripts/steer_center.py` + `steer_center_cmd.sh` | 前轮回正 / 转向量纲验证 | 发现固件忽略 `linear.y=0`、最小步进 1° |
| `scripts/restart_hardware.sh` | 干净重启（清 fastrtps 共享内存、清残留进程） | 固化了三个踩过的坑 |
| `scripts/check_ekf_config.sh` | 校验 EKF 参数真的改了 | 避免"以为改了其实没改" |
| `tools/map_eval.py` | 地图质量评估（碎斑密度、鬼影区、覆盖率） | 定量对比四次建图 |
| `tools/r2_odom.py` | 参数完全可调的里程计（备选方案） | 留给下一步用 |

---

## 六、排查方法论（值得复用）

1. **先量化，再动手**
   没有"0.78°/min"这个数字，就不知道改 `use_mag` 对不对。

2. **用两个独立来源互相对照**
   IMU 是尺子，`/odom_raw` 是被告。两个一减，误差就现形了。

3. **症状本身就是线索**
   "开始准、后面偏"直接指向"误差在累积"，而不是"一开始就错"。
   后者是标定问题，前者是模型问题。

4. **分层排查，不要跳层**
   传感器 → 底盘模型 → SLAM → 融合层。
   我们在前三个层次改了四处，最后才发现根因在**融合层**——
   前面全是必要的排除。

5. **改一个，验证一个**
   每次只动一个参数，改完立刻用工具验证方向对不对。

6. **跨地图比较要用"密度"不要用"占比"**
   碎斑"占 occupied 的百分比"会随面积变化失真，
   换成"个/m²"才可比。（这个坑我们也踩了）

---

## 七、还没做的

| 项 | 说明 |
|---|---|
| 里程计尺度标定 | `linear_scale_x/y` 仍是出厂 1.0，实测有约 6% 误差 |
| 用 `r2_odom` + 标定 `yaw_bias_per_m` | 备选方案；现在问题已解决，暂不需要 |
| 同场地对照复测 | 想留一份严格的"改进前 vs 改进后"对比，需要在同一路线重跑 |
| 导航复测 | 用 `room_04` 跑 AMCL + Nav2，验证定位稳定性 |

---

## 八、文件清单（本次改动的落点）

| 文件 | 改动 |
|---|---|
| `imu_filter_param.yaml`（install + src） | `use_mag: true`，原文件备份 `.bak` |
| `ekf_x1_x3.yaml`（install + src） | `odom0_config` 只留 vx，原文件备份 `.orig_bak` |
| `yahboomcar_bringup_R2_launch.py`（install + src） | `wheelbase: 0.2681`、Buzzer 改道、禁用原厂手柄节点，备份 `.bak` |
| `config/slam_toolbox_mapping.yaml` | slam_toolbox 参数（本项目新增） |
| `src/r2_mapping_bringup/launch/mapping_slam_toolbox.launch.py` | 建图 launch（本项目新增） |
| `src/r2_mapping_perception/.../scan_filter_node.py` | 扫描滤波（本项目新增） |
| `src/r2_mapping_bringup/rviz/rviz_mapping_lite.rviz` | 轻量 RViz 配置（本项目新增） |

