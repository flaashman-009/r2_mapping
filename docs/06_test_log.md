# 上车记录

每次上车追加一条。模板：

```markdown
## YYYY-MM-DD 第 N 次

- 目的：
- 地图名：
- 关键参数：
  - slam_toolbox:
  - 速度：
  - 外参（x/y/z/yaw）：
- 自检结果（`check_all.sh` 输出摘要）：
  - /scan：
  - /odom_raw：
  - /imu：
  - TF：
  - 时间戳互差：
- 现象与问题：
- 结论 / 下一步：
```

---

## 2026-08-27 历史基线（来自 `vvc_ros2` 项目）

- 轴距 0.2681 m、轮距 0.1646 m、最大转向 45°、转向比 1.000、零位偏置 0°。
- 后轮架起：左右轮 187.3 / 187.5 ticks per 0.1 s（差 0.1%）。
- 落地直行：右后轮快约 1.6%–2.6%，可解释轻微左偏。
- 0.5 m/s 直道 5 m 闭环：odom x ≈ 4.81–4.86 m，IMU 航向终点 ≈ ±0.02 rad。
- 最终 `steer_trim_deg = −2.9`。
- 板载 `imu/yaw_deg` 跨 ±180° 会 360° 跳变，已加 unwrap。

## 2026-08-29 历史基线（来自 `vvc_ros2` 项目）

- Nav2 闭环出现 AMCL 漂移，规划器丢路径，任务 ABORTED。
- 根因：EKF 融合了 `/odom_raw` 的假 yaw，与 LiDAR/AMCL 不一致。
- 补丁方向：EKF 的 odom yaw/vyaw 权重置零；AMCL `transform_tolerance` 收紧到 0.25。
- 已有地图：`environment_final.yaml`（resolution 0.05，origin [-18.6, -16.9, 0]）。

---

## 本项目记录

### 待填：第一次建图

- 目的：跑通 Gate A 自检 + 首次 slam_toolbox 建图
- 状态：未开始

