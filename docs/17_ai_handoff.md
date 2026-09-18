# 交接说明（写给接手的 AI）

> 这份文档是给**接手这个工程的 AI 助手**看的。
> 目标：让你在 10 分钟内知道「现在是什么状态、哪些是确定的事实、哪些坑不要再踩」，
> 不需要重新做一遍已经做过的排查。
>
> 人看的版本是 `13_delivery_summary.md` 和 `14_architecture.md`，可以先扫一眼。

---

## 0. 一句话现状

**建图链路已完成并产出可用的地图；导航链路的根因已定位并修复，但最后一次调参
（`update_min_d` / `max_beams`）还没有实车验证。**

当前最大未解问题：**长走廊里 AMCL 的"似然场平坦"导致位姿滑移**。

---

## 1. 接手后先做的事（5 分钟）

```bash
# ① 拿到车的当前 IP（重点：IP 每次都变，见第 3 节）
# ② 连上去自检
ssh jetson@<车IP>
bash ~/r2_mapping/scripts/usb_status.sh      # 外设齐不齐
bash ~/r2_mapping/scripts/verify_changes.sh  # 参数有没有到位

# ③ 起导航
bash ~/r2_mapping/scripts/nav.sh

# ④ 另开终端挂记录仪（排查任何问题都靠它）
python3 -u ~/r2_mapping/scripts/nav_watch.py
```

---

## 2. 工程是什么

Yahboom ROSMaster R2（**阿克曼**）小车上，用 ROS2 Humble 做 **2D 建图 + 导航**。

代码在 Windows（`C:\Users\zhuchenju\Desktop\建图\r2_mapping`），
通过 GitHub（[flaashman-009/r2_mapping](https://github.com/flaashman-009/r2_mapping)）
和虚拟机（`flaash@192.168.232.129`，本地裸仓库 `~/repos/r2_mapping.git`）同步，
最后 **scp 到小车**（`jetson@<车IP>:~/r2_mapping`，**小车不参与 git**）。

```
硬件层：STM32 底盘(/dev/myserial) + YDLIDAR 4ROS(/dev/ydlidar)
定位层：EKF（robot_localization）→ AMCL（Nav2）
规划层：planner_server + controller_server(TEB)
适配层：cmd_vel_ackermann.py   ★本项目自写，最容易被改坏的地方
保护层：localization_guard.py  ★本项目自写
```

---

## 3. 环境与访问（**这里有坑**）

| 项 | 值 |
|---|---|
| **小车 IP** | **每次都变**（见过 `192.168.43.16` / `.15` / `.10` / `192.168.49.8` / `192.168.0.232`）。**每次先确认** |
| 小车 SSH | `ssh jetson@<IP>`，密码见用户的本地保管记录（**不在仓库里**） |
| **SSH 首次连接** | 必须加 `-o StrictHostKeyChecking=accept-new`，否则会**卡在"是否信任主机"的交互提示上**（表现为命令挂死） |
| ROS | Humble，`ROS_DOMAIN_ID=28`（`scripts/env.sh` 会设） |
| 虚拟机 | `flaash@192.168.232.129`，Ubuntu 22.04，VS Code 已装 |
| Windows git 代理 | `127.0.0.1:7890`（加速器端口，已配进全局 git config） |

### 三个反复踩的工具坑

**坑 1：PowerShell 吃引号**

从 Windows 经 ssh 拼带消息体的 `ros2 topic pub`，引号会被吞掉 → 命令**静默失败**。
2026-09-16 的一次急停就栽在这上面（以为发了零速，实际一条没发，车继续跑）。

**规则：含引号的 ROS 命令一律写成 `.sh` 文件再执行。**

**坑 2：`pkill -f` 自匹配**

如果同一条命令里既有 `pkill -f Ackman` 又有 `grep Ackman`，pkill 会**把执行命令的
shell 自己杀掉**，命令中断且无输出。模式里加方括号规避：`Ackman_[d]river`。

**坑 3：ros2 发现服务抖动**

`ros2 topic list` 会漏报**正在正常发布**的话题（实测 `/odom` 稳定 10 Hz，但
`nav.sh` 卡在"等 /odom 超时"45 秒）。`ros2 daemon stop` 后即恢复。
`nav.sh` 已加自动重试，别把它删掉。

---

## 4. 两个自写节点：**改之前必须读懂为什么**

### `scripts/cmd_vel_ackermann.py` —— 转向适配

**为什么存在**（2026-09-16 实车实测，不是推测）：

这台车底盘有两条转向通道：

| 通道 | 实测行为 |
|---|---|
| `linear.y` | 直接给角度，**值 × 1000 = 度数**。`0.02→20.0°`、`0.03→30.0°`、`0.001→1.0°`，任何车速都立即生效 |
| `angular.z` | 固件自己换算，但**车静止时完全不起作用**；**行驶中发 0 时"保持上一次角度"，不回正** |

坑后者是致命的：Nav2 的语义是"ω=0 → 直行"，底盘理解成"不更新转向"。
实测运动 567 帧里有 **67 帧 `angular.z=0`，那 67 帧前轮全部卡在 −25°** ——
**车在画弧，控制器却以为在走直线**。点云因此持续偏移、误差累积。

**本节点的做法**：不依赖固件换算，自己按阿克曼模型算，走 `linear.y`：

    δ = atan(ω · L / v)      L = 0.2681 m（轴距，实测）
    δ 限幅 ±40°（倒车 ±22°）
    转速率限制 45°/s + 死区 1.5°
    δ≈0 时强制发 ±1°（固件把 0 当"不更新"）

**已实测验证**：`v=0.23, ω=0.5` → 算出 `δ=30.2°` → `linear.y=0.0302` → **舵机精确到 30.0°**。

### `scripts/localization_guard.py` —— 定位护卫

**为什么存在**：AMCL 会崩。实测最严重的一次：`map→odom` **跳 90 米、航向翻转 177°**，
导致"车朝目标反方向开"。

**做法**：只订阅 `/amcl_pose`（不用地图、不用激光，代价接近零）：

    协方差 > 20 持续 2 秒  →  发 /cmd_vel_halt 停车 + 提示用 RViz 的 2D Pose Estimate 重定位
    协方差 < 1.0 持续 5 秒  →  自动解除

**已实测触发过一次**（协方差 255 → 停车 → 30 秒后恢复 0.52 → 自动解除），有效。

**不需要"取消 goal"**：Nav2 里发新 goal 会自动抢占旧 goal。

---

## 5. 当前状态：哪些是确定的，哪些还需要验证

### ✅ 已实测确认（可以直接信）

| 结论 | 证据 |
|---|---|
| 转向必须走 `linear.y`，`angular.z` 有"0=保持"的坑 | 架起车逐条发指令实测 |
| 适配节点换算正确 | `v=0.23,ω=0.5 → δ=30.2° → 舵机 30.0°` |
| AMCL 用原始 `/scan` 会和地图不一致 | 地图是 `/scan_filtered` 建的（屏蔽车尾 ±120–150°） |
| 点云残差是有效的定位健康指标 | **静止且位姿正确时精确为 0.000 m** |
| 导航链路接线正确 | `/cmd_vel_nav`(2 发布者) → 适配节点 → `/cmd_vel`(1 发布 1 订阅)；节点无重复 |
| `map→odom` 启动即存在 | 加了 `set_initial_pose: true`，RViz 不再白屏 |
| 地图 room_05 可用 | 墙厚 2.24 格、碎斑 3.8%、24 个大障碍块 |

### ⚠️ 已实现但**尚未实车验证**

| 改动 | 值 | 预期效果 |
|---|---|---|
| `update_min_d` | 0.05 → **0.15** | 停车/低速时滑移机会减少 ~70% |
| `update_min_a` | 0.05 → **0.10** | 同上 |
| `max_beams` | 180 → **360** | 长走廊方向约束翻倍 |
| 倒车限幅 | **±22°** | 治"倒车时转向被数学放大"（实测倒到 35°） |
| `recovery_alpha_fast/slow` | 0.05 / 0.0005 | 减少随机粒子跳走 |
| 定位护卫 | cov>20 停 | 已触发过一次，算部分验证 |

### ❓ 仍是推测 / 未解决

1. **长走廊"似然场平坦"是当前最大未解问题**
   - 实测：车真实位移 0.34 m，AMCL 估计却移动了 **3.9 m**
   - 最后一段行驶：`map→odom` 在 **40 米范围**内摆动、>1m 跳变 **39 次**
   - 推断原因：走廊方向得分几乎不变，粒子的唯一约束来自里程计；车慢下来约束就消失
   - **这是"到 goal 前很准、停下就不准"的真正原因**

2. 雷达外参**未标定**（`config/lidar_extrinsics.yaml` 里还是 `null`）
3. 里程计尺度**未正式标定**（有工具 `odom_calibrate.py`）
4. 这台车的驱动**不发布 `/imu/yaw_deg`**，所以航向只有 madgwick 一个来源，**没有独立交叉验证**

---

## 6. 必须知道的坑（都踩过，别重踩）

| # | 坑 | 表现 | 处理 |
|---|---|---|---|
| 1 | 底盘 `angular.z=0` 不回正 | 车画弧却以为在直行 | 走 `linear.y`，别改回去 |
| 2 | Nav2 自带 `velocity_smoother` | **会把 `linear.y` 丢掉**（实测发 0.02 出来是 0） | 本项目 launch **故意不启动它** |
| 3 | 低速时转向角被放大 | `δ=atan(ωL/\|v\|)`，\|v\| 小就爆 | 倒车单独限幅 ±22° |
| 4 | AMCL 只在收到激光才发 TF | 不设初始位姿 → RViz 全黑 | 已设 `set_initial_pose: true` |
| 5 | `nav.sh stop` 杀不掉 launch 父进程 | 攒出**两个 controller_server 抢 /cmd_vel** | 文件名是 `navigation_r2.launch.py`（**点号**），pkill 模式要写对 |
| 6 | Windows 的 git 默认转 CRLF | shell 脚本到 Linux 报 `bad interpreter: /bin/bash^M` | 仓库有 `.gitattributes` 强制 LF，**别删** |
| 7 | 修改 `nav_r2.yaml` 后不重启不生效 | 参数看着改了但行为没变 | 改完跑 `nav.sh stop && nav.sh`，再用 `verify_amcl_tuning.sh` 确认 |
| 8 | 导航目标发到没建过图的区域 | 定位在"平地"上滑走 | 发目标前在 RViz 里确认目标是白色区域 |

---

## 7. 待办（按优先级）

| 优先级 | 事项 | 验收标准 |
|---|---|---|
| **P0** | 实车验证 `update_min_d=0.15` + `max_beams=360` | 记录仪里 **"停车前后位姿滑移" < 1 m**（当前 3.9 m） |
| **P1** | 长走廊定位稳定性 | `map→odom` 单步跳变 >1m 的次数显著下降（当前 420 次/趟） |
| P2 | 雷达外参标定 | 车正对墙，扫描呈一条直线，误差 < 2° |
| P3 | 里程计尺度标定 | 推车 10 m，odom 读数误差 < 2% |
| P4 | 地图补全 | 若要在未知区域导航，需重扫（当前覆盖率 18.5%） |
| P5 | 把 `r2_mapping` 做成正式 ROS2 包 | 目前 launch 全靠**绝对路径**，未 colcon build |

---

## 8. 工具清单（排查问题先跑这些）

```bash
bash ~/r2_mapping/scripts/usb_status.sh        # 外设齐不齐（雷达/底盘/手柄）
bash ~/r2_mapping/scripts/verify_nav_live.sh   # 导航链路接线对不对
bash ~/r2_mapping/scripts/check_no_dup.sh      # 有没有重复节点
bash ~/r2_mapping/scripts/verify_amcl_tuning.sh # AMCL 调参生效没
bash ~/r2_mapping/scripts/why_no_map.sh        # RViz 没地图时排查
bash ~/r2_mapping/scripts/estop.sh             # 急停（带底盘反馈验证）
```

电脑端分析（不需要 ROS）：

```powershell
python .\tools\analysis\analyze_nav_watch.py .\logs\xxx.csv --png .\logs\diag.png
python .\tools\analysis\deep_nav_report.py   .\logs\xxx.csv
python .\tools\analysis\pose_distance_to_known.py .\logs\xxx.csv .\maps\room_05.yaml
```

**记录仪 CSV 的列含义见 `logs/README.md`**，两个最重要的判据：

    res_med   点云残差   静止且正确=0.000 / 正常 0.05~0.20 / 开始漂 >0.35
    amcl_cov  协方差     正常 0.05~0.5 / 开始漂 几十 / 迷失 100~400

---

## 9. 关键文件位置

| 想改什么 | 打开 |
|---|---|
| **转向限幅 / 死区 / 倒车限幅** | `config/nav_r2.yaml` → `cmd_vel_ackermann:` 段 |
| TEB 控制器参数 | 同上 → `controller_server.FollowPath` |
| AMCL 定位参数 | 同上 → `amcl:` 段 |
| 代价地图 / 车体尺寸 | 同上 → `local_costmap` / `global_costmap` |
| 导航启动流程 | `src/r2_mapping_bringup/launch/navigation_r2.launch.py` |
| 建图参数 | `src/r2_mapping_bringup/config/slam_toolbox_mapping.yaml` |
| 转向换算逻辑 | `scripts/cmd_vel_ackermann.py` |
| 迷失保护策略 | `scripts/localization_guard.py` |

**架构和数据流看 `docs/14_architecture.md`。**

---

## 10. 给接手 AI 的协作约定

1. **用户在 Windows 上操作，代码在 Windows，车在局域网**
   - IP 每次都变，**先问或先探测**
   - 改完代码要 **scp 到小车**才生效

2. **沟通用中文，先讲原理再给命令**
   - 用户不熟 ROS 内部机制（AMCL/TEB/似然场这些概念要解释）
   - 喜欢**量化证据**，不接受"我觉得"
   - 明确说过"希望以后能自己调参数，不是完全依赖 AI" → **文档要写清参数含义**

3. **不要只报结论，要报证据**
   - 这个项目所有关键结论都有实测数据支撑（在 `logs/*.csv` 里）
   - 用户已经被"看起来对但其实是猜"坑过

4. **说话要直，不要粉饰**
   - 出过的事故（急停失败那次）要如实说清原因，用户接受得了

---

## 11. 绝对不要做的事

| 禁止 | 原因 |
|---|---|
| 重新加回 `velocity_smoother` | 会把转向通道 `linear.y` 丢掉 |
| 把转向改回 `angular.z` | 底盘"发 0 保持上次角度"，车会画弧 |
| 删掉 `.gitattributes` | shell 脚本会在 Linux 上因 CRLF 报错 |
| 在机器人**静止**时让它继续跑导航 | AMCL 无约束会自漂（当前最大问题） |
| 把密码/密钥/公网 IP 写进仓库 | **仓库是公开的** |
| 从 PowerShell 直接拼带引号的 `ros2 topic pub` | 引号被吞，命令静默失败 |
| 用 `pkill -f xxx` 同时又在同一命令行里 `grep xxx` | 会杀掉自己 |
