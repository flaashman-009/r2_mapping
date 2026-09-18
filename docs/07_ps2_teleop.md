# R2 PS2 手柄遥控操作手册

> 本文基于小车上的实际代码与实测数据整理（2026-09-11）。
> 数据来源：`yahboom_joy_R2.py`、`Ackman_driver_R2.py`、
> `laser_bringup_tg_launch.py`，以及 `/joy`、`/voltage` 的实测抓取。

---

## 1. 链路：手柄是怎么把车开动的

```
PS2 手柄
  └─(2.4G USB 接收器)→ /dev/input/js0
        └─ joy_node            → /joy          15 Hz，8 轴 15 键
              └─ yahboom_joy_R2 → /cmd_vel     ← 必须先"解锁"才会发
                    └─ Ackman_driver_R2 → /dev/myserial(115200) → STM32 → 电机
```

四个关键点：

1. `/dev/input/js0` 存在不代表能遥控，只代表接收器被识别了。
2. `yahboom_joy_R2` 有个**软件锁** `Joy_active`，初始是 `False`，
   不解锁就一个字节都不会往 `/cmd_vel` 发。
3. 只有 `Ackman_driver_R2` 独占 `/dev/myserial`；
   它一挂，整车指令通路就断了（手柄再按也没用）。
4. 建图、定位、遥控三者共用这条底盘链路，**同一时间只能有一套驱动在跑**。

---

## 2. 实测硬件信息（2026-09-11）

| 项目 | 值 |
|---|---|
| 接收器 | `0079:181c` DragonRise "Controller"，`usb-1-2.4.2` |
| 设备节点 | `/dev/input/js0`（同时有 `/dev/input/event2`） |
| `/joy` 频率 | **约 15 Hz** |
| 轴数量 | **8**（索引 0–7），松开时 `axes[4]=axes[5]=1.0` |
| 按键数量 | **15**（索引 0–14） |
| 底盘板电压 | **12.2 V**（3S，约 85–90%） |
| 固件 edition | **3.6** |
| 蜂鸣器 | 由 `/Buzzer` 触发，驱动端 `car.set_beep()` |

---

## 3. 启动流程

### 3.1 首次准备（每个终端都要）

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
```

### 3.2 起遥控全套（一个命令够了）

```bash
ros2 launch ~/closed_loop/laser_bringup_tg_launch.py robot_type:=r2 rplidar_type:=4ROS
```

这一条会拉起：`Ackman_driver_R2`（开串口）、`base_node_R2`、
`imu_filter_madgwick`、`robot_localization` EKF、`robot_state_publisher`、
`joint_state_publisher`、`yahboom_joy_R2`、`joy_node`。

> ⚠️ 不要用 `R2_joy_ctrl.launch.py`，那是多机版，
> 指令发到 `robot1/cmd_vel`，单车上表现为"手柄完全没反应"。

### 3.3 什么时候可以只起手柄节点

只想验证手柄（不碰电机、不会叫）：

```bash
ros2 run joy joy_node
```

没有 `Ackman_driver_R2`，`/cmd_vel` 没人听，绝对安全。

---

## 4. 操作步骤（标准流程）

### 第 1 步：确认手柄在线

```bash
ros2 topic hz /joy            # 应该有 ~15 Hz
ros2 topic echo /joy --once
```

看不到数据 → 接收器没插好或手柄没开，见第 8 节。

### 第 2 步：解锁（按 `buttons[11]`）

> ⚠️ 用**原厂** `yahboom_joy_R2` 时这一步做不到，因为它等你按 `buttons[9]`，
> 而这个手柄不产生索引 9（见 6.2）。下面按使用 `ps2_teleop` 来写。

按一下**蜂鸣器键**（就是会"叫三声"的那个），`ps2_teleop` 的终端会打出：

```
[WARN] ★ 已解锁，可以遥控
```

再按一次会打 `☆ 已上锁` 并发零速。

**没看到 `★` 就说明没解锁，手柄一定开不动车**，不要以为是车坏了。
先按第 8 节的排查表确认 `/joy` 有数据。

### 第 3 步：确认指令通路

再开一个终端：

```bash
ros2 topic echo /cmd_vel
```

推摇杆，这里应该有数据。没有 → 还没解锁，或 `yahboom_joy_R2` 没起来。

### 第 4 步：低速试车

1. **先按一次档位键把速度降到 1/4**（见第 5 节 `buttons[13]`）。
2. 左摇杆**轻推**前进，确认方向正确。
3. 捏住油门不松手地感受一下速度，再决定要不要全速。
4. 转向用右摇杆。

### 第 5 步：停车与退出

**按一次 Start** → 上锁，并立即发一次零速度指令。

退出整套：

```bash
# 回到跑 launch 的那个终端
Ctrl-C
```

---

## 5. 按键与轴的功能表

> 下表左列是**软件索引**（代码里写死的），右列是它在代码中的作用。
> 物理按键与索引的对应关系见第 6 节，必须在实车上核对一次。

### 5.1 轴（`axes[]`）

| 索引 | 作用 | 代码 |
|---|---|---|
| `axes[1]` | **前后** | `linear.x = axes[1] × 1.0 × linear_Gear` |
| `axes[2]` | **转向** | `linear.y = axes[2] × 5.0 × linear_Gear` |
| `axes[0]`, `axes[3]` | 本分支未使用 | — |
| `axes[4]`, `axes[5]` | 松开时读数为 `1.0` | 未使用 |

死区：`filter_data()` 把幅度 < 0.2 的轴直接当 0，所以轻微推杆不动是正常的。

### 5.2 按键（`buttons[]`）

| 索引 | 作用 | 说明 |
|---|---|---|
| `buttons[9]` | **遥控解锁 / 上锁** | 最常用。翻转 `Joy_active`，发 `/JoyState`，并发一次零速度 |
| `buttons[13]` | **速度档（减速）** | 按一次把 `linear_Gear` 从 1.0 降到 0.25，**且降不回来**（代码 bug，见下） |
| `buttons[14]` | 转向档 | 看起来能循环，但**实际无效**（见下） |
| `buttons[11]` | **蜂鸣器开关** | 翻转 `Buzzer_active` 并发 `/Buzzer` |
| `buttons[7]` | RGB 灯 | `RGBLight_index` 0→6 循环 |

### 5.3 必须知道的"坑"

**坑 1：转向档是假的**

`user_jetson()` 里转向输出用的是 `linear.y`，而 `linear.y` 只乘了
`linear_Gear`，没乘 `angular_Gear`。而且 `twist.angular.z` 那一行是注释掉的。
所以 `buttons[14]` 按了没有效果。

**坑 2：速度档只能降不能升**

```python
if self.linear_Gear == 1.0:      self.linear_Gear = 1.0/4
elif self.linear_Gear == 1.0/3:  self.linear_Gear = 2.0/4
elif self.linear_Gear == 2.0/3:  self.linear_Gear = 1
```

从 1.0 按一次变成 0.25，而 `0.25` 既不等于 `1/3` 也不等于 `2/3`，
所以**再也回不到全速**，除非重启 `yahboom_joy_R2`。

用法建议：想全速就别碰它；想安全就按一次降到 25%，重启节点才恢复。

**坑 3：转向量纲 = deg/1000（已实测确认）**

驱动端 `vy = msg.linear.y * 1.0` 直接送进 `set_car_motion()`。
实测（`scripts/steer_center.py --probe`）：

| 指令 `linear.y` | `/vel_raw.linear.y` 反馈 |
|---|---|
| `0.01` | `+10.00`（度） |
| `0.03` | 应为 `+30.00` |

所以就是 **`linear.y = 转向角(deg) / 1000`**，和文档记录一致。
`ps2_teleop` 的 `max_steer = 0.02` 换算过来是 **20°** 的最大转角。

> 原厂手柄节点输出能到 ±5.0，等于 ±5000°，
> 一推杆就会把转向打到满舵。这也是不用它的理由之一。

**坑 4：`linear.y = 0` 不会回正，而且最小步进只有 1°**

这是本车最反直觉的一条。实测（`steer_center.py --probe`）：

```
基线              : -1.00 deg
发 +0.01（+10°）  : +10.00 deg   ← 指令生效
发 0.0            : +10.00 deg   ← 完全不动！固件忽略 0
再等 1 s          : +10.00 deg
```

继续测最小可分辨率：

| 指令 | 反馈 |
|---|---|
| `0.0001`（0.1°） | 忽略 |
| `0.0005`（0.5°） | 忽略 |
| `0.001`（1.0°） | ✅ 生效 → +1.00° |

结论：

1. 固件把 `linear.y == 0` 当成"不更新转向"，所以**松杆后前轮停在原地**；
2. 固件的**最小转向步进是 1°**，所以"回正"能做到的极限就是 ±1°；
3. `ps2_teleop` 用 `steer_epsilon = 0.001` 代替 0 来强制回中——
   这是能生效的最小值，不是随手取的。

停车/回正时如果绕过 `ps2_teleop` 手动发指令，也必须发非零值：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.001, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 5.4 每条指令发三遍

```python
for i in range(3): self.pub_cmdVel.publish(twist)
```

15 Hz 的 `/joy` 会产生约 45 Hz 的 `/cmd_vel`，这是厂商写法，不是故障。

---

## 6. 实测对照表（2026-09-11 实测，可直接用）

### 6.1 轴

| 动作 | 原始 `/dev/input/js0` | `joy_node` 的 `/joy` | 代码里怎么用 |
|---|---|---|---|
| 左摇杆**推上** | `AXIS 1 = -32767` | `axes[1] = +1.000` | `linear.x` 为正 → **前进** |
| 左摇杆**推下** | `AXIS 1 = +32767` | `axes[1] = -1.000` | `linear.x` 为负 → 后退 |
| 右摇杆**推左** | `AXIS 2 = -32767` | `axes[2] = +1.000` | `linear.y` 为正 |
| 右摇杆**推右** | `AXIS 2 = +32767` | `axes[2] = -1.000` | `linear.y` 为负 |

结论：

- 轴号与 `ps2_teleop` 默认值一致（`axis_linear=1`、`axis_steer=2`）；
- **前后方向不需要反转**（`axis_linear_invert` 保持 false）；
- 转向的正负含义（正 = 左还是右）仍需上车确认。

> ⚠️ 注意 `joy_node`（SDL 实现）对轴的正负号和内核 joystick API **是相反的**。
> 用 `scripts/js_raw.py` 看原始值调试时，别被符号搞混。

### 6.2 按键

| 物理键 | 索引 | 状态 |
|---|---|---|
| 蜂鸣器键 | `buttons[11]` | ✅ 已验证（按一次车叫 3 声，`ps2_teleop` 也用它当解锁键） |
| 面键 / L 键 / Select | 待核对 | ⬜ 未测 |
| 原厂代码期望的"解锁键" | `buttons[9]` | ❌ **这个手柄不产生索引 9**，所以原厂节点永远解锁不了 |

### 6.3 踩过的坑：MODE 键

**症状**：`/joy` 里 4 个摇杆轴恒等于 0，但按键正常。

**原因**：手柄处在**数字模式（DIGITAL）**，此模式下摇杆完全不输出。

**处理**：按手柄正中间的 **MODE / ANALOG** 键切到模拟模式。

**这个坑最坑的地方**：这块手柄**没有指示灯**，按下去看不出任何变化，
很容易误判成"按键没反应"。实测确认：按完摇杆立刻就有输出了。

**排查命令**（绕过 joy_node，直接看硬件）：

```bash
python3 ~/r2_mapping/scripts/js_raw.py
```

推摇杆有 `AXIS` 输出 → 手柄正常，问题在 ROS 侧；
连 `AXIS` 都没有 → 手柄/接收器的问题。

---

### 6.4 怎么把物理按键和索引对上号

代码只知道索引，不知道你手上那个键叫什么。核对方法（2 分钟）：

**终端 A**

```bash
ros2 run joy joy_node
```

**终端 B**

```bash
ros2 topic echo /joy
```

然后**一次只按一个键**，看 `buttons:` 数组里哪个位置从 0 变成 1。
例如按下去后看到这一行第 10 个数字变成 1，那就是 `buttons[9]`
（数组从 0 开始数）。

核对时**不要跑 `laser_bringup_tg_launch.py`**，这样按任何键都不会动、
也不会叫。

建议记录成一张表（下面只填了已核实的行）：

| 物理按键 | 索引 | 实测日期 |
|---|---|---|
| 蜂鸣器键（同时用作 `ps2_teleop` 的解锁键） | 11 | 2026-09-11 |
| 面键 / L 键 / Select | 待填 | |

> 📌 待办：面键和 L 键的索引还没核对。
> 不影响使用——解锁键和两个摇杆轴都已经实测确认了。

---

## 7. 急停

按优先级：

1. **断电**（最快、最彻底）。
2. **再按一次 Start** → 上锁并发零速度。
3. 回到跑 launch 的终端按 **Ctrl-C**。
4. 另一个终端里杀掉驱动进程：

```bash
fuser -k /dev/myserial        # 杀掉占用底盘串口的进程
```

> 别用 `pkill -f Ackman_driver_R2`：如果你当前 shell 的命令行里也含这个字符串，
> pkill 会把自己那条命令一起匹配上。

5. 手动发零速度（注意 `linear.y` 给个非零小量，`0` 不会让前轮回正）：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.001, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 8. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `/joy` 没有数据 | 接收器没插 / 手柄没开机 / 没电 | 查 `ls -l /dev/input/js*`，重新插接收器 |
| `/joy` 有数据但车不动 | **没解锁** | 按 Start，看 `/JoyState` 是否 `true` |
| 已解锁但仍不动 | `Ackman_driver_R2` 没起来 / 串口被占 | `fuser -v /dev/myserial`，`ros2 node list` |
| **按 Start 车会叫** | 按到的键实际是 `buttons[11]`（蜂鸣器），不是 `buttons[9]` | 按第 6 节核对索引；这同时意味着解锁没生效 |
| 按键全部没反应 | 用了 `R2_joy_ctrl.launch.py`（多机版） | 改用 `laser_bringup_tg_launch.py` |
| 车一直叫停不下来 | 蜂鸣器被锁在开的状态 | 起驱动后 `ros2 topic pub --once /Buzzer std_msgs/msg/Bool "{data: false}"`，或断电 |
| 推杆没反应、要推很大才有 | 死区 0.2 | 正常现象 |
| 一推转向就猛打 | `linear.y` 量纲问题（见坑 3） | 架起车确认后再落地 |
| 后退/转向方向相反 | 摇杆方向与代码假设不同 | 核对 `axes[1]` / `axes[2]` 的正负，必要时在 launch 里改映射 |

---

## 9. 安全清单（落地前逐条确认）

- [ ] 电压 ≥ 11.0 V（当前 12.2 V）
- [ ] `/dev/myserial` 存在，且没有其它进程占用
- [ ] 只跑了一套底盘驱动（没有同时跑建图/定位/差速）
- [ ] 上电前手柄已开机，`/joy` 有数据
- [ ] 已按 Start 解锁，`/JoyState = true`（**这样才知道上锁键是哪个**）
- [ ] 已经试过"再按一次 Start 能在 0.5 s 内停车"
- [ ] 速度档已降到 1/4
- [ ] 场地清空，人手可以随时断电
- [ ] 第一次测试**四轮架空**

> ⚠️ 历史事故：2026-08-29 用 PS2 遥控时后轮突然自行转动、向后跑，
> 手柄和电脑指令全部失效，只能断电；之后底盘板串口一直枚举不到。
> 上述清单就是为了避免重演。
