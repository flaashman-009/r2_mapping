# 里程计调参手册（自己动手版）

这份文档的目的是：**让你不依赖我，自己改参数、自己看效果**。

对应节点：`src/r2_mapping_tools/r2_mapping_tools/r2_odom.py`

---

## 1. 为什么要有这个节点

原厂 `base_node_R2.cpp` 的航向模型是：

```cpp
steer_angle = vel_raw.linear.y            // 底盘反馈的转向角（度）
R      = wheelbase / tan(steer_angle)
omega  = v / R
heading += omega * dt
x += v * cos(heading) * dt
y += v * sin(heading) * dt
```

三个问题：

1. `wheelbase` 出厂默认 **0.25**，而 R2 实测轴距 **0.2681**（差 7.24%），
   且**任何 launch 都没设过** → 每次转弯多转 7.24%
2. **完全不看左右轮速度差**。这台车右后轮比左轮快 1.6–2.6%，
   转向角为 0 时车其实在弯，但里程计认为在走直线
3. 用矩形法积分（`x += v·cosθ·dt`），不如中点法准

`r2_odom` 保留阿克曼几何，补上两个可标定项：

```
delta_eff = delta − steer_zero_deg              # 转向零位偏置
omega = v × tan(delta_eff) / wheelbase          # 阿克曼几何
      + v × yaw_bias_per_m                      # 轮速不对称的额外偏航
```

`yaw_bias_per_m` 随速度线性变化，正好对应"左右轮速度差 ∝ 速度"这一物理事实。

---

## 2. 切换与恢复

### 切到实验节点

```bash
bash ~/r2_mapping/scripts/start_odom_experimental.sh
```

它会自动：
1. 停掉原厂 `base_node_R2`
2. 启动 `r2_odom`（前台运行，看得到日志）

**前提**：硬件（底盘 + 雷达）已经在跑。没有的话先
`bash ~/r2_mapping/scripts/restart_hardware.sh`。

### 恢复原厂

```bash
# 先 Ctrl-C 停掉实验节点，然后
bash ~/r2_mapping/scripts/restart_hardware.sh
```

重启硬件会把原厂 `base_node_R2` 一起带起来。

### 只停实验节点

```bash
bash ~/r2_mapping/scripts/start_odom_experimental.sh --stop
```

---

## 3. 四个可调参数

启动时会打印当前值：

```
[r2_odom]:   轴距 0.2681 m | 转向零位 +0.00° | 额外偏航 +0.0000 rad/m | 尺度 1.0000
```

### 3.1 `wheelbase` —— 轴距

| | |
|---|---|
| 默认 | `0.2681` |
| 单位 | 米 |
| 怎么定 | **实测值**：前轮中心到后轮中心的水平距离 |

已经填对了。除非你换了车架，否则不用动。

### 3.2 `linear_scale` —— 距离尺度

| | |
|---|---|
| 默认 | `1.0` |
| 单位 | 无量纲 |
| 怎么定 | `linear_scale = 卷尺实测距离 / 里程计读数` |

**标定步骤**：

1. 在前轮位置的地板上贴一条胶带
2. 记下当前里程计读数：
   ```bash
   ros2 topic echo /odom_raw --once --field pose.pose.position
   ```
3. **手推车走 1 米左右**（走直线，车轮要滚动不能拖）
4. 再读一次，算 Δx
5. 贴第二条胶带，用卷尺量距离
6. ```bash
   python3 -c "print('scale =', round(0.98/1.05, 4))"   # 换成你的两个数
   ```
7. 应用：
   ```bash
   ros2 param set /r2_odom linear_scale 0.9333
   ```

### 3.3 `yaw_bias_per_m` —— 额外偏航 ⭐ 重点

| | |
|---|---|
| 默认 | `0.0` |
| 单位 | **弧度/米** |
| 物理含义 | 车每往前走 1 米，因为左右轮速差而额外转过的角度 |
| 符号 | 往**左**转为正（逆时针） |

**这是补偿"机械原因走不直"的关键参数。**

**标定步骤**：

1. 看着 `r2_odom` 的日志，每 5 秒打一行：
   ```
   [r2_odom] 路程  10.35 m | x +10.2 y +0.8 | odom 航向 +0.0° | IMU 航向 +5.8° | 差 +5.8°
   ```
   - `路程` = 里程计累计走过的距离
   - `odom 航向` = 里程计算出来的航向
   - `IMU 航向` = IMU 测到的真实航向（**这才是真值**）
   - `差` = 两者之差

2. **遥控直行**（转向摇杆保持居中），走 10 米左右，**尽量走直线**

3. 读日志里的 `路程` 和 `差`：
   ```
   yaw_bias_per_m = 差(弧度) / 路程(米)
   ```

   例：路程 10.35 m，差 +5.8°
   ```bash
   python3 -c "import math; print('yaw_bias_per_m =', round(math.radians(5.8)/10.35, 4))"
   # → 0.0098
   ```

4. 应用：
   ```bash
   ros2 param set /r2_odom yaw_bias_per_m 0.0098
   ```

5. **再走一次验证**：这次 `差` 应该接近 0。

### 3.4 `steer_zero_deg` —— 转向零位偏置

| | |
|---|---|
| 默认 | `0.0` |
| 单位 | 度 |
| 怎么定 | 让车**前轮摆正**，读 `/vel_raw` 的 `linear.y` |

```bash
ros2 topic echo /vel_raw --once --field linear.y
```

如果车摆正时反馈是 `-2.9`，就填 `-2.9`。

**注意**：这是"机械直行时舵机的读数"，**不是**你手动加的 `steer_trim`。

---

## 4. 调参的三个命令

```bash
# 看所有参数和当前值
ros2 param list /r2_odom
ros2 param get /r2_odom yaw_bias_per_m

# 改一个（立刻生效，不用重启）
ros2 param set /r2_odom yaw_bias_per_m 0.0098

# 看日志里的实时对比
# （r2_odom 是前台运行的，日志直接打在终端上）
```

---

## 5. 推荐的调参顺序

```
1. wheelbase        → 确认是 0.2681（已填好）
2. steer_zero_deg   → 车摆正，读 /vel_raw.linear.y，填进去
3. yaw_bias_per_m   → 直行 10 m，看日志的"差"，算出来填进去
4. linear_scale     → 推车 1 m，用卷尺量，算出来填进去
5. 重跑一次建图，对比效果
```

**一次只改一个参数**，改完走一段看日志，确认方向对了再改下一个。
同时改多个会分不清是哪个起的作用。

---

## 6. 怎么判断改对了

| 现象 | 说明 |
|---|---|
| 直行时日志的"差"接近 0 | `yaw_bias_per_m` 标定正确 ✅ |
| 推车 1 米，里程计读数 ≈ 卷尺实测 | `linear_scale` 标定正确 ✅ |
| 转弯时地图不再被"拧" | 整体模型对了 ✅ |

**注意**：`yaw_bias_per_m` 是让**里程计如实反映车的弯曲**，
不是让车走直。**车该弯还是弯**——你要的是"里程计知道它在弯"，
这样 SLAM 的运动先验才和现实一致。

想让车走直是**另一个问题**（机械修，或者控制器加 trim），别混在一起。

---

## 7. 参数存在哪

目前只能**运行时改**（重启就回默认）。要固化下来，改文件里的默认值：

```
src/r2_mapping_tools/r2_mapping_tools/r2_odom.py
```

找到这一段：

```python
self.declare_parameter("wheelbase", 0.2681)
self.declare_parameter("steer_zero_deg", 0.0)
self.declare_parameter("yaw_bias_per_m", 0.0)
self.declare_parameter("linear_scale", 1.0)
```

把括号里的数字改成标定值即可。

