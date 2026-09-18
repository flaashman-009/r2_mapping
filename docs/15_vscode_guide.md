# 在 VS Code 里看这个工程

> 工程在 Windows 上：`C:\Users\zhuchenju\Desktop\建图\r2_mapping`
> 小车上的镜像：`/home/jetson/r2_mapping`

---

## 1. 打开工程

```powershell
cd C:\Users\zhuchenju\Desktop\建图\r2_mapping
code .
```

`code .` 会用 VS Code 打开**整个工程**（左侧资源管理器能看到所有目录）。

---

## 2. 推荐装的扩展

| 扩展 | 作用 |
|---|---|
| **Python** | 语法高亮、调试 |
| **Pylance** | 类型检查、跳转定义 |
| **YAML** | 编辑 `nav_r2.yaml` 时校验缩进（**这个最重要**——YAML 缩进错一个空格，整个参数会静默失效） |
| ROS（可选） | 消息 / launch 支持 |

### 关于 Pylance 报错

如果看到 `无法解析导入 Rosmaster_Lib` 或 `无法解析导入 rclpy`：

**这是正常的。** `Rosmaster_Lib` 只装在小车上，`rclpy` 只在 ROS 环境里，
Windows 上都没有。不影响阅读和编辑代码。

---

## 3. 看代码的顺序（推荐）

### 第一步：总体设计

```
docs/14_architecture.md     数据流图 + 文件职责表 + 已知的坑
docs/10_before_after.md     改过什么、为什么改
```

### 第二步：两个"非标准"节点（本工程的核心）

这两个节点是本工程自己写的，也是解决问题的关键：

```
scripts/cmd_vel_ackermann.py      转向适配（ω → 前轮转向角）
    _on_cmd        接收 /cmd_vel_nav
    _tick          算 δ、限幅、限速率，发 /cmd_vel
    _on_halt       定位护卫叫停时的处理

scripts/localization_guard.py     定位护卫（协方差太大就停车）
    _on_amcl       读协方差
    _tick          判断该停车还是该解除
    _halt          发 /cmd_vel_halt
```

### 第三步：导航启动流程

```
src/r2_mapping_bringup/launch/navigation_r2.launch.py
```

注意里面的 `remappings`：**Nav2 的输出被改到 `/cmd_vel_nav`**，
再由 `cmd_vel_ackermann` 转成底盘认的 `/cmd_vel`。这是本工程最特别的地方。

### 第四步：参数

```
config/nav_r2.yaml
    amcl                    定位参数
    controller_server       TEB 控制器 + 到位判定
    local_costmap / global_costmap   代价地图 + 车体尺寸
    cmd_vel_ackermann       ★ 转向限幅全部在这里
```

### 第五步：运维脚本

```
scripts/nav.sh          一键启动 / 停止 / 状态
scripts/env.sh          所有脚本都 source 它（环境变量 + 两个原厂工作区）
scripts/estop.sh        急停
scripts/nav_watch.py    行车记录仪（排查问题的第一工具）
```

---

## 4. 目录速览

```
r2_mapping/
├── README.md                  从这里开始
├── config/                    参数（改参数主要在这）
├── docs/                      文档 00 ~ 15
├── maps/                      生成的地图
├── bags/                      录制的数据
├── logs/                      记录仪 CSV + 诊断图
├── scripts/                   运维入口 + 运行时节点
├── tools/analysis/            电脑端分析工具（不依赖 ROS）
└── src/                       ROS2 包
    ├── r2_mapping_bringup/    launch + rviz
    ├── r2_mapping_perception/ 扫描滤波节点
    └── r2_mapping_tools/      地图评估、里程计标定、手柄遥控
```

---

## 5. 常用操作

### 搜索某个参数在哪儿

`Ctrl + Shift + F` 全局搜索，例如搜 `update_min_d`：

```
config/nav_r2.yaml         定义（改这里）
docs/14_architecture.md    说明为什么这么设
```

### 跳到某个函数

`Ctrl + P` 打开文件，再 `Ctrl + Shift + O` 跳函数。

### 看某次跑的记录

在 VS Code 的终端里：

```powershell
cd C:\Users\zhuchenju\Desktop\建图\r2_mapping
python .\tools\analysis\analyze_nav_watch.py .\logs\nav_watch_0918_100241.csv --png .\logs\diag.png
```

---

## 6. 改完代码怎么同步到小车

这个工程**没有用 git**（和 `car_drive_mode_switch` 不同），同步方式是 scp：

```powershell
scp .\config\nav_r2.yaml jetson@192.168.43.16:~/r2_mapping/config/
scp .\scripts\cmd_vel_ackermann.py jetson@192.168.43.16:~/r2_mapping/scripts/
```

改哪个传哪个，避免覆盖车上别的改动。

**传完一定要在车上验证语法**，否则下次启动才发现问题：

```bash
python3 -m py_compile ~/r2_mapping/scripts/cmd_vel_ackermann.py   # Python 文件
bash -n ~/r2_mapping/scripts/nav.sh                              # shell 脚本
bash ~/r2_mapping/scripts/verify_changes.sh                      # 参数文件
```

---

## 7. Windows / 小车 的分工

| 在哪做 | 做什么 |
|---|---|
| **Windows + VS Code** | 改代码、改参数、看文档、分析 CSV |
| **小车终端** | 启动、跑车、看实时日志 |
| **RViz（小车屏幕）** | 发目标、看地图和点云 |

**注意**：launch 文件里用的是**绝对路径**（`~/r2_mapping/...`），
所以文件传到小车的对应位置就行，**不需要 colcon 编译**。

例外：`src/r2_mapping_tools/` 和 `src/r2_mapping_perception/` 里有些是
按包名调用的（`ros2 run r2_mapping_tools ps2_teleop`），那些需要 colcon build，
或者直接用文件路径跑。
