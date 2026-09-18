# 电脑端分析工具

这些脚本**不依赖 ROS**，在 Windows 或任意装了 Python 的机器上跑。
输入是 `logs/` 里的记录 CSV 或 `maps/` 里的 PGM/YAML。

## 依赖

    pip install numpy matplotlib pyyaml

只用 `map_ascii.py` / `pgm_to_png.py` 的话，`numpy + matplotlib` 就够。
`pose_vs_map.py` / `pose_distance_to_known.py` 需要 `numpy`——
距离变换是自实现的，**不需要 scipy**（Windows 上装 scipy 经常出问题）。

## 工具清单

| 脚本 | 用途 |
|---|---|
| `analyze_nav_watch.py` | 一次跑完的记录体检：运动区间、转向平滑度、残差趋势、跳变统计 |
| `deep_nav_report.py` | 深入分析：残差异常段、协方差分布、map→odom 跳变幅度、航向累计漂移 |
| `plot_nav.py` | 把记录画成四联图（位置 / 航向 / 残差 / 指令） |
| `steer_channel.py` | 判定底盘把 `angular.z` 当什么用（转向角？角速度？） |
| `pose_vs_map.py` | 位姿落在"空闲 / 未知 / 障碍"哪种格子上 |
| `pose_distance_to_known.py` | **更准的版本**：位姿到最近"已建图区域"的实际距离 |
| `map_ascii.py` | 把地图降采样成字符画，快速看结构 |
| `pgm_to_png.py` | 把地图渲染成 PNG（白=空闲 黑=障碍 灰=未知） |

## 典型用法

    # 一次跑完的完整体检 + 出图
    python analyze_nav_watch.py logs/nav_watch_0918_100241.csv --png logs/diag.png

    # 深入分析（看崩溃点）
    python deep_nav_report.py logs/nav_watch_0918_100241.csv

    # 位姿到底在不在已建图区域内
    python pose_distance_to_known.py logs/nav_watch_0918_100241.csv maps/room_05.yaml

    # 把地图渲染出来看
    python pgm_to_png.py ../maps/room_05.yaml

## 记录数据从哪来

车上的记录仪 `scripts/nav_watch.py` 会把数据写到
`~/r2_mapping/logs/nav_watch_<日期时间>.csv`，拷回电脑再分析：

    scp jetson@<车IP>:~/r2_mapping/logs/nav_watch_0918_100241.csv .\logs\

## 两个关键概念（看图时会用到）

**点云残差** —— 把激光的每个点按当前位姿投到地图上，看点离最近的墙有多远。

    静止且位姿正确    0.000 m   （精确为 0，是个很好的标定）
    正常行驶          0.05 ~ 0.20 m
    开始漂            > 0.35 m
    彻底迷失          > 0.5 m

**map→odom 跳变** —— AMCL 的修正量。单步跳 >1 m 说明 AMCL 把自己"瞬移"了，
这是定位崩溃的直接标志。
