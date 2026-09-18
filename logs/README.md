# logs/ —— 行车记录数据

这个目录里的文件**不进 git**（每次跑车都会产生，体积增长很快）。

## 里面会有什么

| 文件 | 产生者 | 内容 |
|---|---|---|
| `nav_watch_<日期>_<时间>.csv` | `scripts/nav_watch.py` | 8 路信号的完整时间序列（见下表） |
| `nav_diag*.png` | `tools/analysis/analyze_nav_watch.py` | 四联诊断图 |
| `watch.log` 等 | 各脚本 | 运行日志 |

## CSV 里有哪些列

| 列 | 含义 |
|---|---|
| `t` | 相对时间（秒） |
| `odom_x/y/yaw` | EKF 里程计（**也是 TF 用的那份**） |
| `raw_x/y/yaw` | 原厂里程计（对照） |
| `imu_yaw` | madgwick 姿态的 yaw（EKF 融合的就是它） |
| `board_yaw` | 底盘板载 yaw —— **这台车的驱动不发布 `/imu/yaw_deg`，所以恒为 0** |
| `amcl_x/y/yaw` | AMCL 输出的位姿 |
| `amcl_cov` | AMCL 协方差（前三个对角元之和）—— **定位可信度的直接指标** |
| `mo_x/y/yaw` | `map→odom` 变换（AMCL 的修正量） |
| `cmd_vx` | 下发的线速度 |
| `cmd_steer` | 下发的转向（`linear.y`，值×1000 = 度） |
| `cmd_w` | 下发的 `angular.z`（**恒为 0**，本工程不用它转向） |
| `vel_steer` | 底盘反馈的实际转角（度） |
| `res_med` / `res_p90` | **点云-地图残差**（中位数 / 90 分位，米） |
| `res_ok` / `res_far` | 参与统计的光束数 / 其中离障碍 >0.5m 的比例 |

## 两个最重要的判据

**`res_med`（点云残差）**

    静止且位姿正确    0.000 m   （精确为 0，很好的标定）
    正常行驶          0.05 ~ 0.20 m
    开始漂            > 0.35 m

**`amcl_cov`（协方差）**

    正常              0.05 ~ 0.5
    开始漂            几十
    迷失              100 ~ 400

## 怎么分析

```powershell
cd C:\Users\zhuchenju\Desktop\建图\r2_mapping
python .\tools\analysis\analyze_nav_watch.py .\logs\nav_watch_0918_100241.csv --png .\logs\diag.png
python .\tools\analysis\deep_nav_report.py .\logs\nav_watch_0918_100241.csv
```

## 想保留某一次的数据

```bash
git add -f logs/nav_watch_0918_100241.csv
```
