#!/usr/bin/env python3
"""把 nav_watch CSV 画成诊断图。"""

import csv
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 中文字体（Windows）
for _f in ("Microsoft YaHei", "SimHei", "DengXian"):
    try:
        matplotlib.rcParams["font.sans-serif"] = [_f]
        matplotlib.rcParams["axes.unicode_minus"] = False
        break
    except Exception:
        pass


def main(argv):
    csv_path, png = argv[0], argv[1]
    rows = list(csv.DictReader(open(csv_path, encoding="utf-8")))
    g = lambda r, k: float(r[k])
    t = [g(r, "t") for r in rows]
    t0 = t[0]
    tt = [x - t0 for x in t]

    def series(k):
        return [g(r, k) for r in rows]

    def degs(k):
        return [math.degrees(v) for v in series(k)]

    fig, ax = plt.subplots(4, 1, figsize=(15, 13), sharex=True)

    # 1 位置
    ax[0].plot(tt, series("odom_x"), label="odom x", lw=1.2)
    ax[0].plot(tt, series("odom_y"), label="odom y", lw=1.2)
    ax[0].plot(tt, series("amcl_x"), label="amcl x", lw=1.2, alpha=.8)
    ax[0].plot(tt, series("amcl_y"), label="amcl y", lw=1.2, alpha=.8)
    ax[0].axvspan(84, 215, color="orange", alpha=.12)
    ax[0].set_ylabel("position (m)")
    ax[0].legend(fontsize=8, ncol=2)
    ax[0].grid(alpha=.3)
    ax[0].set_title("(1) 位置：odom vs AMCL", fontsize=10, loc="left")

    # 2 航向
    ax[1].plot(tt, degs("odom_yaw"), label="odom yaw (EKF)", lw=1.3)
    ax[1].plot(tt, degs("amcl_yaw"), label="amcl yaw", lw=1.3, alpha=.8)
    ax[1].plot(tt, degs("imu_yaw"), label="imu yaw (madgwick)", lw=1, alpha=.5)
    ax[1].axvspan(84, 215, color="orange", alpha=.12, label="行驶时段")
    ax[1].set_ylabel("yaw (deg)")
    ax[1].legend(fontsize=8, ncol=2)
    ax[1].grid(alpha=.3)
    ax[1].set_title("(2) 航向：三条曲线基本平行 → 航向源没跳，是整体位姿被带偏",
                    fontsize=10, loc="left")

    # 3 残差
    ax[2].plot(tt, series("res_med"), label="残差中位数", lw=1.4)
    ax[2].plot(tt, series("res_p90"), label="残差 p90", lw=1, alpha=.6)
    ax[2].axhline(0.25, ls="--", lw=.9, color="orange")
    ax[2].axhline(0.5, ls="--", lw=.9, color="red")
    ax[2].axvspan(84, 215, color="orange", alpha=.12)
    ax[2].set_ylabel("scan-map residual (m)")
    ax[2].legend(fontsize=8)
    ax[2].grid(alpha=.3)
    ax[2].set_title("(3) 点云-地图残差：静止时=0，一发目标就往上涨（橙/红虚线=警戒线）",
                    fontsize=10, loc="left")

    # 4 转向指令 vs 实际转角
    ax3 = ax[3]
    ax3.plot(tt, series("cmd_w"), label="cmd angular.z (TEB 输出)", lw=1, color="C0")
    ax3.plot(tt, [v / 57.3 for v in series("vel_steer")],
             label="实际转角 δ (度→rad 换算)", lw=1.2, color="C3", alpha=.85)
    ax3.axvspan(84, 215, color="orange", alpha=.12)
    ax3.set_ylabel("rad")
    ax3.set_xlabel("t (s)")
    ax3.legend(fontsize=8, loc="upper left")
    ax3.grid(alpha=.3)
    ax3.set_title("(4) 铁证：实际转角 ≈ angular.z × 57.3 → 底盘把 angular.z 当"
                  "\"弧度制转向角\"执行，而 TEB 发的是角速度 ω", fontsize=10, loc="left")

    fig.tight_layout()
    fig.savefig(png, dpi=100)
    print("saved", png)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
