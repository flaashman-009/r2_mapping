#!/usr/bin/env python3
"""判定底盘把 angular.z 当什么用：转向角？角速度？

把 /cmd_vel.angular.z 分箱，看底盘反馈的实际转角 vel_steer 跟着怎么变。
如果 vel_steer ≈ angular.z × 57.3（弧度转角度），说明底盘把 angular.z
当成"弧度制的转向角"直接用 —— 而 TEB 发的是角速度 ω。
"""

import csv
import statistics
import sys


def main(argv=None):
    path = argv[0] if argv else "logs/nav_watch_0916_220239.csv"
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    mv = [r for r in rows if abs(float(r["cmd_vx"])) > 0.005]
    if not mv:
        print("没有运动数据")
        return 1

    print("=" * 60)
    print(" angular.z 指令 -> 底盘实际转角（运动期间 {} 帧）".format(len(mv)))
    print("=" * 60)
    bins = {}
    for r in mv:
        w = float(r["cmd_w"])
        s = float(r["vel_steer"])
        if abs(w) < 0.05:
            continue
        bins.setdefault(round(w, 1), []).append(s)
    print("%10s %8s %12s %12s" % ("cmd_w", "样本", "实测转角中位", "w*57.3"))
    for k in sorted(bins):
        v = bins[k]
        print("%10.1f %8d %12.1f %12.1f" % (k, len(v), statistics.median(v),
                                            k * 57.2958))

    W = [float(r["cmd_w"]) for r in mv if abs(float(r["cmd_w"])) > 0.05]
    S = [float(r["vel_steer"]) for r in mv if abs(float(r["cmd_w"])) > 0.05]
    n = len(W)
    if n > 1:
        mw, ms = sum(W) / n, sum(S) / n
        den = sum((x - mw) ** 2 for x in W)
        a = sum((x - mw) * (y - ms) for x, y in zip(W, S)) / den if den else 0
        print("\n线性拟合  vel_steer = a * cmd_w:")
        print("    a = {:.2f}".format(a))
        print("    （57.3 = 底盘把 angular.z 按'弧度制转向角'直接用）")

    sat = sum(1 for r in mv if abs(float(r["vel_steer"])) > 40)
    print("\n转向饱和：实际转角 |δ| > 40°（机械极限 ±44°）的帧 "
          "{}/{} = {:.1f}%".format(sat, len(mv), 100.0 * sat / len(mv)))

    chg = sum(1 for a_, b_ in zip([float(r["cmd_w"]) for r in mv],
                                  [float(r["cmd_w"]) for r in mv][1:])
              if abs(b_ - a_) > 1e-3)
    flip = sum(1 for a_, b_ in zip([float(r["cmd_w"]) for r in mv],
                                   [float(r["cmd_w"]) for r in mv][1:])
               if a_ * b_ < -1e-6)
    span = float(mv[-1]["t"]) - float(mv[0]["t"])
    print("\nangular.z 变化 {} 次 / {:.0f} 秒 = {:.2f} 次/秒，符号翻转 {} 次".format(
        chg, span, chg / span, flip))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
