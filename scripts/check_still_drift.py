#!/usr/bin/env python3
"""静止漂移自检：车不动的时候，里程计会不会自己"走"。

为什么需要它（2026-09-19 实测）：
    车停住、cmd_vx = 0 的 350 秒里，原厂里程计认为车在原地绕半径 0.8 m
    的圈转了 6.4 圈，EKF 跟着漂了 31 米，AMCL 再跟着漂 ——
    表现就是"车停下来，点云突然对不上地图"。

    修法是在 /odom_raw 和 EKF 之间加了"里程计静止门"（odom_gate.py）。
    这个脚本用来验证它到底有没有效果。

用法（车保持完全静止，别碰它）：
    python3 check_still_drift.py            # 默认测 60 秒
    python3 check_still_drift.py --seconds 120
"""

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class StillDrift(Node):

    def __init__(self):
        super().__init__("still_drift")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=50)
        self.rec = {"odom": [], "gated": [], "raw": []}
        self.create_subscription(Odometry, "/odom",
                                 lambda m: self._add("odom", m), qos)
        self.create_subscription(Odometry, "/odom_gated",
                                 lambda m: self._add("gated", m), qos)
        self.create_subscription(Odometry, "/odom_raw",
                                 lambda m: self._add("raw", m), qos)

    def _add(self, key, msg):
        p = msg.pose.pose.position
        self.rec[key].append((p.x, p.y))


def spread(pts):
    """返回最大位移（相对第一帧）。"""
    if len(pts) < 2:
        return float("nan")
    x0, y0 = pts[0]
    return max(math.hypot(x - x0, y - y0) for x, y in pts)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = StillDrift()

    print("=" * 62)
    print(" 静止漂移自检")
    print("=" * 62)
    print(" ⚠️  接下来 {:.0f} 秒内**不要碰车**".format(args.seconds))
    print("")

    t0 = time.monotonic()
    last = t0
    while rclpy.ok() and time.monotonic() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if now - last >= 10.0:
            last = now
            print("   {:>3.0f}s   odom {:.2f} m | gated {:.2f} m | raw {:.2f} m".format(
                now - t0, spread(node.rec["odom"]),
                spread(node.rec["gated"]), spread(node.rec["raw"])))

    print("")
    print("--- 结果（{:.0f} 秒内最大位移）---".format(args.seconds))
    d_odom = spread(node.rec["odom"])
    d_gate = spread(node.rec["gated"])
    d_raw = spread(node.rec["raw"])
    print("  /odom_raw    {:.3f} m   （原厂里程计）".format(d_raw)
          if d_raw == d_raw else "  /odom_raw    无数据")
    print("  /odom_gated  {:.3f} m   （静止门之后）".format(d_gate)
          if d_gate == d_gate else "  /odom_gated  无数据")
    print("  /odom        {:.3f} m   （EKF 输出，也就是 TF 用的那份）".format(d_odom)
          if d_odom == d_odom else "  /odom        无数据")
    print("")
    print("--- 判读 ---")
    if d_odom != d_odom:
        print("  /odom 没有数据 —— EKF 没起来，或者 odom_gate 没跑")
    elif d_odom < 0.05:
        print("  ✅ 很好：车没动，/odom 也没动")
    elif d_odom < 0.5:
        print("  ✅ 可接受（修之前 350 秒漂 31 米，约合 60 秒 5.3 米）")
    elif d_odom < 2.0:
        print("  ⚠️ 还有残余漂移，把结果发我看")
    else:
        print("  ❌ 没起作用 —— 静止门可能没拦住（看上面 raw 和 gated 的对比）")
    print("")
    print("  对比看法：理论上 /odom_raw 会漂、/odom_gated 应该几乎不动。")
    print("  如果两个都漂一样多 → 静止门没生效。")

    try:
        node.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
