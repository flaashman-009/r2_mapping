#!/usr/bin/env python3
"""前轮舵机"一直修方向"的诊断工具。

现象：导航时前轮舵机持续小幅来回动，像 F1 暖胎。

本工具采一段数据，把三个可能的原因一次查清：

  1. **几个节点在发 /cmd_vel？**  ← 最常见
     两个发布者（比如 ps2_teleop 和导航控制器）会互相打架，
     舵机在两边来回跳。

  2. **转向指令本身在抖吗？**
     看 linear.y 的变化次数和幅度。如果指令在快速小幅变化，
     那是局部规划器（TEB 尤其明显）造成的。

  3. **实际转角跟得上指令吗？**
     对比 /cmd_vel.linear.y（指令）和 /vel_raw.linear.y（实际角度）。
     两者趋势一致 → 舵机在忠实执行（问题在指令侧）
     指令稳定但舵机乱动 → 机械/固件问题

用法（**导航跑起来、车在动的时候**执行）：

    python3 steer_debug.py
    python3 steer_debug.py --duration 30
    python3 steer_debug.py --stationary     # 车静止时测（看定位有没有抖）

也可以直接：
    ros2 run r2_mapping_tools steer_debug
"""

import argparse
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class SteerDebug(Node):

    def __init__(self):
        super().__init__("steer_debug")
        self.cmd_samples = []      # (t, linear.x, linear.y)
        self.vel_samples = []      # (t, 实际转角 deg)
        self.amcl_xy = []          # (t, x, y)
        self.t0 = time.monotonic()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 50)
        self.create_subscription(Twist, "/vel_raw", self._on_vel, qos)
        self.create_subscription(Odometry, "/amcl_pose", self._on_amcl, qos)
        self.create_subscription(Twist, "/odom", self._on_odom, qos)
        self.odom_xy = []

    def _stamp(self):
        return time.monotonic() - self.t0

    def _on_cmd(self, msg):
        self.cmd_samples.append((self._stamp(), msg.linear.x, msg.linear.y))

    def _on_vel(self, msg):
        self.vel_samples.append((self._stamp(), msg.linear.y))

    def _on_amcl(self, msg):
        p = msg.pose.pose.position
        self.amcl_xy.append((self._stamp(), p.x, p.y))

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_xy.append((self._stamp(), p.x, p.y))


def count_changes(values, thresh):
    n = 0
    for a, b in zip(values, values[1:]):
        if abs(b - a) > thresh:
            n += 1
    return n


def jitter(xy):
    """返回位置抖动量（相邻采样位移的中位数，米）。"""
    ds = []
    for (_, x0, y0), (_, x1, y1) in zip(xy, xy[1:]):
        ds.append(math.hypot(x1 - x0, y1 - y0))
    if not ds:
        return 0.0
    return statistics.median(ds)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="前轮舵机抖动的诊断",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--duration", type=float, default=20.0, help="采样时长（秒）")
    parser.add_argument("--stationary", action="store_true",
                        help="车静止时测（只看定位抖动）")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = SteerDebug()

    # ---------- 1. /cmd_vel 的发布者 ----------
    print("")
    print("=" * 62)
    print(" 前轮舵机抖动诊断")
    print("=" * 62)
    print("")
    print("[1] /cmd_vel 的发布者")
    try:
        infos = node.get_publishers_info_by_topic("/cmd_vel")
    except Exception as exc:
        infos = []
        print("    查询失败：{}".format(exc))
    if not infos:
        print("    （没有发布者 —— 导航/遥控没在跑？）")
    for info in infos:
        ns = info.node_namespace.rstrip("/")
        print("    - {}{}".format(ns, info.node_name))
    n_pub = len(infos)
    print("    共 {} 个".format(n_pub))
    if n_pub >= 2:
        print("")
        print("    >>> 两个以上发布者！这就是舵机来回跳的直接原因。")
        print("        关掉多余的那个（通常是 ps2_teleop：按解锁键上锁）")
    print("")

    # ---------- 2. 采样 ----------
    print("[2] 采样 {:.0f} 秒 ...".format(args.duration))
    end = time.monotonic() + args.duration
    last = time.monotonic()
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now - last >= 5.0:
            print("    {}s  cmd={} vel_raw={}".format(
                int(now - (end - args.duration)),
                len(node.cmd_samples), len(node.vel_samples)))
            last = now
    print("")

    # ---------- 3. 转向指令 ----------
    print("[3] 转向指令 /cmd_vel.linear.y")
    if len(node.cmd_samples) < 10:
        print("    数据太少（{} 帧）".format(len(node.cmd_samples)))
        print("    → 没有节点在发 /cmd_vel，或者发得太稀疏")
    else:
        ys = [s[1] for s in node.cmd_samples]
        t = [s[0] for s in node.cmd_samples]
        span = max(1e-6, t[-1] - t[0])
        chg = count_changes(ys, 1e-4)
        print("    帧数      : {}".format(len(ys)))
        print("    范围      : {:+.4f} ~ {:+.4f}".format(min(ys), max(ys)))
        print("    标准差    : {:.5f}".format(statistics.pstdev(ys)))
        print("    变化次数  : {}  （{:.1f} 次/秒）".format(chg, chg / span))
        rate = chg / span
        if rate > 5:
            print("    → 指令在**快速抖动**（局部规划器造成的）")
        elif rate > 0.5:
            print("    → 指令在**缓慢调整**（正常跟踪路径）")
        else:
            print("    → 指令**基本静止**")
    print("")

    # ---------- 4. 实际转角 ----------
    print("[4] 实际转角 /vel_raw.linear.y（单位：度）")
    if len(node.vel_samples) < 10:
        print("    数据太少（{} 帧）".format(len(node.vel_samples)))
    else:
        vs = [s[1] for s in node.vel_samples]
        t = [s[0] for s in node.vel_samples]
        span = max(1e-6, t[-1] - t[0])
        chg = count_changes(vs, 0.5)
        print("    帧数      : {}".format(len(vs)))
        print("    范围      : {:+.1f}° ~ {:+.1f}°".format(min(vs), max(vs)))
        print("    变化次数  : {}  （{:.1f} 次/秒）".format(chg, chg / span))
    print("")

    # ---------- 5. 定位抖动 ----------
    print("[5] 定位稳定性")
    if len(node.amcl_xy) >= 10:
        j = jitter(node.amcl_xy)
        print("    /amcl_pose 每帧位移中位数: {:.4f} m".format(j))
        if j > 0.05:
            print("    → 定位在抖（每帧跳 {:.1f} cm）".format(j * 100))
        else:
            print("    → 定位稳定")
    else:
        print("    /amcl_pose 没数据（{} 帧）—— 导航没在跑，或没设初始位姿".format(
            len(node.amcl_xy)))
    if len(node.odom_xy) >= 10:
        j = jitter(node.odom_xy)
        print("    /odom      每帧位移中位数: {:.4f} m".format(j))
    print("")

    # ---------- 6. 结论 ----------
    print("=" * 62)
    print(" 结论")
    print("=" * 62)
    if n_pub >= 2:
        print("  舵机来回跳的原因：**{} 个节点同时在发 /cmd_vel**".format(n_pub))
        print("  处理：关掉多余的那个（ps2_teleop 按解锁键上锁，或 Ctrl-C）")
    elif len(node.cmd_samples) >= 10:
        ys = [s[1] for s in node.cmd_samples]
        t = [s[0] for s in node.cmd_samples]
        rate = count_changes(ys, 1e-4) / max(1e-6, t[-1] - t[0])
        if rate > 5:
            print("  指令只有 1 个发布者，但它在快速抖动 → **局部规划器造成**")
            print("  TEB 每个控制周期重新优化轨迹，转向输出天然抖。")
            print("  处理：换 DWB（navigation_dwa_launch.py），或调 TEB 参数")
        elif rate > 0.5:
            print("  指令在正常范围内调整，属于跟踪路径的正常行为。")
            print("  如果实际转角抖动明显大于指令，才是舵机/机械问题。")
        else:
            print("  指令稳定。如果舵机还在动，问题在**舵机或固件**，不在指令侧。")
    else:
        print("  没有采集到 /cmd_vel 数据 —— 导航/遥控可能没在跑。")
        print("  请在**车正在动的时候**再跑一次这个工具。")
    print("")

    try:
        node.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

