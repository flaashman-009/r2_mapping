#!/usr/bin/env python3
"""LaserScan 统计分析：判断雷达是否扫到自己车身。

用途：导航中途停车/中止，最常见的元凶之一就是**车体自遮挡**——
雷达扫到自己车身，局部代价地图在车周围标出一圈障碍，
控制器因此找不到任何可行轨迹。

本工具会给出：
  - 有效/无效光束数量
  - 有效距离的 min / 中位数 / mean / max
  - 近距离返回（默认 <0.35 m）的占比
  - 这些近点按 30° 分桶的分布 ← **关键**：
      如果集中在某几个固定扇区，就是车体自遮挡；
      如果均匀散布，那多半是环境里的近处物体。

用法：
    python3 scan_stats.py
    python3 scan_stats.py --topic /scan --near 0.4 --samples 5
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanStats(Node):

    def __init__(self, topic, near, samples):
        super().__init__("scan_stats")
        self.near = near
        self.need = samples
        self.count = 0
        self.scan = None
        self.done = False
        # 雷达多数是 BEST_EFFORT 发布
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(LaserScan, topic, self._on_scan, qos)
        self.get_logger().info(
            "订阅 {}，采集 {} 帧后出报告".format(topic, samples)
        )

    def _on_scan(self, msg):
        self.scan = msg
        self.count += 1
        if self.count >= self.need:
            self.done = True

    def report(self):
        msg = self.scan
        n = len(msg.ranges)
        valid = []
        near_angles = []
        angle = msg.angle_min

        for value in msg.ranges:
            if value is not None and math.isfinite(value) and value > 0.0:
                valid.append(value)
                if value < self.near:
                    near_angles.append(angle)
            angle += msg.angle_increment

        print("")
        print("=" * 60)
        print(" LaserScan 统计")
        print("=" * 60)
        print("  frame_id      : {}".format(msg.header.frame_id))
        print("  光束数        : {}（angle_increment {:.5f} rad）".format(
            n, msg.angle_increment))
        print("  角度范围      : {:.1f}° ~ {:.1f}°".format(
            math.degrees(msg.angle_min), math.degrees(msg.angle_max)))
        print("  量程          : {:.2f} ~ {:.2f} m".format(msg.range_min, msg.range_max))
        print("")
        print("  有效点        : {} / {}  ({:.1f}%)".format(
            len(valid), n, 100.0 * len(valid) / max(1, n)))
        print("  无效点        : {}（inf / nan / <=0）".format(n - len(valid)))

        if not valid:
            print("\n  没有任何有效点，检查雷达本身。")
            return

        valid.sort()
        mean_v = sum(valid) / len(valid)
        median_v = valid[len(valid) // 2]
        print("")
        print("  最小距离      : {:.3f} m".format(valid[0]))
        print("  中位距离      : {:.3f} m".format(median_v))
        print("  平均距离      : {:.3f} m".format(mean_v))
        print("  最大距离      : {:.3f} m".format(valid[-1]))

        near_ratio = 100.0 * len(near_angles) / len(valid)
        print("")
        print("  近点 (<{:.2f} m) : {} 个，占有效点的 {:.1f}%".format(
            self.near, len(near_angles), near_ratio))

        if near_angles:
            buckets = {}
            for a in near_angles:
                deg = math.degrees(a)
                b = int(math.floor(deg / 30.0)) * 30
                buckets[b] = buckets.get(b, 0) + 1
            print("  近点的角度分布（每 30° 一桶，0° = 车头方向）：")
            for b in sorted(buckets):
                bar = "#" * min(40, buckets[b] // 2 + 1)
                print("    {:>5}° ~ {:>4}° : {:>4}  {}".format(
                    b, b + 30, buckets[b], bar))

            if near_ratio > 5.0:
                print("")
                print("  ⚠️ 近点比例偏高。如果它们集中在固定扇区（尤其是车尾/两侧），")
                print("     就是**车体自遮挡**——请用 r2_mapping_perception/scan_filter_node")
                print("     屏蔽对应扇区，否则代价地图会在车周围刷一圈假障碍，")
                print("     导航会停在半路。")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="LaserScan 统计（车体自遮挡诊断）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--near", type=float, default=0.35,
                        help="近距离阈值，米")
    parser.add_argument("--samples", type=int, default=3)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = ScanStats(args.topic, args.near, args.samples)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.scan is not None:
            node.report()
        else:
            print("没有收到任何 {} 数据。".format(args.topic))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
