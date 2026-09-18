#!/usr/bin/env python3
"""LaserScan 滤波节点：量程裁剪 + 车体自遮挡扇区屏蔽。

为什么需要它：
  1. YDLIDAR 在车体/支架附近会有自遮挡，扫出一圈贴着小车走的假点，
     这些点在 SLAM 里被当成"墙"，会让地图糊成一团或者直接失配。
  2. 超出有效量程的远点噪声多，室内建图没必要保留。

默认**不启用**（mapping.launch.py 里 use_scan_filter:=false），
只有确认车体自遮挡严重时才开，避免多加一层延迟。

参数：
  input_topic    输入话题，默认 /scan
  output_topic   输出话题，默认 /scan_filtered
  frame_id       输出 header.frame_id 覆盖；留空表示保持原样
  min_range      小于该距离的点置为 inf，默认 0.15
  max_range      大于该距离的点置为 inf，默认 12.0
  blind_sectors  屏蔽扇区，角度制、逆时针为正、0° 为车头方向。
                 用逗号分隔多个扇区，例如 "120:150,-150:-120"。
                 默认空字符串。
                 （注意：这个参数声明为字符串而不是数组 —— 空数组会被
                   rclpy 推断成 BYTE_ARRAY，命令行传 STRING_ARRAY 会报
                   InvalidParameterTypeException。踩过这个坑。）
  angle_offset_deg 全局角度偏置（用于修正 LiDAR 安装朝向之外的微调），默认 0
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


def _parse_sectors(raw):
    """把 ['120:240', '-15:15'] / '120:240' / '[]' 统一成 [(120,240), ...]。"""
    if raw is None:
        return []
    if isinstance(raw, str):
        text = raw.strip()
        if not text or text == "[]":
            return []
        text = text.strip("[]")
        items = [item.strip().strip("'\"") for item in text.split(",")]
    else:
        items = []
        for entry in raw:
            if isinstance(entry, str):
                items.extend(
                    item.strip().strip("'\"") for item in entry.strip("[]").split(",")
                )
            else:
                items.append(entry)

    sectors = []
    for item in items:
        if not item:
            continue
        if isinstance(item, (list, tuple)) and len(item) == 2:
            start, end = float(item[0]), float(item[1])
        else:
            text = str(item).strip().strip("[]'\"")
            if ":" not in text:
                continue
            start_text, _, end_text = text.partition(":")
            try:
                start, end = float(start_text), float(end_text)
            except ValueError:
                continue
        if math.isclose(start, end):
            continue
        if end < start:
            start, end = end, start
        sectors.append((start, end))
    return sectors


class ScanFilter(Node):
    def __init__(self):
        super().__init__("scan_filter")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_filtered")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("min_range", 0.15)
        self.declare_parameter("max_range", 12.0)
        self.declare_parameter("blind_sectors", "")
        self.declare_parameter("angle_offset_deg", 0.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.angle_offset = math.radians(
            float(self.get_parameter("angle_offset_deg").value)
        )
        self.sectors = _parse_sectors(self.get_parameter("blind_sectors").value)
        self.sectors_rad = [
            (math.radians(start), math.radians(end)) for start, end in self.sectors
        ]

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub = self.create_publisher(LaserScan, self.output_topic, qos)
        self.sub = self.create_subscription(
            LaserScan, self.input_topic, self._on_scan, qos
        )

        self._frames = 0
        self._invalid = 0
        self._range_dropped = 0
        self._sector_masked = 0
        self._empty_frames = 0
        self._last_report = self.get_clock().now()

        self.get_logger().info(
            "scan_filter 启动：{} -> {}{} 量程 [{:.2f}, {:.2f}] m{}".format(
                self.input_topic,
                self.output_topic,
                "，frame_id -> " + self.frame_id if self.frame_id else "",
                self.min_range,
                self.max_range,
                "，屏蔽扇区 " + str(self.sectors) + " deg" if self.sectors else "",
            )
        )

    def _in_blind_sector(self, angle):
        for start, end in self.sectors_rad:
            if start <= angle <= end:
                return True
        return False

    def _on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        if self.frame_id:
            out.header.frame_id = self.frame_id
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = max(msg.range_min, self.min_range)
        out.range_max = min(msg.range_max, self.max_range)
        out.intensities = msg.intensities

        invalid = 0
        range_dropped = 0
        sector_masked = 0
        ranges = []
        angle = msg.angle_min + self.angle_offset
        for value in msg.ranges:
            if value is None or not math.isfinite(value) or value <= 0.0:
                # NaN / inf / 非正值统一成 inf：
                # SLAM 会按 range_max 处理，绝不会当成"近距离障碍"。
                invalid += 1
                drop = True
            elif value < self.min_range or value > self.max_range:
                range_dropped += 1
                drop = True
            elif self.sectors_rad and self._in_blind_sector(angle):
                sector_masked += 1
                drop = True
            else:
                drop = False

            if drop:
                ranges.append(math.inf)
            else:
                ranges.append(float(value))
            angle += msg.angle_increment

        out.ranges = ranges
        self.pub.publish(out)

        self._frames += 1
        self._invalid += invalid
        self._range_dropped += range_dropped
        self._sector_masked += sector_masked
        if invalid + range_dropped + sector_masked == len(ranges):
            self._empty_frames += 1

        now = self.get_clock().now()
        if (now - self._last_report).nanoseconds >= 10_000_000_000:
            beams = max(1, self._frames * max(1, len(ranges)))
            self.get_logger().info(
                "scan_filter: {} 帧 | 无效点 {:.1f}% | 超量程 {:.1f}% "
                "| 扇区屏蔽 {:.1f}% | 整帧无效 {} 次".format(
                    self._frames,
                    100.0 * self._invalid / beams,
                    100.0 * self._range_dropped / beams,
                    100.0 * self._sector_masked / beams,
                    self._empty_frames)
            )
            self._frames = 0
            self._invalid = 0
            self._range_dropped = 0
            self._sector_masked = 0
            self._empty_frames = 0
            self._last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
