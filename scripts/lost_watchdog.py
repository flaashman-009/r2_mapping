#!/usr/bin/env python3
"""迷失看门狗（低成本版）：发现定位不可信就停车报警。

## 为什么需要它

AMCL 在**车停着不动**时会失去约束（没有新的运动信息，只能靠激光反复
打分；我们这张图 80.9% 是未知区，很多地方没墙可比）。它的自救机制会
随机撒粒子，撒多了就可能被拽到一个错误位姿上。

实测（2026-09-17，40 分钟记录）：
    运动时  协方差中位 0.055      ← 很好
    静止时  协方差中位 0.419      ← 差 8 倍
    t=1636s 车停着：协方差一路 0.06 → 28 → 155，map->odom 瞬移 12 m，
            之后 36 秒点云残差 4.3 m
    2026-09-16 更严重：map->odom 跳 90 m、转 171°，航向整体翻转，
            导致车"朝目标反方向开"

**真正的危险不是"停车后自己开"，而是：位姿已经飘了但你不知道，
再发一个目标 → Nav2 拿着错误位姿规划 → 车朝反方向走。**

## 设计目标：几乎零代价

早期版本要订阅地图和激光、建整张图的距离场（9.6 MB）、依赖 numpy+scipy。
本版本**只用两条 AMCL 本来就会发出的信号**，不碰地图、不碰激光、不碰 TF：

  **信号 1 —— AMCL 自己报的协方差**（/amcl_pose 里自带，不用算）
      健康 < 0.5，开始漂 12，迷失 155~385  →  阈值 5 分得很开

  **信号 2 —— "幻影运动"**
      车明明没动（里程计速度≈0），AMCL 报的位姿却自己在跑。
      这直接命中我们的场景：车停着时 AMCL 自漂 12 m。

代价对比：
              依赖           内存        计算              需要的输入
  旧版   numpy+scipy    ~9.6 MB    240 点坐标变换/次   地图+激光+TF
  本版   只要 rclpy     几百字节   10 Hz 几次比较       /amcl_pose /odom

## 用法

    python3 lost_watchdog.py
    python3 lost_watchdog.py --ros-args -p cov_threshold:=8.0

想更灵敏（但要装 numpy/scipy）可以打开 -p use_residual:=true，
它会额外算点云-地图残差，比协方差早 3~5 秒发现异常。
"""

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class LostWatchdog(Node):

    def __init__(self):
        super().__init__("lost_watchdog")

        self.declare_parameter("halt_topic", "/cmd_vel_halt")
        # 信号 1：AMCL 协方差
        self.declare_parameter("cov_threshold", 5.0)
        self.declare_parameter("cov_hold", 3.0)
        # 信号 2：幻影运动（车没动，位姿自己跑）
        self.declare_parameter("still_speed", 0.02)      # m/s
        self.declare_parameter("still_yawrate", 0.10)    # rad/s
        self.declare_parameter("phantom_move", 0.40)     # m
        self.declare_parameter("phantom_yaw", 0.17)      # rad (~10°)
        self.declare_parameter("window", 3.0)            # s
        # 单步跳变（AMCL 瞬移）
        self.declare_parameter("jump_threshold", 1.0)    # m
        self.declare_parameter("arm_cov", 1.0)           # 协方差低于它才算"定位好过"
        self.declare_parameter("clear_time", 5.0)
        self.declare_parameter("auto_clear", True)
        self.declare_parameter("check_rate", 5.0)
        # 可选：更灵敏但更贵的点云残差
        self.declare_parameter("use_residual", False)
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("res_threshold", 0.35)

        halt_topic = self.get_parameter("halt_topic").value
        self.cov_th = self.get_parameter("cov_threshold").value
        self.cov_hold = self.get_parameter("cov_hold").value
        self.still_v = self.get_parameter("still_speed").value
        self.still_w = self.get_parameter("still_yawrate").value
        self.ph_move = self.get_parameter("phantom_move").value
        self.ph_yaw = self.get_parameter("phantom_yaw").value
        self.win = self.get_parameter("window").value
        self.jump_th = self.get_parameter("jump_threshold").value
        self.arm_cov = self.get_parameter("arm_cov").value
        self.clear_t = self.get_parameter("clear_time").value
        self.auto_clear = self.get_parameter("auto_clear").value
        self.use_res = self.get_parameter("use_residual").value
        self.res_th = self.get_parameter("res_threshold").value
        scan_topic = self.get_parameter("scan_topic").value
        rate = self.get_parameter("check_rate").value

        self.cov = 0.0
        self.n_pose = 0
        self.armed = False
        self.halted = False
        self.bad_since = None
        self.good_since = None
        self.n_halt = 0

        # 里程计速度（判断车有没有动）
        self.v = 0.0
        self.w = 0.0
        # AMCL 位姿滑动窗口
        self.hist = deque()
        self.last_pose = None

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._on_amcl, qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.pub = self.create_publisher(Bool, halt_topic, 10)

        # 可选的残差检查（默认关闭）
        self.dist_field = None
        self.map_info = None
        self.last_scan = None
        self.tf_buffer = None
        if self.use_res:
            self._setup_residual(scan_topic)

        self.create_timer(1.0 / max(0.5, rate), self._tick)
        self.get_logger().info(
            "迷失看门狗（低成本版）启动：协方差阈值 {:.1f}（持续 {:.0f}s）| "
            "幻影运动 {:.2f} m/{:.0f}s | 单步跳变 {:.1f} m | "
            "残差检查 {}".format(
                self.cov_th, self.cov_hold, self.ph_move, self.win,
                self.jump_th, "开" if self.use_res else "关（省 CPU）"))

    # ------------------------------------------------------------------
    def _setup_residual(self, scan_topic):
        """只有显式要求时才加载地图相关的东西。"""
        try:
            import numpy as np
            from scipy.ndimage import distance_transform_edt
            from nav_msgs.msg import OccupancyGrid
            from sensor_msgs.msg import LaserScan
            from rclpy.duration import Duration
            from rclpy.qos import (QoSDurabilityPolicy, QoSProfile as QP)
            from rclpy.time import Time
            from tf2_ros import Buffer, TransformListener
        except Exception as exc:
            self.get_logger().error(
                "use_residual 打开了但缺依赖（{}），退回只用协方差".format(exc))
            self.use_res = False
            return
        self._np = np
        self._edt = distance_transform_edt
        self._Time = Time
        self._Duration = Duration
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QP(depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        q = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, q)
        self.use_res = True

    def _on_map(self, msg):
        w, h = msg.info.width, msg.info.height
        occ = self._np.array(msg.data, dtype=self._np.int16).reshape(h, w) >= 65
        if not occ.any():
            return
        self.dist_field = self._edt(~occ) * msg.info.resolution
        self.map_info = msg.info

    def _on_scan(self, msg):
        self.last_scan = msg

    # ------------------------------------------------------------------
    def _on_amcl(self, msg):
        self.n_pose += 1
        self.cov = (msg.pose.covariance[0] + msg.pose.covariance[7]
                    + msg.pose.covariance[35])
        p = msg.pose.pose
        now = time.monotonic()
        x, y = p.position.x, p.position.y
        yaw = quat_to_yaw(p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w)
        # 单步跳变：AMCL 一瞬间把自己挪走
        if self.last_pose is not None:
            dt, lx, ly, lyaw = self.last_pose
            if now - dt < 0.5:
                d = math.hypot(x - lx, y - ly)
                dy = abs(wrap(yaw - lyaw))
                if d > self.jump_th or dy > 0.6:
                    self._halt("AMCL 位姿瞬移 {:.1f} m / {:.0f}°（单步）".format(
                        d, math.degrees(dy)))
        self.last_pose = (now, x, y, yaw)
        self.hist.append((now, x, y, yaw))
        while self.hist and now - self.hist[0][0] > self.win:
            self.hist.popleft()

    def _on_odom(self, msg):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    # ------------------------------------------------------------------
    def _phantom(self):
        """车没动，但 AMCL 位姿在跑 → 返回位移和转角。"""
        if len(self.hist) < 3:
            return 0.0, 0.0
        t0, x0, y0, a0 = self.hist[0]
        t1, x1, y1, a1 = self.hist[-1]
        if t1 - t0 < self.win * 0.7:
            return 0.0, 0.0
        return math.hypot(x1 - x0, y1 - y0), abs(wrap(a1 - a0))

    def _residual(self):
        if not self.use_res or self.dist_field is None or \
                self.tf_buffer is None or self.last_scan is None:
            return float("nan")
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", self.last_scan.header.frame_id, self._Time(),
                self._Duration(seconds=0.1))
        except Exception:
            return float("nan")
        msg = self.last_scan
        info = self.map_info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        q = tf.transform.rotation
        th = quat_to_yaw(q.x, q.y, q.z, q.w)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        ct, st = math.cos(th), math.sin(th)
        step = max(1, len(msg.ranges) // 240)
        d = []
        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if not math.isfinite(r) or r <= msg.range_min or r >= msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            gx = int((tx + lx * ct - ly * st - ox) / res)
            gy = int((ty + lx * st + ly * ct - oy) / res)
            if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
                continue
            d.append(float(self.dist_field[gy, gx]))
        if len(d) < 10:
            return float("nan")
        return float(self._np.median(d))

    # ------------------------------------------------------------------
    def _halt(self, reason):
        if self.halted:
            return
        self.halted = True
        self.n_halt += 1
        m = Bool()
        m.data = True
        self.pub.publish(m)
        self.get_logger().error("★★★ 定位不可信 → 停车！原因：{}".format(reason))
        self.get_logger().error(
            "     车已停。**在定位恢复之前不要发新目标**，"
            "否则会拿着错误位姿规划出反向路径。")

    def _resume(self):
        self.halted = False
        m = Bool()
        m.data = False
        self.pub.publish(m)
        self.get_logger().warn("定位恢复，解除停车，可以继续导航")

    def _tick(self):
        now = time.monotonic()
        still = abs(self.v) < self.still_v and abs(self.w) < self.still_w
        dmove, dyaw = self._phantom() if still else (0.0, 0.0)
        res = self._residual()

        bad, why = False, ""
        if self.cov > self.cov_th:
            bad, why = True, "AMCL 协方差 {:.1f} > {:.1f}".format(
                self.cov, self.cov_th)
        elif dmove > self.ph_move or dyaw > self.ph_yaw:
            bad, why = True, "车没动但位姿漂了 {:.2f} m / {:.0f}°（{}秒内）".format(
                dmove, math.degrees(dyaw), self.win)
        elif math.isfinite(res) and res > self.res_th:
            bad, why = True, "点云残差 {:.2f} m".format(res)

        if bad:
            self.good_since = None
            if not self.armed:
                return
            if self.bad_since is None:
                self.bad_since = now
            elif not self.halted and now - self.bad_since >= self.cov_hold:
                self._halt(why)
        else:
            self.bad_since = None
            # 必须先真的收到过一批位姿，否则"协方差 0"只是还没数据
            if not self.armed and self.n_pose > 20 and self.cov < self.arm_cov:
                self.armed = True
                self.get_logger().info(
                    "定位良好（协方差 {:.2f}，已收到 {} 条位姿）→ "
                    "看门狗武装，之后一旦迷失就会叫停".format(
                        self.cov, self.n_pose))
            if self.halted and self.auto_clear:
                if self.good_since is None:
                    self.good_since = now
                elif now - self.good_since >= self.clear_t:
                    self._resume()
            elif not self.halted:
                self.good_since = None


def main():
    rclpy.init()
    node = LostWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("看门狗统计：触发停车 {} 次".format(node.n_halt))
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
