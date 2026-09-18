#!/usr/bin/env python3
"""导航行驶记录仪：抓"定位什么时候开始错"。

记录 8 路信号到 CSV，并在检出异常时**实时报警**：

  1. /odom        —— EKF 里程计（也就是 TF 用的那份）
  2. /odom_raw    —— 原厂里程计（对照）
  3. /imu/data    —— madgwick 姿态 yaw（EKF 融合的就是它）
  4. /imu/yaw_deg —— 底盘板载 yaw（独立来源，用来判断谁在跳）
  5. /amcl_pose   —— AMCL 位姿 + 协方差（定位不确定度）
  6. map->odom    —— AMCL 的修正量（跳变 = AMCL 把自己"瞬移"了）
  7. /cmd_vel 与 /vel_raw —— 指令与实际转角
  8. scan_to_map_residual —— 点云到地图最近障碍的距离（中位数 / 90 分位）

第 8 项是抓"缓慢漂移"的关键，也是这一版新增的：
     正常      0.05 ~ 0.20 m（约一个栅格到一层墙厚）
     正在滑走  持续爬升到 0.3 m 以上
     彻底迷失  > 0.5 m，p90 很大
漂移是渐进的，map->odom 只会在最后"崩"的一下跳 —— 只看跳变会漏掉前面几分钟。

报警判据：
  * IMU yaw 单步跳 > 2°              → 磁力计受干扰，航向源坏了
  * map->odom 单步跳 > 0.15 m / 5°   → AMCL 突然重新收敛（"迷失"事件）
  * 点云残差中位数 > 0.35 m          → 点云已经开始对不上地图

**日志默认写到 ~/r2_mapping/logs/**，不要用 /tmp —— /tmp 重启后会被清空，
2026-09-16 那次记录就是这样丢的。

用法：
    python3 ~/r2_mapping/scripts/nav_watch.py
    python3 ~/r2_mapping/scripts/nav_watch.py --csv ~/r2_mapping/logs/xxx.csv

跑完 Ctrl-C，或直接看终端里的报警行。
"""

import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

try:
    import numpy as np
    from scipy.ndimage import distance_transform_edt
    HAVE_NUMPY = True
except Exception:
    HAVE_NUMPY = False

try:
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    HAVE_TF2 = True
except Exception:
    HAVE_TF2 = False


COLUMNS = ("odom_x", "odom_y", "odom_yaw", "raw_x", "raw_y", "raw_yaw",
           "imu_yaw", "board_yaw", "amcl_x", "amcl_y", "amcl_yaw", "amcl_cov",
           "mo_x", "mo_y", "mo_yaw", "cmd_vx", "cmd_steer", "cmd_w",
           "vel_steer", "res_med", "res_p90", "res_ok", "res_far")


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def deg(a):
    return math.degrees(a)


class NavWatch(Node):

    def __init__(self, csv_path, res_interval=1.0):
        super().__init__("nav_watch")
        self.t0 = time.monotonic()
        self.csv_path = csv_path
        self.res_interval = res_interval
        self.fh = open(csv_path, "w", encoding="utf-8")
        self.fh.write("t," + ",".join(COLUMNS) + "\n")

        self.last = {}
        self.n_imu_jump = 0
        self.n_mo_jump = 0
        self.n_res_bad = 0
        self.max_cov = 0.0
        self.max_res_med = 0.0
        self.last_print = 0.0
        self.res_warned = False

        # 转向通道统计：用一趟行驶数据直接判定"谁在驱动舵机"
        self.n_cmd_y = 0        # /cmd_vel.linear.y 非零的帧数
        self.n_cmd_w = 0        # /cmd_vel.angular.z 非零的帧数
        self.n_fb_steer = 0     # /vel_raw.linear.y 非零的帧数（舵机真的动了）

        self.cur = {k: 0.0 for k in COLUMNS}
        self.dirty = False

        self.dist_field = None      # 每格到最近障碍的距离（米）
        self.map_info = None
        self.last_res_t = 0.0
        self.n_scan = 0
        self.n_amcl = 0

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=100)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.create_subscription(Odometry, "/odom_raw", self._on_raw, qos)
        self.create_subscription(Imu, "/imu/data", self._on_imu, qos)
        self.create_subscription(Float64, "/imu/yaw_deg", self._on_board, qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._on_amcl, qos)
        self.create_subscription(TFMessage, "/tf", self._on_tf, qos)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 50)
        self.create_subscription(Twist, "/vel_raw", self._on_vel, qos)

        map_qos = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos)

        if HAVE_TF2:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None

        self.create_timer(0.1, self._flush)
        self.create_timer(10.0, self._selfcheck)

    def _stamp(self):
        return time.monotonic() - self.t0

    # ---------------------------------------------------------- 地图/残差
    def _on_map(self, msg):
        if not HAVE_NUMPY:
            return
        w, h = msg.info.width, msg.info.height
        occ = np.array(msg.data, dtype=np.int16).reshape(h, w) >= 65
        if not occ.any():
            return
        self.dist_field = distance_transform_edt(~occ) * msg.info.resolution
        self.map_info = msg.info

    def _on_scan(self, msg):
        self.n_scan += 1
        if self.dist_field is None or self.tf_buffer is None:
            return
        now = time.monotonic()
        if now - self.last_res_t < self.res_interval:
            return
        self.last_res_t = now
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id, Time(), Duration(seconds=0.2))
        except Exception:
            return

        info = self.map_info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        q = tf.transform.rotation
        th = quat_to_yaw(q.x, q.y, q.z, q.w)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        ct, st = math.cos(th), math.sin(th)

        step = max(1, len(msg.ranges) // 240)
        d, far = [], 0
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
            dist = float(self.dist_field[gy, gx])
            d.append(dist)
            if dist > 0.5:
                far += 1
        if len(d) < 10:
            return
        med = float(np.median(d))
        p90 = float(np.percentile(d, 90))
        self.cur["res_med"] = med
        self.cur["res_p90"] = p90
        self.cur["res_ok"] = float(len(d))
        self.cur["res_far"] = float(far) / len(d)
        self.max_res_med = max(self.max_res_med, med)
        if med > 0.35:
            self.n_res_bad += 1
            if not self.res_warned:
                self.res_warned = True
                self.get_logger().warn(
                    "★★★ 点云残差中位数 {:.2f} m（>0.35）t={:.1f}s "
                    "→ 点云开始对不上地图，正在漂".format(med, self._stamp()))
        elif med < 0.2:
            self.res_warned = False

    # ---------------------------------------------------------- 里程计
    def _on_odom(self, msg):
        p = msg.pose.pose
        self.cur["odom_x"] = p.position.x
        self.cur["odom_y"] = p.position.y
        self.cur["odom_yaw"] = quat_to_yaw(p.orientation.x, p.orientation.y,
                                           p.orientation.z, p.orientation.w)
        self.dirty = True

    def _on_raw(self, msg):
        p = msg.pose.pose
        self.cur["raw_x"] = p.position.x
        self.cur["raw_y"] = p.position.y
        self.cur["raw_yaw"] = quat_to_yaw(p.orientation.x, p.orientation.y,
                                          p.orientation.z, p.orientation.w)

    def _on_imu(self, msg):
        q = msg.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        prev = self.last.get("imu_yaw")
        if prev is not None:
            dd = abs(deg(wrap(yaw - prev)))
            if dd > 2.0:
                self.n_imu_jump += 1
                self.get_logger().warn(
                    "★ IMU(/imu/data) yaw 跳变 {:.1f}°  t={:.1f}s  "
                    "→ 磁力计受干扰或姿态滤波跳变".format(dd, self._stamp()))
        self.last["imu_yaw"] = yaw
        self.cur["imu_yaw"] = yaw

    def _on_board(self, msg):
        self.cur["board_yaw"] = math.radians(msg.data)

    def _on_amcl(self, msg):
        self.n_amcl += 1
        p = msg.pose.pose
        self.cur["amcl_x"] = p.position.x
        self.cur["amcl_y"] = p.position.y
        self.cur["amcl_yaw"] = quat_to_yaw(p.orientation.x, p.orientation.y,
                                           p.orientation.z, p.orientation.w)
        self.cur["amcl_cov"] = msg.pose.covariance[0] + \
            msg.pose.covariance[7] + msg.pose.covariance[35]
        self.max_cov = max(self.max_cov, self.cur["amcl_cov"])

    def _on_tf(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id != "map" or tr.child_frame_id != "odom":
                continue
            x = tr.transform.translation.x
            y = tr.transform.translation.y
            yaw = quat_to_yaw(tr.transform.rotation.x,
                              tr.transform.rotation.y,
                              tr.transform.rotation.z,
                              tr.transform.rotation.w)
            px, py, pyaw = (self.last.get("mo_x"), self.last.get("mo_y"),
                            self.last.get("mo_yaw"))
            if px is not None:
                dd = math.hypot(x - px, y - py)
                dyaw = abs(deg(wrap(yaw - pyaw)))
                if dd > 0.15 or dyaw > 5.0:
                    self.n_mo_jump += 1
                    self.get_logger().warn(
                        "★★ map->odom 跳变 Δ={:.3f} m / {:.1f}°  t={:.1f}s  "
                        "→ AMCL 突然重新收敛".format(dd, dyaw, self._stamp()))
            self.last["mo_x"], self.last["mo_y"], self.last["mo_yaw"] = x, y, yaw
            self.cur["mo_x"], self.cur["mo_y"], self.cur["mo_yaw"] = x, y, yaw

    def _on_cmd(self, msg):
        self.cur["cmd_vx"] = msg.linear.x
        self.cur["cmd_steer"] = msg.linear.y
        self.cur["cmd_w"] = msg.angular.z
        if abs(msg.linear.y) > 0.002:
            self.n_cmd_y += 1
        if abs(msg.angular.z) > 0.02:
            self.n_cmd_w += 1

    def _on_vel(self, msg):
        self.cur["vel_steer"] = msg.linear.y
        if abs(msg.linear.y) > 1.0:      # 单位是"度"
            self.n_fb_steer += 1

    # ---------------------------------------------------------- 自检/落盘
    def _selfcheck(self):
        t = self._stamp()
        if t < 15:
            return
        bad = []
        if self.n_amcl == 0:
            bad.append("没有 /amcl_pose（Nav2/AMCL 没在跑？）")
        if self.dist_field is None:
            bad.append("没有 /map（map_server 没起来？）")
        if self.n_scan == 0:
            bad.append("没有 /scan（雷达没起来？）")
        if bad:
            self.get_logger().warn("自检：{}".format("；".join(bad)))
        else:
            self.get_logger().info(
                "自检正常：scan {} 帧 / amcl {} 帧 / 地图已加载".format(
                    self.n_scan, self.n_amcl))

    def _flush(self):
        if not self.dirty:
            return
        self.dirty = False
        row = [self._stamp()] + [self.cur[k] for k in COLUMNS]
        self.fh.write(",".join("{:.6f}".format(v) for v in row) + "\n")
        self.fh.flush()

        now = time.monotonic()
        if now - self.last_print >= 5.0:
            self.last_print = now
            self.get_logger().info(
                "t={:6.1f}s odom_yaw={:+.1f}° imu_yaw={:+.1f}° "
                "amcl=({:+.2f},{:+.2f}) cov={:.4f} "
                "残差 med={:.2f} p90={:.2f} m | 跳变 imu {} / mapodom {} / 残差 {}".format(
                    self._stamp(), deg(self.cur["odom_yaw"]),
                    deg(self.cur["imu_yaw"]), self.cur["amcl_x"],
                    self.cur["amcl_y"], self.cur["amcl_cov"],
                    self.cur["res_med"], self.cur["res_p90"],
                    self.n_imu_jump, self.n_mo_jump, self.n_res_bad))

    def close(self):
        try:
            self.fh.close()
        except Exception:
            pass


def main(argv=None):
    default_csv = os.path.join(
        os.path.expanduser("~"), "r2_mapping", "logs",
        "nav_watch_{}.csv".format(time.strftime("%m%d_%H%M%S")))
    ap = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--csv", default=default_csv)
    ap.add_argument("--res-interval", type=float, default=1.0,
                    help="点云残差的计算间隔（秒）")
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    os.makedirs(os.path.dirname(os.path.abspath(args.csv)), exist_ok=True)

    rclpy.init()
    node = NavWatch(args.csv, args.res_interval)

    print("=" * 68)
    print(" 导航记录仪已启动")
    print("   CSV : {}".format(os.path.abspath(args.csv)))
    print("   numpy={}  tf2={}".format(HAVE_NUMPY, HAVE_TF2))
    if not HAVE_NUMPY:
        print("   ** 没有 numpy/scipy，点云残差这一路会跳过")
    print(" 现在可以让车动起来。Ctrl-C 结束。")
    print("=" * 68)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        print("\n--- 汇总 ---")
        print("IMU yaw 跳变次数      : {}".format(node.n_imu_jump))
        print("map->odom 跳变次数    : {}".format(node.n_mo_jump))
        print("点云残差超限次数      : {}".format(node.n_res_bad))
        print("点云残差中位数最大值  : {:.3f} m".format(node.max_res_med))
        print("AMCL 协方差最大值     : {:.4f}".format(node.max_cov))
        print("")
        print("--- 转向通道判定（用这一趟的数据直接判）---")
        print("  /cmd_vel.linear.y  非零帧 : {:6d}".format(node.n_cmd_y))
        print("  /cmd_vel.angular.z 非零帧 : {:6d}".format(node.n_cmd_w))
        print("  /vel_raw.linear.y  非零帧 : {:6d}  (底盘反馈实际转角，"
              "非零=舵机真的动了)".format(node.n_fb_steer))
        if node.n_fb_steer > 20:
            if node.n_cmd_y > 20 and node.n_cmd_w <= 20:
                print("  → 转向走 **linear.y**（TEB 不产生这个量，"
                      "所以必须加 ω→linear.y 的适配节点）")
            elif node.n_cmd_w > 20 and node.n_cmd_y <= 20:
                print("  → 转向走 **angular.z**（TEB 直接驱动舵机，"
                      "重点是尺度和量纲对不对）")
            elif node.n_cmd_y > 20 and node.n_cmd_w > 20:
                print("  → 两个通道都在发，需要再单独测一次（架起车）")
            else:
                print("  → 舵机在动但两个指令都基本是 0，"
                      "说明是固件自己解析，需要架起车单独测")
        else:
            print("  → 舵机基本没动，这一趟数据判不出来")
        print("CSV: {}".format(os.path.abspath(args.csv)))
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
