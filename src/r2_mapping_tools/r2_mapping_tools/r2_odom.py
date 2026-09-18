#!/usr/bin/env python3
"""R2 里程计节点（替代原厂 base_node_R2）—— 参数全部可调、可运行时修改。

为什么重写
----------
原厂 `base_node_R2.cpp` 的航向模型是：

    steer_angle = vel_raw.linear.y          # 底盘反馈的转向角（度）
    R = wheelbase / tan(steer_angle)
    omega = v / R = v * tan(steer_angle) / wheelbase
    heading += omega * dt
    x += v * cos(heading) * dt
    y += v * sin(heading) * dt

三个问题：
  1. `wheelbase` 出厂默认 0.25，而 R2 实测轴距 0.2681（差 7.24%），
     且任何 launch 都没设过 —— 每次转弯多转 7.24%
  2. **完全不考虑左右轮速度差**。这台车右后轮比左轮快 1.6-2.6%，
     转向角为 0 时车其实在弯，但里程计认为在走直线
  3. 用矩形法积分（x += v·cosθ·dt），不如中点法准

本节点在保留阿克曼模型的基础上补了两个可标定项：

    delta_eff = delta - steer_zero_deg            # 转向零位偏置
    omega = v * tan(delta_eff) / wheelbase        # 阿克曼几何
          + v * yaw_bias_per_m                    # 轮速不对称造成的额外偏航

`yaw_bias_per_m` 是**每走 1 米额外产生的航向变化（弧度/米）**，
它随速度线性变化，正好对应"左右轮速度差与速度成正比"这一物理事实。
标定它之后，里程计就能**如实反映车实际在弯**，SLAM 的运动先验才和现实一致。

参数（全部可以 `ros2 param set` 运行时修改）
-------------------------------------------
  wheelbase         轴距，默认 0.2681（实测值）
  steer_zero_deg    转向零位偏置（度）。车摆正时舵机反馈是多少就填多少
  yaw_bias_per_m    每米额外偏航（弧度/米）。标定方法见下
  linear_scale      距离尺度，标定方法见下
  track             轮距，默认 0.1646（备用，差速模型用）
  max_steer_deg     转向角限幅，防止 tan 爆掉，默认 60
  deadband_mps      速度死区，小于此值当静止，默认 0.005
  publish_tf        是否发 odom->base_footprint（EKF 在跑时保持 false）
  use_imu_reference 是否订阅 /imu/data 做对照打印
  log_interval_s    对照打印间隔

用法
----
    # 单独跑（需要先把原厂 base_node_R2 停掉，否则两个都发 /odom_raw）
    python3 r2_odom.py

    # 或者用脚本一键切换
    bash ~/r2_mapping/scripts/start_odom_experimental.sh

    # 运行时改参数（立刻生效，不用重启）
    ros2 param set /r2_odom yaw_bias_per_m 0.12
    ros2 param set /r2_odom linear_scale 0.94

标定方法
--------
**linear_scale**（距离尺度）
    手推车走 1 米，对比刊程计读数和卷尺实测：
        linear_scale = 卷尺实测 / 里程计读数

**yaw_bias_per_m**（额外偏航）
    遥控直行（转向摇杆居中）走一段，比如 10 米，看节点打印的
    "IMU 航向 vs odom 航向"差异：
        yaw_bias_per_m = (IMU 航向变化 − 期望值) / 走过的距离
    如果车自己往左弯、IMU 显示转了多少，就填多少。
    符号：往左转为正（逆时针）。

**steer_zero_deg**（转向零位）
    车摆正、转向反馈稳定后，读 `ros2 topic echo /vel_raw --field linear.y`，
    那个值就是零位偏置。
"""

import math
import sys

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class R2Odom(Node):

    def __init__(self):
        super().__init__("r2_odom")

        # ---------------- 可标定参数 ----------------
        self.declare_parameter("wheelbase", 0.2681)
        self.declare_parameter("steer_zero_deg", 0.0)
        self.declare_parameter("yaw_bias_per_m", 0.0)
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("track", 0.1646)
        self.declare_parameter("max_steer_deg", 60.0)
        self.declare_parameter("deadband_mps", 0.005)
        # ---------------- 其他 ----------------
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("use_imu_reference", True)
        self.declare_parameter("log_interval_s", 5.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("vel_topic", "/vel_raw")
        self.declare_parameter("odom_topic", "/odom_raw")

        g = self.get_parameter
        self.wheelbase = float(g("wheelbase").value)
        self.steer_zero = float(g("steer_zero_deg").value)
        self.yaw_bias_per_m = float(g("yaw_bias_per_m").value)
        self.linear_scale = float(g("linear_scale").value)
        self.track = float(g("track").value)
        self.max_steer = abs(float(g("max_steer_deg").value))
        self.deadband = abs(float(g("deadband_mps").value))
        self.publish_tf = bool(g("publish_tf").value)
        self.use_imu = bool(g("use_imu_reference").value)
        self.log_interval = float(g("log_interval_s").value)
        self.odom_frame = g("odom_frame").value
        self.base_frame = g("base_frame").value
        vel_topic = g("vel_topic").value
        odom_topic = g("odom_topic").value

        # ---------------- 状态 ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.distance = 0.0          # 累计路程（标定用）
        self._last_time = None
        self._last_vel = None
        self.imu_yaw = None
        self.imu_yaw0 = None
        self._last_log = None
        self._dt_dropped = 0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(Twist, vel_topic, self._on_vel, 50)
        if self.use_imu:
            self.create_subscription(Imu, "/imu/data", self._on_imu, qos)

        self.pub = self.create_publisher(Odometry, odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info(
            "r2_odom 启动：{} -> {}".format(vel_topic, odom_topic))
        self.get_logger().info(
            "  轴距 {:.4f} m | 转向零位 {:+.2f}° | 额外偏航 {:+.4f} rad/m "
            "| 尺度 {:.4f}".format(
                self.wheelbase, self.steer_zero, self.yaw_bias_per_m,
                self.linear_scale))
        self.get_logger().info(
            "  运行时可改：ros2 param set /r2_odom <参数名> <值>")
        if self.publish_tf:
            self.get_logger().warn(
                "  publish_tf=true —— 如果 EKF 也在跑，odom->base_footprint 会有两个发布者！")

    # ------------------------------------------------------------------
    def _on_imu(self, msg):
        q = msg.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        if self.imu_yaw0 is None:
            self.imu_yaw0 = yaw
        self.imu_yaw = math.atan2(math.sin(yaw - self.imu_yaw0),
                                  math.cos(yaw - self.imu_yaw0))

    def _on_vel(self, msg):
        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            return
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        # dt 异常（丢消息、卡顿）直接丢掉，否则积分会飞
        if dt <= 0.0 or dt > 0.5:
            self._dt_dropped += 1
            return

        v = float(msg.linear.x) * self.linear_scale
        if abs(v) < self.deadband:
            v = 0.0

        delta_deg = float(msg.linear.y) - self.steer_zero
        delta_deg = max(-self.max_steer, min(self.max_steer, delta_deg))
        delta_rad = math.radians(delta_deg)

        # 阿克曼几何 + 轮速不对称的线性补偿
        omega = 0.0
        if abs(v) > 0.0:
            omega = v * math.tan(delta_rad) / self.wheelbase
            omega += v * self.yaw_bias_per_m

        # 中点法积分：比原厂的矩形法更准
        theta_mid = self.theta + omega * dt * 0.5
        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta += omega * dt
        self.distance += abs(v) * dt

        self._publish(now, v, omega)
        self._maybe_log(now)

    # ------------------------------------------------------------------
    def _publish(self, now, v, omega):
        quat_z = math.sin(self.theta * 0.5)
        quat_w = math.cos(self.theta * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = quat_z
        odom.pose.pose.orientation.w = quat_w
        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.001
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega
        self.pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = quat_z
            tf.transform.rotation.w = quat_w
            self.tf_broadcaster.sendTransform(tf)

    def _maybe_log(self, now):
        if self.log_interval <= 0.0:
            return
        if self._last_log is None:
            self._last_log = now
            return
        if (now - self._last_log).nanoseconds / 1e9 < self.log_interval:
            return
        self._last_log = now

        odom_deg = math.degrees(self.theta)
        msg = "[r2_odom] 路程 {:6.2f} m | x {:+.3f} y {:+.3f} | odom 航向 {:+.1f}°".format(
            self.distance, self.x, self.y, odom_deg)
        if self.imu_yaw is not None:
            imu_deg = math.degrees(self.imu_yaw)
            msg += " | IMU 航向 {:+.1f}° | 差 {:+.1f}°".format(
                imu_deg, imu_deg - odom_deg)
        if self._dt_dropped:
            msg += " | 丢弃异常dt {}".format(self._dt_dropped)
        self.get_logger().info(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = R2Odom()
    try:
        rclpy.spin(node)
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

