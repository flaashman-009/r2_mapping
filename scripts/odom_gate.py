#!/usr/bin/env python3
"""里程计静止门：车停着的时候别让里程计"继续走"。

## 为什么需要它（2026-09-19 实测，这是"停下来就飘"的真正根因）

现象：导航到终点、车停稳后，**点云与地图突然对不上，位姿漂走**。

抓到的数据（车静止的 350 秒里，`cmd_vx = 0.000` 没有速度指令）：

| 量 | 值 |
|---|---|
| IMU 航向累计变化 | **−0.1 度**（车真的没动） |
| 原厂里程计 raw_yaw 累计变化 | **−2308 度（6.4 圈！）** |
| 原厂里程计位置 | 画了个半径 0.8 m 的圆，往返摆 |
| EKF 位置漂移 | **31 米** |
| 前轮实际转角 `vel_steer` | **+19°，350 秒一动不动** |
| 我们下发的 `cmd_steer` | −1.2°（想让它回正，没生效） |

### 发散链条

    底盘固件在 vx = 0 时不更新转向 → 前轮卡在最后一次的 +19°
              ↓
    base_node_R2 用 ω = vx·tan(δ)/L 算航向，它拿着 δ=19°
    和停车时轮子的微小往复（vx 有 ±0.1 m/s 级的噪声）
              ↓
    算出持续非零的角速度 → 航向单向累积 6 圈 → 位置画圆
              ↓
    EKF 只融合它的 vx（我们一开始就把 x,y,yaw 都关掉了），
    于是把这条"螺旋"积分出来 → 漂 31 米
              ↓
    AMCL 的运动模型跟着漂 → 位姿偏 → 点云残差 0.85 持续 4 分钟

**注意：这不是 AMCL 的错，是喂给它的里程计在车静止时发散了。**

## 本节点做什么

夹在 `/odom_raw` 和 EKF 之间：**车速低于阈值时，把 twist 置零**，
让 EKF 不积分这段"幻影运动"。带滞回，避免在阈值附近抖。

    /odom_raw  →  odom_gate  →  /odom_gated  →  EKF

姿态（pose）部分原样透传 —— EKF 只用 twist 里的 vx，位置由它自己积分。

用法：
    python3 odom_gate.py
    python3 odom_gate.py --ros-args -p deadband:=0.03
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class OdomGate(Node):

    def __init__(self):
        super().__init__("odom_gate")

        self.declare_parameter("input_topic", "/odom_raw")
        self.declare_parameter("output_topic", "/odom_gated")
        # 低于 deadband 认为"静止"，把 twist 置零
        self.declare_parameter("deadband", 0.03)          # m/s
        # 滞回：超过 release 才认为"真的在动"
        self.declare_parameter("release", 0.05)           # m/s
        # 角速度也一起判断（原地转时 vx 可能很小）
        self.declare_parameter("yawrate_deadband", 0.05)  # rad/s
        self.declare_parameter("log_every", 0)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self.deadband = self.get_parameter("deadband").value
        self.release = self.get_parameter("release").value
        self.w_deadband = self.get_parameter("yawrate_deadband").value
        self.log_every = int(self.get_parameter("log_every").value)

        self.gated = False      # True = 正在"静止"，输出置零
        self.n_in = 0
        self.n_zeroed = 0

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, in_topic, self._on_odom, qos)
        self.pub = self.create_publisher(Odometry, out_topic, qos)

        self.get_logger().info(
            "里程计静止门启动：{} -> {} | 死区 {:.3f} m/s（释放 {:.3f}）| "
            "角速度死区 {:.3f} rad/s".format(
                in_topic, out_topic, self.deadband, self.release,
                self.w_deadband))
        self.get_logger().info(
            "作用：车停着时不让里程计继续积分，治「停下来就飘」"
            "（实测停车 6 分钟里程计漂 31 米）")

    def _on_odom(self, msg):
        self.n_in += 1
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        if self.gated:
            # 已经判定静止：要超过 release 才解除
            if abs(vx) > self.release or abs(wz) > self.w_deadband * 2:
                self.gated = False
        else:
            if abs(vx) < self.deadband and abs(wz) < self.w_deadband:
                self.gated = True

        if self.gated:
            self.n_zeroed += 1
            out = msg
            out.twist.twist.linear.x = 0.0
            out.twist.twist.linear.y = 0.0
            out.twist.twist.angular.z = 0.0
            if self.log_every and self.n_zeroed % self.log_every == 0:
                self.get_logger().info(
                    "静止中：把 vx {:.4f} 置零（已挡 {} 帧）".format(
                        vx, self.n_zeroed))
        else:
            out = msg

        self.pub.publish(out)


def main():
    rclpy.init()
    node = OdomGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            "统计：收到 {} 帧，其中 {} 帧被判为静止并置零（{:.1f}%）".format(
                node.n_in, node.n_zeroed,
                100.0 * node.n_zeroed / max(1, node.n_in)))
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
