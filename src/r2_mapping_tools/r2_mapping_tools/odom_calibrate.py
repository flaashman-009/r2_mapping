#!/usr/bin/env python3
"""里程计尺度标定：测出 odom 的距离偏差，算出该写的 linear_scale。

原理
----
轮式里程计的距离 = 编码器计数 x 每米多少 tick 的倒数。
"每米多少 tick" 依赖轮径，而轮径会随载重、磨损、地面软硬变化，
所以出厂值必然有偏差。本工具测出这个偏差，给出修正系数。

两个模式
--------
**自动行驶模式**（默认）：车会自己往前开一段，全程 0.3 m/s，走完自动停。

    python3 odom_calibrate.py --ros-args -p target_m:=1.0 -p speed:=0.3

    流程：
      1. 把车摆好，在前轮位置的地板上贴一条胶带
      2. 回车，车开始走
      3. 车停下后，在**现在前轮的位置**再贴一条胶带
      4. 用卷尺量两条胶带之间的距离 = 实测距离
      5. 按提示输入实测距离

**推车模式**（更安全，不用电机）：

    python3 odom_calibrate.py --ros-args -p push_mode:=true

    流程：
      1. 摆好车，贴第一条胶带，回车
      2. **用手推车** 走过 1 m 左右（尽量走直线，别推太快）
      3. 贴第二条胶带，回车
      4. 量距离，输入

安全
----
- 自动模式下前方留 2 m 净空，人站在电源旁边
- 车轮必须**落地**（标定的就是落地后的滚动半径）
- 速度默认 0.3 m/s；想更稳可以设 0.2
- 建议量 3 次取中位数；单次误差 < 5% 合格

输出
----
工具会算 建议 scale = 实测距离 / 里程计读数，
并告诉你要改哪个文件。

注意：`linear_scale_x` 到底是"乘"还是"除"，
取决于 base_node_R2 的实现。改完必须**再标定一次验证**——
如果第二次的读数更准了，方向就对了；如果更糟，取倒数。
"""

import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomCalibrate(Node):

    def __init__(self):
        super().__init__("r2_odom_calibrate")

        self.declare_parameter("target_m", 1.0)
        self.declare_parameter("speed", 0.3)
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("timeout_s", 30.0)
        self.declare_parameter("push_mode", False)

        self.target = float(self.get_parameter("target_m").value)
        self.speed = abs(float(self.get_parameter("speed").value))
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.push_mode = bool(self.get_parameter("push_mode").value)
        odom_topic = self.get_parameter("odom_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value

        self.sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.pub = self.create_publisher(Twist, cmd_topic, 10)

        self.pose = None          # (x, y, yaw)
        self.start = None
        self.travel = 0.0
        self.lateral = 0.0
        self.yaw_delta = 0.0
        self.phase = "idle"
        self.started_at = None
        self.timer = self.create_timer(0.05, self._tick)

    # ------------------------------------------------------------------
    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def _start_ref(self):
        """记录起点，并把 travel 清零。"""
        if self.pose is None:
            return False
        self.start = self.pose
        self.travel = 0.0
        self.lateral = 0.0
        self.yaw_delta = 0.0
        return True

    def _update_from_pose(self):
        """按当前位置更新累计位移（用相邻点距离累加，避免依赖直线假设）。"""
        if self.pose is None or self.start is None:
            return
        x, y, yaw = self.pose
        sx, sy, syaw = self.start
        self.travel = math.hypot(x - sx, y - sy)
        # 横向：把位移投影到起点朝向的法向
        dx, dy = x - sx, y - sy
        self.lateral = -math.sin(syaw) * dx + math.cos(syaw) * dy
        self.yaw_delta = math.atan2(math.sin(yaw - syaw), math.cos(yaw - syaw))

    # ------------------------------------------------------------------
    def _tick(self):
        self._update_from_pose()
        if self.phase != "run":
            return

        cmd = Twist()
        cmd.linear.x = self.speed
        self.pub.publish(cmd)

        if self.started_at is not None:
            elapsed = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
            if self.timeout_s > 0.0 and elapsed > self.timeout_s:
                self.get_logger().warn(
                    "超时 {:.0f} s（只走了 {:.3f} m），停车".format(self.timeout_s, self.travel))
                self.pub.publish(Twist())
                self.phase = "done"
                return

        if self.travel >= self.target:
            self.pub.publish(Twist())
            self.phase = "done"

    # ------------------------------------------------------------------
    def run_drive(self):
        print("")
        print("=" * 60)
        print(" 里程计标定 —— 自动行驶模式")
        print("=" * 60)
        print(" 1. 车摆好，前方留 2 m 净空，人站在电源旁边")
        print(" 2. 在【当前前轮位置】的地板上贴一条胶带")
        print(" 3. 回车开始，车会以 {:.2f} m/s 前进 {:.2f} m 后自动停".format(
            self.speed, self.target))
        input(" 准备好了按 Enter ... ")

        if not self._start_ref():
            print(" 还没收到 {} 数据，先确认硬件在跑。".format(
                self.get_parameter("odom_topic").value))
            return

        self.phase = "run"
        self.started_at = self.get_clock().now()
        print(" 车在行驶 ...")
        while rclpy.ok() and self.phase != "done":
            rclpy.spin_once(self, timeout_sec=0.05)
        self.pub.publish(Twist())

        print(" 车已停下。")
        print(" 4. 在【现在前轮的位置】再贴一条胶带")
        print(" 5. 用卷尺量两条胶带之间的距离")
        self._report()

    def run_push(self):
        print("")
        print("=" * 60)
        print(" 里程计标定 —— 推车模式（不用电机）")
        print("=" * 60)
        print(" 1. 车摆好，在【当前前轮位置】的地板上贴一条胶带")
        input(" 按 Enter 记录起点 ... ")

        if not self._start_ref():
            print(" 还没收到 {} 数据，先确认硬件在跑。".format(
                self.get_parameter("odom_topic").value))
            return

        input(" 现在用手把车往前推 1 米左右（尽量走直线，别推快），\n"
              " 推到位后按 Enter ... ")
        rclpy.spin_once(self, timeout_sec=0.3)
        self._update_from_pose()

        print(" 2. 在【现在前轮的位置】再贴一条胶带")
        print(" 3. 用卷尺量两条胶带之间的距离")
        self._report()

    # ------------------------------------------------------------------
    def _report(self):
        print("")
        print("=" * 60)
        print(" 标定结果")
        print("=" * 60)
        print(" 里程计读数   : {:.4f} m".format(self.travel))
        print(" 航向变化     : {:+.2f}°  （应该接近 0，大了说明没走直）".format(
            math.degrees(self.yaw_delta)))
        print(" 横向偏移     : {:+.4f} m".format(self.lateral))
        print("")

        try:
            raw = input(" 请输入卷尺实测距离（米），直接回车跳过计算: ").strip()
        except EOFError:
            raw = ""
        if not raw:
            print(" 已跳过。建议 scale 需要实测距离才能算。")
            return

        try:
            actual = float(raw)
        except ValueError:
            print(" 输入不是数字，跳过。")
            return

        if self.travel < 0.05 or actual <= 0.0:
            print(" 数值太小，无法计算。")
            return

        scale = actual / self.travel
        print("")
        print(" ├─ 里程计读数 : {:.4f} m".format(self.travel))
        print(" ├─ 卷尺实测   : {:.4f} m".format(actual))
        print(" ├─ 相对误差   : {:+.2f}%".format(100.0 * (self.travel - actual) / actual))
        print(" └─ 建议 scale : {:.4f}".format(scale))
        print("")
        print(" 改这里（base_node 那段）:")
        print("   ~/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_bringup/launch/")
        print("       yahboomcar_bringup_R2_launch.py")
        print("   'linear_scale_x': {:.4f},".format(scale))
        print("   'linear_scale_y': {:.4f},".format(scale))
        print("")
        print(" ⚠️ 这个参数是乘还是除取决于 base_node_R2 的实现。")
        print("    改完【必须再标定一次验证】：读数更准了方向就对，")
        print("    更糟就把 scale 取倒数（{:.4f}）。".format(1.0 / scale))
        print("=" * 60)


def main(argv=None):
    rclpy.init(args=argv)
    node = OdomCalibrate()
    try:
        if node.push_mode:
            node.run_push()
        else:
            node.run_drive()
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

