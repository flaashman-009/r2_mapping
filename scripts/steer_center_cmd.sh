#!/usr/bin/env bash
# 让前轮回正（发一个小非零转向指令）
#
# 为什么不能用 0：
#   实测这台车的底盘固件把 linear.y == 0 当成"不更新转向"，
#   所以持续发 0 前轮会停在原地不动。必须发一个非零值。
#   而固件的最小转向步进是 1°，所以 0.001（=1°）就是能生效的最小值。
#
# 用法：
#   bash steer_center_cmd.sh            # 回到 +1°（默认）
#   bash steer_center_cmd.sh -0.001     # 回到 -1°
#   bash steer_center_cmd.sh 0.005      # 回到 +5°
#
# 验证：
#   ros2 topic echo /vel_raw --once --field linear.y
#   应该显示接近你发的值（单位是度）

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

EPS="${1:-0.001}"

echo "发转向指令 linear.y = ${EPS}（${EPS} x 1000 = $(python3 -c "print(${EPS}*1000)") 度）"
echo "持续 1.5 秒，确保送达 ..."

timeout 2 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: ${EPS}, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  > /dev/null 2>&1

sleep 1

echo "当前转向反馈："
timeout 10 ros2 topic echo /vel_raw --once --field linear.y 2>/dev/null | tail -1

echo ""
echo "（想再回正一点就差一下：bash steer_center_cmd.sh -${EPS}）"

