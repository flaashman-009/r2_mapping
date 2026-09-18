#!/usr/bin/env bash
# 把转向通道彻底问清楚：linear.y 和 angular.z 到底谁在驱动前轮
# 全程 linear.x = 0，车不会走，只动前轮。

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

pub() {  # pub <linear.y> <angular.z>
  timeout 4 ros2 topic pub --rate 20 -t 30 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: $1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $2}}" \
    >/dev/null 2>&1
}
rd() {
  timeout 5 ros2 topic echo /vel_raw --once --field linear.y 2>/dev/null \
    | grep -oE '^-?[0-9]+(\.[0-9]+)?' | head -1
}
rdx() {
  timeout 5 ros2 topic echo /vel_raw --once --field linear.x 2>/dev/null \
    | grep -oE '^-?[0-9]+(\.[0-9]+)?' | head -1
}

echo "=== 驱动健康检查 ==="
echo "  驱动进程数： $(pgrep -cf 'Ackman_[d]river')"
echo "  /vel_raw 的 vx = $(rdx)   (车静止应为 0.0)"
echo "  串口占用： $(fuser /dev/myserial 2>&1 | tr -d '\n')"
echo

printf '%-34s %-14s\n' "动作" "实际转角(度)"

printf '%-34s %-14s\n' "① baseline（什么都不发）" "$(rd)"
pub 0.02 0.0;  sleep 1.0; printf '%-34s %-14s\n' "② linear.y=0.02" "$(rd)"
pub -0.02 0.0; sleep 1.0; printf '%-34s %-14s\n' "③ linear.y=-0.02" "$(rd)"
pub 0.0 0.0;   sleep 1.0; printf '%-34s %-14s\n' "④ linear.y=0, angular.z=0" "$(rd)"
pub 0.0 0.5;   sleep 1.0; printf '%-34s %-14s\n' "⑤ angular.z=0.5（y=0）" "$(rd)"
pub 0.0 -0.5;  sleep 1.0; printf '%-34s %-14s\n' "⑥ angular.z=-0.5（y=0）" "$(rd)"
pub 0.001 0.0; sleep 1.0; printf '%-34s %-14s\n' "⑦ 回正 linear.y=0.001" "$(rd)"
echo
echo "判读："
echo "  ②③ 动 而 ⑤⑥ 不动  -> 转向走 linear.y（我上一轮的结论是错的）"
echo "  ⑤⑥ 动 而 ②③ 不动  -> 转向走 angular.z"
echo "  都不动              -> 驱动/串口/底盘供电有问题"
