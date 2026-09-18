#!/usr/bin/env bash
# 验证"看门狗叫停 → 适配节点停车"这条链路。
#
# 安全设计：全程 linear.x = 0，车不会往前走，只会动前轮。

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

echo "=============================================="
echo " 看门狗止停链路测试（车不会动，只动前轮）"
echo "=============================================="

if ! pgrep -f "cmd_vel_[a]ckermann" >/dev/null; then
  echo "适配节点没在跑，先启动..."
  nohup python3 -u "${SCRIPT_DIR}/cmd_vel_ackermann.py" \
    >/tmp/ack_halt.log 2>&1 </dev/null &
  sleep 4
fi

pub_nav() {  # pub_nav <v> <w>
  local m
  m="$(printf '{linear: {x: %s, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: %s}}' "$1" "$2")"
  timeout 8 ros2 topic pub --rate 20 /cmd_vel_nav geometry_msgs/msg/Twist "$m" \
    >/dev/null 2>&1 &
  echo $!
}

pub_halt() {  # pub_halt true|false
  timeout 3 ros2 topic pub --once /cmd_vel_halt std_msgs/msg/Bool \
    "{data: $1}" >/dev/null 2>&1
}

read_steer() {
  timeout 4 ros2 topic echo /vel_raw --once --field linear.y 2>/dev/null \
    | grep -oE '^-?[0-9]+(\.[0-9]+)?' | head -1
}
read_cmd() {
  timeout 4 ros2 topic echo /cmd_vel --once --field linear.y 2>/dev/null \
    | grep -oE '^-?[0-9]+(\.[0-9]+)?' | head -1
}

echo
echo "[1] 持续发 /cmd_vel_nav: v=0, ω=0.5  （期望前轮 ≈ 30°）"
P="$(pub_nav 0.0 0.5)"
sleep 5
echo "    /cmd_vel.linear.y = $(read_cmd)  (= 目标角度)"
echo "    实际转角          = $(read_steer) 度"

echo
echo "[2] 发 halt = true （期望前轮回正到约 1°）"
pub_halt true
sleep 4
echo "    /cmd_vel.linear.y = $(read_cmd)"
echo "    实际转角          = $(read_steer) 度"

echo
echo "[3] 发 halt = false （期望前轮回到 ≈ 30°）"
pub_halt false
sleep 5
echo "    /cmd_vel.linear.y = $(read_cmd)"
echo "    实际转角          = $(read_steer) 度"

kill "$P" 2>/dev/null
echo
echo "--- 适配节点日志 ---"
tail -6 /tmp/ack_halt.log 2>/dev/null
echo
echo "（测试结束，车始终没有前进）"
