#!/usr/bin/env bash
# 验证"低速限幅"生效：低速 + 大角速度指令，输出转角应该被限到 20° 左右。
# 完全隔离：输入输出都走测试话题，真车不动。

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

IN=/cmd_vel_tin
OUT=/cmd_vel_tout

pkill -f "input_topic:=${IN}" 2>/dev/null
sleep 1
nohup python3 -u "${SCRIPT_DIR}/cmd_vel_ackermann.py" --ros-args \
  -p "input_topic:=${IN}" -p "output_topic:=${OUT}" \
  -p wheelbase:=0.2681 -p max_steer_deg:=40.0 \
  -p low_speed:=0.12 -p low_speed_steer_deg:=20.0 \
  -p reverse_steer_deg:=22.0 -p max_steer_rate_dps:=45.0 \
  -p steer_deadband_deg:=1.5 \
  >/tmp/lowspeed_test.log 2>&1 </dev/null &
sleep 4
sed -n '1,2p' /tmp/lowspeed_test.log
echo

pub() {
  local m
  m="$(printf '{linear: {x: %s, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: %s}}' "$1" "$2")"
  timeout 5 ros2 topic pub --rate 20 "$IN" geometry_msgs/msg/Twist "$m" >/dev/null 2>&1
}
read_out() {
  timeout 3 ros2 topic echo "$OUT" --once --field linear.y 2>/dev/null \
    | grep -oE '^-?[0-9]+(\.[0-9]+)?' | head -1
}

printf '%-26s %-16s %-16s\n' "输入" "理论 δ（无限幅）" "实测输出"
check() {
  pub "$1" "$2" &
  sleep 3.2
  d="$(read_out)"
  deg="$(echo "${d:-0}" | awk '{printf "%.1f", $1*1000}')"
  th="$3"
  printf '%-26s %-16s %-16s\n' "v=$1 ω=$2" "$th" "${deg}°"
  wait
}

# 低速 + 大角速度：这是"到终点减速"时的典型组合
check 0.05 0.45   "atan(.45*.268/.05)=67°"
check 0.08 0.45   "atan(.45*.268/.08)=56°"
check 0.12 0.45   "atan(.45*.268/.12)=45°"
# 正常速度：应该放行到 40°
check 0.23 0.45   "atan(.45*.268/.23)=28°"
check 0.23 0.90   "atan(.90*.268/.23)=46°→40"

echo
pkill -f "input_topic:=${IN}" 2>/dev/null
echo "（测试节点已停，真车没收到任何指令）"
