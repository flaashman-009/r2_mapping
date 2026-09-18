#!/usr/bin/env bash
# velocity_smoother 放行测试：一次发 x / y / z 三个分量，看它放行哪些
#
# 用法：bash test_smoother2.sh
#
# **会同时发前进速度！车会往前走 0.1 m/s，持续约 3 秒（约 30 cm）。**
# 请确保前方有空间。只想测转向就把 X 改成 0。

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

X="${1:-0.10}"      # 前进速度，默认 0.1（想不前进就传 0）
Y="${2:-0.02}"      # 转向（linear.y），0.02 = 20 度
Z="${3:-0.50}"      # 角速度，这台车不用，用来验证平滑器放不放行
DUR="${4:-3}"       # 持续秒数

OUT=/tmp/smoother2.txt
IN=/tmp/smoother2_in.txt

echo "=========================================="
echo " velocity_smoother 放行测试"
echo "=========================================="
echo " 发送： x=${X}  y=${Y}  z=${Z}"
echo " 持续 ${DUR} 秒（前进约 $(awk "BEGIN{printf \"%.1f\", ${X}*${DUR}*100}") cm）"
echo ""

rm -f "$OUT" "$IN"

echo "--- 前置检查：/odom 是否健康（平滑器可能要它）---"
if timeout 6 ros2 topic list 2>/dev/null | grep -qx /odom; then
  echo "    /odom 存在"
  timeout 6 ros2 topic hz /odom 2>/dev/null | grep average | head -1 | sed 's/^/    /'
else
  echo "    ** /odom 不存在 —— 平滑器可能因此不发指令"
fi
echo ""

# 录 /cmd_vel 的完整内容
timeout 12 ros2 topic echo /cmd_vel >"$OUT" 2>/dev/null &
E1=$!
# 同时录 /cmd_vel_nav 确认自己的指令确实发出去了
timeout 12 ros2 topic echo /cmd_vel_nav --field linear.y >"$IN" 2>/dev/null &
E2=$!
sleep 1

timeout "${DUR}" ros2 topic pub --rate 20 /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: ${X}, y: ${Y}, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${Z}}}" \
  >/dev/null 2>&1

sleep 2
wait $E1 2>/dev/null
wait $E2 2>/dev/null

echo "--- 我发出去的 /cmd_vel_nav.linear.y 去重（前 5 个）---"
grep -E '^-?[0-9]' "$IN" | sort -u | head -5

echo ""
echo "--- /cmd_vel 收到的内容（前 12 行）---"
head -12 "$OUT"

echo ""
echo "--- /cmd_vel 里各分量的去重值 ---"
for f in "linear:" "x:" "y:" "z:" "angular:"; do :; done
echo "  linear.x 的值："
grep -A3 "^linear:" "$OUT" | grep -E '^\s+x:' | awk '{print $2}' | sort -u | head -5 | sed 's/^/    /'
echo "  linear.y 的值："
grep -A3 "^linear:" "$OUT" | grep -E '^\s+y:' | awk '{print $2}' | sort -u | head -5 | sed 's/^/    /'
echo "  angular.z 的值："
grep -A4 "^angular:" "$OUT" | grep -E '^\s+z:' | awk '{print $2}' | sort -u | head -5 | sed 's/^/    /'

echo ""
echo "=========================================="
echo " 判读"
echo "=========================================="
echo "  发出去的是 x=${X}  y=${Y}  z=${Z}"
echo "  上面三个列表说明了平滑器放行了哪些分量。"
echo "  如果 x 和 z 有值、只有 y 是 0 → 平滑器结构上就不处理 linear.y"
