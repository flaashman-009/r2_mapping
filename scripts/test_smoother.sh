#!/usr/bin/env bash
# 测试 velocity_smoother 会不会把 linear.y（转向）吃掉
#
# 背景：这台车的转向走 linear.y，而 Nav2 的 velocity_smoother 默认
# max_velocity = [0.5, 0.0, 2.5] —— 第二项（y）是 0，会把转向夹成 0。
#
# 做法：往 /cmd_vel_nav 发一个纯转向指令（linear.x=0，车不会前进），
#       同时录 /cmd_vel 的输出，对比两者。
#
# **只发转向，车不会移动**，但前轮会打一次方向（约 20°）。
# 请确保前轮周围没人/没东西。

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

IN_Y="${1:-0.02}"      # 0.02 = 20 度
OUT=/tmp/smoother_out.txt

echo "=========================================="
echo " velocity_smoother 测试"
echo "=========================================="
echo " 往 /cmd_vel_nav 发：linear.x=0  linear.y=${IN_Y}"
echo " 同时录 /cmd_vel（平滑后）"
echo ""

# 先看一眼平滑器的生命周期状态（没激活就不会转发）
echo "--- 生命周期 ---"
timeout 8 ros2 lifecycle get /velocity_smoother 2>&1 | head -1
echo ""

rm -f "$OUT"
timeout 14 ros2 topic echo /cmd_vel --field linear.y >"$OUT" 2>/dev/null &
ECHO_PID=$!
sleep 1

# 持续发 8 秒（比 velocity_timeout 长得多）
timeout 8 ros2 topic pub --rate 20 /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: ${IN_Y}, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  >/dev/null 2>&1

sleep 2
wait "$ECHO_PID" 2>/dev/null

echo "--- /cmd_vel 收到的值（前 10 个）---"
grep -E '^-?[0-9]' "$OUT" | head -10

echo ""
echo "--- 去重后的值 ---"
grep -E '^-?[0-9]' "$OUT" | sort -u | head -10

N="$(grep -E '^-?[0-9]' "$OUT" 2>/dev/null | wc -l)"
N="${N:-0}"
echo ""
echo "--- 统计：共收到 ${N} 条 ---"

if [ "$N" -eq 0 ]; then
  echo "  ** /cmd_vel 没数据 —— 可能 velocity_smoother 没在跑，或者它不发空指令"
  echo "     检查：ros2 node list | grep velocity"
elif grep -qE '^0(\.0+)?$' "$OUT"; then
  echo "  >>> 全是 0：**velocity_smoother 把转向夹掉了**（max_velocity 第二项是 0）"
  echo "      这就是舵机行为异常的原因。修法：给 nav_r2.yaml 加 velocity_smoother 段"
else
  echo "  >>> 有非 0 值：平滑器没有吃掉转向，问题在别处"
fi

echo ""
echo "对照：发给它的是 ${IN_Y}"
