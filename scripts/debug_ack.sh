#!/usr/bin/env bash
# 排查适配节点为什么没反应：看两端的原始数据

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

pkill -f "cmd_vel_[a]ckermann" 2>/dev/null
sleep 1
nohup python3 -u "${SCRIPT_DIR}/cmd_vel_ackermann.py" >/tmp/ack_dbg.log 2>&1 </dev/null &
sleep 5

echo "=== 适配节点进程 ==="
pgrep -af "cmd_vel_[a]ckermann" | head -2

echo
echo "=== /cmd_vel_nav 图信息（订阅者应为 cmd_vel_ackermann）==="
ros2 topic info /cmd_vel_nav

echo
echo "=== /cmd_vel 图信息（发布者应为 cmd_vel_ackermann）==="
ros2 topic info /cmd_vel

echo
echo "=== 手动发一条，看 ros2 topic pub 的报错 ==="
MSG="$(printf '{linear: {x: %s, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: %s}}' 0.23 0.5)"
echo "消息内容: $MSG"
timeout 6 ros2 topic pub --rate 10 -t 10 /cmd_vel_nav geometry_msgs/msg/Twist "$MSG" 2>&1 | tail -5

echo
echo "=== 适配节点日志 ==="
cat /tmp/ack_dbg.log

echo
echo "=== 适配节点输出（读 /cmd_vel）==="
timeout 5 ros2 topic echo /cmd_vel --once 2>&1 | head -12

pkill -f "cmd_vel_[a]ckermann" 2>/dev/null
echo "（已停）"
