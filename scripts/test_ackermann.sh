#!/usr/bin/env bash
# 验证 cmd_vel_ackermann 适配节点的换算和限幅。
# 车请架起来：会真的动前轮。
#
# 注意：必须"边发边录"。ros2 topic echo 启动就要 1 秒多，如果先发后读，
# 等读的时候指令已经停了超过看门狗（0.5 s），适配节点早把转向复位了，
# 读到的全是复位值 —— 这就是上一版测试全显示 1.0° 的原因。

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

echo "=== 启动适配节点 ==="
pkill -f "cmd_vel_[a]ckermann" 2>/dev/null
sleep 1
nohup python3 -u "${SCRIPT_DIR}/cmd_vel_ackermann.py" \
  --ros-args -p wheelbase:=0.2681 -p max_steer_deg:=40.0 \
  -p max_steer_rate_dps:=70.0 \
  >/tmp/ack_test.log 2>&1 </dev/null &
sleep 4
sed -n '1,3p' /tmp/ack_test.log
echo

check() {
  local v="$1" w="$2" note="$3"
  local f="/tmp/ack_out.txt" g="/tmp/ack_fb.txt"
  local msg
  # 用 printf 拼消息，别手写大括号 —— 少一个括号 ros2 会静默解析失败
  msg="$(printf '{linear: {x: %s, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: %s}}' "$v" "$w")"
  rm -f "$f" "$g"

  # 先起发布，等它进入稳态，**只在稳态窗口里录**。
  # （先录后发会录到看门狗复位后的值，那是上一版的坑）
  timeout 8 ros2 topic pub --rate 20 /cmd_vel_nav geometry_msgs/msg/Twist \
    "$msg" >/dev/null 2>&1 &
  local ppid=$!
  sleep 3.5
  timeout 1.5 ros2 topic echo /cmd_vel --field linear.y >"$f" 2>/dev/null &
  timeout 1.5 ros2 topic echo /vel_raw --field linear.y >"$g" 2>/dev/null &
  wait
  kill "$ppid" 2>/dev/null
  wait "$ppid" 2>/dev/null

  local deg
  deg="$(grep -E '^-?[0-9]' "$f" 2>/dev/null | sort -n \
         | awk '{a[NR]=$1} END{if(NR>0) printf "%.1f", a[int(NR/2)+1]*1000; else print "-"}')"
  local st
  st="$(grep -E '^-?[0-9]' "$g" 2>/dev/null | sort -n \
        | awk '{a[NR]=$1} END{if(NR>0) printf "%.1f", a[int(NR/2)+1]; else print "-"}')"
  printf '%-20s %-26s %-12s %-10s\n' "v=$v w=$w" "$note" "${deg}" "${st:-?}"
}

printf '%-20s %-26s %-12s %-10s\n' "输入" "期望" "输出(度)" "前轮实测"
check 0.23  0.0  "0 -> 强制 1 度"
check 0.23  0.3  "atan(wL/v)=19 度"
check 0.23  0.5  "30 度"
check 0.23 -0.5  "-30 度"
check 0.23  2.0  "限幅 40 度"
check 0.23 -2.0  "限幅 -40 度"

echo
echo "=== 适配节点日志 ==="
tail -2 /tmp/ack_test.log
pkill -f "cmd_vel_[a]ckermann" 2>/dev/null
echo "（适配节点已停）"
