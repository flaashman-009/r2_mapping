#!/usr/bin/env bash
# 给舵机一个阶跃指令，记录实际转角随时间的变化。
# 用途：看舵机能不能达到指令角度、响应有多快。
# 用法： bash steer_step.sh <linear.y>   例： bash steer_step.sh 0.03   (=30度)

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

Y="${1:-0.03}"
F=/tmp/steer_step.txt

MSG="$(printf '{linear: {x: 0.0, y: %s, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' "$Y")"
echo "阶跃指令 linear.y = ${Y}  ( = $(echo "$Y" | awk '{printf "%.0f", $1*1000}') 度 )"

rm -f "$F"
timeout 9 ros2 topic echo /vel_raw --field linear.y >"$F" 2>/dev/null &
E=$!
sleep 1.5
timeout 5 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "$MSG" >/dev/null 2>&1
wait "$E" 2>/dev/null

echo "--- 实际转角随时间（每 0.5 秒取一个中位）---"
grep -E '^-?[0-9]' "$F" | awk '
{ v[NR]=$1 }
END{
  n=NR; i=1; s=1
  while (i<=n) {
    cnt=0; q=0
    for (j=i; j<i+5 && j<=n; j++) { a[cnt++]=v[j] }
    # 简单中位
    for (x=0;x<cnt;x++) for (y=x+1;y<cnt;y++) if (a[y]<a[x]) {t=a[x];a[x]=a[y];a[y]=t}
    printf "  t=%4.1fs  %6.1f 度\n", s*0.5, a[int(cnt/2)]
    i+=5; s++
  }
}'
echo "--- 最后 5 个采样 ---"
grep -E '^-?[0-9]' "$F" | tail -5
