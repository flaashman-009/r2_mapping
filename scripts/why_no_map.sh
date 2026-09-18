#!/usr/bin/env bash
# 排查"RViz 里看不到地图"

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

echo "=== [1] 关键进程 ==="
for p in "Ackman_[d]river" "ydlidar" "ekf_[n]ode" "map_[s]erver" \
         "nav2_[a]mcl" "controller_[s]erver" "[r]viz2" "cmd_vel_[a]ckermann" \
         "scan_[f]ilter"; do
  n="$(pgrep -cf "$p")"
  printf '    %-22s %s\n' "$p" "$n"
done

echo
echo "=== [2] 雷达设备 ==="
ls -l /dev/ydlidar 2>&1

echo
echo "=== [3] 话题（只看关键的）==="
for t in /scan /scan_filtered /map /odom /amcl_pose /cmd_vel /cmd_vel_nav; do
  printf '    %-16s ' "$t"
  if timeout 6 ros2 topic list 2>/dev/null | grep -qx "$t"; then
    echo "存在"
  else
    echo "缺失"
  fi
done

echo
echo "=== [4] map_server 生命周期 ==="
timeout 8 ros2 lifecycle get /map_server 2>&1 | head -2 | sed 's/^/    /'

echo
echo "=== [5] /map 发布者数 ==="
timeout 8 ros2 topic info /map 2>&1 | sed 's/^/    /'

echo
echo "=== [6] nav 日志里和 map 有关的行 ==="
grep -iE 'map_server|map_io|yaml_filename' /tmp/r2_nav.log 2>/dev/null | tail -10 | sed 's/^/    /'

echo
echo "=== [7] nav 日志最后 12 行 ==="
tail -12 /tmp/r2_nav.log 2>/dev/null | sed 's/^/    /'

echo
echo "=== [8] RViz 配置里的 Map 显示 ==="
CFG="${R2_MAPPING_ROOT:-$HOME/r2_mapping}/src/r2_mapping_bringup/rviz/rviz_localization.rviz"
grep -n -A6 'rviz_default_plugins/Map' "$CFG" 2>/dev/null | head -30 | sed 's/^/    /'
echo "    ---- 固定坐标系 ----"
grep -n -A3 'Fixed Frame' "$CFG" 2>/dev/null | head -6 | sed 's/^/    /'
