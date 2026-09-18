#!/usr/bin/env bash
# 检查 /scan 的 frame_id 落在 TF 树的哪里
set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

echo "===== /scan 的 header ====="
timeout 8 ros2 topic echo /scan --once --field header 2>/dev/null

echo
echo "===== 扫描范围（角度分辨率 / 点数）====="
timeout 8 ros2 topic echo /scan --once 2>/dev/null | sed -n \
  -e '/angle_min/p' -e '/angle_max/p' -e '/angle_increment/p' \
  -e '/range_min/p' -e '/range_max/p' -e '/time_increment/p'

echo
echo "===== range 数组的实际点数 ====="
timeout 8 ros2 topic echo /scan --once --field ranges 2>/dev/null | head -3

echo
echo "===== 两个 laser 坐标系相对 base_link 的变换 ====="
for f in laser laser_link laser_frame; do
  echo "--- base_link -> $f ---"
  timeout 6 ros2 run tf2_ros tf2_echo base_link "$f" 2>&1 | head -5
done

echo
echo "===== URDF 里和 laser 有关的 link/joint ====="
urdf="$(ros2 param get /robot_state_publisher robot_description 2>/dev/null | head -c 200000)"
printf '%s' "${urdf}" | tr '>' '>\n' | grep -i -e laser -e 'frame_id' | head -20
