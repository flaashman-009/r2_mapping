#!/usr/bin/env bash
# 给 AMCL 设置初始位姿。
#
# 用法：./scripts/set_initial_pose.sh X Y YAW_DEG
# 例：  ./scripts/set_initial_pose.sh 0.0 0.0 0.0
#
# 车必须摆在建图时的起点、朝向一致，否则 AMCL 会收敛到错误的位置。

set -o pipefail

if [[ $# -lt 3 ]]; then
  echo "用法：$0 X Y YAW_DEG" >&2
  echo "例：  $0 0.0 0.0 0.0" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh"

X="$1"
Y="$2"
YAW_DEG="$3"

read -r QZ QW <<<"$(python3 -c "import math; y=math.radians(${YAW_DEG}); print(math.sin(y/2), math.cos(y/2))")"

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
{header: {frame_id: map},
 pose: {
   pose: {position: {x: ${X}, y: ${Y}, z: 0.0},
          orientation: {z: ${QZ}, w: ${QW}}},
   covariance: [0.25, 0, 0, 0, 0, 0,
                0, 0.25, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0.25]}}
"

echo "已设置初始位姿：x=${X} y=${Y} yaw=${YAW_DEG}deg"
echo "验证：ros2 topic echo /amcl_pose --once"

