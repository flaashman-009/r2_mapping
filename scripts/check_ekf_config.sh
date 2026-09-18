#!/usr/bin/env bash
# 校验 EKF 参数文件里 odom0_config / imu0_config 的实际值
#
# 用途：改完 ekf_x1_x3.yaml 之后确认改动生效，
#       避免"以为改了其实没改"或"改错了位置"。

set -o pipefail

TARGETS=(
  "$HOME/yahboomcar_ros2_ws/software/library_ws/install/robot_localization/share/robot_localization/params/ekf_x1_x3.yaml"
  "$HOME/yahboomcar_ros2_ws/software/library_ws/src/robot_localization/params/ekf_x1_x3.yaml"
)

echo "=================================================="
echo " EKF 参数校验"
echo "=================================================="

for f in "${TARGETS[@]}"; do
  echo ""
  echo "文件: ${f}"
  if [[ ! -f "$f" ]]; then
    echo "  不存在"
    continue
  fi
  python3 - "$f" <<'PY'
import sys
import yaml

names = ["x", "y", "z", "roll", "pitch", "yaw",
         "vx", "vy", "vz", "vroll", "vpitch", "vyaw",
         "ax", "ay", "az"]

cfg = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
p = cfg["ekf_filter_node"]["ros__parameters"]

for key in ("odom0_config", "imu0_config", "odom0", "imu0", "world_frame"):
    print("  {:<14} = {}".format(key, p.get(key)))

odom = p.get("odom0_config") or []
imu = p.get("imu0_config") or []
if len(odom) == 15:
    on = [names[i] for i, v in enumerate(odom) if v]
    print("  -> odom0 提供 : {}".format(on or "无"))
if len(imu) == 15:
    on = [names[i] for i, v in enumerate(imu) if v]
    print("  -> imu0  提供 : {}".format(on or "无"))

# 重点提示
if len(odom) == 15 and (odom[0] or odom[1]):
    print("  ⚠️ odom0 仍在融合 x/y 位置（base_node 用错误航向积分出来的路径）")
elif len(odom) == 15:
    print("  ✅ odom0 已不融合位置，只用速度 —— EKF 会用 IMU 航向自己积分")
PY
done

echo ""
echo "=================================================="

