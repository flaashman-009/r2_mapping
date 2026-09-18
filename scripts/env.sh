#!/usr/bin/env bash
# 公共环境：ROS 环境、工作区 source、ROS_DOMAIN_ID。
# 其它脚本用 source 引入：  source "$(dirname "$0")/env.sh"
#
# 可覆盖的环境变量（在调用前 export 即可）：
#   ROS_DOMAIN_ID      默认 28
#   R2_MAPPING_ROOT    默认本仓库根目录
#   R2_YAHBOOM_LIB_WS  默认 /home/jetson/yahboomcar_ros2_ws/software/library_ws
#   R2_YAHBOOM_WS      默认 /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
#   R2_VENDOR_BRINGUP  原厂硬件 bringup 的 launch 文件
#
# 注意：ROS 的 setup.bash 与 `set -u` 不兼容，本文件不启用 nounset。

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-28}"
export ROBOT_TYPE="${ROBOT_TYPE:-r2}"
export RPLIDAR_TYPE="${RPLIDAR_TYPE:-4ROS}"

_R2_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export R2_MAPPING_ROOT="${R2_MAPPING_ROOT:-$(cd "${_R2_ENV_DIR}/.." && pwd)}"

export R2_YAHBOOM_LIB_WS="${R2_YAHBOOM_LIB_WS:-/home/jetson/yahboomcar_ros2_ws/software/library_ws}"
export R2_YAHBOOM_WS="${R2_YAHBOOM_WS:-/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws}"
export R2_VENDOR_BRINGUP="${R2_VENDOR_BRINGUP:-/home/jetson/closed_loop/laser_bringup_tg_launch.py}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  else
    echo "[env] 找不到 /opt/ros/humble/setup.bash —— 本脚本需要在 Jetson（ROS2 Humble）上运行" >&2
  fi
fi

for _r2_setup in \
  "${R2_YAHBOOM_LIB_WS}/install/setup.bash" \
  "${R2_YAHBOOM_WS}/install/setup.bash" \
  "${R2_MAPPING_ROOT}/install/setup.bash"
do
  if [[ -f "${_r2_setup}" ]]; then
    # shellcheck disable=SC1090
    source "${_r2_setup}"
  fi
done
unset _r2_setup

export R2_VENDOR_BRINGUP

