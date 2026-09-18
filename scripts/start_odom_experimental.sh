#!/usr/bin/env bash
# 实验：用 r2_odom 替换原厂 base_node_R2（所有参数可运行时修改）
#
# 为什么要换：
#   原厂 base_node_R2.cpp 的航向模型只信转向角 ——
#     omega = v * tan(steer) / wheelbase
#   而且 wheelbase 用出厂默认 0.25（实测 0.2681，差 7.24%），
#   且完全不考虑左右轮速度差（这台车右轮快 1.6-2.6%）。
#
#   r2_odom 保留阿克曼几何，但补了两个可标定项：
#     - steer_zero_deg    转向零位偏置
#     - yaw_bias_per_m    每米额外偏航（弧度/米），补偿轮速不对称
#   并且用中点法积分，比原厂矩形法准。
#
# 用法：
#   bash start_odom_experimental.sh
#   bash start_odom_experimental.sh --ros-args -p yaw_bias_per_m:=0.12
#   bash start_odom_experimental.sh --stop      # 只停实验节点
#
# 运行时调参（另开终端）：
#   ros2 param set /r2_odom yaw_bias_per_m 0.12
#   ros2 param set /r2_odom linear_scale 0.94
#   ros2 param list /r2_odom
#
# 恢复原厂：
#   Ctrl-C 本脚本，然后 bash restart_hardware.sh

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

ODOM_PY="${R2_MAPPING_ROOT}/src/r2_mapping_tools/r2_mapping_tools/r2_odom.py"

# ---- 停掉原厂 base_node_R2 ----
stop_base_node() {
  local pids
  pids="$(pgrep -f base_node_R2 2>/dev/null || true)"
  if [[ -n "${pids}" ]]; then
    # shellcheck disable=SC2086
    kill ${pids} 2>/dev/null || true
    sleep 2
    pids="$(pgrep -f base_node_R2 2>/dev/null || true)"
    if [[ -n "${pids}" ]]; then
      # shellcheck disable=SC2086
      kill -9 ${pids} 2>/dev/null || true
    fi
    echo "已停掉原厂 base_node_R2"
  else
    echo "原厂 base_node_R2 没在跑（可能已经切过了）"
  fi
}

# ---- 停掉我们的实验节点 ----
stop_r2_odom() {
  local pids
  pids="$(pgrep -f r2_odom.py 2>/dev/null || true)"
  if [[ -n "${pids}" ]]; then
    # shellcheck disable=SC2086
    kill ${pids} 2>/dev/null || true
    sleep 1
    echo "已停掉 r2_odom"
  else
    echo "r2_odom 没在跑"
  fi
}

if [[ "${1:-}" == "--stop" ]]; then
  stop_r2_odom
  echo ""
  echo "注意：原厂 base_node_R2 不会自动恢复。"
  echo "     要恢复整个硬件栈：bash ${SCRIPT_DIR}/restart_hardware.sh"
  exit 0
fi

if [[ ! -f "${ODOM_PY}" ]]; then
  echo "找不到 ${ODOM_PY}" >&2
  exit 1
fi

echo "=================================================="
echo " 切换到实验版里程计 r2_odom"
echo "=================================================="

stop_r2_odom >/dev/null 2>&1 || true
stop_base_node

# 确认底盘反馈还在（vel_raw 是 r2_odom 的唯一输入）
if ! timeout 6 ros2 topic list 2>/dev/null | grep -qx '/vel_raw'; then
  echo "警告：/vel_raw 不在话题列表里，底盘驱动可能没跑。" >&2
  echo "     先跑：bash ${SCRIPT_DIR}/restart_hardware.sh" >&2
fi

echo ""
echo "启动 r2_odom（Ctrl-C 退出）..."
echo "运行时调参： ros2 param set /r2_odom <参数> <值>"
echo "查看参数：   ros2 param list /r2_odom"
echo ""

exec python3 -u "${ODOM_PY}" "$@"

