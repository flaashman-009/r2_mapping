#!/usr/bin/env bash
# 修 velocity_smoother 把转向夹成 0 的问题（运行时生效，不用重启导航）
#
# 背景：这台车转向走 linear.y，而 Nav2 velocity_smoother 默认
#       max_velocity = [0.5, 0.0, 2.5]，第二项是 0，会把转向夹掉。
#
# 用法：
#   bash fix_smoother.sh          # 改参数 + 验证
#   bash fix_smoother.sh only     # 只改参数不验证
#
# 注意：这是**运行时**修改，重启导航就没了。
#       永久生效要改 nav_r2.yaml（已经改好了，见文件末尾的 velocity_smoother 段）。

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

MAX="[0.3, 0.05, 0.0]"
MIN="[-0.3, -0.05, 0.0]"

echo "=========================================="
echo " 修 velocity_smoother 的转向限幅"
echo "=========================================="
echo ""
echo "--- 改前 ---"
timeout 12 ros2 param get /velocity_smoother max_velocity 2>&1 | tail -1
timeout 12 ros2 param get /velocity_smoother min_velocity 2>&1 | tail -1
echo ""

echo "--- 改成 max=${MAX}  min=${MIN} ---"
timeout 15 ros2 param set /velocity_smoother max_velocity "${MAX}" 2>&1 | tail -1
timeout 15 ros2 param set /velocity_smoother min_velocity "${MIN}" 2>&1 | tail -1

echo ""
echo "--- 改后 ---"
timeout 12 ros2 param get /velocity_smoother max_velocity 2>&1 | tail -1
timeout 12 ros2 param get /velocity_smoother min_velocity 2>&1 | tail -1

if [[ "${1:-}" == "only" ]]; then
  exit 0
fi

echo ""
echo "--- 验证（往 /cmd_vel_nav 发 0.02 转向，看 /cmd_vel 出来什么）---"
bash "${SCRIPT_DIR}/test_smoother.sh" 0.02

