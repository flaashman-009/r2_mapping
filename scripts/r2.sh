#!/usr/bin/env bash
# R2 一键启动 / 停止 / 状态
#
# 用法：
#   bash r2.sh                # 打开菜单（推荐第一次用）
#   bash r2.sh map            # 一键建图（硬件 + 滤波 + slam_toolbox）
#   bash r2.sh nav            # 一键导航（硬件 + AMCL + Nav2），默认 room_04
#   bash r2.sh nav room_05    # 指定地图
#   bash r2.sh hw             # 只起硬件
#   bash r2.sh teleop         # 手柄遥控
#   bash r2.sh rviz-map       # RViz 建图视图
#   bash r2.sh rviz-nav       # RViz 导航视图
#   bash r2.sh status         # 看当前状态
#   bash r2.sh log map        # 看某个组件的日志
#   bash r2.sh stop           # 停全部
#
# 所有后台进程的日志在 /tmp/r2_*.log

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

R2M="${R2_MAPPING_ROOT:-$HOME/r2_mapping}"
MAP_DEFAULT="${R2M}/maps/room_04.yaml"
NAV_PARAMS="${R2M}/config/nav_r2.yaml"
MAP_LAUNCH="${R2M}/src/r2_mapping_bringup/launch/mapping_slam_toolbox.launch.py"
RVIZ_MAP="${R2M}/src/r2_mapping_bringup/rviz/rviz_mapping_lite.rviz"
RVIZ_NAV="${R2M}/src/r2_mapping_bringup/rviz/rviz_localization.rviz"
TELEOP="${R2M}/src/r2_mapping_tools/r2_mapping_tools/ps2_teleop.py"

# ------------------------------------------------------------------ 工具
is_up() {
  case "$1" in
    hw)     pgrep -f Ackman_driver_R2 >/dev/null 2>&1 ;;
    slam)   pgrep -f async_slam_toolbox_node >/dev/null 2>&1 ;;
    filter) pgrep -f scan_filter_node >/dev/null 2>&1 ;;
    nav)    pgrep -f nav2_container >/dev/null 2>&1 ;;
    teleop) pgrep -f ps2_teleop.py >/dev/null 2>&1 ;;
    rviz)   pgrep -f rviz2 >/dev/null 2>&1 ;;
    *)      return 1 ;;
  esac
}

start_bg() {
  local name="$1" logname="$2"
  shift 2
  local log="/tmp/r2_${logname}.log"
  if is_up "$name"; then
    echo "  [$name] 已经在跑，跳过"
    return 0
  fi
  rm -f "$log"
  nohup setsid "$@" >"$log" 2>&1 </dev/null &
  echo "  [$name] 启动中 ...  日志：${log}"
}

wait_topic() {
  local topic="$1" secs="${2:-30}" i=0
  while [ "$i" -lt "$secs" ]; do
    if timeout 3 ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      echo "  ${topic} 就绪"
      return 0
    fi
    sleep 1
    i=$((i + 1))
  done
  echo "  ** 等待 ${topic} 超时（${secs}s）—— 看日志：bash $0 log <组件>"
  return 1
}

# ------------------------------------------------------------------ 动作
act_hw() {
  echo "=== 启动硬件（底盘 + 雷达 + IMU + EKF + TF）==="
  start_bg hw hw ros2 launch yahboomcar_nav laser_bringup_launch.py
  wait_topic /scan 40
  echo ""
  echo "  验证：ros2 topic hz /scan /odom /vel_raw"
}

act_map() {
  echo "=== 启动建图（硬件 + 扫描滤波 + slam_toolbox）==="
  echo "  launch：${MAP_LAUNCH}"
  start_bg slam map ros2 launch "${MAP_LAUNCH}"
  wait_topic /scan_filtered 50
  wait_topic /map 60
  echo ""
  echo "  下一步："
  echo "    1) bash $0 rviz-map     打开 RViz 看地图"
  echo "    2) bash $0 teleop       起手柄遥控"
  echo "    3) 走闭合路径，回到起点"
  echo "    4) ros2 launch yahboomcar_nav save_map_launch.py map_path:=${R2M}/maps/room_05"
}

act_nav() {
  local map="${1:-$MAP_DEFAULT}"
  if [ ! -f "$map" ]; then
    # 允许只给名字
    if [ -f "${R2M}/maps/${map}.yaml" ]; then
      map="${R2M}/maps/${map}.yaml"
    else
      echo "  ** 找不到地图：${map}"
      ls -1 "${R2M}/maps/"*.yaml 2>/dev/null | sed 's/^/     /'
      return 1
    fi
  fi

  echo "=== 启动导航 ==="
  echo "  地图  ：${map}"
  echo "  参数  ：${NAV_PARAMS}"

  start_bg hw hw ros2 launch yahboomcar_nav laser_bringup_launch.py
  wait_topic /odom 40

  start_bg nav nav ros2 launch yahboomcar_nav navigation_teb_launch.py \
    map:="$map" params_file:="$NAV_PARAMS"

  # map_server 发布 /map 说明地图加载成功
  wait_topic /map 60

  echo ""
  echo "  下一步（**必须做，否则规划器起不来**）："
  echo "    1) bash $0 rviz-nav       打开 RViz"
  echo "    2) 在 RViz 里点 2D Pose Estimate 设置初始位姿"
  echo "    3) 等 global_costmap 激活（终端不再刷 waiting for transform）"
  echo "    4) 点 2D Goal Pose 发目标"
  echo ""
  echo "  也可以用脚本设初始位姿（车在建图起点时）："
  echo "    ~/r2_mapping/scripts/set_initial_pose.sh 0 0 0"
}

act_teleop() {
  echo "=== 启动手柄遥控 ==="
  if is_up teleop; then
    echo "  [teleop] 已经在跑"
    echo "  想重启：pkill -f ps2_teleop.py"
    return 0
  fi
  echo "  ** 这是前台运行，Ctrl-C 退出 **"
  echo ""
  exec python3 -u "${TELEOP}"
}

act_rviz() {
  local cfg="$1" tag="$2"
  if [ -z "${DISPLAY:-}" ]; then
    echo "  ** 没有 DISPLAY 变量，RViz 起不来"
    echo "     如果你是通过 SSH 连的，需要在**小车的图形终端**里运行这个脚本"
    return 1
  fi
  start_bg rviz "$tag" rviz2 -d "$cfg"
  sleep 3
  echo "  RViz 已启动（配置：$(basename "$cfg")）"
}

act_status() {
  echo "=========================================="
  echo " R2 状态"
  echo "=========================================="
  printf "  %-10s %s\n" "硬件"   "$(is_up hw && echo '运行中' || echo '未运行')"
  printf "  %-10s %s\n" "扫描滤波" "$(is_up filter && echo '运行中' || echo '未运行')"
  printf "  %-10s %s\n" "slam"   "$(is_up slam && echo '运行中' || echo '未运行')"
  printf "  %-10s %s\n" "nav2"   "$(is_up nav && echo '运行中' || echo '未运行')"
  printf "  %-10s %s\n" "遥控"   "$(is_up teleop && echo '运行中' || echo '未运行')"
  printf "  %-10s %s\n" "RViz"   "$(is_up rviz && echo '运行中' || echo '未运行')"
  echo ""

  local port
  port="$(fuser /dev/myserial 2>/dev/null | tr -d ' ')"
  if [ -n "$port" ]; then
    echo "  串口 /dev/myserial : 被占用（PID ${port}）"
  else
    echo "  串口 /dev/myserial : 空闲"
  fi

  echo ""
  echo "  --- 节点 ---"
  timeout 12 ros2 node list 2>/dev/null | sed 's/^/    /' | head -25

  echo ""
  echo "  --- /cmd_vel 发布者（排查舵机抖动用）---"
  timeout 10 ros2 topic info /cmd_vel 2>/dev/null | sed 's/^/    /'

  echo ""
  echo "  日志：ls -1 /tmp/r2_*.log"
}

act_log() {
  local name="${1:-}"
  if [ -z "$name" ]; then
    echo "用法：bash $0 log <map|nav|hw|teleop>"
    echo ""
    ls -1 /tmp/r2_*.log 2>/dev/null | sed 's/^/  /'
    return 1
  fi
  local f="/tmp/r2_${name}.log"
  [ -f "$f" ] || { echo "  没有 ${f}"; return 1; }
  echo "=== tail -40 ${f} ==="
  tail -40 "$f"
}

act_stop() {
  echo "=== 停止全部 ==="
  pkill -f rviz2 2>/dev/null && echo "  RViz 已停"
  pkill -f ps2_teleop.py 2>/dev/null && echo "  遥控已停"
  pkill -f navigation_teb_launch 2>/dev/null && echo "  导航 launch 已停"
  pkill -f nav2_container 2>/dev/null && echo "  nav2 容器已停"
  sleep 2
  bash "${SCRIPT_DIR}/restart_hardware.sh" --no-start
  echo ""
  echo "完成。检查：bash $0 status"
}

# ------------------------------------------------------------------ 菜单
show_menu() {
  cat <<EOF
==========================================
 R2 一键启动
==========================================

  1) 建图        硬件 + 扫描滤波 + slam_toolbox
  2) 导航        硬件 + AMCL + Nav2（用 room_04）
  3) 只起硬件    底盘 + 雷达 + IMU + EKF
  4) 手柄遥控    ps2_teleop
  5) RViz 建图
  6) RViz 导航
  7) 查看状态
  8) 看日志
  9) 全部停止
  0) 退出

  也可以直接用命令：
    bash $0 map | nav | hw | teleop | status | stop

EOF
  printf " 请输入数字："
  read -r choice
  echo ""
  case "$choice" in
    1) act_map ;;
    2) act_nav ;;
    3) act_hw ;;
    4) act_teleop ;;
    5) act_rviz "$RVIZ_MAP" map ;;
    6) act_rviz "$RVIZ_NAV" nav ;;
    7) act_status ;;
    8)
      printf " 日志名（map/nav/hw/teleop）："
      read -r n
      act_log "$n"
      ;;
    9) act_stop ;;
    0) echo "退出" ;;
    *) echo "无效选择" ;;
  esac
}

# ------------------------------------------------------------------ 入口
case "${1:-}" in
  "")       show_menu ;;
  map)      act_map ;;
  nav)      act_nav "${2:-}" ;;
  hw)       act_hw ;;
  teleop)   act_teleop ;;
  rviz-map) act_rviz "$RVIZ_MAP" map ;;
  rviz-nav) act_rviz "$RVIZ_NAV" nav ;;
  status)   act_status ;;
  log)      act_log "${2:-}" ;;
  stop)     act_stop ;;
  -h|--help|help)
    sed -n '2,20p' "$0" | sed 's/^# \?//'
    ;;
  *)        echo "未知参数：$1"; echo "试试：bash $0" ;;
esac

