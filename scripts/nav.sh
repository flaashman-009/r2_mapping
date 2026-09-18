#!/usr/bin/env bash
# R2 导航一键启动 —— 一条命令把导航跑起来
#
# 用法：
#   bash nav.sh                 # 用 room_04（最新那张图）
#   bash nav.sh room_03         # 用指定地图（名字就够，不用写路径）
#   bash nav.sh /path/to/x.yaml # 也可以给完整路径
#   bash nav.sh stop            # 停止全部
#   bash nav.sh status          # 看状态
#
# 它会依次做：
#   1. 清掉旧的残留进程（避免两个驱动抢串口）
#   2. 起硬件，等 /odom 就绪
#   3. 起 Nav2（AMCL + 规划 + 控制），等 /map 就绪
#   4. 开 RViz（导航视图）
#   5. 打印下一步该干什么
#
# 日志在 /tmp/r2_hw.log 和 /tmp/r2_nav.log

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

R2M="${R2_MAPPING_ROOT:-$HOME/r2_mapping}"
# 2026-09-18：默认地图换成 room_05（9-17 重录，覆盖率更高、墙更锐利）
DEFAULT_MAP="${R2M}/maps/room_05.yaml"
NAV_PARAMS="${R2M}/config/nav_r2.yaml"
RVIZ_CFG="${R2M}/src/r2_mapping_bringup/rviz/rviz_localization.rviz"
# 2026-09-15：改用本项目自己的导航 launch，**不带 velocity_smoother**。
# Nav2 自带的 navigation_launch.py 会启动 velocity_smoother，而它按
# "转向 = angular.z" 设计，会把这台车的 linear.y（转向）丢掉，
# 导致转向指令到不了底盘（实测：发 0.02 进去，出来是 0）。
# 详见 src/r2_mapping_bringup/launch/navigation_r2.launch.py 的注释。
NAV_LAUNCH="${R2M}/src/r2_mapping_bringup/launch/navigation_r2.launch.py"

# ---------------------------------------------------------------- 清理模式
# 停止导航时要杀掉的进程。两个注意点：
#   1. 模式里的 [x] 是防止 pkill 把自己的命令行也匹配上
#   2. launch 文件名是 navigation_r2.launch.py（**点号**），
#      写成下划线会匹配不到，父进程活下来就会不停重启被杀的节点 —— 
#      2026-09-17 踩过：留下过两个 controller_server 同时发 /cmd_vel_nav
NAV_KILL_PATTERNS=(
  "navigation_r2[._]launch"
  "navigation_teb_[l]launch"
  "laser_bringup_[l]launch"
  "cmd_vel_[a]ckermann"
  "lost_[w]atchdog"
  "localization_[g]uard"
  "scan_[f]ilter_node"
  "nav_[w]atch"
  "/opt/ros/humble/lib/nav2_[a]"
  "/opt/ros/humble/lib/nav2_[b]"
  "/opt/ros/humble/lib/nav2_[c]"
  "/opt/ros/humble/lib/nav2_[l]"
  "/opt/ros/humble/lib/nav2_[m]"
  "/opt/ros/humble/lib/nav2_[p]"
  "/opt/ros/humble/lib/nav2_[s]"
  "nav2_[c]ontainer"
  "costmap_[c]onverter"
)

kill_nav() {
  local p
  for p in "${NAV_KILL_PATTERNS[@]}"; do
    pkill -f "$p" 2>/dev/null && echo "    停了：${p}"
  done
}

MAP_FILE=""

# ------------------------------------------------------------------ 工具
wait_topic() {
  local topic="$1" secs="${2:-40}" i=0
  while [ "$i" -lt "$secs" ]; do
    if timeout 3 ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      echo "    ✓ ${topic} 就绪"
      return 0
    fi
    # 2026-09-18 修：ROS 发现服务（daemon）抖动时，topic list 会**漏报**
    # 已经在正常发布的话题 —— 实测 /odom 稳定跑在 10 Hz，但 nav.sh 卡在
    # "等 /odom 超时" 45 秒。这里加两条退路：
    #   ① 每 10 秒重启一次 daemon，逼它重新发现
    #   ② 直接试着收一条数据，能收到就算就绪
    if [ $((i % 10)) -eq 5 ]; then
      ros2 daemon stop >/dev/null 2>&1
    fi
    if timeout 6 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      echo "    ✓ ${topic} 就绪（topic list 漏报，但数据能收到）"
      return 0
    fi
    sleep 1
    i=$((i + 1))
  done
  echo "    ✗ 等 ${topic} 超时（${secs}s）"
  echo "      排查：ros2 daemon stop; sleep 2; ros2 topic list | grep ${topic}"
  echo "            如果话题在、数据也有，直接手动起 Nav2："
  echo "            ros2 launch ~/r2_mapping/src/r2_mapping_bringup/launch/navigation_r2.launch.py \\"
  echo "              map:=~/r2_mapping/maps/room_05.yaml"
  return 1
}

resolve_map() {
  local m="$1"
  if [ -f "$m" ]; then
    MAP_FILE="$m"
  elif [ -f "${R2M}/maps/${m}.yaml" ]; then
    MAP_FILE="${R2M}/maps/${m}.yaml"
  elif [ -f "${R2M}/maps/${m}" ]; then
    MAP_FILE="${R2M}/maps/${m}"
  else
    return 1
  fi
  return 0
}

# ------------------------------------------------------------------ 状态
show_status() {
  echo "=========================================="
  echo " 导航状态"
  echo "=========================================="
  printf "  %-12s %s\n" "硬件" "$(pgrep -f Ackman_driver_R2 >/dev/null && echo 运行中 || echo 未运行)"
  # 新 launch 不用 nav2_container（节点是独立进程），所以直接看 controller_server
  printf "  %-12s %s\n" "Nav2" "$(pgrep -f controller_server >/dev/null && echo 运行中 || echo 未运行)"
  printf "  %-12s %s\n" "RViz" "$(pgrep -f rviz2 >/dev/null && echo 运行中 || echo 未运行)"
  echo ""

  local p
  p="$(fuser /dev/myserial 2>/dev/null | tr -d ' ')"
  if [ -n "$p" ]; then
    echo "  串口 /dev/myserial : 被占用（PID ${p}）"
  else
    echo "  串口 /dev/myserial : 空闲"
  fi
  echo ""
  echo "  /cmd_vel 发布者（应该只有 1 个）："
  timeout 10 ros2 topic info /cmd_vel 2>/dev/null | sed 's/^/    /'
  echo ""
  local mw
  # 只接受纯数字，避免把 "topic does not appear to be published" 之类的
  # 提示当成宽度（ros2 topic echo 的警告会混进 stdout）
  mw="$(timeout 8 ros2 topic echo /map --once --field info.width 2>/dev/null \
        | grep -E '^[0-9]+$' | head -1)"
  if [ -n "$mw" ]; then
    echo "  地图加载：宽度 ${mw} 格（说明 map_server 正常）"
  else
    echo "  地图加载：（/map 无数据 —— Nav2 可能没起来）"
  fi
  echo ""
  echo "  生命周期："
  for n in map_server amcl planner_server controller_server bt_navigator; do
    printf "    %-18s " "$n"
    timeout 6 ros2 lifecycle get "/$n" 2>&1 | head -1
  done
}

# ------------------------------------------------------------------ 停止
do_stop() {
  echo "=== 停止导航 ==="
  pkill -f rviz2 2>/dev/null && echo "  RViz 已停"

  # 2026-09-17 修：原来只杀 launch 父进程。但我们的 launch 是用 setsid 起的，
  # 父进程被杀后各节点会变成孤儿进程继续活着 —— 实测攒出过**两个
  # controller_server 同时往 /cmd_vel_nav 发指令**，命令互相打架。
  # 所以这里逐个把 Nav2 的各节点进程也杀掉。
  # 注意：模式里的 [x] 是防止 pkill 把自己的命令行也匹配上。
  for pat in "navigation_r2_[l]aunch" "navigation_teb_[l]aunch" \
             "laser_bringup_[l]aunch" \
             "cmd_vel_[a]ckermann" "scan_[f]ilter_node" "nav_[w]atch" \
             "/opt/ros/humble/lib/nav2_[a]" \
             "/opt/ros/humble/lib/nav2_[b]" \
             "/opt/ros/humble/lib/nav2_[c]" \
             "/opt/ros/humble/lib/nav2_[l]" \
             "/opt/ros/humble/lib/nav2_[m]" \
             "/opt/ros/humble/lib/nav2_[p]" \
             "/opt/ros/humble/lib/nav2_[s]" \
             "nav2_[c]ontainer" "costmap_[c]onverter"; do
    pkill -f "$pat" 2>/dev/null && echo "  停了：${pat}"
  done
  kill_nav
  sleep 2

  local n
  n="$(pgrep -cf '/opt/ros/humble/lib/nav2_' 2>/dev/null || echo 0)"
  echo "  剩余 Nav2 节点进程：${n}"

  bash "${SCRIPT_DIR}/restart_hardware.sh" --no-start
  echo ""
  echo "完成。"
}

# ------------------------------------------------------------------ 启动
do_start() {
  echo "=========================================="
  echo " R2 导航一键启动"
  echo "=========================================="
  echo "  地图：${MAP_FILE}"
  echo "  参数：${NAV_PARAMS}"
  echo ""

  # --- 1. 清残留 ---
  echo "[1/5] 清理旧进程 ..."
  pkill -f rviz2 2>/dev/null
  for pat in "navigation_r2_[l]aunch" "navigation_teb_[l]aunch" \
             "laser_bringup_[l]aunch" \
             "cmd_vel_[a]ckermann" "scan_[f]ilter_node" "nav_[w]atch" \
             "/opt/ros/humble/lib/nav2_[a]" \
             "/opt/ros/humble/lib/nav2_[b]" \
             "/opt/ros/humble/lib/nav2_[c]" \
             "/opt/ros/humble/lib/nav2_[l]" \
             "/opt/ros/humble/lib/nav2_[m]" \
             "/opt/ros/humble/lib/nav2_[p]" \
             "/opt/ros/humble/lib/nav2_[s]" \
             "nav2_[c]ontainer" "costmap_[c]onverter"; do
    pkill -f "$pat" 2>/dev/null
  done
  kill_nav
  sleep 2
  bash "${SCRIPT_DIR}/restart_hardware.sh" --no-start >/dev/null 2>&1
  echo "    完成"

  # --- 2. 硬件 ---
  echo "[2/5] 启动硬件（底盘 + 雷达 + IMU + EKF + TF）..."
  rm -f /tmp/r2_hw.log
  nohup setsid ros2 launch yahboomcar_nav laser_bringup_launch.py \
    >/tmp/r2_hw.log 2>&1 </dev/null &
  if ! wait_topic /odom 45; then
    echo ""
    echo "  ** 硬件没起来，看日志：tail -30 /tmp/r2_hw.log"
    return 1
  fi

  # --- 3. Nav2 ---
  echo "[3/5] 启动 Nav2（AMCL + 规划 + 控制）..."
  rm -f /tmp/r2_nav.log
  nohup setsid ros2 launch "${NAV_LAUNCH}" \
    map:="${MAP_FILE}" params_file:="${NAV_PARAMS}" \
    >/tmp/r2_nav.log 2>&1 </dev/null &
  if ! wait_topic /map 70; then
    echo ""
    echo "  ** Nav2 没起来，看日志：tail -40 /tmp/r2_nav.log"
    return 1
  fi

  # --- 4. RViz ---
  echo "[4/5] 启动 RViz ..."
  if [ -z "${DISPLAY:-}" ]; then
    echo "    ✗ 没有 DISPLAY 变量，RViz 起不来"
    echo "      （如果是 SSH 连的，请在小车的图形终端里跑这个脚本）"
  else
    nohup setsid rviz2 -d "${RVIZ_CFG}" >/tmp/r2_rviz.log 2>&1 </dev/null &
    sleep 3
    echo "    ✓ RViz 已启动"
  fi

  # --- 5. 提示 ---
  echo "[5/5] 完成"
  echo ""
  echo "=========================================="
  echo " 现在在 RViz 里做两步"
  echo "=========================================="
  echo ""
  echo "  ① 点 2D Pose Estimate 设初始位姿"
  echo "     在图上点车的实际位置，按住拖出箭头指向车头方向"
  echo ""
  echo "     2026-09-16 起：AMCL 会自动用 (0,0,0) 初始化，"
  echo "     所以地图会**立刻**显示出来，不用再盲点了。"
  echo "     只有当车实际不在建图起点 (0,0,0) 时，才需要用"
  echo "     2D Pose Estimate 拖一下把位姿修正到真实位置。"
  echo ""
  echo "     判断位姿对不对：激光点贴合地图上的黑色墙线"
  echo ""
  echo "  ② 点 2D Goal Pose 发目标"
  echo "     点目标位置，拖出箭头表示到达时的朝向，松手车就走"
  echo ""
  echo "  第一次发近一点（同房间 1-2 米），确认能停准再发远的"
  echo ""
  echo "=========================================="
  echo " 其他"
  echo "=========================================="
  echo "  状态： bash $0 status"
  echo "  停止： bash $0 stop"
  echo "  日志： tail -f /tmp/r2_nav.log"
  echo "  急停： fuser -k /dev/myserial"
}

# ------------------------------------------------------------------ 入口
case "${1:-}" in
  stop)
    do_stop
    ;;
  status)
    show_status
    ;;
  -h|--help|help)
    sed -n '2,18p' "$0" | sed 's/^# \?//'
    ;;
  "")
    if resolve_map "$DEFAULT_MAP"; then do_start; else
      echo "** 找不到默认地图 ${DEFAULT_MAP}"
      ls -1 "${R2M}/maps/"*.yaml 2>/dev/null | sed 's/^/   /'
      exit 1
    fi
    ;;
  *)
    if resolve_map "$1"; then
      do_start
    else
      echo "** 找不到地图：$1"
      echo "   可用的："
      ls -1 "${R2M}/maps/"*.yaml 2>/dev/null | sed 's/^/     /'
      exit 1
    fi
    ;;
esac
