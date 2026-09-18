#!/usr/bin/env bash
# 校验 2026-09-16 这批改动是否都到位

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
R2M="${R2_MAPPING_ROOT:-$HOME/r2_mapping}"

echo "=============================================="
echo " 改动校验"
echo "=============================================="

echo
echo "[1] Python 语法"
for f in "${R2M}/scripts/cmd_vel_ackermann.py" \
         "${R2M}/scripts/lost_watchdog.py" \
         "${R2M}/src/r2_mapping_bringup/launch/navigation_r2.launch.py"; do
  if python3 -m py_compile "$f" 2>/dev/null; then
    echo "    OK   $(basename "$f")"
  else
    echo "    失败 $(basename "$f")"
  fi
done

echo
echo "[2] YAML 内容"
python3 - "$R2M/config/nav_r2.yaml" <<'PY'
import sys, yaml
p = sys.argv[1]
d = yaml.safe_load(open(p, encoding="utf-8"))
a = d["amcl"]["ros__parameters"]
c = d["controller_server"]["ros__parameters"]
t = c["FollowPath"]
lc = d["local_costmap"]["local_costmap"]["ros__parameters"]
gc = d["global_costmap"]["global_costmap"]["ros__parameters"]

def line(label, ok, val):
    print("    {} {:<38} {}".format("OK " if ok else "!! ", label, val))

line("AMCL scan_topic", a["scan_topic"] == "scan_filtered", a["scan_topic"])
line("AMCL max_beams", a["max_beams"] >= 150, a["max_beams"])
line("AMCL laser_max_range", a["laser_max_range"] <= 20, a["laser_max_range"])
line("TEB footprint_model.type", t.get("footprint_model.type") == "polygon",
     t.get("footprint_model.type"))
line("TEB min_turning_radius", "min_turning_radius" in t,
     t.get("min_turning_radius"))
line("TEB weight_kinematics_turning_radius",
     "weight_kinematics_turning_radius" in t,
     t.get("weight_kinematics_turning_radius"))
line("TEB max_vel_theta", t.get("max_vel_theta", 9) <= 0.8,
     t.get("max_vel_theta"))
line("TEB acc_lim_theta", t.get("acc_lim_theta", 9) <= 1.0,
     t.get("acc_lim_theta"))
line("TEB homotopy 关掉", t.get("enable_homotopy_class_planning") is False,
     t.get("enable_homotopy_class_planning"))
line("controller min_y_velocity_threshold",
     c["min_y_velocity_threshold"] <= 0.01,
     c["min_y_velocity_threshold"])
line("backup_speed", abs(d["behavior_server"]["ros__parameters"]["backup"]
                         ["backup_speed"]) <= 0.3,
     d["behavior_server"]["ros__parameters"]["backup"]["backup_speed"])
line("local costmap footprint", "footprint" in lc, lc.get("footprint"))
line("global costmap footprint", "footprint" in gc, gc.get("footprint"))
line("local scan 用 filtered",
     lc["obstacle_layer"]["scan"]["topic"] == "/scan_filtered",
     lc["obstacle_layer"]["scan"]["topic"])
line("global scan 用 filtered",
     gc["obstacle_layer"]["scan"]["topic"] == "/scan_filtered",
     gc["obstacle_layer"]["scan"]["topic"])
line("controller_frequency <= 12", c["controller_frequency"] <= 12.0,
     c["controller_frequency"])
line("TEB max_vel_theta <= 0.5", t.get("max_vel_theta", 9) <= 0.5,
     t.get("max_vel_theta"))
line("goal_checker 已放宽",
     c["goal_checker"]["xy_goal_tolerance"] >= 0.28,
     "{} / {}".format(c["goal_checker"]["xy_goal_tolerance"],
                      c["goal_checker"]["yaw_goal_tolerance"]))
print("    TEB 参数总数 {}".format(len(t)))
PY

echo
echo "[3] launch 里的关键行"
L="${R2M}/src/r2_mapping_bringup/launch/navigation_r2.launch.py"
for kw in "cmd_vel_nav" "scan_filter_node" "lost_watchdog" \
          "steer_deadband" "lost_watchdog," ; do
  printf '    %-22s %s 次\n' "$kw" "$(grep -c "$kw" "$L")"
done

echo
echo "[4] 转向适配节点的新参数"
for kw in "steer_deadband_deg" "halt_topic" "max_steer_rate_dps" ; do
  printf '    %-22s %s 次\n' "$kw" \
    "$(grep -c "$kw" "${R2M}/scripts/cmd_vel_ackermann.py")"
done

echo
echo "=============================================="
