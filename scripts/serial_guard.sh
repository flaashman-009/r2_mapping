#!/usr/bin/env bash
# 底盘串口占用检查 —— 建图前的第一道闸门。
#
# /dev/myserial 同一时间只能被一个进程占用；本脚本只做检查，不杀进程。
#
# 用法：
#   ./scripts/serial_guard.sh            # 只报告
#   ./scripts/serial_guard.sh --strict   # 被占用或设备缺失时返回非零

set -o pipefail

PORT="${R2_SERIAL_PORT:-/dev/myserial}"
LIDAR_PORT="${R2_LIDAR_PORT:-/dev/ydlidar}"
STRICT=0
[[ "${1:-}" == "--strict" ]] && STRICT=1

status=0

echo "=================================================="
echo " 底盘串口检查：${PORT}"
echo "=================================================="

if [[ ! -e "${PORT}" ]]; then
  echo "[x] 设备不存在：${PORT}"
  echo "    可能原因：USB 没插好 / 底盘板未上电 / udev 规则没生效 / 控制板 USB 硬件故障"
  echo "    排查见 docs/05_troubleshooting.md 第 1 节"
  status=1
else
  echo "[+] 设备存在：$(ls -l "${PORT}")"
  if command -v udevadm >/dev/null 2>&1; then
    udevadm info -q property -n "${PORT}" 2>/dev/null \
      | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT)=' \
      | sed 's/^/    /'
  fi
fi

echo ""
echo "-- 占用进程 --"
occupied=0

if command -v fuser >/dev/null 2>&1 && [[ -e "${PORT}" ]]; then
  holders="$(fuser -v "${PORT}" 2>&1 || true)"
  if [[ -n "${holders}" ]]; then
    echo "${holders}" | sed 's/^/    /'
    occupied=1
  fi
elif command -v lsof >/dev/null 2>&1 && [[ -e "${PORT}" ]]; then
  holders="$(lsof "${PORT}" 2>/dev/null || true)"
  if [[ -n "${holders}" ]]; then
    echo "${holders}" | sed 's/^/    /'
    occupied=1
  fi
else
  echo "    （fuser/lsof 都没装，跳过；可安装 psmisc 或 lsof）"
fi

if [[ "${occupied}" -eq 0 ]]; then
  echo "    没有进程占用（正常）"
else
  status=1
fi

echo ""
echo "-- 会抢占串口的进程 --"
pattern='Ackman_driver_R2|r2_driver_monitor|r2_center_steer|rosmaster_main|wifi_rosmaster|r2_diff_driver'
if pgrep -af "${pattern}" 2>/dev/null; then
  echo "    ⚠️ 上面这些进程都会打开 ${PORT}，同一时间只能留一个。"
  status=1
else
  echo "    没有发现（正常）"
fi

echo ""
echo "-- LiDAR 串口 --"
if [[ -e "${LIDAR_PORT}" ]]; then
  echo "    [+] ${LIDAR_PORT} 存在"
else
  echo "    [x] ${LIDAR_PORT} 不存在，LiDAR 起不来"
  status=1
fi

echo ""
echo "=================================================="
if [[ "${status}" -eq 0 ]]; then
  echo " 结论：串口就绪"
else
  echo " 结论：存在问题，先解决再启动 driver"
fi
echo "=================================================="

if [[ "${STRICT}" -eq 1 ]]; then
  exit "${status}"
fi
exit 0

