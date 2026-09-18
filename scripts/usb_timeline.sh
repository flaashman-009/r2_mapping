#!/usr/bin/env bash
# 还原 USB 外设的插拔时间线：雷达到底几点几分从总线上消失的

echo "=================================================="
echo " USB 外设时间线"
echo "=================================================="

echo
echo "[1] 所有 USB 设备的接入 / 断开 / 复位事件"
journalctl --since "2026-09-16 20:00" 2>/dev/null \
  | grep -iE 'usb [0-9]+-[0-9.:-]+.*(new |disconnect|reset|device descriptor|cannot enable)' \
  | head -50

echo
echo "[2] ttyUSB / ch34x / cp210x 驱动事件"
journalctl --since "2026-09-16 20:00" 2>/dev/null \
  | grep -iE 'ttyUSB|ch34x|cp210|ftdi|usbserial' | head -40

echo
echo "[3] 系统关机 / 重启 / 电源相关事件"
journalctl --since "2026-09-16 20:00" 2>/dev/null \
  | grep -iE 'shutdown|reboot|power|Starting .*System|Reached target.*(Shutdown|Reboot)' \
  | head -20

echo
echo "[4] 当前这次启动是从几点开始的"
who -b 2>/dev/null
uptime -s 2>/dev/null

echo
echo "[5] journal 覆盖的时间范围"
journalctl --no-pager -n1 2>/dev/null | head -1
