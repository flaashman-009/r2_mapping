#!/usr/bin/env bash
# 检查记录仪需要的 Python 库，并准备好持久化日志目录
set -o pipefail

echo "===== Python 库 ====="
python3 - <<'PY'
for name in ("numpy", "scipy", "cv2", "tf2_ros", "rclpy"):
    try:
        mod = __import__(name)
        ver = getattr(mod, "__version__", "(无版本号)")
        print("  {:<10} OK   {}".format(name, ver))
    except Exception as exc:
        print("  {:<10} 缺失 ({})".format(name, exc))
PY

echo
echo "===== 日志目录 ====="
mkdir -p "$HOME/r2_mapping/logs"
ls -ld "$HOME/r2_mapping/logs"

echo
echo "===== 磁盘 ====="
df -h "$HOME" | tail -1

echo
echo "===== 已有的历史记录 ====="
ls -lt "$HOME/r2_mapping/logs" 2>/dev/null | head -10
