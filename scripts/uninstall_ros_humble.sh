#!/usr/bin/env bash
# 卸载 ROS 2 Humble；不安装 Jazzy。需 sudo。
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "请用: sudo $0"
  exit 1
fi

echo "[1/4] 卸载 ros-humble-* 包..."
mapfile -t pkgs < <(dpkg -l 'ros-humble-*' 2>/dev/null | awk '/^ii/{print $2}')
if ((${#pkgs[@]} > 0)); then
  apt-get remove --purge -y "${pkgs[@]}"
else
  echo "  无已安装的 ros-humble-* 包"
fi

echo "[2/4] autoremove..."
apt-get autoremove --purge -y

echo "[3/4] 删除 /opt/ros/humble（若存在）..."
rm -rf /opt/ros/humble
if [[ -d /opt/ros ]] && [[ -z "$(ls -A /opt/ros 2>/dev/null)" ]]; then
  rmdir /opt/ros || true
fi

echo "[4/4] 禁用 ROS apt 源（若存在）..."
for f in /etc/apt/sources.list.d/ros-fish.sources /etc/apt/sources.list.d/ros2*.list \
         /etc/apt/sources.list.d/ros-fish.list /etc/apt/sources.list.d/ros-fish.list.distUpgrade; do
  if [[ -f "$f" ]]; then
    mv -f "$f" "${f}.disabled" 2>/dev/null || true
    echo "  已处理 $f"
  fi
done
apt-get update -qq || true

echo
echo "完成。请开新终端验证："
echo "  echo \"\$ROS_DISTRO\"; which ros2; ls /opt/ros 2>&1"
echo "（~/.bashrc 中的 humble source 已去掉；暂不安装 Jazzy）"
