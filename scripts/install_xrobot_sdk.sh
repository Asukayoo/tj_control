#!/usr/bin/env bash
# 编译 XRoboToolkit Python 绑定 xrobotoolkit_sdk（依赖系统 pybind11-dev + cmake）
#
# 用法: bash scripts/install_xrobot_sdk.sh [XRoboToolkit-PC-Service-Pybind 目录]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYBIND_DIR="${1:-${XROBOT_PYBIND_DIR:-${HOME}/repos/XRoboToolkit-PC-Service-Pybind}}"

if [[ ! -d "${PYBIND_DIR}" ]]; then
  echo "[fail] 未找到 ${PYBIND_DIR}" >&2
  echo "       请 clone: git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git" >&2
  exit 1
fi

if [[ ! -f "${PYBIND_DIR}/lib/libPXREARobotSDK.so" ]]; then
  echo "[fail] 缺少 ${PYBIND_DIR}/lib/libPXREARobotSDK.so" >&2
  echo "       请在该仓库内先执行 setup_ubuntu.sh 或按 README 准备 lib/" >&2
  exit 1
fi

if ! command -v cmake >/dev/null 2>&1; then
  echo "[fail] 需要 cmake" >&2
  exit 1
fi

if [[ ! -f /usr/lib/cmake/pybind11/pybind11Config.cmake ]] &&
   [[ ! -f /usr/share/cmake/pybind11/pybind11Config.cmake ]]; then
  echo "[fail] 需要 pybind11-dev（例如: sudo apt install pybind11-dev）" >&2
  exit 1
fi

PYTHON_CMD=""
if command -v python3 >/dev/null 2>&1; then
  PYTHON_CMD="$(command -v python3)"
elif [[ -x "${HOME}/miniconda3/bin/python3" ]]; then
  PYTHON_CMD="${HOME}/miniconda3/bin/python3"
else
  echo "[fail] 未找到 python3" >&2
  exit 1
fi

BUILD_DIR="${PYBIND_DIR}/build"
echo "[build] ${PYBIND_DIR} → ${BUILD_DIR}"
cmake -S "${PYBIND_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE="${PYTHON_CMD}"
cmake --build "${BUILD_DIR}" -j"$(nproc)"

export LD_LIBRARY_PATH="${PYBIND_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONPATH="${BUILD_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
if ! "${PYTHON_CMD}" -c "import xrobotoolkit_sdk as xrt; print('[ok] xrobotoolkit_sdk', xrt.__file__)"; then
  echo "[fail] 编译完成但 import 失败" >&2
  exit 1
fi

echo "[ok] 请将以下变量写入环境（start_pico_teleop.sh 已通过 pico_teleop_env.sh 自动设置）:"
echo "     export XROBOT_PYBIND_DIR=\"${PYBIND_DIR}\""
