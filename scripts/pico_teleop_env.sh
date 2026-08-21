# shellcheck shell=bash
# Pico 遥操作公共环境（由 start_pico_teleop.sh 与各 tmux 窗 source）

if [[ -z "${TJ_REPO:-}" ]]; then
  TJ_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi
cd "${TJ_REPO}"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # ROS setup 会引用尚未定义的变量（如 AMENT_TRACE_SETUP_FILES）
  _ros_nounset=0
  [[ $- == *u* ]] && _ros_nounset=1
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
  (( _ros_nounset )) && set -u
  unset _ros_nounset
fi

SDK_LIB="${TJ_REPO}/TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/contrlSDK100343"
export LD_LIBRARY_PATH="${SDK_LIB}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
if [[ -d /opt/ros/jazzy/lib ]]; then
  export LD_LIBRARY_PATH="/opt/ros/jazzy/lib:${LD_LIBRARY_PATH}"
fi

export PYTHONPATH="${TJ_REPO}${PYTHONPATH:+:${PYTHONPATH}}"

# XRoboToolkit Python 绑定（pico_data_receiver / pico_udp_publisher 依赖）
XROBOT_PYBIND_DIR="${XROBOT_PYBIND_DIR:-${HOME}/repos/XRoboToolkit-PC-Service-Pybind}"
if [[ -d "${XROBOT_PYBIND_DIR}/lib" ]]; then
  export LD_LIBRARY_PATH="${XROBOT_PYBIND_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
_xrt_so=""
if [[ -d "${XROBOT_PYBIND_DIR}/build" ]]; then
  _xrt_so="$(find "${XROBOT_PYBIND_DIR}/build" -maxdepth 1 -name 'xrobotoolkit_sdk*.so' -print -quit 2>/dev/null || true)"
fi
if [[ -n "${_xrt_so}" ]]; then
  export PYTHONPATH="${XROBOT_PYBIND_DIR}/build${PYTHONPATH:+:${PYTHONPATH}}"
fi
unset _xrt_so

# 非交互 shell 可能没有 conda/python；优先 miniconda，再系统 python3
if [[ -d "${HOME}/miniconda3/bin" ]]; then
  export PATH="${HOME}/miniconda3/bin:${PATH}"
fi
if [[ -d "${HOME}/anaconda3/bin" ]]; then
  export PATH="${HOME}/anaconda3/bin:${PATH}"
fi
