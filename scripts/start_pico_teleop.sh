#!/usr/bin/env bash
# 一键启动 Pico 遥操作：后台 2 进程 + 前台 test_rt_teleop（std::cin 交互选仿真/型号）
#
# 用法:
#   bash scripts/start_pico_teleop.sh           # 前台遥操作，终端 stdin 交互
#   bash scripts/start_pico_teleop.sh --stop    # 停止后台 Pico 服务/发布
#   bash scripts/start_pico_teleop.sh --replay  # 发布节点用 CSV 回放
#   bash scripts/start_pico_teleop.sh --skip-grant-caps
#
# 环境变量: PICO_SERVICE, BUILD_DIR
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_SH="${ROOT}/scripts/pico_teleop_env.sh"
RUN_DIR="${ROOT}/.cache/pico_teleop"
BUILD_DIR="${BUILD_DIR:-${ROOT}/build}"
TELEOP_BIN="${BUILD_DIR}/tj_test/test_rt_teleop"
PICO_SERVICE="${PICO_SERVICE:-${HOME}/repos/XRoboToolkit-PC-Service/RoboticsService/bin/runService.sh}"
SERVICE_PID="${RUN_DIR}/pico_service.pid"
PUB_PID="${RUN_DIR}/pico_pub.pid"

REPLAY=0
DO_STOP=0
SKIP_CAPS=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --stop) DO_STOP=1; shift ;;
    --replay) REPLAY=1; shift ;;
    --skip-grant-caps) SKIP_CAPS=1; shift ;;
    -h|--help)
      sed -n '2,12p' "$0"
      exit 0
      ;;
    *)
      echo "[fail] 未知参数: $1" >&2
      exit 1
      ;;
  esac
done

resolve_python() {
  if command -v python3 >/dev/null 2>&1; then
    command -v python3
    return
  fi
  if command -v python >/dev/null 2>&1; then
    command -v python
    return
  fi
  echo "[fail] 未找到 python3/python，无法启动 pico_udp_publisher" >&2
  echo "       请安装 Python 3 或把 miniconda 加入 PATH" >&2
  exit 1
}

check_xrobot_sdk() {
  local py="$1"
  if "${py}" -c "import xrobotoolkit_sdk" >/dev/null 2>&1; then
    return 0
  fi
  echo "[fail] 缺少 Python 模块 xrobotoolkit_sdk（Pico 数据绑定）" >&2
  echo "       一键编译: bash scripts/install_xrobot_sdk.sh" >&2
  echo "       或手动: cd ~/repos/XRoboToolkit-PC-Service-Pybind && bash ../tj_control/scripts/install_xrobot_sdk.sh" >&2
  return 1
}

ensure_background_running() {
  local pf="$1"
  local name="$2"
  local log="$3"
  sleep 1
  if [[ ! -f "${pf}" ]]; then
    echo "[fail] ${name} 未写入 pid 文件" >&2
    exit 1
  fi
  local pid
  pid="$(cat "${pf}")"
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[fail] ${name} 启动后立即退出 (pid ${pid})，见日志:" >&2
    tail -n 8 "${log}" >&2 || true
    stop_background
    exit 1
  fi
}

source_env() {
  # shellcheck source=/dev/null
  export TJ_REPO="${ROOT}"
  source "${ENV_SH}"
}

resolve_rt_cpu() {
  if [[ -n "${TJ_RT_CPU:-}" ]]; then
    echo "${TJ_RT_CPU}"
    return
  fi
  local ncpu
  ncpu="$(nproc)"
  if (( ncpu >= 4 )); then
    echo $((ncpu - 2))
  else
    echo $((ncpu - 1))
  fi
}

bg_cpuset_excluding() {
  local rt_cpu="$1"
  local ncpu="$2"
  local list=""
  local i
  for ((i = 0; i < ncpu; ++i)); do
    if (( i != rt_cpu )); then
      list+="${i},"
    fi
  done
  echo "${list%,}"
}

kill_pidfile() {
  local pf="$1"
  local name="$2"
  if [[ -f "${pf}" ]]; then
    local pid
    pid="$(cat "${pf}")"
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
      echo "[ok] 已停止 ${name} (pid ${pid})"
    fi
    rm -f "${pf}"
  fi
}

stop_background() {
  kill_pidfile "${PUB_PID}" "pico_udp_publisher"
  kill_pidfile "${SERVICE_PID}" "Pico PC 服务"
}

if [[ "${DO_STOP}" -eq 1 ]]; then
  stop_background
  exit 0
fi

if [[ ! -f "${ENV_SH}" ]]; then
  echo "[fail] 缺少 ${ENV_SH}" >&2
  exit 1
fi

if [[ ! -f "${PICO_SERVICE}" ]]; then
  echo "[fail] Pico 服务不存在: ${PICO_SERVICE}" >&2
  exit 1
fi

if [[ ! -f "${TELEOP_BIN}" ]]; then
  echo "[fail] 未找到 ${TELEOP_BIN}，请先编译 test_rt_teleop" >&2
  exit 1
fi

if [[ -f "${SERVICE_PID}" ]] && kill -0 "$(cat "${SERVICE_PID}")" 2>/dev/null; then
  echo "[fail] Pico 后台进程已在运行，先执行: bash scripts/start_pico_teleop.sh --stop" >&2
  exit 1
fi

mkdir -p "${RUN_DIR}"
source_env

RT_CPU="$(resolve_rt_cpu)"
export TJ_RT_CPU="${RT_CPU}"
BG_CPUSET="$(bg_cpuset_excluding "${RT_CPU}" "$(nproc)")"
echo "[rt] 控制环绑核 cpu=${RT_CPU}；后台进程 cpuset=${BG_CPUSET}"

if [[ -r /sys/kernel/realtime ]]; then
  rt="$(cat /sys/kernel/realtime)"
  if [[ "${rt}" == "1" ]]; then
    echo "[ok] PREEMPT_RT 内核"
  else
    echo "[warn] 当前内核 realtime=${rt}，建议 PREEMPT_RT"
  fi
else
  echo "[warn] 未检测到 PREEMPT_RT 标记"
fi

if [[ "${SKIP_CAPS}" -eq 0 ]]; then
  if ! getcap "${TELEOP_BIN}" 2>/dev/null | grep -q 'cap_sys_nice'; then
    echo "[info] 授予 SCHED_FIFO 能力（需 sudo）..."
    bash "${ROOT}/tj_test/grant_rt_caps.sh" "${BUILD_DIR}"
  fi
fi

LOG_SERVICE="${RUN_DIR}/pico_service.log"
LOG_PUB="${RUN_DIR}/pico_pub.log"

echo "[1/3] 启动 Pico PC 服务 → ${LOG_SERVICE}"
taskset -c "${BG_CPUSET}" bash "${PICO_SERVICE}" >>"${LOG_SERVICE}" 2>&1 &
echo $! >"${SERVICE_PID}"
ensure_background_running "${SERVICE_PID}" "Pico PC 服务" "${LOG_SERVICE}"

PYTHON_CMD="$(resolve_python)"
echo "[info] UDP 发布 Python: ${PYTHON_CMD}"
check_xrobot_sdk "${PYTHON_CMD}" || exit 1

if [[ "${REPLAY}" -eq 1 ]]; then
  PUB_CMD=("${PYTHON_CMD}" -m python.teleop.pico_udp_publisher
    --replay-csv data/test_teleop_data/pico_record_20260615_220409.csv
    --trigger 1.0 --loop)
else
  PUB_CMD=("${PYTHON_CMD}" -m python.teleop.pico_udp_publisher)
fi

echo "[2/3] 启动 Pico UDP 发布 → ${LOG_PUB}"
taskset -c "${BG_CPUSET}" "${PUB_CMD[@]}" >>"${LOG_PUB}" 2>&1 &
echo $! >"${PUB_PID}"
ensure_background_running "${PUB_PID}" "pico_udp_publisher" "${LOG_PUB}"

cleanup_on_exit() {
  local code=$?
  echo ""
  echo "[stop] 停止后台 Pico 服务/发布..."
  stop_background
  exit "${code}"
}
trap cleanup_on_exit INT TERM

echo "[3/3] 前台 mv_control 遥操作（stdin：硬件/仿真 → URDF → 进入循环）"
echo ""

teleop_code=0
"${TELEOP_BIN}" || teleop_code=$?

stop_background
exit "${teleop_code}"
