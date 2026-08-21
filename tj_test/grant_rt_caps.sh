#!/usr/bin/env bash
# 编译后执行；仅 MakeHardRtSessionOptions 需要 SCHED_FIFO
# 仿真/CSV 录制请用默认 Session（勿 mlock 大 recorder）
# 用法: bash tj_test/grant_rt_caps.sh [build_dir]
# 可在任意目录执行；未传参时自动查找 <repo>/build 或 build_refactor
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

resolve_build_dir() {
  if [[ $# -ge 1 && -n "${1}" ]]; then
    echo "$(cd "${1}" && pwd)"
    return
  fi
  for candidate in "${ROOT}/build" "${ROOT}/build_refactor"; do
    if [[ -f "${candidate}/tj_test/test_movj" ]]; then
      echo "${candidate}"
      return
    fi
  done
  echo "${ROOT}/build"
}

BUILD_DIR="$(resolve_build_dir "$@")"

for name in test_movj test_enable test_servo test_rt_teleop; do
  bin="${BUILD_DIR}/tj_test/${name}"
  if [[ ! -f "${bin}" ]]; then
    echo "[grant_rt_caps] skip: ${bin} not found" >&2
    continue
  fi
  sudo setcap cap_sys_nice,cap_ipc_lock+ep "${bin}"
  echo "[grant_rt_caps] ${bin}: $(getcap "${bin}")"
done
