#!/usr/bin/env bash
# 停止 MiMo Agent 一键环境
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/.." && pwd)"
PID_FILE="$ROOT/.cursor/mimo-agent/pids.env"
PORT="${MIMO_PROXY_PORT:-8765}"

if [[ -f "$PID_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$PID_FILE"
  [[ -n "${TUNNEL_PID:-}" ]] && kill "$TUNNEL_PID" 2>/dev/null || true
  [[ -n "${PROXY_PID:-}" ]] && kill "$PROXY_PID" 2>/dev/null || true
  rm -f "$PID_FILE"
fi

fuser -k "${PORT}/tcp" 2>/dev/null || true
pkill -f "mimo_cursor_proxy.py" 2>/dev/null || true
echo "[mimo-agent] 已停止代理与隧道"
