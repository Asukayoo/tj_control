#!/usr/bin/env bash
# MiMo Agent 统一入口：start | stop | status | verify
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"

usage() {
  cat <<EOF
用法: bash $0 <命令>

  start    一键启动（代理 + 公网隧道 + 写 Cursor 配置）
  stop     停止代理与隧道
  status   查看运行状态
  verify   验证 API（/v1/models、/v1/responses）
  install  安装 cloudflared（Ubuntu 默认 apt 无此包）
  check    检查 Cursor BYOK 配置是否正确

环境变量:
  MIMO_PUBLIC_BASE_URL  已有公网 URL 时跳过隧道，例: https://xxx.trycloudflare.com/v1
  MIMO_PROXY_PORT       本地代理端口，默认 8765

示例:
  bash $0 start
  MIMO_PUBLIC_BASE_URL='https://xxx.trycloudflare.com/v1' bash $0 start
  bash $0 stop
EOF
}

cmd="${1:-}"
case "$cmd" in
  start)  exec bash "$DIR/start_mimo_agent.sh" ;;
  stop)   exec bash "$DIR/stop_mimo_agent.sh" ;;
  verify) exec bash "$DIR/verify_mimo_proxy.sh" ;;
  install) exec bash "$DIR/install_cloudflared.sh" ;;
  check)   exec python3 "$DIR/apply_mimo_cursor_config.py" --check ;;
  status)
    ROOT="$(cd "$DIR/.." && pwd)"
    PID_FILE="$ROOT/.cursor/mimo-agent/pids.env"
    PORT="${MIMO_PROXY_PORT:-8765}"
    if [[ -f "$PID_FILE" ]]; then
      # shellcheck disable=SC1090
      source "$PID_FILE"
      echo "[mimo-agent] 代理 PID=${PROXY_PID:-?} 隧道 PID=${TUNNEL_PID:-无}"
      echo "[mimo-agent] 公网 URL=${PUBLIC_BASE:-未设置}"
    else
      echo "[mimo-agent] 未运行（无 $PID_FILE）"
    fi
    if curl -sf "http://127.0.0.1:${PORT}/v1/models" >/dev/null 2>&1; then
      echo "[mimo-agent] 本地代理 :${PORT} 正常"
    else
      echo "[mimo-agent] 本地代理 :${PORT} 未响应"
    fi
    ;;
  -h|--help|help|"")
    usage
    [[ -n "$cmd" ]] || exit 0
    ;;
  *)
    echo "未知命令: $cmd" >&2
    usage >&2
    exit 1
    ;;
esac
