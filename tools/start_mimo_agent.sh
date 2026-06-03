#!/usr/bin/env bash
# MiMo Agent 一键启动：代理 + 公网隧道 + 写入 Cursor 配置
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/.." && pwd)"
CONFIG="$ROOT/.cursor/mimo.config.json"
STATE_DIR="$ROOT/.cursor/mimo-agent"
PID_FILE="$STATE_DIR/pids.env"
LOG_DIR="$STATE_DIR/logs"
PORT="${MIMO_PROXY_PORT:-8765}"
PROXY_HOST="${MIMO_PROXY_HOST:-127.0.0.1}"

mkdir -p "$STATE_DIR" "$LOG_DIR"

log() { echo "[mimo-agent] $*"; }

read_config() {
  python3 - "$CONFIG" <<'PY'
import json, sys
cfg = json.load(open(sys.argv[1], encoding="utf-8"))
print(cfg.get("cursor_agent_model") or "mimo-v2.5-pro")
print(cfg.get("model", "mimo-v2.5-pro"))
print(cfg.get("api_key", ""))
print(cfg.get("proxy", {}).get("public_base_url", ""))
PY
}

stop_old() {
  if [[ -f "$PID_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$PID_FILE" 2>/dev/null || true
    [[ -n "${PROXY_PID:-}" ]] && kill "$PROXY_PID" 2>/dev/null || true
    [[ -n "${TUNNEL_PID:-}" ]] && kill "$TUNNEL_PID" 2>/dev/null || true
  fi
  fuser -k "${PORT}/tcp" 2>/dev/null || true
  sleep 0.5
}

ensure_cloudflared() {
  if command -v cloudflared >/dev/null 2>&1; then
    command -v cloudflared
    return
  fi
  local bin="$DIR/cloudflared"
  if [[ -x "$bin" ]] && "$bin" --version >/dev/null 2>&1; then
    echo "$bin"
    return
  fi
  log "未找到 cloudflared，运行安装脚本..."
  bash "$DIR/install_cloudflared.sh"
  echo "$bin"
}

start_proxy() {
  log "启动本地代理 :${PORT} ..."
  python3 "$DIR/mimo_cursor_proxy.py" >"$LOG_DIR/proxy.log" 2>&1 &
  PROXY_PID=$!
  for _ in $(seq 1 20); do
    if curl -sf "http://${PROXY_HOST}:${PORT}/v1/models" >/dev/null; then
      log "代理就绪 (PID=$PROXY_PID)"
      return
    fi
    sleep 0.5
  done
  log "代理启动失败，见 $LOG_DIR/proxy.log"
  kill "$PROXY_PID" 2>/dev/null || true
  exit 1
}

start_tunnel() {
  local existing="$1"
  # 环境变量优先（便于复用已有 ngrok / trycloudflare URL）
  if [[ -n "${MIMO_PUBLIC_BASE_URL:-}" ]]; then
    PUBLIC_BASE="${MIMO_PUBLIC_BASE_URL%/}/v1"
    TUNNEL_PID=""
    log "使用环境变量公网 URL: $PUBLIC_BASE"
    return
  fi
  if [[ -n "$existing" ]]; then
    if curl -sf --max-time 8 "${existing%/v1}/v1/models" >/dev/null 2>&1; then
      PUBLIC_BASE="$existing"
      TUNNEL_PID=""
      log "复用可用公网 URL: $PUBLIC_BASE"
      return
    fi
    log "已有公网 URL 不可用，重新建立隧道..."
  fi

  local cf log="$LOG_DIR/tunnel.log"
  cf="$(ensure_cloudflared)"
  log "启动公网隧道 (cloudflared)..."
  "$cf" tunnel --url "http://${PROXY_HOST}:${PORT}" >"$log" 2>&1 &
  TUNNEL_PID=$!

  PUBLIC_BASE=""
  for _ in $(seq 1 60); do
    PUBLIC_BASE="$(grep -oE 'https://[a-zA-Z0-9-]+\.trycloudflare\.com' "$log" | head -1 || true)"
    [[ -n "$PUBLIC_BASE" ]] && break
    sleep 1
  done

  if [[ -z "$PUBLIC_BASE" ]]; then
    log "隧道 URL 获取失败，见 $log"
    log "可手动运行: cloudflared tunnel --url http://${PROXY_HOST}:${PORT}"
    exit 1
  fi
  PUBLIC_BASE="${PUBLIC_BASE%/}/v1"
  log "公网 URL: $PUBLIC_BASE"
}

apply_cursor() {
  local base="$1"
  if pgrep -x cursor >/dev/null 2>&1 || pgrep -f "/usr/share/cursor" >/dev/null 2>&1; then
    log "错误: Cursor 正在运行，配置写入后会被覆盖。"
    log "请完全退出 Cursor，再执行:"
    log "  MIMO_PUBLIC_BASE_URL='$base' python3 $DIR/apply_mimo_cursor_config.py --base-url '$base'"
    return 1
  fi
  MIMO_PUBLIC_BASE_URL="$base" python3 "$DIR/apply_mimo_cursor_config.py" --base-url "$base"
}

verify_all() {
  local base="$1" key="$2" model="$3"
  log "验证代理与 Responses API ..."
  curl -sf "${base%/v1}/v1/models" >/dev/null
  curl -sf "${base%/v1}/v1/responses" \
    -H "Authorization: Bearer $key" \
    -H "Content-Type: application/json" \
    -d "{\"model\":\"${model}\",\"input\":\"只回复OK\",\"max_output_tokens\":10}" \
    | python3 -c "import sys,json; d=json.load(sys.stdin); assert d.get('status')=='completed', d; print('Responses API OK:', d.get('output_text','')[:20])"
}

save_pids() {
  cat >"$PID_FILE" <<EOF
PROXY_PID=${PROXY_PID:-}
TUNNEL_PID=${TUNNEL_PID:-}
PUBLIC_BASE=${PUBLIC_BASE:-}
PORT=${PORT}
EOF
}

print_summary() {
  local agent_model="$1" backend_model="$2"
  cat <<EOF

========================================
  MiMo Agent 环境已启动
========================================
  Agent 选:   ${agent_model}  （不是 ${backend_model}）
  后端模型:   ${backend_model}
  公网 URL:   ${PUBLIC_BASE}
  代理日志:   ${LOG_DIR}/proxy.log
  隧道日志:   ${LOG_DIR}/tunnel.log
  停止命令:   bash ${DIR}/stop_mimo_agent.sh

  Cursor 操作:
  1. 完全退出并重启 Cursor
  2. Settings → Models 确认 OpenAI Key 已开启
  3. Agent 模式选择: ${agent_model}

  关于 Auto / 内置模型:
  - Auto、Claude、Composer 等仍可使用
  - 仅选 ${agent_model} 时走 MiMo 代理

EOF
}

main() {
  [[ -f "$CONFIG" ]] || { log "缺少配置: $CONFIG"; exit 1; }

  mapfile -t CFG_LINES < <(read_config)
  AGENT_MODEL="${CFG_LINES[0]:-mimo-v2.5-pro}"
  BACKEND_MODEL="${CFG_LINES[1]:-mimo-v2.5-pro}"
  API_KEY="${CFG_LINES[2]:-}"
  EXISTING_PUBLIC="${CFG_LINES[3]:-}"

  stop_old
  start_proxy
  start_tunnel "$EXISTING_PUBLIC"
  if ! apply_cursor "$PUBLIC_BASE"; then
    save_pids
    log "代理与隧道已启动，但 Cursor 配置未写入（请先关闭 Cursor 再 apply）"
    print_summary "$AGENT_MODEL" "$BACKEND_MODEL"
    exit 1
  fi
  verify_all "$PUBLIC_BASE" "$API_KEY" "$AGENT_MODEL"
  save_pids
  print_summary "$AGENT_MODEL" "$BACKEND_MODEL"

  log "服务运行中，按 Ctrl+C 停止..."
  wait "${PROXY_PID}"
}

main "$@"
