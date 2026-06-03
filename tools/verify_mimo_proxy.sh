#!/usr/bin/env bash
# MiMo Agent 环境验证
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/.." && pwd)"
CONFIG="$ROOT/.cursor/mimo.config.json"
PID_FILE="$ROOT/.cursor/mimo-agent/pids.env"

read_cfg() {
  python3 - "$CONFIG" <<'PY'
import json, sys
p = sys.argv[1]
try:
  cfg = json.load(open(p, encoding="utf-8"))
except FileNotFoundError:
  cfg = {}
proxy = cfg.get("proxy", {})
public = (proxy.get("public_base_url") or "").strip().rstrip("/")
port = proxy.get("port", 8765)
print(cfg.get("model", "mimo-v2.5-pro"))
print(cfg.get("api_key", ""))
print(public or f"http://127.0.0.1:{port}")
PY
}

mapfile -t L < <(read_cfg)
MODEL="${L[0]}"
KEY="${L[1]}"
PROXY="${MIMO_PROXY:-${L[2]}}"

if [[ -f "$PID_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$PID_FILE"
  [[ -n "${PUBLIC_BASE:-}" ]] && PROXY="${PUBLIC_BASE}"
fi

BASE="${PROXY%/v1}"

echo "=== 1. 代理 /v1/models ==="
curl -sf "$BASE/v1/models" | python3 -m json.tool

echo "=== 2. Agent /v1/responses ==="
curl -sf "$BASE/v1/responses" \
  -H "Authorization: Bearer $KEY" \
  -H "Content-Type: application/json" \
  -d "{\"model\":\"${MODEL}\",\"input\":\"只回复OK\",\"max_output_tokens\":10}" \
  | python3 -c "import sys,json; d=json.load(sys.stdin); assert d.get('model')=='mimo-v2.5-pro', d; print('PASS responses model=', d['model'], 'text=', d.get('output_text','')[:30])"

echo "=== 3. tools 格式 ==="
curl -sf "$BASE/v1/responses" \
  -H "Authorization: Bearer $KEY" \
  -H "Content-Type: application/json" \
  -d "{\"model\":\"${MODEL}\",\"input\":\"hi\",\"tools\":[{\"type\":\"function\",\"name\":\"read_file\",\"description\":\"Read\",\"parameters\":{\"type\":\"object\",\"properties\":{\"path\":{\"type\":\"string\"}}}}],\"max_output_tokens\":30}" \
  | python3 -c "import sys,json; d=json.load(sys.stdin); assert 'error' not in d, d; print('PASS tools ok')"

echo "=== 4. stream responses ==="
curl -sf "$BASE/v1/responses" \
  -H "Authorization: Bearer $KEY" \
  -H "Content-Type: application/json" \
  -d '{"model":"glm-4.7","input":"只回复OK","stream":true,"max_output_tokens":10}' \
  | grep -q "response.output_text.delta" && echo "PASS stream responses" || { echo "FAIL stream responses"; exit 1; }

echo "全部通过。Cursor Agent 请选择: mimo-v2.5-pro（勿选 glm-4.7）"
