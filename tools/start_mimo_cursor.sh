#!/usr/bin/env bash
# 启动 MiMo 代理，供 Cursor Agent BYOK 使用
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
PORT="${MIMO_PROXY_PORT:-8765}"
# Cursor 白名单内、且不走 OpenAI 官方路由的别名（按优先级尝试）
export MIMO_CURSOR_ALIAS="${MIMO_CURSOR_ALIAS:-glm-4.7}"
export MIMO_CURSOR_ALIASES="${MIMO_CURSOR_ALIASES:-glm-4.7,kimi-k2.5-custom,deepseek-chat}"

fuser -k "${PORT}/tcp" 2>/dev/null || true
sleep 0.5

echo ">>> 启动 MiMo 代理 (端口 ${PORT})..."
python3 "$DIR/mimo_cursor_proxy.py" &
PROXY_PID=$!
sleep 1

if ! curl -sf "http://127.0.0.1:${PORT}/v1/models" >/dev/null; then
  echo "代理启动失败"
  kill "$PROXY_PID" 2>/dev/null || true
  exit 1
fi

echo ""
echo "=========================================="
echo "  代理已就绪 (实际模型: mimo-v2.5-pro)"
echo "=========================================="
echo ""
echo "【重要】Cursor Agent 请求经 Cursor 云端转发，"
echo "  localhost 不可用！Base URL 必须是公网地址。"
echo ""
echo "方案 A - 直连 MiMo（跳过本地代理，仅 Chat 模式可能可用）:"
echo "  Base URL: https://token-plan-cn.xiaomimimo.com/v1"
echo "  模型名:   mimo-v2.5-pro"
echo ""
echo "方案 B - Agent 模式（需公网代理 + 白名单别名）:"
echo "  1. 用 ngrok/cloudflared 暴露端口 ${PORT}:"
echo "     cloudflared tunnel --url http://127.0.0.1:${PORT}"
echo "     或: ngrok http ${PORT}"
echo "  2. Cursor Settings → Models:"
echo "     - OpenAI API Key: 你的 tp-... Key (开启)"
echo "     - Override Base URL: https://<公网地址>/v1"
echo "     - 添加模型名: ${MIMO_CURSOR_ALIAS}"
echo "     - 备选: kimi-k2.5-custom / deepseek-chat"
echo "  3. Agent 模式选择上述模型名"
echo ""
echo "验证: bash $DIR/verify_mimo_proxy.sh"
echo "代理 PID: $PROXY_PID (Ctrl+C 停止)"
wait $PROXY_PID
