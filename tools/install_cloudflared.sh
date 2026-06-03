#!/usr/bin/env bash
# 安装 cloudflared（Ubuntu 默认 apt 源无此包）
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
BIN="$DIR/cloudflared"
DEB="$DIR/cloudflared.deb"
# Cloudflare 官方 deb（pkg.cloudflare.com，国内一般比 GitHub 快）
DEB_URL="https://pkg.cloudflare.com/cloudflared/pool/main/c/cloudflared/cloudflared_2024.12.2_amd64.deb"

log() { echo "[install-cloudflared] $*"; }

if command -v cloudflared >/dev/null 2>&1; then
  log "系统已安装: $(cloudflared --version)"
  exit 0
fi

if [[ -x "$BIN" ]] && "$BIN" --version >/dev/null 2>&1; then
  log "本地可用: $BIN ($("$BIN" --version))"
  exit 0
fi

log "从 Cloudflare 官方源下载 deb ..."
curl -fsSL --connect-timeout 20 --max-time 600 -L -o "$DEB" "$DEB_URL"

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
dpkg-deb -x "$DEB" "$TMP"
cp "$TMP/usr/bin/cloudflared" "$BIN"
chmod +x "$BIN"
log "安装完成: $BIN ($("$BIN" --version))"

log ""
log "可选：系统级安装（需 sudo）"
log "  sudo dpkg -i $DEB"
log "或添加官方 apt 源后 apt install cloudflared："
log "  curl -fsSL https://pkg.cloudflare.com/cloudflare-main.gpg | sudo tee /usr/share/keyrings/cloudflare-main.gpg >/dev/null"
log "  echo 'deb [signed-by=/usr/share/keyrings/cloudflare-main.gpg] https://pkg.cloudflare.com/cloudflared any main' | sudo tee /etc/apt/sources.list.d/cloudflared.list"
log "  sudo apt update && sudo apt install cloudflared"
