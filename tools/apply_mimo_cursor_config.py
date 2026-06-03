#!/usr/bin/env python3
"""将 .cursor/mimo.config.json 写入 Cursor 内部存储，绕过 Settings UI。"""

import json
import os
import shutil
import sqlite3
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / ".cursor" / "mimo.config.json"
CURSOR_DB = Path.home() / ".config/Cursor/User/globalStorage/state.vscdb"
APP_KEY = "src.vs.platform.reactivestorage.browser.reactiveStorageServiceImpl.persistentStorage.applicationUser"
OPENAI_KEY = "cursorAuth/openAIKey"

MIMO_MODEL_ENTRY = {
    "name": "mimo-v2.5-pro",
    "defaultOn": False,
    "parameterDefinitions": [],
    "variants": [],
    "legacySlugs": [],
    "idAliases": [],
    "supportsAgent": True,
    "degradationStatus": 0,
    "supportsThinking": True,
    "supportsImages": True,
    "supportsMaxMode": True,
    "clientDisplayName": "mimo-v2.5-pro",
    "serverModelName": "mimo-v2.5-pro",
    "supportsNonMaxMode": True,
    "isRecommendedForBackgroundComposer": False,
    "supportsPlanMode": True,
    "isUserAdded": True,
    "inputboxShortModelName": "mimo-v2.5-pro",
    "supportsSandboxing": True,
    "namedModelSectionIndex": 1,
}

# Agent 白名单别名（Cursor 云端校验通过，代理映射到 mimo-v2.5-pro）
AGENT_MODEL_ALIASES = ("glm-4.7", "kimi-k2.5-custom", "deepseek-chat")


def make_model_entry(name: str) -> dict:
    """生成模型条目。"""
    return {
        **MIMO_MODEL_ENTRY,
        "name": name,
        "clientDisplayName": name,
        "serverModelName": name,
        "inputboxShortModelName": name,
    }


def cursor_running() -> bool:
    """检测 Cursor 是否在运行。"""
    import subprocess

    for pat in ("cursor", "Cursor"):
        try:
            r = subprocess.run(["pgrep", "-x", pat], capture_output=True, check=False)
            if r.returncode == 0:
                return True
        except OSError:
            pass
    try:
        r = subprocess.run(["pgrep", "-f", "/usr/share/cursor"], capture_output=True, check=False)
        return r.returncode == 0
    except OSError:
        return False


def load_config():
    """加载 MiMo 配置。"""
    if not CONFIG_PATH.exists():
        raise FileNotFoundError(f"缺少配置文件: {CONFIG_PATH}")
    with CONFIG_PATH.open(encoding="utf-8") as f:
        cfg = json.load(f)
    for key in ("model", "base_url", "api_key"):
        if not cfg.get(key):
            raise ValueError(f"配置缺少字段: {key}")
    return cfg


def backup_db(db_path: Path):
    """备份 Cursor 状态库。"""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = db_path.with_suffix(f".vscdb.bak.{stamp}")
    shutil.copy2(db_path, backup)
    return backup


def upsert_model(models: list, entry: dict) -> list:
    """确保自定义模型存在于 availableDefaultModels2。"""
    name = entry["name"]
    for i, m in enumerate(models):
        if m.get("name") == name:
            models[i] = {**m, **entry}
            return models
    models.append(entry)
    return models


def resolve_base_url(cfg: dict) -> str:
    """解析 Cursor 应使用的 Base URL。"""
    env_public = os.environ.get("MIMO_PUBLIC_BASE_URL", "").strip().rstrip("/")
    if env_public:
        return env_public
    proxy = cfg.get("proxy") or {}
    if proxy.get("enabled"):
        public = (proxy.get("public_base_url") or "").strip().rstrip("/")
        if public:
            return public
        host = proxy.get("host", "127.0.0.1")
        port = proxy.get("port", 8765)
        return f"http://{host}:{port}/v1"
    return cfg["base_url"].rstrip("/")


def save_public_url(public_url: str):
    """将公网隧道 URL 写回配置文件。"""
    with CONFIG_PATH.open(encoding="utf-8") as f:
        cfg = json.load(f)
    cfg.setdefault("proxy", {})["public_base_url"] = public_url.rstrip("/")
    with CONFIG_PATH.open("w", encoding="utf-8") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)
        f.write("\n")


def apply(cfg: dict):
    """写入 Cursor globalStorage。"""
    if not CURSOR_DB.exists():
        raise FileNotFoundError(f"未找到 Cursor 数据库: {CURSOR_DB}")

    backup = backup_db(CURSOR_DB)
    con = sqlite3.connect(CURSOR_DB)
    cur = con.cursor()

    cur.execute("SELECT value FROM ItemTable WHERE key=?", (APP_KEY,))
    row = cur.fetchone()
    if not row:
        raise RuntimeError("未找到 Cursor applicationUser 存储项")
    app_data = json.loads(row[0])

    base_url = resolve_base_url(cfg)
    agent_model = cfg.get("cursor_agent_model") or "glm-4.7"

    # 必须开启 BYOK，且 Base URL 指向代理（含 /v1/responses 转换）
    app_data["openAIBaseUrl"] = base_url
    app_data["useOpenAIKey"] = True

    models = app_data.get("availableDefaultModels2", [])
    for alias in AGENT_MODEL_ALIASES:
        models = upsert_model(models, make_model_entry(alias))
    models = upsert_model(models, make_model_entry(cfg["model"]))
    app_data["availableDefaultModels2"] = models

    cur.execute(
        "UPDATE ItemTable SET value=? WHERE key=?",
        (json.dumps(app_data, ensure_ascii=False), APP_KEY),
    )
    cur.execute(
        "INSERT OR REPLACE INTO ItemTable(key, value) VALUES(?, ?)",
        (OPENAI_KEY, cfg["api_key"]),
    )
    con.commit()
    con.close()
    return backup, base_url, agent_model


def read_cursor_state():
    """读取当前 Cursor BYOK 配置。"""
    if not CURSOR_DB.exists():
        return None
    con = sqlite3.connect(CURSOR_DB)
    cur = con.cursor()
    cur.execute("SELECT value FROM ItemTable WHERE key=?", (APP_KEY,))
    row = cur.fetchone()
    con.close()
    if not row:
        return None
    app = json.loads(row[0])
    return {
        "openAIBaseUrl": app.get("openAIBaseUrl"),
        "useOpenAIKey": app.get("useOpenAIKey"),
    }


def main():
    """入口。"""
    if "--help" in sys.argv:
        print("用法: python3 tools/apply_mimo_cursor_config.py [--base-url URL] [--force]")
        print("环境变量: MIMO_PUBLIC_BASE_URL 可覆盖公网 Base URL")
        print("请先完全关闭 Cursor，再执行本脚本（运行中会被覆盖）。")
        return 0

    if "--check" in sys.argv:
        st = read_cursor_state()
        cfg = load_config()
        expected = resolve_base_url(cfg)
        print("当前 Cursor 配置:")
        print(f"  openAIBaseUrl:  {st.get('openAIBaseUrl') if st else 'N/A'}")
        print(f"  useOpenAIKey:   {st.get('useOpenAIKey') if st else 'N/A'}")
        print(f"  期望 Base URL:  {expected}")
        ok = st and st.get("useOpenAIKey") is True and st.get("openAIBaseUrl") == expected
        print("  状态:", "OK" if ok else "需要修复（请关闭 Cursor 后运行 apply）")
        return 0 if ok else 1

    force = "--force" in sys.argv
    if cursor_running() and not force:
        print("错误: Cursor 正在运行，写入后会被覆盖。")
        print("请完全退出 Cursor（所有窗口），再执行本脚本。")
        print("或加 --force 强制写入（不推荐）。")
        return 1

    cfg = load_config()
    for i, arg in enumerate(sys.argv):
        if arg == "--base-url" and i + 1 < len(sys.argv):
            os.environ["MIMO_PUBLIC_BASE_URL"] = sys.argv[i + 1].rstrip("/")

    public = os.environ.get("MIMO_PUBLIC_BASE_URL", "").strip()
    if public:
        save_public_url(public)

    backup, base_url, agent_model = apply(cfg)
    proxy = cfg.get("proxy") or {}
    print("已写入 Cursor 内部配置:")
    print(f"  Agent 模型: {agent_model}  （必须选 isUserAdded 的 BYOK 模型，勿选 glm-4.7 等内置模型）")
    print(f"  后端模型:   {cfg['model']}")
    print(f"  Base URL:   {base_url}")
    print(f"  useOpenAIKey: True")
    print(f"  API Key:    {cfg['api_key'][:8]}...{cfg['api_key'][-4:]}")
    print(f"  备份:       {backup}")
    print()
    if proxy.get("enabled") and "127.0.0.1" in base_url:
        print("⚠️  Agent 模式需公网 URL！请运行: bash tools/mimo_agent.sh start")
        print()
    print("请重启 Cursor，Agent 模式选择:", agent_model)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
