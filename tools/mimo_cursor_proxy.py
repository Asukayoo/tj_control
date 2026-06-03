#!/usr/bin/env python3
"""MiMo Cursor 代理：Responses API 转 Chat Completions，并修复 tools schema。"""

import json
import os
import sys
import time
import uuid
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

TARGET = os.environ.get("MIMO_TARGET", "https://token-plan-cn.xiaomimimo.com")
REAL_MODEL = os.environ.get("MIMO_MODEL", "mimo-v2.5-pro")
HOST = os.environ.get("MIMO_PROXY_HOST", "127.0.0.1")
PORT = int(os.environ.get("MIMO_PROXY_PORT", "8765"))
CURSOR_ALIASES = os.environ.get(
    "MIMO_CURSOR_ALIASES",
    "mimo-v2.5-pro,glm-4.7,kimi-k2.5-custom,deepseek-chat",
).split(",")


class Seq:
    """SSE 序号计数。"""

    def __init__(self):
        self.n = 0

    def next(self):
        v = self.n
        self.n += 1
        return v


def fix_tools(tools):
    """扁平 tools -> MiMo 嵌套 function 格式。"""
    if not tools:
        return tools
    fixed = []
    for tool in tools:
        if not isinstance(tool, dict):
            fixed.append(tool)
            continue
        if tool.get("type") == "function" and "function" not in tool:
            fn = {k: tool[k] for k in ("name", "description", "parameters", "strict") if k in tool}
            fixed.append({"type": "function", "function": fn})
        else:
            fixed.append(tool)
    return fixed


def _text_from_content(content):
    """从 Responses/Chat 多种 content 结构提取文本。"""
    if content is None:
        return ""
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        parts = []
        for p in content:
            if isinstance(p, str):
                parts.append(p)
            elif isinstance(p, dict):
                parts.append(p.get("text") or p.get("output_text") or "")
        return "".join(parts)
    return str(content)


def responses_input_to_messages(body):
    """Responses API input -> Chat Completions messages。"""
    messages = []
    instructions = body.get("instructions")
    if instructions:
        messages.append({"role": "system", "content": instructions})

    inp = body.get("input", "")
    if isinstance(inp, str) and inp:
        messages.append({"role": "user", "content": inp})
    elif isinstance(inp, list):
        for item in inp:
            if isinstance(item, str):
                messages.append({"role": "user", "content": item})
                continue
            if not isinstance(item, dict):
                continue
            itype = item.get("type", "")
            if itype == "function_call_output":
                messages.append({
                    "role": "tool",
                    "tool_call_id": item.get("call_id") or item.get("id") or "",
                    "content": item.get("output") or _text_from_content(item.get("content")),
                })
                continue
            if itype == "function_call":
                messages.append({
                    "role": "assistant",
                    "content": None,
                    "tool_calls": [{
                        "id": item.get("call_id") or item.get("id") or f"call_{uuid.uuid4().hex[:8]}",
                        "type": "function",
                        "function": {
                            "name": item.get("name", ""),
                            "arguments": item.get("arguments") or "{}",
                        },
                    }],
                })
                continue
            role = item.get("role") or "user"
            if role == "developer":
                role = "system"
            text = _text_from_content(item.get("content")) or item.get("text") or ""
            if text or role == "assistant":
                messages.append({"role": role, "content": text})

    if not messages:
        messages.append({"role": "user", "content": "hello"})
    return messages


def responses_to_chat(body):
    """Responses 请求体 -> Chat Completions 请求体。"""
    chat = {
        "model": REAL_MODEL,
        "messages": responses_input_to_messages(body),
        "thinking": {"type": "disabled"},
    }
    if body.get("tools"):
        chat["tools"] = fix_tools(body.get("tools"))
    if body.get("max_output_tokens"):
        chat["max_tokens"] = body["max_output_tokens"]
    elif body.get("max_tokens"):
        chat["max_tokens"] = body["max_tokens"]
    if body.get("stream"):
        chat["stream"] = True
    if body.get("temperature") is not None:
        chat["temperature"] = body["temperature"]
    return chat


def chat_to_responses(chat_resp):
    """Chat Completions 响应 -> Responses API 响应。"""
    if "error" in chat_resp:
        return chat_resp
    choice = chat_resp.get("choices", [{}])[0]
    msg = choice.get("message", {})
    content = msg.get("content") or msg.get("reasoning_content") or ""
    output = []

    for tc in msg.get("tool_calls") or []:
        fn = tc.get("function") or {}
        output.append({
            "type": "function_call",
            "id": tc.get("id") or f"fc_{uuid.uuid4().hex[:8]}",
            "call_id": tc.get("id") or f"fc_{uuid.uuid4().hex[:8]}",
            "name": fn.get("name", ""),
            "arguments": fn.get("arguments") or "{}",
        })

    if content:
        output.append({
            "type": "message",
            "id": f"msg_{uuid.uuid4().hex[:8]}",
            "role": "assistant",
            "status": "completed",
            "content": [{"type": "output_text", "text": content}],
        })

    finish = choice.get("finish_reason") or "stop"
    status = "completed" if finish in ("stop", "tool_calls", "length") else "incomplete"
    return {
        "id": f"resp_{chat_resp.get('id', uuid.uuid4().hex)}",
        "object": "response",
        "created_at": chat_resp.get("created") or int(time.time()),
        "model": chat_resp.get("model") or REAL_MODEL,
        "status": status,
        "output": output,
        "output_text": content,
        "usage": chat_resp.get("usage"),
    }


def patch_chat_request(body):
    """改写 Chat Completions 请求。"""
    body["model"] = REAL_MODEL
    if "tools" in body:
        body["tools"] = fix_tools(body.get("tools"))
    if not isinstance(body.get("thinking"), dict):
        body["thinking"] = {"type": "disabled"}
    return body


def build_auth_headers(headers):
    """提取鉴权头。"""
    req_headers = {"Content-Type": "application/json"}
    auth = headers.get("Authorization") or headers.get("authorization")
    if auth:
        req_headers["Authorization"] = auth
    api_key = headers.get("api-key") or headers.get("Api-Key")
    if api_key:
        req_headers["api-key"] = api_key
    return req_headers


def call_mimo(path, headers, body_bytes):
    """转发到 MiMo API（非流式）。"""
    url = f"{TARGET}{path}"
    req_headers = build_auth_headers(headers)
    req = urllib.request.Request(url, data=body_bytes, headers=req_headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=300) as resp:
            return resp.status, resp.read(), resp.headers.get("Content-Type", "application/json")
    except urllib.error.HTTPError as err:
        return err.code, err.read(), err.headers.get("Content-Type", "application/json")


def open_mimo_stream(path, headers, body_bytes):
    """打开 MiMo 流式连接。"""
    url = f"{TARGET}{path}"
    req_headers = build_auth_headers(headers)
    req = urllib.request.Request(url, data=body_bytes, headers=req_headers, method="POST")
    return urllib.request.urlopen(req, timeout=300)


def iter_chat_sse(resp):
    """解析 Chat Completions SSE。"""
    while True:
        line = resp.readline()
        if not line:
            break
        line = line.strip()
        if not line or not line.startswith(b"data:"):
            continue
        payload = line[5:].strip()
        if payload == b"[DONE]":
            break
        try:
            yield json.loads(payload)
        except json.JSONDecodeError:
            continue


class ProxyHandler(BaseHTTPRequestHandler):
    """OpenAI 兼容代理。"""

    def log_message(self, fmt, *args):
        sys.stderr.write(f"[mimo-proxy] {self.address_string()} {fmt % args}\n")

    def _read_body(self):
        length = int(self.headers.get("Content-Length", 0))
        return self.rfile.read(length) if length else b""

    def _send(self, code, data, content_type="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(data)

    def _write_sse(self, event_type, data):
        """写入 SSE 事件。"""
        if isinstance(data, dict) and "type" not in data:
            data = {**data, "type": event_type}
        chunk = f"event: {event_type}\ndata: {json.dumps(data, ensure_ascii=False)}\n\n"
        self.wfile.write(chunk.encode())
        self.wfile.flush()

    def _stream_responses(self, body, hdrs):
        """Responses 流式：Chat SSE -> Responses SSE。"""
        chat_body = responses_to_chat(body)
        chat_body["stream"] = True
        requested_model = body.get("model") or REAL_MODEL
        resp_id = f"resp_{uuid.uuid4().hex}"
        seq = Seq()
        output_index = 0
        msg_item_id = None
        msg_started = False
        text_buf = ""
        tool_items = {}  # index -> {item_id, call_id, name, args}

        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        base = {
            "id": resp_id,
            "object": "response",
            "status": "in_progress",
            "model": requested_model,
            "output": [],
        }
        self._write_sse("response.created", {
            "response": base,
            "sequence_number": seq.next(),
        })

        try:
            upstream = open_mimo_stream(
                "/v1/chat/completions",
                hdrs,
                json.dumps(chat_body).encode(),
            )
        except urllib.error.HTTPError as err:
            err_body = err.read().decode("utf-8", errors="replace")
            self._write_sse("error", {
                "code": str(err.code),
                "message": err_body,
                "sequence_number": seq.next(),
            })
            return

        usage = None
        finish_reason = None
        try:
            for chunk in iter_chat_sse(upstream):
                if chunk.get("usage"):
                    usage = chunk["usage"]
                choice = (chunk.get("choices") or [{}])[0]
                finish_reason = choice.get("finish_reason") or finish_reason
                delta = choice.get("delta") or {}

                content = delta.get("content") or delta.get("reasoning_content") or ""
                if content:
                    if not msg_started:
                        msg_started = True
                        msg_item_id = f"msg_{uuid.uuid4().hex[:8]}"
                        self._write_sse("response.output_item.added", {
                            "output_index": output_index,
                            "item": {
                                "id": msg_item_id,
                                "type": "message",
                                "role": "assistant",
                                "status": "in_progress",
                            },
                            "sequence_number": seq.next(),
                        })
                        self._write_sse("response.content_part.added", {
                            "item_id": msg_item_id,
                            "output_index": output_index,
                            "content_index": 0,
                            "part": {"type": "output_text", "text": ""},
                            "sequence_number": seq.next(),
                        })
                    text_buf += content
                    self._write_sse("response.output_text.delta", {
                        "item_id": msg_item_id,
                        "output_index": output_index,
                        "content_index": 0,
                        "delta": content,
                        "sequence_number": seq.next(),
                    })

                for tc in delta.get("tool_calls") or []:
                    idx = tc.get("index", 0)
                    if idx not in tool_items:
                        item_id = f"fc_{uuid.uuid4().hex[:8]}"
                        call_id = tc.get("id") or item_id
                        fn = tc.get("function") or {}
                        tool_items[idx] = {
                            "item_id": item_id,
                            "call_id": call_id,
                            "name": fn.get("name") or "",
                            "args": "",
                            "started": False,
                            "output_index": output_index + (1 if msg_started else 0) + idx,
                        }
                    item = tool_items[idx]
                    fn = tc.get("function") or {}
                    if fn.get("name"):
                        item["name"] = fn["name"]
                    if fn.get("arguments"):
                        if not item["started"]:
                            item["started"] = True
                            self._write_sse("response.output_item.added", {
                                "output_index": item["output_index"],
                                "item": {
                                    "id": item["item_id"],
                                    "type": "function_call",
                                    "status": "in_progress",
                                    "call_id": item["call_id"],
                                    "name": item["name"],
                                },
                                "sequence_number": seq.next(),
                            })
                        item["args"] += fn["arguments"]
                        self._write_sse("response.function_call_arguments.delta", {
                            "item_id": item["item_id"],
                            "output_index": item["output_index"],
                            "delta": fn["arguments"],
                            "sequence_number": seq.next(),
                        })
        finally:
            upstream.close()

        final_output = []
        if msg_started and msg_item_id:
            self._write_sse("response.output_text.done", {
                "item_id": msg_item_id,
                "output_index": 0,
                "content_index": 0,
                "text": text_buf,
                "sequence_number": seq.next(),
            })
            self._write_sse("response.content_part.done", {
                "item_id": msg_item_id,
                "output_index": 0,
                "content_index": 0,
                "part": {"type": "output_text", "text": text_buf},
                "sequence_number": seq.next(),
            })
            msg_item = {
                "id": msg_item_id,
                "type": "message",
                "role": "assistant",
                "status": "completed",
                "content": [{"type": "output_text", "text": text_buf}],
            }
            self._write_sse("response.output_item.done", {
                "output_index": 0,
                "item": msg_item,
                "sequence_number": seq.next(),
            })
            final_output.append(msg_item)

        for idx in sorted(tool_items.keys()):
            item = tool_items[idx]
            if not item["started"]:
                continue
            self._write_sse("response.function_call_arguments.done", {
                "item_id": item["item_id"],
                "output_index": item["output_index"],
                "name": item["name"],
                "arguments": item["args"] or "{}",
                "sequence_number": seq.next(),
            })
            fc_item = {
                "id": item["item_id"],
                "type": "function_call",
                "status": "completed",
                "call_id": item["call_id"],
                "name": item["name"],
                "arguments": item["args"] or "{}",
            }
            self._write_sse("response.output_item.done", {
                "output_index": item["output_index"],
                "item": fc_item,
                "sequence_number": seq.next(),
            })
            final_output.append(fc_item)

        status = "completed"
        if finish_reason and finish_reason not in ("stop", "tool_calls", "length"):
            status = "incomplete"

        completed = {
            "id": resp_id,
            "object": "response",
            "status": status,
            "model": requested_model,
            "output": final_output,
            "output_text": text_buf,
            "usage": usage,
        }
        self._write_sse("response.completed", {
            "response": completed,
            "sequence_number": seq.next(),
        })
        self.close_connection = True

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Authorization, Content-Type, api-key")
        self.end_headers()

    def do_GET(self):
        if self.path.rstrip("/") == "/v1/models":
            payload = {
                "object": "list",
                "data": [
                    {"id": n.strip(), "object": "model", "owned_by": "mimo-proxy"}
                    for n in CURSOR_ALIASES if n.strip()
                ],
            }
            self._send(200, json.dumps(payload).encode())
            return
        code, data, ct = call_mimo(self.path, dict(self.headers), None)
        self._send(code, data, ct)

    def do_POST(self):
        raw = self._read_body()
        try:
            body = json.loads(raw) if raw else {}
        except json.JSONDecodeError:
            self._send(400, json.dumps({"error": "invalid json"}).encode())
            return

        path = self.path.split("?", 1)[0]
        hdrs = dict(self.headers)

        # Cursor Agent 使用 /v1/responses，MiMo 仅支持 /v1/chat/completions
        if path.rstrip("/") == "/v1/responses":
            model = body.get("model", "?")
            if body.get("stream"):
                self.log_message("POST /v1/responses stream model=%s tools=%d", model, len(body.get("tools") or []))
                self._stream_responses(body, hdrs)
                return
            chat_body = responses_to_chat(body)
            self.log_message("POST /v1/responses model=%s tools=%d", model, len(chat_body.get("tools") or []))
            code, data, _ = call_mimo("/v1/chat/completions", hdrs, json.dumps(chat_body).encode())
            try:
                chat_resp = json.loads(data)
            except json.JSONDecodeError:
                self._send(code, data)
                return
            if code >= 400:
                self._send(code, data)
                return
            self._send(200, json.dumps(chat_to_responses(chat_resp)).encode())
            return

        body = patch_chat_request(body)
        self.log_message("POST %s model=%s tools=%d", path, REAL_MODEL, len(body.get("tools") or []))
        code, data, ct = call_mimo(path, hdrs, json.dumps(body).encode())
        self._send(code, data, ct)


def main():
    class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
        daemon_threads = True

    server = ThreadingHTTPServer((HOST, PORT), ProxyHandler)
    print(f"MiMo Cursor 代理: http://{HOST}:{PORT}/v1")
    print(f"  真实模型: {REAL_MODEL}")
    print(f"  支持: /v1/chat/completions, /v1/responses(Agent)")
    print(f"  Cursor Base URL 请填此地址（Agent 需公网可达）")
    server.serve_forever()


if __name__ == "__main__":
    main()
