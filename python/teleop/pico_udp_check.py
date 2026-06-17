"""监听 Pico UDP 包并校验（发布方自测用）。"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys

PICO_MAGIC = 0x5049434F
PICO_UDP_FMT = "<IIQ14dffB"
PICO_UDP_SIZE = struct.calcsize(PICO_UDP_FMT)


def check_packet(data: bytes) -> dict:
  if len(data) < PICO_UDP_SIZE:
    return {"ok": False, "reason": f"short {len(data)}"}
  vals = struct.unpack(PICO_UDP_FMT, data[:PICO_UDP_SIZE])
  magic, seq, ts = vals[0], vals[1], vals[2]
  if magic != PICO_MAGIC:
    return {"ok": False, "reason": f"bad_magic 0x{magic:08X}"}
  rp = vals[3:10]
  qw = rp[6]
  qn = math.sqrt(sum(q * q for q in rp[3:7]))
  finite = all(math.isfinite(v) for v in rp)
  return {
      "ok": finite and qn > 1e-6,
      "seq": seq,
      "ts": ts,
      "right_x": rp[0],
      "right_qw": qw,
      "quat_norm": qn,
      "flags": vals[-1],
  }


def main() -> None:
  parser = argparse.ArgumentParser(description="Pico UDP 监听校验")
  parser.add_argument("--port", type=int, default=30101)
  parser.add_argument("--count", type=int, default=20)
  args = parser.parse_args()

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.bind(("127.0.0.1", args.port))
  sock.settimeout(5.0)
  print(f"[check] listen :{args.port}, expect {args.count} pkts", flush=True)

  ok_n = 0
  for i in range(args.count):
    try:
      data, _ = sock.recvfrom(2048)
    except socket.timeout:
      print(f"[check] timeout after {i} pkts", file=sys.stderr)
      sys.exit(1)
    info = check_packet(data)
    if info["ok"]:
      ok_n += 1
    if i == 0 or (i + 1) % 10 == 0:
      print(
          f"[check] #{i+1} seq={info.get('seq')} ok={info.get('ok')} "
          f"x={info.get('right_x', 0):.3f} qw={info.get('right_qw', 0):.4f} "
          f"|q|={info.get('quat_norm', 0):.4f}"
      )
  print(f"[check] PASS valid={ok_n}/{args.count}")
  sock.close()


if __name__ == "__main__":
  main()
