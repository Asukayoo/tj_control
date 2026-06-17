#!/usr/bin/env python3
"""test_ServoPByPico ref_cart → Pico 模拟发布自测。"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
_TELEOP = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
  sys.path.insert(0, str(_ROOT))
if str(_TELEOP) not in sys.path:
  sys.path.insert(0, str(_TELEOP))

from one_euro_filter import PoseOneEuroFilter, OneEuroParams, pose_xyzw_is_valid  # noqa: E402
from pico_servo_cart_source import cart_row_to_pose7, iter_servo_cart_dir, load_ref_cart  # noqa: E402

DEFAULT_DIR = _ROOT / "data/test_ServoPByPico"


def validate_cart_dir(dir_path: Path, max_rows: int) -> tuple[int, int, int]:
  """返回 (检查行数, 原始有效, 滤波后有效)。"""
  right = load_ref_cart(dir_path / "right_ref_cart.csv")
  left = load_ref_cart(dir_path / "left_ref_cart.csv")
  n = min(len(right), len(left), max_rows)
  filt = PoseOneEuroFilter(OneEuroParams(1.15, 0.5, 1.2), OneEuroParams(1.5, 0.5, 1.2))
  dt = 0.02
  raw_ok = 0
  filt_ok = 0
  for i in range(n):
    rp = cart_row_to_pose7(right[i])
    lp = cart_row_to_pose7(left[i])
    if pose_xyzw_is_valid(rp):
      raw_ok += 1
    if pose_xyzw_is_valid(lp):
      raw_ok += 1
    rp_pos, rp_q = filt.filter_pose_xyzw(dt, rp[:3], rp[3:7])
    lp_pos, lp_q = filt.filter_pose_xyzw(dt, lp[:3], lp[3:7])
    if pose_xyzw_is_valid(np.concatenate([rp_pos, rp_q])):
      filt_ok += 1
    if pose_xyzw_is_valid(np.concatenate([lp_pos, lp_q])):
      filt_ok += 1
  return n, raw_ok, filt_ok


def run_udp_test(dir_path: Path, port: int, frames: int) -> int:
  check_py = _TELEOP / "pico_udp_check.py"
  pub_py = _TELEOP / "pico_udp_publisher.py"
  check = subprocess.Popen(
      [sys.executable, str(check_py), "--port", str(port), "--count", str(frames)],
      stdout=subprocess.PIPE,
      stderr=subprocess.STDOUT,
      text=True,
  )
  time.sleep(0.2)
  pub = subprocess.run(
      [
          sys.executable,
          str(pub_py),
          "--replay-servo-dir",
          str(dir_path),
          "--port",
          str(port),
          "--max-frames",
          str(frames),
          "--decimate",
          "20",
      ],
      capture_output=True,
      text=True,
  )
  out_check, _ = check.communicate(timeout=30)
  print(pub.stdout)
  if pub.returncode != 0:
    print(pub.stderr, file=sys.stderr)
  print(out_check)
  if pub.returncode != 0:
    return 1
  if "PASS valid=" not in out_check:
    return 1
  return 0


def main() -> None:
  parser = argparse.ArgumentParser(description="ServoPByPico cart 模拟 Pico 自测")
  parser.add_argument(
      "--dir",
      type=Path,
      default=DEFAULT_DIR,
      help="含 left/right_ref_cart.csv 的目录",
  )
  parser.add_argument("--validate-rows", type=int, default=500)
  parser.add_argument("--udp-frames", type=int, default=50)
  parser.add_argument("--port", type=int, default=30102)
  parser.add_argument("--skip-udp", action="store_true")
  args = parser.parse_args()

  if not args.dir.is_dir():
    print(f"[FAIL] 目录不存在: {args.dir}", file=sys.stderr)
    raise SystemExit(1)

  print(f"[selftest] 目录={args.dir}")
  n, raw_ok, filt_ok = validate_cart_dir(args.dir, args.validate_rows)
  print(f"[selftest] 校验 {n} 行×2臂: raw_valid={raw_ok} filt_valid={filt_ok}")
  if raw_ok < n * 2:
    print(f"[FAIL] 原始位姿含非法四元数", file=sys.stderr)
    raise SystemExit(1)
  if filt_ok < n * 2:
    print(f"[FAIL] One Euro 滤波后含非法四元数", file=sys.stderr)
    raise SystemExit(1)
  print("[selftest] PASS 数据+滤波")

  # 抽样迭代器
  sample = next(iter_servo_cart_dir(args.dir, decimate=20, loop=False))
  rp = sample["right_controller"]
  print(
      f"[selftest] 样本 cycle={sample['cycle']} "
      f"R_pos=[{rp[0]:.4f},{rp[1]:.4f},{rp[2]:.4f}] "
      f"R_qw={rp[6]:.4f}"
  )

  if args.skip_udp:
    raise SystemExit(0)

  print(f"[selftest] UDP 端到端 port={args.port} frames={args.udp_frames}")
  code = run_udp_test(args.dir, args.port, args.udp_frames)
  if code != 0:
    print("[FAIL] UDP 自测失败", file=sys.stderr)
    raise SystemExit(1)
  print("[selftest] PASS UDP 发布")


if __name__ == "__main__":
  main()
