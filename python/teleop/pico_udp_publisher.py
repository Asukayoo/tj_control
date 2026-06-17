"""PICO 数据 50Hz 采样 + One Euro 滤波 + UDP 发布。"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterator

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
  sys.path.insert(0, str(_ROOT))

from pico_data_receiver import PicoDataReceiver  # noqa: E402

try:
  from python.teleop.one_euro_filter import (
      OneEuroParams,
      PoseOneEuroFilter,
      pose_xyzw_is_valid,
  )
  from python.teleop.pico_csv_source import iter_pico_csv
  from python.teleop.pico_servo_cart_source import iter_servo_cart_dir
except ImportError:
  from one_euro_filter import (  # type: ignore
      OneEuroParams,
      PoseOneEuroFilter,
      pose_xyzw_is_valid,
  )
  from pico_csv_source import iter_pico_csv  # type: ignore
  from pico_servo_cart_source import iter_servo_cart_dir  # type: ignore

PICO_MAGIC = 0x5049434F
PICO_UDP_FMT = "<IIQ14dffB"
PICO_UDP_SIZE = struct.calcsize(PICO_UDP_FMT)
DEFAULT_POS_PARAMS = OneEuroParams(1.15, 0.5, 1.2)
DEFAULT_ORI_PARAMS = OneEuroParams(1.5, 0.5, 1.2)
DEFAULT_REPLAY_CSV = _ROOT / "data/test_teleop_data/pico_record_20260615_220409.csv"


def pack_pico_packet(
    seq: int,
    timestamp_ns: int,
    right_pose: np.ndarray,
    left_pose: np.ndarray,
    right_trigger: float,
    left_trigger: float,
    pose_valid: bool,
) -> bytes:
  """位姿 [x,y,z,qx,qy,qz,qw]；flags bit2=位姿有效。"""
  rp = np.asarray(right_pose, dtype=np.float64)
  lp = np.asarray(left_pose, dtype=np.float64)
  flags = 0
  if right_trigger >= 0.99:
    flags |= 1
  if left_trigger >= 0.99:
    flags |= 2
  if pose_valid:
    flags |= 4
  return struct.pack(
      PICO_UDP_FMT,
      PICO_MAGIC,
      seq,
      timestamp_ns,
      *rp[:7],
      *lp[:7],
      float(right_trigger),
      float(left_trigger),
      flags,
  )


def _process_pose(
    raw_pose: np.ndarray,
    filt: PoseOneEuroFilter | None,
    dt: float,
    last_good: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray | None, bool]:
  raw = np.asarray(raw_pose, dtype=np.float64)
  if not pose_xyzw_is_valid(raw):
    if last_good is not None:
      return last_good.copy(), last_good, False
    return raw, None, False
  if filt is None:
    return raw, raw.copy(), True
  pos_f, quat_f = filt.filter_pose_xyzw(dt, raw[:3], raw[3:7])
  out = np.concatenate([pos_f, quat_f])
  if not pose_xyzw_is_valid(out):
    if last_good is not None:
      return last_good.copy(), last_good, False
    return out, None, False
  return out, out.copy(), True


def _publish_loop(
    frame_iter: Iterator[Dict[str, Any]],
    host: str,
    port: int,
    rate_hz: float,
    pos_params: OneEuroParams,
    ori_params: OneEuroParams,
    use_filter: bool,
    dry_run: bool,
    max_frames: int,
    max_ticks: int,
    source_name: str,
) -> int:
  period = 1.0 / max(1e-3, rate_hz)
  dt = period
  sock = None if dry_run else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  dest = (host, port)

  right_f = PoseOneEuroFilter(pos_params, ori_params) if use_filter else None
  left_f = PoseOneEuroFilter(pos_params, ori_params) if use_filter else None
  last_good_r: np.ndarray | None = None
  last_good_l: np.ndarray | None = None

  mode = "dry-run" if dry_run else f"udp→{host}:{port}"
  filt = "one_euro" if use_filter else "raw"
  print(f"[pico_udp] source={source_name}  {mode}  {rate_hz:.0f}Hz  "
        f"filter={filt}  pkt={PICO_UDP_SIZE}B", flush=True)

  seq = 0
  sent = 0
  skipped = 0
  ticks = 0
  next_t = time.perf_counter()
  try:
    for snap in frame_iter:
      ticks += 1
      ts = int(snap["timestamp_ns"])
      rp_raw = np.asarray(snap["right_controller"], dtype=np.float64)
      lp_raw = np.asarray(snap["left_controller"], dtype=np.float64)
      rt = float(snap["right_trigger"])
      lt = float(snap["left_trigger"])

      right_pose, last_good_r, ok_r = _process_pose(rp_raw, right_f, dt, last_good_r)
      left_pose, last_good_l, ok_l = _process_pose(lp_raw, left_f, dt, last_good_l)
      pose_valid = ok_r and ok_l

      if not pose_valid:
        skipped += 1
      else:
        seq += 1
        pkt = pack_pico_packet(seq, ts, right_pose, left_pose, rt, lt, pose_valid)
        if dry_run:
          if seq == 1 or seq % max(1, int(rate_hz)) == 0:
            print(
                f"[pico_udp] seq={seq} R_pos={right_pose[0]:.3f} "
                f"R_qw={right_pose[6]:.4f} valid=1",
                flush=True,
            )
        else:
          assert sock is not None
          sock.sendto(pkt, dest)
        sent += 1

      if max_frames > 0 and sent >= max_frames:
        break
      if max_ticks > 0 and ticks >= max_ticks:
        break

      next_t += period
      sleep_s = next_t - time.perf_counter()
      if sleep_s > 0.0:
        time.sleep(sleep_s)
      else:
        next_t = time.perf_counter()
  except KeyboardInterrupt:
    print(f"\n[pico_udp] 中断 sent={sent} skipped={skipped}", flush=True)
    return 0 if sent > 0 else 1
  finally:
    if sock is not None:
      sock.close()

  print(f"[pico_udp] 完成 sent={sent} skipped={skipped} ticks={ticks}", flush=True)
  return 0 if sent > 0 else 1


def _wait_live_pose(rx: PicoDataReceiver, timeout_s: float) -> bool:
  deadline = time.perf_counter() + max(0.1, timeout_s)
  while time.perf_counter() < deadline:
    snap = rx.read_all()
    rp = np.asarray(snap["poses"]["right_controller"], dtype=np.float64)
    lp = np.asarray(snap["poses"]["left_controller"], dtype=np.float64)
    if pose_xyzw_is_valid(rp) or pose_xyzw_is_valid(lp):
      print("[pico_udp] SDK 位姿已就绪", flush=True)
      return True
    time.sleep(0.05)
  print("[pico_udp] WARN: SDK 超时仍无有效位姿（PC Service/头显/手柄？）", flush=True)
  return False


def live_frame_iter(rx: PicoDataReceiver) -> Iterator[Dict[str, Any]]:
  while True:
    snap = rx.read_all()
    yield {
        "timestamp_ns": int(snap["timestamp_ns"]),
        "right_controller": np.asarray(snap["poses"]["right_controller"], dtype=np.float64),
        "left_controller": np.asarray(snap["poses"]["left_controller"], dtype=np.float64),
        "right_trigger": float(snap["values"]["right_trigger"]),
        "left_trigger": float(snap["values"]["left_trigger"]),
    }


def run_live(
    host: str,
    port: int,
    rate_hz: float,
    pos_params: OneEuroParams,
    ori_params: OneEuroParams,
    use_filter: bool,
    dry_run: bool,
    warmup_s: float,
    max_frames: int,
    max_ticks: int,
) -> int:
  with PicoDataReceiver() as rx:
    time.sleep(0.5)
    _wait_live_pose(rx, warmup_s)
    return _publish_loop(
        live_frame_iter(rx),
        host,
        port,
        rate_hz,
        pos_params,
        ori_params,
        use_filter,
        dry_run,
        max_frames,
        max_ticks,
        "live_sdk",
    )


def run_replay(
    csv_path: Path,
    host: str,
    port: int,
    rate_hz: float,
    pos_params: OneEuroParams,
    ori_params: OneEuroParams,
    use_filter: bool,
    dry_run: bool,
    loop: bool,
    max_frames: int,
    max_ticks: int,
) -> int:
  if not csv_path.is_file():
    print(f"[pico_udp] FAIL: CSV 不存在 {csv_path}", file=sys.stderr)
    return 1
  return _publish_loop(
      iter_pico_csv(csv_path, loop=loop),
      host,
      port,
      rate_hz,
      pos_params,
      ori_params,
      use_filter,
      dry_run,
      max_frames,
      max_ticks,
      f"replay:{csv_path.name}",
  )


def run_replay_servo_cart(
    dir_path: Path,
    host: str,
    port: int,
    rate_hz: float,
    pos_params: OneEuroParams,
    ori_params: OneEuroParams,
    use_filter: bool,
    dry_run: bool,
    loop: bool,
    max_frames: int,
    max_ticks: int,
    decimate: int,
    trigger: float,
) -> int:
  if not dir_path.is_dir():
    print(f"[pico_udp] FAIL: 目录不存在 {dir_path}", file=sys.stderr)
    return 1
  try:
    frame_iter = iter_servo_cart_dir(
        dir_path, decimate=decimate, loop=loop, trigger=trigger
    )
  except FileNotFoundError as e:
    print(f"[pico_udp] FAIL: {e}", file=sys.stderr)
    return 1
  return _publish_loop(
      frame_iter,
      host,
      port,
      rate_hz,
      pos_params,
      ori_params,
      use_filter,
      dry_run,
      max_frames,
      max_ticks,
      f"servo_cart:{dir_path.name}",
  )


def main() -> None:
  parser = argparse.ArgumentParser(description="PICO → One Euro → UDP 发布")
  parser.add_argument("--host", default="127.0.0.1")
  parser.add_argument("--port", type=int, default=30101)
  parser.add_argument("--rate", type=float, default=50.0)
  parser.add_argument("--no-filter", action="store_true", help="直接发布原始位姿")
  parser.add_argument("--dry-run", action="store_true", help="不发 UDP，仅打印/统计")
  parser.add_argument("--replay-csv", type=Path, default=None, help="从 pico_record CSV 回放")
  parser.add_argument(
      "--replay-servo-dir",
      type=Path,
      default=None,
      help="从 test_ServoPByPico 的 left/right_ref_cart.csv 模拟 Pico",
  )
  parser.add_argument(
      "--decimate",
      type=int,
      default=20,
      help="servo_cart 降采样步长（1kHz 数据发 50Hz 用 20）",
  )
  parser.add_argument("--trigger", type=float, default=1.0, help="模拟扳机值")
  parser.add_argument("--loop", action="store_true", help="回放循环")
  parser.add_argument("--max-frames", type=int, default=0, help="发送 N 帧后退出，0=不限")
  parser.add_argument("--max-ticks", type=int, default=0, help="循环 N 次后退出（无有效位姿时配合 live）")
  parser.add_argument("--warmup", type=float, default=5.0, help="live 模式等待有效位姿 [s]")
  parser.add_argument("--pos-min-cutoff", type=float, default=DEFAULT_POS_PARAMS.min_cutoff)
  parser.add_argument("--pos-beta", type=float, default=DEFAULT_POS_PARAMS.beta)
  parser.add_argument("--pos-d-cutoff", type=float, default=DEFAULT_POS_PARAMS.derivative_cutoff)
  parser.add_argument("--ori-min-cutoff", type=float, default=DEFAULT_ORI_PARAMS.min_cutoff)
  parser.add_argument("--ori-beta", type=float, default=DEFAULT_ORI_PARAMS.beta)
  parser.add_argument("--ori-d-cutoff", type=float, default=DEFAULT_ORI_PARAMS.derivative_cutoff)
  args = parser.parse_args()

  pos_p = OneEuroParams(args.pos_min_cutoff, args.pos_beta, args.pos_d_cutoff)
  ori_p = OneEuroParams(args.ori_min_cutoff, args.ori_beta, args.ori_d_cutoff)
  use_filter = not args.no_filter

  if args.replay_servo_dir is not None:
    code = run_replay_servo_cart(
        args.replay_servo_dir, args.host, args.port, args.rate, pos_p, ori_p,
        use_filter, args.dry_run, args.loop, args.max_frames, args.max_ticks,
        args.decimate, args.trigger,
    )
  elif args.replay_csv is not None:
    code = run_replay(
        args.replay_csv, args.host, args.port, args.rate, pos_p, ori_p,
        use_filter, args.dry_run, args.loop, args.max_frames, args.max_ticks,
    )
  else:
    code = run_live(
        args.host, args.port, args.rate, pos_p, ori_p,
        use_filter, args.dry_run, args.warmup, args.max_frames, args.max_ticks,
    )
  raise SystemExit(code)


if __name__ == "__main__":
  main()
