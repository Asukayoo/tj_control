"""单独探测 Pico SDK 原始位姿 vs One Euro 滤波（先跑通再开 UDP 发布）。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

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
except ImportError:
  from one_euro_filter import (  # type: ignore
      OneEuroParams,
      PoseOneEuroFilter,
      pose_xyzw_is_valid,
  )

DEFAULT_POS_PARAMS = OneEuroParams(1.15, 0.5, 1.2)
DEFAULT_ORI_PARAMS = OneEuroParams(1.5, 0.5, 1.2)


def _fmt_pose(name: str, pose: np.ndarray, valid: bool) -> str:
  p = np.asarray(pose, dtype=np.float64)
  qn = float(np.linalg.norm(p[3:7])) if np.all(np.isfinite(p[3:7])) else float("nan")
  return (
      f"{name}: pos=[{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}] "
      f"quat=[{p[3]:.4f},{p[4]:.4f},{p[5]:.4f},{p[6]:.4f}] "
      f"|norm|={qn:.4f} valid={int(valid)}"
  )


def run(
    rate_hz: float,
    seconds: float,
    use_filter: bool,
    pos_params: OneEuroParams,
    ori_params: OneEuroParams,
) -> int:
  period = 1.0 / max(1e-3, rate_hz)
  dt = period
  right_f = PoseOneEuroFilter(pos_params, ori_params) if use_filter else None
  left_f = PoseOneEuroFilter(pos_params, ori_params) if use_filter else None

  raw_valid = 0
  filt_valid = 0
  samples = 0
  raw_nan_quat = 0

  print(f"[probe] rate={rate_hz:.0f}Hz  filter={'on' if use_filter else 'off'}  "
        f"duration={seconds:.1f}s")
  print("[probe] 需 PC Service + 头显/手柄已连接", flush=True)

  t_end = time.perf_counter() + max(0.1, seconds)
  next_t = time.perf_counter()
  try:
    with PicoDataReceiver() as rx:
      time.sleep(0.5)
      deadline = time.perf_counter() + 3.0
      while time.perf_counter() < deadline:
        snap = rx.read_all()
        rp = np.asarray(snap["poses"]["right_controller"], dtype=np.float64)
        if pose_xyzw_is_valid(rp):
          print("[probe] SDK 位姿已就绪", flush=True)
          break
        time.sleep(0.05)
      while time.perf_counter() < t_end:
        snap = rx.read_all()
        rp = np.asarray(snap["poses"]["right_controller"], dtype=np.float64)
        lp = np.asarray(snap["poses"]["left_controller"], dtype=np.float64)
        rt = float(snap["values"]["right_trigger"])
        lt = float(snap["values"]["left_trigger"])

        raw_r_ok = pose_xyzw_is_valid(rp)
        raw_l_ok = pose_xyzw_is_valid(lp)
        samples += 1
        if raw_r_ok or raw_l_ok:
          raw_valid += 1
        if not raw_r_ok and np.all(np.isfinite(rp[:3])):
          raw_nan_quat += 1

        if use_filter and right_f is not None and left_f is not None:
          rp_pos, rp_q = right_f.filter_pose_xyzw(dt, rp[:3], rp[3:7])
          lp_pos, lp_q = left_f.filter_pose_xyzw(dt, lp[:3], lp[3:7])
          fr = np.concatenate([rp_pos, rp_q])
          fl = np.concatenate([lp_pos, lp_q])
          filt_r_ok = pose_xyzw_is_valid(fr)
          filt_l_ok = pose_xyzw_is_valid(fl)
          if filt_r_ok or filt_l_ok:
            filt_valid += 1
        else:
          fr, fl = rp, lp
          filt_r_ok, filt_l_ok = raw_r_ok, raw_l_ok

        if samples == 1 or samples % max(1, int(rate_hz)) == 0:
          print(
              f"[probe] n={samples} trig=({lt:.2f},{rt:.2f}) "
              f"raw_R_valid={int(raw_r_ok)} filt_R_valid={int(filt_r_ok)}"
          )
          print(f"  {_fmt_pose('raw_R', rp, raw_r_ok)}")
          if use_filter:
            print(f"  {_fmt_pose('flt_R', fr, filt_r_ok)}")

        next_t += period
        sleep_s = next_t - time.perf_counter()
        if sleep_s > 0.0:
          time.sleep(sleep_s)
        else:
          next_t = time.perf_counter()
  except KeyboardInterrupt:
    print("\n[probe] 中断")

  print("\n[probe] 汇总:")
  print(f"  samples={samples}  raw_valid_frames={raw_valid}  "
        f"raw_bad_quat_right={raw_nan_quat}")
  if use_filter:
    print(f"  filt_valid_frames={filt_valid}")
  if raw_valid == 0:
    print("  结论: SDK 原始四元数无效（零/NaN）→ 先检查 PC Service / 设备连接。")
  elif use_filter and filt_valid < raw_valid:
    print("  结论: 滤波后有效帧变少 → 检查 one_euro_filter 或原始跳变过大。")
  elif use_filter and filt_valid == raw_valid and raw_valid > 0:
    print("  结论: 原始与滤波均有效 → 可启动 pico_udp_publisher。")
  elif not use_filter and raw_valid > 0:
    print("  结论: 原始位姿有效 → 可加 --filter 再测滤波。")
  return 0 if (use_filter and filt_valid > 0) or (not use_filter and raw_valid > 0) else 1


def main() -> None:
  parser = argparse.ArgumentParser(description="Pico SDK 原始/滤波位姿探测")
  parser.add_argument("--rate", type=float, default=50.0)
  parser.add_argument("--seconds", type=float, default=10.0)
  parser.add_argument("--filter", action="store_true", help="启用 One Euro 滤波对比")
  parser.add_argument("--pos-min-cutoff", type=float, default=DEFAULT_POS_PARAMS.min_cutoff)
  parser.add_argument("--pos-beta", type=float, default=DEFAULT_POS_PARAMS.beta)
  parser.add_argument("--pos-d-cutoff", type=float, default=DEFAULT_POS_PARAMS.derivative_cutoff)
  parser.add_argument("--ori-min-cutoff", type=float, default=DEFAULT_ORI_PARAMS.min_cutoff)
  parser.add_argument("--ori-beta", type=float, default=DEFAULT_ORI_PARAMS.beta)
  parser.add_argument("--ori-d-cutoff", type=float, default=DEFAULT_ORI_PARAMS.derivative_cutoff)
  args = parser.parse_args()

  pos_p = OneEuroParams(args.pos_min_cutoff, args.pos_beta, args.pos_d_cutoff)
  ori_p = OneEuroParams(args.ori_min_cutoff, args.ori_beta, args.ori_d_cutoff)
  raise SystemExit(run(args.rate, args.seconds, args.filter, pos_p, ori_p))


if __name__ == "__main__":
  main()
