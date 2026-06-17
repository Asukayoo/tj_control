#!/usr/bin/env python3
"""One Euro 对比：1kHz 源数据 → 50Hz 截取 → 滤波（与 Pico 发布链路一致）。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
_TELEOP = ROOT / "python/teleop"
if str(_TELEOP) not in sys.path:
  sys.path.insert(0, str(_TELEOP))

from one_euro_filter import (  # noqa: E402
  OneEuroParams,
  PoseOneEuroFilter,
  _quat_conj_wxyz,
  _quat_ln_wxyz,
  _quat_mul_wxyz,
  xyzw_to_wxyz,
)
from pico_servo_cart_source import cart_row_to_pose7, load_ref_cart  # noqa: E402

DEFAULT_DIR = ROOT / "data/test_ServoPByPico"
SOURCE_HZ = 1000.0  # ref_cart 每行 = 1kHz 一拍
PICO_HZ = 50.0      # Pico / ServoPByPico 外环频率
DEFAULT_POS = OneEuroParams(1.15, 0.5, 1.2)
DEFAULT_ORI = OneEuroParams(1.5, 0.5, 1.2)


def _quat_unwrap_wxyz(q: np.ndarray) -> np.ndarray:
  out = q.copy()
  for i in range(1, len(out)):
    if float(np.dot(out[i], out[i - 1])) < 0.0:
      out[i] = -out[i]
  return out


def _omega_series(q_wxyz: np.ndarray, dt: float) -> np.ndarray:
  n = len(q_wxyz)
  w = np.zeros((n, 3), dtype=np.float64)
  for i in range(1, n):
    dq = _quat_mul_wxyz(q_wxyz[i], _quat_conj_wxyz(q_wxyz[i - 1]))
    w[i] = _quat_ln_wxyz(dq) / dt
  if n > 1:
    w[0] = w[1]
  return w


def _vel_fd(x: np.ndarray, dt: float) -> np.ndarray:
  v = np.zeros_like(x)
  if len(x) < 2:
    return v
  v[1:] = (x[1:] - x[:-1]) / dt
  v[0] = v[1]
  return v


def load_arm_series_1khz(
    dir_path: Path,
    arm: str,
    cycle_start: int,
    max_samples_1k: int,
) -> tuple[np.ndarray, np.ndarray]:
  """1kHz 逐行加载 ref_cart。"""
  fname = "right_ref_cart.csv" if arm == "right" else "left_ref_cart.csv"
  rows = load_ref_cart(dir_path / fname)
  poses: list[np.ndarray] = []
  cycles: list[int] = []
  for i, row in enumerate(rows):
    cyc = int(row.get("cycle", i))
    if cyc < cycle_start:
      continue
    poses.append(cart_row_to_pose7(row))
    cycles.append(cyc)
    if max_samples_1k > 0 and len(poses) >= max_samples_1k:
      break
  if not poses:
    raise ValueError(f"无样本 arm={arm} cycle_start={cycle_start}")
  return np.asarray(cycles, dtype=np.float64), np.stack(poses, axis=0)


def subsample_1khz_to_hz(
    cycles: np.ndarray,
    poses: np.ndarray,
    out_hz: float,
) -> tuple[np.ndarray, np.ndarray, float]:
  """从 1kHz 序列按步长截取到 out_hz（50Hz → 每 20 点取 1）。"""
  step = max(1, int(round(SOURCE_HZ / out_hz)))
  idx = np.arange(0, len(poses), step)
  dt = 1.0 / out_hz
  return cycles[idx], poses[idx], dt


def filter_poses(
    poses_xyzw: np.ndarray, dt: float, pos_p: OneEuroParams, ori_p: OneEuroParams
) -> np.ndarray:
  filt = PoseOneEuroFilter(pos_p, ori_p)
  out = np.zeros_like(poses_xyzw)
  for i in range(len(poses_xyzw)):
    p = poses_xyzw[i]
    pos_f, q_f = filt.filter_pose_xyzw(dt, p[:3], p[3:7])
    out[i] = np.concatenate([pos_f, q_f])
  return out


def poses_to_wxyz(poses_xyzw: np.ndarray) -> np.ndarray:
  q = np.zeros((len(poses_xyzw), 4), dtype=np.float64)
  for i, p in enumerate(poses_xyzw):
    wxyz = xyzw_to_wxyz(p[3:7])
    q[i] = wxyz if wxyz is not None else np.array([1.0, 0.0, 0.0, 0.0])
  return _quat_unwrap_wxyz(q)


def plot_component_compare(
    ax: plt.Axes,
    t: np.ndarray,
    raw: np.ndarray,
    filt: np.ndarray,
    labels: tuple[str, ...],
    ylabel: str,
    title: str,
) -> None:
  colors = ("C0", "C1", "C2", "C3")
  for k, name in enumerate(labels):
    c = colors[k % len(colors)]
    ax.plot(t, raw[:, k], color=c, ls="-", lw=1.0, alpha=0.85, label=f"{name} raw")
    ax.plot(t, filt[:, k], color=c, ls="--", lw=1.2, label=f"{name} filt")
  ax.set_ylabel(ylabel)
  ax.set_title(title)
  ax.grid(True, alpha=0.3)
  ax.legend(loc="best", fontsize=7, ncol=2)


def build_figures(
    t: np.ndarray,
    pos_raw: np.ndarray,
    pos_filt: np.ndarray,
    quat_raw: np.ndarray,
    quat_filt: np.ndarray,
    v_raw: np.ndarray,
    v_filt: np.ndarray,
    w_raw: np.ndarray,
    w_filt: np.ndarray,
    arm: str,
    out_hz: float,
) -> tuple[plt.Figure, plt.Figure]:
  tag = f"1kHz→{out_hz:.0f}Hz subsample, One Euro dt={1000/out_hz:.0f}ms"
  fig_pos, axes_pos = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
  fig_pos.suptitle(f"{arm.upper()} Position — {tag}")
  plot_component_compare(
      axes_pos[0], t, pos_raw, pos_filt, ("x", "y", "z"), "pos [m]", "position xyz @50Hz"
  )
  plot_component_compare(
      axes_pos[1], t, v_raw, v_filt, ("vx", "vy", "vz"), "vel [m/s]", "linear velocity @50Hz"
  )
  axes_pos[1].set_xlabel("t [s]")
  fig_pos.tight_layout()

  fig_ori, axes_ori = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
  fig_ori.suptitle(f"{arm.upper()} Orientation — {tag}")
  plot_component_compare(
      axes_ori[0], t, quat_raw, quat_filt, ("w", "x", "y", "z"), "quat wxyz", "orientation wxyz @50Hz"
  )
  plot_component_compare(
      axes_ori[1], t, w_raw, w_filt, ("wx", "wy", "wz"), "omega [rad/s]", "angular velocity @50Hz"
  )
  axes_ori[1].set_xlabel("t [s]")
  fig_ori.tight_layout()
  return fig_pos, fig_ori


def main() -> int:
  parser = argparse.ArgumentParser(
      description="1kHz 源 → 50Hz 截取 → One Euro 滤波对比"
  )
  parser.add_argument("--dir", type=Path, default=DEFAULT_DIR)
  parser.add_argument("--arm", choices=("left", "right"), default="right")
  parser.add_argument(
      "--out-hz",
      type=float,
      default=PICO_HZ,
      help="从 1kHz 截取后的序列频率（默认 50，与 Pico 发布一致）",
  )
  parser.add_argument("--cycle-start", type=int, default=1380)
  parser.add_argument(
      "--duration",
      type=float,
      default=16.0,
      help="片段时长 [s]（按 1kHz 加载，再截取到 out-hz）",
  )
  parser.add_argument("--save-dir", type=Path, default=None)
  parser.add_argument("--show", action="store_true")
  args = parser.parse_args()

  max_1k = max(1, int(args.duration * SOURCE_HZ))
  cycles_1k, raw_1k = load_arm_series_1khz(
      args.dir, args.arm, args.cycle_start, max_1k
  )

  # 1) 1kHz → out_hz 截取（raw，不滤波）
  cycles_sub, raw_50, dt = subsample_1khz_to_hz(cycles_1k, raw_1k, args.out_hz)
  # 2) 在 50Hz 序列上 One Euro 滤波
  filt_50 = filter_poses(raw_50, dt, DEFAULT_POS, DEFAULT_ORI)

  t = (cycles_sub - cycles_sub[0]) / SOURCE_HZ

  pos_raw = raw_50[:, :3]
  pos_filt = filt_50[:, :3]
  quat_raw = poses_to_wxyz(raw_50)
  quat_filt = poses_to_wxyz(filt_50)
  v_raw = _vel_fd(pos_raw, dt)
  v_filt = _vel_fd(pos_filt, dt)
  w_raw = _omega_series(quat_raw, dt)
  w_filt = _omega_series(quat_filt, dt)

  fig_pos, fig_ori = build_figures(
      t, pos_raw, pos_filt, quat_raw, quat_filt, v_raw, v_filt, w_raw, w_filt,
      args.arm, args.out_hz,
  )

  dv = np.linalg.norm(v_raw - v_filt, axis=1)
  dw = np.linalg.norm(w_raw - w_filt, axis=1)
  step = int(round(SOURCE_HZ / args.out_hz))
  print(
      f"source={SOURCE_HZ:.0f}Hz  subsample_step={step}  out={args.out_hz:.0f}Hz  "
      f"dt={dt*1e3:.1f}ms  samples_1k={len(raw_1k)}  samples_out={len(raw_50)}  "
      f"duration={t[-1]:.2f}s  max|dv|={dv.max():.4f}m/s  max|dw|={dw.max():.4f}rad/s"
  )

  if args.save_dir:
    args.save_dir.mkdir(parents=True, exist_ok=True)
    fig_pos.savefig(args.save_dir / f"one_euro_{args.arm}_position_50hz.png", dpi=150)
    fig_ori.savefig(args.save_dir / f"one_euro_{args.arm}_orientation_50hz.png", dpi=150)
    print(f"saved -> {args.save_dir}")

  if args.show:
    plt.show()
  else:
    plt.close("all")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
