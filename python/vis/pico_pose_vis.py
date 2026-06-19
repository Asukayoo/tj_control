#!/usr/bin/env python3
"""Pico 硬件位姿实时 3D 可视化：50 Hz 轮询 SDK，matplotlib 显示头显与双手柄。"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

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
  from python.teleop.pico_coord import (
      quat_xyzw_to_rot3,
      transform_pose_sdk_to_fluz,
  )
except ImportError:
  from one_euro_filter import (  # type: ignore
      OneEuroParams,
      PoseOneEuroFilter,
      pose_xyzw_is_valid,
  )
  from pico_coord import (  # type: ignore
      quat_xyzw_to_rot3,
      transform_pose_sdk_to_fluz,
  )

DEFAULT_POS_PARAMS = OneEuroParams(1.15, 0.5, 1.2)
DEFAULT_ORI_PARAMS = OneEuroParams(1.5, 0.5, 1.2)
AXIS_LEN_M = 0.08
TRAIL_LEN = 50
VIEW_SPAN_M = 1.2

_DEVICE_STYLE = {
    "headset": {"color": "#2ecc71", "marker": "o", "label": "Headset"},
    "left_controller": {"color": "#3498db", "marker": "s", "label": "Left Ctrl"},
    "right_controller": {"color": "#e74c3c", "marker": "s", "label": "Right Ctrl"},
}


def _fmt_pose_line(label: str, pose: np.ndarray, valid: bool) -> str:
  p = np.asarray(pose, dtype=np.float64)
  tag = "OK" if valid else "hold"
  return (
      f"{label} [{tag}] "
      f"pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) "
      f"quat(xyzw)=({p[3]:+.4f},{p[4]:+.4f},{p[5]:+.4f},{p[6]:+.4f})"
  )


@dataclass
class DeviceState:
  """单设备位姿与轨迹缓存。"""

  pose: np.ndarray
  valid: bool
  trail: List[np.ndarray]


def _process_pose(
    raw_pose: np.ndarray,
    filt: PoseOneEuroFilter | None,
    dt: float,
    last_good: np.ndarray | None,
) -> Tuple[np.ndarray, np.ndarray | None, bool]:
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


def _wait_live_pose(rx: PicoDataReceiver, timeout_s: float) -> bool:
  deadline = time.perf_counter() + max(0.1, timeout_s)
  while time.perf_counter() < deadline:
    snap = rx.read_all()
    for name in ("headset", "right_controller", "left_controller"):
      if pose_xyzw_is_valid(np.asarray(snap["poses"][name], dtype=np.float64)):
        print(f"[pico_vis] SDK 位姿已就绪 ({name})", flush=True)
        return True
    time.sleep(0.05)
  print("[pico_vis] WARN: 超时仍无有效位姿（PC Service/头显/手柄？）", flush=True)
  return False


class PicoPosePlot:
  """matplotlib 3D 位姿画布。"""

  def __init__(
      self,
      axis_len: float,
      trail_len: int,
      view_span: float,
      use_filter: bool,
      frame: str,
  ) -> None:
    self._axis_len = axis_len
    self._trail_len = trail_len
    self._view_span = view_span
    self._use_filter = use_filter
    self._frame = frame
    data_mode = "filtered" if use_filter else "raw"
    frame_label = "FLUZ(X前Y左Z上)" if frame == "fluz" else "SDK(X右Y上Z里)"
    self._fig = plt.figure(figsize=(10, 7))
    self._ax = self._fig.add_subplot(111, projection="3d")
    self._status = self._fig.text(
        0.02, 0.98, "", transform=self._fig.transFigure, va="top", fontsize=8, family="monospace",
    )
    if frame == "fluz":
      self._ax.set_xlabel("X 前 [m]")
      self._ax.set_ylabel("Y 左 [m]")
      self._ax.set_zlabel("Z 上 [m]")
    else:
      self._ax.set_xlabel("X 右 [m]")
      self._ax.set_ylabel("Y 上 [m]")
      self._ax.set_zlabel("Z 里 [m]")
    self._ax.set_title(f"Pico Pose 50 Hz ({data_mode}, {frame_label})")
    self._ax.view_init(elev=25, azim=-60)

  def draw(self, devices: Dict[str, DeviceState], frame_idx: int, fps: float) -> None:
    self._ax.cla()
    if self._frame == "fluz":
      self._ax.set_xlabel("X 前 [m]")
      self._ax.set_ylabel("Y 左 [m]")
      self._ax.set_zlabel("Z 上 [m]")
    else:
      self._ax.set_xlabel("X 右 [m]")
      self._ax.set_ylabel("Y 上 [m]")
      self._ax.set_zlabel("Z 里 [m]")
    data_mode = "filtered" if self._use_filter else "raw"
    frame_label = "FLUZ(X前Y左Z上)" if self._frame == "fluz" else "SDK(X右Y上Z里)"
    self._ax.set_title(f"Pico Pose 50 Hz ({data_mode}, {frame_label})")

    valid_pts: List[np.ndarray] = []
    info_lines = [f"frame={frame_idx}  fps={fps:.1f}  RGB=设备XYZ"]
    for name, style in _DEVICE_STYLE.items():
      dev = devices[name]
      pos = dev.pose[:3]
      info_lines.append(_fmt_pose_line(style["label"], dev.pose, dev.valid))
      if dev.valid:
        valid_pts.append(pos)
      self._ax.scatter(
          pos[0], pos[1], pos[2],
          c=style["color"], marker=style["marker"], s=60,
          label=f"{style['label']} {'OK' if dev.valid else 'hold'}",
      )
      if dev.valid:
        rot = quat_xyzw_to_rot3(dev.pose[3:7])
        for axis_idx, axis_color in enumerate(("r", "g", "b")):
          direction = rot[:, axis_idx] * self._axis_len
          self._ax.quiver(
              pos[0], pos[1], pos[2],
              direction[0], direction[1], direction[2],
              color=axis_color, linewidth=1.2, arrow_length_ratio=0.25,
          )
      if self._frame == "fluz" and name == "headset":
        o = np.zeros(3, dtype=np.float64)
        for axis_idx, (axis_color, lbl) in enumerate(
            (("r", "X前"), ("g", "Y左"), ("b", "Z上"))
        ):
          e = np.zeros(3, dtype=np.float64)
          e[axis_idx] = self._axis_len * 1.5
          self._ax.quiver(
              o[0], o[1], o[2], e[0], e[1], e[2],
              color=axis_color, alpha=0.45, linewidth=1.0, arrow_length_ratio=0.2,
          )
      if len(dev.trail) >= 2:
        trail = np.asarray(dev.trail, dtype=np.float64)
        self._ax.plot(
            trail[:, 0], trail[:, 1], trail[:, 2],
            color=style["color"], alpha=0.35, linewidth=1.0,
        )

    if valid_pts:
      center = np.mean(np.vstack(valid_pts), axis=0)
    else:
      center = np.zeros(3, dtype=np.float64)
    half = self._view_span * 0.5
    self._ax.set_xlim(center[0] - half, center[0] + half)
    self._ax.set_ylim(center[1] - half, center[1] + half)
    self._ax.set_zlim(center[2] - half, center[2] + half)
    self._ax.set_box_aspect((1, 1, 1))
    self._ax.legend(loc="upper right", fontsize=8)
    self._status.set_text("\n".join(info_lines))
    self._fig.canvas.draw_idle()


def run(
    rate_hz: float,
    warmup_s: float,
    use_filter: bool,
    pos_params: OneEuroParams,
    ori_params: OneEuroParams,
    axis_len: float,
    trail_len: int,
    view_span: float,
    frame: str,
) -> int:
  period = 1.0 / max(1e-3, rate_hz)
  dt = period
  filt_map = {
      name: PoseOneEuroFilter(pos_params, ori_params) if use_filter else None
      for name in _DEVICE_STYLE
  }
  last_good: Dict[str, np.ndarray | None] = {name: None for name in _DEVICE_STYLE}
  trails: Dict[str, List[np.ndarray]] = {name: [] for name in _DEVICE_STYLE}
  devices: Dict[str, DeviceState] = {
      name: DeviceState(np.zeros(7, dtype=np.float64), False, trails[name])
      for name in _DEVICE_STYLE
  }

  plot = PicoPosePlot(axis_len, trail_len, view_span, use_filter, frame)
  rx_holder: List[PicoDataReceiver] = []
  frame_idx = 0
  fps = 0.0
  t_last = time.perf_counter()

  def on_frame(_i: int) -> None:
    nonlocal frame_idx, fps, t_last
    rx = rx_holder[0]
    snap = rx.read_all()
    for name in _DEVICE_STYLE:
      raw = np.asarray(snap["poses"][name], dtype=np.float64)
      pose, last_good[name], valid = _process_pose(raw, filt_map[name], dt, last_good[name])
      if valid and frame == "fluz":
        pose = transform_pose_sdk_to_fluz(pose)
        last_good[name] = pose.copy()
      if valid:
        trails[name].append(pose[:3].copy())
        if len(trails[name]) > trail_len:
          trails[name].pop(0)
      devices[name] = DeviceState(pose, valid, trails[name])

    now = time.perf_counter()
    dt_fps = now - t_last
    if dt_fps > 1e-6:
      fps = 0.9 * fps + 0.1 * (1.0 / dt_fps)
    t_last = now
    frame_idx += 1
    plot.draw(devices, frame_idx, fps)

  filt_label = "one_euro" if use_filter else "raw"
  frame_label = "fluz(X前Y左Z上)" if frame == "fluz" else "sdk"
  print(f"[pico_vis] rate={rate_hz:.0f}Hz  data={filt_label}  frame={frame_label}  "
        f"需 PC Service + 设备已连接", flush=True)

  with PicoDataReceiver() as rx:
    rx_holder.append(rx)
    time.sleep(0.5)
    _wait_live_pose(rx, warmup_s)
    _ = FuncAnimation(plot._fig, on_frame, interval=period * 1000.0, cache_frame_data=False)
    plt.show()

  return 0


def main() -> None:
  parser = argparse.ArgumentParser(description="Pico 头显/手柄位姿 3D 可视化")
  parser.add_argument("--rate", type=float, default=50.0, help="采样/刷新频率 Hz")
  parser.add_argument("--warmup", type=float, default=5.0, help="等待有效位姿超时 [s]")
  filt_grp = parser.add_mutually_exclusive_group()
  filt_grp.add_argument("--filter", action="store_true", help="启用 One Euro 滤波")
  filt_grp.add_argument(
      "--no-filter",
      action="store_true",
      help="显示 SDK 原始位姿（默认）",
  )
  parser.add_argument(
      "--frame",
      choices=("sdk", "fluz"),
      default="sdk",
      help="输出坐标系：sdk=Pico 原始；fluz=右手系 X前 Y左 Z上",
  )
  parser.add_argument("--axis-len", type=float, default=AXIS_LEN_M, help="局部坐标轴长度 [m]")
  parser.add_argument("--trail-len", type=int, default=TRAIL_LEN, help="轨迹点数")
  parser.add_argument("--view-span", type=float, default=VIEW_SPAN_M, help="视窗边长 [m]")
  parser.add_argument("--pos-min-cutoff", type=float, default=DEFAULT_POS_PARAMS.min_cutoff)
  parser.add_argument("--pos-beta", type=float, default=DEFAULT_POS_PARAMS.beta)
  parser.add_argument("--pos-d-cutoff", type=float, default=DEFAULT_POS_PARAMS.derivative_cutoff)
  parser.add_argument("--ori-min-cutoff", type=float, default=DEFAULT_ORI_PARAMS.min_cutoff)
  parser.add_argument("--ori-beta", type=float, default=DEFAULT_ORI_PARAMS.beta)
  parser.add_argument("--ori-d-cutoff", type=float, default=DEFAULT_ORI_PARAMS.derivative_cutoff)
  args = parser.parse_args()

  pos_p = OneEuroParams(args.pos_min_cutoff, args.pos_beta, args.pos_d_cutoff)
  ori_p = OneEuroParams(args.ori_min_cutoff, args.ori_beta, args.ori_d_cutoff)
  raise SystemExit(run(
      args.rate, args.warmup, args.filter, pos_p, ori_p,
      args.axis_len, args.trail_len, args.view_span, args.frame,
  ))


if __name__ == "__main__":
  main()
