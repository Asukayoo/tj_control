"""从 test_ServoPByPico 的 ref_cart CSV 模拟 Pico 原始位姿（mm→m，qwxyz→xyzw）。"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any, Dict, Iterator, List

import numpy as np

MM_TO_M = 1.0 / 1000.0


def cart_row_to_pose7(row: Dict[str, str]) -> np.ndarray:
  """ref_cart 行 px,py,pz[mm] + qw,qx,qy,qz → SDK [x,y,z,qx,qy,qz,qw][m]。"""
  px = float(row["px"]) * MM_TO_M
  py = float(row["py"]) * MM_TO_M
  pz = float(row["pz"]) * MM_TO_M
  qw = float(row["qw"])
  qx = float(row["qx"])
  qy = float(row["qy"])
  qz = float(row["qz"])
  n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
  if n < 1e-12 or not all(math.isfinite(v) for v in (px, py, pz, qw, qx, qy, qz)):
    return np.array([px, py, pz, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)
  inv = 1.0 / n
  qw, qx, qy, qz = qw * inv, qx * inv, qy * inv, qz * inv
  return np.array([px, py, pz, qx, qy, qz, qw], dtype=np.float64)


def load_ref_cart(path: Path) -> List[Dict[str, str]]:
  with open(path, newline="") as f:
    return list(csv.DictReader(f))


def iter_servo_cart_dir(
    dir_path: str | Path,
    decimate: int = 20,
    loop: bool = True,
    trigger: float = 1.0,
) -> Iterator[Dict[str, Any]]:
  """读取 left/right_ref_cart.csv，按 decimate 降采样（默认 1kHz→50Hz）。"""
  root = Path(dir_path)
  left_path = root / "left_ref_cart.csv"
  right_path = root / "right_ref_cart.csv"
  if not left_path.is_file() or not right_path.is_file():
    raise FileNotFoundError(f"缺少 ref_cart: {left_path} / {right_path}")

  left_rows = load_ref_cart(left_path)
  right_rows = load_ref_cart(right_path)
  n = min(len(left_rows), len(right_rows))
  if n == 0:
    return

  step = max(1, decimate)
  indices = list(range(0, n, step))

  while True:
    for i in indices:
      cycle = int(left_rows[i].get("cycle", i))
      ts_ns = cycle * 1_000_000  # 1kHz 周期 → ns
      yield {
          "timestamp_ns": ts_ns,
          "right_controller": cart_row_to_pose7(right_rows[i]),
          "left_controller": cart_row_to_pose7(left_rows[i]),
          "right_trigger": trigger,
          "left_trigger": trigger,
          "cycle": cycle,
      }
    if not loop:
      break
