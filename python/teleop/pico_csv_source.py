"""从 pico_record_*.csv 读取位姿（回放测试用）。"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Dict, Iterator, List

import numpy as np


def _row_pose_xyzw(row: Dict[str, str], prefix: str) -> np.ndarray:
  """CSV [x,y,z,qw,qx,qy,qz] → SDK [x,y,z,qx,qy,qz,qw]。"""
  return np.array(
      [
          float(row[f"{prefix}_x"]),
          float(row[f"{prefix}_y"]),
          float(row[f"{prefix}_z"]),
          float(row[f"{prefix}_qx"]),
          float(row[f"{prefix}_qy"]),
          float(row[f"{prefix}_qz"]),
          float(row[f"{prefix}_qw"]),
      ],
      dtype=np.float64,
  )


def load_pico_csv(path: str | Path) -> List[Dict[str, Any]]:
  rows: List[Dict[str, Any]] = []
  with open(path, newline="") as f:
    for row in csv.DictReader(f):
      rows.append(
          {
              "timestamp_ns": int(row["timestamp_ns"]),
              "right_controller": _row_pose_xyzw(row, "right_ctrl"),
              "left_controller": _row_pose_xyzw(row, "left_ctrl"),
              "headset": _row_pose_xyzw(row, "headset"),
              "right_trigger": float(row["right_trigger"]),
              "left_trigger": float(row["left_trigger"]),
          }
      )
  return rows


def iter_pico_csv(path: str | Path, loop: bool = True) -> Iterator[Dict[str, Any]]:
  frames = load_pico_csv(path)
  if not frames:
    return
  while True:
    for snap in frames:
      yield snap
    if not loop:
      break
