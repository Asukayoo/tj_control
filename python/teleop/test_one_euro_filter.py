"""One Euro 单元测试（无需 Pico 硬件）。"""

from __future__ import annotations

import math
import unittest

import numpy as np

from one_euro_filter import (
    OneEuroFilterQuat,
    OneEuroParams,
    PoseOneEuroFilter,
    pose_xyzw_is_valid,
)


class TestOneEuroFilter(unittest.TestCase):
  def test_zero_quat_input_no_nan(self) -> None:
    f = OneEuroFilterQuat(OneEuroParams())
    out = f.filter(0.02, np.zeros(4))
    self.assertTrue(np.all(np.isfinite(out)))
    self.assertAlmostEqual(float(np.linalg.norm(out)), 1.0, places=5)

  def test_rotation_sequence_finite(self) -> None:
    f = OneEuroFilterQuat(OneEuroParams())
    dt = 0.02
    for i in range(200):
      a = i * 0.05
      q = np.array([math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)], dtype=np.float64)
      out = f.filter(dt, q)
      self.assertTrue(np.all(np.isfinite(out)), msg=f"step {i}")

  def test_pose_filter_from_sdk_layout(self) -> None:
    pf = PoseOneEuroFilter()
    pos = np.array([0.1, 0.2, 0.3], dtype=np.float64)
    quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    for _ in range(50):
      pos_f, quat_f = pf.filter_pose_xyzw(0.02, pos, quat)
      pose = np.concatenate([pos_f, quat_f])
      self.assertTrue(pose_xyzw_is_valid(pose))


if __name__ == "__main__":
  unittest.main()
