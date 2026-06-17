#!/usr/bin/env python3
"""RT teleop 可视化入口（1 kHz 关节 UDP）。"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

if __name__ == "__main__":
  vis = Path(__file__).resolve().parent / "urdf_pybullet_vis.py"
  sys.argv[0] = str(vis)
  runpy.run_path(str(vis), run_name="__main__")
