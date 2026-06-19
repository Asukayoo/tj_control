"""Pico SDK 世界系 → 机器人常用右手系（X前 Y左 Z上）固定变换。"""

from __future__ import annotations

import numpy as np

# SDK 文档：左手系 X右 Y上 Z里；目标 T：右手系 X前 Y左 Z上
# 位置：v_T = R @ v_P（与 Unity→ROS 位置 remap 一致）
R_PICO_SDK_TO_FLUZ = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)


def quat_xyzw_to_rot3(q_xyzw: np.ndarray) -> np.ndarray:
  """四元数 [qx,qy,qz,qw] → 3×3 旋转矩阵。"""
  q = np.asarray(q_xyzw, dtype=np.float64).reshape(4)
  n = float(np.linalg.norm(q))
  if n < 1e-12:
    return np.eye(3, dtype=np.float64)
  qx, qy, qz, qw = q / n
  return np.array(
      [
          [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
          [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
          [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
      ],
      dtype=np.float64,
  )


def rot3_to_quat_xyzw(rot: np.ndarray) -> np.ndarray:
  """3×3 旋转矩阵 → [qx,qy,qz,qw]。"""
  m = np.asarray(rot, dtype=np.float64).reshape(3, 3)
  tr = float(np.trace(m))
  if tr > 0.0:
    s = np.sqrt(tr + 1.0) * 2.0
    qw = 0.25 * s
    qx = (m[2, 1] - m[1, 2]) / s
    qy = (m[0, 2] - m[2, 0]) / s
    qz = (m[1, 0] - m[0, 1]) / s
  elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
    s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
    qw = (m[2, 1] - m[1, 2]) / s
    qx = 0.25 * s
    qy = (m[0, 1] + m[1, 0]) / s
    qz = (m[0, 2] + m[2, 0]) / s
  elif m[1, 1] > m[2, 2]:
    s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
    qw = (m[0, 2] - m[2, 0]) / s
    qx = (m[0, 1] + m[1, 0]) / s
    qy = 0.25 * s
    qz = (m[1, 2] + m[2, 1]) / s
  else:
    s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    qw = (m[1, 0] - m[0, 1]) / s
    qx = (m[0, 2] + m[2, 0]) / s
    qy = (m[1, 2] + m[2, 1]) / s
    qz = 0.25 * s
  q = np.array([qx, qy, qz, qw], dtype=np.float64)
  n = float(np.linalg.norm(q))
  if n < 1e-12:
    return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
  q = q / n
  if q[3] < 0.0:
    q = -q
  return q


def quat_sdk_to_fluz(
    q_xyzw: np.ndarray,
    rot_tp: np.ndarray | None = None,
) -> np.ndarray:
  """姿态：相似变换 R @ R_body @ R.T（保持 identity，旋转轴/方向正确）。"""
  r = R_PICO_SDK_TO_FLUZ if rot_tp is None else np.asarray(rot_tp, dtype=np.float64)
  rb = quat_xyzw_to_rot3(q_xyzw)
  rt = r @ rb @ r.T
  return rot3_to_quat_xyzw(rt)


def transform_pose_sdk_to_fluz(
    pose7: np.ndarray,
    rot_tp: np.ndarray | None = None,
) -> np.ndarray:
  """SDK [x,y,z,qx,qy,qz,qw] → FLUZ 右手系同格式位姿。"""
  p = np.asarray(pose7, dtype=np.float64).reshape(7)
  r = R_PICO_SDK_TO_FLUZ if rot_tp is None else np.asarray(rot_tp, dtype=np.float64)
  pos_t = r @ p[:3]
  quat_t = quat_sdk_to_fluz(p[3:7], r)
  return np.concatenate([pos_t, quat_t])


def relative_pose(
    child7: np.ndarray,
    parent7: np.ndarray,
) -> np.ndarray:
  """child 在 parent 系下的位姿 [x,y,z,qx,qy,qz,qw]（与 C++ PoseInverse+Compose 一致）。"""
  child = np.asarray(child7, dtype=np.float64).reshape(7)
  parent = np.asarray(parent7, dtype=np.float64).reshape(7)
  r_parent = quat_xyzw_to_rot3(parent[3:7])
  r_child = quat_xyzw_to_rot3(child[3:7])
  r_parent_inv = r_parent.T
  pos_rel = r_parent_inv @ (child[:3] - parent[:3])
  quat_rel = rot3_to_quat_xyzw(r_parent_inv @ r_child)
  return np.concatenate([pos_rel, quat_rel])
