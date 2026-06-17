"""One Euro Filter（对齐 OSVR-Core EigenFilters.h + VRPN 四元数导数）。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

_PI = math.pi
_IDENTITY_WXYZ = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


@dataclass
class OneEuroParams:
  """滤波参数：min_cutoff [Hz], beta, derivative_cutoff [Hz]。"""

  min_cutoff: float = 1.0
  beta: float = 0.5
  derivative_cutoff: float = 1.0


def _compute_alpha(dt: float, cutoff: float) -> float:
  if dt <= 0.0 or cutoff <= 0.0:
    return 1.0
  tau = 1.0 / (2.0 * _PI * cutoff)
  return 1.0 / (1.0 + tau / dt)


def normalize_quat_wxyz(q: np.ndarray) -> np.ndarray | None:
  """归一化四元数 wxyz；非法输入返回 None。"""
  q = np.asarray(q, dtype=np.float64).reshape(4)
  if not np.all(np.isfinite(q)):
    return None
  n = float(np.linalg.norm(q))
  if n < 1e-12:
    return None
  q = q / n
  if q[0] < 0.0:
    q = -q
  return q


def xyzw_to_wxyz(q_xyzw: np.ndarray) -> np.ndarray | None:
  qx, qy, qz, qw = np.asarray(q_xyzw, dtype=np.float64).reshape(4)
  return normalize_quat_wxyz(np.array([qw, qx, qy, qz], dtype=np.float64))


def wxyz_to_xyzw(q_wxyz: np.ndarray) -> np.ndarray:
  w, x, y, z = np.asarray(q_wxyz, dtype=np.float64).reshape(4)
  return np.array([x, y, z, w], dtype=np.float64)


def pose_xyzw_is_valid(pose7: np.ndarray) -> bool:
  """SDK 位姿 [x,y,z,qx,qy,qz,qw] 是否可用于滤波/发布。"""
  p = np.asarray(pose7, dtype=np.float64).reshape(7)
  if not np.all(np.isfinite(p)):
    return False
  return float(np.linalg.norm(p[3:7])) >= 1e-6


def _quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
  aw, ax, ay, az = a
  bw, bx, by, bz = b
  return np.array(
      [
          aw * bw - ax * bx - ay * by - az * bz,
          aw * bx + ax * bw + ay * bz - az * by,
          aw * by - ax * bz + ay * bw + az * bx,
          aw * bz + ax * by - ay * bx + az * bw,
      ],
      dtype=np.float64,
  )


def _quat_conj_wxyz(q: np.ndarray) -> np.ndarray:
  w, x, y, z = q
  return np.array([w, -x, -y, -z], dtype=np.float64)


def _quat_ln_wxyz(q: np.ndarray) -> np.ndarray:
  """对齐 OSVR util::quat_ln：单位四元数 -> 旋转向量。"""
  qn = normalize_quat_wxyz(q)
  if qn is None:
    return np.zeros(3, dtype=np.float64)
  w = float(np.clip(qn[0], -1.0, 1.0))
  v = qn[1:4]
  angle = 2.0 * math.acos(w)
  s = math.sqrt(max(0.0, 1.0 - w * w))
  if s < 1e-9:
    return np.zeros(3, dtype=np.float64)
  return (angle / s) * v


def _quat_slerp_wxyz(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
  """对齐 Eigen Quaternion::slerp(t, other)：从 q0 插向 q1。"""
  a = normalize_quat_wxyz(q0)
  b = normalize_quat_wxyz(q1)
  if a is None:
    return b if b is not None else _IDENTITY_WXYZ.copy()
  if b is None:
    return a
  t = float(np.clip(t, 0.0, 1.0))
  dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
  if dot < 0.0:
    b = -b
    dot = -dot
  if dot > 0.9995:
    out = a + t * (b - a)
    out_n = normalize_quat_wxyz(out)
    return out_n if out_n is not None else a
  theta = math.acos(dot)
  sin_theta = math.sin(theta)
  if sin_theta < 1e-9:
    return a
  w0 = math.sin((1.0 - t) * theta) / sin_theta
  w1 = math.sin(t * theta) / sin_theta
  blended = normalize_quat_wxyz(w0 * a + w1 * b)
  return blended if blended is not None else a


class LowPassFilterVec3:
  """向量低通（线性）。"""

  def __init__(self) -> None:
    self._hat: np.ndarray | None = None
    self._first = True

  def filter(self, x: np.ndarray, alpha: float) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64)
    if self._first:
      self._first = False
      self._hat = x.copy()
      return self._hat
    assert self._hat is not None
    if not math.isfinite(alpha):
      return self._hat
    self._hat = alpha * x + (1.0 - alpha) * self._hat
    return self._hat

  @property
  def state(self) -> np.ndarray | None:
    return self._hat


class LowPassFilterQuat:
  """四元数低通（SLERP，对齐 OSVR computeStep Quaternion）。"""

  def __init__(self) -> None:
    self._hat: np.ndarray | None = None
    self._first = True

  def filter(self, q_wxyz: np.ndarray, alpha: float) -> np.ndarray:
    q = normalize_quat_wxyz(q_wxyz)
    if q is None:
      if self._hat is not None:
        return self._hat
      return _IDENTITY_WXYZ.copy()
    if self._first or self._hat is None:
      self._first = False
      self._hat = q.copy()
      return self._hat
    if not math.isfinite(alpha):
      return self._hat
    out = _quat_slerp_wxyz(self._hat, q, alpha)
    self._hat = out
    return self._hat

  @property
  def state(self) -> np.ndarray | None:
    return self._hat


class OneEuroFilterVec3:
  """三维 One Euro Filter。"""

  def __init__(self, params: OneEuroParams) -> None:
    self._params = params
    self._x = LowPassFilterVec3()
    self._dx = LowPassFilterVec3()
    self._first = True

  def filter(self, dt: float, x: np.ndarray) -> np.ndarray:
    if dt <= 0.0:
      dt = 1.0
    x = np.asarray(x, dtype=np.float64)
    if self._first:
      self._first = False
      dx = np.zeros(3, dtype=np.float64)
    else:
      prev = self._x.state
      assert prev is not None
      dx = (x - prev) / dt
    self._dx.filter(dx, _compute_alpha(dt, self._params.derivative_cutoff))
    dx_hat = self._dx.state
    assert dx_hat is not None
    cutoff = self._params.min_cutoff + self._params.beta * float(np.linalg.norm(dx_hat))
    return self._x.filter(x, _compute_alpha(dt, cutoff))


class OneEuroFilterQuat:
  """四元数 One Euro Filter（wxyz，SLERP 低通）。"""

  def __init__(self, params: OneEuroParams) -> None:
    self._params = params
    self._x = LowPassFilterQuat()
    self._dx = LowPassFilterVec3()
    self._first = True

  def filter(self, dt: float, q_wxyz: np.ndarray) -> np.ndarray:
    if dt <= 0.0:
      dt = 1.0
    q = normalize_quat_wxyz(q_wxyz)
    if q is None:
      state = self._x.state
      return state if state is not None else _IDENTITY_WXYZ.copy()
    if self._first:
      self._first = False
      dx = np.zeros(3, dtype=np.float64)
    else:
      prev = self._x.state
      assert prev is not None
      dq = _quat_mul_wxyz(q, _quat_conj_wxyz(prev))
      dx = _quat_ln_wxyz(dq) / dt
    self._dx.filter(dx, _compute_alpha(dt, self._params.derivative_cutoff))
    dx_hat = self._dx.state
    assert dx_hat is not None
    cutoff = self._params.min_cutoff + self._params.beta * float(np.linalg.norm(dx_hat))
    return self._x.filter(q, _compute_alpha(dt, cutoff))

  def get_state_xyzw(self) -> np.ndarray:
    state = self._x.state
    if state is None:
      return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return wxyz_to_xyzw(state)


class PoseOneEuroFilter:
  """位姿滤波：位置 + 四元数。"""

  def __init__(
      self,
      pos_params: OneEuroParams | None = None,
      ori_params: OneEuroParams | None = None,
  ) -> None:
    self._pos = OneEuroFilterVec3(pos_params or OneEuroParams())
    self._ori = OneEuroFilterQuat(ori_params or OneEuroParams())

  def filter_pose_xyzw(
      self, dt: float, pos: np.ndarray, quat_xyzw: np.ndarray
  ) -> Tuple[np.ndarray, np.ndarray]:
    """输入 SDK 位姿 [x,y,z] + [qx,qy,qz,qw]，返回滤波后同格式。"""
    if dt <= 0.0:
      dt = 1.0
    pos_f = self._pos.filter(dt, pos)
    q_wxyz = xyzw_to_wxyz(quat_xyzw)
    if q_wxyz is None:
      return pos_f, self._ori.get_state_xyzw()
    q_f = self._ori.filter(dt, q_wxyz)
    return pos_f, wxyz_to_xyzw(q_f)
