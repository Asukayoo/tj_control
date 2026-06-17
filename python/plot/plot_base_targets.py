#!/usr/bin/env python3
"""可视化 Pico 映射后的 Link_Base 笛卡尔目标位姿（left/right）。"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TELEOP = ROOT / "data/test_teleop_data"
DEFAULT_OUT = ROOT / "data/test_ServoPByPico"
CONTROL_DT = 0.001
STREAM_SERVO_CYCLES = 40  # 与 in_data.hpp kStreamServoCycles 一致（内部样条 40ms）
# Pico teleop 录制/回放频率，与 test_SetvoPByPico kPicoDataHz 一致
PICO_PLAYBACK_HZ = 90.0
PICO_PLAYBACK_DT = 1.0 / PICO_PLAYBACK_HZ
# 无 run_meta 时的默认对齐频率
DEFAULT_SERVO_HZ = PICO_PLAYBACK_HZ
SERVO_HZ = 1.0 / (STREAM_SERVO_CYCLES * CONTROL_DT)  # 内部 25Hz
M2MM = 1000.0


def detect_servo_hz(t: np.ndarray, default: float = DEFAULT_SERVO_HZ) -> float:
    """从 targets.csv 时间列推断外部 Servo 调用频率。"""
    if len(t) < 2:
        return default
    dt = float(np.median(np.diff(t)))
    if dt <= 0.0:
        return default
    return 1.0 / dt


def load_run_meta(out_dir: Path) -> dict[str, str]:
    """读取 C++ 测试写出的 run_meta.txt（servo_hz 与 C++ kServoHz 一致）。"""
    meta: dict[str, str] = {}
    path = out_dir / "run_meta.txt"
    if not path.is_file():
        return meta
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        key, val = line.split("=", 1)
        meta[key.strip()] = val.strip()
    return meta


def resolve_servo_hz(out_dir: Path, t: np.ndarray | None = None) -> float:
    """优先 run_meta.txt，其次 targets 时间列，最后默认 90Hz（Pico）。"""
    meta = load_run_meta(out_dir)
    if "servo_hz" in meta:
        return float(meta["servo_hz"])
    if t is not None and len(t) >= 2:
        return detect_servo_hz(t)
    return DEFAULT_SERVO_HZ


def normalize_quat(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / np.maximum(n, 1e-12)


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1[..., 0], q1[..., 1], q1[..., 2], q1[..., 3]
    w2, x2, y2, z2 = q2[..., 0], q2[..., 1], q2[..., 2], q2[..., 3]
    out = np.stack(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        axis=-1,
    )
    return normalize_quat(out)


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    qv = np.stack([x, y, z], axis=-1)
    t = 2.0 * np.cross(qv, v)
    return v + w[..., None] * t + np.cross(qv, t)


def pose_compose(
    p_pos: np.ndarray, p_q: np.ndarray, c_pos: np.ndarray, c_q: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    out_q = quat_mul(p_q, c_q)
    out_pos = p_pos + quat_rotate(p_q, c_pos)
    return out_pos, normalize_quat(out_q)


def pose_inverse(pos: np.ndarray, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    inv_q = normalize_quat(np.array([q[0], -q[1], -q[2], -q[3]]))
    return -quat_rotate(inv_q, pos), inv_q


def pico_to_abs_target(
    ref_pico_pos: np.ndarray,
    ref_pico_q: np.ndarray,
    pico_pos: np.ndarray,
    pico_q: np.ndarray,
    anchor_pos: np.ndarray,
    anchor_q: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """与 motion.cpp PicoToAbsTarget 一致。"""
    inv_pos, inv_q = pose_inverse(ref_pico_pos, ref_pico_q)
    d_pos, d_q = pose_compose(inv_pos, inv_q, pico_pos, pico_q)
    return pose_compose(anchor_pos, anchor_q, d_pos, d_q)


def load_pico_txt(path: Path) -> tuple[np.ndarray, np.ndarray]:
    arr = np.loadtxt(path)
    return arr[:, :3] * M2MM, normalize_quat(arr[:, 3:7])


def load_targets_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t"]) for r in rows])
    pos = np.array([[float(r[k]) for k in ("px", "py", "pz")] for r in rows])
    quat = normalize_quat(
        np.array([[float(r[k]) for k in ("qw", "qx", "qy", "qz")] for r in rows])
    )
    return t, pos, quat


def load_anchor_from_cart(cart_csv: Path) -> tuple[np.ndarray, np.ndarray]:
    with cart_csv.open(newline="") as f:
        r0 = next(csv.DictReader(f))
    pos = np.array([float(r0["px"]), float(r0["py"]), float(r0["pz"])])
    quat = normalize_quat(
        np.array([float(r0["qw"]), float(r0["qx"]), float(r0["qy"]), float(r0["qz"])])
    )
    return pos, quat


def map_pico_to_base(
    pico_pos: np.ndarray,
    pico_quat: np.ndarray,
    anchor_pos: np.ndarray,
    anchor_quat: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    ref_pos, ref_q = pico_pos[0], pico_quat[0]
    n = len(pico_pos)
    pos = np.zeros((n, 3))
    quat = np.zeros((n, 4))
    for i in range(n):
        pos[i], quat[i] = pico_to_abs_target(
            ref_pos, ref_q, pico_pos[i], pico_quat[i], anchor_pos, anchor_quat
        )
    return pos, normalize_quat(quat)


def quat_z_axis(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    return np.column_stack(
        [
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ]
    )


def load_arm_targets(
    arm: str,
    teleop_dir: Path,
    out_dir: Path,
) -> dict[str, np.ndarray]:
    targets_csv = out_dir / f"{arm}_targets.csv"
    if targets_csv.is_file():
        t, pos, quat = load_targets_csv(targets_csv)
        hz = resolve_servo_hz(out_dir, t)
        # 与 C++ 一致：按帧序号 / servo_hz 作为相对时间
        t = np.arange(len(pos), dtype=float) / hz
    else:
        pico_pos, pico_quat = load_pico_txt(teleop_dir / f"{arm}.txt")
        cart_csv = out_dir / f"{arm}_cart.csv"
        if not cart_csv.is_file():
            raise FileNotFoundError(f"缺少 {targets_csv} 或 {cart_csv}")
        anchor_pos, anchor_quat = load_anchor_from_cart(cart_csv)
        pos, quat = map_pico_to_base(pico_pos, pico_quat, anchor_pos, anchor_quat)
        hz = resolve_servo_hz(out_dir)
        t = np.arange(len(pos), dtype=float) / hz
    return {"t": t, "pos": pos, "quat": quat, "arm": arm, "servo_hz": hz}


def plot_position_xyz_both(left: dict, right: dict, save: Path | None) -> plt.Figure:
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.suptitle(f"Link_Base Cartesian Targets @ {left.get('servo_hz', DEFAULT_SERVO_HZ):.0f} Hz")
    names = ("X", "Y", "Z")
    for ax, k, name in zip(axes, range(3), names):
        ax.plot(left["t"], left["pos"][:, k], "C0-", lw=1.0, label="left")
        ax.plot(right["t"], right["pos"][:, k], "C1-", lw=1.0, label="right")
        ax.set_ylabel(f"{name} [mm]")
        ax.grid(True, alpha=0.3)
        if k == 0:
            ax.legend(loc="best")
    axes[-1].set_xlabel("t [s]")
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=150)
    return fig


def plot_position_3d_both(left: dict, right: dict, save: Path | None) -> plt.Figure:
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(
        left["pos"][:, 0], left["pos"][:, 1], left["pos"][:, 2], "C0-", lw=1.0, label="left"
    )
    ax.plot(
        right["pos"][:, 0],
        right["pos"][:, 1],
        right["pos"][:, 2],
        "C1-",
        lw=1.0,
        label="right",
    )
    for data, color, name in ((left, "C0", "left"), (right, "C1", "right")):
        ax.scatter(
            data["pos"][0, 0], data["pos"][0, 1], data["pos"][0, 2],
            color=color, s=40, marker="o",
        )
        ax.scatter(
            data["pos"][-1, 0], data["pos"][-1, 1], data["pos"][-1, 2],
            color=color, s=40, marker="^",
        )
    ax.set_xlabel("px [mm]")
    ax.set_ylabel("py [mm]")
    ax.set_zlabel("pz [mm]")
    ax.set_title("Link_Base Target Trajectory 3D (o=start, ^=end)")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=150)
    return fig


def plot_orientation_z(left: dict, right: dict, save: Path | None) -> plt.Figure:
    fig, axes = plt.subplots(1, 2, figsize=(11, 5), subplot_kw={"projection": "3d"})
    fig.suptitle("目标姿态：TCP Z 轴方向 (Link_Base)")
    for ax, data, title in zip(axes, (left, right), ("Left", "Right")):
        z = quat_z_axis(data["quat"])
        ax.plot(z[:, 0], z[:, 1], z[:, 2], lw=1.0)
        ax.scatter(z[0, 0], z[0, 1], z[0, 2], color="green", s=30)
        ax.scatter(z[-1, 0], z[-1, 1], z[-1, 2], color="red", s=30)
        lim = 1.05
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(-lim, lim)
        ax.set_box_aspect((1, 1, 1))
        ax.set_title(title)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=150)
    return fig


def print_stats(left: dict, right: dict) -> None:
    for data in (left, right):
        arm = data["arm"]
        p = data["pos"]
        anchor = p[0]
        span = p.max(axis=0) - p.min(axis=0)
        delta = np.linalg.norm(p - anchor, axis=1)
        print(
            f"{arm}: frames={len(p)}  anchor=[{anchor[0]:.1f},{anchor[1]:.1f},{anchor[2]:.1f}] mm  "
            f"max|delta|={delta.max():.1f} mm  span=[{span[0]:.1f},{span[1]:.1f},{span[2]:.1f}] mm"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="可视化 Pico→Link_Base 映射后的左右臂笛卡尔目标"
    )
    parser.add_argument("--teleop-dir", type=Path, default=DEFAULT_TELEOP)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument(
        "--save-dir",
        type=Path,
        default=ROOT / "data/test_ServoPByPico/plots",
    )
    parser.add_argument("--show", action="store_true", help="弹窗显示")
    args = parser.parse_args()

    left = load_arm_targets("left", args.teleop_dir, args.data_dir)
    right = load_arm_targets("right", args.teleop_dir, args.data_dir)
    hz = left.get("servo_hz", DEFAULT_SERVO_HZ)
    print_stats(left, right)
    print(f"servo_hz={hz:.1f} (Pico playback, aligned with test_SetvoPByPico)")

    args.save_dir.mkdir(parents=True, exist_ok=True)
    plot_position_xyz_both(left, right, args.save_dir / "base_targets_xyz.png")
    plot_position_3d_both(left, right, args.save_dir / "base_targets_3d.png")
    plot_orientation_z(left, right, args.save_dir / "base_targets_orient.png")
    print(f"已保存: {args.save_dir}/base_targets_{{xyz,3d,orient}}.png")

    if args.show:
        plt.show()
    else:
        plt.close("all")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
