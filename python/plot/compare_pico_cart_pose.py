#!/usr/bin/env python3
"""目标位姿 vs 实际 cart 对比；按外部 Servo 周期（默认 50Hz）对齐时间轴。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

_PLOT_DIR = Path(__file__).resolve().parent
if str(_PLOT_DIR) not in sys.path:
    sys.path.insert(0, str(_PLOT_DIR))

from plot_base_targets import (  # noqa: E402
    CONTROL_DT,
    DEFAULT_SERVO_HZ,
    detect_servo_hz,
    load_anchor_from_cart,
    load_pico_txt,
    load_targets_csv,
    map_pico_to_base,
    normalize_quat,
    resolve_servo_hz,
)

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TELEOP = ROOT / "data/test_teleop_data"
DEFAULT_OUT = ROOT / "data/test_ServoPByPico"
DOF = 7
R2D = 180.0 / np.pi


def load_joint_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    t = data[:, 0]
    q = data[:, 1 : 1 + DOF]
    v = data[:, 1 + DOF : 1 + 2 * DOF]
    return t, q, v


def slice_joint_window(
    t: np.ndarray,
    q: np.ndarray,
    v: np.ndarray,
    t_start: float,
    t_end: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    mask = (t >= t_start) & (t <= t_end)
    if not np.any(mask):
        return np.array([]), np.empty((0, DOF)), np.empty((0, DOF))
    return t[mask] - t_start, q[mask], v[mask]


def plot_joint_qv(
    arm: str,
    t: np.ndarray,
    q: np.ndarray,
    v: np.ndarray,
    axes_q: np.ndarray,
    axes_v: np.ndarray,
) -> None:
    for i in range(DOF):
        ax_q = axes_q[i]
        ax_v = axes_v[i]
        ax_q.plot(t, q[:, i] * R2D, "C0-", lw=0.9)
        ax_v.plot(t, v[:, i] * R2D, "C1-", lw=0.9)
        ax_q.set_ylabel(f"J{i} [deg]")
        ax_v.set_ylabel(f"J{i} [deg/s]")
        ax_q.grid(True, alpha=0.3)
        ax_v.grid(True, alpha=0.3)
        if i == 0:
            ax_q.set_title(f"{arm.upper()} joint q @ 1kHz")
            ax_v.set_title(f"{arm.upper()} joint v @ 1kHz")
        if i == DOF - 1:
            ax_q.set_xlabel("t [s] (from Servo start)")
            ax_v.set_xlabel("t [s] (from Servo start)")
        vmax = float(np.max(np.abs(v[:, i]))) * R2D
        ax_v.text(
            0.02,
            0.95,
            f"max|v|={vmax:.1f} deg/s",
            transform=ax_v.transAxes,
            fontsize=7,
            va="top",
        )


def joint_vel_stats(v: np.ndarray) -> tuple[np.ndarray, float]:
    peak = np.max(np.abs(v), axis=0) * R2D
    return peak, float(peak.max())


def load_cart_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    t = data[:, 0]
    pos = data[:, 1:4]
    quat = normalize_quat(data[:, 4:8])
    return t, pos, quat


def load_base_targets(
    arm: str, teleop_dir: Path, out_dir: Path
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    targets_csv = out_dir / f"{arm}_targets.csv"
    if targets_csv.is_file():
        t, pos, quat = load_targets_csv(targets_csv)
        hz = resolve_servo_hz(out_dir, t)
        t = np.arange(len(pos), dtype=float) / hz
        return t, pos, quat, hz

    pico_pos, pico_quat = load_pico_txt(teleop_dir / f"{arm}.txt")
    cart_csv = out_dir / f"{arm}_cart.csv"
    anchor_pos, anchor_quat = load_anchor_from_cart(cart_csv)
    pos, quat = map_pico_to_base(pico_pos, pico_quat, anchor_pos, anchor_quat)
    hz = resolve_servo_hz(out_dir)
    t = np.arange(len(pos), dtype=float) / hz
    return t, pos, quat, hz


def servo_start_time(cart_t: np.ndarray, joint_t: np.ndarray | None = None) -> float:
    """Servo 段起始绝对时间：cart 首样本与 joint 首样本的较早者。"""
    t0 = float(cart_t[0])
    if joint_t is not None and len(joint_t) > 0:
        t0 = min(t0, float(joint_t[0]))
    return t0


def resample_cart_to_servo(
    cart_t: np.ndarray,
    cart_pos: np.ndarray,
    cart_quat: np.ndarray,
    rel_t: np.ndarray,
    t_servo0: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """将 1kHz cart 重采样到外部 Servo 相对时间轴 rel_t（秒，从 0 起）。"""
    t_abs = t_servo0 + rel_t
    pos = np.column_stack(
        [np.interp(t_abs, cart_t, cart_pos[:, k]) for k in range(3)]
    )
    idx = np.searchsorted(cart_t, t_abs, side="left")
    idx = np.clip(idx, 0, len(cart_t) - 1)
    prev = np.maximum(idx - 1, 0)
    pick = np.where(
        np.abs(cart_t[idx] - t_abs) < np.abs(cart_t[prev] - t_abs), idx, prev
    )
    quat = normalize_quat(cart_quat[pick])
    return rel_t, pos, quat


def trim_to_common_length(*arrays: np.ndarray) -> tuple[np.ndarray, ...]:
    n = min(len(a) for a in arrays)
    return tuple(a[:n] for a in arrays)


def plot_xyz_compare(
    arm: str,
    t: np.ndarray,
    target_pos: np.ndarray,
    cart_pos: np.ndarray,
    axes: np.ndarray,
    servo_hz: float,
) -> None:
    labels = ("X", "Y", "Z")
    for k, (ax, name) in enumerate(zip(axes, labels)):
        ax.plot(t, target_pos[:, k], "C0-", lw=1.2, label="target")
        ax.plot(t, cart_pos[:, k], "C1--", lw=1.0, label="cart FK")
        err = np.abs(target_pos[:, k] - cart_pos[:, k])
        ax.set_ylabel(f"{name} [mm]")
        ax.grid(True, alpha=0.3)
        if k == 0:
            ax.set_title(f"{arm.upper()} @ {servo_hz:.0f} Hz")
            ax.legend(loc="best", fontsize=8)
        if k == 2:
            ax.set_xlabel("t [s] (Servo period aligned)")
        ax.text(
            0.02,
            0.95,
            f"max|d{name}|={err.max():.2f} mm",
            transform=ax.transAxes,
            fontsize=8,
            va="top",
        )


def plot_3d_compare(
    left_tgt: np.ndarray,
    left_cart: np.ndarray,
    right_tgt: np.ndarray,
    right_cart: np.ndarray,
) -> plt.Figure:
    fig = plt.figure(figsize=(10, 5))
    for i, (tgt, cart, title) in enumerate(
        (
            (left_tgt, left_cart, "Left"),
            (right_tgt, right_cart, "Right"),
        )
    ):
        ax = fig.add_subplot(1, 2, i + 1, projection="3d")
        ax.plot(*tgt.T, "C0-", lw=1.0, label="target")
        ax.plot(*cart.T, "C1--", lw=1.0, label="cart FK")
        ax.scatter(*tgt[0], color="C0", s=30, marker="o")
        ax.scatter(*cart[0], color="C1", s=30, marker="x")
        ax.set_title(title)
        ax.set_xlabel("px [mm]")
        ax.set_ylabel("py [mm]")
        ax.set_zlabel("pz [mm]")
        ax.legend(fontsize=8)
    fig.suptitle("Link_Base trajectory 3D")
    fig.tight_layout()
    return fig


def pos_error_stats(target: np.ndarray, cart: np.ndarray) -> dict[str, float]:
    d = np.linalg.norm(target - cart, axis=1)
    return {
        "max_mm": float(d.max()),
        "mean_mm": float(d.mean()),
        "rms_mm": float(np.sqrt((d**2).mean())),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="目标 vs cart 对比（Pico 90Hz 对齐，cart 重采样到同周期）"
    )
    parser.add_argument("--teleop", type=Path, default=DEFAULT_TELEOP)
    parser.add_argument("--data", type=Path, default=DEFAULT_OUT)
    parser.add_argument(
        "--servo-hz",
        type=float,
        default=None,
        help="外部 Servo 调用频率；默认从 targets.csv 推断",
    )
    parser.add_argument(
        "--save-dir",
        type=Path,
        default=None,
        help="保存 png；不指定则仅 --show 时弹窗",
    )
    parser.add_argument("--show", action="store_true", help="弹窗显示")
    args = parser.parse_args()

    lt, l_tgt, _, hz_l = load_base_targets("left", args.teleop, args.data)
    rt, r_tgt, _, hz_r = load_base_targets("right", args.teleop, args.data)
    servo_hz = args.servo_hz or hz_l
    if abs(hz_l - hz_r) > 1e-3:
        print(f"warn: left/right servo_hz differ: {hz_l} vs {hz_r}, use {servo_hz}")
    servo_dt = 1.0 / servo_hz

    # 相对时间轴：帧序号 / servo_hz，与 C++ ServoPByPico 调用周期一致
    n = min(len(lt), len(rt))
    rel_t = np.arange(n, dtype=float) * servo_dt

    l_cart_t, l_cart_pos, l_cart_q = load_cart_csv(args.data / "left_cart.csv")
    r_cart_t, r_cart_pos, r_cart_q = load_cart_csv(args.data / "right_cart.csv")
    l_jt_raw, _, _ = load_joint_csv(args.data / "left_joint.csv")
    r_jt_raw, _, _ = load_joint_csv(args.data / "right_joint.csv")
    t0 = min(
        servo_start_time(l_cart_t, l_jt_raw),
        servo_start_time(r_cart_t, r_jt_raw),
    )

    _, l_cart_rs, _ = resample_cart_to_servo(
        l_cart_t, l_cart_pos, l_cart_q, rel_t, t0
    )
    _, r_cart_rs, _ = resample_cart_to_servo(
        r_cart_t, r_cart_pos, r_cart_q, rel_t, t0
    )

    t, l_tgt, l_cart_rs, r_tgt, r_cart_rs = trim_to_common_length(
        rel_t, l_tgt[:n], l_cart_rs, r_tgt[:n], r_cart_rs
    )

    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex="col")
    fig.suptitle(
        f"Target vs Cart FK @ {servo_hz:.0f} Hz  "
        f"(aligned, ctrl={1/CONTROL_DT:.0f}Hz cart resampled)"
    )
    plot_xyz_compare("left", t, l_tgt, l_cart_rs, axes[:, 0], servo_hz)
    plot_xyz_compare("right", t, r_tgt, r_cart_rs, axes[:, 1], servo_hz)
    fig.tight_layout()
    fig3d = plot_3d_compare(l_tgt, l_cart_rs, r_tgt, r_cart_rs)

    t_end_abs = t0 + t[-1]
    l_jt_raw, l_q_raw, l_v_raw = load_joint_csv(args.data / "left_joint.csv")
    r_jt_raw, r_q_raw, r_v_raw = load_joint_csv(args.data / "right_joint.csv")
    tj_l, l_q, l_v = slice_joint_window(l_jt_raw, l_q_raw, l_v_raw, t0, t_end_abs)
    tj_r, r_q, r_v = slice_joint_window(r_jt_raw, r_q_raw, r_v_raw, t0, t_end_abs)
    tj_l, l_q, l_v, tj_r, r_q, r_v = trim_to_common_length(
        tj_l, l_q, l_v, tj_r, r_q, r_v
    )

    fig_jq, axes_jq = plt.subplots(DOF, 2, figsize=(12, 14), sharex="col")
    fig_jv, axes_jv = plt.subplots(DOF, 2, figsize=(12, 14), sharex="col")
    fig_jq.suptitle("Joint position (ref, Servo window)")
    fig_jv.suptitle("Joint velocity (ref, Servo window)")
    plot_joint_qv("left", tj_l, l_q, l_v, axes_jq[:, 0], axes_jv[:, 0])
    plot_joint_qv("right", tj_r, r_q, r_v, axes_jq[:, 1], axes_jv[:, 1])
    fig_jq.tight_layout()
    fig_jv.tight_layout()

    ls = pos_error_stats(l_tgt, l_cart_rs)
    rs = pos_error_stats(r_tgt, r_cart_rs)
    l_peak, l_vmax = joint_vel_stats(l_v)
    r_peak, r_vmax = joint_vel_stats(r_v)
    print(
        f"servo_hz={servo_hz:.1f}  t0={t0:.3f}s  frames={len(t)}  "
        f"duration={t[-1]:.3f}s"
    )
    print(
        f"left  pos err: max={ls['max_mm']:.3f} mean={ls['mean_mm']:.3f} "
        f"rms={ls['rms_mm']:.3f} mm"
    )
    print(
        f"right pos err: max={rs['max_mm']:.3f} mean={rs['mean_mm']:.3f} "
        f"rms={rs['rms_mm']:.3f} mm"
    )
    print(
        f"left  joint v peak [deg/s]: {np.array2string(l_peak, precision=1)} "
        f"max={l_vmax:.1f}"
    )
    print(
        f"right joint v peak [deg/s]: {np.array2string(r_peak, precision=1)} "
        f"max={r_vmax:.1f}"
    )

    if args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save_dir / "compare_xyz.png", dpi=150)
        fig3d.savefig(args.save_dir / "compare_3d.png", dpi=150)
        fig_jq.savefig(args.save_dir / "compare_joint_q.png", dpi=150)
        fig_jv.savefig(args.save_dir / "compare_joint_v.png", dpi=150)
        print(f"saved plots -> {args.save_dir}")

    if args.show:
        plt.show()
    else:
        plt.close("all")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
