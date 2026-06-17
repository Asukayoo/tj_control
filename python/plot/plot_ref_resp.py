#!/usr/bin/env python3
"""test_enable / test_movj 等 ref/resp CSV 可视化（8 窗口：左右各 4）。"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import ScalarFormatter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

_PLOT_DIR = Path(__file__).resolve().parent
if str(_PLOT_DIR) not in sys.path:
    sys.path.insert(0, str(_PLOT_DIR))

from plot_base_targets import CONTROL_DT, normalize_quat  # noqa: E402

DOF = 7
R2D = 180.0 / np.pi
PATH_RE = re.compile(r"(?:^|\s)(/[^\s]+)")


def disable_scientific_notation(ax: plt.Axes) -> None:
    """坐标轴刻度使用普通小数，禁用科学计数法。"""
    axes = [ax.xaxis, ax.yaxis]
    if hasattr(ax, "zaxis"):
        axes.append(ax.zaxis)
    for axis in axes:
        fmt = ScalarFormatter(useOffset=False)
        fmt.set_scientific(False)
        axis.set_major_formatter(fmt)


def parse_data_dir(text: str) -> Path:
    """从纯路径或终端粘贴文本中提取数据目录。"""
    text = text.strip()
    if not text:
        raise ValueError("未提供数据目录")
    direct = Path(text.splitlines()[0].strip())
    if direct.is_dir():
        return direct.resolve()
    for match in PATH_RE.finditer(text):
        candidate = Path(match.group(1))
        if candidate.is_dir():
            return candidate.resolve()
    raise FileNotFoundError(f"未找到有效目录: {text!r}")


def infer_dt(data_dir: Path, cycle: np.ndarray) -> float:
    """优先 timing.csv 中位周期，否则 1 kHz。"""
    timing = data_dir / "timing.csv"
    if timing.is_file():
        data = np.loadtxt(timing, delimiter=",", skiprows=1)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] >= 3 and len(data) >= 2:
            periods = data[:, 2]
            periods = periods[periods > 0]
            if len(periods) > 0:
                return float(np.median(periods)) * 1e-6
    if len(cycle) >= 2:
        dc = np.diff(cycle.astype(float))
        dc = dc[dc > 0]
        if len(dc) > 0:
            return float(np.median(dc)) * CONTROL_DT
    return CONTROL_DT


def load_joint_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data[:, 0], data[:, 1 : 1 + DOF]


def load_cart_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    cycle = data[:, 0]
    pos = data[:, 1:4]
    quat = normalize_quat(data[:, 4:8])
    return cycle, pos, quat


def diff_velocity(values: np.ndarray, dt: float) -> np.ndarray:
    """中心差分速度，首尾用前向/后向差分。"""
    n = len(values)
    vel = np.zeros_like(values)
    if n < 2:
        return vel
    vel[1:-1] = (values[2:] - values[:-2]) / (2.0 * dt)
    vel[0] = (values[1] - values[0]) / dt
    vel[-1] = (values[-1] - values[-2]) / dt
    return vel


def quat_conj(q: np.ndarray) -> np.ndarray:
    out = q.copy()
    out[..., 1:] *= -1.0
    return out


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


def unify_quat_hemisphere(quat: np.ndarray) -> np.ndarray:
    """相邻帧点积为负时取反，消除 q/-q 等价导致的差分尖峰。"""
    q = normalize_quat(quat.copy())
    for i in range(1, len(q)):
        if np.dot(q[i - 1], q[i]) < 0.0:
            q[i] *= -1.0
    return q


def quat_z_axis(quat: np.ndarray) -> np.ndarray:
    """R(q) * e_z，已在单位球上。"""
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    return np.column_stack(
        [
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ]
    )


def angular_speed(quat: np.ndarray, dt: float) -> np.ndarray:
    """四元数差分得到标量角速度 |ω| [rad/s]。"""
    qn = unify_quat_hemisphere(quat)
    n = len(qn)
    omega = np.zeros(n)
    if n < 2:
        return omega
    dq = quat_mul(qn[1:], quat_conj(qn[:-1]))
    w = np.clip(dq[:, 0], -1.0, 1.0)
    angle = 2.0 * np.arccos(w)
    omega[1:] = angle / dt
    omega[0] = omega[1]
    return omega


def draw_unit_sphere(ax: plt.Axes) -> None:
    u = np.linspace(0.0, 2.0 * np.pi, 36)
    v = np.linspace(0.0, np.pi, 18)
    xs = np.outer(np.cos(u), np.sin(v))
    ys = np.outer(np.sin(u), np.sin(v))
    zs = np.outer(np.ones_like(u), np.cos(v))
    ax.plot_wireframe(xs, ys, zs, color="0.85", linewidth=0.4, alpha=0.6)
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_zlim(-1.0, 1.0)
    ax.set_box_aspect((1, 1, 1))


def plot_ref_resp_pair(
    ax: plt.Axes,
    t: np.ndarray,
    ref: np.ndarray,
    resp: np.ndarray,
    ylabel: str,
    title: str,
    ref_scale: float = 1.0,
) -> None:
    ax.plot(t, ref * ref_scale, "C0-", lw=0.9, label="ref")
    ax.plot(t, resp * ref_scale, "C1--", lw=0.9, label="resp")
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=7)
    disable_scientific_notation(ax)


def load_arm_data(data_dir: Path, arm: str) -> dict[str, np.ndarray]:
    prefix = arm.lower()
    j_ref_c, j_ref_q = load_joint_csv(data_dir / f"{prefix}_ref_joint.csv")
    j_resp_c, j_resp_q = load_joint_csv(data_dir / f"{prefix}_resp_joint.csv")
    c_ref_c, c_ref_p, c_ref_q = load_cart_csv(data_dir / f"{prefix}_ref_cart.csv")
    c_resp_c, c_resp_p, c_resp_q = load_cart_csv(data_dir / f"{prefix}_resp_cart.csv")

    n = min(len(j_ref_c), len(j_resp_c), len(c_ref_c), len(c_resp_c))
    cycle = j_ref_c[:n]
    dt = infer_dt(data_dir, cycle)
    t = cycle.astype(float) * dt

    return {
        "t": t,
        "dt": dt,
        "j_ref_q": j_ref_q[:n],
        "j_resp_q": j_resp_q[:n],
        "c_ref_p": c_ref_p[:n],
        "c_resp_p": c_resp_p[:n],
        "c_ref_q": c_ref_q[:n],
        "c_resp_q": c_resp_q[:n],
    }


def plot_arm_windows(arm: str, data: dict[str, np.ndarray]) -> list[plt.Figure]:
    """单臂 4 个窗口：关节位置/速度、笛卡尔 xyz、姿态球+角速度。"""
    t = data["t"]
    dt = data["dt"]
    figures: list[plt.Figure] = []

    # 窗口 1：关节位置
    fig1, axes1 = plt.subplots(DOF, 1, figsize=(10, 12), sharex=True)
    fig1.suptitle(f"{arm.upper()} joint position ref vs resp", fontsize=11)
    for i in range(DOF):
        plot_ref_resp_pair(
            axes1[i],
            t,
            data["j_ref_q"][:, i],
            data["j_resp_q"][:, i],
            "q [deg]",
            f"J{i}",
            R2D,
        )
    axes1[-1].set_xlabel("t [s]")
    fig1.tight_layout()
    figures.append(fig1)

    # 窗口 2：关节速度（位置差分）
    j_ref_v = diff_velocity(data["j_ref_q"], dt)
    j_resp_v = diff_velocity(data["j_resp_q"], dt)
    fig2, axes2 = plt.subplots(DOF, 1, figsize=(10, 12), sharex=True)
    fig2.suptitle(f"{arm.upper()} joint velocity (diff) ref vs resp", fontsize=11)
    for i in range(DOF):
        plot_ref_resp_pair(
            axes2[i],
            t,
            j_ref_v[:, i],
            j_resp_v[:, i],
            "dq/dt [deg/s]",
            f"J{i}",
            R2D,
        )
    axes2[-1].set_xlabel("t [s]")
    fig2.tight_layout()
    figures.append(fig2)

    # 窗口 3：笛卡尔 xyz 位置 + 线速度
    c_ref_v = diff_velocity(data["c_ref_p"], dt)
    c_resp_v = diff_velocity(data["c_resp_p"], dt)
    fig3, axes3 = plt.subplots(6, 1, figsize=(10, 11), sharex=True)
    fig3.suptitle(f"{arm.upper()} cartesian xyz ref vs resp", fontsize=11)
    labels = ("px", "py", "pz", "vx", "vy", "vz")
    units = ("[mm]", "[mm]", "[mm]", "[mm/s]", "[mm/s]", "[mm/s]")
    series = (
        (data["c_ref_p"], data["c_resp_p"]),
        (c_ref_v, c_resp_v),
    )
    idx = 0
    for block, block_name in zip(series, ("position", "velocity")):
        ref_arr, resp_arr = block
        for k in range(3):
            plot_ref_resp_pair(
                axes3[idx],
                t,
                ref_arr[:, k],
                resp_arr[:, k],
                f"{labels[idx]} {units[idx]}",
                f"{labels[idx]} ({block_name})",
            )
            idx += 1
    axes3[-1].set_xlabel("t [s]")
    fig3.tight_layout()
    figures.append(fig3)

    # 窗口 4：姿态 Z 轴单位球 + 角速度标量
    z_ref = quat_z_axis(data["c_ref_q"])
    z_resp = quat_z_axis(data["c_resp_q"])
    w_ref = angular_speed(data["c_ref_q"], dt)
    w_resp = angular_speed(data["c_resp_q"], dt)

    fig4 = plt.figure(figsize=(12, 5))
    fig4.suptitle(f"{arm.upper()} orientation (R·e_z on S²) & |ω|", fontsize=11)
    ax3d = fig4.add_subplot(1, 2, 1, projection="3d")
    draw_unit_sphere(ax3d)
    ax3d.plot(z_ref[:, 0], z_ref[:, 1], z_ref[:, 2], "C0-", lw=1.0, label="ref")
    ax3d.plot(z_resp[:, 0], z_resp[:, 1], z_resp[:, 2], "C1--", lw=1.0, label="resp")
    ax3d.scatter(*z_ref[0], color="C0", s=20, marker="o")
    ax3d.scatter(*z_resp[0], color="C1", s=20, marker="x")
    ax3d.set_xlabel("x")
    ax3d.set_ylabel("y")
    ax3d.set_zlabel("z")
    ax3d.legend(fontsize=8)
    disable_scientific_notation(ax3d)

    ax2d = fig4.add_subplot(1, 2, 2)
    ax2d.plot(t, w_ref * R2D, "C0-", lw=0.9, label="ref |ω|")
    ax2d.plot(t, w_resp * R2D, "C1--", lw=0.9, label="resp |ω|")
    ax2d.set_xlabel("t [s]")
    ax2d.set_ylabel("|ω| [deg/s]")
    ax2d.set_title("angular speed (quat diff)")
    ax2d.grid(True, alpha=0.3)
    ax2d.legend(fontsize=8)
    disable_scientific_notation(ax2d)
    fig4.tight_layout()
    figures.append(fig4)

    return figures


def main() -> int:
    parser = argparse.ArgumentParser(
        description="绘制 test_enable/test_movj 的 ref/resp 对比（8 窗口）"
    )
    parser.add_argument(
        "input",
        nargs="?",
        default="",
        help="数据目录，或粘贴含路径的终端输出",
    )
    parser.add_argument(
        "--save-dir",
        type=Path,
        default=None,
        help="保存 png 到目录；不指定则仅 --show 时弹窗",
    )
    parser.add_argument("--show", action="store_true", help="弹窗显示")
    args = parser.parse_args()

    if not args.input:
        parser.error("请提供数据目录路径")
    data_dir = parse_data_dir(args.input)
    print(f"data_dir: {data_dir}")

    required = [
        "left_ref_joint.csv",
        "left_resp_joint.csv",
        "left_ref_cart.csv",
        "left_resp_cart.csv",
        "right_ref_joint.csv",
        "right_resp_joint.csv",
        "right_ref_cart.csv",
        "right_resp_cart.csv",
    ]
    missing = [name for name in required if not (data_dir / name).is_file()]
    if missing:
        raise FileNotFoundError(f"缺少 CSV: {', '.join(missing)}")

    all_figs: list[plt.Figure] = []
    for arm in ("left", "right"):
        arm_data = load_arm_data(data_dir, arm)
        print(
            f"{arm}: n={len(arm_data['t'])}  dt={arm_data['dt']*1e3:.3f} ms  "
            f"duration={arm_data['t'][-1]:.3f} s"
        )
        all_figs.extend(plot_arm_windows(arm, arm_data))

    if args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)
        names = [
            "left_joint_q",
            "left_joint_v",
            "left_cart_xyz",
            "left_orient",
            "right_joint_q",
            "right_joint_v",
            "right_cart_xyz",
            "right_orient",
        ]
        for fig, name in zip(all_figs, names):
            fig.savefig(args.save_dir / f"{name}.png", dpi=150)
        print(f"saved {len(all_figs)} figures -> {args.save_dir}")

    if args.show:
        plt.show()
    else:
        plt.close("all")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
