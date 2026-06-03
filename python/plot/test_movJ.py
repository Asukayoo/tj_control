#!/usr/bin/env python3
"""绘制 test_movJ 采集的左右臂关节/笛卡尔轨迹数据。"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

DT = 0.001  # 控制周期 [s]
DOF = 7


def attach_legend_toggle(ax: plt.Axes, artists: list) -> None:
    """图例项可点击以隐藏/显示对应曲线/散点。"""
    leg = ax.legend(loc="best", fontsize=8)
    for leg_line, artist in zip(leg.get_lines(), artists):
        leg_line.set_picker(5)
        leg_line._artist = artist  # noqa: SLF001

    def on_pick(event):
        leg_line = event.artist
        if not hasattr(leg_line, "_artist"):
            return
        artist = leg_line._artist
        visible = not artist.get_visible()
        artist.set_visible(visible)
        leg_line.set_alpha(1.0 if visible else 0.2)
        event.canvas.draw_idle()

    ax.figure.canvas.mpl_connect("pick_event", on_pick)


def read_csv_dict(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"空文件: {path}")
    cols = rows[0].keys()
    data = {c: np.array([float(r[c]) for r in rows]) for c in cols}
    if "t" not in data:
        data["t"] = np.arange(len(rows)) * DT
    return data


def quat_to_rotmat(qw: np.ndarray, qx: np.ndarray,
                   qy: np.ndarray, qz: np.ndarray) -> np.ndarray:
    """Hamilton 四元数 (w,x,y,z) → 旋转矩阵，与 Eigen::Quaterniond 一致。"""
    norm = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    norm = np.maximum(norm, 1e-12)
    w, x, y, z = qw / norm, qx / norm, qy / norm, qz / norm
    return np.stack([
        np.stack([1.0 - 2.0 * (y * y + z * z),
                  2.0 * (x * y - w * z),
                  2.0 * (x * z + w * y)], axis=-1),
        np.stack([2.0 * (x * y + w * z),
                  1.0 - 2.0 * (x * x + z * z),
                  2.0 * (y * z - w * x)], axis=-1),
        np.stack([2.0 * (x * z - w * y),
                  2.0 * (y * z + w * x),
                  1.0 - 2.0 * (x * x + y * y)], axis=-1),
    ], axis=-2)


def quat_rotate_z(qw: np.ndarray, qx: np.ndarray,
                  qy: np.ndarray, qz: np.ndarray) -> np.ndarray:
    """显式计算 R(q) @ e_z，e_z = [0,0,1]。"""
    rot = quat_to_rotmat(qw, qx, qy, qz)
    ez = np.array([0.0, 0.0, 1.0])
    return rot @ ez


def draw_unit_sphere(ax: Axes3D, alpha: float = 0.12) -> None:
    u = np.linspace(0.0, 2.0 * np.pi, 40)
    v = np.linspace(0.0, np.pi, 20)
    xs = np.outer(np.cos(u), np.sin(v))
    ys = np.outer(np.sin(u), np.sin(v))
    zs = np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(xs, ys, zs, color="lightgray", alpha=alpha,
                    linewidth=0.0, antialiased=True, shade=False)


def plot_joint_window(arm: str, joint: dict[str, np.ndarray]) -> None:
    t = joint["t"]
    fig, (ax_q, ax_v) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig.canvas.manager.set_window_title(f"{arm} - Joint")  # type: ignore[union-attr]

    q_lines: list[Line2D] = []
    v_lines: list[Line2D] = []
    for i in range(DOF):
        q_lines.append(ax_q.plot(t, joint[f"q{i}"], label=f"q{i}")[0])
        v_lines.append(ax_v.plot(t, joint[f"v{i}"], label=f"v{i}")[0])

    ax_q.set_ylabel("position [rad]")
    ax_v.set_ylabel("velocity [rad/s]")
    ax_v.set_xlabel("time [s]")
    ax_q.set_title(f"{arm} joint space")
    ax_q.grid(True, alpha=0.3)
    ax_v.grid(True, alpha=0.3)
    attach_legend_toggle(ax_q, q_lines)
    attach_legend_toggle(ax_v, v_lines)
    fig.tight_layout()


def plot_cart_linear_window(arm: str, cart: dict[str, np.ndarray]) -> None:
    t = cart["t"]
    px, py, pz = cart["px"], cart["py"], cart["pz"]
    lin_speed = np.linalg.norm(np.column_stack([cart["vx"], cart["vy"], cart["vz"]]),
                               axis=1)

    fig = plt.figure(figsize=(11, 5))
    fig.canvas.manager.set_window_title(f"{arm} - Cartesian Linear")  # type: ignore[union-attr]

    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    traj = ax3.plot(px, py, pz, color="C0", label="trajectory")[0]
    start = ax3.scatter(px[0], py[0], pz[0], color="green", s=30, label="start")
    end = ax3.scatter(px[-1], py[-1], pz[-1], color="red", s=30, label="end")
    ax3.set_xlabel("x [mm]")
    ax3.set_ylabel("y [mm]")
    ax3.set_zlabel("z [mm]")
    ax3.set_title("Cartesian position")
    attach_legend_toggle(ax3, [traj, start, end])

    ax2 = fig.add_subplot(1, 2, 2)
    spd = ax2.plot(t, lin_speed, color="C1", label="|v|")[0]
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("linear speed [mm/s]")
    ax2.set_title("Linear velocity magnitude")
    ax2.grid(True, alpha=0.3)
    attach_legend_toggle(ax2, [spd])
    fig.tight_layout()


def plot_cart_angular_window(arm: str, cart: dict[str, np.ndarray]) -> None:
    t = cart["t"]
    qw, qx, qy, qz = cart["qw"], cart["qx"], cart["qy"], cart["qz"]
    z_axis = quat_rotate_z(qw, qx, qy, qz)
    ang_speed = np.linalg.norm(np.column_stack([cart["wx"], cart["wy"], cart["wz"]]),
                               axis=1)

    fig = plt.figure(figsize=(11, 5))
    fig.canvas.manager.set_window_title(f"{arm} - Orientation")  # type: ignore[union-attr]

    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    draw_unit_sphere(ax3)
    ori = ax3.plot(z_axis[:, 0], z_axis[:, 1], z_axis[:, 2],
                   color="C2", linewidth=2, label="R·e_z")[0]
    start = ax3.scatter(z_axis[0, 0], z_axis[0, 1], z_axis[0, 2],
                        color="green", s=40, label="start")
    end = ax3.scatter(z_axis[-1, 0], z_axis[-1, 1], z_axis[-1, 2],
                      color="red", s=40, label="end")
    lim = 1.05
    ax3.set_xlim(-lim, lim)
    ax3.set_ylim(-lim, lim)
    ax3.set_zlim(-lim, lim)
    ax3.set_box_aspect((1, 1, 1))
    ax3.set_xlabel("x")
    ax3.set_ylabel("y")
    ax3.set_zlabel("z")
    ax3.set_title("Orientation (R·e_z on unit sphere)")
    attach_legend_toggle(ax3, [ori, start, end])

    ax2 = fig.add_subplot(1, 2, 2)
    w_line = ax2.plot(t, ang_speed, color="C3", label="|ω|")[0]
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("angular speed [rad/s]")
    ax2.set_title("Angular velocity magnitude")
    ax2.grid(True, alpha=0.3)
    attach_legend_toggle(ax2, [w_line])
    fig.tight_layout()


def plot_arm(arm: str, data_dir: Path) -> None:
    joint = read_csv_dict(data_dir / f"{arm}_joint.csv")
    cart = read_csv_dict(data_dir / f"{arm}_cart.csv")
    name = arm.capitalize()
    plot_joint_window(name, joint)
    plot_cart_linear_window(name, cart)
    plot_cart_angular_window(name, cart)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot test_movJ recorded data")
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path("/home/yxc/tj_control/data/test_movJ"),
        help="CSV 数据目录",
    )
    args = parser.parse_args()
    data_dir: Path = args.data_dir
    if not data_dir.is_dir():
        raise SystemExit(f"数据目录不存在: {data_dir}")

    for arm in ("left", "right"):
        if not (data_dir / f"{arm}_joint.csv").exists():
            raise SystemExit(f"缺少 {arm}_joint.csv")
        if not (data_dir / f"{arm}_cart.csv").exists():
            raise SystemExit(f"缺少 {arm}_cart.csv")

    for arm in ("left", "right"):
        plot_arm(arm, data_dir)

    print("已打开 6 个窗口（Left/Right 各 3 个），点击图例可隐藏/显示曲线。")
    plt.show()


if __name__ == "__main__":
    main()
