#!/usr/bin/env python3
"""离线绘制 pico_teleop.csv：raw vs One Euro 滤波对比，结束后落盘 CSV + PNG。"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
_TELEOP = ROOT / "python/teleop"
if str(_TELEOP) not in sys.path:
    sys.path.insert(0, str(_TELEOP))

from one_euro_filter import (  # noqa: E402
    OneEuroParams,
    PoseOneEuroFilter,
    wxyz_to_xyzw,
    xyzw_to_wxyz,
)

DEFAULT_CSV = ROOT / "data/test_rt_teleop/pico_teleop.csv"
CONTROL_HZ = 1000.0
DEFAULT_POS = OneEuroParams(1.15, 0.5, 1.2)
DEFAULT_ORI = OneEuroParams(1.5, 0.5, 1.2)
PICO_DT_FALLBACK = 1.0 / 50.0

POS_COLS = ("x", "y", "z")
QUAT_COLS = ("qw", "qx", "qy", "qz")


def unwrap_quat_wxyz(q: np.ndarray) -> np.ndarray:
    """相邻点同半球连续化，避免 ±q 跳变。输入/输出 shape (N, 4) = wxyz。"""
    out = np.asarray(q, dtype=np.float64).copy()
    if out.ndim != 2 or out.shape[1] != 4:
        raise ValueError(f"expect (N,4) quat, got {out.shape}")
    for i in range(1, len(out)):
        if float(np.dot(out[i], out[i - 1])) < 0.0:
            out[i] = -out[i]
    return out


def load_pico_teleop(csv_path: Path, require_fresh: bool) -> dict[str, np.ndarray]:
    data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    if data.size == 0:
        raise ValueError(f"CSV 为空: {csv_path}")
    if data.ndim == 0:
        data = np.array([data], dtype=data.dtype)

    valid = data["valid"].astype(np.int32)
    fresh = data["fresh"].astype(np.int32)
    if require_fresh:
        mask = (valid == 1) & (fresh == 1)
    else:
        mask = np.ones(len(data), dtype=bool)

    if not np.any(mask):
        raise ValueError(
            f"无可用采样点 (valid/fresh 过滤后为空): {csv_path}  "
            f"valid={int(valid.sum())} fresh={int(fresh.sum())} 可用 --all"
        )

    cycle = data["cycle"][mask].astype(np.float64)
    t = cycle / CONTROL_HZ

    def arm_arrays(prefix: str) -> tuple[np.ndarray, np.ndarray]:
        pos = np.column_stack(
            [
                data[f"{prefix}_ctrl_x"][mask],
                data[f"{prefix}_ctrl_y"][mask],
                data[f"{prefix}_ctrl_z"][mask],
            ]
        ).astype(np.float64)
        quat = np.column_stack(
            [
                data[f"{prefix}_ctrl_qw"][mask],
                data[f"{prefix}_ctrl_qx"][mask],
                data[f"{prefix}_ctrl_qy"][mask],
                data[f"{prefix}_ctrl_qz"][mask],
            ]
        ).astype(np.float64)
        return pos, unwrap_quat_wxyz(quat)

    r_pos, r_quat = arm_arrays("right")
    l_pos, l_quat = arm_arrays("left")
    return {
        "t": t,
        "cycle": cycle,
        "right_pos": r_pos,
        "right_quat": r_quat,
        "left_pos": l_pos,
        "left_quat": l_quat,
        "n_total": int(len(data)),
        "n_used": int(mask.sum()),
    }


def estimate_dt(t: np.ndarray) -> float:
    if len(t) < 2:
        return PICO_DT_FALLBACK
    diffs = np.diff(t)
    diffs = diffs[diffs > 1e-9]
    if diffs.size == 0:
        return PICO_DT_FALLBACK
    return float(np.median(diffs))


def filter_arm_poses(
    pos_wxyz: np.ndarray,
    quat_wxyz: np.ndarray,
    dt: float,
    pos_p: OneEuroParams,
    ori_p: OneEuroParams,
) -> tuple[np.ndarray, np.ndarray]:
    """CSV 为 wxyz；滤波 API 要 xyzw；返回滤波后 pos + quat wxyz。"""
    filt = PoseOneEuroFilter(pos_p, ori_p)
    n = len(pos_wxyz)
    pos_f = np.zeros_like(pos_wxyz)
    quat_f = np.zeros_like(quat_wxyz)
    for i in range(n):
        q_xyzw = wxyz_to_xyzw(quat_wxyz[i])
        p_f, q_f_xyzw = filt.filter_pose_xyzw(dt, pos_wxyz[i], q_xyzw)
        pos_f[i] = p_f
        q_wxyz = xyzw_to_wxyz(q_f_xyzw)
        quat_f[i] = q_wxyz if q_wxyz is not None else quat_wxyz[i]
    return pos_f, unwrap_quat_wxyz(quat_f)


def _plot_compare(
    ax: plt.Axes,
    t: np.ndarray,
    raw: np.ndarray,
    filt: np.ndarray,
    labels: tuple[str, ...],
    ylabel: str,
    title: str,
) -> None:
    colors = ("C0", "C1", "C2", "C3")
    for k, name in enumerate(labels):
        c = colors[k % len(colors)]
        ax.plot(t, raw[:, k], color=c, ls="-", lw=1.0, alpha=0.85, label=f"{name} raw")
        ax.plot(t, filt[:, k], color=c, ls="--", lw=1.2, label=f"{name} filt")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=7, ncol=2)


def build_arm_figure(
    t: np.ndarray,
    pos_raw: np.ndarray,
    pos_filt: np.ndarray,
    quat_raw: np.ndarray,
    quat_filt: np.ndarray,
    arm: str,
) -> plt.Figure:
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig.suptitle(f"{arm.upper()} Pico teleop — raw vs One Euro")
    _plot_compare(
        axes[0], t, pos_raw, pos_filt, POS_COLS, "pos [m]", f"{arm} position xyz"
    )
    _plot_compare(
        axes[1],
        t,
        quat_raw,
        quat_filt,
        QUAT_COLS,
        "quat",
        f"{arm} orientation qw,qx,qy,qz",
    )
    axes[1].set_xlabel("t [s] (cycle/1000)")
    fig.tight_layout()
    return fig


def save_raw_filt_csv(
    out_path: Path,
    cycle: np.ndarray,
    t: np.ndarray,
    right_pos_raw: np.ndarray,
    right_quat_raw: np.ndarray,
    right_pos_filt: np.ndarray,
    right_quat_filt: np.ndarray,
    left_pos_raw: np.ndarray,
    left_quat_raw: np.ndarray,
    left_pos_filt: np.ndarray,
    left_quat_filt: np.ndarray,
) -> None:
    header = ["cycle", "t"]
    for arm in ("right", "left"):
        for kind in ("raw", "filt"):
            for c in POS_COLS:
                header.append(f"{arm}_{kind}_{c}")
            for c in QUAT_COLS:
                header.append(f"{arm}_{kind}_{c}")

    n = len(cycle)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(n):
            row: list[float | int] = [int(cycle[i]), float(t[i])]
            for pos, quat in (
                (right_pos_raw[i], right_quat_raw[i]),
                (right_pos_filt[i], right_quat_filt[i]),
                (left_pos_raw[i], left_quat_raw[i]),
                (left_pos_filt[i], left_quat_filt[i]),
            ):
                row.extend(float(x) for x in pos)
                row.extend(float(x) for x in quat)
            writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="pico_teleop.csv：One Euro 滤波前/后对比落盘与可视化"
    )
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument(
        "--all",
        action="store_true",
        help="用全部采样（默认仅 valid==1 且 fresh==1）",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="输出目录（默认=CSV 所在目录）；写入 CSV + PNG",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="落盘后额外 plt.show()",
    )
    parser.add_argument("--pos-min-cutoff", type=float, default=DEFAULT_POS.min_cutoff)
    parser.add_argument("--pos-beta", type=float, default=DEFAULT_POS.beta)
    parser.add_argument("--pos-d-cutoff", type=float, default=DEFAULT_POS.derivative_cutoff)
    parser.add_argument("--ori-min-cutoff", type=float, default=DEFAULT_ORI.min_cutoff)
    parser.add_argument("--ori-beta", type=float, default=DEFAULT_ORI.beta)
    parser.add_argument("--ori-d-cutoff", type=float, default=DEFAULT_ORI.derivative_cutoff)
    args = parser.parse_args()

    if not args.csv.is_file():
        print(f"[FAIL] CSV 不存在: {args.csv}", file=sys.stderr)
        return 1

    try:
        series = load_pico_teleop(args.csv, require_fresh=not args.all)
    except ValueError as e:
        print(f"[FAIL] {e}", file=sys.stderr)
        return 1

    t = series["t"]
    cycle = series["cycle"]
    dt = estimate_dt(t)
    pos_p = OneEuroParams(args.pos_min_cutoff, args.pos_beta, args.pos_d_cutoff)
    ori_p = OneEuroParams(args.ori_min_cutoff, args.ori_beta, args.ori_d_cutoff)

    r_pos_f, r_quat_f = filter_arm_poses(
        series["right_pos"], series["right_quat"], dt, pos_p, ori_p
    )
    l_pos_f, l_quat_f = filter_arm_poses(
        series["left_pos"], series["left_quat"], dt, pos_p, ori_p
    )

    out_dir = args.save if args.save is not None else args.csv.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    csv_out = out_dir / "pico_teleop_raw_filt.csv"
    save_raw_filt_csv(
        csv_out,
        cycle,
        t,
        series["right_pos"],
        series["right_quat"],
        r_pos_f,
        r_quat_f,
        series["left_pos"],
        series["left_quat"],
        l_pos_f,
        l_quat_f,
    )

    fig_r = build_arm_figure(
        t, series["right_pos"], r_pos_f, series["right_quat"], r_quat_f, "right"
    )
    fig_l = build_arm_figure(
        t, series["left_pos"], l_pos_f, series["left_quat"], l_quat_f, "left"
    )
    fig_r.savefig(out_dir / "pico_teleop_right.png", dpi=150)
    fig_l.savefig(out_dir / "pico_teleop_left.png", dpi=150)

    print(
        f"csv={args.csv}  samples_total={series['n_total']}  "
        f"used={series['n_used']}  duration={t[-1] - t[0]:.2f}s  "
        f"dt={dt * 1e3:.2f}ms  mask={'all' if args.all else 'valid+fresh'}  "
        f"saved -> {out_dir}"
    )
    print(f"  {csv_out.name}  pico_teleop_right.png  pico_teleop_left.png")

    if args.show:
        plt.show()
    else:
        plt.close(fig_r)
        plt.close(fig_l)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
