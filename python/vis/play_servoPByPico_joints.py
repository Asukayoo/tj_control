#!/usr/bin/env python3
"""读取 test_servoPByPico 关节 CSV（1000Hz），按 50Hz 重采样后经 UDP 发送 [rad]。

配合 urdf_pybullet_vis.py 使用：
  1) 先启动 urdf_pybullet_vis.py（绑定 UDP 端口）
  2) 再启动本脚本发送轨迹
"""

from __future__ import annotations

import argparse
import csv
import socket
import struct
import time
from pathlib import Path

DOF_ARM = 7
DOF = 14
SRC_DT = 0.001  # CSV 原始采样 1000 Hz
HZ = 50
DT = 1.0 / HZ  # 播放 50 Hz = 20 ms
UDP_FMT = "<14d"  # 小端 float64：左 7 + 右 7 [rad]
UDP_PKT_SIZE = struct.calcsize(UDP_FMT)

DEFAULT_LEFT = Path("/home/yxc/tj_control/data/test_servoPByPico/left_joint.csv")
DEFAULT_RIGHT = Path("/home/yxc/tj_control/data/test_servoPByPico/right_joint.csv")
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 30100


def load_joint_csv(path: Path) -> tuple[list[float], list[list[float]]]:
    """读取 joint CSV（1000Hz），返回 (t[s], q[rad] 列表)。"""
    times: list[float] = []
    joints: list[list[float]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["t"]))
            joints.append([float(row[f"q{i}"]) for i in range(DOF_ARM)])
    if not joints:
        raise ValueError(f"空文件: {path}")
    return times, joints


def sample_at_time(times: list[float], joints: list[list[float]], t: float) -> list[float]:
    """从 1000Hz 序列按时间 t 取最近样本；超出末尾保持最后一帧。"""
    if t <= times[0]:
        return joints[0]
    idx = 0
    while idx + 1 < len(times) and times[idx + 1] <= t + 1e-9:
        idx += 1
    return joints[idx]


def build_playback_grid(
    left_times: list[float],
    left_q: list[list[float]],
    right_times: list[float],
    right_q: list[list[float]],
) -> tuple[list[float], list[list[float]], list[list[float]]]:
    """将 1000Hz CSV 对齐到 50Hz 时间网格，返回 (t_play, left_q, right_q)。"""
    t0 = min(left_times[0], right_times[0])
    t1 = max(left_times[-1], right_times[-1])
    n_steps = int(round((t1 - t0) / DT)) + 1

    t_play: list[float] = []
    l_out: list[list[float]] = []
    r_out: list[list[float]] = []
    for i in range(n_steps):
        t = t0 + i * DT
        t_play.append(t)
        l_out.append(sample_at_time(left_times, left_q, t))
        r_out.append(sample_at_time(right_times, right_q, t))
    return t_play, l_out, r_out


def pack_packet(left_q: list[float], right_q: list[float]) -> bytes:
    if len(left_q) != DOF_ARM or len(right_q) != DOF_ARM:
        raise ValueError("每臂需 7 个关节角")
    return struct.pack(UDP_FMT, *(left_q + right_q))


def run(
    left_csv: Path,
    right_csv: Path,
    host: str,
    port: int,
    speed: float,
) -> None:
    left_t, left_q = load_joint_csv(left_csv)
    right_t, right_q = load_joint_csv(right_csv)
    t_play, l_play, r_play = build_playback_grid(left_t, left_q, right_t, right_q)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dt = DT / max(speed, 1e-9)

    print(f"left : {left_csv} src_rows={len(left_q)} @ {1/SRC_DT:.0f}Hz")
    print(f"right: {right_csv} src_rows={len(right_q)} @ {1/SRC_DT:.0f}Hz")
    print(f"play : {len(t_play)} frames @ {HZ}Hz (DT={DT*1000:.0f} ms, 从 1000Hz 对齐采样)")
    print(f"unit : q [rad]")
    print(f"UDP  : {host}:{port}  packet={UDP_PKT_SIZE}B  wall_dt={dt*1000:.3f} ms")
    print(f"time : [{t_play[0]:.3f}, {t_play[-1]:.3f}] s  speed={speed}x")
    print("开始发送… Ctrl+C 退出", flush=True)

    next_tick = time.perf_counter()
    sent = 0
    try:
        for lq, rq in zip(l_play, r_play):
            pkt = pack_packet(lq, rq)
            sock.sendto(pkt, (host, port))
            sent += 1

            next_tick += dt
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.perf_counter()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        sock.close()
        print(f"已发送 {sent} 包")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="播放 test_servoPByPico joint CSV → UDP 50Hz（配合 urdf_pybullet_vis.py）"
    )
    parser.add_argument("--left", type=Path, default=DEFAULT_LEFT)
    parser.add_argument("--right", type=Path, default=DEFAULT_RIGHT)
    parser.add_argument("--host", default=DEFAULT_HOST, help="UDP 目标地址")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP 端口")
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="播放倍速，1.0=实时 50Hz（20ms/帧）",
    )
    args = parser.parse_args()

    for p in (args.left, args.right):
        if not p.is_file():
            raise SystemExit(f"文件不存在: {p}")

    run(args.left, args.right, args.host, args.port, args.speed)


if __name__ == "__main__":
    main()
