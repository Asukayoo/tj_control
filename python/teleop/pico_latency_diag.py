"""Pico 遥操作延时/卡顿诊断：区分 SDK 通信 vs UDP 发布/接收。

用法:
  # 离线分析上次遥操作落盘
  python3 -m python.teleop.pico_latency_diag --csv data/test_rt_teleop/pico_teleop.csv

  # 实时：仅测 Pico SDK（PC Service → xrobotoolkit_sdk）
  python3 -m python.teleop.pico_latency_diag --sdk --seconds 15

  # 实时：仅测 UDP 到达间隔（pico_udp_publisher → mv_control 同机 loopback）
  python3 -m python.teleop.pico_latency_diag --udp --seconds 15

  # 同时测 SDK + UDP（需发布节点已在跑）
  python3 -m python.teleop.pico_latency_diag --sdk --udp --seconds 15
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

PICO_MAGIC = 0x5049434F
PICO_UDP_FMT = "<IIQ14dffB"
PICO_UDP_SIZE = struct.calcsize(PICO_UDP_FMT)
CONTROL_HZ = 500.0
RECORD_HZ = 50.0
EXPECT_RECORD_CYCLE_DELTA = int(CONTROL_HZ / RECORD_HZ)


def _pct(arr: np.ndarray, p: float) -> float:
    if arr.size == 0:
        return float("nan")
    return float(np.percentile(arr, p))


def _summ_ms(arr_ms: np.ndarray) -> str:
    if arr_ms.size == 0:
        return "n=0"
    return (
        f"n={arr_ms.size} min={arr_ms.min():.2f} p50={np.median(arr_ms):.2f} "
        f"p95={_pct(arr_ms, 95):.2f} p99={_pct(arr_ms, 99):.2f} max={arr_ms.max():.2f} ms"
    )


def analyze_csv(csv_path: Path, spike_ms: float) -> int:
    if not csv_path.is_file():
        print(f"[diag] FAIL: 不存在 {csv_path}", file=sys.stderr)
        return 1

    data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    if data.size == 0:
        print("[diag] FAIL: CSV 为空")
        return 1
    if data.ndim == 0:
        data = np.array([data], dtype=data.dtype)

    valid = data["valid"].astype(np.int32) == 1
    fresh = data["fresh"].astype(np.int32) == 1
    n = len(data)
    vidx = np.where(valid)[0]

    print(f"[diag] 离线分析: {csv_path}")
    print(f"  samples={n} valid={int(valid.sum())} fresh={int(fresh.sum())}")

    cyc = data["cycle"].astype(np.int64)
    dc = np.diff(cyc)
    bad_cyc = int(np.sum(dc != EXPECT_RECORD_CYCLE_DELTA))
    print(
        f"  记录节拍 cycle_delta: p50={np.median(dc):.0f} "
        f"期望={EXPECT_RECORD_CYCLE_DELTA} 异常={bad_cyc}/{max(1, len(dc))}"
    )

    seq = data["seq"].astype(np.int64)
    if len(vidx) > 1:
        ds = np.diff(seq[vidx])
        gap_n = int(np.sum(ds > 1))
        dup_n = int(np.sum(ds <= 0))
        print(f"  UDP seq_delta(valid): p50={np.median(ds):.0f} gap>1={gap_n} dup={dup_n}")
        print(f"  seq 范围: {int(seq[valid].min())}..{int(seq[valid].max())} "
              f"valid帧={int(valid.sum())}")

    ts = data["timestamp_ns"].astype(np.int64)
    if len(vidx) > 1:
        dts_ms = np.diff(ts[vidx]).astype(np.float64) / 1e6
        # 过滤设备时间戳回绕/重连（>5s 视为异常点，单独统计）
        sane = dts_ms[(dts_ms >= 0) & (dts_ms <= 5000)]
        wild = dts_ms[dts_ms > 5000]
        print(f"  Pico 设备 timestamp 间隔: {_summ_ms(sane)}")
        if wild.size:
            print(f"  设备 timestamp 超大跳变(>5s): {wild.size} 次 max={wild.max():.0f} ms")
        spike = sane[sane > spike_ms]
        print(f"  设备 timestamp 尖峰(>{spike_ms:.0f}ms): {spike.size} 次 "
              f"({100.0 * spike.size / max(1, sane.size):.1f}%)")

        rx = data["right_ctrl_x"].astype(np.float64)
        dx = np.abs(np.diff(rx[vidx]))
        big_dx = int(np.sum(dx > 0.05))
        print(
            f"  右手 x 逐步变化 |dx|: p50={np.median(dx):.5f} "
            f"p95={_pct(dx, 95):.5f} max={dx.max():.5f} >5cm={big_dx}"
        )

        # 尖峰时位姿是否冻结（SDK 卡顿特征）
        if spike.size:
            frozen = 0
            for i in np.where(sane > spike_ms)[0]:
                j, k = vidx[i], vidx[i + 1]
                if abs(float(rx[k]) - float(rx[j])) < 1e-4:
                    frozen += 1
            print(
                f"  尖峰期间位姿几乎不变(冻结): {frozen}/{spike.size} "
                f"→ 更像 Pico SDK/头显通信卡顿，而非 UDP 丢包"
            )

    stale_n = int(np.sum(~fresh & valid))
    print(f"  stale(valid 但 fresh=0): {stale_n}")

    print("\n[diag] 结论:")
    if int(valid.sum()) == 0:
        print("  - 未收到有效 Pico UDP → 查发布节点/端口")
        return 1

    seq_ok = len(vidx) <= 1 or int(np.sum(np.diff(seq[vidx]) > 1)) == 0
    if seq_ok and bad_cyc == 0:
        print("  - UDP 发布→mv_control 接收：seq 连续、记录节拍正常，**本地链路无明显丢包/延时**")
    elif not seq_ok:
        print("  - UDP seq 有缺口 → 发布丢帧或 500Hz 环未及时 poll（查 CPU 抢占/端口冲突）")
    if len(vidx) > 1:
        sane = np.diff(ts[vidx]).astype(np.float64) / 1e6
        sane = sane[(sane >= 0) & (sane <= 5000)]
        if sane.size and (_pct(sane, 95) > 35 or np.sum(sane > spike_ms) > max(3, sane.size * 0.02)):
            print(
                "  - Pico 设备 timestamp 抖动/冻结明显 → **优先查 PC Service / 头显无线 / USB**"
            )
            print("    建议: python3 -m python.teleop.pico_latency_diag --sdk --seconds 15")
        elif sane.size:
            print("  - Pico 设备 timestamp 较稳定 → 若仍卡顿，查 One Euro 滤波/50Hz 指令阶跃/控制 IK")
    return 0


def probe_sdk(seconds: float, rate_hz: float, spike_ms: float) -> int:
    from pico_data_receiver import PicoDataReceiver

    period = 1.0 / max(1e-3, rate_hz)
    print(f"[diag] SDK 探测 @ {rate_hz:.0f}Hz  {seconds:.1f}s（需 PC Service + 头显）")

    ts_hist: list[int] = []
    wall_hist: list[float] = []
    pose_hist: list[np.ndarray] = []
    t_end = time.perf_counter() + max(0.5, seconds)
    next_t = time.perf_counter()

    try:
        with PicoDataReceiver(auto_init=True) as rx:
            time.sleep(0.3)
            while time.perf_counter() < t_end:
                snap = rx.read_all()
                ts = int(snap["timestamp_ns"])
                rp = np.asarray(snap["poses"]["right_controller"], dtype=np.float64)
                ts_hist.append(ts)
                wall_hist.append(time.perf_counter())
                pose_hist.append(rp.copy())

                next_t += period
                sl = next_t - time.perf_counter()
                if sl > 0:
                    time.sleep(sl)
                else:
                    next_t = time.perf_counter()
    except Exception as e:
        print(f"[diag] SDK 失败: {e}", file=sys.stderr)
        return 1

    if len(ts_hist) < 2:
        print("[diag] 样本不足")
        return 1

    ts_arr = np.asarray(ts_hist, dtype=np.int64)
    wall_ms = np.diff(np.asarray(wall_hist)) * 1000.0
    dev_ms = np.diff(ts_arr).astype(np.float64) / 1e6
    dev_ms = dev_ms[(dev_ms >= 0) & (dev_ms <= 5000)]

    print(f"  轮询间隔(wall): {_summ_ms(wall_ms)}")
    print(f"  设备 timestamp 间隔: {_summ_ms(dev_ms)}")
    spike = dev_ms[dev_ms > spike_ms]
    print(f"  设备 timestamp 尖峰(>{spike_ms:.0f}ms): {spike.size}")

    frozen = 0
    for i in np.where(dev_ms > spike_ms)[0]:
        if i + 1 < len(pose_hist):
            if np.linalg.norm(pose_hist[i + 1][:3] - pose_hist[i][:3]) < 1e-4:
                frozen += 1
    if spike.size:
        print(f"  尖峰时位姿冻结: {frozen}/{spike.size}")

    unchanged_ts = int(np.sum(np.diff(ts_arr) == 0))
    print(f"  连续采样 timestamp 不变: {unchanged_ts}/{len(ts_arr)-1}")

    print("\n[diag] SDK 结论:")
    if dev_ms.size == 0:
        print("  - 设备 timestamp 异常（回绕/未更新）")
        return 1
    if _pct(dev_ms, 95) > 35 or spike.size > max(3, dev_ms.size * 0.02):
        print("  - **SDK/头显通信有卡顿**（与 UDP 无关）")
    else:
        print("  - SDK 更新较稳定；若遥操作仍卡，再测 --udp")
    return 0


def probe_udp(port: int, seconds: float, spike_ms: float) -> int:
    print(f"[diag] UDP 探测 :{port}  {seconds:.1f}s（需 pico_udp_publisher 运行）")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", port))
    sock.settimeout(1.0)

    wall_hist: list[float] = []
    seq_hist: list[int] = []
    ts_hist: list[int] = []
    t_end = time.perf_counter() + max(0.5, seconds)

    try:
        while time.perf_counter() < t_end:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                print("[diag] UDP 超时：发布节点未运行？", file=sys.stderr)
                return 1
            if len(data) < PICO_UDP_SIZE:
                continue
            vals = struct.unpack(PICO_UDP_FMT, data[:PICO_UDP_SIZE])
            if vals[0] != PICO_MAGIC:
                continue
            wall_hist.append(time.perf_counter())
            seq_hist.append(int(vals[1]))
            ts_hist.append(int(vals[2]))
    finally:
        sock.close()

    if len(wall_hist) < 2:
        print("[diag] UDP 样本不足")
        return 1

    arr_ms = np.diff(np.asarray(wall_hist)) * 1000.0
    seq_d = np.diff(np.asarray(seq_hist))
    dev_ms = np.diff(np.asarray(ts_hist)).astype(np.float64) / 1e6
    dev_ms = dev_ms[(dev_ms >= 0) & (dev_ms <= 5000)]

    print(f"  包到达间隔(wall): {_summ_ms(arr_ms)}  期望≈20ms@50Hz")
    gap = int(np.sum(seq_d > 1))
    print(f"  seq_delta: p50={np.median(seq_d):.0f} gap>1={gap}")
    print(f"  包内设备 timestamp 间隔: {_summ_ms(dev_ms)}")
    spike = arr_ms[arr_ms > spike_ms]
    print(f"  到达间隔尖峰(>{spike_ms:.0f}ms): {spike.size}")

    print("\n[diag] UDP 结论:")
    if _pct(arr_ms, 95) > 30 or gap > 0:
        print("  - **UDP 到达不稳或丢 seq** → 查发布进程调度/CPU 抢占/端口")
    elif dev_ms.size and (_pct(dev_ms, 95) > 35):
        print("  - UDP 到达稳定，但包内设备 timestamp 抖 → 根因在 SDK/头显，非 socket")
    else:
        print("  - UDP 链路正常（本地 loopback 延时通常 <1ms）")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description="Pico 延时/卡顿诊断")
    parser.add_argument("--csv", type=Path, default=None, help="离线分析 pico_teleop.csv")
    parser.add_argument("--sdk", action="store_true", help="实时测 Pico SDK")
    parser.add_argument("--udp", action="store_true", help="实时测 UDP 到达")
    parser.add_argument("--port", type=int, default=30101)
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--rate", type=float, default=50.0, help="SDK 轮询 Hz")
    parser.add_argument("--spike-ms", type=float, default=50.0, help="尖峰阈值 [ms]")
    args = parser.parse_args()

    if not args.csv and not args.sdk and not args.udp:
        args.csv = _ROOT / "data/test_rt_teleop/pico_teleop.csv"
        if not args.csv.is_file():
            parser.error("请指定 --csv / --sdk / --udp")

    rc = 0
    if args.csv is not None:
        rc = max(rc, analyze_csv(args.csv, args.spike_ms))
    if args.sdk:
        rc = max(rc, probe_sdk(args.seconds, args.rate, args.spike_ms))
    if args.udp:
        rc = max(rc, probe_udp(args.port, args.seconds, args.spike_ms))
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
