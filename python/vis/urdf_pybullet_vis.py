#!/usr/bin/env python3
"""PyBullet 双臂 URDF 可视化：UDP 订阅 14 关节角 [rad]，500Hz 刷新。"""

from __future__ import annotations

import argparse
import re
import socket
import struct
import sys
import tempfile
import time
from pathlib import Path

import pybullet as p
import pybullet_data

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from python.robot_models import prompt_robot_model, resolve_model_urdf  # noqa: E402

DOF = 14
HZ = 500
DT = 1.0 / HZ  # 500 Hz = 2 ms
UDP_FMT = "<14d"  # 小端 float64：左臂 7 + 右臂 7 [rad]
UDP_PKT_SIZE = struct.calcsize(UDP_FMT)

# URDF 可控关节名（Joint0 为 fixed，仅 Joint1~7）
JOINT_NAMES = [f"Joint{i}_L" for i in range(1, 8)] + [
    f"Joint{i}_R" for i in range(1, 8)
]


def prepare_urdf(urdf_path: Path) -> Path:
    """将 mesh 路径替换为本地绝对路径，供 PyBullet 加载。"""
    mesh_dir = (urdf_path.parent.parent / "meshes").resolve()
    if not mesh_dir.is_dir():
        raise FileNotFoundError(f"mesh 目录不存在: {mesh_dir}")

    mesh_prefix = str(mesh_dir).replace("\\", "/") + "/"
    text = urdf_path.read_text(encoding="utf-8")
    text = re.sub(
        r'package://[^"]+/meshes/',
        mesh_prefix,
        text,
    )
    text = re.sub(
        r'\.\./meshes/',
        mesh_prefix,
        text,
    )
    tmp = tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".urdf",
        prefix="marvin_fixed_",
        delete=False,
        encoding="utf-8",
    )
    tmp.write(text)
    tmp.flush()
    tmp.close()
    return Path(tmp.name)


def build_joint_index_map(robot_id: int) -> list[int]:
    """按 JOINT_NAMES 顺序返回 PyBullet 关节索引。"""
    name_to_idx: dict[str, int] = {}
    for ji in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, ji)
        name_to_idx[info[1].decode("utf-8")] = ji

    missing = [n for n in JOINT_NAMES if n not in name_to_idx]
    if missing:
        raise RuntimeError(f"URDF 缺少关节: {missing}")

    return [name_to_idx[n] for n in JOINT_NAMES]


def parse_joint_packet(data: bytes) -> list[float] | None:
    if len(data) < UDP_PKT_SIZE:
        return None
    return list(struct.unpack(UDP_FMT, data[:UDP_PKT_SIZE]))


def run(urdf: Path, udp_port: int, gui: bool) -> None:
    fixed_urdf = prepare_urdf(urdf)

    mode = p.GUI if gui else p.DIRECT
    client = p.connect(mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1 if gui else 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=2.2,
        cameraYaw=50,
        cameraPitch=-25,
        cameraTargetPosition=[0.0, 0.0, 0.8],
    )

    robot_id = p.loadURDF(
        str(fixed_urdf),
        basePosition=[0.0, 0.0, 0.0],
        baseOrientation=[0.0, 0.0, 0.0, 1.0],
        useFixedBase=True,
        flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
    )
    joint_indices = build_joint_index_map(robot_id)

    # 初始零位
    q_cmd = [0.0] * DOF
    for ji, q in zip(joint_indices, q_cmd):
        p.resetJointState(robot_id, ji, q)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", udp_port))
    sock.setblocking(False)

    print(f"URDF: {urdf}", flush=True)
    print(
        f"UDP 监听 0.0.0.0:{udp_port}，包格式 {UDP_PKT_SIZE}B = 14×float64 [rad]",
        flush=True,
    )
    print(f"关节顺序: {', '.join(JOINT_NAMES)}", flush=True)
    print(f"{HZ}Hz 刷新（DT={DT*1000:.0f} ms），Ctrl+C 退出", flush=True)

    next_tick = time.perf_counter()
    pkt_count = 0
    try:
        while True:
            # 非阻塞收包，保留最新关节角
            while True:
                try:
                    data, addr = sock.recvfrom(2048)
                except BlockingIOError:
                    break
                parsed = parse_joint_packet(data)
                if parsed is not None:
                    q_cmd = parsed
                    pkt_count += 1
                else:
                    print(f"忽略非法包 len={len(data)} from {addr}", file=sys.stderr)

            for ji, q in zip(joint_indices, q_cmd):
                p.resetJointState(robot_id, ji, q)

            if gui:
                p.stepSimulation()

            next_tick += DT
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                # 周期跟不上时重新对齐
                next_tick = time.perf_counter()

    except KeyboardInterrupt:
        print(f"\n退出，累计收到 {pkt_count} 个有效 UDP 包")
    finally:
        sock.close()
        p.disconnect(client)
        fixed_urdf.unlink(missing_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="PyBullet 双臂 URDF + UDP 关节可视化")
    parser.add_argument(
        "--model",
        choices=("615", "696"),
        default=None,
        help="已废弃：启动时强制交互选择；若传入仅打印 WARN",
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=None,
        help="显式 URDF 路径（跳过交互选择，仅调试用）",
    )
    parser.add_argument("--udp-port", type=int, default=30100, help="UDP 端口")
    parser.add_argument("--no-gui", action="store_true", help="无头模式 DIRECT")
    args = parser.parse_args()

    if args.urdf is not None:
        urdf = args.urdf
        model_label = "custom"
    else:
        if args.model is not None:
            print("[WARN] --model 已忽略，启动时将交互选择 URDF", flush=True)
        model_label = prompt_robot_model()
        urdf = resolve_model_urdf(model_label)
    if not urdf.is_file():
        raise SystemExit(f"URDF 不存在: {urdf}")

    print(f"型号: {model_label}\nurdf: {urdf}", flush=True)
    run(urdf, args.udp_port, gui=not args.no_gui)


if __name__ == "__main__":
    main()
