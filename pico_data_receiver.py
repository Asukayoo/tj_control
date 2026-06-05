"""
PICO 数据接收模块（独立版）
===========================
本文件把从 PICO 头显/手柄接收数据的全部逻辑集中到一处，方便单独阅读、调试、复用。

数据是怎么传过来的？
--------------------
    PICO 设备  ──(无线/USB)──>  XRoboToolkit PC Service（后台服务，需先启动）
                                        │  写入共享内存
                                        ▼
                              xrobotoolkit_sdk (xrt)  ← 本文件依赖的 Python 绑定
                                        │  轮询读取（无回调，只能主动 get）
                                        ▼
                                  PicoDataReceiver

关键点：
  • SDK 只提供"轮询"接口（每次 get_* 立刻返回最新一帧），没有事件回调。
    因此真实数据更新率取决于 PC Service / 设备，而你读到的频率取决于你的循环快慢。
  • 使用前必须 xrt.init()，退出时 xrt.close()。
  • PC Service 必须已在运行且设备已连接，否则 get_* 返回的是默认/零值。

坐标与数据约定
--------------
  • 位姿 pose：长度 7 的数组 [x, y, z, qx, qy, qz, qw]
        - 前三个是位置（米），后四个是四元数，顺序是 (qx, qy, qz, qw)！
        - 注意 w 在【最后】。很多库（如 meshcat / scipy 的某些接口）用 (w, x, y, z)，
          搬运到那些库时务必重排，见 pose_quat_xyzw_to_wxyz()。
  • 扳机/握把 trigger/grip：float，范围约 [0, 1]，0=松开 1=完全按下。
  • 按钮 button：bool。
  • 摇杆 axis：[x, y]，范围约 [-1, 1]。
  • 手部追踪 hand tracking：27 x 7 数组，每行是一个关节的 [x,y,z,qx,qy,qz,qw]；
    追踪质量差时返回 None。
  • 时间戳：纳秒整数（设备时钟）。

用法
----
    # 作为库使用：
    from pico_data_receiver import PicoDataReceiver
    rx = PicoDataReceiver()              # 内部调用 xrt.init()
    snap = rx.read_all()                 # 取一帧完整快照（dict）
    pose = rx.get_pose("right_controller")
    rx.close()

    # 或用上下文管理器（自动 init/close）：
    with PicoDataReceiver() as rx:
        print(rx.get_pose("right_controller"))

    # 直接运行做实时监视（类似 read_pico.py，但带结构化输出）：
    python pico_data_receiver.py            # 默认 20 Hz 打印
    python pico_data_receiver.py --rate 50  # 50 Hz
    python pico_data_receiver.py --once     # 只打印一帧后退出
"""

from __future__ import annotations

import argparse
import time
from typing import Any, Dict, List, Optional

import numpy as np

# xrobotoolkit_sdk 是 XRoboToolkit 的 Python 绑定，封装了与 PC Service 通信的共享内存读取。
import xrobotoolkit_sdk as xrt


# ===========================================================================
#  小工具：四元数顺序转换
# ===========================================================================
def pose_quat_xyzw_to_wxyz(pose: np.ndarray) -> np.ndarray:
    """把 SDK 的 [x,y,z, qx,qy,qz,qw] 位姿转成 [x,y,z, qw,qx,qy,qz]。

    很多数学库（meshcat.transformations 等）使用 (w, x, y, z) 顺序，
    而 PICO SDK 给的是 (x, y, z, w)。需要时用本函数重排，避免姿态算错。
    """
    p = np.asarray(pose, dtype=np.float64)
    return np.array([p[0], p[1], p[2], p[6], p[3], p[4], p[5]], dtype=np.float64)


# ===========================================================================
#  PICO 数据接收器
# ===========================================================================
class PicoDataReceiver:
    """集中封装所有"从 PICO 接收数据"的 SDK 调用。

    设计为薄封装：每个方法直接对应一个/一组 xrt.* 轮询调用，并把"魔法名字串"
    映射到具体函数，便于上层按名字取数据，也方便日后替换底层 SDK。
    """

    # 设备位姿来源：名字 -> SDK 取位姿函数
    _POSE_FUNCS = {
        "left_controller": "get_left_controller_pose",
        "right_controller": "get_right_controller_pose",
        "headset": "get_headset_pose",
    }
    # 扳机/握把模拟量：名字 -> SDK 函数
    _AXIS_VALUE_FUNCS = {
        "left_trigger": "get_left_trigger",
        "right_trigger": "get_right_trigger",
        "left_grip": "get_left_grip",
        "right_grip": "get_right_grip",
    }
    # 按钮布尔量：名字 -> SDK 函数
    _BUTTON_FUNCS = {
        "A": "get_A_button",
        "B": "get_B_button",
        "X": "get_X_button",
        "Y": "get_Y_button",
        "left_menu_button": "get_left_menu_button",
        "right_menu_button": "get_right_menu_button",
        "left_axis_click": "get_left_axis_click",
        "right_axis_click": "get_right_axis_click",
    }

    def __init__(self, auto_init: bool = True):
        """构造时默认调用 xrt.init() 建立与 PC Service 的连接。

        Args:
            auto_init: 是否在构造时自动 init。若外部已 init，可传 False。
        """
        self._initialized = False
        if auto_init:
            self.init()

    # ---- 生命周期 --------------------------------------------------------
    def init(self) -> None:
        """初始化 SDK（连接 PC Service 的共享内存）。重复调用是安全的。"""
        if not self._initialized:
            xrt.init()
            self._initialized = True
            print("[PicoDataReceiver] XRoboToolkit SDK 初始化完成。")

    def close(self) -> None:
        """释放 SDK 资源。退出前务必调用。"""
        if self._initialized:
            xrt.close()
            self._initialized = False
            print("[PicoDataReceiver] SDK 已关闭。")

    def __enter__(self) -> "PicoDataReceiver":
        self.init()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ---- 时间戳 ----------------------------------------------------------
    def get_timestamp_ns(self) -> int:
        """返回设备端最新数据的时间戳（纳秒）。可用于判断数据是否在更新。"""
        return xrt.get_time_stamp_ns()

    # ---- 位姿（头显 / 左右手柄）------------------------------------------
    def get_pose(self, name: str) -> np.ndarray:
        """按名字取位姿，返回 [x, y, z, qx, qy, qz, qw]。

        Args:
            name: "left_controller" | "right_controller" | "headset"
        """
        func_name = self._POSE_FUNCS.get(name)
        if func_name is None:
            raise ValueError(f"未知位姿来源: {name}. 可选: {list(self._POSE_FUNCS)}")
        return np.asarray(getattr(xrt, func_name)(), dtype=np.float64)

    # ---- 扳机 / 握把（模拟量）-------------------------------------------
    def get_value(self, name: str) -> float:
        """按名字取扳机/握把模拟量（float, 约 0~1）。

        Args:
            name: "left_trigger" | "right_trigger" | "left_grip" | "right_grip"
        """
        func_name = self._AXIS_VALUE_FUNCS.get(name)
        if func_name is None:
            raise ValueError(f"未知模拟量: {name}. 可选: {list(self._AXIS_VALUE_FUNCS)}")
        return float(getattr(xrt, func_name)())

    # ---- 按钮（布尔量）--------------------------------------------------
    def get_button(self, name: str) -> bool:
        """按名字取按钮状态（bool）。

        Args:
            name: A/B/X/Y, left/right_menu_button, left/right_axis_click
        """
        func_name = self._BUTTON_FUNCS.get(name)
        if func_name is None:
            raise ValueError(f"未知按钮: {name}. 可选: {list(self._BUTTON_FUNCS)}")
        return bool(getattr(xrt, func_name)())

    # ---- 摇杆 ------------------------------------------------------------
    def get_axis(self, controller: str) -> List[float]:
        """取摇杆 [x, y]（约 -1~1）。

        Args:
            controller: "left" | "right"
        """
        if controller.lower() == "left":
            return list(xrt.get_left_axis())
        elif controller.lower() == "right":
            return list(xrt.get_right_axis())
        raise ValueError(f"未知手柄: {controller}. 可选: 'left', 'right'.")

    # ---- 手部追踪 --------------------------------------------------------
    def get_hand_tracking(self, hand: str) -> Optional[np.ndarray]:
        """取手部追踪 27x7 数组（每行 [x,y,z,qx,qy,qz,qw]）。

        追踪质量低（未激活）时返回 None。

        Args:
            hand: "left" | "right"
        """
        if hand.lower() == "left":
            if not xrt.get_left_hand_is_active():
                return None
            return np.asarray(xrt.get_left_hand_tracking_state(), dtype=np.float64)
        elif hand.lower() == "right":
            if not xrt.get_right_hand_is_active():
                return None
            return np.asarray(xrt.get_right_hand_tracking_state(), dtype=np.float64)
        raise ValueError(f"未知手: {hand}. 可选: 'left', 'right'.")

    # ---- 运动追踪器（Motion Tracker）------------------------------------
    def get_motion_trackers(self) -> Dict[str, Dict[str, Any]]:
        """取所有运动追踪器数据，按序列号(serial)分组。

        每个追踪器含 pose / velocity / acceleration。无追踪器时返回空字典。
        """
        n = xrt.num_motion_data_available()
        if n == 0:
            return {}
        poses = xrt.get_motion_tracker_pose()
        vels = xrt.get_motion_tracker_velocity()
        accs = xrt.get_motion_tracker_acceleration()
        serials = xrt.get_motion_tracker_serial_numbers()
        out: Dict[str, Dict[str, Any]] = {}
        for i in range(n):
            out[serials[i]] = {
                "pose": np.asarray(poses[i], dtype=np.float64),
                "velocity": np.asarray(vels[i], dtype=np.float64),
                "acceleration": np.asarray(accs[i], dtype=np.float64),
            }
        return out

    # ---- 全身追踪（Body Tracking）--------------------------------------
    def get_body_tracking(self) -> Optional[Dict[str, np.ndarray]]:
        """取全身追踪数据（24 个关节），不可用时返回 None。

        返回字典：
          poses        (24, 7) [x,y,z,qx,qy,qz,qw]
          velocities   (24, 6) [vx,vy,vz,wx,wy,wz]
          accelerations(24, 6) [ax,ay,az,wax,way,waz]
        """
        if not xrt.is_body_data_available():
            return None
        return {
            "poses": np.asarray(xrt.get_body_joints_pose(), dtype=np.float64),
            "velocities": np.asarray(xrt.get_body_joints_velocity(), dtype=np.float64),
            "accelerations": np.asarray(xrt.get_body_joints_acceleration(), dtype=np.float64),
        }

    # ---- 一次性取完整快照 ------------------------------------------------
    def read_all(self) -> Dict[str, Any]:
        """轮询一次，返回当前所有可用数据的结构化快照（dict）。

        适合做数据记录 / 调试可视化。注意：每个字段都是独立 get，
        因此严格来说不是"同一时刻"的原子快照，但在高频轮询下足够接近。
        """
        return {
            "timestamp_ns": self.get_timestamp_ns(),
            "poses": {name: self.get_pose(name) for name in self._POSE_FUNCS},
            "values": {name: self.get_value(name) for name in self._AXIS_VALUE_FUNCS},
            "buttons": {name: self.get_button(name) for name in self._BUTTON_FUNCS},
            "axes": {"left": self.get_axis("left"), "right": self.get_axis("right")},
            "hand_tracking": {
                "left": self.get_hand_tracking("left"),
                "right": self.get_hand_tracking("right"),
            },
            "motion_trackers": self.get_motion_trackers(),
            "body_tracking": self.get_body_tracking(),
        }


# ===========================================================================
#  命令行实时监视（独立运行入口）
# ===========================================================================
def _format_pose(p: np.ndarray) -> str:
    return ("pos=[{:+.3f} {:+.3f} {:+.3f}]  "
            "quat(xyzw)=[{:+.3f} {:+.3f} {:+.3f} {:+.3f}]").format(*p[:7])


def _monitor(rate_hz: float, once: bool) -> None:
    """实时打印接收到的 PICO 数据，用于检验数据质量 / 连接是否正常。"""
    period = 1.0 / max(1e-3, rate_hz)
    with PicoDataReceiver() as rx:
        time.sleep(0.5)  # 给 PC Service 一点时间填充首帧数据
        try:
            while True:
                snap = rx.read_all()
                # 清屏 + 回到左上角（ANSI），实现原地刷新
                if not once:
                    print("\033[2J\033[H", end="")
                print(f"timestamp_ns : {snap['timestamp_ns']}")
                for name, pose in snap["poses"].items():
                    print(f"{name:16s}: {_format_pose(pose)}")
                vals = snap["values"]
                print("trigger/grip : L_trig={:.2f} R_trig={:.2f} L_grip={:.2f} R_grip={:.2f}".format(
                    vals["left_trigger"], vals["right_trigger"],
                    vals["left_grip"], vals["right_grip"]))
                btn = snap["buttons"]
                print("buttons      : A={A} B={B} X={X} Y={Y}  "
                      "Lmenu={left_menu_button} Rmenu={right_menu_button}".format(**btn))
                print("axes         : left={}  right={}".format(
                    [round(v, 3) for v in snap["axes"]["left"]],
                    [round(v, 3) for v in snap["axes"]["right"]]))
                ht = snap["hand_tracking"]
                print("hand_tracking: left={}  right={}".format(
                    "active" if ht["left"] is not None else "inactive",
                    "active" if ht["right"] is not None else "inactive"))
                mt = snap["motion_trackers"]
                print(f"motion_trackers: {len(mt)} 个  body_tracking: "
                      f"{'available' if snap['body_tracking'] else 'none'}")

                if once:
                    break
                time.sleep(period)
        except KeyboardInterrupt:
            print("\n[PicoDataReceiver] 收到中断，退出监视。")


def main() -> None:
    parser = argparse.ArgumentParser(description="PICO 数据接收 / 实时监视")
    parser.add_argument("--rate", type=float, default=20.0, help="打印频率 Hz（默认 20）")
    parser.add_argument("--once", action="store_true", help="只取一帧并打印后退出")
    args = parser.parse_args()
    _monitor(args.rate, args.once)


if __name__ == "__main__":
    main()
