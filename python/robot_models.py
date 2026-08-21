"""Marvin 双臂 URDF 型号注册（615 / 696）。"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

MODELS: dict[str, Path] = {
    "696": ROOT
    / "urdf/Marvin_M6S_CCS_696_ urdf/urdf/"
    "Marvin M6-S-CCS-696-V4.0_Base_and_Stand_Asm urdf.urdf",
    "615": ROOT
    / "urdf/Marvin_M3S_CCS_615_urdf/urdf/"
    "Marvin_M3-S-CCS-615-V2.0_Base_and_Stand_Asm.urdf",
}

DEFAULT_MODEL = "696"

# 交互菜单：0=696，1=615（与 test_rt_teleop 一致）
_MODEL_BY_CHOICE = ("696", "615")


def resolve_model_urdf(model: str) -> Path:
  key = model.strip()
  if key not in MODELS:
    raise ValueError(f"未知型号 {model!r}，可选: {', '.join(sorted(MODELS))}")
  path = MODELS[key]
  if not path.is_file():
    raise FileNotFoundError(f"URDF 不存在: {path}")
  return path


def prompt_robot_model() -> str:
  """每次启动强制选择 URDF 型号，返回 '615' / '696'。"""
  paths = {m: resolve_model_urdf(m) for m in _MODEL_BY_CHOICE}
  while True:
    print("选择 URDF 型号（可视化与控制必须一致）:")
    for i, m in enumerate(_MODEL_BY_CHOICE):
      print(f"  {i} = {m}\n      {paths[m]}")
    raw = input("请选择 [0/1]: ").strip()
    if raw in ("0", "1"):
      return _MODEL_BY_CHOICE[int(raw)]
    print("无效输入，请重新选择。")
