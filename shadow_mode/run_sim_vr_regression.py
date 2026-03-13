#!/usr/bin/env python3
"""SIM-VR 回归测试（不依赖真实 Quest）.

覆盖用例:
  1) 零输入稳定性
  2) clutch 边沿首帧
  3) 单轴方向
  4) 旋转补偿开关
  5) 压力与鲁棒性（90Hz逻辑步进）
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

WORKSPACE = Path("/home/ok/Desktop/openarmx_ws")
if str(WORKSPACE) not in sys.path:
    sys.path.insert(0, str(WORKSPACE))

from shadow_mode.vr_controller import config as cfg
from shadow_mode.vr_controller.tracker import ArmTracker
from shadow_mode.vr_controller.transforms import quat_to_rotmat


@dataclass
class CaseResult:
    name: str
    passed: bool
    detail: str


def nearly_equal(a: np.ndarray, b: np.ndarray, eps: float = 1e-6) -> bool:
    return np.max(np.abs(a - b)) <= eps


def run_case_zero_stability() -> CaseResult:
    t = ArmTracker("right")
    now = 0.0
    for _ in range(200):
        t.on_pose(np.array([0.1, -0.2, 0.3]), np.eye(3), now)
        now += 1.0 / 90.0
        pos, rot = t.compute_target()
        if pos is not None or rot is not None:
            return CaseResult("zero_input_stability", False, "clutch OFF 时不应输出目标")
    return CaseResult("zero_input_stability", True, "clutch OFF 保持冻结")


def run_case_clutch_edge() -> CaseResult:
    old_offset = cfg.GRIPPER_ROT_OFFSET_ENABLED
    cfg.GRIPPER_ROT_OFFSET_ENABLED = True
    try:
        t = ArmTracker("right")
        now = 1.0
        pose = np.array([0.2, 0.1, 0.4])
        t.on_pose(pose, np.eye(3), now)
        t.on_clutch(True, now)
        t.capture_reference(np.array([0.3, -0.2, 0.1]), np.eye(3))
        t.update_gain_ramp(now)  # gain=0
        pos, rot = t.compute_target()
        if pos is None or rot is None:
            return CaseResult("clutch_edge_first_frame", False, "ON 首帧应有目标")
        if not nearly_equal(pos, np.array([0.3, -0.2, 0.1]), 1e-9):
            return CaseResult("clutch_edge_first_frame", False, "ON 首帧位置应严格等于参考位姿")
        if not nearly_equal(rot, np.eye(3), 1e-9):
            return CaseResult("clutch_edge_first_frame", False, "ON 首帧旋转应严格等于参考旋转")
        return CaseResult("clutch_edge_first_frame", True, "首帧稳定，无额外跳变")
    finally:
        cfg.GRIPPER_ROT_OFFSET_ENABLED = old_offset


def run_case_single_axis() -> CaseResult:
    t = ArmTracker("right")
    base = np.array([0.0, 0.0, 0.0])
    t.on_pose(base, np.eye(3), 0.0)
    t.on_clutch(True, 0.0)
    t.capture_reference(np.array([1.0, 2.0, 3.0]), np.eye(3))
    t.current_gain = 1.0

    tests = [
        (np.array([0.1, 0.0, 0.0]), np.array([1.0 + 0.1 * cfg.POSITION_SCALE, 2.0, 3.0]), "X"),
        (np.array([0.0, 0.1, 0.0]), np.array([1.0, 2.0 + 0.1 * cfg.POSITION_SCALE, 3.0]), "Y"),
        (np.array([0.0, 0.0, 0.1]), np.array([1.0, 2.0, 3.0 + 0.1 * cfg.POSITION_SCALE]), "Z"),
    ]
    for delta, expected, axis in tests:
        t.on_pose(base + delta, np.eye(3), 0.01)
        pos, _ = t.compute_target()
        if pos is None or not nearly_equal(pos, expected, 1e-8):
            return CaseResult("single_axis_direction", False, f"{axis} 轴方向不符合预期")
    return CaseResult("single_axis_direction", True, "XYZ 单轴方向通过")


def run_case_rot_offset_toggle() -> CaseResult:
    t = ArmTracker("right")
    t.on_pose(np.array([0.0, 0.0, 0.0]), np.eye(3), 0.0)
    t.on_clutch(True, 0.0)
    t.capture_reference(np.array([0.0, 0.0, 0.0]), np.eye(3))

    # OFF: 始终保持单位旋转
    old = cfg.GRIPPER_ROT_OFFSET_ENABLED
    cfg.GRIPPER_ROT_OFFSET_ENABLED = False
    t.current_gain = 1.0
    _, rot_off = t.compute_target()
    if rot_off is None or not nearly_equal(rot_off, np.eye(3), 1e-8):
        cfg.GRIPPER_ROT_OFFSET_ENABLED = old
        return CaseResult("rotation_offset_toggle", False, "补偿 OFF 时不应额外旋转")

    # ON: gain=0 时不应突变，gain=1 时应变化
    cfg.GRIPPER_ROT_OFFSET_ENABLED = True
    t.current_gain = 0.0
    _, rot_g0 = t.compute_target()
    t.current_gain = 1.0
    _, rot_g1 = t.compute_target()
    cfg.GRIPPER_ROT_OFFSET_ENABLED = old

    if rot_g0 is None or not nearly_equal(rot_g0, np.eye(3), 1e-8):
        return CaseResult("rotation_offset_toggle", False, "补偿 ON 且 gain=0 时不应突变")
    if rot_g1 is None or nearly_equal(rot_g1, np.eye(3), 1e-3):
        return CaseResult("rotation_offset_toggle", False, "补偿 ON 且 gain=1 时应体现补偿")
    return CaseResult("rotation_offset_toggle", True, "补偿开关行为正确")


def run_case_stress() -> CaseResult:
    t = ArmTracker("right")
    t.on_pose(np.array([0.0, 0.0, 0.0]), np.eye(3), 0.0)
    t.on_clutch(True, 0.0)
    t.capture_reference(np.array([0.0, 0.0, 0.0]), np.eye(3))
    t.current_gain = 1.0

    # 90Hz * 600s = 54000 steps (逻辑压力，不sleep)
    steps = 54000
    for i in range(steps):
        x = 0.1 * math.sin(i * 0.003)
        y = 0.1 * math.cos(i * 0.002)
        z = 0.05 * math.sin(i * 0.005)
        yaw = 0.2 * math.sin(i * 0.004)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        rot = quat_to_rotmat(0.0, 0.0, qz, qw)
        t.on_pose(np.array([x, y, z]), rot, i / 90.0)
        pos, out_rot = t.compute_target()
        if pos is None or out_rot is None:
            return CaseResult("stress_90hz", False, f"第 {i} 步输出为空")
        if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(out_rot)):
            return CaseResult("stress_90hz", False, f"第 {i} 步出现 NaN/Inf")
    return CaseResult("stress_90hz", True, f"{steps} 步通过")


def main():
    cases = [
        run_case_zero_stability(),
        run_case_clutch_edge(),
        run_case_single_axis(),
        run_case_rot_offset_toggle(),
        run_case_stress(),
    ]
    print("=" * 72)
    print("SIM-VR Regression Results")
    print("=" * 72)
    all_pass = True
    for c in cases:
        all_pass &= c.passed
        status = "PASS" if c.passed else "FAIL"
        print(f"[{status}] {c.name}: {c.detail}")

    report = Path("/home/ok/Desktop/openarmx_ws/shadow_mode/test_report_sim_vr.md")
    report.write_text(
        "# SIM-VR 回归测试报告\n\n"
        + "\n".join(
            [f"- [{'PASS' if c.passed else 'FAIL'}] `{c.name}`: {c.detail}" for c in cases]
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"\n报告已写入: {report}")
    raise SystemExit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
