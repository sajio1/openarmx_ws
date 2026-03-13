#!/usr/bin/env python3
"""检查关节旋转轴方向是否与期望一致.

默认期望来自 Meta Quest 工程师提供定义:
  left : [-Y, -X, -Z, -Y, -Z, +X, -Y]
  right: [+Y, -X, -Z, -Y, -Z, +X, +Y]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pinocchio as pin


EXPECTED = {
    "left": ["-Y", "-X", "-Z", "-Y", "-Z", "+X", "-Y"],
    "right": ["+Y", "-X", "-Z", "-Y", "-Z", "+X", "+Y"],
}


def axis_to_label(v: np.ndarray) -> str:
    idx = int(np.argmax(np.abs(v)))
    sign = "+" if v[idx] >= 0 else "-"
    name = ["X", "Y", "Z"][idx]
    return f"{sign}{name}"


def load_model(workspace: Path):
    urdf = workspace / "install" / "openarmx_description" / "share" / "openarmx_description" / "urdf" / "robot" / "openarmx_bimanual_sim.urdf"
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    q = pin.neutral(model)
    pin.forwardKinematics(model, data, q)
    pin.computeJointJacobians(model, data, q)
    return model, data


def actual_axes(model, data, side: str) -> list[str]:
    labels = []
    for i in range(1, 8):
        name = f"openarmx_{side}_joint{i}"
        jid = model.getJointId(name)
        col = model.joints[jid].idx_v
        jac = pin.getJointJacobian(model, data, jid, pin.ReferenceFrame.WORLD)
        w = jac[3:, col]
        w = w / np.linalg.norm(w)
        labels.append(axis_to_label(w))
    return labels


def main():
    parser = argparse.ArgumentParser(description="关节轴方向一致性检查")
    parser.add_argument("--workspace", default="/home/ok/Desktop/openarmx_ws", help="workspace 路径")
    args = parser.parse_args()

    ws = Path(args.workspace).expanduser().resolve()
    model, data = load_model(ws)

    print("=" * 72)
    print("Joint Axis Consistency (world frame @ neutral)")
    print("=" * 72)
    any_fail = False

    for side in ("right", "left"):
        exp = EXPECTED[side]
        act = actual_axes(model, data, side)
        print(f"\n{side.upper()}:")
        for i, (a, e) in enumerate(zip(act, exp), start=1):
            ok = a == e
            any_fail = any_fail or (not ok)
            status = "OK" if ok else "MISMATCH"
            print(f"  J{i}: actual={a:>3s} expected={e:>3s}  [{status}]")

    print("\n" + ("RESULT: FAIL" if any_fail else "RESULT: PASS"))


if __name__ == "__main__":
    main()
