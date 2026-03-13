#!/usr/bin/env python3
"""
Parse SMPL-X NPZ files and print structure, shapes, and arm-joint statistics.

Usage:
    .venv/bin/python3 parse_smpl.py smpl_data_2_eric/handshake
    .venv/bin/python3 parse_smpl.py smpl_data_2_eric/fold_arms
"""

import sys
import os
import numpy as np

# SMPL-X body joint names (21 joints in pose_body, axis-angle 3 each)
SMPLX_BODY_JOINTS = [
    "L_Hip", "R_Hip", "Spine1",
    "L_Knee", "R_Knee", "Spine2",
    "L_Ankle", "R_Ankle", "Spine3",
    "L_Foot", "R_Foot", "Neck",
    "L_Collar", "R_Collar", "Head",
    "L_Shoulder", "R_Shoulder", "L_Elbow",
    "R_Elbow", "L_Wrist", "R_Wrist",
]

ARM_JOINT_INDICES = {
    "L_Collar": 12, "R_Collar": 13,
    "L_Shoulder": 15, "R_Shoulder": 16,
    "L_Elbow": 17, "R_Elbow": 18,
    "L_Wrist": 19, "R_Wrist": 20,
}

ROBOT_JOINT_LIMITS = {
    "joint1": (-1.25, 3.0),
    "joint2": (-1.70, 1.70),
    "joint3": (-1.57, 1.57),
    "joint4": (0.0, 1.8),
    "joint5": (-1.50, 1.50),
    "joint6": (-0.75, 0.75),
    "joint7": (-1.40, 1.40),
}


def print_section(title: str):
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


def print_npz_contents(path: str):
    """Load an NPZ and print every key with shape/dtype/sample values."""
    data = np.load(path, allow_pickle=True)
    for key in sorted(data.files):
        arr = data[key]
        print(f"  {key:25s}  shape={str(arr.shape):18s}  dtype={arr.dtype}")
        if arr.ndim == 0:
            print(f"    {'value':>10s}: {arr}")
        elif arr.size <= 20:
            print(f"    {'values':>10s}: {arr}")
        elif arr.ndim >= 2:
            print(f"    {'first row':>10s}: {arr[0][:min(10, arr.shape[-1])]}")
    return data


def print_arm_joint_stats(pose_body: np.ndarray):
    """Print per-axis statistics for all 8 arm joints."""
    n_frames = pose_body.shape[0]
    print(f"\n  Frames: {n_frames}")
    print(f"  {'Joint':<14s} {'Axis':>4s} {'Min':>9s} {'Max':>9s} {'Mean':>9s} {'Std':>9s}")
    print(f"  {'-'*55}")

    for name in ["L_Collar", "L_Shoulder", "L_Elbow", "L_Wrist",
                  "R_Collar", "R_Shoulder", "R_Elbow", "R_Wrist"]:
        idx = ARM_JOINT_INDICES[name]
        joint_data = pose_body[:, idx * 3:(idx + 1) * 3]
        for ax_i, ax_name in enumerate(["x", "y", "z"]):
            v = joint_data[:, ax_i]
            print(f"  {name:<14s} {ax_name:>4s} {v.min():9.4f} {v.max():9.4f} "
                  f"{v.mean():9.4f} {v.std():9.4f}")
        if name in ("L_Wrist", "R_Wrist"):
            pass
        elif name.startswith("L_") and "Wrist" not in name:
            pass
        print()


def print_trajectory_vs_limits(traj_data):
    """Compare pre-retargeted joint ranges against robot limits."""
    print(f"  {'Side':<6s} {'Joint':<8s} {'Traj Min':>9s} {'Traj Max':>9s} "
          f"{'Lim Low':>9s} {'Lim High':>9s} {'OK?':>5s}")
    print(f"  {'-'*55}")

    for side in ["left", "right"]:
        key = f"{side}_joints"
        if key not in traj_data.files:
            print(f"  {side}: key '{key}' not found")
            continue
        joints = traj_data[key]
        for j in range(joints.shape[1]):
            jname = f"joint{j + 1}"
            lo, hi = ROBOT_JOINT_LIMITS[jname]
            vmin, vmax = joints[:, j].min(), joints[:, j].max()
            ok = "Yes" if vmin >= lo - 0.01 and vmax <= hi + 0.01 else "NO"
            print(f"  {side:<6s} {jname:<8s} {vmin:9.4f} {vmax:9.4f} "
                  f"{lo:9.4f} {hi:9.4f} {ok:>5s}")
        print()


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <motion_folder>")
        print(f"  e.g.: {sys.argv[0]} smpl_data_2_eric/handshake")
        sys.exit(1)

    folder = sys.argv[1]

    # --- Raw SMPL-X file ---
    raw_path = os.path.join(folder, "smplx_motion.npz")
    if os.path.exists(raw_path):
        print_section(f"Raw SMPL-X: {raw_path}")
        raw = print_npz_contents(raw_path)

        if "pose_body" in raw.files:
            print_section("Arm Joint Statistics (axis-angle from pose_body)")
            print_arm_joint_stats(raw["pose_body"])

            print_section("SMPL-X Body Joint Index Reference")
            for i, name in enumerate(SMPLX_BODY_JOINTS):
                marker = " <-- ARM" if name in ARM_JOINT_INDICES else ""
                print(f"  [{i:2d}] {name}{marker}")
    else:
        print(f"[SKIP] {raw_path} not found")

    # --- Pre-retargeted trajectory file ---
    traj_path = os.path.join(folder, "smplx_motion_trajectory.npz")
    if os.path.exists(traj_path):
        print_section(f"Pre-retargeted Trajectory: {traj_path}")
        traj = print_npz_contents(traj_path)

        print_section("Trajectory Joints vs. Robot Limits (base, no bimanual offsets)")
        print_trajectory_vs_limits(traj)
    else:
        print(f"\n[SKIP] {traj_path} not found")

    # --- 8D variant (if present) ---
    traj8d_path = os.path.join(folder, "smplx_motion_trajectory_8d.npz")
    if os.path.exists(traj8d_path):
        print_section(f"8D Trajectory Variant: {traj8d_path}")
        print_npz_contents(traj8d_path)


if __name__ == "__main__":
    main()
