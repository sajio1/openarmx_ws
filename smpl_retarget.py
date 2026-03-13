#!/usr/bin/env python3
"""
Direct SMPL-X arm retargeting to OpenArm (no IK).

Input:
  - smplx_motion.npz (raw SMPL-X)
Output:
  - OpenArm trajectory npz with:
      left_joints (N, 7), right_joints (N, 7),
      left_gripper (N, 1), right_gripper (N, 1),
      left_cmd (N, 8), right_cmd (N, 8), fps, source
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as R


# SMPL-X body joint indices in pose_body (21 * 3)
ARM_IDX = {
    "left": {"collar": 12, "shoulder": 15, "elbow": 17, "wrist": 19},
    "right": {"collar": 13, "shoulder": 16, "elbow": 18, "wrist": 20},
}


# Base per-joint limits from config.
BASE_LIMITS = np.array(
    [
        [-1.25, 3.0],   # joint1
        [-1.70, 1.70],  # joint2
        [-1.57, 1.57],  # joint3
        [0.0, 1.80],    # joint4
        [-1.50, 1.50],  # joint5
        [-0.75, 0.75],  # joint6
        [-1.40, 1.40],  # joint7
    ],
    dtype=np.float64,
)


# Bimanual URDF offsets from xacro.
BIMANUAL_OFFSET = {
    "left": np.array([-2.094396, -np.pi / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64),
    "right": np.array([0.0, np.pi / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64),
}

# Shoulder-frame coarse Z alignment (code-level tuning, no config change).
# Derived from runtime comparison against available reference trajectories.
SHOULDER_ALIGN_Z_DEG = {"left": 75.0, "right": -75.0}

DEBUG_LOG_PATH = "/home/sajio/vscode_robotic/openarmx_ws/.cursor/debug-50716b.log"
DEBUG_SESSION_ID = "50716b"
DEBUG_RUN_ID = f"run_{int(time.time() * 1000)}"


def _debug_log(hypothesis_id: str, location: str, message: str, data: dict) -> None:
    payload = {
        "sessionId": DEBUG_SESSION_ID,
        "runId": DEBUG_RUN_ID,
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    with open(DEBUG_LOG_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=True) + "\n")


def _rot_angle_deg(rot: R) -> float:
    return float(np.linalg.norm(rot.as_rotvec()) * 180.0 / np.pi)


def _joint_vec(pose_body: np.ndarray, idx: int) -> np.ndarray:
    return pose_body[:, idx * 3 : (idx + 1) * 3]


def _smooth(x: np.ndarray, window: int, poly: int) -> np.ndarray:
    if x.shape[0] < 5 or window < 5:
        return x
    win = int(window)
    if win % 2 == 0:
        win += 1
    win = min(win, x.shape[0] if x.shape[0] % 2 == 1 else x.shape[0] - 1)
    if win < 5:
        return x
    return savgol_filter(x, window_length=win, polyorder=min(poly, win - 2), axis=0, mode="interp")


def _decompose_shoulder(
    collar_aa: np.ndarray, shoulder_aa: np.ndarray, side: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Shoulder chain mapping:
      SMPL collar+shoulder rotation -> OpenArm joint1/2/3
      OpenArm chain is Z -(-X)- Z, so q2 = -middle_angle_of_ZXZ.
    """
    n = collar_aa.shape[0]
    q1 = np.zeros(n)
    q2 = np.zeros(n)
    q3 = np.zeros(n)
    rz_align = np.deg2rad(SHOULDER_ALIGN_Z_DEG[side])
    r_align = R.from_euler("z", rz_align, degrees=False)
    for i in range(n):
        r_collar = R.from_rotvec(collar_aa[i])
        r_shoulder = R.from_rotvec(shoulder_aa[i])
        r_total = r_align * (r_collar * r_shoulder)
        a, b, c = r_total.as_euler("ZXZ", degrees=False)
        q1[i] = a
        q2[i] = -b if side == "left" else b
        q3[i] = c
    return q1, q2, q3


def retarget_side(pose_body: np.ndarray, side: str) -> np.ndarray:
    idx = ARM_IDX[side]
    collar = _joint_vec(pose_body, idx["collar"])
    shoulder = _joint_vec(pose_body, idx["shoulder"])
    elbow = _joint_vec(pose_body, idx["elbow"])
    wrist = _joint_vec(pose_body, idx["wrist"])
    # region agent log
    _debug_log(
        "H1",
        "smpl_retarget.py:retarget_side",
        "input_arm_vectors_frame0",
        {
            "side": side,
            "collar0": np.round(collar[0], 6).tolist(),
            "shoulder0": np.round(shoulder[0], 6).tolist(),
            "elbow0": np.round(elbow[0], 6).tolist(),
            "wrist0": np.round(wrist[0], 6).tolist(),
        },
    )
    # endregion

    # joint1..3 from shoulder complex
    j1, j2, j3 = _decompose_shoulder(collar, shoulder, side)

    # joint4..7 direct from elbow/wrist components (no IK).
    # Make elbow flexion positive for both sides.
    if side == "left":
        j4 = -elbow[:, 1]
    else:
        j4 = elbow[:, 1]
    j5 = elbow[:, 2]
    j6 = wrist[:, 0]
    # joint7 axis differs by side in URDF (+/-Y), compensate with sign.
    j7 = wrist[:, 1] * (-1.0 if side == "left" else 1.0)
    # region agent log
    _debug_log(
        "H2",
        "smpl_retarget.py:retarget_side",
        "mapped_arm_joints_pre_stack",
        {
            "side": side,
            "frame0": np.round([j1[0], j2[0], j3[0], j4[0], j5[0], j6[0], j7[0]], 6).tolist(),
            "j1_range": [float(np.min(j1)), float(np.max(j1))],
            "j2_range": [float(np.min(j2)), float(np.max(j2))],
            "j3_range": [float(np.min(j3)), float(np.max(j3))],
        },
    )
    # endregion

    out = np.stack([j1, j2, j3, j4, j5, j6, j7], axis=1)
    return out


def clip_to_limits(joints: np.ndarray, limits: np.ndarray) -> np.ndarray:
    low = limits[:, 0][None, :]
    high = limits[:, 1][None, :]
    return np.minimum(np.maximum(joints, low), high)


def side_limits(side: str, apply_offsets: bool) -> np.ndarray:
    lim = BASE_LIMITS.copy()
    if not apply_offsets:
        return lim
    offset = BIMANUAL_OFFSET[side]
    lim[:, 0] += offset
    lim[:, 1] += offset
    return lim


def main() -> None:
    parser = argparse.ArgumentParser(description="Retarget SMPL-X arm motion to OpenArm (no IK).")
    parser.add_argument("input_npz", type=Path, help="Path to smplx_motion.npz")
    parser.add_argument("--output", type=Path, default=None, help="Output npz path")
    parser.add_argument(
        "--apply-offsets",
        action="store_true",
        help="Apply bimanual joint offsets (normally keep this off for trajectory output).",
    )
    parser.add_argument("--smooth-window", type=int, default=9, help="Savitzky-Golay window (odd preferred)")
    parser.add_argument("--smooth-poly", type=int, default=2, help="Savitzky-Golay polynomial order")
    args = parser.parse_args()
    # region agent log
    _debug_log(
        "H5",
        "smpl_retarget.py:main",
        "runtime_args",
        {
            "input_npz": str(args.input_npz),
            "output": str(args.output) if args.output is not None else "default",
            "apply_offsets": bool(args.apply_offsets),
            "smooth_window": int(args.smooth_window),
            "smooth_poly": int(args.smooth_poly),
            "shoulder_align_z_deg": SHOULDER_ALIGN_Z_DEG,
        },
    )
    # endregion

    data = np.load(args.input_npz, allow_pickle=True)
    if "pose_body" not in data.files:
        raise KeyError("input npz has no 'pose_body' key")

    pose_body = data["pose_body"]
    n = pose_body.shape[0]
    fps = float(data["mocap_frame_rate"][0]) if "mocap_frame_rate" in data.files else 30.0
    apply_offsets = args.apply_offsets
    # region agent log
    if "root_orient" in data.files:
        root = R.from_rotvec(data["root_orient"][0])
        spine1 = R.from_rotvec(pose_body[0, 2 * 3 : 3 * 3])
        spine2 = R.from_rotvec(pose_body[0, 5 * 3 : 6 * 3])
        spine3 = R.from_rotvec(pose_body[0, 8 * 3 : 9 * 3])
        torso = root * spine1 * spine2 * spine3

        lc = R.from_rotvec(pose_body[0, 12 * 3 : 13 * 3])
        ls = R.from_rotvec(pose_body[0, 15 * 3 : 16 * 3])
        rc = R.from_rotvec(pose_body[0, 13 * 3 : 14 * 3])
        rs = R.from_rotvec(pose_body[0, 16 * 3 : 17 * 3])

        left_local = lc * ls
        right_local = rc * rs
        left_full = torso * left_local
        right_full = torso * right_local
        left_delta = left_full * left_local.inv()
        right_delta = right_full * right_local.inv()

        _debug_log(
            "H6",
            "smpl_retarget.py:main",
            "torso_chain_influence_frame0",
            {
                "torso_rotvec": np.round(torso.as_rotvec(), 6).tolist(),
                "left_local_rotvec": np.round(left_local.as_rotvec(), 6).tolist(),
                "left_full_rotvec": np.round(left_full.as_rotvec(), 6).tolist(),
                "left_delta_deg": _rot_angle_deg(left_delta),
                "right_local_rotvec": np.round(right_local.as_rotvec(), 6).tolist(),
                "right_full_rotvec": np.round(right_full.as_rotvec(), 6).tolist(),
                "right_delta_deg": _rot_angle_deg(right_delta),
            },
        )
    # endregion

    left = retarget_side(pose_body, "left")
    right = retarget_side(pose_body, "right")
    left_pre = left.copy()
    right_pre = right.copy()
    # region agent log
    _debug_log(
        "H1",
        "smpl_retarget.py:main",
        "pre_offset_frame0_and_ranges",
        {
            "left_frame0": np.round(left_pre[0], 6).tolist(),
            "right_frame0": np.round(right_pre[0], 6).tolist(),
            "left_j123_range": [
                [float(np.min(left_pre[:, 0])), float(np.max(left_pre[:, 0]))],
                [float(np.min(left_pre[:, 1])), float(np.max(left_pre[:, 1]))],
                [float(np.min(left_pre[:, 2])), float(np.max(left_pre[:, 2]))],
            ],
            "right_j123_range": [
                [float(np.min(right_pre[:, 0])), float(np.max(right_pre[:, 0]))],
                [float(np.min(right_pre[:, 1])), float(np.max(right_pre[:, 1]))],
                [float(np.min(right_pre[:, 2])), float(np.max(right_pre[:, 2]))],
            ],
        },
    )
    # endregion

    if apply_offsets:
        left += BIMANUAL_OFFSET["left"][None, :]
        right += BIMANUAL_OFFSET["right"][None, :]
    # region agent log
    _debug_log(
        "H5",
        "smpl_retarget.py:main",
        "post_offset_frame0",
        {
            "apply_offsets": bool(apply_offsets),
            "left_frame0": np.round(left[0], 6).tolist(),
            "right_frame0": np.round(right[0], 6).tolist(),
        },
    )
    # endregion

    left_before_smooth = left.copy()
    right_before_smooth = right.copy()
    left = _smooth(left, args.smooth_window, args.smooth_poly)
    right = _smooth(right, args.smooth_window, args.smooth_poly)
    # region agent log
    _debug_log(
        "H4",
        "smpl_retarget.py:main",
        "smooth_delta_frame0",
        {
            "left_delta_frame0": np.round((left[0] - left_before_smooth[0]), 6).tolist(),
            "right_delta_frame0": np.round((right[0] - right_before_smooth[0]), 6).tolist(),
        },
    )
    # endregion

    left_before_clip = left.copy()
    right_before_clip = right.copy()
    left = clip_to_limits(left, side_limits("left", apply_offsets))
    right = clip_to_limits(right, side_limits("right", apply_offsets))
    left_clip_count = int(np.count_nonzero(np.abs(left - left_before_clip) > 1e-9))
    right_clip_count = int(np.count_nonzero(np.abs(right - right_before_clip) > 1e-9))
    # region agent log
    _debug_log(
        "H3",
        "smpl_retarget.py:main",
        "clip_effect",
        {
            "left_clip_count": left_clip_count,
            "right_clip_count": right_clip_count,
            "left_frame0_after_clip": np.round(left[0], 6).tolist(),
            "right_frame0_after_clip": np.round(right[0], 6).tolist(),
        },
    )
    # endregion
    # region agent log
    alt_left = left.copy()
    alt_right = right.copy()
    alt_left[:, 0] = ((alt_left[:, 0] + np.pi) % (2.0 * np.pi)) - np.pi
    alt_right[:, 0] = ((alt_right[:, 0] + np.pi) % (2.0 * np.pi)) - np.pi
    _debug_log(
        "H8",
        "smpl_retarget.py:main",
        "euler_branch_frame0",
        {
            "left_j1_frame0": float(left[0, 0]),
            "left_j1_alt_frame0": float(alt_left[0, 0]),
            "right_j1_frame0": float(right[0, 0]),
            "right_j1_alt_frame0": float(alt_right[0, 0]),
        },
    )
    # endregion

    left_gripper = np.zeros((n, 1), dtype=np.float64)
    right_gripper = np.zeros((n, 1), dtype=np.float64)
    left_cmd = np.concatenate([left, left_gripper], axis=1)
    right_cmd = np.concatenate([right, right_gripper], axis=1)

    if args.output is None:
        args.output = args.input_npz.parent / "smplx_motion_trajectory_openarmx.npz"

    np.savez(
        args.output,
        left_joints=left,
        right_joints=right,
        left_gripper=left_gripper,
        right_gripper=right_gripper,
        left_cmd=left_cmd,
        right_cmd=right_cmd,
        fps=np.array([fps], dtype=np.float64),
        source=np.array([str(args.input_npz)], dtype="<U256"),
    )

    print(f"[OK] Saved retargeted trajectory: {args.output}")
    print(f"     frames={n}, fps={fps:.2f}, apply_offsets={apply_offsets}")
    for side_name, joints in (("left", left), ("right", right)):
        lim = side_limits(side_name, apply_offsets)
        print(f"  {side_name}_joints ranges:")
        for j in range(7):
            vmin = float(joints[:, j].min())
            vmax = float(joints[:, j].max())
            print(
                f"    joint{j+1}: [{vmin: .4f}, {vmax: .4f}] "
                f"limit [{lim[j,0]: .4f}, {lim[j,1]: .4f}]"
            )


if __name__ == "__main__":
    main()
