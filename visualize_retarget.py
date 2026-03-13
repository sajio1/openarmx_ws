#!/usr/bin/env python3
"""
Visualize OpenArm retargeted trajectory.

Modes:
  - meshcat: interactive 3D viewer
  - video:   export MP4 via matplotlib
  - both:    run meshcat, then export video
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import time
from pathlib import Path

import numpy as np
import pinocchio as pin


LEFT_JOINT_NAMES = [f"openarmx_left_joint{i}" for i in range(1, 8)]
RIGHT_JOINT_NAMES = [f"openarmx_right_joint{i}" for i in range(1, 8)]
LEFT_FINGER = "openarmx_left_finger_joint1"
RIGHT_FINGER = "openarmx_right_finger_joint1"

LEFT_LINKS = [f"openarmx_left_link{i}" for i in range(0, 8)]
RIGHT_LINKS = [f"openarmx_right_link{i}" for i in range(0, 8)]
DEBUG_LOG_PATH = "/home/sajio/vscode_robotic/openarmx_ws/.cursor/debug-50716b.log"
DEBUG_SESSION_ID = "50716b"
DEBUG_RUN_ID = f"viz_{int(time.time() * 1000)}"


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


def load_trajectory(npz_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    data = np.load(npz_path, allow_pickle=True)
    if "left_joints" not in data.files or "right_joints" not in data.files:
        raise KeyError("trajectory npz must contain left_joints and right_joints")
    left = data["left_joints"]
    right = data["right_joints"]
    left_gripper = data["left_gripper"] if "left_gripper" in data.files else np.zeros((left.shape[0], 1))
    right_gripper = data["right_gripper"] if "right_gripper" in data.files else np.zeros((right.shape[0], 1))
    fps = float(data["fps"][0]) if "fps" in data.files else 30.0
    return left, right, left_gripper, right_gripper, fps


def build_robot(urdf_path: Path, workspace_src: Path):
    os.environ["ROS_PACKAGE_PATH"] = str(workspace_src)
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        str(urdf_path),
        package_dirs=[str(workspace_src)],
    )
    data = model.createData()
    return model, data, collision_model, visual_model


def set_frame_q(
    model: pin.Model,
    q: np.ndarray,
    left_joints: np.ndarray,
    right_joints: np.ndarray,
    left_gripper: float,
    right_gripper: float,
) -> np.ndarray:
    for i, name in enumerate(LEFT_JOINT_NAMES):
        jid = model.getJointId(name)
        idx = model.joints[jid].idx_q
        q[idx] = left_joints[i]
    for i, name in enumerate(RIGHT_JOINT_NAMES):
        jid = model.getJointId(name)
        idx = model.joints[jid].idx_q
        q[idx] = right_joints[i]
    if LEFT_FINGER in model.names:
        jid = model.getJointId(LEFT_FINGER)
        q[model.joints[jid].idx_q] = left_gripper
    if RIGHT_FINGER in model.names:
        jid = model.getJointId(RIGHT_FINGER)
        q[model.joints[jid].idx_q] = right_gripper
    return q


def link_positions(model: pin.Model, data: pin.Data, q: np.ndarray, link_names: list[str]) -> np.ndarray:
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pts = []
    for name in link_names:
        fid = model.getFrameId(name)
        t = data.oMf[fid].translation
        pts.append(np.array([t[0], t[1], t[2]], dtype=np.float64))
    return np.asarray(pts)


def _frame_metrics(model: pin.Model, data: pin.Data, left: np.ndarray, right: np.ndarray, left_grip: float, right_grip: float):
    q = pin.neutral(model)
    q = set_frame_q(model, q, left, right, left_grip, right_grip)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    def pos(name: str):
        return data.oMf[model.getFrameId(name)].translation.copy()

    p_l_shoulder = pos("openarmx_left_link2")
    p_r_shoulder = pos("openarmx_right_link2")
    p_l_hand = pos("openarmx_left_link7")
    p_r_hand = pos("openarmx_right_link7")
    return {
        "left_hand_to_left_shoulder": float(np.linalg.norm(p_l_hand - p_l_shoulder)),
        "right_hand_to_right_shoulder": float(np.linalg.norm(p_r_hand - p_r_shoulder)),
        "left_hand_to_right_shoulder": float(np.linalg.norm(p_l_hand - p_r_shoulder)),
        "right_hand_to_left_shoulder": float(np.linalg.norm(p_r_hand - p_l_shoulder)),
        "left_hand": np.round(p_l_hand, 6).tolist(),
        "right_hand": np.round(p_r_hand, 6).tolist(),
    }


def _trajectory_motion_metrics(model: pin.Model, data: pin.Data, left: np.ndarray, right: np.ndarray, left_grip: np.ndarray, right_grip: np.ndarray):
    q = pin.neutral(model)
    l_xyz = []
    r_xyz = []
    for i in range(left.shape[0]):
        q = set_frame_q(model, q, left[i], right[i], float(left_grip[i, 0]), float(right_grip[i, 0]))
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        l_xyz.append(data.oMf[model.getFrameId("openarmx_left_link7")].translation.copy())
        r_xyz.append(data.oMf[model.getFrameId("openarmx_right_link7")].translation.copy())
    l_xyz = np.asarray(l_xyz)
    r_xyz = np.asarray(r_xyz)
    l_d = np.diff(l_xyz, axis=0)
    r_d = np.diff(r_xyz, axis=0)
    axis = ["x", "y", "z"]

    def corr(a: np.ndarray, b: np.ndarray) -> float:
        if np.std(a) < 1e-9 or np.std(b) < 1e-9:
            return 0.0
        return float(np.corrcoef(a, b)[0, 1])

    return {
        "left_disp_std_xyz": {axis[i]: float(np.std(l_d[:, i])) for i in range(3)},
        "right_disp_std_xyz": {axis[i]: float(np.std(r_d[:, i])) for i in range(3)},
        "left_dom_axis": axis[int(np.argmax(np.std(l_d, axis=0)))],
        "right_dom_axis": axis[int(np.argmax(np.std(r_d, axis=0)))],
        "lr_corr_xyz": {axis[i]: corr(l_d[:, i], r_d[:, i]) for i in range(3)},
        "lr_corr_neg_xyz": {axis[i]: corr(l_d[:, i], -r_d[:, i]) for i in range(3)},
    }


def play_meshcat(
    model: pin.Model,
    data: pin.Data,
    collision_model: pin.GeometryModel,
    visual_model: pin.GeometryModel,
    left: np.ndarray,
    right: np.ndarray,
    left_grip: np.ndarray,
    right_grip: np.ndarray,
    fps: float,
    once: bool,
    speed: float,
):
    try:
        from pinocchio.visualize import MeshcatVisualizer
    except Exception as exc:
        raise RuntimeError(f"Meshcat visualizer unavailable: {exc}") from exc

    # Robot visual from URDF package resources
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel("openarmx")

    q = pin.neutral(model)
    dt = 1.0 / max(fps * max(speed, 1e-6), 1.0)
    viewer_url = None
    if hasattr(viz, "viewer") and hasattr(viz.viewer, "url"):
        try:
            viewer_url = viz.viewer.url()
        except Exception:
            viewer_url = None
    if viewer_url:
        print(f"[meshcat] Viewer URL: {viewer_url}")
    print("[meshcat] Playing full robot mesh animation. Press Ctrl+C to stop.")
    while True:
        for i in range(left.shape[0]):
            q = set_frame_q(model, q, left[i], right[i], float(left_grip[i, 0]), float(right_grip[i, 0]))
            viz.display(q)
            time.sleep(dt)
        if once:
            break


def export_video(
    model: pin.Model,
    data: pin.Data,
    left: np.ndarray,
    right: np.ndarray,
    left_grip: np.ndarray,
    right_grip: np.ndarray,
    fps: float,
    output_path: Path,
):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter, PillowWriter

    q = pin.neutral(model)
    n = left.shape[0]
    fps = max(fps, 1.0)

    fig = plt.figure(figsize=(13, 8))
    ax3d = fig.add_subplot(121, projection="3d")
    axj = fig.add_subplot(222)
    axg = fig.add_subplot(224)

    use_ffmpeg = shutil.which("ffmpeg") is not None
    if use_ffmpeg:
        writer = FFMpegWriter(fps=int(round(fps)))
    else:
        writer = PillowWriter(fps=int(round(fps)))
        output_path = output_path.with_suffix(".gif")
        print("[warn] ffmpeg not found, fallback to GIF export.")

    # Precompute for stable axis limits
    all_pts = []
    for i in range(n):
        q = set_frame_q(model, q, left[i], right[i], float(left_grip[i, 0]), float(right_grip[i, 0]))
        lpts = link_positions(model, data, q, LEFT_LINKS)
        rpts = link_positions(model, data, q, RIGHT_LINKS)
        all_pts.append(np.vstack([lpts, rpts]))
    all_pts = np.vstack(all_pts)
    xyz_min = all_pts.min(axis=0) - 0.05
    xyz_max = all_pts.max(axis=0) + 0.05

    with writer.saving(fig, str(output_path), dpi=100):
        for i in range(n):
            ax3d.cla()
            axj.cla()
            axg.cla()

            q = set_frame_q(model, q, left[i], right[i], float(left_grip[i, 0]), float(right_grip[i, 0]))
            lpts = link_positions(model, data, q, LEFT_LINKS)
            rpts = link_positions(model, data, q, RIGHT_LINKS)

            ax3d.plot(lpts[:, 0], lpts[:, 1], lpts[:, 2], "c-o", linewidth=2, markersize=3, label="left_arm")
            ax3d.plot(rpts[:, 0], rpts[:, 1], rpts[:, 2], "m-o", linewidth=2, markersize=3, label="right_arm")
            ax3d.set_xlim(xyz_min[0], xyz_max[0])
            ax3d.set_ylim(xyz_min[1], xyz_max[1])
            ax3d.set_zlim(xyz_min[2], xyz_max[2])
            ax3d.set_xlabel("X")
            ax3d.set_ylabel("Y")
            ax3d.set_zlabel("Z")
            ax3d.set_title(f"OpenArm Retarget  frame={i+1}/{n}")
            ax3d.legend(loc="upper left")

            t = np.arange(i + 1) / fps
            axj.plot(t, left[: i + 1, 3], "c-", label="left_joint4(elbow)")
            axj.plot(t, right[: i + 1, 3], "m-", label="right_joint4(elbow)")
            axj.set_xlabel("Time (s)")
            axj.set_ylabel("Angle (rad)")
            axj.set_title("Elbow Trajectory")
            axj.grid(True, alpha=0.3)
            axj.legend(loc="upper right")

            axg.plot(t, left_grip[: i + 1, 0], "c--", label="left_gripper")
            axg.plot(t, right_grip[: i + 1, 0], "m--", label="right_gripper")
            axg.set_xlabel("Time (s)")
            axg.set_ylabel("Open (m)")
            axg.set_title("Gripper")
            axg.grid(True, alpha=0.3)
            axg.legend(loc="upper right")

            fig.tight_layout()
            writer.grab_frame()
            if (i + 1) % max(int(fps), 1) == 0 or i == n - 1:
                print(f"[video] frame {i+1}/{n}")

    if use_ffmpeg:
        print(f"[OK] Video saved: {output_path}")
    else:
        print(f"[OK] GIF saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Visualize retargeted OpenArm trajectory.")
    parser.add_argument("trajectory_npz", type=Path, help="Path to retargeted trajectory npz")
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path("src/openarmx_description/urdf/robot/openarmx_bimanual_sim.urdf"),
        help="OpenArm URDF path",
    )
    parser.add_argument(
        "--mode",
        choices=["meshcat", "video", "both"],
        default="meshcat",
        help="Visualization mode",
    )
    parser.add_argument("--once", action="store_true", help="For meshcat mode: play one cycle and exit")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier for meshcat")
    parser.add_argument("--output-video", type=Path, default=Path("retarget_openarmx.mp4"), help="Video path (or GIF fallback)")
    args = parser.parse_args()

    workspace_src = Path("src").resolve()
    left, right, left_grip, right_grip, fps = load_trajectory(args.trajectory_npz)
    model, data, collision_model, visual_model = build_robot(args.urdf.resolve(), workspace_src)
    # region agent log
    base_metrics = _frame_metrics(model, data, left[0], right[0], float(left_grip[0, 0]), float(right_grip[0, 0]))
    cand_right_j1 = right[0].copy()
    cand_right_j1[0] += np.pi / 2.0
    m_j1 = _frame_metrics(model, data, left[0], cand_right_j1, float(left_grip[0, 0]), float(right_grip[0, 0]))
    cand_right_j2 = right[0].copy()
    cand_right_j2[1] = -cand_right_j2[1]
    m_j2 = _frame_metrics(model, data, left[0], cand_right_j2, float(left_grip[0, 0]), float(right_grip[0, 0]))
    cand_both = right[0].copy()
    cand_both[0] += np.pi / 2.0
    cand_both[1] = -cand_both[1]
    m_both = _frame_metrics(model, data, left[0], cand_both, float(left_grip[0, 0]), float(right_grip[0, 0]))
    _debug_log(
        "H9",
        "visualize_retarget.py:main",
        "frame0_distance_candidates",
        {
            "trajectory_npz": str(args.trajectory_npz),
            "base": base_metrics,
            "cand_right_j1_plus90": m_j1,
            "cand_right_j2_sign_flip": m_j2,
            "cand_j1_plus90_j2_flip": m_both,
            "right_frame0_base": np.round(right[0], 6).tolist(),
            "right_frame0_cand_j1_plus90": np.round(cand_right_j1, 6).tolist(),
            "right_frame0_cand_j2_flip": np.round(cand_right_j2, 6).tolist(),
            "right_frame0_cand_both": np.round(cand_both, 6).tolist(),
        },
    )
    # endregion
    # region agent log
    motion_metrics = _trajectory_motion_metrics(model, data, left, right, left_grip, right_grip)
    _debug_log(
        "H10",
        "visualize_retarget.py:main",
        "trajectory_motion_axis_metrics",
        {
            "trajectory_npz": str(args.trajectory_npz),
            "metrics": motion_metrics,
        },
    )
    # region agent log
    def _variant_metrics(left_v: np.ndarray, right_v: np.ndarray):
        return _trajectory_motion_metrics(model, data, left_v, right_v, left_grip, right_grip)

    left_a = left.copy()
    right_a = right.copy()
    left_a[:, [0, 2]] = left_a[:, [2, 0]]
    right_a[:, [0, 2]] = right_a[:, [2, 0]]

    left_b = left.copy()
    right_b = right.copy()
    left_b[:, 0] = -left_b[:, 0]
    right_b[:, 0] = -right_b[:, 0]

    left_c = left.copy()
    right_c = right.copy()
    left_c[:, 2] = -left_c[:, 2]
    right_c[:, 2] = -right_c[:, 2]

    left_d = left.copy()
    right_d = right.copy()
    left_d[:, 0] = left_d[:, 2]
    left_d[:, 2] = -left[:, 0]
    right_d[:, 0] = right_d[:, 2]
    right_d[:, 2] = -right[:, 0]

    _debug_log(
        "H11",
        "visualize_retarget.py:main",
        "trajectory_variant_axis_metrics",
        {
            "base": motion_metrics,
            "swap_j1_j3": _variant_metrics(left_a, right_a),
            "negate_j1": _variant_metrics(left_b, right_b),
            "negate_j3": _variant_metrics(left_c, right_c),
            "j1_from_j3_j3_from_neg_j1": _variant_metrics(left_d, right_d),
        },
    )
    # endregion

    if args.mode in ("video", "both"):
        export_video(model, data, left, right, left_grip, right_grip, fps, args.output_video)

    if args.mode in ("meshcat", "both"):
        play_meshcat(
            model,
            data,
            collision_model,
            visual_model,
            left,
            right,
            left_grip,
            right_grip,
            fps,
            once=args.once,
            speed=args.speed,
        )


if __name__ == "__main__":
    main()
