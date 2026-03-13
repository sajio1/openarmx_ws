#!/usr/bin/env python3
"""把 strict 原语数据拼装成可回放的 quest jsonl."""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

IN_PATH = Path("/home/ok/Desktop/openarmx_ws/shadow_mode/analysis_outputs/primitives_dataset_strict.json")
OUT_PATH = Path("/home/ok/Desktop/openarmx_ws/shadow_mode/analysis_outputs/strict_replay_sequence.jsonl")
HZ = 90.0


def pose_msg(x: float, y: float, z: float) -> dict:
    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }


def emit_line(fp, t, topic, msg):
    fp.write(json.dumps({"t": t, "topic": topic, "msg": msg}, ensure_ascii=True) + "\n")


def emit_pose_block(fp, t0: float, left_xyz: np.ndarray, right_xyz: np.ndarray):
    n = min(len(left_xyz), len(right_xyz))
    for i in range(n):
        t = t0 + i / HZ
        emit_line(fp, t, "/quest3/left_hand_pose", pose_msg(*left_xyz[i].tolist()))
        emit_line(fp, t, "/quest3/right_hand_pose", pose_msg(*right_xyz[i].tolist()))
        emit_line(fp, t, "/quest3/left_gripper", {"data": 1.0})
        emit_line(fp, t, "/quest3/right_gripper", {"data": 1.0})
    return t0 + (n - 1) / HZ if n > 0 else t0


def main():
    obj = json.loads(IN_PATH.read_text(encoding="utf-8"))

    # 基准手位（胸前控制姿态）: 仅用于构造输入，不是机器人绝对位置
    left_base = np.array([0.25, 0.15, 0.20], dtype=np.float64)
    right_base = np.array([0.25, -0.15, 0.20], dtype=np.float64)

    # 平移原语（只用 xyz）
    tx_r = np.array(obj["translation"]["right"]["+x"]["xyz"], dtype=np.float64)
    ty_r = np.array(obj["translation"]["right"]["+y"]["xyz"], dtype=np.float64)
    tz_r = np.array(obj["translation"]["right"]["+z"]["xyz"], dtype=np.float64)
    tx_l = np.array(obj["translation"]["left"]["+x"]["xyz"], dtype=np.float64)
    ty_l = np.array(obj["translation"]["left"]["+y"]["xyz"], dtype=np.float64)
    tz_l = np.array(obj["translation"]["left"]["+z"]["xyz"], dtype=np.float64)

    with OUT_PATH.open("w", encoding="utf-8") as fp:
        t = time.time()

        # Case A: static + clutch on/off
        emit_line(fp, t, "/quest3/clutch", {"data": False})
        for _ in range(int(1.0 * HZ)):
            t += 1.0 / HZ
            emit_line(fp, t, "/quest3/left_hand_pose", pose_msg(*left_base.tolist()))
            emit_line(fp, t, "/quest3/right_hand_pose", pose_msg(*right_base.tolist()))
            emit_line(fp, t, "/quest3/left_gripper", {"data": 1.0})
            emit_line(fp, t, "/quest3/right_gripper", {"data": 1.0})

        emit_line(fp, t + 1.0 / HZ, "/quest3/clutch", {"data": True})
        t += 1.0 / HZ

        # Case B: right +x/+y/+z
        for block in (tx_r, ty_r, tz_r):
            left = np.repeat(left_base[None, :], len(block), axis=0)
            right = right_base[None, :] + block
            t = emit_pose_block(fp, t, left, right) + 1.0 / HZ

        # Case C: left +x/+y/+z
        for block in (tx_l, ty_l, tz_l):
            left = left_base[None, :] + block
            right = np.repeat(right_base[None, :], len(block), axis=0)
            t = emit_pose_block(fp, t, left, right) + 1.0 / HZ

        emit_line(fp, t + 1.0 / HZ, "/quest3/clutch", {"data": False})

    print(OUT_PATH)


if __name__ == "__main__":
    main()
