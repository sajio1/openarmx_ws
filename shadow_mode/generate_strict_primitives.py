#!/usr/bin/env python3
"""生成严格版动作原语（数学模板）.

输出:
  - shadow_mode/analysis_outputs/primitives_dataset_strict.json
  - shadow_mode/analysis_outputs/primitives_strict_overview.png

规则:
  1) 平移 +X/+Y/+Z: 单轴三角波，其余两轴恒为0
  2) 旋转 roll/pitch/yaw: 单轴 sin(pi*t/T)，其余两轴恒为0
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

OUT_DIR = Path("/home/ok/Desktop/openarmx_ws/shadow_mode/analysis_outputs")
OUT_DIR.mkdir(parents=True, exist_ok=True)


def triangle_wave(t: np.ndarray, amp: float, T: float) -> np.ndarray:
    """0->amp->0 三角波."""
    half = T / 2.0
    y = np.zeros_like(t)
    rise = t <= half
    fall = ~rise
    y[rise] = amp * (t[rise] / half)
    y[fall] = amp * (1.0 - (t[fall] - half) / half)
    return y


def sin_pi_wave(t: np.ndarray, amp: float, T: float) -> np.ndarray:
    """amp*sin(pi*t/T), 0->amp->0."""
    return amp * np.sin(np.pi * t / T)


def build_translation_primitive(axis: int, name: str, amp: float, T: float, hz: int) -> dict:
    n = int(T * hz) + 1
    t = np.linspace(0.0, T, n, dtype=np.float64)
    xyz = np.zeros((n, 3), dtype=np.float64)
    xyz[:, axis] = triangle_wave(t, amp, T)
    return {
        "name": name,
        "t": t.round(6).tolist(),
        "xyz": xyz.round(6).tolist(),
        "amplitude_m": float(amp),
    }


def build_rotation_primitive(axis: int, name: str, amp: float, T: float, hz: int) -> dict:
    n = int(T * hz) + 1
    t = np.linspace(0.0, T, n, dtype=np.float64)
    rpy = np.zeros((n, 3), dtype=np.float64)
    rpy[:, axis] = sin_pi_wave(t, amp, T)
    return {
        "name": name,
        "t": t.round(6).tolist(),
        "rpy_rad": rpy.round(6).tolist(),
        "amplitude_rad": float(amp),
    }


def save_plot(dataset: dict, out_png: Path):
    # 延迟导入，避免无 matplotlib 环境时影响 json 生成
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(3, 3, figsize=(15, 10))
    axs = axs.flatten()

    # clutch schematic
    ax = axs[0]
    t = np.array([0.0, 0.6, 0.6, 1.2, 1.2, 1.8, 1.8, 2.4])
    y = np.array([0, 0, 1, 1, 0, 0, 1, 1])
    ax.step(t, y, where="post")
    ax.set_yticks([0, 1], ["OFF", "ON"])
    ax.set_title("clutch_events_strict")
    ax.set_xlabel("time(s)")
    ax.grid(alpha=0.25)

    def plot_xyz(ax, d, title):
        tt = d["t"]
        xyz = np.array(d["xyz"], dtype=np.float64)
        ax.plot(tt, xyz[:, 0], label="x")
        ax.plot(tt, xyz[:, 1], label="y")
        ax.plot(tt, xyz[:, 2], label="z")
        ax.set_title(title)
        ax.set_xlabel("time(s)")
        ax.set_ylabel("delta(m)")
        ax.grid(alpha=0.25)

    def plot_rpy(ax, d, title):
        tt = d["t"]
        rpy = np.array(d["rpy_rad"], dtype=np.float64)
        ax.plot(tt, rpy[:, 0], label="roll")
        ax.plot(tt, rpy[:, 1], label="pitch")
        ax.plot(tt, rpy[:, 2], label="yaw")
        ax.set_title(title)
        ax.set_xlabel("time(s)")
        ax.set_ylabel("angle(rad)")
        ax.grid(alpha=0.25)

    rt = dataset["translation"]["right"]
    lt = dataset["translation"]["left"]
    rr = dataset["rotation"]["right"]
    lr = dataset["rotation"]["left"]

    plot_xyz(axs[1], rt["+x"], "right +x strict")
    plot_xyz(axs[2], rt["+y"], "right +y strict")
    plot_xyz(axs[3], rt["+z"], "right +z strict")
    plot_xyz(axs[4], lt["+x"], "left +x strict")
    plot_xyz(axs[5], lt["+y"], "left +y strict")
    plot_xyz(axs[6], lt["+z"], "left +z strict")
    plot_rpy(axs[7], rr["roll"], "right roll strict")

    ax = axs[8]
    for key in ("pitch", "yaw"):
        d = rr[key]
        tt = d["t"]
        rpy = np.array(d["rpy_rad"], dtype=np.float64)
        idx = 1 if key == "pitch" else 2
        ax.plot(tt, rpy[:, idx], label=key)
    ax.set_title("right pitch/yaw strict")
    ax.set_xlabel("time(s)")
    ax.set_ylabel("angle(rad)")
    ax.grid(alpha=0.25)

    for i in (1, 7, 8):
        axs[i].legend(fontsize=8)

    plt.tight_layout()
    plt.savefig(out_png, dpi=170)


def main():
    hz = 90
    T_pos = 1.2
    T_rot = 1.2
    A_pos = 0.12  # m
    A_rot = 0.35  # rad

    dataset = {
        "meta": {
            "type": "strict_math_templates",
            "translation_waveform": "triangle",
            "rotation_waveform": "sin(pi*t/T)",
            "hz": hz,
            "duration_pos_s": T_pos,
            "duration_rot_s": T_rot,
            "amplitude_pos_m": A_pos,
            "amplitude_rot_rad": A_rot,
        },
        "clutch": {
            "pattern": ["OFF", "ON", "OFF", "ON"],
            "note": "示意，实际回放可按用例定义"
        },
        "translation": {"right": {}, "left": {}},
        "rotation": {"right": {}, "left": {}},
    }

    # translation (+X/+Y/+Z)
    axes = {"+x": 0, "+y": 1, "+z": 2}
    for side in ("right", "left"):
        for name, idx in axes.items():
            dataset["translation"][side][name] = build_translation_primitive(
                axis=idx,
                name=f"{side}_{name}_strict",
                amp=A_pos,
                T=T_pos,
                hz=hz,
            )

    # rotation (roll/pitch/yaw)
    raxes = {"roll": 0, "pitch": 1, "yaw": 2}
    for side in ("right", "left"):
        for name, idx in raxes.items():
            dataset["rotation"][side][name] = build_rotation_primitive(
                axis=idx,
                name=f"{side}_{name}_strict",
                amp=A_rot,
                T=T_rot,
                hz=hz,
            )

    out_json = OUT_DIR / "primitives_dataset_strict.json"
    out_json.write_text(json.dumps(dataset, ensure_ascii=False, indent=2), encoding="utf-8")

    out_png = OUT_DIR / "primitives_strict_overview.png"
    try:
        save_plot(dataset, out_png)
        print(out_png)
    except Exception as e:
        print(f"plot_error: {e}")
    print(out_json)


if __name__ == "__main__":
    main()
