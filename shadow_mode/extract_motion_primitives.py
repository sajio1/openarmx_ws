#!/usr/bin/env python3
"""从已录制 case 中提取高置信度动作原语.

输出:
  - shadow_mode/analysis_outputs/primitives_dataset.json
  - shadow_mode/analysis_outputs/primitives_summary.md

说明:
  - clutch: 来自 case4
  - 左右 +X/+Y/+Z: 分别来自 case2/3
  - 右手 roll/pitch/yaw: 来自 case5 四元数时序
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

CASES = {
    "case2_right_xyz": Path("/tmp/case2_right_xyz.jsonl"),
    "case3_left_xyz": Path("/tmp/case3_left_xyz.jsonl"),
    "case4_clutch_edges": Path("/tmp/case4_clutch_edges.jsonl"),
    "case5_right_rot": Path("/tmp/case5_right_rot.jsonl"),
}

OUT_DIR = Path("/home/ok/Desktop/openarmx_ws/shadow_mode/analysis_outputs")
OUT_DIR.mkdir(parents=True, exist_ok=True)


@dataclass
class PoseSeries:
    t: np.ndarray
    p: np.ndarray
    q: np.ndarray | None = None


def load_rows(path: Path) -> list[dict]:
    rows = []
    with path.open("r", encoding="utf-8") as fp:
        for ln in fp:
            ln = ln.strip()
            if not ln:
                continue
            rows.append(json.loads(ln))
    return rows


def clutch_segments(rows: list[dict]) -> list[tuple[float, float]]:
    clutch = [r for r in rows if r.get("topic") == "/quest3/clutch"]
    clutch.sort(key=lambda r: float(r["t"]))
    if not clutch:
        return []
    segs = []
    on_t = None
    prev = bool(clutch[0]["msg"]["data"])
    if prev:
        on_t = float(clutch[0]["t"])
    for r in clutch[1:]:
        v = bool(r["msg"]["data"])
        t = float(r["t"])
        if (not prev) and v:
            on_t = t
        elif prev and (not v) and on_t is not None:
            segs.append((on_t, t))
            on_t = None
        prev = v
    if on_t is not None:
        segs.append((on_t, float(clutch[-1]["t"])))
    return segs


def extract_pose(rows: list[dict], topic: str) -> PoseSeries | None:
    rec = [r for r in rows if r.get("topic") == topic]
    if len(rec) < 2:
        return None
    t = np.array([float(r["t"]) for r in rec], dtype=np.float64)
    p = np.array(
        [[float(r["msg"]["position"]["x"]), float(r["msg"]["position"]["y"]), float(r["msg"]["position"]["z"])] for r in rec],
        dtype=np.float64,
    )
    q = np.array(
        [[float(r["msg"]["orientation"]["x"]), float(r["msg"]["orientation"]["y"]), float(r["msg"]["orientation"]["z"]), float(r["msg"]["orientation"]["w"])] for r in rec],
        dtype=np.float64,
    )
    return PoseSeries(t=t, p=p, q=q)


def crop(series: PoseSeries, t0: float, t1: float) -> PoseSeries | None:
    m = (series.t >= t0) & (series.t <= t1)
    if m.sum() < 2:
        return None
    q = series.q[m] if series.q is not None else None
    return PoseSeries(t=series.t[m], p=series.p[m], q=q)


def ma(x: np.ndarray, k: int = 7) -> np.ndarray:
    if k <= 1 or len(x) < k:
        return x.copy()
    pad = k // 2
    xp = np.pad(x, ((pad, pad), (0, 0)), mode="edge")
    ker = np.ones(k, dtype=np.float64) / k
    y = np.zeros_like(x)
    for i in range(x.shape[1]):
        y[:, i] = np.convolve(xp[:, i], ker, mode="valid")
    return y


def normalize_segment(t: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    t0 = t[0]
    vn = v - v[0]
    return t - t0, vn


def extract_axis_primitive(series: PoseSeries, axis_idx: int, label: str, window_s: float = 1.3) -> dict:
    t = series.t
    p = ma(series.p, 7)
    v = p[:, axis_idx] - p[0, axis_idx]
    peak_i = int(np.argmax(v))
    peak_t = t[peak_i]
    t0 = peak_t - window_s / 2.0
    t1 = peak_t + window_s / 2.0
    m = (t >= t0) & (t <= t1)
    tt, vv = normalize_segment(t[m], p[m])
    amp = float(np.max(vv[:, axis_idx]) - np.min(vv[:, axis_idx]))
    return {
        "name": label,
        "t": tt.round(6).tolist(),
        "xyz": vv.round(6).tolist(),
        "amplitude_m": amp,
        "peak_time_s": float(peak_t - t[0]),
    }


def quat_to_euler_xyz(q: np.ndarray) -> np.ndarray:
    # q: [x,y,z,w]
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    # roll
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    # pitch
    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * (np.pi / 2), np.arcsin(sinp))
    # yaw
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.column_stack([roll, pitch, yaw])


def unwrap_relative(e: np.ndarray) -> np.ndarray:
    u = np.unwrap(e, axis=0)
    return u - u[0]


def extract_rot_primitives(series: PoseSeries) -> list[dict]:
    assert series.q is not None
    e = unwrap_relative(quat_to_euler_xyz(series.q))
    e = ma(e, 9)
    t = series.t
    abs_e = np.abs(e)
    dom = np.argmax(abs_e, axis=1)  # 0 roll,1 pitch,2 yaw
    mag = np.max(abs_e, axis=1)
    thr = np.percentile(mag, 55)
    valid = mag >= thr

    # run-length segments with stable dominant axis
    segs: list[tuple[int, int, int]] = []
    i = 0
    n = len(t)
    while i < n:
        if not valid[i]:
            i += 1
            continue
        a = int(dom[i])
        j = i + 1
        while j < n and valid[j] and int(dom[j]) == a:
            j += 1
        if j - i >= 18:  # >=0.2s at ~90Hz
            segs.append((i, j, a))
        i = j

    # keep first strong segment for each axis by span
    best = {}
    for i, j, a in segs:
        span = float(np.max(e[i:j, a]) - np.min(e[i:j, a]))
        if span < 0.08:  # ~4.5deg
            continue
        if (a not in best) or (span > best[a][0]):
            best[a] = (span, i, j)

    names = {0: "roll", 1: "pitch", 2: "yaw"}
    out = []
    for a in (0, 1, 2):
        if a not in best:
            # fallback: 取该轴绝对值峰值附近窗口，确保 roll/pitch/yaw 都能提取
            col = np.abs(e[:, a])
            k = int(np.argmax(col))
            w = 60  # ~0.67s
            i = max(0, k - w // 2)
            j = min(len(t), i + w)
            span = float(np.max(e[i:j, a]) - np.min(e[i:j, a]))
            if span < 0.04:
                continue
            best[a] = (span, i, j)
        span, i, j = best[a]
        tt = t[i:j] - t[i]
        xyz = np.zeros((j - i, 3), dtype=np.float64)
        xyz[:, a] = e[i:j, a]
        out.append(
            {
                "name": names[a],
                "t": tt.round(6).tolist(),
                "rpy_rad": xyz.round(6).tolist(),
                "amplitude_rad": float(span),
            }
        )
    return out


def main():
    out = {
        "meta": {
            "frame_semantics": "quest_control_absolute_pose_in_ros",
            "note": "从录制case提取的高置信度动作原语",
        },
        "clutch": {},
        "translation": {"right": {}, "left": {}},
        "rotation": {"right": {}, "left": {}},
    }

    # clutch primitives
    rows4 = load_rows(CASES["case4_clutch_edges"])
    clutch_msgs = [r for r in rows4 if r.get("topic") == "/quest3/clutch"]
    clutch_msgs.sort(key=lambda r: float(r["t"]))
    events = []
    prev = None
    for r in clutch_msgs:
        t = float(r["t"])
        v = bool(r["msg"]["data"])
        if prev is None or v != prev:
            events.append({"t": t, "state": "ON" if v else "OFF"})
        prev = v
    if events:
        t0 = events[0]["t"]
        for e in events:
            e["t_rel"] = round(e["t"] - t0, 6)
    out["clutch"]["events"] = events

    # right/left xyz primitives from case2/3 longest on segment
    rows2 = load_rows(CASES["case2_right_xyz"])
    seg2 = max(clutch_segments(rows2), key=lambda s: s[1] - s[0])
    right2 = crop(extract_pose(rows2, "/quest3/right_hand_pose"), *seg2)
    out["translation"]["right"]["+x"] = extract_axis_primitive(right2, 0, "right_+x")
    out["translation"]["right"]["+y"] = extract_axis_primitive(right2, 1, "right_+y")
    out["translation"]["right"]["+z"] = extract_axis_primitive(right2, 2, "right_+z")

    rows3 = load_rows(CASES["case3_left_xyz"])
    seg3 = max(clutch_segments(rows3), key=lambda s: s[1] - s[0])
    left3 = crop(extract_pose(rows3, "/quest3/left_hand_pose"), *seg3)
    out["translation"]["left"]["+x"] = extract_axis_primitive(left3, 0, "left_+x")
    out["translation"]["left"]["+y"] = extract_axis_primitive(left3, 1, "left_+y")
    out["translation"]["left"]["+z"] = extract_axis_primitive(left3, 2, "left_+z")

    # right rpy from case5
    rows5 = load_rows(CASES["case5_right_rot"])
    seg5 = max(clutch_segments(rows5), key=lambda s: s[1] - s[0])
    right5 = crop(extract_pose(rows5, "/quest3/right_hand_pose"), *seg5)
    right_rot = extract_rot_primitives(right5)
    for r in right_rot:
        out["rotation"]["right"][r["name"]] = r

    # left rpy: 当前数据集没有独立左手旋转动作，不伪造
    out["rotation"]["left"]["missing"] = "需要单独录制 left_rot case 才能高置信提取"

    out_path = OUT_DIR / "primitives_dataset.json"
    out_path.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")

    # summary
    lines = [
        "# 动作原语提取摘要",
        "",
        "## 已提取",
        "- clutch ON/OFF 事件序列（来自 case4）",
        "- right +x/+y/+z（来自 case2）",
        "- left +x/+y/+z（来自 case3）",
        "- right roll/pitch/yaw（来自 case5 四元数）",
        "",
        "## 待补齐",
        "- left roll/pitch/yaw：当前数据无独立左手旋转动作，未伪造",
        "",
        f"- 输出文件: `{out_path}`",
    ]
    (OUT_DIR / "primitives_summary.md").write_text("\n".join(lines), encoding="utf-8")
    print(str(out_path))
    print(str(OUT_DIR / "primitives_summary.md"))


if __name__ == "__main__":
    main()
