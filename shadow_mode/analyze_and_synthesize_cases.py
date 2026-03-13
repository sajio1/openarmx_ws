#!/usr/bin/env python3
"""分析并合成 Quest 录制动作数据.

输入: /tmp/case*.jsonl
输出:
  - shadow_mode/analysis_outputs/quality_report.md
  - shadow_mode/analysis_outputs/cleaned_<case>_<hand>.csv
  - shadow_mode/analysis_outputs/action_templates.json

目标:
  1) 自动找出 clutch ON 控制窗口
  2) 清理脏点(短时跳变/丢跟踪后的异常步长)
  3) 统一重采样到 90Hz
  4) 生成可复用的“标准动作模板”
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

CASES = [
    ("case1_static", "/tmp/case1_static.jsonl"),
    ("case2_right_xyz", "/tmp/case2_right_xyz.jsonl"),
    ("case3_left_xyz", "/tmp/case3_left_xyz.jsonl"),
    ("case4_clutch_edges", "/tmp/case4_clutch_edges.jsonl"),
    ("case5_right_rot", "/tmp/case5_right_rot.jsonl"),
    ("case6_bimanual", "/tmp/case6_bimanual.jsonl"),
]

OUT_DIR = Path("/home/ok/Desktop/openarmx_ws/shadow_mode/analysis_outputs")
OUT_DIR.mkdir(parents=True, exist_ok=True)


@dataclass
class Traj:
    t: np.ndarray  # (N,)
    p: np.ndarray  # (N,3)


def load_rows(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def extract_pose(rows: Iterable[dict], topic: str) -> Traj | None:
    rec = [r for r in rows if r.get("topic") == topic]
    if len(rec) < 2:
        return None
    t = np.array([float(r["t"]) for r in rec], dtype=np.float64)
    p = np.array(
        [
            [
                float(r["msg"]["position"]["x"]),
                float(r["msg"]["position"]["y"]),
                float(r["msg"]["position"]["z"]),
            ]
            for r in rec
        ],
        dtype=np.float64,
    )
    return Traj(t=t, p=p)


def extract_clutch_segments(rows: Iterable[dict]) -> list[tuple[float, float]]:
    clutch = [r for r in rows if r.get("topic") == "/quest3/clutch"]
    if not clutch:
        return []
    clutch.sort(key=lambda r: float(r["t"]))
    segs: list[tuple[float, float]] = []
    on_t: float | None = None
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


def clip_to_window(traj: Traj, t0: float, t1: float) -> Traj | None:
    mask = (traj.t >= t0) & (traj.t <= t1)
    if mask.sum() < 2:
        return None
    return Traj(t=traj.t[mask], p=traj.p[mask])


def moving_average(x: np.ndarray, win: int = 5) -> np.ndarray:
    if win <= 1 or len(x) < win:
        return x.copy()
    pad = win // 2
    xp = np.pad(x, ((pad, pad), (0, 0)), mode="edge")
    k = np.ones(win, dtype=np.float64) / win
    y = np.zeros_like(x)
    for c in range(x.shape[1]):
        y[:, c] = np.convolve(xp[:, c], k, mode="valid")
    return y


def clean_spikes(traj: Traj) -> tuple[Traj, int]:
    t = traj.t.copy()
    p = traj.p.copy()
    if len(t) < 4:
        return Traj(t, p), 0
    d = np.linalg.norm(np.diff(p, axis=0), axis=1)
    med = float(np.median(d))
    mad = float(np.median(np.abs(d - med))) + 1e-9
    thr = med + 6.0 * mad
    spikes = np.where(d > thr)[0]
    fixed = 0
    for i in spikes:
        # fix point i+1 by interpolation from neighbors
        j = i + 1
        if 1 <= j < len(p) - 1:
            p[j] = 0.5 * (p[j - 1] + p[j + 1])
            fixed += 1
    p = moving_average(p, win=5)
    return Traj(t=t, p=p), fixed


def resample_90hz(traj: Traj) -> Traj:
    t0, t1 = float(traj.t[0]), float(traj.t[-1])
    if t1 <= t0:
        return traj
    dt = 1.0 / 90.0
    tn = np.arange(t0, t1 + 1e-9, dt, dtype=np.float64)
    pn = np.column_stack(
        [np.interp(tn, traj.t, traj.p[:, c]) for c in range(3)]
    ).astype(np.float64)
    return Traj(t=tn, p=pn)


def normalize_to_start(traj: Traj) -> Traj:
    return Traj(t=traj.t - traj.t[0], p=traj.p - traj.p[0])


def write_csv(path: Path, traj: Traj):
    with path.open("w", encoding="utf-8", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["time_s", "x", "y", "z"])
        for i in range(len(traj.t)):
            w.writerow([f"{traj.t[i]:.6f}", f"{traj.p[i,0]:.6f}", f"{traj.p[i,1]:.6f}", f"{traj.p[i,2]:.6f}"])


def dominant_hand(left: Traj | None, right: Traj | None) -> str:
    def energy(tr: Traj | None) -> float:
        if tr is None or len(tr.p) < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(tr.p, axis=0), axis=1)))
    er = energy(right)
    el = energy(left)
    return "right" if er >= el else "left"


def make_template(traj: Traj, name: str) -> dict:
    # normalize duration to 200 samples for easy replay/comparison
    n = 200
    if len(traj.t) < 2:
        return {"name": name, "samples": []}
    tn = np.linspace(traj.t[0], traj.t[-1], n)
    pn = np.column_stack([np.interp(tn, traj.t, traj.p[:, c]) for c in range(3)])
    amp = np.max(np.abs(pn), axis=0)
    amp[amp < 1e-6] = 1.0
    pnorm = pn / amp
    return {
        "name": name,
        "duration_s": float(traj.t[-1] - traj.t[0]),
        "scale_xyz": amp.tolist(),
        "samples": pnorm.round(6).tolist(),
    }


def main():
    report_lines = ["# 动作数据分析与合成报告", ""]
    templates: list[dict] = []

    for case_name, case_path in CASES:
        path = Path(case_path)
        if not path.exists():
            report_lines.append(f"- {case_name}: 文件不存在 `{case_path}`")
            continue
        rows = load_rows(path)
        left = extract_pose(rows, "/quest3/left_hand_pose")
        right = extract_pose(rows, "/quest3/right_hand_pose")
        segs = extract_clutch_segments(rows)

        report_lines.append(f"## {case_name}")
        report_lines.append(f"- 总消息数: {len(rows)}")
        report_lines.append(f"- clutch 控制段数: {len(segs)}")

        # 选择主控制段（最长 ON 段）
        if segs:
            seg = max(segs, key=lambda s: s[1] - s[0])
            report_lines.append(f"- 主控制段: {seg[0]:.3f} -> {seg[1]:.3f} (时长 {seg[1]-seg[0]:.3f}s)")
            left_w = clip_to_window(left, seg[0], seg[1]) if left else None
            right_w = clip_to_window(right, seg[0], seg[1]) if right else None
        else:
            left_w, right_w = left, right
            report_lines.append("- 无 clutch 段，使用全段")

        hand = dominant_hand(left_w, right_w)
        src = right_w if hand == "right" else left_w
        if src is None:
            report_lines.append("- 无有效 pose 数据")
            report_lines.append("")
            continue

        cleaned, fixed = clean_spikes(src)
        rs = resample_90hz(cleaned)
        norm = normalize_to_start(rs)

        span = np.ptp(norm.p, axis=0)
        report_lines.append(f"- 主动作手: **{hand}**")
        report_lines.append(f"- 清理脏点数量: {fixed}")
        report_lines.append(f"- 重采样后点数: {len(norm.t)} @90Hz")
        report_lines.append(f"- 位移跨度(m): x={span[0]:.4f}, y={span[1]:.4f}, z={span[2]:.4f}")

        out_csv = OUT_DIR / f"cleaned_{case_name}_{hand}.csv"
        write_csv(out_csv, norm)
        report_lines.append(f"- 清洗输出: `{out_csv}`")

        templates.append(make_template(norm, case_name))
        report_lines.append("")

    (OUT_DIR / "quality_report.md").write_text("\n".join(report_lines), encoding="utf-8")
    (OUT_DIR / "action_templates.json").write_text(
        json.dumps({"templates": templates}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(str(OUT_DIR / "quality_report.md"))
    print(str(OUT_DIR / "action_templates.json"))


if __name__ == "__main__":
    main()
