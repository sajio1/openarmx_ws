#!/usr/bin/env python3
"""
Quest 3 Hand Tracker — Live preview + Video export

Live:  Matplotlib ~2-5fps (够看)
Close: 自动合成 30fps MP4 视频 (所有数据重绘, 丝滑)

Data: /quest3/* (Unity raw: X+=right, Y+=up, Z+=forward)

Layout (2x2):
  Top-Left:  Top-Down (XZ)
  Top-Right: Front (XY)
  Bot-Left:  Side (ZY)
  Bot-Right: Pinch timeline
"""

import sys
import time
import threading
import collections
import os
import csv

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FFMpegWriter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


GRIPPER_HISTORY_SEC = 10
VIDEO_FPS = 30
OUTPUT_DIR = '/home/ok/Desktop/openarmx_ws'


class DataRecorder(Node):
    """订阅 Quest3 话题, 同时保存所有带时间戳的原始数据用于离线渲染."""

    def __init__(self):
        super().__init__('quest3_visualizer')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(PoseStamped, '/quest3/right_hand_pose', self._rp_cb, qos)
        self.create_subscription(Float32, '/quest3/right_gripper', self._rg_cb, qos)
        self.create_subscription(PoseStamped, '/quest3/left_hand_pose', self._lp_cb, qos)
        self.create_subscription(Float32, '/quest3/left_gripper', self._lg_cb, qos)

        self.t0 = time.time()
        self.lock = threading.Lock()

        # ── 全量录制 (append only, for video) ──
        self.right_poses = []   # [(t, x, y, z), ...]
        self.left_poses = []
        self.right_grips = []   # [(t, v), ...]
        self.left_grips = []

        # ── 实时预览 (尾部) ──
        self.MAX_TRAIL = 300
        self._rt = collections.deque(maxlen=self.MAX_TRAIL)
        self._lt = collections.deque(maxlen=self.MAX_TRAIL)
        self._rp = (0., 0., 0.)
        self._lp = (0., 0., 0.)
        self._rgh = collections.deque(maxlen=1500)
        self._lgh = collections.deque(maxlen=1500)
        self._rg = 0.0
        self._lg = 0.0
        self._rc = 0
        self._lc = 0

        # Hz
        self._rts = collections.deque(maxlen=100)
        self._lts = collections.deque(maxlen=100)
        self._rhz = 0.0
        self._lhz = 0.0

    def _t(self):
        return time.time() - self.t0

    def _hz(self, dq):
        now = time.time()
        dq.append(now)
        while dq and now - dq[0] > 2.0:
            dq.popleft()
        if len(dq) >= 2:
            dt = dq[-1] - dq[0]
            return (len(dq) - 1) / dt if dt > 0 else 0
        return 0

    def _rp_cb(self, msg):
        p = msg.pose.position
        t = self._t()
        with self.lock:
            self.right_poses.append((t, p.x, p.y, p.z))
            self._rp = (p.x, p.y, p.z)
            self._rt.append((p.x, p.y, p.z))
            self._rc += 1
            self._rhz = self._hz(self._rts)

    def _lp_cb(self, msg):
        p = msg.pose.position
        t = self._t()
        with self.lock:
            self.left_poses.append((t, p.x, p.y, p.z))
            self._lp = (p.x, p.y, p.z)
            self._lt.append((p.x, p.y, p.z))
            self._lc += 1
            self._lhz = self._hz(self._lts)

    def _rg_cb(self, msg):
        t = self._t()
        with self.lock:
            self.right_grips.append((t, msg.data))
            self._rg = msg.data
            self._rgh.append((t, msg.data))

    def _lg_cb(self, msg):
        t = self._t()
        with self.lock:
            self.left_grips.append((t, msg.data))
            self._lg = msg.data
            self._lgh.append((t, msg.data))

    def snapshot(self):
        """实时预览用的快照."""
        with self.lock:
            return {
                'rt': list(self._rt), 'lt': list(self._lt),
                'rp': self._rp, 'lp': self._lp,
                'rgh': list(self._rgh), 'lgh': list(self._lgh),
                'rg': self._rg, 'lg': self._lg,
                'rhz': self._rhz, 'lhz': self._lhz,
                'rc': self._rc, 'lc': self._lc,
                't0': self.t0,
            }

    def all_data(self):
        """视频渲染用的全量数据."""
        with self.lock:
            return {
                'rp': list(self.right_poses),
                'lp': list(self.left_poses),
                'rg': list(self.right_grips),
                'lg': list(self.left_grips),
            }


# ═══════════════════════════════════════════════════
#  Live Preview  (低帧率, 够看就行)
# ═══════════════════════════════════════════════════

def draw_view(ax, lt, rt, lp, rp, xi, yi, xlabel, ylabel, title):
    ax.cla()
    ax.set_title(title, fontsize=9)
    ax.set_xlabel(xlabel, fontsize=7)
    ax.set_ylabel(ylabel, fontsize=7)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.15)
    if lt:
        xs = [p[xi] for p in lt]
        ys = [p[yi] for p in lt]
        ax.plot(xs, ys, 'c.', markersize=1, alpha=0.4)
        ax.plot(lp[xi], lp[yi], 'co', markersize=10)
    if rt:
        xs = [p[xi] for p in rt]
        ys = [p[yi] for p in rt]
        ax.plot(xs, ys, 'r.', markersize=1, alpha=0.4)
        ax.plot(rp[xi], rp[yi], 'ro', markersize=10)


def live_update(frame_num, node, axes, fps_dq):
    s = node.snapshot()

    now = time.time()
    fps_dq.append(now)
    fps = 0
    if len(fps_dq) >= 2:
        dt = fps_dq[-1] - fps_dq[0]
        if dt > 0:
            fps = (len(fps_dq) - 1) / dt

    ax_top, ax_front, ax_side, ax_grip = axes

    draw_view(ax_top, s['lt'], s['rt'], s['lp'], s['rp'],
              0, 2, 'X', 'Z',
              f'Top-Down (XZ)  L:{s["lhz"]:.0f}Hz R:{s["rhz"]:.0f}Hz  ~{fps:.0f}fps')
    draw_view(ax_front, s['lt'], s['rt'], s['lp'], s['rp'],
              0, 1, 'X', 'Y', 'Front (XY) - POV')
    draw_view(ax_side, s['lt'], s['rt'], s['lp'], s['rp'],
              2, 1, 'Z', 'Y', 'Side (ZY)')

    ax_grip.cla()
    ax_grip.set_title(f'Pinch  L:{s["lg"]:.3f}  R:{s["rg"]:.3f}', fontsize=9)
    ax_grip.set_xlabel('Time (s)', fontsize=7)
    ax_grip.set_ylabel('Pinch', fontsize=7)
    ax_grip.set_ylim(-0.05, 1.05)
    ax_grip.grid(True, alpha=0.15)

    t_now = time.time() - s['t0']
    ax_grip.set_xlim(max(0, t_now - GRIPPER_HISTORY_SEC), t_now + 0.3)

    if s['lgh']:
        ts, vs = zip(*s['lgh'])
        ax_grip.plot(ts, vs, 'c-', lw=1.2, alpha=0.8, label='Left')
    if s['rgh']:
        ts, vs = zip(*s['rgh'])
        ax_grip.plot(ts, vs, 'r-', lw=1.2, alpha=0.8, label='Right')
    if s['lgh'] or s['rgh']:
        ax_grip.legend(loc='upper left', fontsize=7)


# ═══════════════════════════════════════════════════
#  CSV Data Export  (关闭窗口后自动保存原始数据)
# ═══════════════════════════════════════════════════

def save_csv(data):
    """保存全量原始数据为 CSV 文件."""
    ts_str = time.strftime('%Y%m%d_%H%M%S')
    base = os.path.join(OUTPUT_DIR, f'quest3_data_{ts_str}')

    files_saved = []

    # right_poses: t, x, y, z
    if data['rp']:
        path = base + '_right_pose.csv'
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'x', 'y', 'z'])
            w.writerows(data['rp'])
        files_saved.append((path, len(data['rp'])))

    # left_poses: t, x, y, z
    if data['lp']:
        path = base + '_left_pose.csv'
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'x', 'y', 'z'])
            w.writerows(data['lp'])
        files_saved.append((path, len(data['lp'])))

    # right_grips: t, value
    if data['rg']:
        path = base + '_right_grip.csv'
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'pinch'])
            w.writerows(data['rg'])
        files_saved.append((path, len(data['rg'])))

    # left_grips: t, value
    if data['lg']:
        path = base + '_left_grip.csv'
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'pinch'])
            w.writerows(data['lg'])
        files_saved.append((path, len(data['lg'])))

    if files_saved:
        print('[CSV] Saved raw data:')
        for p, n in files_saved:
            sz = os.path.getsize(p) / 1024
            print(f'  {p}  ({n} rows, {sz:.1f} KB)')
    else:
        print('[CSV] No data to save.')

    return files_saved


# ═══════════════════════════════════════════════════
#  Offline Video Render  (关闭窗口后自动执行)
# ═══════════════════════════════════════════════════

def render_video(data):
    """用全量数据重绘每一帧, 导出 30fps MP4."""
    rp = np.array(data['rp']) if data['rp'] else None  # (N, 4): t,x,y,z
    lp = np.array(data['lp']) if data['lp'] else None
    rg = np.array(data['rg']) if data['rg'] else None  # (N, 2): t,v
    lg = np.array(data['lg']) if data['lg'] else None

    if rp is None and lp is None:
        print('[Video] No data recorded, skipping video export.')
        return

    # 时间范围
    all_t = []
    if rp is not None and len(rp): all_t.append(rp[-1, 0])
    if lp is not None and len(lp): all_t.append(lp[-1, 0])
    if not all_t:
        print('[Video] No pose data, skipping.')
        return
    t_max = max(all_t)
    n_frames = int(t_max * VIDEO_FPS) + 1

    if n_frames < 2:
        print('[Video] Recording too short, skipping.')
        return

    print(f'[Video] Rendering {n_frames} frames ({t_max:.1f}s) @ {VIDEO_FPS}fps ...')

    plt.style.use('dark_background')
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Quest 3 Hand Tracker (Replay)', fontsize=12, fontweight='bold')

    ax_top = axes[0][0]
    ax_front = axes[0][1]
    ax_side = axes[1][0]
    ax_grip = axes[1][1]

    trail_sec = 5.0  # 拖尾秒数

    ts_str = time.strftime('%Y%m%d_%H%M%S')
    out_path = os.path.join(OUTPUT_DIR, f'quest3_recording_{ts_str}.mp4')

    writer = FFMpegWriter(fps=VIDEO_FPS, metadata={'title': 'Quest3 Hand Tracker'})

    def draw_frame_at(t):
        for ax in [ax_top, ax_front, ax_side, ax_grip]:
            ax.cla()

        # 提取到时间 t 为止的数据
        trail_start = max(0, t - trail_sec)

        def get_trail(arr, t_start, t_end):
            if arr is None or len(arr) == 0:
                return None
            mask = (arr[:, 0] >= t_start) & (arr[:, 0] <= t_end)
            return arr[mask] if mask.any() else None

        def get_current(arr, t_end):
            if arr is None or len(arr) == 0:
                return None
            mask = arr[:, 0] <= t_end
            if not mask.any():
                return None
            return arr[mask][-1]

        rt_trail = get_trail(rp, trail_start, t)
        lt_trail = get_trail(lp, trail_start, t)
        r_cur = get_current(rp, t)
        l_cur = get_current(lp, t)

        def plot_view(ax, xi, yi, xlabel, ylabel, title):
            ax.set_title(title, fontsize=9)
            ax.set_xlabel(xlabel, fontsize=7)
            ax.set_ylabel(ylabel, fontsize=7)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.15)
            if lt_trail is not None:
                ax.plot(lt_trail[:, xi+1], lt_trail[:, yi+1], 'c.', ms=1, alpha=0.4)
            if l_cur is not None:
                ax.plot(l_cur[xi+1], l_cur[yi+1], 'co', ms=10)
            if rt_trail is not None:
                ax.plot(rt_trail[:, xi+1], rt_trail[:, yi+1], 'r.', ms=1, alpha=0.4)
            if r_cur is not None:
                ax.plot(r_cur[xi+1], r_cur[yi+1], 'ro', ms=10)

        plot_view(ax_top, 0, 2, 'X', 'Z', f'Top-Down (XZ)  t={t:.2f}s')
        plot_view(ax_front, 0, 1, 'X', 'Y', 'Front (XY)')
        plot_view(ax_side, 2, 1, 'Z', 'Y', 'Side (ZY)')

        # Pinch
        ax_grip.set_title('Pinch', fontsize=9)
        ax_grip.set_xlabel('Time (s)', fontsize=7)
        ax_grip.set_ylabel('Pinch', fontsize=7)
        ax_grip.set_ylim(-0.05, 1.05)
        ax_grip.grid(True, alpha=0.15)
        grip_start = max(0, t - GRIPPER_HISTORY_SEC)
        ax_grip.set_xlim(grip_start, t + 0.3)

        if lg is not None and len(lg):
            mask = (lg[:, 0] >= grip_start) & (lg[:, 0] <= t)
            if mask.any():
                d = lg[mask]
                ax_grip.plot(d[:, 0], d[:, 1], 'c-', lw=1.2, alpha=0.8, label='Left')
        if rg is not None and len(rg):
            mask = (rg[:, 0] >= grip_start) & (rg[:, 0] <= t)
            if mask.any():
                d = rg[mask]
                ax_grip.plot(d[:, 0], d[:, 1], 'r-', lw=1.2, alpha=0.8, label='Right')

        ax_grip.legend(loc='upper left', fontsize=7)
        fig.tight_layout(rect=[0, 0, 1, 0.96])

    with writer.saving(fig, out_path, dpi=100):
        for i in range(n_frames):
            t = i / VIDEO_FPS
            draw_frame_at(t)
            writer.grab_frame()
            if (i + 1) % VIDEO_FPS == 0 or i == n_frames - 1:
                pct = (i + 1) / n_frames * 100
                print(f'  [{pct:5.1f}%] {i+1}/{n_frames} frames')

    plt.close(fig)
    sz = os.path.getsize(out_path) / (1024 * 1024)
    print(f'[Video] Done! {out_path}  ({sz:.1f} MB)')


# ═══════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════

def main():
    rclpy.init()
    node = DataRecorder()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # ── Live preview ──
    plt.style.use('dark_background')
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Quest 3 Hand Tracker  [LIVE]  (close window to export video)',
                 fontsize=12, fontweight='bold')
    ax_list = [axes[0][0], axes[0][1], axes[1][0], axes[1][1]]
    fps_dq = collections.deque(maxlen=30)

    ani = animation.FuncAnimation(
        fig, live_update, fargs=(node, ax_list, fps_dq),
        interval=100,  # ~10fps target, 实际看CPU
        cache_frame_data=False
    )

    print('='*60)
    print('  Quest 3 Hand Tracker — LIVE MODE')
    print('  Close the window or Ctrl+C to stop & export video')
    print('='*60)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass

    # ── 窗口关闭后, 导出视频 ──
    print('\n[Info] Live preview closed. Preparing video export...')
    data = node.all_data()
    total = len(data['rp']) + len(data['lp'])
    print(f'[Info] Recorded: {len(data["rp"])} right poses, {len(data["lp"])} left poses, '
          f'{len(data["rg"])} right grips, {len(data["lg"])} left grips')

    if total > 0:
        save_csv(data)
        render_video(data)
    else:
        print('[Info] No data recorded, nothing to export.')

    node.destroy_node()
    rclpy.shutdown()
    print('[Info] Done. Goodbye!')


if __name__ == '__main__':
    main()
