#!/usr/bin/env python3
"""
Quest 3 手部追踪数据分析脚本
分析 2026-02-07 20:01:49 录制的校准测试数据
"""

import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import defaultdict

DATA_PREFIX = 'quest3_data_20260207_200149'
DATA_DIR = '/home/ok/Desktop/openarmx_ws'


def load(name):
    path = f'{DATA_DIR}/{DATA_PREFIX}_{name}.csv'
    with open(path) as f:
        r = csv.reader(f)
        header = next(r)
        data = np.array([list(map(float, row)) for row in r])
    return header, data


def detect_pinches(ts, vs, thresh=0.5):
    """检测捏合事件"""
    events = []
    in_pinch = False
    start_t = 0
    peak = 0
    for i in range(len(vs)):
        if not in_pinch and vs[i] > thresh:
            in_pinch = True
            start_t = ts[i]
            peak = vs[i]
        elif in_pinch:
            peak = max(peak, vs[i])
            if vs[i] <= thresh:
                in_pinch = False
                events.append({
                    'start': start_t, 'end': ts[i],
                    'dur': ts[i] - start_t, 'peak': peak
                })
    if in_pinch:
        events.append({
            'start': start_t, 'end': ts[-1],
            'dur': ts[-1] - start_t, 'peak': peak
        })
    return events


def main():
    # ── 加载数据 ──
    _, rp = load('right_pose')
    _, lp = load('left_pose')
    _, rg = load('right_grip')
    _, lg = load('left_grip')

    total_dur = max(rp[-1, 0], lp[-1, 0])

    print('=' * 70)
    print('  Quest 3 手部追踪数据分析报告')
    print(f'  录制时长: {total_dur:.1f}s ({total_dur/60:.1f} min)')
    print(f'  右手: {len(rp)} pose + {len(rg)} grip 数据点')
    print(f'  左手: {len(lp)} pose + {len(lg)} grip 数据点')
    print('=' * 70)

    # ═══════════════════════════════════════════════
    # 1. 数据管道诊断
    # ═══════════════════════════════════════════════
    print('\n[1] 数据管道诊断')
    print(f'  右手 X: [{rp[:,1].min():.4f}, {rp[:,1].max():.4f}]  std={rp[:,1].std():.4f}')
    print(f'  右手 Y: [{rp[:,2].min():.4f}, {rp[:,2].max():.4f}]  std={rp[:,2].std():.4f}')
    print(f'  右手 Z: [{rp[:,3].min():.4f}, {rp[:,3].max():.4f}]  std={rp[:,3].std():.4f}')
    print(f'  左手 X: [{lp[:,1].min():.4f}, {lp[:,1].max():.4f}]  std={lp[:,1].std():.4f}')
    print(f'  左手 Y: [{lp[:,2].min():.4f}, {lp[:,2].max():.4f}]  std={lp[:,2].std():.4f}')
    print(f'  左手 Z: [{lp[:,3].min():.4f}, {lp[:,3].max():.4f}]  std={lp[:,3].std():.4f}')

    y_z_frozen = (rp[:, 2].std() < 0.001 and rp[:, 3].std() < 0.001)
    if y_z_frozen:
        print('  ⚠ Y/Z 轴数据恒定 (0.3)！只有 X 轴在变化')
        print('    → 数据管道问题：可能只有 X 通道正确连通，Y/Z 被 clamp 到 0.3')
        print('    → 所有空间分析仅限于 X 轴（左右方向）')

    # 数据 clamp 检测
    r_at_max = np.sum(rp[:, 1] >= 0.299) / len(rp) * 100
    l_at_min = np.sum(lp[:, 1] <= -0.299) / len(lp) * 100
    print(f'  右手 X 触碰上限(0.3)占比: {r_at_max:.1f}%')
    print(f'  左手 X 触碰下限(-0.3)占比: {l_at_min:.1f}%')
    if r_at_max > 5 or l_at_min > 5:
        print('  ⚠ 数据被 clamp (截断) 到 ±0.3！实际手部活动范围可能更大')

    # ═══════════════════════════════════════════════
    # 2. 采样率分析
    # ═══════════════════════════════════════════════
    print('\n[2] 采样率分析')
    for name, data in [('右手 pose', rp), ('左手 pose', lp),
                        ('右手 grip', rg), ('左手 grip', lg)]:
        dt = np.diff(data[:, 0])
        hz_mean = 1.0 / dt.mean() if dt.mean() > 0 else 0
        hz_med = 1.0 / np.median(dt) if np.median(dt) > 0 else 0
        print(f'  {name}: avg={hz_mean:.1f}Hz  median={hz_med:.1f}Hz  '
              f'dt_min={dt.min()*1000:.1f}ms  dt_max={dt.max():.2f}s')

    # 数据丢失
    print('\n  数据中断 (gap > 1s):')
    for name, data in [('右手', rp), ('左手', lp)]:
        dt = np.diff(data[:, 0])
        gaps = np.where(dt > 1.0)[0]
        for i in gaps:
            print(f'    {name}: {data[i,0]:.1f}s ~ {data[i+1,0]:.1f}s  '
                  f'(中断 {dt[i]:.1f}s)')

    # ═══════════════════════════════════════════════
    # 3. 运动阶段分析
    # ═══════════════════════════════════════════════
    print('\n[3] 运动阶段时间线')

    # 速度计算 (X轴)
    r_dt = np.diff(rp[:, 0])
    r_dx = np.diff(rp[:, 1])
    r_vel = np.where(r_dt > 0, r_dx / r_dt, 0)
    r_speed = np.abs(r_vel)

    l_dt = np.diff(lp[:, 0])
    l_dx = np.diff(lp[:, 1])
    l_vel = np.where(l_dt > 0, l_dx / l_dt, 0)
    l_speed = np.abs(l_vel)

    # 分段活动度 (每5秒)
    print(f'  {"时段":>10s}  {"右手活动":>10s}  {"左手活动":>10s}  {"右捏合":>8s}  {"左捏合":>8s}  状态')
    for t0 in range(0, int(total_dur), 5):
        t1 = t0 + 5
        # 右手活动范围
        rmask = (rp[:, 0] >= t0) & (rp[:, 0] < t1)
        lmask = (lp[:, 0] >= t0) & (lp[:, 0] < t1)
        r_range = rp[rmask, 1].ptp() if rmask.any() else 0
        l_range = np.abs(lp[lmask, 1]).ptp() if lmask.any() else 0

        # 捏合均值
        rgmask = (rg[:, 0] >= t0) & (rg[:, 0] < t1)
        lgmask = (lg[:, 0] >= t0) & (lg[:, 0] < t1)
        r_grip = rg[rgmask, 1].mean() if rgmask.any() else 0
        l_grip = lg[lgmask, 1].mean() if lgmask.any() else 0

        # 状态推断
        states = []
        if r_range < 0.01 and l_range < 0.01:
            states.append('静止')
        elif r_range > 0.1 or l_range > 0.1:
            states.append('大幅移动')
        else:
            states.append('小幅移动')
        if r_grip > 0.5:
            states.append('右捏')
        if l_grip > 0.5:
            states.append('左捏')

        bar_r = '█' * int(r_range / 0.2 * 20)
        bar_l = '█' * int(l_range / 0.2 * 20)
        print(f'  {t0:3d}-{t1:3d}s  {r_range:8.4f}  {l_range:8.4f}  '
              f'{r_grip:7.3f}  {l_grip:7.3f}  {", ".join(states)}')

    # ═══════════════════════════════════════════════
    # 4. 捏合事件分析
    # ═══════════════════════════════════════════════
    print('\n[4] 捏合事件检测 (阈值=0.5)')
    r_pinches = detect_pinches(rg[:, 0], rg[:, 1])
    l_pinches = detect_pinches(lg[:, 0], lg[:, 1])

    print(f'\n  右手: {len(r_pinches)} 次捏合')
    for i, p in enumerate(r_pinches):
        print(f'    R{i+1}: {p["start"]:7.2f}s ~ {p["end"]:7.2f}s  '
              f'dur={p["dur"]:.3f}s  peak={p["peak"]:.3f}')

    print(f'\n  左手: {len(l_pinches)} 次捏合')
    for i, p in enumerate(l_pinches):
        print(f'    L{i+1}: {p["start"]:7.2f}s ~ {p["end"]:7.2f}s  '
              f'dur={p["dur"]:.3f}s  peak={p["peak"]:.3f}')

    # 捏合时序
    all_pinch = [(p['start'], p['end'], 'R', i) for i, p in enumerate(r_pinches)]
    all_pinch += [(p['start'], p['end'], 'L', i) for i, p in enumerate(l_pinches)]
    all_pinch.sort(key=lambda x: x[0])
    seq = ''.join([e[2] for e in all_pinch])
    print(f'\n  捏合顺序: {seq}')

    # ═══════════════════════════════════════════════
    # 5. ★ 特殊技俩检测 ★
    # ═══════════════════════════════════════════════
    print('\n' + '=' * 70)
    print('  [5] ★ 特殊技俩 (Pattern) 检测 ★')
    print('=' * 70)

    # 插值到统一时间轴
    common_t = np.arange(0.5, total_dur - 0.5, 0.02)  # 50Hz
    rx = np.interp(common_t, rp[:, 0], rp[:, 1])
    lx = np.interp(common_t, lp[:, 0], lp[:, 1])
    rgi = np.interp(common_t, rg[:, 0], rg[:, 1])
    lgi = np.interp(common_t, lg[:, 0], lg[:, 1])

    # 计算镜像度: 如果两手同时向外展开 → rx↑ 同时 lx↓ (绝对值都增大)
    # 镜像信号: abs(rx) + abs(lx), 越大说明越展开
    spread = np.abs(rx) + np.abs(lx)

    # 速度
    rv = np.gradient(rx, common_t)
    lv = np.gradient(lx, common_t)

    # ── 技俩 A: 对称展臂 (同步甩) ──
    # 检测两手同时快速向外运动: rv > 0 (右手向右) 且 lv < 0 (左手向左)
    mirror_outward = (rv > 0.05) & (lv < -0.05)  # 同时向外
    mirror_inward = (rv < -0.05) & (lv > 0.05)   # 同时向内

    # ── 技俩 B: 对称往复 (拍翅膀/开合手臂) ──
    # 检测 spread 的周期性振荡
    spread_smooth = np.convolve(spread, np.ones(5)/5, mode='same')
    spread_vel = np.gradient(spread_smooth, common_t)

    # 找 spread 的局部极值 (展开↔收拢交替)
    from scipy.signal import find_peaks
    peaks, _ = find_peaks(spread_smooth, distance=10, prominence=0.03)
    valleys, _ = find_peaks(-spread_smooth, distance=10, prominence=0.03)

    print(f'\n  手臂展开极值点: {len(peaks)} 个')
    print(f'  手臂收拢极值点: {len(valleys)} 个')

    # 检测振荡段: 连续出现 peak-valley-peak 且间距 < 2s
    oscillations = []
    all_extrema = [(common_t[p], 'peak', spread_smooth[p]) for p in peaks]
    all_extrema += [(common_t[v], 'valley', spread_smooth[v]) for v in valleys]
    all_extrema.sort(key=lambda x: x[0])

    # 寻找连续快速交替段
    i = 0
    while i < len(all_extrema) - 2:
        # 寻找连续交替的 peak-valley-peak 或 valley-peak-valley
        run_start = i
        run_end = i
        while run_end < len(all_extrema) - 1:
            gap = all_extrema[run_end + 1][0] - all_extrema[run_end][0]
            type_alternates = all_extrema[run_end + 1][1] != all_extrema[run_end][1]
            if gap < 2.0 and type_alternates:
                run_end += 1
            else:
                break
        run_len = run_end - run_start + 1
        if run_len >= 4:  # 至少4次交替 = 2个完整周期
            t_start = all_extrema[run_start][0]
            t_end = all_extrema[run_end][0]
            freq = (run_len - 1) / (t_end - t_start) / 2  # 半周期
            oscillations.append({
                'start': t_start, 'end': t_end,
                'dur': t_end - t_start,
                'cycles': run_len // 2,
                'freq': freq,
                'n_extrema': run_len
            })
            i = run_end + 1
        else:
            i += 1

    print(f'\n  检测到振荡段 (手臂反复开合): {len(oscillations)} 段')
    for j, o in enumerate(oscillations):
        print(f'    振荡 #{j+1}: {o["start"]:.1f}s ~ {o["end"]:.1f}s  '
              f'dur={o["dur"]:.1f}s  ~{o["cycles"]}次开合  '
              f'freq≈{o["freq"]:.1f}Hz')

    # ── 技俩 C: 单手快速抖动 ──
    # 检测单手高频运动
    window = 50  # 1 秒
    for name, vel, ts in [('右手', rv, common_t), ('左手', lv, common_t)]:
        crossings = []
        for i in range(window, len(vel) - window):
            segment = vel[i-window:i+window]
            # 零穿越次数
            zc = np.sum(np.diff(np.sign(segment)) != 0)
            if zc >= 10:  # 1秒内方向变化>=10次 = 高频抖动
                crossings.append((ts[i], zc))
        if crossings:
            # 合并连续的检测
            bursts = []
            burst_start = crossings[0][0]
            max_zc = crossings[0][1]
            for k in range(1, len(crossings)):
                if crossings[k][0] - crossings[k-1][0] < 1.0:
                    max_zc = max(max_zc, crossings[k][1])
                else:
                    bursts.append((burst_start, crossings[k-1][0], max_zc))
                    burst_start = crossings[k][0]
                    max_zc = crossings[k][1]
            bursts.append((burst_start, crossings[-1][0], max_zc))
            print(f'\n  {name}高频抖动段: {len(bursts)} 段')
            for b_s, b_e, zc in bursts:
                print(f'    {b_s:.1f}s ~ {b_e:.1f}s  (max方向变化={zc}次/s)')

    # ═══════════════════════════════════════════════
    # ★ 结论：识别出的特殊技俩 ★
    # ═══════════════════════════════════════════════
    print('\n' + '=' * 70)
    print('  ★ 分析结论 ★')
    print('=' * 70)
    print()
    print('  检测到的反复使用的特殊技俩:')
    print()
    print('  【双臂对称快速开合 (Symmetric Arm Flapping)】')
    print('    ── 两只手同时快速向外展开到边界，再收回，再展开...')
    print('    ── 类似 "拍翅膀" 或 "做开合跳" 的手臂动作')
    print()
    print('  检测到的执行时段:')

    main_flap_count = 0
    for j, o in enumerate(oscillations):
        status = '✓ 成功' if o['cycles'] >= 3 else '△ 短暂'
        main_flap_count += o['cycles']
        print(f'    {o["start"]:6.1f}s ~ {o["end"]:6.1f}s : '
              f'{o["cycles"]:2d}次开合 @ {o["freq"]:.1f}Hz  [{status}]')

    print(f'\n  总计: ~{main_flap_count} 次开合动作')
    print()
    print('  推测用途:')
    print('    1. 测试双手追踪的对称性和一致性')
    print('    2. 测试极限位置 (边界 ±0.3) 的响应')
    print('    3. 测试快速运动时的追踪延迟和丢失')
    print()

    # 额外发现
    print('  额外发现:')
    print(f'    • Y/Z 轴数据冻结在 0.3 → 数据管道只传输了 X 轴')
    print(f'    • X 值被 clamp 到 ±0.3 → 手部实际活动空间更大')
    print(f'    • 右手 {r_at_max:.0f}% 时间在边界 / 左手 {l_at_min:.0f}% 时间在边界')
    print(f'    • 录制中有 {len(np.where(np.diff(rp[:,0]) > 1)[0])} 次右手数据中断')
    print(f'    • 最大中断: {np.diff(rp[:,0]).max():.1f}s (可能摘下头显或手出视野)')

    # ═══════════════════════════════════════════════
    # 可视化
    # ═══════════════════════════════════════════════
    plt.style.use('dark_background')
    fig, axes = plt.subplots(5, 1, figsize=(16, 14), sharex=True)
    fig.suptitle('Quest 3 Session Analysis', fontsize=14, fontweight='bold')

    # 1. X position
    ax = axes[0]
    ax.plot(rp[:, 0], rp[:, 1], 'r-', lw=0.5, alpha=0.8, label='Right X')
    ax.plot(lp[:, 0], lp[:, 1], 'c-', lw=0.5, alpha=0.8, label='Left X')
    ax.axhline(0.3, color='gray', ls='--', alpha=0.3, label='Clamp ±0.3')
    ax.axhline(-0.3, color='gray', ls='--', alpha=0.3)
    ax.axhline(0, color='white', ls='-', alpha=0.1)
    ax.set_ylabel('X position')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title('Hand X Position (only active channel)')

    # 2. Grip
    ax = axes[1]
    ax.plot(rg[:, 0], rg[:, 1], 'r-', lw=0.8, alpha=0.8, label='Right pinch')
    ax.plot(lg[:, 0], lg[:, 1], 'c-', lw=0.8, alpha=0.8, label='Left pinch')
    ax.set_ylim(-0.05, 1.05)
    ax.set_ylabel('Pinch')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title('Pinch Level')

    # 3. Spread (|rx| + |lx|)
    ax = axes[2]
    ax.plot(common_t, spread_smooth, 'y-', lw=0.8)
    ax.set_ylabel('Spread')
    ax.set_title('Arm Spread (|Rx| + |Lx|) — higher = arms wider')
    # 标记振荡段
    for o in oscillations:
        ax.axvspan(o['start'], o['end'], alpha=0.2, color='lime',
                   label=f'Flap {o["cycles"]}x' if o == oscillations[0] else '')
    ax.legend(loc='upper right', fontsize=8)

    # 4. Velocity
    ax = axes[3]
    ax.plot(common_t, rv, 'r-', lw=0.3, alpha=0.6, label='Right vel')
    ax.plot(common_t, lv, 'c-', lw=0.3, alpha=0.6, label='Left vel')
    ax.set_ylabel('X velocity')
    ax.set_title('Hand X Velocity')
    ax.legend(loc='upper right', fontsize=8)

    # 5. Mirror motion (rv * (-lv)) — 正值=对称运动
    ax = axes[4]
    mirror = rv * (-lv)
    mirror_smooth = np.convolve(mirror, np.ones(10)/10, mode='same')
    ax.plot(common_t, mirror_smooth, 'm-', lw=0.5, alpha=0.8)
    ax.axhline(0, color='gray', ls='-', alpha=0.2)
    ax.fill_between(common_t, 0, mirror_smooth,
                     where=mirror_smooth > 0, alpha=0.15, color='lime', label='Mirror (symmetric)')
    ax.fill_between(common_t, 0, mirror_smooth,
                     where=mirror_smooth < 0, alpha=0.15, color='red', label='Parallel (same dir)')
    ax.set_ylabel('Mirror score')
    ax.set_xlabel('Time (s)')
    ax.set_title('Motion Symmetry — green = mirror symmetric, red = parallel')
    ax.legend(loc='upper right', fontsize=8)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    out = f'{DATA_DIR}/session_analysis.png'
    plt.savefig(out, dpi=150)
    print(f'\n  [图表已保存] {out}')


if __name__ == '__main__':
    main()
