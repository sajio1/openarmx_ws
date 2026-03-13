#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN 接口管理脚本

查看 CAN 缓存信息、统计、队列，以及开启/关闭/清理操作。

用法:
    python3 can_register.py              # 查看所有 CAN 状态
    python3 can_register.py can2        # 查看指定接口
    python3 can_register.py can2 --up   # 开启 can2
    python3 can_register.py can2 --down # 关闭 can2
    python3 can_register.py --up        # 开启 can2, can3
    python3 can_register.py --down      # 关闭 can2, can3
    python3 can_register.py --reset     # 关闭后重新开启 (清理 buffer)
"""

import argparse
import subprocess
import sys
from pathlib import Path

# 默认 CAN 接口 (与 start_shadow / calibrate_motors 一致)
DEFAULT_CANS = ["can2", "can3"]
SUDO_PASSWORD = "123456"


def run_sudo(cmd: str) -> subprocess.CompletedProcess:
    full = f"echo {SUDO_PASSWORD} | sudo -S {cmd}"
    return subprocess.run(full, shell=True, capture_output=True, text=True)


def get_can_interfaces() -> list[str]:
    """获取系统中存在的 CAN 接口"""
    result = subprocess.run(
        "ip link show | grep -oE 'can[0-9]+' | sort -u",
        shell=True, capture_output=True, text=True
    )
    return result.stdout.strip().split() if result.stdout.strip() else []


def read_sysfs(iface: str, path: str, default: str = "-") -> str:
    """读取 sysfs 文件"""
    try:
        p = Path(f"/sys/class/net/{iface}") / path
        if p.exists():
            return p.read_text().strip()
    except Exception:
        pass
    return default


def get_can_state(iface: str) -> dict:
    """获取 CAN 接口状态"""
    result = subprocess.run(
        f"ip -details -statistics link show {iface} 2>/dev/null",
        shell=True, capture_output=True, text=True
    )
    raw = result.stdout

    # 解析状态
    state = "unknown"
    if "state UP" in raw or "state UNKNOWN" in raw:
        state = "UP"
    elif "state DOWN" in raw:
        state = "DOWN"

    # 解析统计 (ip 输出: RX:/TX: 在上一行，数值在下一行)
    rx_bytes = rx_packets = rx_dropped = rx_errors = 0
    tx_bytes = tx_packets = tx_dropped = tx_errors = 0
    lines = raw.split("\n")
    for i, line in enumerate(lines):
        if "RX:" in line and i + 1 < len(lines):
            parts = [p for p in lines[i + 1].split() if p.isdigit()]
            if len(parts) >= 4:
                rx_bytes, rx_packets, rx_errors, rx_dropped = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
        elif "TX:" in line and i + 1 < len(lines):
            parts = [p for p in lines[i + 1].split() if p.isdigit()]
            if len(parts) >= 4:
                tx_bytes, tx_packets, tx_errors, tx_dropped = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])

    # 队列信息
    tx_queue_len = read_sysfs(iface, "tx_queue_len", "10")
    mtu = read_sysfs(iface, "mtu", "16")

    # berr-counter
    berr_tx = berr_rx = 0
    if "berr-counter" in raw:
        import re
        m = re.search(r"berr-counter tx (\d+) rx (\d+)", raw)
        if m:
            berr_tx, berr_rx = int(m.group(1)), int(m.group(2))

    return {
        "state": state,
        "mtu": mtu,
        "tx_queue_len": tx_queue_len,
        "rx_bytes": rx_bytes,
        "rx_packets": rx_packets,
        "rx_dropped": rx_dropped,
        "rx_errors": rx_errors,
        "tx_bytes": tx_bytes,
        "tx_packets": tx_packets,
        "tx_dropped": tx_dropped,
        "tx_errors": tx_errors,
        "berr_tx": berr_tx,
        "berr_rx": berr_rx,
        "raw": raw,
    }


def print_can_status(iface: str, info: dict, verbose: bool = False):
    """打印 CAN 状态"""
    state_icon = "🟢" if info["state"] == "UP" else "🔴"
    print(f"\n{'='*60}")
    print(f"  {state_icon} {iface}  [{info['state']}]  MTU={info['mtu']}  tx_queue_len={info['tx_queue_len']}")
    print(f"{'='*60}")
    print(f"  RX:  {info['rx_packets']:>12} packets   {info['rx_bytes']:>12} bytes")
    print(f"        dropped={info['rx_dropped']}  errors={info['rx_errors']}")
    print(f"  TX:  {info['tx_packets']:>12} packets   {info['tx_bytes']:>12} bytes")
    print(f"        dropped={info['tx_dropped']}  errors={info['tx_errors']}")
    print(f"  berr-counter: tx={info['berr_tx']}  rx={info['berr_rx']}")
    # 剩余队列容量 (SocketCAN 无直接 API，用 tx_queue_len 表示最大队列长度)
    print(f"  tx_queue_len (max): {info['tx_queue_len']} frames")
    if verbose:
        print(f"\n  --- ip -details -statistics ---")
        for line in info["raw"].split("\n")[1:]:
            print(f"  {line}")


def can_up(iface: str, bitrate: int = 1000000) -> bool:
    """开启 CAN 接口"""
    run_sudo(f"ip link set {iface} down 2>/dev/null")
    subprocess.run("sleep 0.1", shell=True)
    r = run_sudo(f"ip link set {iface} type can bitrate {bitrate}")
    if r.returncode != 0:
        return False
    r = run_sudo(f"ip link set {iface} up")
    return r.returncode == 0


def can_down(iface: str) -> bool:
    """关闭 CAN 接口"""
    r = run_sudo(f"ip link set {iface} down 2>/dev/null")
    return r.returncode == 0


def can_reset(iface: str, bitrate: int = 1000000) -> bool:
    """重置 CAN (down -> up，清理 buffer)"""
    can_down(iface)
    subprocess.run("sleep 0.3", shell=True)
    return can_up(iface, bitrate)


def main():
    parser = argparse.ArgumentParser(
        description="CAN 接口管理 - 查看状态、开启/关闭、清理 buffer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("interfaces", nargs="*", default=DEFAULT_CANS,
                        help=f"CAN 接口 (默认: {' '.join(DEFAULT_CANS)})")
    parser.add_argument("--up", action="store_true", help="开启 CAN")
    parser.add_argument("--down", action="store_true", help="关闭 CAN")
    parser.add_argument("--reset", action="store_true", help="重置 CAN (清理 buffer)")
    parser.add_argument("--bitrate", type=int, default=1000000, help="比特率 (默认 1000000)")
    parser.add_argument("-v", "--verbose", action="store_true", help="显示详细信息")
    parser.add_argument("--all", action="store_true", help="操作系统中所有 CAN 接口")

    args = parser.parse_args()

    if args.all:
        interfaces = get_can_interfaces()
        if not interfaces:
            print("未检测到 CAN 接口")
            return 1
    else:
        interfaces = args.interfaces

    # 执行操作
    if args.down:
        print("\n[关闭 CAN]")
        for iface in interfaces:
            if can_down(iface):
                print(f"  ✓ {iface} 已关闭")
            else:
                print(f"  ✗ {iface} 关闭失败 (可能不存在)")

    if args.up:
        print("\n[开启 CAN]")
        for iface in interfaces:
            if can_up(iface, args.bitrate):
                print(f"  ✓ {iface} 已开启 (bitrate={args.bitrate})")
            else:
                print(f"  ✗ {iface} 开启失败")

    if args.reset:
        print("\n[重置 CAN - 清理 buffer]")
        for iface in interfaces:
            if can_reset(iface, args.bitrate):
                print(f"  ✓ {iface} 已重置")
            else:
                print(f"  ✗ {iface} 重置失败")

    # 显示状态
    print("\n" + "="*60)
    print("  CAN 接口状态")
    print("="*60)

    for iface in interfaces:
        info = get_can_state(iface)
        if info["state"] != "unknown" or args.verbose:
            print_can_status(iface, info, args.verbose)
        else:
            print(f"\n  ⚪ {iface}: 不存在或无法读取")

    print("\n✓ 完成\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
