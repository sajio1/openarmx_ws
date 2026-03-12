#!/usr/bin/env python3
"""
诊断脚本：实时读取右臂 motor_id 1 和 motor_id 2 的角度
用于验证 J1/J2 接反问题

使用方法：
  1. 运行此脚本
  2. 左右摇晃右臂的物理 J2（肩部左右摆动）
  3. 观察哪个 motor_id 的数值在大幅变化
"""

import subprocess
import time
import signal
import sys

running = True

def signal_handler(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, signal_handler)

SUDO_PASS = "123456"

def run_sudo(cmd):
    """用密码执行 sudo 命令"""
    result = subprocess.run(
        f"echo {SUDO_PASS} | sudo -S {cmd}",
        shell=True, capture_output=True, text=True
    )
    return result.returncode

def setup_can():
    """启用 CAN 接口"""
    for ch in ["can2", "can3"]:
        print(f"  正在启用 {ch}...")
        run_sudo(f"ip link set {ch} down")
        run_sudo(f"ip link set {ch} type can bitrate 1000000")
        rc = run_sudo(f"ip link set {ch} up")
        if rc == 0:
            print(f"  ✓ {ch} 已启用")
        else:
            print(f"  ✗ {ch} 启用失败")
            sys.exit(1)

def teardown_can():
    """关闭 CAN 接口"""
    for ch in ["can2", "can3"]:
        print(f"  正在关闭 {ch}...")
        run_sudo(f"ip link set {ch} down")
        print(f"  ✓ {ch} 已关闭")

def main():
    global running

    print("=" * 70)
    print("右臂 J1/J2 接线诊断工具 (ROS2 底层版)")
    print("=" * 70)
    print()

    # 启用 CAN
    print("[1/4] 启用 CAN 接口...")
    setup_can()
    print()

    # 导入 openarm 库 (在CAN启用后)
    print("[2/4] 加载 OpenArm CAN 库...")
    try:
        # 使用底层 openarm_can 库
        import openarm.can.socket.openarm as openarm_module
        from openarm.robstride_motor.rs_motor_constants import MotorType, CallbackMode
    except ImportError:
        print("  ✗ 无法导入 openarm 库，尝试使用 openarmx_arm_driver...")
        from openarmx_arm_driver import Robot
        use_driver = True
    else:
        use_driver = False
        print("  ✓ openarm 库加载成功")
    print()

    print("[3/4] 初始化并使能电机...")
    
    if use_driver:
        # 使用 openarmx_arm_driver
        robot = Robot(right_can_channel='can2', left_can_channel='can3', auto_enable_can=False)
        print("  正在使能右臂电机...")
        robot.right_arm.enable_all()
        time.sleep(0.5)
        print("  ✓ 电机已使能")
        
        def read_motors():
            s1 = robot.right_arm.get_status(1)
            s2 = robot.right_arm.get_status(2)
            a1 = s1.get('angle', 0.0) if s1 else 0.0
            a2 = s2.get('angle', 0.0) if s2 else 0.0
            return a1, a2
        
        def cleanup():
            print("  正在失能电机...")
            robot.right_arm.disable_all()
            robot.shutdown()
    else:
        # 使用底层 openarm_can 库
        OpenArm = openarm_module.OpenArm
        
        print("  正在初始化 can2 (右臂)...")
        arm = OpenArm("can2", False)  # can_fd=False
        
        # 初始化电机
        motor_types = [
            MotorType.RS04, MotorType.RS04,  # J1, J2
            MotorType.RS03, MotorType.RS03,  # J3, J4
            MotorType.RS00, MotorType.RS00, MotorType.RS00  # J5, J6, J7
        ]
        motor_ids = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
        arm.init_arm_motors(motor_types, motor_ids, motor_ids)
        arm.init_gripper_motor(MotorType.RS00, 0x08, 0x08)
        
        # 设置回调模式
        arm.set_callback_mode_all(CallbackMode.STATE)
        
        # 使能所有电机
        print("  正在使能电机...")
        arm.enable_all()
        time.sleep(0.5)
        arm.refresh_all()
        arm.recv_all(2000)
        print("  ✓ 电机已使能")
        
        def read_motors():
            arm.refresh_all()
            arm.recv_all(1500)
            motors = arm.get_arm().get_motors()
            if len(motors) >= 2:
                return motors[0].get_position(), motors[1].get_position()
            return 0.0, 0.0
        
        def cleanup():
            print("  正在失能电机...")
            arm.disable_all()
    
    print()
    print("[4/4] 开始读取电机位置...")
    print()
    print("请左右摇晃右臂肩部，观察哪个 motor_id 数值变化大")
    print("按 Ctrl+C 退出")
    print()
    print(f"{'时间':>6s}  |  {'motor_id=1':>14s}  |  {'motor_id=2':>14s}")
    print("-" * 70)

    start = time.time()
    m1_min, m1_max = float('inf'), float('-inf')
    m2_min, m2_max = float('inf'), float('-inf')

    try:
        while running:
            a1, a2 = read_motors()
            
            # 跟踪极值
            m1_min, m1_max = min(m1_min, a1), max(m1_max, a1)
            m2_min, m2_max = min(m2_min, a2), max(m2_max, a2)
            m1_range = m1_max - m1_min
            m2_range = m2_max - m2_min

            elapsed = time.time() - start

            # 条形图
            bar1_len = int(min(abs(a1), 1.5) / 1.5 * 15)
            bar2_len = int(min(abs(a2), 1.5) / 1.5 * 15)
            bar1 = ("+" if a1 >= 0 else "-") + "█" * bar1_len + "░" * (15 - bar1_len)
            bar2 = ("+" if a2 >= 0 else "-") + "█" * bar2_len + "░" * (15 - bar2_len)

            print(
                f"\r{elapsed:6.1f}s  |  "
                f"m1={a1:+7.4f} {bar1} Δ={m1_range:.3f}  |  "
                f"m2={a2:+7.4f} {bar2} Δ={m2_range:.3f}",
                end="", flush=True
            )

            time.sleep(0.02)  # 50Hz

    except KeyboardInterrupt:
        pass
    
    print("\n")
    print("=" * 70)
    print("统计结果:")
    print(f"  motor_id=1: 范围 [{m1_min:+.4f}, {m1_max:+.4f}], 变化幅度 Δ = {m1_range:.4f} rad ({m1_range*57.3:.1f}°)")
    print(f"  motor_id=2: 范围 [{m2_min:+.4f}, {m2_max:+.4f}], 变化幅度 Δ = {m2_range:.4f} rad ({m2_range*57.3:.1f}°)")
    print()
    if m1_range > m2_range * 2:
        print("  → motor_id=1 变化大，对应你摇晃的物理关节")
    elif m2_range > m1_range * 2:
        print("  → motor_id=2 变化大，对应你摇晃的物理关节")
    else:
        print("  → 两者变化相近，请更明显地只摇晃一个关节")
    print("=" * 70)
    print()

    print("[清理] 关闭...")
    cleanup()
    teardown_can()
    print("\n✓ 全部完成")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        print(f"\n错误: {e}")
        traceback.print_exc()
        try:
            teardown_can()
        except:
            pass
        sys.exit(1)
