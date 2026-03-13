#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenArmX 电机校准脚本

用于校准电机零点或检查电机状态。支持真实硬件和模拟模式。

用法:
    # === 真实硬件模式 ===
    # 检查所有电机状态
    python3 calibrate_motors.py --check
    
    # 校准所有电机零点
    python3 calibrate_motors.py --zero
    
    # 只校准左臂 J4
    python3 calibrate_motors.py --zero --arm left --joint 4
    
    # 只校准右臂所有关节
    python3 calibrate_motors.py --zero --arm right
    
    # === 模拟模式 (sim) ===
    # 检查 ROS2 模拟关节状态
    python3 calibrate_motors.py --sim --check
    
    # 发送零点指令到 ROS2 (让模拟机器人回到零点)
    python3 calibrate_motors.py --sim --zero
"""

import argparse
import subprocess
import sys
import time
import atexit

# 配置
CAN_INTERFACES = {
    "right": "can2",
    "left": "can3",
}
SUDO_PASSWORD = "123456" # 上传git的时候别把密码上传了

# 不重要，输密码而已
def run_sudo(cmd: str) -> bool:
    """执行 sudo 命令"""
    full_cmd = f"echo {SUDO_PASSWORD} | sudo -S {cmd}"
    result = subprocess.run(full_cmd, shell=True, capture_output=True, text=True)
    return result.returncode == 0

# if something better?
def cleanup_environment():
    """清理环境：杀掉 ROS2 进程，关闭 CAN 接口"""
    print("\n[0] 清理环境...")
    
    # 杀掉 ROS2 相关进程
    patterns = ["ros2", "rviz", "openarmx", "robot_state_publisher", "controller_manager"]
    killed = []
    
    for pattern in patterns:
        result = subprocess.run(
            f"pkill -9 -f '{pattern}' 2>/dev/null",
            shell=True, capture_output=True
        )
        if result.returncode == 0:
            killed.append(pattern)
    
    if killed:
        print(f"  ✓ 已终止进程: {', '.join(killed)}")
    else:
        print(f"  → 无 ROS2 进程需要清理")
    
    # 关闭 CAN 接口
    for arm, can in CAN_INTERFACES.items():
        run_sudo(f"ip link set {can} down 2>/dev/null")
    print(f"  ✓ CAN 接口已重置")
    
    # 等待资源释放
    time.sleep(1)
    print(f"  ✓ 环境清理完成")


def setup_can():
    """启动 CAN 接口"""
    print("\n[1] 启动 CAN 接口...")
    success = True
    for arm, can in CAN_INTERFACES.items():
        # 先关闭再启动，确保干净状态
        run_sudo(f"ip link set {can} down 2>/dev/null")
        time.sleep(0.1)
        if run_sudo(f"ip link set {can} up type can bitrate 1000000"):
            print(f"  ✓ {can} ({arm}臂) 已启动")
        else:
            print(f"  ✗ {can} ({arm}臂) 启动失败")
            success = False
    return success


def cleanup_can():
    """关闭 CAN 接口"""
    print("\n[清理] 关闭 CAN 接口...")
    for arm, can in CAN_INTERFACES.items():
        run_sudo(f"ip link set {can} down 2>/dev/null")
        print(f"  ✓ {can} ({arm}臂) 已关闭")


def check_motors(arm_filter: str = None):
    """检查电机状态"""
    from openarmx_arm_driver import Arm
    
    print("\n[2] 检查电机状态...")
    print("=" * 60)
    
    arms_to_check = list(CAN_INTERFACES.items())
    if arm_filter:
        arms_to_check = [(arm_filter, CAN_INTERFACES[arm_filter])]
    
    for side, can in arms_to_check:
        print(f"\n【{side.upper()} 臂 ({can})】")
        try:
            arm = Arm(can, side=side, auto_enable_can=False)
            
            print(f"  {'关节':<6} {'角度 (rad)':<15} {'速度':<12} {'状态'}")
            print(f"  {'-'*50}")
            
            for motor_id in [1, 2, 3, 4, 5, 6, 7, 8]:
                info = arm.get_status(motor_id)
                if info:
                    angle = info.get('angle', 0)
                    vel = info.get('velocity', 0)
                    # 判断是否需要校准
                    status = "✓ 正常" if abs(angle) < 0.1 else "⚠ 需要校准"
                    if motor_id == 8:
                        status = "夹爪"
                    print(f"  J{motor_id:<5} {angle:+.4f}         {vel:+.4f}       {status}")
                else:
                    print(f"  J{motor_id:<5} {'无响应':<15}")
            
            arm.close()
        except Exception as e:
            print(f"  错误: {e}")
    
    print("\n" + "=" * 60)


def calibrate_zero(arm_filter: str = None, joint_filter: int = None):
    """校准电机零点"""
    from openarmx_arm_driver import Arm
    
    print("\n[2] 校准电机零点...")
    print("=" * 60)
    
    arms_to_calibrate = list(CAN_INTERFACES.items())
    if arm_filter:
        arms_to_calibrate = [(arm_filter, CAN_INTERFACES[arm_filter])]
    
    for side, can in arms_to_calibrate:
        print(f"\n【{side.upper()} 臂 ({can})】")
        try:
            arm = Arm(can, side=side, auto_enable_can=False)
            
            motors_to_calibrate = [1, 2, 3, 4, 5, 6, 7] if joint_filter is None else [joint_filter]
            
            for motor_id in motors_to_calibrate:
                # 先读取当前角度
                info = arm.get_status(motor_id)
                if info is None:
                    print(f"  ⚠ J{motor_id}: 无响应，跳过")
                    continue
                    
                old_angle = info.get('angle', 0)
                
                # 设置零点
                result = arm.set_zero(motor_id)
                
                if result == 0:
                    print(f"  ✓ J{motor_id}: 零点已设置 (原角度: {old_angle:+.4f} rad)")
                else:
                    print(f"  ✗ J{motor_id}: 设置失败 (code={result})")
                
                time.sleep(0.05)
            
            arm.close()
        except Exception as e:
            print(f"  错误: {e}")
    
    print("\n" + "=" * 60)
    
    # 校准后重新检查
    print("\n[3] 验证校准结果...")
    check_motors(arm_filter)


# ═══════════════════════════════════════════════════════════════════════════════
# 模拟模式 (SIM) - 通过 ROS2 topic 操作
# ═══════════════════════════════════════════════════════════════════════════════

def sim_check_joints(arm_filter: str = None):
    """[SIM] 检查 ROS2 模拟关节状态"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    
    print("\n[SIM] 检查 ROS2 关节状态...")
    print("=" * 60)
    
    rclpy.init()
    node = Node('sim_calibrate_check')
    
    joint_data = {}
    received = [False]
    
    def callback(msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            joint_data[name] = pos
        received[0] = True
    
    node.create_subscription(JointState, '/joint_states', callback, 10)
    
    # 等待数据
    print("  → 等待 /joint_states 数据...")
    timeout = 5.0
    start = time.time()
    while not received[0] and (time.time() - start) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if not received[0]:
        print("  ✗ 未收到数据 (ROS2 是否已启动?)")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # 显示关节状态
    for side in ["right", "left"]:
        if arm_filter and side != arm_filter:
            continue
        print(f"\n【{side.upper()} 臂 (ROS2 模拟)】")
        print(f"  {'关节':<6} {'角度 (rad)':<15} {'状态'}")
        print(f"  {'-'*40}")
        
        for i in range(1, 8):
            name = f"openarmx_{side}_joint{i}"
            if name in joint_data:
                angle = joint_data[name]
                status = "✓ 零点" if abs(angle) < 0.01 else f"偏离 {abs(angle):.3f}"
                print(f"  J{i:<5} {angle:+.4f}         {status}")
            else:
                print(f"  J{i:<5} {'未找到':<15}")
        
        # 夹爪
        gripper_name = f"openarmx_{side}_finger_joint1"
        if gripper_name in joint_data:
            print(f"  EE    {joint_data[gripper_name]:+.4f}         夹爪")
    
    print("\n" + "=" * 60)
    
    node.destroy_node()
    rclpy.shutdown()


def sim_zero_joints(arm_filter: str = None):
    """[SIM] 发送零点指令到 ROS2 forward_position_controller"""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    
    print("\n[SIM] 发送零点指令到 ROS2...")
    print("=" * 60)
    
    rclpy.init()
    node = Node('sim_calibrate_zero')
    
    # 创建发布器
    right_pub = node.create_publisher(
        Float64MultiArray, '/right_forward_position_controller/commands', 10)
    left_pub = node.create_publisher(
        Float64MultiArray, '/left_forward_position_controller/commands', 10)
    
    # 等待发布器就绪
    time.sleep(0.5)
    
    # 零点指令: 7个关节 + 1个夹爪 (夹爪设为全开 0.05)
    zero_cmd = Float64MultiArray()
    zero_cmd.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
    
    # 发送指令
    if arm_filter is None or arm_filter == "right":
        right_pub.publish(zero_cmd)
        print("  ✓ 右臂: 已发送零点指令 [0,0,0,0,0,0,0,0.05]")
    
    if arm_filter is None or arm_filter == "left":
        left_pub.publish(zero_cmd)
        print("  ✓ 左臂: 已发送零点指令 [0,0,0,0,0,0,0,0.05]")
    
    # 持续发送几次确保生效
    for i in range(10):
        if arm_filter is None or arm_filter == "right":
            right_pub.publish(zero_cmd)
        if arm_filter is None or arm_filter == "left":
            left_pub.publish(zero_cmd)
        time.sleep(0.05)
    
    print("\n" + "=" * 60)
    print("  → 零点指令已发送，机器人应该回到零点位置")
    
    node.destroy_node()
    rclpy.shutdown()
    
    # 验证
    print("\n[SIM] 验证零点状态...")
    time.sleep(0.5)
    sim_check_joints(arm_filter)


def main():
    parser = argparse.ArgumentParser(
        description="OpenArmX 电机校准脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    parser.add_argument("--check", action="store_true",
                        help="检查电机状态 (不校准)")
    parser.add_argument("--zero", action="store_true",
                        help="设置电机零点")
    parser.add_argument("--arm", choices=["left", "right"],
                        help="只操作指定臂 (默认: 两臂)")
    parser.add_argument("--joint", type=int, choices=[1,2,3,4,5,6,7,8],
                        help="只操作指定关节 (默认: 所有关节)")
    parser.add_argument("--no-cleanup", action="store_true",
                        help="跳过环境清理 (不推荐)")
    parser.add_argument("--sim", action="store_true",
                        help="模拟模式: 通过 ROS2 topic 操作，不需要真实硬件")
    
    args = parser.parse_args()
    
    # 默认行为：检查状态
    if not args.check and not args.zero:
        args.check = True
    
    # ═══════════════════════════════════════════════════════════════════════
    # 模拟模式 (SIM) - 不需要 CAN，直接通过 ROS2 操作
    # ═══════════════════════════════════════════════════════════════════════
    if args.sim:
        print("╔══════════════════════════════════════════════════════════════╗")
        print("║         OpenArmX 电机校准工具 [SIM 模式]                     ║")
        print("║         通过 ROS2 topic 操作，无需真实硬件                   ║")
        print("╚══════════════════════════════════════════════════════════════╝")
        
        try:
            if args.check:
                sim_check_joints(args.arm)
            elif args.zero:
                sim_zero_joints(args.arm)
            
            print("\n✓ [SIM] 操作完成")
            
        except KeyboardInterrupt:
            print("\n\n⚠ 用户中断")
        except Exception as e:
            print(f"\n✗ 错误: {e}")
            import traceback
            traceback.print_exc()
            return 1
        
        return 0
    
    # ═══════════════════════════════════════════════════════════════════════
    # 真实硬件模式 - 需要 CAN 接口
    # ═══════════════════════════════════════════════════════════════════════
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║              OpenArmX 电机校准工具                           ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    
    # 注册清理函数
    atexit.register(cleanup_can)
    
    try:
        # 先清理环境
        if not args.no_cleanup:
            cleanup_environment()
        
        # 启动 CAN
        if not setup_can():
            print("\n✗ CAN 接口启动失败")
            return 1
        
        # 执行操作
        if args.check:
            check_motors(args.arm)
        elif args.zero:
            calibrate_zero(args.arm, args.joint)
        
        print("\n✓ 操作完成")
        
    except KeyboardInterrupt:
        print("\n\n⚠ 用户中断")
    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
