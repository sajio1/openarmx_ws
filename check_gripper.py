#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
诊断 Gripper 电机状态
只读取状态，不发送控制命令
"""

from openarmx_arm_driver import Robot

def main():
    print("=" * 50)
    print("Gripper 电机诊断工具 (右臂)")
    print("=" * 50)
    
    robot = None
    try:
        print("\n[1] 连接机器人 (can2)...")
        robot = Robot(
            right_can_channel='can2',
            left_can_channel=None,  # 只连接右臂
            auto_enable_can=False   # CAN 已经启动了
        )
        
        print("[2] 读取所有电机状态...")
        print("\n" + "-" * 50)
        print("右臂电机状态:")
        print("-" * 50)
        
        for motor_id in range(1, 9):
            name = f"J{motor_id}" if motor_id < 8 else "Gripper"
            
            # 尝试多次读取
            status = None
            for retry in range(3):
                status = robot.right_arm.get_status(motor_id)
                if status is not None:
                    break
            
            if status:
                angle = status.get('angle', 'N/A')
                velocity = status.get('velocity', 'N/A')
                torque = status.get('torque', 'N/A')
                error = status.get('error', 'N/A')
                mode = status.get('mode', 'N/A')
                
                if motor_id == 8:
                    print(f"\n  >>> {name} (motor_id={motor_id}) <<<")
                    print(f"      角度:   {angle}")
                    print(f"      速度:   {velocity}")
                    print(f"      力矩:   {torque}")
                    print(f"      错误码: {error}")
                    print(f"      模式:   {mode}")
                    print(f"      原始数据: {status}")
                else:
                    if isinstance(angle, (int, float)):
                        print(f"  {name}: 角度={angle:.4f} rad")
                    else:
                        print(f"  {name}: 角度={angle}")
            else:
                print(f"  {name}: ❌ 无法读取")
        
        print("\n" + "=" * 50)
        print("诊断完成")
        print("=" * 50)
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if robot:
            try:
                robot.cleanup()
            except:
                pass

if __name__ == "__main__":
    main()
