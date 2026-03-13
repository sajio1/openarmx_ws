"""CLI 入口."""

import argparse

import rclpy
from rclpy.executors import MultiThreadedExecutor

from . import config as cfg
from .node import VRTeleopIKNode


def parse_args():
    parser = argparse.ArgumentParser(
        description="VR Teleop IK Control (支持 J4/L4 肘部约束, 见 config.py)")
    parser.add_argument(
        '--no-robot', action='store_true',
        help='仅 IK 仿真, 不发送关节指令')
    parser.add_argument(
        '--viz', action='store_true',
        help='启用 meshcat 3D 可视化')
    parser.add_argument(
        '--scale', type=float, default=cfg.POSITION_SCALE,
        help=f'VR→机器人 位置缩放 (默认: {cfg.POSITION_SCALE})')
    parser.add_argument(
        '--no-clutch', action='store_true',
        help='[调试] 绕过 clutch 检查，收到 VR 数据即开始控制 (危险!)')
    parser.add_argument(
        '--input-unity', action='store_true', default=None,
        help='输入按 Unity 坐标处理 (默认, Quest 原始数据)')
    parser.add_argument(
        '--input-ros', action='store_true',
        help='输入已是 ROS 坐标, 跳过 Unity→ROS 转换')
    parser.add_argument(
        '--rot-offset', action='store_true',
        help='启用 TCP 180°旋转补偿')
    parser.add_argument(
        '--debug-topics', action='store_true',
        help='发布 /debug/* 诊断 topic')
    return parser.parse_args()


def main():
    args = parse_args()
    cfg.POSITION_SCALE = args.scale
    
    # 调试模式: 绕过 clutch 检查
    if args.no_clutch:
        cfg.CLUTCH_BYPASS = True
        print("[警告] CLUTCH_BYPASS=ON, 收到 VR 数据即开始控制!")

    # 输入语义: 默认 Unity 输入 (Quest 原始数据)
    if args.input_ros:
        cfg.INPUT_POSE_IS_ROS = True
        cfg.APPLY_UNITY_TO_ROS = False
        print("[配置] 输入模式: ROS (跳过坐标转换)")
    else:
        cfg.INPUT_POSE_IS_ROS = False
        cfg.APPLY_UNITY_TO_ROS = True
        print("[配置] 输入模式: Unity (启用 Unity→ROS 转换)")

    # 旋转补偿: 默认关闭，按需开启
    cfg.GRIPPER_ROT_OFFSET_ENABLED = args.rot_offset
    print(f"[配置] TCP旋转补偿: {'ON' if cfg.GRIPPER_ROT_OFFSET_ENABLED else 'OFF'}")

    # 调试 topic
    cfg.DEBUG_TOPICS_ENABLED = args.debug_topics
    if cfg.DEBUG_TOPICS_ENABLED:
        print("[配置] 调试topic: ON (/debug/*)")

    rclpy.init()
    node = VRTeleopIKNode(
        send_to_robot=not args.no_robot,
        enable_viz=args.viz,
    )

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.get_logger().info("正在关闭...")
        node.destroy_node()
        rclpy.shutdown()
