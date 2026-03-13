#!/usr/bin/env python3
"""TF 可视化调试工具.

在 RViz 中显示：
1. 肩膀 (link2) 和肘部 (link4) 的坐标系
2. 末端执行器位置的 Quest 3 坐标系（与 ROS 坐标系对比）

用法:
    python3 debug_tf_visualizer.py

然后在 RViz 中:
    1. 添加 TF 显示
    2. 只勾选需要的 frames:
       - openarmx_right_link2 (右肩)
       - openarmx_right_link4 (右肘)
       - openarmx_left_link2 (左肩)
       - openarmx_left_link4 (左肘)
       - quest_right_hand (Quest 坐标系)
       - quest_left_hand (Quest 坐标系)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np


class TFVisualizerNode(Node):
    """发布调试用 TF frames."""
    
    def __init__(self):
        super().__init__('debug_tf_visualizer')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ── Quest 坐标系相对于 ROS 坐标系的旋转 ──
        # Quest 3: X右, Y上, Z后
        # ROS:     X前, Y左, Z上
        #
        # 在 ROS 坐标系中，Quest 各轴的方向:
        #   Quest X (右)  → ROS -Y = (0, -1, 0)
        #   Quest Y (上)  → ROS +Z = (0, 0, 1)
        #   Quest Z (后)  → ROS -X = (-1, 0, 0)
        self.quest_quat = self._rotation_matrix_to_quat(np.array([
            [ 0,  0, -1],  # Quest X in ROS
            [-1,  0,  0],  # Quest Y in ROS
            [ 0,  1,  0],  # Quest Z in ROS
        ]).T)  # 转置因为列向量是轴
        
        # 定时发布 (10 Hz)
        self.timer = self.create_timer(0.1, self._publish_tf)
        
        self.get_logger().info('=== TF 可视化调试工具 ===')
        self.get_logger().info('发布的 frames:')
        self.get_logger().info('  quest_right_hand (Quest坐标系, 挂载在 openarmx_right_hand_tcp)')
        self.get_logger().info('  quest_left_hand  (Quest坐标系, 挂载在 openarmx_left_hand_tcp)')
        self.get_logger().info('')
        self.get_logger().info('RViz 中推荐只显示以下 frames:')
        self.get_logger().info('  ✓ vr_reference         (VR参考原点, 两臂中心)')
        self.get_logger().info('  ✓ openarmx_right_link2 (右肩)')
        self.get_logger().info('  ✓ openarmx_right_link4 (右肘)')
        self.get_logger().info('  ✓ openarmx_left_link2  (左肩)')
        self.get_logger().info('  ✓ openarmx_left_link4  (左肘)')
        self.get_logger().info('  ✓ openarmx_right_hand_tcp (右手TCP)')
        self.get_logger().info('  ✓ openarmx_left_hand_tcp  (左手TCP)')
        self.get_logger().info('  ✓ quest_right_hand     (Quest坐标系)')
        self.get_logger().info('  ✓ quest_left_hand      (Quest坐标系)')
        self.get_logger().info('')
        self.get_logger().info('坐标系颜色: R=X轴, G=Y轴, B=Z轴')
        self.get_logger().info('  ROS:   X前(红), Y左(绿), Z上(蓝)')
        self.get_logger().info('  Quest: X右(红), Y上(绿), Z后(蓝)')
    
    def _rotation_matrix_to_quat(self, R: np.ndarray) -> tuple:
        """旋转矩阵 → 四元数 (x, y, z, w)"""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return (x, y, z, w)
    
    def _publish_tf(self):
        """发布 Quest 坐标系 TF."""
        now = self.get_clock().now().to_msg()
        
        # 右手 Quest 坐标系 (挂载在 TCP)
        t_right = TransformStamped()
        t_right.header.stamp = now
        t_right.header.frame_id = 'openarmx_right_hand_tcp'
        t_right.child_frame_id = 'quest_right_hand'
        t_right.transform.translation.x = 0.05  # 稍微偏移，便于观察
        t_right.transform.translation.y = 0.0
        t_right.transform.translation.z = 0.0
        t_right.transform.rotation.x = self.quest_quat[0]
        t_right.transform.rotation.y = self.quest_quat[1]
        t_right.transform.rotation.z = self.quest_quat[2]
        t_right.transform.rotation.w = self.quest_quat[3]
        
        # 左手 Quest 坐标系 (挂载在 TCP)
        t_left = TransformStamped()
        t_left.header.stamp = now
        t_left.header.frame_id = 'openarmx_left_hand_tcp'
        t_left.child_frame_id = 'quest_left_hand'
        t_left.transform.translation.x = 0.05
        t_left.transform.translation.y = 0.0
        t_left.transform.translation.z = 0.0
        t_left.transform.rotation.x = self.quest_quat[0]
        t_left.transform.rotation.y = self.quest_quat[1]
        t_left.transform.rotation.z = self.quest_quat[2]
        t_left.transform.rotation.w = self.quest_quat[3]
        
        self.tf_broadcaster.sendTransform([t_right, t_left])


def main():
    rclpy.init()
    node = TFVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
