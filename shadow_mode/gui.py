#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Control 实时关节控制测试工具
通过 ros2_control 的 forward_position_controller 控制电机
用于验证 v10_simple_hardware.cpp 的 J1/J2 修复是否正确

对比测试方法：
1. 运行此脚本，拖动 J1 滑块
2. 观察物理机器人：
   - 如果肩膀上下动 → J1 正确 ✓
   - 如果肩膀左右动 → J1/J2 还是反的 ✗
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QGroupBox, QPushButton, QTextEdit
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject


class ROS2Bridge(QObject):
    """ROS2 和 Qt 之间的桥梁"""
    joint_states_updated = Signal(dict)  # {joint_name: position}
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.right_pub = None
        self.left_pub = None
        self.joint_states = {}
        self.spin_thread = None
        self.running = False
    
    def start(self):
        """启动 ROS2 节点"""
        rclpy.init()
        self.node = rclpy.create_node('ros2_control_test_gui')
        
        # 发布器 - forward_position_controller
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.right_pub = self.node.create_publisher(
            Float64MultiArray, 
            '/right_forward_position_controller/commands', 
            qos
        )
        self.left_pub = self.node.create_publisher(
            Float64MultiArray, 
            '/left_forward_position_controller/commands', 
            qos
        )
        
        # 订阅 joint_states
        self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_cb,
            10
        )
        
        # 后台 spin
        self.running = True
        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()
        
        self.node.get_logger().info("ROS2 Control Test GUI 已启动")
        return True
    
    def _spin_loop(self):
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
    
    def _joint_states_cb(self, msg: JointState):
        """处理 joint_states 回调"""
        states = {}
        for name, pos in zip(msg.name, msg.position):
            states[name] = pos
        self.joint_states = states
        self.joint_states_updated.emit(states)
    
    def send_right_arm(self, positions: list):
        """发送右臂命令 (8个值: 7关节 + 夹爪)"""
        if self.get_external_command_publishers():
            return
        msg = Float64MultiArray()
        msg.data = positions
        self.right_pub.publish(msg)
    
    def send_left_arm(self, positions: list):
        """发送左臂命令"""
        if self.get_external_command_publishers():
            return
        msg = Float64MultiArray()
        msg.data = positions
        self.left_pub.publish(msg)

    def get_external_command_publishers(self) -> list[str]:
        """查询控制 topic 的外部发布者（排除本 GUI 节点自身）.

        返回发布者节点名列表（去重、排序）.
        """
        if not self.node:
            return []

        my_name = self.node.get_name()
        topics = [
            "/right_forward_position_controller/commands",
            "/left_forward_position_controller/commands",
        ]
        external = set()
        for topic in topics:
            try:
                infos = self.node.get_publishers_info_by_topic(topic)
            except Exception:
                infos = []
            for info in infos:
                name = getattr(info, "node_name", "")
                if name and name != my_name:
                    external.add(name)
        return sorted(external)
    
    def shutdown(self):
        self.running = False
        if self.spin_thread and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)
        if self.node:
            self.node.destroy_node()
            self.node = None
        if rclpy.ok():
            rclpy.shutdown()


# ===========================================================================
# [PATCH 2025-02-18] 关节安全限制 (直接来自 URDF)
# 与 ROS2 ros2_control 的限位保持一致
# ===========================================================================
JOINT_LIMITS = {
    # 右臂
    "openarmx_right_joint1": (-1.25, 3.5),
    "openarmx_right_joint2": (-0.05, 3.27),
    "openarmx_right_joint3": (-1.57, 1.57),
    "openarmx_right_joint4": (0.0, 2.4),
    "openarmx_right_joint5": (-1.5, 1.5),
    "openarmx_right_joint6": (-0.75, 0.75),
    "openarmx_right_joint7": (-1.5, 1.5),
    "openarmx_right_finger_joint1": (0.0, 0.05),
    # 左臂
    "openarmx_left_joint1": (-3.34, 1.41),
    "openarmx_left_joint2": (-3.27, 0.05),
    "openarmx_left_joint3": (-1.57, 1.57),
    "openarmx_left_joint4": (0.0, 2.4),
    "openarmx_left_joint5": (-1.5, 1.5),
    "openarmx_left_joint6": (-0.75, 0.75),
    "openarmx_left_joint7": (-1.5, 1.5),
    "openarmx_left_finger_joint1": (0.0, 0.05),
}

# ===========================================================================
# [PATCH 2025-02-18] GUI 滑块语义翻转（暂时禁用，恢复与 URDF 一致）
# mirror=True 时，滑块显示方向翻转，但发送的 ROS2 值仍是 URDF 原始值
# ===========================================================================
JOINT_MIRROR = {
    # 暂时禁用 mirror，保持与 URDF 一致
    # "openarmx_left_joint1": True,
    # "openarmx_left_joint2": True,
}
# [PATCH END]


class JointSlider(QWidget):
    """单个关节滑块"""
    def __init__(self, name, joint_ros_name, parent=None):
        super().__init__(parent)
        self.name = name
        self.joint_ros_name = joint_ros_name  # 如 "openarmx_right_joint1"
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # 关节名
        self.label = QLabel(f"{name}:")
        self.label.setFixedWidth(40)
        layout.addWidget(self.label)
        
        # 滑块
        self.slider = QSlider(Qt.Horizontal)
        
        # ===========================================================================
        # [PATCH 2025-02-18] 按关节设置不同的滑块范围 + mirror 支持
        # ===========================================================================
        self.is_gripper = "finger" in joint_ros_name.lower() or name == "EE"
        
        if joint_ros_name in JOINT_LIMITS:
            min_val, max_val = JOINT_LIMITS[joint_ros_name]
        else:
            min_val, max_val = (-3.14, 3.14) if not self.is_gripper else (0.0, 0.05)
        
        # 检查是否需要 mirror（翻转滑块显示方向）
        self.mirror = JOINT_MIRROR.get(joint_ros_name, False)
        
        if self.is_gripper:
            self.scale = 1000.0
            self.unit = "m"
            self.slider.setRange(int(min_val * self.scale), int(max_val * self.scale))
        else:
            self.scale = 100.0
            self.unit = "rad"
            self.slider.setRange(int(min_val * self.scale), int(max_val * self.scale))
        
        # 如果 mirror，翻转滑块的显示方向
        if self.mirror:
            self.slider.setInvertedAppearance(True)
        
        self.slider.setValue(0)
        self.min_val = min_val
        self.max_val = max_val
        # [PATCH END]
        layout.addWidget(self.slider)
        
        # 命令值 (显示真实的 ROS2 值)
        self.cmd_label = QLabel("cmd: 0.00")
        self.cmd_label.setFixedWidth(80)
        layout.addWidget(self.cmd_label)
        
        # 实际值 (来自 /joint_states)
        self.state_label = QLabel("state: --")
        self.state_label.setFixedWidth(80)
        layout.addWidget(self.state_label)
        
        # mirror 标记显示
        if self.mirror:
            mirror_label = QLabel("◀▶")
            mirror_label.setToolTip("滑块方向已翻转 (mirror)")
            mirror_label.setFixedWidth(20)
            layout.addWidget(mirror_label)
        
        self.slider.valueChanged.connect(self.on_value_changed)
    
    def on_value_changed(self, value):
        # 滑块值直接就是 ROS2 值（mirror 只影响显示方向，不影响数值）
        val = value / self.scale
        self.cmd_label.setText(f"cmd: {val:.3f}")
    
    def get_value(self):
        # 返回 ROS2 原始值
        return self.slider.value() / self.scale
    
    def set_value(self, val):
        """设置滑块值 (val: rad 或 m，ROS2 原始值)"""
        self.slider.blockSignals(True)
        self.slider.setValue(int(val * self.scale))
        self.slider.blockSignals(False)
        self.cmd_label.setText(f"cmd: {val:.3f}")
    
    def update_state(self, rad):
        self.state_label.setText(f"state: {rad:.2f}")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ros2 = None
        self.control_enabled = False
        
        self.setWindowTitle("ROS2 Control 测试 - J1/J2 验证")
        self.setMinimumSize(600, 750)
        
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        
        # 说明
        info = QLabel(
            "⚠️ 测试方法：拖动 J1 滑块，观察物理机器人\n"
            "   - 肩膀上下动 → J1 正确 ✓\n"
            "   - 肩膀左右动 → J1/J2 还是反的 ✗"
        )
        info.setStyleSheet("background: #fff3cd; padding: 10px; border-radius: 5px;")
        layout.addWidget(info)
        
        # 按钮
        btn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("连接 ROS2")
        self.connect_btn.clicked.connect(self.toggle_connect)
        btn_layout.addWidget(self.connect_btn)
        
        self.control_btn = QPushButton("开始控制")
        self.control_btn.clicked.connect(self.toggle_control)
        self.control_btn.setEnabled(False)
        btn_layout.addWidget(self.control_btn)
        
        self.sync_btn = QPushButton("同步当前位置")
        self.sync_btn.clicked.connect(self.sync_positions)
        self.sync_btn.setEnabled(False)
        btn_layout.addWidget(self.sync_btn)
        
        self.zero_btn = QPushButton("全部归零")
        self.zero_btn.clicked.connect(self.reset_all)
        btn_layout.addWidget(self.zero_btn)
        
        layout.addLayout(btn_layout)
        
        # 右臂
        right_group = QGroupBox("右臂 (Right) - 通过 /right_forward_position_controller/commands")
        right_layout = QVBoxLayout(right_group)
        self.right_sliders = []
        right_joint_names = [
            ("J1", "openarmx_right_joint1"),
            ("J2", "openarmx_right_joint2"),
            ("J3", "openarmx_right_joint3"),
            ("J4", "openarmx_right_joint4"),
            ("J5", "openarmx_right_joint5"),
            ("J6", "openarmx_right_joint6"),
            ("J7", "openarmx_right_joint7"),
            ("EE", "openarmx_right_finger_joint1"),
        ]
        for name, ros_name in right_joint_names:
            slider = JointSlider(name, ros_name)
            right_layout.addWidget(slider)
            self.right_sliders.append(slider)
        layout.addWidget(right_group)
        
        # 左臂
        left_group = QGroupBox("左臂 (Left) - 通过 /left_forward_position_controller/commands")
        left_layout = QVBoxLayout(left_group)
        self.left_sliders = []
        left_joint_names = [
            ("J1", "openarmx_left_joint1"),
            ("J2", "openarmx_left_joint2"),
            ("J3", "openarmx_left_joint3"),
            ("J4", "openarmx_left_joint4"),
            ("J5", "openarmx_left_joint5"),
            ("J6", "openarmx_left_joint6"),
            ("J7", "openarmx_left_joint7"),
            ("EE", "openarmx_left_finger_joint1"),
        ]
        for name, ros_name in left_joint_names:
            slider = JointSlider(name, ros_name)
            left_layout.addWidget(slider)
            self.left_sliders.append(slider)
        layout.addWidget(left_group)
        
        # 状态
        self.status_label = QLabel("未连接 ROS2")
        self.status_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.status_label)
        
        # 控制定时器 (50Hz)
        self.timer = QTimer()
        self.timer.timeout.connect(self.send_control)
    
    def toggle_connect(self):
        if self.ros2 is None:
            self.connect_ros2()
        else:
            self.disconnect_ros2()
    
    def connect_ros2(self):
        self.connect_btn.setEnabled(False)
        self.status_label.setText("正在连接 ROS2...")
        QApplication.processEvents()
        
        try:
            self.ros2 = ROS2Bridge()
            self.ros2.joint_states_updated.connect(self.on_joint_states)
            self.ros2.start()
            
            self.connect_btn.setText("断开 ROS2")
            self.connect_btn.setEnabled(True)
            self.control_btn.setEnabled(True)
            self.sync_btn.setEnabled(True)
            self.status_label.setText("已连接 ROS2 - 等待 /joint_states...")
            
        except Exception as e:
            self.status_label.setText(f"连接错误: {e}")
            self.connect_btn.setEnabled(True)
            self.ros2 = None
    
    def disconnect_ros2(self):
        self.timer.stop()
        self.control_enabled = False
        self.control_btn.setText("开始控制")
        
        if self.ros2:
            try:
                self.ros2.shutdown()
            except:
                pass
            self.ros2 = None
        
        self.connect_btn.setText("连接 ROS2")
        self.control_btn.setEnabled(False)
        self.sync_btn.setEnabled(False)
        self.status_label.setText("已断开 ROS2")
    
    def on_joint_states(self, states: dict):
        """更新显示的状态值"""
        for slider in self.right_sliders + self.left_sliders:
            if slider.joint_ros_name in states:
                slider.update_state(states[slider.joint_ros_name])
        
        if not self.control_enabled:
            self.status_label.setText(f"已连接 - 收到 {len(states)} 个关节状态")
    
    def sync_positions(self):
        """将滑块同步到当前关节位置"""
        if not self.ros2 or not self.ros2.joint_states:
            self.status_label.setText("没有收到 joint_states，无法同步")
            return
        
        for slider in self.right_sliders + self.left_sliders:
            if slider.joint_ros_name in self.ros2.joint_states:
                slider.set_value(self.ros2.joint_states[slider.joint_ros_name])
        
        self.status_label.setText("已同步到当前位置")
    
    def toggle_control(self):
        if self.control_enabled:
            self.timer.stop()
            self.control_enabled = False
            self.control_btn.setText("开始控制")
            self.status_label.setText("控制已停止")
        else:
            # 安全互斥: 若已有其他节点发布控制命令，禁止 GUI 抢占
            external = self.ros2.get_external_command_publishers() if self.ros2 else []
            if external:
                names = ", ".join(external)
                self.status_label.setText(f"拒绝启动控制: 检测到外部发布者 [{names}]")
                return
            self.control_enabled = True
            self.control_btn.setText("停止控制")
            self.timer.start(20)  # 50Hz
            self.status_label.setText("控制中 (50Hz) - 通过 ros2_control")
    
    def send_control(self):
        if not self.ros2 or not self.control_enabled:
            return
        
        try:
            # 控制过程中持续检查互斥，避免运行时被外部发布者抢占
            external = self.ros2.get_external_command_publishers()
            if external:
                names = ", ".join(external)
                self.timer.stop()
                self.control_enabled = False
                self.control_btn.setText("开始控制")
                self.status_label.setText(f"控制已自动停止: 外部发布者 [{names}]")
                return

            # 右臂 - 按顺序发送 8 个值
            right_positions = [s.get_value() for s in self.right_sliders]
            self.ros2.send_right_arm(right_positions)
            
            # 左臂
            left_positions = [s.get_value() for s in self.left_sliders]
            self.ros2.send_left_arm(left_positions)
            
        except Exception as e:
            self.status_label.setText(f"控制错误: {e}")
    
    def reset_all(self):
        for slider in self.right_sliders + self.left_sliders:
            slider.set_value(0)
    
    def closeEvent(self, event):
        self.disconnect_ros2()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
