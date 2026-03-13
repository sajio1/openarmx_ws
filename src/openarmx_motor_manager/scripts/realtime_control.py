#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时关节控制工具 - 拖动滑块实时控制机械臂
"""

import sys
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QGroupBox, QGridLayout, QPushButton
)
from PySide6.QtCore import Qt, QTimer
from openarmx_arm_driver import Robot, get_available_can_interfaces, pair_can_channels


class JointSlider(QWidget):
    """单个关节滑块"""
    def __init__(self, name, parent=None):
        super().__init__(parent)
        self.name = name
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.label = QLabel(f"{name}:")
        self.label.setFixedWidth(40)
        layout.addWidget(self.label)
        
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(-314, 314)  # -3.14 to 3.14 rad
        self.slider.setValue(0)
        layout.addWidget(self.slider)
        
        self.value_label = QLabel("0.00")
        self.value_label.setFixedWidth(50)
        layout.addWidget(self.value_label)
        
        self.slider.valueChanged.connect(self.on_value_changed)
    
    def on_value_changed(self, value):
        rad = value / 100.0
        self.value_label.setText(f"{rad:.2f}")
    
    def get_value(self):
        return self.slider.value() / 100.0
    
    def set_value(self, rad):
        self.slider.setValue(int(rad * 100))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.robot = None
        self.control_enabled = False
        
        self.setWindowTitle("实时关节控制")
        self.setMinimumSize(500, 600)
        
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        
        # 连接按钮
        btn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("连接机器人")
        self.connect_btn.clicked.connect(self.toggle_connect)
        btn_layout.addWidget(self.connect_btn)
        
        self.control_btn = QPushButton("开始控制")
        self.control_btn.clicked.connect(self.toggle_control)
        self.control_btn.setEnabled(False)
        btn_layout.addWidget(self.control_btn)
        
        self.zero_btn = QPushButton("全部归零")
        self.zero_btn.clicked.connect(self.reset_all)
        btn_layout.addWidget(self.zero_btn)
        
        layout.addLayout(btn_layout)
        
        # 右臂
        right_group = QGroupBox("右臂 (Right)")
        right_layout = QVBoxLayout(right_group)
        self.right_sliders = []
        for i in range(1, 9):
            name = f"J{i}" if i < 8 else "EE"
            slider = JointSlider(name)
            right_layout.addWidget(slider)
            self.right_sliders.append(slider)
        layout.addWidget(right_group)
        
        # 左臂
        left_group = QGroupBox("左臂 (Left)")
        left_layout = QVBoxLayout(left_group)
        self.left_sliders = []
        for i in range(1, 9):
            name = f"J{i}" if i < 8 else "EE"
            slider = JointSlider(name)
            left_layout.addWidget(slider)
            self.left_sliders.append(slider)
        layout.addWidget(left_group)
        
        # 状态
        self.status_label = QLabel("未连接")
        layout.addWidget(self.status_label)
        
        # 控制定时器 (50Hz)
        self.timer = QTimer()
        self.timer.timeout.connect(self.send_control)
    
    def toggle_connect(self):
        if self.robot is None:
            self.connect_robot()
        else:
            self.disconnect_robot()
    
    def connect_robot(self):
        self.connect_btn.setEnabled(False)
        self.status_label.setText("正在连接 can2-can3...")
        QApplication.processEvents()
        
        try:
            self.robot = Robot(
                right_can_channel='can2',
                left_can_channel='can3',
                auto_enable_can=True
            )
            
            self.robot.set_mode_all('mit')
            self.robot.enable_all()
            
            self.connect_btn.setText("断开连接")
            self.connect_btn.setEnabled(True)
            self.control_btn.setEnabled(True)
            self.status_label.setText("已连接: can2(右) - can3(左)")
            
        except Exception as e:
            self.status_label.setText(f"连接错误: {e}")
            self.connect_btn.setEnabled(True)
    
    def disconnect_robot(self):
        self.timer.stop()
        self.control_enabled = False
        self.control_btn.setText("开始控制")
        
        if self.robot:
            try:
                self.robot.disable_all()
                self.robot.shutdown()
            except:
                pass
            self.robot = None
        
        self.connect_btn.setText("连接机器人")
        self.control_btn.setEnabled(False)
        self.status_label.setText("已断开")
    
    def toggle_control(self):
        if self.control_enabled:
            self.timer.stop()
            self.control_enabled = False
            self.control_btn.setText("开始控制")
            self.status_label.setText("控制已停止")
        else:
            self.control_enabled = True
            self.control_btn.setText("停止控制")
            self.timer.start(20)  # 50Hz
            self.status_label.setText("控制中 (50Hz)")
    
    def send_control(self):
        if not self.robot or not self.control_enabled:
            return
        
        # 右臂电机映射: GUI索引 -> 实际motor_id  wtf? 难道是底层sdk就有问题？明天查一下
        # J1/J2 物理上是反的
        RIGHT_ARM_MAP = {
            0: 2,  # GUI J1 -> motor_id 2
            1: 1,  # GUI J2 -> motor_id 1
            2: 3,  # J3
            3: 4,  # J4
            4: 5,  # J5
            5: 6,  # J6
            6: 7,  # J7
            7: 8,  # EE
        }
        
        try:
            # 右臂 (使用映射)
            for i, slider in enumerate(self.right_sliders):
                motor_id = RIGHT_ARM_MAP[i]
                pos = slider.get_value()
                self.robot.right_arm.move_joint_mit(
                    motor_id=motor_id,
                    position=pos,
                    kp=10.0,
                    kd=1.0
                )
            
            # 左臂 (正常映射)
            for i, slider in enumerate(self.left_sliders):
                motor_id = i + 1
                pos = slider.get_value()
                self.robot.left_arm.move_joint_mit(
                    motor_id=motor_id,
                    position=pos,
                    kp=10.0,
                    kd=1.0
                )
        except Exception as e:
            self.status_label.setText(f"控制错误: {e}")
    
    def reset_all(self):
        for slider in self.right_sliders + self.left_sliders:
            slider.set_value(0)
    
    def closeEvent(self, event):
        self.disconnect_robot()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
