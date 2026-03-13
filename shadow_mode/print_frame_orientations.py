#!/usr/bin/env python3
"""打印机器人每个 frame 相对于世界坐标系的旋转矩阵."""

import os
import sys
import numpy as np

# 设置路径
WORKSPACE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, WORKSPACE)

# 设置 ROS_PACKAGE_PATH
MESH_PACKAGE_PATH = os.path.join(WORKSPACE, "install", "openarmx_description", "share")
if "ROS_PACKAGE_PATH" not in os.environ:
    os.environ["ROS_PACKAGE_PATH"] = MESH_PACKAGE_PATH
elif MESH_PACKAGE_PATH not in os.environ["ROS_PACKAGE_PATH"]:
    os.environ["ROS_PACKAGE_PATH"] = MESH_PACKAGE_PATH + ":" + os.environ["ROS_PACKAGE_PATH"]

import placo

URDF_PATH = os.path.join(
    WORKSPACE, "install", "openarmx_description", "share",
    "openarmx_description", "urdf", "robot", "openarmx_bimanual_sim.urdf"
)


def rotation_matrix_to_axis_info(R: np.ndarray) -> dict:
    """将旋转矩阵转换为轴向信息."""
    # R 的列向量是局部坐标系的轴在世界坐标系中的方向
    x_axis = R[:, 0]  # 局部 X 轴在世界系中的方向
    y_axis = R[:, 1]  # 局部 Y 轴在世界系中的方向
    z_axis = R[:, 2]  # 局部 Z 轴在世界系中的方向
    
    def axis_to_world(v):
        """将单位向量转换为世界轴名称."""
        abs_v = np.abs(v)
        max_idx = np.argmax(abs_v)
        sign = "+" if v[max_idx] > 0 else "-"
        axis_names = ["X", "Y", "Z"]
        if abs_v[max_idx] > 0.99:  # 接近对齐
            return f"{sign}{axis_names[max_idx]}"
        else:
            return f"({v[0]:+.2f}, {v[1]:+.2f}, {v[2]:+.2f})"
    
    return {
        "local_X": axis_to_world(x_axis),
        "local_Y": axis_to_world(y_axis),
        "local_Z": axis_to_world(z_axis),
    }


def main():
    print("=" * 80)
    print("机器人各 Frame 相对于世界坐标系的旋转矩阵")
    print("=" * 80)
    print()
    print("世界坐标系 (ROS): X=前, Y=左, Z=上")
    print()
    
    # 加载机器人
    robot = placo.RobotWrapper(URDF_PATH, placo.Flags.collision_as_visual)
    robot.update_kinematics()
    
    # 要打印的 frames
    frames = [
        # 右臂
        ("openarmx_right_link0", "右臂基座"),
        ("openarmx_right_link1", "右臂 J1"),
        ("openarmx_right_link2", "右臂 J2 (肩)"),
        ("openarmx_right_link3", "右臂 J3"),
        ("openarmx_right_link4", "右臂 J4 (肘)"),
        ("openarmx_right_link5", "右臂 J5"),
        ("openarmx_right_link6", "右臂 J6"),
        ("openarmx_right_link7", "右臂 J7 (腕)"),
        ("openarmx_right_hand_tcp", "右手 TCP"),
        # 左臂
        ("openarmx_left_link0", "左臂基座"),
        ("openarmx_left_link1", "左臂 J1"),
        ("openarmx_left_link2", "左臂 J2 (肩)"),
        ("openarmx_left_link3", "左臂 J3"),
        ("openarmx_left_link4", "左臂 J4 (肘)"),
        ("openarmx_left_link5", "左臂 J5"),
        ("openarmx_left_link6", "左臂 J6"),
        ("openarmx_left_link7", "左臂 J7 (腕)"),
        ("openarmx_left_hand_tcp", "左手 TCP"),
    ]
    
    print("-" * 80)
    print(f"{'Frame':<30} {'位置 (X, Y, Z)':<25} {'局部轴 → 世界轴'}")
    print("-" * 80)
    
    for frame_name, desc in frames:
        try:
            T = robot.get_T_world_frame(frame_name)
            pos = T[:3, 3]
            rot = T[:3, :3]
            
            axis_info = rotation_matrix_to_axis_info(rot)
            
            print(f"\n{desc} ({frame_name})")
            print(f"  位置: ({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f})")
            print(f"  局部 X → 世界 {axis_info['local_X']}")
            print(f"  局部 Y → 世界 {axis_info['local_Y']}")
            print(f"  局部 Z → 世界 {axis_info['local_Z']}")
            print(f"  旋转矩阵:")
            print(f"    [{rot[0,0]:+.4f}  {rot[0,1]:+.4f}  {rot[0,2]:+.4f}]")
            print(f"    [{rot[1,0]:+.4f}  {rot[1,1]:+.4f}  {rot[1,2]:+.4f}]")
            print(f"    [{rot[2,0]:+.4f}  {rot[2,1]:+.4f}  {rot[2,2]:+.4f}]")
            
        except Exception as e:
            print(f"\n{desc} ({frame_name}): 获取失败 - {e}")
    
    print()
    print("=" * 80)
    print("VR 参考坐标系 (vr_reference)")
    print("=" * 80)
    print("  位置: (0.0, 0.0, 0.698) - 两臂中心")
    print("  旋转: 与世界坐标系对齐 (单位矩阵)")
    print()


if __name__ == "__main__":
    main()
