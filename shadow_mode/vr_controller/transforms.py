"""Quest 3 (Unity) ↔ ROS 坐标系变换.

Unity 坐标系 (左手系, Quest 应用实际输出):
  X: 右
  Y: 上
  Z: 前 (指向用户前方)

ROS 世界坐标系 (右手系):
  X: 前
  Y: 左
  Z: 上

变换矩阵 M (Unity → ROS):
  | 0  0  1 |     ROS_X =  Unity_Z  (前)
  |-1  0  0 |     ROS_Y = -Unity_X  (左)
  | 0  1  0 |     ROS_Z =  Unity_Y  (上)
  det = -1 (含左→右手系反射)

旋转变换: R_ros = M @ R_unity @ M^T (用矩阵变换保证 roll/pitch/yaw 正确对应)
"""

import numpy as np

# Unity → ROS 坐标变换矩阵 (position: p_ros = M @ p_unity)
# rotation: R_ros = M @ R_unity @ M.T
_UNITY_TO_ROS_M = np.array([
    [0.0,  0.0,  1.0],
    [-1.0, 0.0,  0.0],
    [0.0,  1.0,  0.0]
], dtype=np.float64)

def unity_to_ros_pos(ux: float, uy: float, uz: float) -> np.ndarray:
    """Unity 左手系 (X右, Y上, Z前) → ROS 右手系 (X前, Y左, Z上).

    转换规则:
      ROS_X (前) =  Unity_Z (前)
      ROS_Y (左) = -Unity_X (右的反方向)
      ROS_Z (上) =  Unity_Y (上)
    """
    return np.array([uz, -ux, uy], dtype=np.float64)


def unity_to_ros_quat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Unity 左手系四元数 → ROS 右手系四元数, 返回 (x, y, z, w).

    使用完整变换 R_ros = M @ R_unity @ M^T 保证 roll/pitch/yaw 正确对应,
    避免手部 roll 被错误映射为机器人 yaw 等问题。
    """
    R_u = quat_to_rotmat(qx, qy, qz, qw)
    R_ros = _UNITY_TO_ROS_M @ R_u @ _UNITY_TO_ROS_M.T
    return rotmat_to_quat(R_ros)


def remap_rotation_delta(delta_rot: np.ndarray, is_right_arm: bool = False) -> np.ndarray:
    """对局部旋转增量做轴重映射, 修正 yaw/roll/pitch 轴对应。

    使用共轭变换:
      R' = P @ R @ P^T
    其中 P 为轴重排矩阵。该形式保持单位旋转不变, 不会改变中立姿态。

    右臂镜像修正:
      某些模型/链路在右臂会出现 pitch/yaw 方向镜像问题。
      这里不使用转置 (R^T)，而使用符号矩阵共轭:
        S = diag(1, -1, -1)
        R := S @ R @ S
      该变换等效于在 X 轴保持不变的前提下翻转 Y/Z，从而精准反转 Pitch 和 Yaw，
      不干扰 Roll。
    """
    try:
        from . import config as cfg
        enabled = getattr(cfg, "ROTATION_AXIS_REMAP_ENABLED", False)
        axes = tuple(getattr(cfg, "ROTATION_AXIS_REMAP", (0, 1, 2)))
    except ImportError:
        enabled = False
        axes = (0, 1, 2)

    R = delta_rot
    if enabled and axes != (0, 1, 2):
        P = _axis_remap_matrix(axes)
        R = P @ R @ P.T
    if is_right_arm:
        # X(Roll) 反转, Y(Pitch) 反转保持正确, Z(Yaw) 恢复正常
        S = np.diag([-1.0, -1.0, 1.0]).astype(np.float64)
        R = S @ R @ S
    return R


def rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    """3×3 旋转矩阵 → 四元数 (x, y, z, w)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
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
    return np.array([x, y, z, w], dtype=np.float64)


def quat_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """四元数 (x,y,z,w) → 3×3 旋转矩阵
    
    使用乘法代替幂运算以优化性能
    """
    x2, y2, z2 = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    
    return np.array([
        [1.0 - 2.0*(y2 + z2),       2.0*(xy - wz),       2.0*(xz + wy)],
        [      2.0*(xy + wz), 1.0 - 2.0*(x2 + z2),       2.0*(yz - wx)],
        [      2.0*(xz - wy),       2.0*(yz + wx), 1.0 - 2.0*(x2 + y2)]
    ], dtype=np.float64)


def apply_gripper_offset(rot: np.ndarray) -> np.ndarray:
    """应用末端旋转补偿 (可选, 用于微调手下垂↔机器人初始位姿对应).
    
    默认无补偿: 手下垂在身体两侧时应对应机器人零位姿态。
    若朝向不对, 可在 config.GRIPPER_OFFSET_RPY 中配置 (roll,pitch,yaw) 弧度。
    
    Args:
        rot: 3x3 旋转矩阵 (ROS 坐标系下的目标旋转)
    
    Returns:
        补偿后的旋转矩阵
    """
    offset = get_gripper_offset_rot()
    return rot @ offset


def get_gripper_offset_rot() -> np.ndarray:
    """获取末端旋转补偿矩阵 (从 config 读取 RPY, 默认无补偿)."""
    try:
        from . import config as cfg
        rpy = getattr(cfg, 'GRIPPER_OFFSET_RPY', (0.0, 0.0, 0.0))
    except ImportError:
        rpy = (0.0, 0.0, 0.0)
    if rpy == (0.0, 0.0, 0.0):
        return np.eye(3, dtype=np.float64)
    return _rpy_to_rotmat(rpy[0], rpy[1], rpy[2])


def _rpy_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """RPY (弧度) → 3×3 旋转矩阵 (ROS 惯例: 先 yaw 再 pitch 再 roll)."""
    cr, cp, cy = np.cos(roll), np.cos(pitch), np.cos(yaw)
    sr, sp, sy = np.sin(roll), np.sin(pitch), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]
    ], dtype=np.float64)


def _axis_remap_matrix(axes: tuple[int, int, int]) -> np.ndarray:
    """按 (robot_x, robot_y, robot_z) <- hand_axis 构造置换矩阵."""
    if sorted(axes) != [0, 1, 2]:
        return np.eye(3, dtype=np.float64)
    P = np.zeros((3, 3), dtype=np.float64)
    P[0, axes[0]] = 1.0
    P[1, axes[1]] = 1.0
    P[2, axes[2]] = 1.0
    return P
