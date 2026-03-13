"""所有可调参数集中管理."""

from pathlib import Path

# ── 路径 ──
# vr_controller 现在在 shadow_mode/ 下，所以需要上移两级到 workspace
WORKSPACE = Path(__file__).resolve().parent.parent.parent

URDF_PATH = str(
    WORKSPACE / "install" / "openarmx_description" / "share"
    / "openarmx_description" / "urdf" / "robot" / "openarmx_bimanual_sim.urdf"
)

MESH_PACKAGE_PATH = str(
    WORKSPACE / "install" / "openarmx_description" / "share"
)

# ── 控制 ──
CONTROL_HZ      = 90      # 控制循环频率 (Hz)
POSITION_SCALE  = 1.0 # VR → 机器人位置缩放
GRIPPER_MAX_M   = 0.05    # 夹爪全开行程 (米)
PINCH_DEAD_LO   = 0.05    # pinch 下死区
PINCH_DEAD_HI   = 0.98    # pinch 上死区

# ── 输入 pose 语义 ──
# Quest 应用 (ROSBridgeConnection.cs) 默认 useROS2Coordinates=true，
# 在 Unity 端通过 ROSCoordinateConverter.UnityToROS() 已做过 Unity→ROS 转换,
# 所以到达 /quest3/*_hand_pose 的数据已经是 ROS 右手系 (X前, Y左, Z上)。
# 不需要在 ROS 侧再次转换，否则会导致 **双重转换** 造成轴混乱。
#
# 若 Quest 端关闭了 useROS2Coordinates (发原始 Unity 坐标)，
# 则设 INPUT_POSE_IS_ROS=False, APPLY_UNITY_TO_ROS=True。
INPUT_POSE_IS_ROS = True
APPLY_UNITY_TO_ROS = False

# ── 位置轴修正 ──
# Quest 端 ROSCoordinateConverter 输出的"ROS 坐标"是 (X前, Y左, Z上),
# 但如果实际观察到"上→左、左→上"，说明 Y/Z 和机器人约定对调了。
# 设为 True 时，接收后交换 Y 和 Z 分量。
POSITION_SWAP_YZ = False

# ═══════════════════════════════════════════════════════════════════════════════
# 末端旋转补偿 (手下垂 ↔ 机器人初始位姿)
# ═══════════════════════════════════════════════════════════════════════════════
#
# 设计目标: 手下垂自然置于身体两侧时, 应对应机器人零位姿态 (双臂下垂).
# 默认不做固定补偿, 旋转由 Unity→ROS 变换正确映射.
#
# 若手下垂时夹爪朝向仍有偏差, 可启用补偿并配置 GRIPPER_OFFSET_RPY.
# 例如: (3.14159, 0, 0) = 绕 X 轴 180° (旧版 "手掌平放=夹爪朝下" 行为)
#
# ═══════════════════════════════════════════════════════════════════════════════
GRIPPER_ROT_OFFSET_ENABLED = False  # 默认关闭, 手下垂=机器人初始位姿
GRIPPER_OFFSET_RPY = (0.0, 0.0, 0.0)  # 补偿旋转 (roll, pitch, yaw) 弧度

# ── 旋转轴重映射 (用于修正 yaw/roll/pitch 对应关系) ──
# 目的: 当出现“手做 yaw, 机器人做 roll”等轴混用时, 对局部旋转增量做轴重排。
# 映射含义: ROTATION_AXIS_REMAP = (a, b, c)
#   表示 机器人(X,Y,Z) 轴分别对应 手部(a,b,c) 轴, 其中 0=x,1=y,2=z。
# 例如 (2,0,1): 机器人X←手Z, 机器人Y←手X, 机器人Z←手Y
# 当前现象: 手 roll → 机器人 pitch，手 pitch → 机器人 yaw（轴错位）
# 映射 (2,0,1): 机器人 X←手Z, Y←手X, Z←手Y，修正 roll/pitch/yaw 对应
ROTATION_AXIS_REMAP_ENABLED = True
ROTATION_AXIS_REMAP = (2, 0, 1)

# ── 调试输出 ──
DEBUG_TOPICS_ENABLED = True   # 发布 /debug/* 诊断 topic

# ── 安全 ──
DATA_TIMEOUT_SEC = 0.2    # 数据超时 → 冻结 (秒)
MAX_JUMP_M       = 0.15   # 位置跳变阈值 → 重新校准 (米)

# ═══════════════════════════════════════════════════════════════════════════════
# 平滑限速参数 (调试重点区域)
# ═══════════════════════════════════════════════════════════════════════════════
#
# 两种平滑机制可独立开关:
#   1. EMA 低通滤波: 平滑 VR 抖动，减少高频噪声
#   2. 速度限制: 限制关节每帧最大变化量，防止突变
#
# 调试建议:
#   - 先只开 EMA，调 SMOOTH_ALPHA 到手感舒适
#   - 如果还有突变，再开速度限制
#   - 两者可以叠加使用
#
# ═══════════════════════════════════════════════════════════════════════════════

# ── EMA 低通滤波 ──
# 指数移动平均: new = alpha * ik_result + (1-alpha) * last
# alpha 越小越平滑，但延迟越大
SMOOTH_ENABLED  = True    # 是否启用 EMA 平滑
SMOOTH_ALPHA    = 0.3     # 平滑系数 [0.1~1.0]
                          #   0.1 = 非常平滑，延迟大 (适合慢速精细操作)
                          #   0.3 = 较平滑，中等延迟 (推荐起始值)
                          #   0.5 = 轻微平滑，延迟小
                          #   1.0 = 无平滑 (直接使用 IK 结果)

# ── 关节速度限制 ──
# 限制每帧关节角度最大变化量 (rad/frame)
# 实际速度 = MAX_JOINT_DELTA * CONTROL_HZ (rad/s)
# 例: 0.03 rad/frame * 90 Hz = 2.7 rad/s ≈ 155 deg/s
VELOCITY_LIMIT_ENABLED = True   # 是否启用速度限制
MAX_JOINT_DELTA        = 0.03   # 每帧最大变化量 (rad)
                                #   0.01 = 慢速 (~0.9 rad/s = 52 deg/s)
                                #   0.03 = 中速 (~2.7 rad/s = 155 deg/s) 推荐
                                #   0.05 = 快速 (~4.5 rad/s = 258 deg/s)
                                #   0.10 = 很快 (~9 rad/s = 516 deg/s)

# ── 夹爪平滑 (单独控制) ──
GRIPPER_SMOOTH_ALPHA   = 0.2    # 夹爪平滑系数 (可以比关节更平滑)

# ═══════════════════════════════════════════════════════════════════════════════
# Clutch 安全机制 (Snapshot on Trigger + Gain Ramp)
# ═══════════════════════════════════════════════════════════════════════════════
#
# 工作原理:
#   1. Clutch 按下瞬间 → 捕获当前手柄位姿为参考原点 (Snapshot)
#   2. 增益从 0 渐增到 1 (Gain Ramp) → 防止初始抖动
#   3. 机器人从当前物理位置平滑起步，不会"瞬移"
#
# 这样即使举着手按下 clutch，位移增量初始为 0，机器人不会跳动
#
# ═══════════════════════════════════════════════════════════════════════════════

# ── Clutch 话题 ──
# Quest 3 通过 rosbridge 发布的 clutch 状态
# 单个 clutch 同时控制双臂
CLUTCH_TOPIC = '/quest3/clutch'   # std_msgs/Bool: data=true/false

# ── 调试: 禁用 Clutch 检查 ──
# 设为 True 时，不需要按 clutch 就能控制 (仅用于调试！)
# 正式使用时务必设为 False
CLUTCH_BYPASS = False

# ── 增益淡入 (Gain Ramp) ──
# clutch 按下后，控制增益从 0 线性增加到 1 的时间
GAIN_RAMP_ENABLED  = True     # 是否启用增益淡入
GAIN_RAMP_DURATION = 0.5      # 淡入时长 (秒)
                              #   0.3 = 快速淡入
                              #   0.5 = 推荐值
                              #   1.0 = 慢速淡入 (更安全但延迟感明显)

# ── Clutch 松开自动归零 ──
# 当 clutch 松开时，机器人平滑回到零点 (所有关节 = 0)
# 这是安全功能：VR 传感器出问题时，松开 clutch 即可让机器人归零
RETURN_TO_ZERO_ENABLED = True   # 是否启用自动归零
RETURN_TO_ZERO_SPEED   = 0.02   # 归零速度 (rad/frame)
                                # 实际速度 = RETURN_TO_ZERO_SPEED * CONTROL_HZ (rad/s)
                                # 0.02 * 90 = 1.8 rad/s ≈ 103 deg/s
                                #   0.01 = 慢速归零 (~0.9 rad/s)
                                #   0.02 = 中速归零 (~1.8 rad/s) 推荐
                                #   0.05 = 快速归零 (~4.5 rad/s)
RETURN_TO_ZERO_THRESHOLD = 0.01 # 归零完成阈值 (rad), 小于此值视为已归零

# ═══════════════════════════════════════════════════════════════════════════════

# ── Home / 起始姿态 (默认姿态) ──
# 说明:
#   - 该姿态用于“未开始 VR 控制时的保持姿态”和“Clutch OFF 自动归位”的目标。
#   - 不影响 IK 的 neutral regularization（ARM_NEUTRAL_*）与关节限位（JOINT_LIMITS_*）。
#
# 需求: 起始姿态从“手臂下垂”改为“手臂向前伸直”，近似做法是仅旋转 J1 90°。
# 由于左右臂关节方向镜像，这里使用左右相反符号：
#   - 右臂 J1 = +pi/2
#   - 左臂 J1 = -pi/2
HOME_POSE_LEFT_JOINTS  = [-1.57079632679, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
HOME_POSE_RIGHT_JOINTS = [+1.57079632679, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# ── IK 求解器 ──
IK_STEPS    = 10       # 每帧迭代次数
EE_WEIGHT   = 500.0    # 末端权重  (高 = 必须到达)
EE_GAIN     = 100.0    # 末端增益

# ═══════════════════════════════════════════════════════════════════════════════
# 关节锁定 (调试用)
# ═══════════════════════════════════════════════════════════════════════════════
#
# 用于调试坐标系时，锁定部分关节，只测试特定关节的运动
# 锁定的关节会保持在 0 位置
#
# 例: LOCKED_JOINTS = [3, 4, 5, 6, 7] → 只有 J1, J2 可以动
#     LOCKED_JOINTS = [] → 所有关节自由 (正常模式)
#
# ═══════════════════════════════════════════════════════════════════════════════
LOCKED_JOINTS = []  # 所有关节自由 (正常模式)
                                 # 设为 [] 恢复正常

# ── 肘部约束 ──
# "none" = 无约束 (完全自由)
# "j4"   = 关节角度约束 (joints_task), 更柔和
# "l4"   = 空间位置约束 (frame_task), 更刚性
# "pole" = 人体式肘部极向约束 + 中性姿态最小 effort (推荐)
#
# pole 方案思路:
#   1. 用 shoulder→wrist 几何关系推导肘部圆弧
#   2. 用 ELBOW_POLE_VECTOR 选择圆弧上“肘尖”朝向最接近的点
#   3. 再用 ARM_NEUTRAL_JOINT_TARGETS 轻柔拉回中性姿态, 减少多余扭动
#   4. 通过 ELBOW_SINGULARITY_MARGIN 保留一点弯曲，避免手臂打直到奇异点
ELBOW_CONSTRAINT = "pole"

# J4 约束参数 (仅当 ELBOW_CONSTRAINT="j4" 时使用)
JOINT3_TARGET    = 0.0    # joint3 目标角度 (rad)
JOINT4_TARGET    = 0.087  # joint4 目标角度 (rad), ~5 degrees
JOINTS_WEIGHT    = 1.0    # 关节约束权重 (低 = 柔性引导)

# ── L4 位置约束参数 (仅当 ELBOW_CONSTRAINT="l4" 时使用) ──
ELBOW_W_POS = 1.0      # 肘部位置权重
ELBOW_W_ROT = 0.01     # 肘部旋转权重
ELBOW_BACK  = 0.15     # 肘部偏好: 向后  (m)
ELBOW_SIDE  = 0.15     # 肘部偏好: 向外侧 (m)
ELBOW_UP    = 0.10     # 肘部偏好: 向上   (m)

# ── Pole / human-like 肘部约束参数 (仅当 ELBOW_CONSTRAINT="pole" 时使用) ──
# ELBOW_POLE_VECTOR 定义在 body/vr_reference 语义下:
#   (forward, outward, up)
# 其中 outward 始终表示“远离身体外侧”，左右臂会自动镜像。
#
# 常用示例:
#   下垂自然:  (0.0, 0.2, -1.0)
#   向外展开:  (0.0, 1.0,  0.0)
#   向上外展:  (0.2, 0.8,  1.0)   # 类似 Dr. Octopus
#   斜下外展:  (0.2, 0.6, -0.8)
ELBOW_POLE_W_POS = 8.0
ELBOW_POLE_W_ROT = 0.001
ELBOW_POLE_VECTOR = (0.0, 0.25, -1.0)
ELBOW_SINGULARITY_MARGIN = 0.03  # 预留末端可达距离 (m), 防止手臂完全打直

# 中性姿态 regularization: 低权重“省力姿态”
# 两种模式 (二选一):
#   A) 中性 = 关节灵活度中点 (推荐): 设 USE_NEUTRAL_AS_MIDPOINT = True，并配置
#      JOINT_LIMITS_LEFT / JOINT_LIMITS_RIGHT，IK 会用 (min+max)/2 作为每臂每关节的中性目标。
#   B) 中性 = 固定零位: 设 USE_NEUTRAL_AS_MIDPOINT = False，使用 ARM_NEUTRAL_JOINT_TARGETS
#      （左右臂共用同一组值，如 0 或自定义）。
#
# 关节限位 (rad)，与 URDF / shadow_mode/gui.py 一致，用于计算“灵活度中点”。
JOINT_LIMITS_LEFT = {
    1: (-3.34, 1.41),
    2: (-3.27, 0.05),
    3: (-1.57, 1.57),
    4: (0.0, 2.4),
    5: (-1.5, 1.5),
    6: (-0.75, 0.75),
    7: (-1.5, 1.5),
}
JOINT_LIMITS_RIGHT = {
    1: (-1.25, 3.5),
    2: (-0.05, 3.27),
    3: (-1.57, 1.57),
    4: (0.0, 2.4),
    5: (-1.5, 1.5),
    6: (-0.75, 0.75),
    7: (-1.5, 1.5),
}

# True = 中性取 JOINT_LIMITS_* 的中点 (每臂各自); False = 使用 ARM_NEUTRAL_JOINT_TARGETS
USE_NEUTRAL_AS_MIDPOINT = True

ARM_NEUTRAL_WEIGHT = 0.2
# 仅当 USE_NEUTRAL_AS_MIDPOINT = False 时使用
ARM_NEUTRAL_JOINT_TARGETS = {
    1: 0.0,
    2: 0.0,
    3: 0.0,
    4: 0.35,
    5: 0.0,
    6: 0.0,
    7: 0.0,
}

# ── URDF Frame 名称 ──
# 末端执行器: hand_tcp (夹爪末端), 相对于 link7 偏移 8cm
LEFT_EE_FRAME      = "openarmx_left_hand_tcp"
RIGHT_EE_FRAME     = "openarmx_right_hand_tcp"
LEFT_SHOULDER_FRAME = "openarmx_left_link2"
RIGHT_SHOULDER_FRAME = "openarmx_right_link2"
LEFT_ELBOW_FRAME   = "openarmx_left_link4"
RIGHT_ELBOW_FRAME  = "openarmx_right_link4"

# ═══════════════════════════════════════════════════════════════════════════════
# VR 参考坐标系 (IK 目标的参考原点)
# ═══════════════════════════════════════════════════════════════════════════════
#
# 问题: 直接用 world frame 作为参考，VR 动作映射不直观
# 解决: 创建一个位于两臂中心的参考 frame，VR 动作相对于此 frame
#
# 从 URDF 读取的手臂基座位置:
#   右臂: (0.0, -0.031, 0.698)
#   左臂: (0.0,  0.031, 0.698)
#   中心: (0.0,  0.0,   0.698)
#
# ═══════════════════════════════════════════════════════════════════════════════
VR_REFERENCE_FRAME = "vr_reference"      # TF frame 名称
VR_REFERENCE_XYZ   = (0.0, 0.0, 0.698)   # 相对于 world 的位置 (两臂中心)
VR_REFERENCE_RPY   = (0.0, 0.0, 0.0)     # 相对于 world 的旋转 (与 world 对齐)

# ── 关节名前缀 ──
LEFT_JOINT_PREFIX  = "openarmx_left_joint"
RIGHT_JOINT_PREFIX = "openarmx_right_joint"

# ── 日志 ──
STATUS_LOG_SEC = 3.0

# ── 硬件补偿 ──
# 右臂硬件 CAN ID 接线问题: motor_id 1 实际控制物理 J2, motor_id 2 实际控制物理 J1
# [2025-02-18 PATCH] 已在 v10_simple_hardware.cpp 硬件层修复，无需软件补偿
RIGHT_ARM_J1_J2_SWAP = False
