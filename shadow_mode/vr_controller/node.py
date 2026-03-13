"""ROS2 节点: 订阅 VR 数据 → IK 求解 → 发布关节指令."""

import time
import threading
import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32, Float64MultiArray, Bool
from tf2_ros import StaticTransformBroadcaster

from . import config as cfg
from .transforms import unity_to_ros_pos, unity_to_ros_quat, quat_to_rotmat
from .tracker import ArmTracker, TrackingState
from .ik_solver import IKSolver


class VRTeleopIKNode(Node):
    """VR 遥操作 IK 控制节点.

    数据流:
      Quest 3 → /quest3/* → 坐标变换 → IK → /forward_position_controller/commands
    """

    def __init__(self, send_to_robot: bool = True, enable_viz: bool = False):
        super().__init__('vr_teleop_ik_node')
        self.send_to_robot = send_to_robot
        self.enable_viz = enable_viz

        # 追踪器
        self.right = ArmTracker("right")
        self.left = ArmTracker("left")

        # IK (后台加载)
        self.ik: IKSolver | None = None
        self.ik_ready = False
        self.ik_lock = threading.Lock()
        self.last_status_time = 0.0

        # ── 订阅 Quest 3 ──
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            PoseStamped, '/quest3/right_hand_pose',
            self._on_right_pose, sub_qos)
        self.create_subscription(
            PoseStamped, '/quest3/left_hand_pose',
            self._on_left_pose, sub_qos)
        self.create_subscription(
            Float32, '/quest3/right_gripper',
            self._on_right_grip, sub_qos)
        self.create_subscription(
            Float32, '/quest3/left_gripper',
            self._on_left_grip, sub_qos)

        # ══════════════════════════════════════════════════════════════
        # 订阅 Clutch (离合器) 状态 - 单个 clutch 同时控制双臂
        # Clutch 按下 → Snapshot on Trigger + Gain Ramp
        # ══════════════════════════════════════════════════════════════
        clutch_topic = getattr(cfg, 'CLUTCH_TOPIC', '/quest3/clutch')
        
        self.create_subscription(
            Bool, clutch_topic,
            self._on_clutch, sub_qos)

        # ── 发布关节指令 ──
        if self.send_to_robot:
            self.right_pub = self.create_publisher(
                Float64MultiArray,
                '/right_forward_position_controller/commands', 10)
            self.left_pub = self.create_publisher(
                Float64MultiArray,
                '/left_forward_position_controller/commands', 10)

        # ══════════════════════════════════════════════════════════════
        # 平滑限速状态 (保存上一帧的关节值)
        # ══════════════════════════════════════════════════════════════
        self.last_right_joints: list[float] | None = None  # 右臂上一帧关节值
        self.last_left_joints: list[float] | None = None   # 左臂上一帧关节值
        self.last_right_gripper: float = 0.0               # 右夹爪上一帧值
        self.last_left_gripper: float = 0.0                # 左夹爪上一帧值
        
        # ══════════════════════════════════════════════════════════════
        # 归零状态
        # ══════════════════════════════════════════════════════════════
        self.right_at_zero: bool = True   # 右臂是否已归零
        self.left_at_zero: bool = True    # 左臂是否已归零
        self.returning_to_zero: bool = False  # 是否正在归零中

        # ── 调试 topic ──
        self.debug_enabled = bool(getattr(cfg, "DEBUG_TOPICS_ENABLED", False))
        self.debug_pose_pubs: dict[str, object] = {}
        self.debug_joints_pubs: dict[str, object] = {}
        if self.debug_enabled:
            self._setup_debug_publishers()

        # ══════════════════════════════════════════════════════════════
        # TF 发布器 - VR 参考坐标系
        # ══════════════════════════════════════════════════════════════
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_vr_reference_frame()
        
        # 保存 VR 参考坐标系信息 (用于坐标转换)
        self.vr_ref_pos = np.array(getattr(cfg, 'VR_REFERENCE_XYZ', (0.0, 0.0, 0.698)))
        self.vr_ref_rpy = getattr(cfg, 'VR_REFERENCE_RPY', (0.0, 0.0, 0.0))

        # 后台加载 IK
        threading.Thread(target=self._load_ik, daemon=True).start()

        # 控制循环
        self.create_timer(1.0 / cfg.CONTROL_HZ, self._loop)

        mode = "控制机器人" if send_to_robot else "仅仿真"
        bypass_warn = " [CLUTCH_BYPASS=ON, 调试模式!]" if getattr(cfg, 'CLUTCH_BYPASS', False) else ""
        self.get_logger().info(f"节点启动 [{mode}]{bypass_warn}, 正在加载 IK...")
        self.get_logger().info(f"VR 参考坐标系: {cfg.VR_REFERENCE_FRAME} @ ({self.vr_ref_pos[0]:.3f}, {self.vr_ref_pos[1]:.3f}, {self.vr_ref_pos[2]:.3f})")
        if self.debug_enabled:
            self.get_logger().info("调试 topic 已启用: /debug/*")

    # ── TF 发布 ──────────────────────────────────────

    def _publish_vr_reference_frame(self):
        """发布 VR 参考坐标系的 TF (Static Transform).
        
        这个 frame 位于两臂 L0 的中间，作为 VR → 机器人映射的参考原点。
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = getattr(cfg, 'VR_REFERENCE_FRAME', 'vr_reference')
        
        xyz = getattr(cfg, 'VR_REFERENCE_XYZ', (0.0, 0.0, 0.698))
        rpy = getattr(cfg, 'VR_REFERENCE_RPY', (0.0, 0.0, 0.0))
        
        t.transform.translation.x = xyz[0]
        t.transform.translation.y = xyz[1]
        t.transform.translation.z = xyz[2]
        
        # RPY → 四元数
        cr, cp, cy = math.cos(rpy[0]/2), math.cos(rpy[1]/2), math.cos(rpy[2]/2)
        sr, sp, sy = math.sin(rpy[0]/2), math.sin(rpy[1]/2), math.sin(rpy[2]/2)
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
        
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"已发布 TF: world → {t.child_frame_id}")

    # ── VR 回调 ──────────────────────────────────────

    def _on_right_pose(self, msg: PoseStamped):
        # #region agent log
        import json as _json, math as _math
        _LOG = "/home/sajio/vscode_robotic/openarmx_ws/.cursor/debug-62c352.log"
        _p, _o = msg.pose.position, msg.pose.orientation
        # #endregion
        pos, rot = self._convert_pose(msg)
        self.right.on_pose(pos, rot, time.time())
        # #region agent log
        try:
            _eu = [_math.atan2(rot[2,1],rot[2,2]), -_math.asin(max(-1,min(1,rot[2,0]))), _math.atan2(rot[1,0],rot[0,0])]
            with open(_LOG,"a") as _f: _f.write(_json.dumps({"sessionId":"62c352","hypothesisId":"A,B","location":"node.py:_on_right_pose","message":"right_pose_in","data":{"raw_quat":[round(_o.x,4),round(_o.y,4),round(_o.z,4),round(_o.w,4)],"ros_pos":[round(float(pos[0]),4),round(float(pos[1]),4),round(float(pos[2]),4)],"ros_euler_deg":[round(_math.degrees(_eu[0]),1),round(_math.degrees(_eu[1]),1),round(_math.degrees(_eu[2]),1)]},"timestamp":int(time.time()*1000)})+"\n")
        except: pass
        # #endregion
        self._publish_debug_pose("vr_right_input_pose", pos)

    def _on_left_pose(self, msg: PoseStamped):
        pos, rot = self._convert_pose(msg)
        self.left.on_pose(pos, rot, time.time())
        self._publish_debug_pose("vr_left_input_pose", pos)

    def _on_right_grip(self, msg: Float32):
        self.right.on_pinch(max(0.0, min(1.0, msg.data)))

    def _on_left_grip(self, msg: Float32):
        self.left.on_pinch(max(0.0, min(1.0, msg.data)))

    # ══════════════════════════════════════════════════════════════════════════
    # Clutch 回调 (Snapshot on Trigger) - 单个 clutch 同时控制双臂
    # ══════════════════════════════════════════════════════════════════════════

    def _on_clutch(self, msg: Bool):
        """Clutch 状态变化 (同时控制双臂)."""
        now = time.time()
        was_engaged = self.right.clutch_engaged  # 双臂状态同步，查一个即可
        
        # 同时更新左右臂
        self.right.on_clutch(msg.data, now)
        self.left.on_clutch(msg.data, now)
        
        # 日志: clutch 状态变化
        if msg.data and not was_engaged:
            self.get_logger().info(
                f"Clutch ON → Snapshot + Gain Ramp ({cfg.GAIN_RAMP_DURATION}s)")
        elif not msg.data and was_engaged:
            self.get_logger().info("Clutch OFF → 自动归零")

    def _convert_pose(self, msg: PoseStamped) -> tuple[np.ndarray, np.ndarray]:
        p, o = msg.pose.position, msg.pose.orientation

        # 协议基线: Quest 上游已输出 ROS 坐标，默认直接使用
        if getattr(cfg, "INPUT_POSE_IS_ROS", True) and not getattr(cfg, "APPLY_UNITY_TO_ROS", False):
            pos = np.array([p.x, p.y, p.z], dtype=np.float64)
            q = np.array([o.x, o.y, o.z, o.w], dtype=np.float64)
        else:
            pos = unity_to_ros_pos(p.x, p.y, p.z)
            q = unity_to_ros_quat(o.x, o.y, o.z, o.w)

        if getattr(cfg, "POSITION_SWAP_YZ", False):
            pos[1], pos[2] = pos[2], pos[1]

        rot = quat_to_rotmat(q[0], q[1], q[2], q[3])
        return pos, rot

    # ── IK 加载 ──────────────────────────────────────

    def _load_ik(self):
        try:
            ik = IKSolver(enable_viz=self.enable_viz)

            with self.ik_lock:
                self.ik = ik
                self.ik_ready = True

            # 初始化为 Home 姿态，避免 IK 内部状态与已发布姿态不一致导致偏置
            self.last_left_joints = self._home_left()
            self.last_right_joints = self._home_right()
            self.left_at_zero = True
            self.right_at_zero = True

            self.get_logger().info(
                f"IK 就绪 | L: {ik.joint_names_left}")
            self.get_logger().info(
                f"         | R: {ik.joint_names_right}")
            self.get_logger().info(
                "等待 Quest 3 数据 (/quest3/*_hand_pose)...")

        except Exception as e:
            self.get_logger().error(f"IK 加载失败: {e}")
            import traceback
            traceback.print_exc()

    # ── 控制循环 ──────────────────────────────────────

    def _loop(self):
        if not self.ik_ready:
            return

        now = time.time()

        with self.ik_lock:
            ik = self.ik
            # 0. 同步 IK 内部状态到最近一次发布的关节指令(Home/VR)，避免参考捕获偏置
            self._sync_ik_state_from_last_commands(ik)

            # 1. 超时检测
            self.right.check_timeout(now)
            self.left.check_timeout(now)

            # 2. 更新增益淡入 (Gain Ramp)
            self.right.update_gain_ramp(now)
            self.left.update_gain_ramp(now)

            # 3. 参考姿态捕获 (Snapshot on Trigger)
            #    在 clutch 按下瞬间捕获，确保举手动作被忽略
            self._try_capture(self.right, cfg.RIGHT_EE_FRAME)
            self._try_capture(self.left, cfg.LEFT_EE_FRAME)

            # 4. 更新 IK 目标 (仅当 clutch 按下时)
            right_updated = self._update_arm(self.right, ik.right, "right")
            left_updated = self._update_arm(self.left, ik.left, "left")

            # 5. 求解 (仅当有更新时才求解，防止空转时目标飘移)
            if right_updated or left_updated:
                ik.solve()

            # 6. 可视化
            ik.display()

            # 7. 发布 (仅当有 clutch 按下时才发布)
            if self.send_to_robot:
                self._publish(ik)

            # 8. 日志
            if now - self.last_status_time > cfg.STATUS_LOG_SEC:
                self._log_status()
                self.last_status_time = now

    # ── 辅助 ─────────────────────────────────────────

    def _try_capture(self, tracker: ArmTracker, ee_frame: str):
        """需要时捕获参考姿态 (Snapshot on Trigger).
        
        在 clutch 按下瞬间捕获当前手柄位姿为参考原点，
        这样"举手按 clutch"的动作会被完全忽略，机器人从当前位置平滑起步。
        
        注意: 存储的是 TCP 相对于 vr_reference 的位置，不是 world
        """
        if not tracker.needs_recapture or tracker.vr_pos is None:
            return
        if tracker.state != TrackingState.ACTIVE:
            return

        # 获取 TCP 在 world 中的位置
        pos_world, rot = self.ik.get_ee_pose(ee_frame)
        
        # 转换为相对于 vr_reference 的位置
        # pos_local = pos_world - vr_ref_pos
        pos_local = pos_world - self.vr_ref_pos
        
        if tracker.capture_reference(pos_local, rot):
            # Snapshot 完成
            trigger_info = " (Clutch Snapshot)" if tracker.clutch_just_pressed else ""
            self.get_logger().info(
                f"[{tracker.name}] 参考姿态已捕获{trigger_info} "
                f"local=({pos_local[0]:.3f}, {pos_local[1]:.3f}, {pos_local[2]:.3f}) "
                f"[相对于 vr_reference]")
            self._publish_debug_pose(f"vr_{tracker.name}_ref_pose", pos_local)
            tracker.clear_just_pressed()

    def _update_arm(self, tracker: ArmTracker, arm_ik, side: str) -> bool:
        """更新单臂 IK 目标 (不活跃时不更新 = 冻结).
        
        compute_target 返回的是相对于 vr_reference 的位置，
        需要转换回 world frame 给 IK solver。
        
        Returns:
            True 如果目标被更新，False 如果保持不变
        """
        if tracker.state != TrackingState.ACTIVE or tracker.needs_recapture:
            return False

        # target_pos 是相对于 vr_reference 的位置
        target_local, target_rot = tracker.compute_target()
        if target_local is None:
            return False

        # 转换到 world frame: target_world = target_local + vr_ref_pos
        target_world = target_local + self.vr_ref_pos
        self._publish_debug_pose(f"vr_{tracker.name}_target_local", target_local)
        self._publish_debug_pose(f"vr_{tracker.name}_target_world", target_world)

        self.ik.set_ee_target(arm_ik, target_world, target_rot)
        self.ik.update_elbow(arm_ik, target_world, side)
        return True

    def _publish(self, ik: IKSolver):
        """发布关节指令 (带平滑限速 + 自动归零)."""
        
        # ══════════════════════════════════════════════════════════════
        # 判断控制模式: VR 控制 or 自动归零
        # ══════════════════════════════════════════════════════════════
        right_clutch_on = self.right.clutch_engaged
        left_clutch_on = self.left.clutch_engaged
        
        # 初始化保护: 只有收到过 clutch 数据后才启用归零逻辑
        # 防止 Quest 3 还没连接时就开始归零
        right_clutch_ready = self.right.clutch_ever_received
        left_clutch_ready = self.left.clutch_ever_received
        
        # 检查是否需要归位 (clutch 松开 + 已收到过 clutch 数据 + 未归位)
        return_to_zero = getattr(cfg, 'RETURN_TO_ZERO_ENABLED', True)
        need_return_right = return_to_zero and right_clutch_ready and not right_clutch_on and not self.right_at_zero
        need_return_left = return_to_zero and left_clutch_ready and not left_clutch_on and not self.left_at_zero
        
        # 归零状态变化日志
        if (need_return_right or need_return_left) and not self.returning_to_zero:
            self.returning_to_zero = True
            self.get_logger().info("Clutch OFF → 开始自动归位到 Home 姿态...")
        elif not need_return_right and not need_return_left and self.returning_to_zero:
            self.returning_to_zero = False
            self.get_logger().info("归位完成 / Clutch ON")
        
        # ══════════════════════════════════════════════════════════════
        # 右臂
        # ══════════════════════════════════════════════════════════════
        if right_clutch_on:
            # VR 控制模式
            raw_r_vals = ik.extract_joints(ik.right)
            r_vals = raw_r_vals.copy()
            if cfg.RIGHT_ARM_J1_J2_SWAP:
                r_vals[0], r_vals[1] = r_vals[1], r_vals[0]
            r_vals = self._smooth_and_limit(r_vals, self.last_right_joints)
            self.right_at_zero = False  # 开始控制后标记为非零点
            self._publish_debug_joints("ik_right_joints_raw", raw_r_vals)
        elif need_return_right:
            # 自动归位模式
            r_vals = self._move_towards_target(self.last_right_joints, self._home_right())
            self.right_at_zero = self._check_at_target(r_vals, self._home_right())
        else:
            # 未控制/已归位，保持 Home 姿态
            r_vals = self._home_right() if self.last_right_joints is None else self.last_right_joints
        
        self.last_right_joints = r_vals.copy()
        self._publish_debug_joints("ik_right_joints_cmd", r_vals)
        
        # ── 夹爪 ──
        if right_clutch_on:
            gripper_target = cfg.GRIPPER_MAX_M * (1.0 - self.right.pinch)
        else:
            gripper_target = cfg.GRIPPER_MAX_M  # 归零时夹爪全开
        gripper_cmd = self._smooth_gripper(gripper_target, self.last_right_gripper)
        self.last_right_gripper = gripper_cmd
        r_vals.append(gripper_cmd)
        
        msg_r = Float64MultiArray(data=r_vals)
        self.right_pub.publish(msg_r)

        # ══════════════════════════════════════════════════════════════
        # 左臂
        # ══════════════════════════════════════════════════════════════
        if left_clutch_on:
            # VR 控制模式
            raw_l_vals = ik.extract_joints(ik.left)
            l_vals = raw_l_vals.copy()
            l_vals = self._smooth_and_limit(l_vals, self.last_left_joints)
            self.left_at_zero = False
            self._publish_debug_joints("ik_left_joints_raw", raw_l_vals)
        elif need_return_left:
            # 自动归位模式
            l_vals = self._move_towards_target(self.last_left_joints, self._home_left())
            self.left_at_zero = self._check_at_target(l_vals, self._home_left())
        else:
            # 未控制/已归位，保持 Home 姿态
            l_vals = self._home_left() if self.last_left_joints is None else self.last_left_joints
        
        self.last_left_joints = l_vals.copy()
        self._publish_debug_joints("ik_left_joints_cmd", l_vals)
        
        # ── 夹爪 ──
        if left_clutch_on:
            gripper_target = cfg.GRIPPER_MAX_M * (1.0 - self.left.pinch)
        else:
            gripper_target = cfg.GRIPPER_MAX_M  # 归零时夹爪全开
        gripper_cmd = self._smooth_gripper(gripper_target, self.last_left_gripper)
        self.last_left_gripper = gripper_cmd
        l_vals.append(gripper_cmd)
        
        msg_l = Float64MultiArray(data=l_vals)
        self.left_pub.publish(msg_l)

    # ══════════════════════════════════════════════════════════════════════════
    # 平滑限速函数 (调试时可在此处加日志观察效果)
    # ══════════════════════════════════════════════════════════════════════════

    def _smooth_and_limit(
        self, 
        ik_vals: list[float], 
        last_vals: list[float] | None
    ) -> list[float]:
        """对关节值应用 EMA 平滑 + 速度限制.
        
        处理流程:
          1. EMA 低通滤波 (如果启用)
          2. 速度限制 (如果启用)
        
        Args:
            ik_vals: IK 求解出的关节值 (本帧目标)
            last_vals: 上一帧发送的关节值 (None = 首帧)
        
        Returns:
            处理后的关节值
        """
        # 首帧: 直接使用 IK 结果
        if last_vals is None:
            return list(ik_vals)
        
        result = []
        for i, (target, last) in enumerate(zip(ik_vals, last_vals)):
            val = target
            
            # ── Step 1: EMA 低通滤波 ──
            # new = alpha * target + (1-alpha) * last
            # alpha 越小越平滑
            if cfg.SMOOTH_ENABLED:
                val = cfg.SMOOTH_ALPHA * target + (1.0 - cfg.SMOOTH_ALPHA) * last
            
            # ── Step 2: 速度限制 ──
            # 限制每帧最大变化量
            if cfg.VELOCITY_LIMIT_ENABLED:
                delta = val - last
                if abs(delta) > cfg.MAX_JOINT_DELTA:
                    # 超出限制，截断到最大变化量
                    val = last + cfg.MAX_JOINT_DELTA * (1.0 if delta > 0 else -1.0)
            
            result.append(val)
        
        return result

    def _smooth_gripper(self, target: float, last: float) -> float:
        """夹爪单独平滑 (使用独立的平滑系数).
        
        Args:
            target: 目标夹爪值
            last: 上一帧夹爪值
        
        Returns:
            平滑后的夹爪值
        """
        alpha = getattr(cfg, 'GRIPPER_SMOOTH_ALPHA', cfg.SMOOTH_ALPHA)
        return alpha * target + (1.0 - alpha) * last

    # ══════════════════════════════════════════════════════════════════════════
    # 自动归零函数
    # ══════════════════════════════════════════════════════════════════════════

    def _home_left(self) -> list[float]:
        vals = getattr(cfg, "HOME_POSE_LEFT_JOINTS", None)
        if vals is None:
            return [0.0] * 7
        return [float(v) for v in vals]

    def _home_right(self) -> list[float]:
        vals = getattr(cfg, "HOME_POSE_RIGHT_JOINTS", None)
        if vals is None:
            return [0.0] * 7
        return [float(v) for v in vals]

    def _sync_ik_state_from_last_commands(self, ik: IKSolver):
        """将 IK 内部模型状态同步到最近一次发布的关节指令.

        目的:
          - Home 姿态/自动归位是通过 ros2_control 直接发关节指令实现的
          - IK 内部 RobotWrapper 不会自动读取真实 joint_states
          - 若不同步，参考捕获会以 URDF 零位为基准，导致固定偏置（如 90° offset）
        """
        # 选择用于同步的关节值：优先 last_*（已发布），否则 Home
        left_vals = self.last_left_joints if self.last_left_joints is not None else self._home_left()
        right_vals_pub = self.last_right_joints if self.last_right_joints is not None else self._home_right()

        # 如果启用了右臂 J1/J2 交换（发布时做了交换），这里需要反交换回 IK 模型语义
        right_vals = list(right_vals_pub)
        if getattr(cfg, "RIGHT_ARM_J1_J2_SWAP", False) and len(right_vals) >= 2:
            right_vals[0], right_vals[1] = right_vals[1], right_vals[0]

        q = ik.robot.state.q
        # 左臂
        for i, (_, idx) in enumerate(ik.left.joint_indices):
            if i < len(left_vals):
                q[idx] = float(left_vals[i])
        # 右臂
        for i, (_, idx) in enumerate(ik.right.joint_indices):
            if i < len(right_vals):
                q[idx] = float(right_vals[i])

        ik.robot.state.q = q
        ik.robot.update_kinematics()

    def _move_towards_target(self, current_joints: list[float] | None, target: list[float]) -> list[float]:
        """将关节平滑移动向目标姿态.
        
        每帧最多移动 RETURN_TO_ZERO_SPEED (rad)
        
        Args:
            current_joints: 当前关节值 (None = 未初始化)
            target: 目标关节值
        
        Returns:
            移动后的关节值
        """
        if current_joints is None:
            return list(target)
        
        speed = getattr(cfg, 'RETURN_TO_ZERO_SPEED', 0.02)
        result = []
        
        for val, tgt in zip(current_joints, target):
            delta = tgt - val
            if abs(delta) <= speed:
                result.append(float(tgt))
            else:
                result.append(float(val + speed * (1.0 if delta > 0 else -1.0)))
        
        return result

    def _check_at_target(self, joints: list[float], target: list[float]) -> bool:
        """检查关节是否都已到达目标姿态.
        
        Args:
            joints: 关节值列表
            target: 目标关节值
        
        Returns:
            True = 所有关节都在阈值内，视为已到达
        """
        threshold = getattr(cfg, 'RETURN_TO_ZERO_THRESHOLD', 0.01)
        return all(abs(j - t) < threshold for j, t in zip(joints, target))

    def _setup_debug_publishers(self):
        pose_topics = [
            "vr_right_input_pose",
            "vr_left_input_pose",
            "vr_right_ref_pose",
            "vr_left_ref_pose",
            "vr_right_target_local",
            "vr_left_target_local",
            "vr_right_target_world",
            "vr_left_target_world",
        ]
        for name in pose_topics:
            self.debug_pose_pubs[name] = self.create_publisher(PoseStamped, f"/debug/{name}", 10)

        joints_topics = [
            "ik_right_joints_raw",
            "ik_left_joints_raw",
            "ik_right_joints_cmd",
            "ik_left_joints_cmd",
        ]
        for name in joints_topics:
            self.debug_joints_pubs[name] = self.create_publisher(Float64MultiArray, f"/debug/{name}", 10)

    def _publish_debug_pose(self, name: str, pos: np.ndarray):
        if not self.debug_enabled:
            return
        pub = self.debug_pose_pubs.get(name)
        if pub is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if name.endswith("_world"):
            msg.header.frame_id = "world"
        else:
            msg.header.frame_id = getattr(cfg, "VR_REFERENCE_FRAME", "vr_reference")
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.w = 1.0
        pub.publish(msg)

    def _publish_debug_joints(self, name: str, joints: list[float]):
        if not self.debug_enabled:
            return
        pub = self.debug_joints_pubs.get(name)
        if pub is None:
            return
        pub.publish(Float64MultiArray(data=[float(v) for v in joints]))

    def _log_status(self):
        r, l = self.right, self.left
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"[坐标系] vr_reference @ ({self.vr_ref_pos[0]:.3f}, {self.vr_ref_pos[1]:.3f}, {self.vr_ref_pos[2]:.3f})")
        
        # 右手详细调试
        if r.vr_pos is not None:
            self.get_logger().info(
                f"[右手] VR位置(ROS系): X={r.vr_pos[0]:+.3f} Y={r.vr_pos[1]:+.3f} Z={r.vr_pos[2]:+.3f}")
            
            if r.ref_vr_pos is not None:
                delta = r.vr_pos - r.ref_vr_pos
                self.get_logger().info(
                    f"[右手] VR增量:       X={delta[0]:+.3f} Y={delta[1]:+.3f} Z={delta[2]:+.3f}")
            
            if r.ref_robot_pos is not None:
                self.get_logger().info(
                    f"[右手] TCP参考(local): X={r.ref_robot_pos[0]:+.3f} Y={r.ref_robot_pos[1]:+.3f} Z={r.ref_robot_pos[2]:+.3f}")
                
                target_local, _ = r.compute_target()
                if target_local is not None:
                    target_world = target_local + self.vr_ref_pos
                    self.get_logger().info(
                        f"[右手] IK目标(local): X={target_local[0]:+.3f} Y={target_local[1]:+.3f} Z={target_local[2]:+.3f}")
                    self.get_logger().info(
                        f"[右手] IK目标(world): X={target_world[0]:+.3f} Y={target_world[1]:+.3f} Z={target_world[2]:+.3f}")
        
        # 显示关节角度
        if self.last_right_joints:
            j = self.last_right_joints
            self.get_logger().info(
                f"[右臂] 关节: J1={j[0]:+.3f} J2={j[1]:+.3f} | J3={j[2]:+.3f} J4={j[3]:+.3f} (锁定)")
        
        # Clutch 状态
        clutch_status = "ON" if r.clutch_engaged else "OFF"
        gain = r.current_gain
        self.get_logger().info(f"[状态] Clutch={clutch_status} Gain={gain:.2f}")
