"""手臂追踪安全状态机.

管理单只手的 VR 追踪状态, 处理:
  - 数据超时 → 冻结机器人
  - 位置跳变 → 重新校准映射
  - 追踪恢复 → 平滑续接
  - Clutch 触发 → Snapshot on Trigger + Gain Ramp
"""

from enum import Enum

import numpy as np

from . import config as cfg
from .transforms import apply_gripper_offset, remap_rotation_delta


class TrackingState(Enum):
    WAITING = "waiting"   # 等待首次数据
    ACTIVE  = "active"    # 正常追踪中
    LOST    = "lost"      # 数据超时, 已冻结


class ArmTracker:
    """单臂 VR 追踪 + 安全状态管理."""

    def __init__(self, name: str):
        self.name = name
        self.state = TrackingState.WAITING

        # 最新 VR 数据 (ROS 坐标系)
        self.vr_pos: np.ndarray | None = None
        self.vr_rot: np.ndarray | None = None
        self.pinch: float = 0.0
        self.last_data_time: float = 0.0

        # 参考姿态对 (相对映射)
        self.ref_vr_pos: np.ndarray | None = None
        self.ref_vr_rot: np.ndarray | None = None
        self.ref_robot_pos: np.ndarray | None = None
        self.ref_robot_rot: np.ndarray | None = None
        self.needs_recapture: bool = True

        # ══════════════════════════════════════════════════════════════
        # Clutch 安全机制 (Snapshot on Trigger + Gain Ramp)
        # ══════════════════════════════════════════════════════════════
        # 调试模式: 绕过 clutch 检查
        bypass = getattr(cfg, 'CLUTCH_BYPASS', False)
        self.clutch_engaged: bool = bypass     # clutch 当前是否按下
        self.clutch_just_pressed: bool = False # clutch 刚刚按下 (边沿检测)
        self.clutch_ever_received: bool = bypass # 是否收到过 clutch 数据 (初始化保护)
        self.gain_ramp_start_time: float = 0.0 # 增益淡入开始时间
        self.current_gain: float = 1.0 if bypass else 0.0  # 当前控制增益 [0~1]

    # ── 输入 ──────────────────────────────────────────

    def on_pose(self, pos: np.ndarray, rot: np.ndarray, now: float):
        """新 VR 位姿到达."""
        if self.vr_pos is not None:
            if np.linalg.norm(pos - self.vr_pos) > cfg.MAX_JUMP_M:
                self.needs_recapture = True

        self.vr_pos = pos.copy()
        self.vr_rot = rot.copy()
        self.last_data_time = now

        if self.state != TrackingState.ACTIVE:
            self.needs_recapture = True
        self.state = TrackingState.ACTIVE

    def on_pinch(self, value: float):
        """更新捏合度 (含死区)."""
        if value < cfg.PINCH_DEAD_LO:
            value = 0.0
        elif value > cfg.PINCH_DEAD_HI:
            value = 1.0
        self.pinch = value

    # ══════════════════════════════════════════════════════════════════════════
    # Clutch 安全机制
    # ══════════════════════════════════════════════════════════════════════════

    def on_clutch(self, engaged: bool, now: float):
        """处理 clutch 状态变化.
        
        Snapshot on Trigger: clutch 按下瞬间捕获参考姿态
        
        Args:
            engaged: clutch 是否按下 (True=按下, False=松开)
            now: 当前时间戳
        """
        # 标记已收到过 clutch 数据 (初始化保护)
        self.clutch_ever_received = True
        
        # 边沿检测: 从松开变为按下
        if engaged and not self.clutch_engaged:
            self.clutch_just_pressed = True
            self.gain_ramp_start_time = now
            self.current_gain = 0.0
            # 标记需要重新捕获参考姿态 (Snapshot)
            self.needs_recapture = True
        
        # 松开 clutch → 冻结 (不再发送控制)
        if not engaged and self.clutch_engaged:
            self.current_gain = 0.0
        
        self.clutch_engaged = engaged

    def update_gain_ramp(self, now: float):
        """更新增益淡入 (Gain Ramp).
        
        clutch 按下后，增益从 0 线性增加到 1
        
        Args:
            now: 当前时间戳
        """
        if not self.clutch_engaged:
            self.current_gain = 0.0
            return
        
        if not getattr(cfg, 'GAIN_RAMP_ENABLED', True):
            self.current_gain = 1.0
            return
        
        duration = getattr(cfg, 'GAIN_RAMP_DURATION', 0.5)
        elapsed = now - self.gain_ramp_start_time
        
        if elapsed >= duration:
            self.current_gain = 1.0
        else:
            # 线性淡入
            self.current_gain = elapsed / duration
    
    def clear_just_pressed(self):
        """清除 clutch_just_pressed 标志 (在捕获参考姿态后调用)."""
        self.clutch_just_pressed = False

    # ── 安全检查 ──────────────────────────────────────

    def check_timeout(self, now: float):
        """超时 → 冻结."""
        if self.state == TrackingState.ACTIVE:
            if (now - self.last_data_time) > cfg.DATA_TIMEOUT_SEC:
                self.state = TrackingState.LOST
                self.needs_recapture = True

    # ── 映射 ─────────────────────────────────────────

    def capture_reference(
        self, robot_pos: np.ndarray, robot_rot: np.ndarray
    ) -> bool:
        """捕获 VR↔机器人 参考姿态对.

        调用时机: 首次数据 / 跳变后 / 追踪恢复后
        """
        if self.vr_pos is None:
            return False
        self.ref_vr_pos = self.vr_pos.copy()
        self.ref_vr_rot = self.vr_rot.copy()
        self.ref_robot_pos = robot_pos.copy()
        self.ref_robot_rot = robot_rot.copy()
        self.needs_recapture = False
        return True

    def compute_target(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        """计算机器人目标位姿 (相对映射 + 增益淡入).

        位置: target = ref_robot + gain × scale × (vr_now − vr_ref)
        旋转: target = ref_robot × slerp(I, delta_rot, gain)
        
        gain 从 0 渐增到 1，防止 clutch 按下瞬间的抖动传递到机器人
        """
        if self.ref_vr_pos is None or self.vr_pos is None:
            return None, None
        
        # clutch 没按下 → 不控制
        if not self.clutch_engaged:
            return None, None

        # ── 位置 (带增益) ──
        delta_pos = cfg.POSITION_SCALE * (self.vr_pos - self.ref_vr_pos)
        # 增益淡入: 初始时 gain=0, delta_pos 被完全抑制
        target_pos = self.ref_robot_pos + self.current_gain * delta_pos

        # ── 旋转 (带增益) ──
        local_delta_rot = self.ref_vr_rot.T @ self.vr_rot
        # 旋转轴重映射 + 右臂镜像修正
        local_delta_rot = remap_rotation_delta(local_delta_rot, self.name == "right")
        # 增益淡入: 对旋转增量进行插值
        # gain=0 时 target_rot = ref_robot_rot (不旋转)
        # gain=1 时 target_rot = ref_robot_rot @ delta_rot (完整旋转)
        if self.current_gain < 1.0:
            # 简化处理: 对旋转矩阵做线性插值 (近似 SLERP)
            local_delta_rot = (1.0 - self.current_gain) * np.eye(3) + self.current_gain * local_delta_rot
            # 重新正交化 (Gram-Schmidt)
            local_delta_rot = self._orthonormalize(local_delta_rot)
        
        target_rot = self.ref_robot_rot @ local_delta_rot

        # ── 末端旋转补偿 ──
        # clutch ON 首帧(gain=0)必须严格保持参考姿态，避免“ON即动”。
        # 因此补偿只在 gain>0 时生效，并随 gain_ramp 渐入。
        if getattr(cfg, 'GRIPPER_ROT_OFFSET_ENABLED', False) and self.current_gain > 1e-6:
            offset_rot = apply_gripper_offset(np.eye(3))
            blended = (1.0 - self.current_gain) * np.eye(3) + self.current_gain * offset_rot
            blended = self._orthonormalize(blended)
            target_rot = target_rot @ blended

        return target_pos, target_rot
    
    @staticmethod
    def _orthonormalize(R: np.ndarray) -> np.ndarray:
        """Gram-Schmidt 正交化旋转矩阵."""
        u = R[:, 0]
        v = R[:, 1]
        w = R[:, 2]
        
        u = u / np.linalg.norm(u)
        v = v - np.dot(v, u) * u
        v = v / np.linalg.norm(v)
        w = np.cross(u, v)
        
        return np.column_stack([u, v, w])