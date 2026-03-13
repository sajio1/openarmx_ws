"""Placo IK 求解器封装 (支持 J4/L4/Pole 肘部约束).

职责:
  - 加载 URDF, 配置任务 (末端 + 肘部)
  - 提供 solve() 接口
  - 提取关节角度

肘部约束方案:
  - J4: 关节角度约束 (joints_task), 更柔和
  - L4: 空间位置约束 (frame_task), 更刚性
  - Pole: 人体式肘部极向约束 + 中性姿态 regularization
"""

import os
from dataclasses import dataclass, field

import numpy as np
import placo

from . import config as cfg


@dataclass
class ArmIK:
    """单臂 IK 任务 + 关节索引."""
    ee_task: object = None
    elbow_task: object = None  # L4 方案使用
    joint_indices: list[tuple[str, int]] = field(default_factory=list)
    init_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    init_rot: np.ndarray = field(default_factory=lambda: np.eye(3))
    elbow_frame: str = ""
    shoulder_frame: str = ""
    elbow_side: int = 0
    upper_arm_len: float = 0.0
    forearm_len: float = 0.0
    init_elbow_dir: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))


class IKSolver:
    """双臂 IK 求解器."""

    def __init__(self, enable_viz: bool = False):
        self._setup_package_path()
        self._check_urdf()

        self.robot = placo.RobotWrapper(
            cfg.URDF_PATH, placo.Flags.collision_as_visual)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        self.solver.enable_velocity_limits(True)
        self.solver.enable_joint_limits(True)
        self.solver.dt = 0.001

        # 肘部约束方案: "none" / "j4" / "l4" / "pole"
        self.elbow_constraint = getattr(cfg, 'ELBOW_CONSTRAINT', 'none')

        # 任务: 分离创建，避免互相覆盖
        self.elbow_joints_task = None
        self.neutral_task = None
        self.locked_joints_task = None

        # J4 方案: 创建关节约束
        if self.elbow_constraint == "j4":
            self.elbow_joints_task = self.solver.add_joints_task()
            self.elbow_joints_task.configure("j4_elbow", "soft", cfg.JOINTS_WEIGHT)
            self._setup_j4_targets()

        # Pole 方案: 创建中性姿态 regularization
        if self.elbow_constraint == "pole":
            self.neutral_task = self.solver.add_joints_task()
            self.neutral_task.configure("neutral_posture", "soft", cfg.ARM_NEUTRAL_WEIGHT)
            self._setup_neutral_targets()

        self.left = self._setup_arm(
            cfg.LEFT_EE_FRAME, cfg.LEFT_ELBOW_FRAME, cfg.LEFT_SHOULDER_FRAME,
            cfg.LEFT_JOINT_PREFIX, elbow_side=+1)
        self.right = self._setup_arm(
            cfg.RIGHT_EE_FRAME, cfg.RIGHT_ELBOW_FRAME, cfg.RIGHT_SHOULDER_FRAME,
            cfg.RIGHT_JOINT_PREFIX, elbow_side=-1)

        # 关节锁定 (调试用)
        self._setup_locked_joints()

        # meshcat
        self.viz = None
        if enable_viz:
            try:
                from placo_utils.visualization import robot_viz
                self.viz = robot_viz(self.robot)
            except ImportError:
                pass
        
        constraint_name = self.elbow_constraint.upper() if self.elbow_constraint != "none" else "无 (自由)"
        print(f"[IK] 肘部约束方案: {constraint_name}")
        
        # 显示锁定状态
        locked = getattr(cfg, 'LOCKED_JOINTS', [])
        if locked:
            print(f"[IK] ⚠️  调试模式: 锁定关节 J{locked} (只有 J1,J2 可动)")

    # ── 公开接口 ────────────────────────────────────

    def solve(self):
        """执行 IK 求解 (多次迭代以收敛)."""
        for _ in range(cfg.IK_STEPS):
            self.solver.solve(True)
            self.robot.update_kinematics()

    def set_ee_target(self, arm: ArmIK, pos: np.ndarray, rot: np.ndarray):
        """设置末端目标位姿."""
        T = np.eye(4)
        T[:3, 3] = pos
        T[:3, :3] = rot
        arm.ee_task.T_world_frame = T

    def update_elbow(self, arm: ArmIK, ee_pos: np.ndarray, side: str):
        """动态更新肘部约束 (跟随末端位置).
        
        仅 L4 / Pole 方案需要动态更新，J4 方案的关节目标是固定的。
        """
        if self.elbow_constraint == "j4":
            return  # J4 方案不需要动态更新
        
        if arm.elbow_task is None:
            return

        if self.elbow_constraint == "l4":
            if side == "left":
                offset = np.array([-cfg.ELBOW_BACK, +cfg.ELBOW_SIDE, +cfg.ELBOW_UP])
            else:
                offset = np.array([-cfg.ELBOW_BACK, -cfg.ELBOW_SIDE, +cfg.ELBOW_UP])

            T = arm.elbow_task.T_world_frame.copy()
            T[:3, 3] = ee_pos + offset
            arm.elbow_task.T_world_frame = T
            return

        if self.elbow_constraint == "pole":
            elbow_T = self.robot.get_T_world_frame(arm.elbow_frame)
            T = elbow_T.copy()
            T[:3, 3] = self._compute_pole_elbow_target(arm, ee_pos)
            arm.elbow_task.T_world_frame = T

    def get_ee_pose(self, ee_frame: str) -> tuple[np.ndarray, np.ndarray]:
        """获取末端当前位姿 (pos, rot)."""
        T = self.robot.get_T_world_frame(ee_frame)
        return T[:3, 3].copy(), T[:3, :3].copy()

    def extract_joints(self, arm: ArmIK) -> list[float]:
        """从 IK 解中提取关节角度."""
        q = self.robot.state.q
        return [float(q[idx]) for _, idx in arm.joint_indices]

    def display(self):
        """更新 meshcat 可视化."""
        if self.viz is not None:
            self.viz.display(self.robot.state.q)

    @property
    def joint_names_left(self) -> list[str]:
        return [n for n, _ in self.left.joint_indices]

    @property
    def joint_names_right(self) -> list[str]:
        return [n for n, _ in self.right.joint_indices]

    # ── 内部 ────────────────────────────────────────

    @staticmethod
    def _setup_package_path():
        if "ROS_PACKAGE_PATH" not in os.environ:
            os.environ["ROS_PACKAGE_PATH"] = cfg.MESH_PACKAGE_PATH
        elif cfg.MESH_PACKAGE_PATH not in os.environ["ROS_PACKAGE_PATH"]:
            os.environ["ROS_PACKAGE_PATH"] = (
                cfg.MESH_PACKAGE_PATH + ":" + os.environ["ROS_PACKAGE_PATH"]
            )

    @staticmethod
    def _check_urdf():
        if not os.path.exists(cfg.URDF_PATH):
            raise FileNotFoundError(f"URDF 不存在: {cfg.URDF_PATH}")

    def _setup_locked_joints(self):
        """锁定指定关节 (调试用).
        
        使用 joints_task 将关节锁定在 0 位置，权重极高。
        """
        locked = getattr(cfg, 'LOCKED_JOINTS', [])
        if not locked:
            return
        
        if self.locked_joints_task is None:
            self.locked_joints_task = self.solver.add_joints_task()
            self.locked_joints_task.configure("locked_joints", "soft", 10000.0)
        
        joint_targets = {}
        for j_num in locked:
            # 左臂
            left_name = f"{cfg.LEFT_JOINT_PREFIX}{j_num}"
            joint_targets[left_name] = 0.0
            # 右臂
            right_name = f"{cfg.RIGHT_JOINT_PREFIX}{j_num}"
            joint_targets[right_name] = 0.0
        
        self.locked_joints_task.set_joints(joint_targets)

    def _setup_j4_targets(self):
        """设置 J4 方案的关节角度目标."""
        joint_targets = {}
        
        # 左臂
        left_j3 = f"{cfg.LEFT_JOINT_PREFIX}3"
        left_j4 = f"{cfg.LEFT_JOINT_PREFIX}4"
        joint_targets[left_j3] = cfg.JOINT3_TARGET
        joint_targets[left_j4] = cfg.JOINT4_TARGET
        
        # 右臂
        right_j3 = f"{cfg.RIGHT_JOINT_PREFIX}3"
        right_j4 = f"{cfg.RIGHT_JOINT_PREFIX}4"
        joint_targets[right_j3] = cfg.JOINT3_TARGET
        joint_targets[right_j4] = cfg.JOINT4_TARGET
        
        self.elbow_joints_task.set_joints(joint_targets)
        print(f"[IK] J4 约束目标: joint3={cfg.JOINT3_TARGET:.3f}, joint4={cfg.JOINT4_TARGET:.3f} rad")

    def _setup_neutral_targets(self):
        """设置 Pole 方案的中性姿态目标.
        
        若 USE_NEUTRAL_AS_MIDPOINT=True 且配置了 JOINT_LIMITS_LEFT/RIGHT，
        则中性 = 各关节 (min+max)/2，左右臂分别取各自限位中点；
        否则使用 ARM_NEUTRAL_JOINT_TARGETS（左右共用）。
        """
        joint_targets = {}
        use_midpoint = getattr(cfg, "USE_NEUTRAL_AS_MIDPOINT", False)
        limits_left = getattr(cfg, "JOINT_LIMITS_LEFT", None)
        limits_right = getattr(cfg, "JOINT_LIMITS_RIGHT", None)

        if use_midpoint and limits_left and limits_right:
            for j_num in sorted(set(limits_left.keys()) | set(limits_right.keys())):
                lo_l, hi_l = limits_left.get(j_num, (0.0, 0.0))
                lo_r, hi_r = limits_right.get(j_num, (0.0, 0.0))
                joint_targets[f"{cfg.LEFT_JOINT_PREFIX}{j_num}"] = 0.5 * (lo_l + hi_l)
                joint_targets[f"{cfg.RIGHT_JOINT_PREFIX}{j_num}"] = 0.5 * (lo_r + hi_r)
            if joint_targets:
                self.neutral_task.set_joints(joint_targets)
                print("[IK] Pole 中性姿态 = 关节灵活度中点 (左/右分别取限位中点)")
            return
        # 回退: 左右共用 ARM_NEUTRAL_JOINT_TARGETS
        neutral_targets = getattr(cfg, "ARM_NEUTRAL_JOINT_TARGETS", {})
        for j_num, target in sorted(neutral_targets.items()):
            joint_targets[f"{cfg.LEFT_JOINT_PREFIX}{j_num}"] = float(target)
            joint_targets[f"{cfg.RIGHT_JOINT_PREFIX}{j_num}"] = float(target)
        if joint_targets:
            self.neutral_task.set_joints(joint_targets)
            print(
                "[IK] Pole 中性姿态目标: "
                + ", ".join(f"J{j}={float(v):.3f}" for j, v in sorted(neutral_targets.items()))
            )

    def _setup_arm(
        self, ee_frame: str, elbow_frame: str, shoulder_frame: str,
        joint_prefix: str, elbow_side: int,
    ) -> ArmIK:
        arm = ArmIK(
            elbow_frame=elbow_frame,
            shoulder_frame=shoulder_frame,
            elbow_side=elbow_side,
        )

        # 末端任务 (高权重)
        arm.ee_task = self.solver.add_frame_task(ee_frame, np.eye(4))
        arm.ee_task.configure(ee_frame, "soft", cfg.EE_WEIGHT, cfg.EE_GAIN)

        # 肘部任务
        if self.elbow_constraint == "l4":
            arm.elbow_task = self.solver.add_frame_task(elbow_frame, np.eye(4))
            arm.elbow_task.configure(
                elbow_frame, "soft", cfg.ELBOW_W_POS, cfg.ELBOW_W_ROT)
        elif self.elbow_constraint == "pole":
            arm.elbow_task = self.solver.add_frame_task(elbow_frame, np.eye(4))
            arm.elbow_task.configure(
                f"{elbow_frame}_pole", "soft", cfg.ELBOW_POLE_W_POS, cfg.ELBOW_POLE_W_ROT)
        else:
            arm.elbow_task = None

        # 初始位姿 - 将 IK 目标设为当前位姿，防止启动时"飞"
        self.robot.update_kinematics()
        shoulder_T = self.robot.get_T_world_frame(shoulder_frame)
        elbow_T = self.robot.get_T_world_frame(elbow_frame)
        ee_T = self.robot.get_T_world_frame(ee_frame)
        arm.init_pos = ee_T[:3, 3].copy()
        arm.init_rot = ee_T[:3, :3].copy()
        arm.upper_arm_len = float(np.linalg.norm(elbow_T[:3, 3] - shoulder_T[:3, 3]))
        arm.forearm_len = float(np.linalg.norm(ee_T[:3, 3] - elbow_T[:3, 3]))
        arm.init_elbow_dir = self._safe_normalize(
            elbow_T[:3, 3] - shoulder_T[:3, 3],
            fallback=np.array([0.0, float(elbow_side), -1.0], dtype=np.float64),
        )
        
        # 关键：将初始 IK 目标设为当前位姿，而不是原点
        arm.ee_task.T_world_frame = ee_T.copy()

        # 初始化肘部目标
        if self.elbow_constraint == "l4" and arm.elbow_task is not None:
            T_init = np.eye(4)
            T_init[:3, 3] = elbow_T[:3, 3] + np.array([
                -cfg.ELBOW_BACK,
                elbow_side * cfg.ELBOW_SIDE,
                +cfg.ELBOW_UP,
            ])
            T_init[:3, :3] = elbow_T[:3, :3]
            arm.elbow_task.T_world_frame = T_init
        elif self.elbow_constraint == "pole" and arm.elbow_task is not None:
            arm.elbow_task.T_world_frame = elbow_T.copy()

        # 关节索引
        for i in range(1, self.robot.model.njoints):
            name = self.robot.model.names[i]
            if name.startswith(joint_prefix):
                arm.joint_indices.append((name, self.robot.model.joints[i].idx_q))
        arm.joint_indices.sort(key=lambda x: x[0])

        return arm

    def _compute_pole_elbow_target(self, arm: ArmIK, ee_pos: np.ndarray) -> np.ndarray:
        """根据 shoulder→wrist 几何和极向偏好，计算更像人手的肘部目标点."""
        shoulder_pos = self.robot.get_T_world_frame(arm.shoulder_frame)[:3, 3].copy()
        elbow_pos = self.robot.get_T_world_frame(arm.elbow_frame)[:3, 3].copy()

        shoulder_to_wrist = ee_pos - shoulder_pos
        dist_actual = float(np.linalg.norm(shoulder_to_wrist))
        if dist_actual < 1e-9:
            return elbow_pos

        upper = max(arm.upper_arm_len, 1e-6)
        fore = max(arm.forearm_len, 1e-6)
        min_reach = max(abs(upper - fore) + 1e-6, 1e-6)
        max_reach = max(min_reach + 1e-6, upper + fore - getattr(cfg, "ELBOW_SINGULARITY_MARGIN", 0.03))
        dist = min(max(dist_actual, min_reach), max_reach)

        axis = shoulder_to_wrist / dist_actual
        center_dist = (upper * upper - fore * fore + dist * dist) / (2.0 * dist)
        radius_sq = max(upper * upper - center_dist * center_dist, 0.0)
        radius = float(np.sqrt(radius_sq))
        center = shoulder_pos + center_dist * axis

        preferred = self._preferred_elbow_direction(arm.elbow_side)
        pole = preferred - np.dot(preferred, axis) * axis

        if np.linalg.norm(pole) < 1e-9:
            init_pole = arm.init_elbow_dir - np.dot(arm.init_elbow_dir, axis) * axis
            pole = self._safe_normalize(init_pole, fallback=self._orthogonal_vector(axis, arm.elbow_side))
        else:
            pole = self._safe_normalize(pole, fallback=self._orthogonal_vector(axis, arm.elbow_side))

        return center + radius * pole

    def _preferred_elbow_direction(self, elbow_side: int) -> np.ndarray:
        """将配置中的 (forward, outward, up) 转换到 world 方向."""
        forward, outward, up = getattr(cfg, "ELBOW_POLE_VECTOR", (0.0, 0.25, -1.0))
        direction = np.array(
            [float(forward), float(-elbow_side * outward), float(up)],
            dtype=np.float64,
        )
        return self._safe_normalize(
            direction,
            fallback=np.array([0.0, float(-elbow_side), -1.0], dtype=np.float64),
        )

    @staticmethod
    def _safe_normalize(vec: np.ndarray, fallback: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vec))
        if norm < 1e-9:
            fallback_norm = float(np.linalg.norm(fallback))
            if fallback_norm < 1e-9:
                return np.array([1.0, 0.0, 0.0], dtype=np.float64)
            return fallback / fallback_norm
        return vec / norm

    @staticmethod
    def _orthogonal_vector(axis: np.ndarray, elbow_side: int) -> np.ndarray:
        candidates = (
            np.array([0.0, float(-elbow_side), 0.0], dtype=np.float64),
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
            np.array([1.0, 0.0, 0.0], dtype=np.float64),
        )
        for candidate in candidates:
            ortho = candidate - np.dot(candidate, axis) * axis
            if np.linalg.norm(ortho) > 1e-6:
                return ortho / np.linalg.norm(ortho)
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)
