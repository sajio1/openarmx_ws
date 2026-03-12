import time
import threading
import tkinter as tk
import os
from tkinter import ttk
import numpy as np
import placo
import pinocchio as pin
from placo_utils.visualization import frame_viz, robot_frame_viz, robot_viz
import sys
from pathlib import Path

# Add project root to path if needed (copied from original)
ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))


# Configuration
# ROOT_DIR: .../openarmx_ws/src/teleop_vr_pico/openarm_teleop_by_pico
# WORKSPACE_SRC: .../openarmx_ws/src  (where openarm_description lives)
WORKSPACE_SRC = ROOT_DIR.parents[1]
URDF_PATH = str(WORKSPACE_SRC / "openarm_description/urdf/robot/openarm_bimanual_sim_ext.urdf")

# Set ROS_PACKAGE_PATH to find meshes
WORKSPACE_SRC_STR = str(WORKSPACE_SRC)
if "ROS_PACKAGE_PATH" not in os.environ:
    os.environ["ROS_PACKAGE_PATH"] = WORKSPACE_SRC_STR
else:
    if WORKSPACE_SRC_STR not in os.environ["ROS_PACKAGE_PATH"]:
        os.environ["ROS_PACKAGE_PATH"] = WORKSPACE_SRC_STR + ":" + os.environ["ROS_PACKAGE_PATH"]

VIEW_FPS = 50
STEP_SIZE = 0.01  # 1cm per tick
ROT_STEP = 0.05   # radians per tick
HOLD_REPEAT_DELAY = 300  # ms before repeat starts
HOLD_REPEAT_INTERVAL = 50 # ms between repeats

# Global flags
RUNNING = True

# Helper functions for math
def normalize_quaternion(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return quat / norm

def rotation_matrix_y(angle: float) -> np.ndarray:
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)

def rpy_to_matrix(r, p, y):
    # Rotation matrix from Roll-Pitch-Yaw (intrinsic XYZ)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(r), -np.sin(r)],
        [0, np.sin(r), np.cos(r)]
    ])
    Ry = np.array([
        [np.cos(p), 0, np.sin(p)],
        [0, 1, 0],
        [-np.sin(p), 0, np.cos(p)]
    ])
    Rz = np.array([
        [np.cos(y), -np.sin(y), 0],
        [np.sin(y), np.cos(y), 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx

# Shared state for targets
class RobotState:
    def __init__(self):
        self.lock = threading.Lock()
        self.targets = {}
        # Initial values will be set after robot loads
        self.initialized = False
        self.control_mode = "absolute"  # "absolute" or "relative"
        self.reference = {}  # reference pose per arm
        self.relative_offsets = {}  # relative deltas per arm (pos, rot)

    def _capture_reference_locked(self):
        """Capture current targets as reference; caller must hold lock."""
        for arm, target in self.targets.items():
            self.reference[arm] = {
                "pos": target["pos"].copy(),
                "rot": target["rot"].copy(),
            }
            if arm not in self.relative_offsets:
                self.relative_offsets[arm] = {
                    "pos": np.zeros(3),
                    "rot": np.eye(3),
                }

    def _update_targets_from_relative_locked(self, arm: str):
        """Recompute world targets from reference + relative offsets; caller must hold lock."""
        if arm not in self.reference or arm not in self.relative_offsets:
            return
        ref = self.reference[arm]
        rel = self.relative_offsets[arm]
        self.targets[arm]["rot"] = ref["rot"] @ rel["rot"]
        self.targets[arm]["pos"] = ref["pos"] + ref["rot"] @ rel["pos"]

    def set_reference_from_current(self):
        if not self.initialized:
            return
        with self.lock:
            self._capture_reference_locked()
            # Reset relative offsets to zero and align targets to reference
            for arm in self.targets:
                self.relative_offsets[arm] = {
                    "pos": np.zeros(3),
                    "rot": np.eye(3),
                }
                self._update_targets_from_relative_locked(arm)

    def set_control_mode(self, mode: str):
        if mode not in ("absolute", "relative"):
            return
        with self.lock:
            if not self.initialized:
                self.control_mode = mode
                return
            if mode == "relative" and not self.reference:
                # Ensure reference exists for relative control
                self._capture_reference_locked()
            self.control_mode = mode

    def update_target_pos(self, arm, axis, direction):
        if not self.initialized: return
        with self.lock:
            idx = {'x': 0, 'y': 1, 'z': 2}[axis]
            if self.control_mode == "absolute":
                self.targets[arm]['pos'][idx] += direction * STEP_SIZE
            else:
                if arm not in self.relative_offsets:
                    self._capture_reference_locked()
                self.relative_offsets[arm]['pos'][idx] += direction * STEP_SIZE
                self._update_targets_from_relative_locked(arm)

    def update_target_rot(self, arm, axis, direction):
        if not self.initialized: return
        with self.lock:
            # Apply rotation in world frame
            r = 0; p = 0; y = 0
            if axis == 'rx': r = direction * ROT_STEP
            elif axis == 'ry': p = direction * ROT_STEP
            elif axis == 'rz': y = direction * ROT_STEP

            delta_rot = rpy_to_matrix(r, p, y)
            if self.control_mode == "absolute":
                self.targets[arm]['rot'] = delta_rot @ self.targets[arm]['rot']
            else:
                if arm not in self.relative_offsets:
                    self._capture_reference_locked()
                self.relative_offsets[arm]['rot'] = delta_rot @ self.relative_offsets[arm]['rot']
                self._update_targets_from_relative_locked(arm)

state = RobotState()

def ik_thread_func():
    global RUNNING

    # Load robot
    print(f"Loading robot from: {URDF_PATH}")
    try:
        # robot = placo.RobotWrapper(URDF_PATH, placo.Flags.ignore_collisions)
        robot = placo.RobotWrapper(URDF_PATH, placo.Flags.collision_as_visual)
    except Exception as e:
        print(f"Error loading robot: {e}")
        return

    solver = placo.KinematicsSolver(robot)
    solver.mask_fbase(True)

    # Setup Tasks
    try:
        left_task = solver.add_frame_task("openarm_left_link7_pico", np.eye(4))
        left_task.configure("openarm_left_link7_pico", "soft", 500.0, 100.0)

        right_task = solver.add_frame_task("openarm_right_link7_pico", np.eye(4))
        right_task.configure("openarm_right_link7_pico", "soft", 500.0, 100.0)

        # Use joints task instead of link4 frame task for elbow regularization
        # This applies a soft constraint on joint4 (elbow joint) to maintain ~5 degrees
        joints_task = solver.add_joints_task()
        joints_task.configure("joints", "soft", 1.0)  # Low weight for regularization

    except Exception as e:
        print(f"Error configuring tasks (check frame names): {e}")
        return

    solver.enable_velocity_limits(True)
    solver.enable_joint_limits(True)

    viz = robot_viz(robot)

    # Initialize targets based on current pose
    robot.update_kinematics()
    left_pose = robot.get_T_world_frame("openarm_left_link7_pico")
    right_pose = robot.get_T_world_frame("openarm_right_link7_pico")

    # Rotational offset from original script
    # effector_rot_offset = rotation_matrix_y(np.pi / 2.0)
    effector_rot_offset = rotation_matrix_y(0.0)

    with state.lock:
        state.targets['left'] = {
            "pos": left_pose[:3, 3].copy(),
            "rot": left_pose[:3, :3].copy() @ effector_rot_offset.T,
            "task": left_task
        }
        state.targets['right'] = {
            "pos": right_pose[:3, 3].copy(),
            "rot": right_pose[:3, :3].copy() @ effector_rot_offset.T,
            "task": right_task
        }

        # Set joint targets for regularization
        # Get current joint positions and set joint4 targets to ~5 degrees (0.087 radians)
        current_q = robot.state.q.copy()

        # Find joint4 indices for left and right arms
        # Adjust these joint names based on your actual URDF
        joint_targets = {}

        # Set target for left arm joint4 to approximately 5 degrees (0.087 radians)
        if "openarm_left_joint4" in robot.model.names:
            joint_targets["openarm_left_joint3"] = 0.0  
            joint_targets["openarm_left_joint4"] = 0.087  # ~5 degrees in radians

        # Set target for right arm joint4 to approximately 5 degrees (0.087 radians)
        if "openarm_right_joint4" in robot.model.names:
            joint_targets["openarm_right_joint3"] = 0.0 
            joint_targets["openarm_right_joint4"] = 0.087  # ~5 degrees in radians

        # Update joints task with the targets
        joints_task.set_joints(joint_targets)

        print(f"Joint targets set: {joint_targets}")

        # Initialize reference and relative offsets to current pose
        for arm in ["left", "right"]:
            state.reference[arm] = {
                "pos": state.targets[arm]["pos"].copy(),
                "rot": state.targets[arm]["rot"].copy(),
            }
            state.relative_offsets[arm] = {
                "pos": np.zeros(3),
                "rot": np.eye(3),
            }

        state.initialized = True

    solver.dt = 0.001
    dt_sleep = 1.0 / VIEW_FPS

    # IK timing/print config
    print_ik_timing = True
    ik_timing_every_n = 10  # print once every N loop iterations to avoid spamming
    ik_iter = 0

    print("IK Loop started")

    while RUNNING:
        start_time = time.perf_counter()

        with state.lock:
            # Update task targets from state
            T_left = np.eye(4)
            T_left[:3, :3] = state.targets['left']['rot'] @ effector_rot_offset
            T_left[:3, 3] = state.targets['left']['pos']
            state.targets['left']['task'].T_world_frame = T_left

            T_right = np.eye(4)
            T_right[:3, :3] = state.targets['right']['rot'] @ effector_rot_offset
            T_right[:3, 3] = state.targets['right']['pos']
            state.targets['right']['task'].T_world_frame = T_right

        # Solve
        # Multiple steps for convergence
        for _ in range(10):
            solver.solve(True)
            robot.update_kinematics()

        # Viz
        viz.display(robot.state.q)

        # Visualizing frames
        robot_frame_viz(robot, "openarm_left_link7_pico")
        robot_frame_viz(robot, "openarm_right_link7_pico")
        frame_viz("target_left", state.targets['left']['task'].T_world_frame)
        frame_viz("target_right", state.targets['right']['task'].T_world_frame)

        # Optional: visualize link4 positions (even though we're not constraining them directly)
        robot_frame_viz(robot, "openarm_left_link4")
        robot_frame_viz(robot, "openarm_right_link4")

        elapsed = time.perf_counter() - start_time
        if print_ik_timing:
            ik_iter += 1
            if ik_iter % ik_timing_every_n == 0:
                print(f"IK loop time: {elapsed * 1000.0:.3f} ms")
        if elapsed < dt_sleep:
            time.sleep(dt_sleep - elapsed)

class RepeatingButton(ttk.Button):
    def __init__(self, master, **kwargs):
        self.cmd = kwargs.pop('command', None)
        super().__init__(master, **kwargs)
        self.bind('<Button-1>', self.on_press)
        self.bind('<ButtonRelease-1>', self.on_release)
        self.timer = None

    def on_press(self, event):
        if self.cmd:
            self.cmd()
            # Start repeating
            self.timer = self.after(HOLD_REPEAT_DELAY, self.repeat)

    def on_release(self, event):
        if self.timer:
            self.after_cancel(self.timer)
            self.timer = None

    def repeat(self):
        if self.cmd:
            self.cmd()
            self.timer = self.after(HOLD_REPEAT_INTERVAL, self.repeat)

class ControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("OpenArmX IK Control - Joint4 Constraint")

        main_frame = ttk.Frame(root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Top controls
        top_frame = ttk.Frame(main_frame)
        top_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        ttk.Label(top_frame, text="按住按钮连续移动 | Joint4 ~5° 约束").grid(row=0, column=0, columnspan=3, sticky=tk.W)
        self.mode_label = ttk.Label(top_frame, text="当前模式：绝对")
        self.mode_label.grid(row=1, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Button(top_frame, text="切换为相对/绝对", command=self.toggle_mode).grid(row=1, column=1, padx=5, pady=(5, 0))
        ttk.Button(top_frame, text="设当前为参考姿态", command=self.set_reference).grid(row=1, column=2, padx=5, pady=(5, 0))
        self.reference_label = ttk.Label(top_frame, text="参考姿态：未设置")
        self.reference_label.grid(row=2, column=0, columnspan=3, sticky=tk.W, pady=(5, 0))

        # Left Arm Controls
        left_frame = ttk.LabelFrame(main_frame, text="Left Arm", padding="10")
        left_frame.grid(row=1, column=0, padx=5, pady=5)
        self.create_axis_controls(left_frame, "left")

        # Right Arm Controls
        right_frame = ttk.LabelFrame(main_frame, text="Right Arm", padding="10")
        right_frame.grid(row=1, column=1, padx=5, pady=5)
        self.create_axis_controls(right_frame, "right")

    def create_axis_controls(self, parent, arm):
        # Position Axes
        axes = [('X', 'x'), ('Y', 'y'), ('Z', 'z')]
        for i, (label, axis) in enumerate(axes):
            ttk.Label(parent, text=label).grid(row=i, column=0)
            RepeatingButton(parent, text="-", width=3,
                       command=lambda a=arm, ax=axis: state.update_target_pos(a, ax, -1)).grid(row=i, column=1)
            RepeatingButton(parent, text="+", width=3,
                       command=lambda a=arm, ax=axis: state.update_target_pos(a, ax, 1)).grid(row=i, column=2)

        ttk.Separator(parent, orient='horizontal').grid(row=3, column=0, columnspan=3, sticky='ew', pady=5)

        # Rotation Axes
        rot_axes = [('Rx', 'rx'), ('Ry', 'ry'), ('Rz', 'rz')]
        for i, (label, axis) in enumerate(rot_axes):
            row = i + 4
            ttk.Label(parent, text=label).grid(row=row, column=0)
            RepeatingButton(parent, text="<", width=3,
                       command=lambda a=arm, ax=axis: state.update_target_rot(a, ax, -1)).grid(row=row, column=1)
            RepeatingButton(parent, text=">", width=3,
                       command=lambda a=arm, ax=axis: state.update_target_rot(a, ax, 1)).grid(row=row, column=2)

    def toggle_mode(self):
        new_mode = "relative" if state.control_mode == "absolute" else "absolute"
        state.set_control_mode(new_mode)
        self.mode_label.config(text=f"当前模式：{'相对' if new_mode == 'relative' else '绝对'}")

    def set_reference(self):
        state.set_reference_from_current()
        self.reference_label.config(text="参考姿态：已更新")

def main():
    global RUNNING

    # Start IK thread
    t = threading.Thread(target=ik_thread_func)
    t.daemon = True
    t.start()

    # Start GUI
    root = tk.Tk()
    app = ControlGUI(root)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        RUNNING = False
        t.join(timeout=1.0)
        print("Exiting...")

if __name__ == "__main__":
    main()