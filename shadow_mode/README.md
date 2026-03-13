# Shadow Mode - OpenArmX 控制系统

一键启动的机械臂控制系统，包含 CAN 管理、电机校准、ROS2 启动和 GUI 控制。

---

## 安装 / 克隆

将 `shadow_mode` 放入 OpenArmX ROS2 工作空间中：

```bash
cd ~/openarmx_ws   # 或你的工作空间路径
git clone <repo_url> shadow_mode
```

确保工作空间已包含 `openarmx_description`、`openarmx_bringup`、`openarmx_arm_driver` 等依赖包，并已执行 `colcon build`。

---

## 目录结构

```
shadow_mode/
├── start.sh            # 启动入口 (bash)
├── start_shadow.py     # 主启动脚本
├── calibrate_motors.py # 电机校准工具
├── can_register.py     # CAN 接口管理
├── gui.py              # GUI 控制界面
├── gui_value_logic.md  # 数值逻辑说明
└── vr_controller/      # VR 遥操作模块
    ├── __init__.py
    ├── __main__.py     # python -m 入口
    ├── main.py         # CLI 入口
    ├── node.py         # ROS2 节点
    ├── ik_solver.py    # Placo IK 求解器
    ├── tracker.py      # 手臂追踪状态机
    ├── transforms.py   # Unity↔ROS 坐标变换
    └── config.py       # 配置参数
```

---

## 快速启动

```bash
# 方式 1: 正常启动 (真实硬件)
./shadow_mode/start.sh

# 方式 2: 校准 + 启动
./shadow_mode/start.sh -z

# 方式 3: VR 遥操作 (启动 rosbridge)
./shadow_mode/start.sh --vr
./shadow_mode/start.sh -z --vr   # 校准 + VR

# 方式 4: 模拟模式 (无需硬件，用于开发测试)
./shadow_mode/start.sh --sim
./shadow_mode/start.sh --sim --vr   # 模拟 + VR 测试

# 方式 5: 分步操作
python3 shadow_mode/calibrate_motors.py --zero   # 先校准
python3 shadow_mode/start_shadow.py              # 再启动 ROS2
python3 shadow_mode/gui.py                       # 最后开 GUI
```

---

## 文件详解

### 1. `start.sh` - 启动入口

简单的 bash 包装器，传递参数到 `start_shadow.py`。

```bash
./shadow_mode/start.sh           # 正常启动 (真实硬件)
./shadow_mode/start.sh -z        # 校准零点 + 启动
./shadow_mode/start.sh --vr      # 启动 ROS2 + rosbridge (VR遥操作)
./shadow_mode/start.sh -z --vr   # 校准 + ROS2 + rosbridge
./shadow_mode/start.sh --sim     # 模拟模式 (无需硬件)
./shadow_mode/start.sh --sim --vr  # 模拟 + VR 测试
```

---

### 2. `start_shadow.py` - 主启动脚本

**功能**: 一键启动整个控制链路：CAN → ROS2 → forward_position_controller (→ rosbridge)

**参数**:
| 参数 | 说明 |
|------|------|
| `--zero` / `-z` | 启动前校准电机零点 |
| `--vr` | 启动 rosbridge (VR 遥操作，端口 9090) |
| `--sim` | 模拟模式 (无需硬件，用于 VLA 开发/VR 测试) |

**流程**:
1. `step_1_setup_can()` - 启动 CAN 接口 (跳过如果 `--sim`)
2. `step_0_calibrate_zero()` - [可选] 校准电机零点 (跳过如果 `--sim`)
3. `step_2_launch_ros2()` - 启动 ROS2 (`--sim` 使用 fake hardware)
4. `step_3_verify_controllers()` - 验证 controller 已激活
5. `step_4_start_rosbridge()` - [可选] 启动 rosbridge WebSocket (`--vr`)

**配置** (脚本内):
```python
RIGHT_CAN = "can2"
LEFT_CAN = "can3"
CAN_BITRATE = 1000000
SUDO_PASSWORD = "123456"
```

**退出行为**: Ctrl+C 会触发 `cleanup()`:
- 关闭 ROS2 进程
- 禁用所有电机 (disable_all)
- 关闭 CAN 接口

---

### 3. `calibrate_motors.py` - 电机校准工具

**功能**: 独立的电机状态检查和零点校准，不依赖 ROS2。

**用法**:
```bash
python3 calibrate_motors.py              # 检查状态 (默认)
python3 calibrate_motors.py --check      # 检查状态
python3 calibrate_motors.py --zero       # 校准所有电机零点
python3 calibrate_motors.py --zero --arm left          # 只校准左臂
python3 calibrate_motors.py --zero --arm right --joint 4  # 只校准右臂 J4
python3 calibrate_motors.py --no-cleanup # 跳过环境清理
```

**关键函数**:
| 函数 | 说明 |
|------|------|
| `cleanup_environment()` | 杀 ROS2 进程 + 重置 CAN |
| `setup_can()` | 启动 CAN 接口 |
| `check_motors()` | 读取电机角度/速度 |
| `calibrate_zero()` | 调用 `arm.set_zero()` 设置零点 |

**判断逻辑**: 角度 < 0.1 rad 为"正常"，否则"需要校准"

---

### 4. `can_register.py` - CAN 接口管理

**功能**: 查看 CAN 状态、统计、开启/关闭/重置接口。

**用法**:
```bash
python3 can_register.py              # 查看 can2, can3 状态
python3 can_register.py can2         # 查看指定接口
python3 can_register.py --up         # 开启 can2, can3
python3 can_register.py --down       # 关闭 can2, can3
python3 can_register.py --reset      # 重置 (清理 buffer)
python3 can_register.py --all --down # 关闭系统所有 CAN
python3 can_register.py -v           # 显示详细信息
```

**显示内容**:
| 项目 | 说明 |
|------|------|
| 状态 | UP (🟢) / DOWN (🔴) |
| RX/TX | packets, bytes, dropped, errors |
| berr-counter | CAN 总线错误计数 |
| tx_queue_len | 发送队列最大长度 |

---

### 5. `gui.py` - GUI 控制界面

**功能**: PySide6 滑块界面，通过 ROS2 `forward_position_controller` 控制关节。

**ROS2 接口**:
| 类型 | Topic |
|------|-------|
| 发布 | `/right_forward_position_controller/commands` (Float64MultiArray) |
| 发布 | `/left_forward_position_controller/commands` (Float64MultiArray) |
| 订阅 | `/joint_states` (JointState) |

**关键类**:
| 类 | 说明 |
|----|------|
| `ROS2Bridge` | ROS2 节点封装，后台线程 spin |
| `JointSlider` | 单关节滑块组件，支持限位 |
| `MainWindow` | 主窗口，控制逻辑 |

**关节限位** (与 URDF/ros2_control 一致):
```python
JOINT_LIMITS = {
    "openarmx_right_joint1": (-1.25, 3.5),
    "openarmx_right_joint2": (-0.05, 3.27),
    # ... 其他关节
}
```

**控制频率**: 50Hz (20ms 定时器)

**操作**:
1. 点击"连接 ROS2"
2. 点击"同步当前位置" (将滑块对齐到实际关节)
3. 点击"开始控制"
4. 拖动滑块控制机械臂

---

### 6. `vr_controller/` - VR 遥操作模块

**功能**: Meta Quest 3 VR 遥操作，实时 IK 求解控制机械臂。

**启动**:
```bash
# 先启动 ROS2 + rosbridge
./shadow_mode/start.sh --vr
# 或模拟模式测试
./shadow_mode/start.sh --sim --vr

# 新终端启动 VR 控制器
cd ~/Desktop/openarmx_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m shadow_mode.vr_controller
# 或
python3 -m shadow_mode.vr_controller --no-robot --viz  # 仅仿真+可视化
```

**参数**:
| 参数 | 说明 |
|------|------|
| `--no-robot` | 仅 IK 仿真，不发送关节指令 |
| `--viz` | 启用 meshcat 3D 可视化 |
| `--scale` | VR→机器人位置缩放 (默认 1.0) |

**ROS2 接口**:
| 方向 | Topic | 类型 |
|------|-------|------|
| 订阅 | `/quest3/right_hand_pose` | PoseStamped |
| 订阅 | `/quest3/left_hand_pose` | PoseStamped |
| 订阅 | `/quest3/right_gripper` | Float32 |
| 订阅 | `/quest3/left_gripper` | Float32 |
| 发布 | `/right_forward_position_controller/commands` | Float64MultiArray |
| 发布 | `/left_forward_position_controller/commands` | Float64MultiArray |

**核心模块**:
| 文件 | 说明 |
|------|------|
| `node.py` | ROS2 节点，订阅 VR 数据，发布关节指令 |
| `ik_solver.py` | Placo IK 求解器 (末端 + 肘部约束) |
| `tracker.py` | 手臂追踪状态机 (超时冻结、跳变重校准) |
| `transforms.py` | Unity↔ROS 坐标系变换 |
| `config.py` | 所有可调参数 |

**安全机制**:
- 数据超时 (0.2s) → 冻结机器人
- 位置跳变 (>0.15m) → 重新校准映射
- 夹爪死区 (0.05~0.95)

---

## 数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                         shadow_mode                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  start.sh ──→ start_shadow.py                                   │
│                    │                                            │
│                    ├──→ step_1_setup_can()                      │
│                    │         └──→ ip link set canX up           │
│                    │                                            │
│                    ├──→ step_0_calibrate_zero() [--zero]        │
│                    │         └──→ openarmx_arm_driver.Robot     │
│                    │                    └──→ arm.set_zero()     │
│                    │                                            │
│                    ├──→ step_2_launch_ros2()                    │
│                    │         └──→ ros2 launch openarm.bimanual  │
│                    │                                            │
│                    └──→ step_4_start_rosbridge() [--vr]         │
│                              └──→ rosbridge_websocket (9090)    │
│                                        │                        │
│                                        ▼                        │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                    ROS2 Stack                            │   │
│  │  ┌─────────────────┐    ┌──────────────────────────┐    │   │
│  │  │ v10_simple_hw   │◄──►│ forward_position_ctrl    │    │   │
│  │  │ (CAN ↔ Motor)   │    │ (/commands topic)        │    │   │
│  │  └─────────────────┘    └──────────────────────────┘    │   │
│  │           │                         ▲                    │   │
│  │           ▼                         │                    │   │
│  │    /joint_states              /commands                  │   │
│  └───────────┼─────────────────────────┼────────────────────┘   │
│              │                         │                        │
│       ┌──────┴──────┐                  │                        │
│       │             │                  │                        │
│       ▼             ▼                  │                        │
│  gui.py       rosbridge ◄──────────────┘                        │
│  (PySide6)    (ws://9090)                                       │
│       │             │                                           │
│       │             └──→ Quest 3 VR 遥操作                      │
│       │                                                         │
│       └──→ 滑块 ──→ send_right_arm() / send_left_arm()         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## CAN 配置

| 接口 | 用途 | 电机 ID |
|------|------|---------|
| can2 | 右臂 | 1-7 (关节) + 8 (夹爪) |
| can3 | 左臂 | 1-7 (关节) + 8 (夹爪) |

**比特率**: 1,000,000 bps

---

## 常见问题

### 1. ROS2 启动失败
```bash
# 检查 CAN 状态
python3 shadow_mode/can_register.py

# 重置 CAN
python3 shadow_mode/can_register.py --reset
```

### 2. 电机位置不对
```bash
# 检查电机当前角度
python3 shadow_mode/calibrate_motors.py --check

# 重新校准
python3 shadow_mode/calibrate_motors.py --zero
```

### 3. GUI 连接不上
确保 ROS2 已启动且 `/joint_states` 有数据:
```bash
ros2 topic hz /joint_states
```

### 4. CAN 被占用
```bash
# 杀掉占用进程
pkill -9 -f ros2
python3 shadow_mode/can_register.py --reset
```

---

## 依赖

- **Python**: rclpy, PySide6, openarmx_arm_driver, placo, numpy
- **ROS2**: humble, ros2_control, forward_position_controller, rosbridge_server
- **系统**: SocketCAN (ip, can-utils)

安装 rosbridge (VR 功能):
```bash
sudo apt install ros-humble-rosbridge-server
```

安装 placo (VR IK 求解):
```bash
pip install placo placo-utils
```
