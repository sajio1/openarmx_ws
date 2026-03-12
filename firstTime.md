# OpenArmX 命令行上手教程

> **基于您的实际环境配置**
> - 工作空间：`/home/ok/Desktop/openarmx_ws/`
> - CAN 通道：**can2**（左臂）、**can3**（右臂）
> - 密码：`123456`

---

电机扭矩结算公式：
MIT 模式的控制公式是：
τ = Kp * (p_des - p) + Kd * (v_des - v) + τ_ff

其中：
τ: 输出扭矩
Kp: 位置增益
p_des: 期望位置
p: 当前位置
Kd: 速度增益
v_des: 期望速度
v: 当前速度
τ_ff: 前馈扭矩

物理意义：
项	公式部分	物理意义
位置误差项	Kp × (p_des - p_cur)	像弹簧，Kp 越大，"拉力"越强，响应越快
速度阻尼项	Kd × (v_des - v_cur)	像阻尼器，Kd 越大，运动越平滑，减少震荡
前馈扭矩	τ_ff	直接施加的扭矩，用于重力补偿等
参数调节建议：
场景	Kp	Kd
柔顺/安全	低 (5-20)	低 (0.5-2)
正常控制	中 (20-50)	中 (2-5)
快速响应	高 (50-100)	高 (5-10)


## 一、环境准备

### 1.1 打开终端并进入工作空间
```bash
cd /home/ok/Desktop/openarmx_ws
```

### 1.2 加载 ROS2 环境
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

> **提示**：可以将上述命令添加到 `~/.bashrc` 中，每次打开终端自动加载

---

## 二、CAN 接口管理

### 2.1 启用 CAN 接口
```bash
# 启用 can2 (左臂)
sudo ip link set can2 up type can bitrate 1000000

# 启用 can3 (右臂)
sudo ip link set can3 up type can bitrate 1000000
```

### 2.2 检查 CAN 接口状态
```bash
# 查看所有 CAN 接口
ip link show | grep can

# 查看详细状态
ip -details link show can2
ip -details link show can3
```
**正常状态**：显示 `state UP`

### 2.3 监听 CAN 总线数据
```bash
# 监听 can2 (左臂)
candump can2

# 监听 can3 (右臂)
candump can3

# 按 Ctrl+C 停止监听
```

### 2.4 关闭 CAN 接口
```bash
sudo ip link set can2 down
sudo ip link set can3 down
```

---

## 三、电机控制（使用编译好的工具）

### 3.1 检查单个电机状态
```bash
cd /home/ok/Desktop/openarmx_ws

# 检查 can2 上的电机 1
./build/openarmx_can/motor-check 1 can2

# 检查 can3 上的电机 1
./build/openarmx_can/motor-check 1 can3

# 检查指定电机 (电机ID 1-8)
./build/openarmx_can/motor-check <电机ID> <CAN接口>
```

### 3.2 测试夹爪
```bash
# 测试左臂夹爪 (can2, ID=8)
./build/openarmx_can/robstride-demo-gripper can2 8

# 测试右臂夹爪 (can3, ID=8)
./build/openarmx_can/robstride-demo-gripper can3 8
```

### 3.3 完整机械臂演示
```bash
# 7+1 电机演示程序
./build/openarmx_can/robstride-demo-full
```

---

## 四、Python 脚本控制

### 4.1 进入脚本目录
```bash
cd /home/ok/Desktop/openarmx_ws/src/openarmx_motor_manager/scripts
```

### 4.2 检查所有电机状态
```bash
python3 check_motor_status.py
```

### 4.3 使能所有电机
```bash
python3 en_all_motors.py
```

### 4.4 禁用所有电机
```bash
python3 dis_all_motors.py
```

### 4.5 回零操作
```bash
python3 control_motor_gohome.py
```

### 4.6 单电机 MIT 模式测试
```bash
python3 test_motor_one_MIT.py
```
> **注意**：运行前需要修改脚本中的 CAN 通道配置为 `can2`/`can3`

### 4.7 设置零位
```bash
python3 set_motor_zero.py
```

---

## 五、MoveIt 运动规划

### 5.1 启动 MoveIt（仿真模式）
```bash
cd /home/ok/Desktop/openarmx_ws
source install/setup.bash

# 仿真模式（不连接实际硬件）
ros2 launch openarmx_bimanual_moveit_config demo.launch.py use_fake_hardware:=true
```

### 5.2 启动 MoveIt（连接实际硬件）
```bash
# 需要先启用 CAN 和电机
ros2 launch openarmx_bimanual_moveit_config demo.launch.py use_fake_hardware:=false
```

### 5.3 查看可用的启动脚本
```bash
ls /home/ok/Desktop/openarmx_ws/src/openarmx_ros2/openarmx_bimanual_moveit_config/launch/
```

---

## 六、ROS2 常用命令

### 6.1 查看话题
```bash
# 列出所有话题
ros2 topic list

# 查看关节状态
ros2 topic echo /joint_states
```

### 6.2 查看节点
```bash
ros2 node list
```

### 6.3 查看控制器状态
```bash
ros2 control list_controllers
```

### 6.4 查看 TF 坐标系
```bash
ros2 run tf2_tools view_frames
```

---

## 七、轨迹录制与回放

### 7.1 录制轨迹
```bash
cd /home/ok/Desktop/openarmx_ws
source install/setup.bash

# 启动连续录制
ros2 run openarmx_teach record_joint_states_always
```

### 7.2 回放轨迹
```bash
ros2 run openarmx_teach play_joint_trajectory --ros-args -p file:=<轨迹文件路径>
```

---

## 八、故障排查命令

### 8.1 检查 CAN 错误统计
```bash
ip -s link show can2
ip -s link show can3
```

### 8.2 检查 CAN 内核模块
```bash
lsmod | grep can
```

### 8.3 重新加载 PEAK USB 驱动
```bash
sudo rmmod peak_usb
sudo modprobe peak_usb
```

### 8.4 查看系统日志
```bash
dmesg | grep can
```

### 8.5 检查 USB CAN 设备
```bash
lsusb | grep -i peak
```

---

## 九、完整操作流程速查

### 9.1 开机流程
```bash
# 1. 进入工作空间
cd /home/ok/Desktop/openarmx_ws

# 2. 加载环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 3. 启用 CAN 接口
sudo ip link set can2 up type can bitrate 1000000
sudo ip link set can3 up type can bitrate 1000000

# 4. 打开 48V 电源（手动操作）

# 5. 检查电机状态
cd src/openarmx_motor_manager/scripts
python3 check_motor_status.py

# 6. 使能电机
python3 en_all_motors.py

# 7. 回零（可选）
python3 control_motor_gohome.py
```

### 9.2 关机流程
```bash
# 1. 进入脚本目录
cd /home/ok/Desktop/openarmx_ws/src/openarmx_motor_manager/scripts

# 2. 禁用电机
python3 dis_all_motors.py

# 3. 关闭 CAN 接口
sudo ip link set can2 down
sudo ip link set can3 down

# 4. 关闭 48V 电源（手动操作）
```

---

## 十、快速参考

### 路径速查
| 项目 | 路径 |
|------|------|
| 工作空间 | `/home/ok/Desktop/openarmx_ws/` |
| Python 脚本 | `src/openarmx_motor_manager/scripts/` |
| 编译工具 | `build/openarmx_can/` |
| 使用说明书 | `src/使用说明书/` |
| MoveIt 配置 | `src/openarmx_ros2/openarmx_bimanual_moveit_config/` |

### CAN 配置
| 通道 | 连接 | 电机 ID |
|------|------|---------|
| can2 | 左臂 + 左夹爪 | 1-8 |
| can3 | 右臂 + 右夹爪 | 1-8 |

### 常用脚本
| 脚本 | 功能 |
|------|------|
| `check_motor_status.py` | 检查电机状态 |
| `en_all_motors.py` | 使能所有电机 |
| `dis_all_motors.py` | 禁用所有电机 |
| `control_motor_gohome.py` | 回零 |
| `set_motor_zero.py` | 设置零位 |
| `test_motor_one_MIT.py` | 单电机测试 |

---

**⚠️ 安全提示：操作时手始终准备在急停按钮上方！**