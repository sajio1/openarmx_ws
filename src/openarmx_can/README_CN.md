# OpenArmX CAN

[English](README.md) | [中文](#概述)

---

### 概述

OpenArmX CAN 是 OpenArmX 机器人系统的电机控制库，由成都长数机器人有限公司（[openarmx.com](https://openarmx.com)）开发。本库提供了完整的 Robstride 电机 CAN 通信接口，支持 7 自由度机械臂和夹爪控制。

**官方网站**: [openarmx.com](https://openarmx.com)

### 功能特性

- **电机类型支持**: RS00, RS03, RS04, RS06 (Robstride 电机系列)
- **多种控制模式**: 运控模式、位置模式、速度模式、电流模式
- **CAN 总线通信**: 支持 CAN 2.0 和 CAN-FD
- **实时控制**: 高频率电机控制和状态反馈
- **状态监控**: 电机位置、速度、扭矩、温度监控
- **参数配置**: 电机参数读取和设置
- **错误处理**: 完整的错误检测和处理机制

### 电机类型映射

| Robstride 电机 | 关节应用 |
|----------------|----------|
| RS04           | 关节 1-2 |
| RS03           | 关节 3-4 |
| RS00           | 关节 5-7, 夹爪 |

### 包结构

```
openarmx_can/
├── include/openarm/
│   ├── can/
│   │   └── socket/                    # CAN 套接字接口
│   │       └── openarm.hpp            # 主 OpenArm 接口
│   ├── canbus/                        # CAN 总线通信层
│   │   ├── can_socket.hpp
│   │   ├── can_device.hpp
│   │   └── can_device_collection.hpp
│   └── robstride_motor/               # Robstride 电机层
│       ├── rs_motor_constants.hpp     # 常量和类型定义
│       ├── rs_motor.hpp               # 电机基础类
│       ├── rs_motor_control.hpp       # 控制命令编码/解码
│       ├── rs_motor_device.hpp        # CAN 设备管理
│       └── rs_motor_device_collection.hpp  # 设备集合管理
├── src/openarm/
│   ├── can/socket/                    # CAN 套接字实现
│   ├── canbus/                        # CAN 通信实现
│   └── robstride_motor/               # Robstride 电机实现
├── examples/                          # 示例程序
│   ├── robstride_demo.cpp             # 基础演示程序
│   ├── robstride_demo_full.cpp        # 完整 7+1 电机演示
│   ├── robstride_demo_gripper.cpp     # 夹爪专用演示
│   └── python_style_test.cpp          # Python 风格测试程序
├── setup/                             # 配置工具
│   ├── configure_socketcan.sh         # CAN 接口配置脚本
│   ├── set_zero.sh                    # 电机零位设置脚本
│   ├── change_baudrate.py             # 波特率配置脚本
│   └── motor_check.cpp                # 电机诊断工具
├── CMakeLists.txt
├── package.xml
├── README.md
└── README_CN.md
```

### 依赖项

- ROS 2 Humble
- Linux 内核 CAN 支持 (SocketCAN)
- can-utils 软件包
- C++17 编译器
- CMake 3.22+

### 安装

#### 安装依赖

```bash
# 安装 CAN 工具
sudo apt-get update
sudo apt-get install can-utils

# 安装 Python 依赖（用于波特率配置脚本）
pip3 install python-can
```

#### 构建包

```bash
# 进入工作空间根目录
cd /path/to/openarmx_workspace

# 设置 ROS2 环境
source /opt/ros/humble/setup.bash

# 构建本包
colcon build --packages-select openarmx_can

# 设置环境变量
source install/setup.bash

# 验证构建成功
ls build/openarmx_can/
```

#### 构建输出

编译成功后会生成以下可执行文件：

```
./build/openarmx_can/
├── robstride-demo              # 基础演示程序
├── robstride-demo-full         # 完整 7+1 电机演示
├── robstride-demo-gripper      # 夹爪专用测试程序
├── python-style-test           # Python 风格测试程序
├── motor-check                 # 电机诊断工具
└── libopenarmx_can.a           # 静态库
```

### 快速开始

```bash
# 1. 配置 CAN 接口
sudo ./src/openarmx_can/setup/configure_socketcan.sh can0
sudo ./src/openarmx_can/setup/configure_socketcan.sh can1

# 2. 运行演示程序
# 测试单电机
./build/openarmx_can/robstride-demo

# 测试夹爪电机
./build/openarmx_can/robstride-demo-gripper

# 测试 7+1 电机
./build/openarmx_can/robstride-demo-full
```

### 使用指南

#### 1. 配置 CAN 接口

```bash
# 基本 CAN 2.0 设置（推荐）
sudo ./src/openarmx_can/setup/configure_socketcan.sh can0
sudo ./src/openarmx_can/setup/configure_socketcan.sh can1
```

#### 2. 设置电机零位

```bash
# 为单个电机设置零位
sudo ./src/openarmx_can/setup/set_zero.sh can0 5

# 为所有电机设置零位（ID 1-8）
sudo ./src/openarmx_can/setup/set_zero.sh can0 --all
```

#### 3. 电机诊断检查

```bash
# 测试单个电机
./build/openarmx_can/motor-check 5

# 指定 CAN 接口
./build/openarmx_can/motor-check 5 can0
```

### 示例程序

#### 单电机测试

```bash
./build/openarmx_can/robstride-demo
```

#### 完整机械臂演示

```bash
# 运行 7 个手臂电机 + 1 个夹爪电机的完整演示
./build/openarmx_can/robstride-demo-full
```

#### 夹爪测试

```bash
# 夹爪电机测试（推荐用于夹爪调试）
./build/openarmx_can/robstride-demo-gripper

# 指定 CAN 接口和夹爪 ID
./build/openarmx_can/robstride-demo-gripper can0 8

# 使用 CAN-FD 模式
./build/openarmx_can/robstride-demo-gripper can0 8 -fd

# 显示帮助信息
./build/openarmx_can/robstride-demo-gripper -h
```

### 编程接口

#### 基本用法

```cpp
#include <openarm/can/socket/openarm.hpp>
#include <openarm/robstride_motor/rs_motor_constants.hpp>

// 初始化 OpenArm CAN 接口
openarm::can::socket::OpenArm openarm("can0", false);  // CAN 2.0

// 初始化 7 个手臂电机
std::vector<openarm::robstride_motor::MotorType> arm_motor_types = {
    openarm::robstride_motor::MotorType::RS03,  // 关节 1
    openarm::robstride_motor::MotorType::RS03,  // 关节 2
    openarm::robstride_motor::MotorType::RS06,  // 关节 3
    openarm::robstride_motor::MotorType::RS06,  // 关节 4
    openarm::robstride_motor::MotorType::RS00,  // 关节 5
    openarm::robstride_motor::MotorType::RS00,  // 关节 6
    openarm::robstride_motor::MotorType::RS00   // 关节 7
};
std::vector<uint32_t> arm_send_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
std::vector<uint32_t> arm_recv_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

openarm.init_arm_motors(arm_motor_types, arm_send_can_ids, arm_recv_can_ids);

// 初始化夹爪
openarm.init_gripper_motor(openarm::robstride_motor::MotorType::RS00, 0x08, 0x08);

// 使能所有电机
openarm.enable_all();
openarm.recv_all(2000);

// 设置零位
openarm.set_zero_all();
openarm.recv_all(2000);

// 运动控制
std::vector<openarm::robstride_motor::MotionControlParam> motion_params;
openarm::robstride_motor::MotionControlParam param;
param.kp = 2.0;        // 位置增益（推荐温和值）
param.kd = 1.0;        // 微分增益（推荐温和值）
param.position = 0.1;  // 0.1 弧度
param.velocity = 0.0;
param.torque = 0.0;
motion_params.push_back(param);

openarm.get_arm().send_motion_control_commands(motion_params);

// 禁用电机
openarm.disable_all();
```

#### 电机状态读取

```cpp
// 设置状态回调模式
openarm.set_callback_mode_all(openarm::robstride_motor::CallbackMode::STATE);

// 刷新状态
openarm.refresh_all();
openarm.recv_all(500);

// 读取电机状态
for (const auto& motor : openarm.get_arm().get_motors()) {
    std::cout << "电机 ID: " << motor.get_send_can_id() << std::endl;
    std::cout << "位置: " << motor.get_position() << " rad" << std::endl;
    std::cout << "速度: " << motor.get_velocity() << " rad/s" << std::endl;
    std::cout << "力矩: " << motor.get_torque() << " Nm" << std::endl;
    std::cout << "温度: " << motor.get_temperature() << " °C" << std::endl;
}
```

### 工具脚本

#### 1. configure_socketcan.sh

CAN 接口配置脚本。

```bash
# 用法
sudo ./setup/configure_socketcan.sh <接口名> [选项]

# 选项
-fd                    # 启用 CAN-FD 模式
-b <波特率>            # 设置波特率（默认：1000000）
-d <数据波特率>        # 设置 CAN-FD 数据波特率（默认：5000000）
-h                     # 显示帮助

# 示例
sudo ./setup/configure_socketcan.sh can0                    # 标准 CAN 2.0 1Mbps
```

#### 2. set_zero.sh

电机零位设置脚本。

```bash
# 用法
sudo ./setup/set_zero.sh <CAN接口> [电机ID] [--all]

# 示例
sudo ./setup/set_zero.sh can0 5        # 设置电机 ID 5 的零位
sudo ./setup/set_zero.sh can0 --all    # 设置所有电机（1-8）的零位
```

#### 3. change_baudrate.py

电机波特率配置脚本。

```bash
# 用法
python3 ./setup/change_baudrate.py -b <波特率> -c <电机ID> [-s <CAN接口>] [-f]

# 示例
python3 ./setup/change_baudrate.py -b 1000000 -c 5                    # 设置电机 ID 5 波特率为 1Mbps
python3 ./setup/change_baudrate.py -b 500000 -c 1 -s can0 -f          # 设置并保存到闪存

# 支持的波特率: 125000, 250000, 500000, 1000000, 2000000, 4000000, 8000000
```

#### 4. motor-check

电机诊断程序。

```bash
# 用法
./build/openarmx_can/motor-check <电机ID> [CAN接口] [-fd]

# 示例
./build/openarmx_can/motor-check 5           # 测试电机 ID 5
./build/openarmx_can/motor-check 5 can0      # 指定 CAN 接口
```

### 技术规格

#### CAN 通信协议

- **帧格式**: CAN 2.0 扩展帧（29 位 ID）
- **波特率**: 默认 1Mbps，支持 125k-8Mbps
- **电机 ID**: 1-8（Robstride 标准范围）
- **控制频率**: 最高 1kHz

#### 电机控制模式

- **运控模式**: 位置、速度、力矩混合控制（推荐）
- **位置模式**: 纯位置控制
- **速度模式**: 纯速度控制
- **力矩模式**: 纯力矩控制

#### 安全特性

- 电机使能/禁用管理
- 参数范围检查和限制
- 温度监控和保护
- CAN 通信超时检测

### 故障排除

#### 常见问题

1. **CAN 接口未找到**
   ```bash
   # 检查 CAN 接口
   ip link show

   # 如果没有 CAN 接口，加载内核模块
   sudo modprobe can
   sudo modprobe can_raw
   sudo modprobe vcan
   ```

2. **权限不足**
   ```bash
   # 添加用户到 dialout 组
   sudo usermod -a -G dialout $USER

   # 重新登录或重启终端
   ```

3. **电机无响应**
   - 检查 CAN 总线连接
   - 确认电机 ID 配置正确（1-8）
   - 检查 CAN 接口状态：`ip -details link show can0`
   - 使用 candump 监控 CAN 流量：`candump can0`

4. **编译错误**
   ```bash
   # 清理并重新构建
   colcon build --packages-select openarmx_can --cmake-clean-cache
   ```

5. **电机运动过于激烈**
   - 降低 KP/KD 参数值（推荐：KP=2.0, KD=1.0）
   - 减小运动幅度
   - 检查零位设置是否正确

#### 调试命令

```bash
# 监控 CAN 总线流量
candump can0

# 发送测试 CAN 帧
cansend can0 123#DEADBEEF

# 检查 CAN 接口统计
ip -s link show can0

# 查看系统日志
dmesg | grep can
```

---

## 许可证

Copyright (c) Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

## 作者

- **Zhang Li** (张力)
- 公司: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 网站: https://openarmx.com/

## 版本

**当前版本**：1.0.0

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | <https://openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |
