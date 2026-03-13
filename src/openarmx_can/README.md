# OpenArmX CAN

[English](#overview) | [中文](README_CN.md)

---

### Overview

OpenArmX CAN is a motor control library for the OpenArmX robot system, developed by Chengdu Changshu Robot Co., Ltd. ([openarmx.com](https://openarmx.com)). This library provides a complete CAN communication interface for Robstride motors, supporting 7-DOF robotic arm and gripper control.

**Official Website**: [openarmx.com](https://openarmx.com)

### Features

- **Motor Type Support**: RS00, RS03, RS04, RS06 (Robstride motor series)
- **Multiple Control Modes**: Motion control, position mode, velocity mode, current mode
- **CAN Bus Communication**: Supports CAN 2.0 and CAN-FD
- **Real-time Control**: High-frequency motor control and state feedback
- **State Monitoring**: Motor position, velocity, torque, temperature monitoring
- **Parameter Configuration**: Motor parameter read/write
- **Error Handling**: Complete error detection and handling mechanism

### Motor Type Mapping

| Robstride Motor | Joint Application |
|-----------------|-------------------|
| RS04            | Joint 1-2         |
| RS03            | Joint 3-4         |
| RS00            | Joint 5-7, Gripper|

### Package Structure

```
openarmx_can/
├── include/openarm/
│   ├── can/
│   │   └── socket/                    # CAN socket interface
│   │       └── openarm.hpp            # Main OpenArm interface
│   ├── canbus/                        # CAN bus communication layer
│   │   ├── can_socket.hpp
│   │   ├── can_device.hpp
│   │   └── can_device_collection.hpp
│   └── robstride_motor/               # Robstride motor layer
│       ├── rs_motor_constants.hpp     # Constants and type definitions
│       ├── rs_motor.hpp               # Motor base class
│       ├── rs_motor_control.hpp       # Control command encoding/decoding
│       ├── rs_motor_device.hpp        # CAN device management
│       └── rs_motor_device_collection.hpp  # Device collection management
├── src/openarm/
│   ├── can/socket/                    # CAN socket implementation
│   ├── canbus/                        # CAN communication implementation
│   └── robstride_motor/               # Robstride motor implementation
├── examples/                          # Example programs
│   ├── robstride_demo.cpp             # Basic demo program
│   ├── robstride_demo_full.cpp        # Full 7+1 motor demo
│   ├── robstride_demo_gripper.cpp     # Gripper-specific demo
│   └── python_style_test.cpp          # Python-style test program
├── setup/                             # Configuration tools
│   ├── configure_socketcan.sh         # CAN interface configuration script
│   ├── set_zero.sh                    # Motor zero position script
│   ├── change_baudrate.py             # Baudrate configuration script
│   └── motor_check.cpp                # Motor diagnostic tool
├── CMakeLists.txt
├── package.xml
├── README.md
└── README_CN.md
```

### Dependencies

- ROS 2 Humble
- Linux kernel CAN support (SocketCAN)
- can-utils package
- C++17 compiler
- CMake 3.22+

### Installation

#### Install Dependencies

```bash
# Install CAN tools
sudo apt-get update
sudo apt-get install can-utils

# Install Python dependencies (for baudrate configuration script)
pip3 install python-can
```

#### Build the Package

```bash
# Enter workspace root directory
cd /path/to/openarmx_workspace

# Set ROS2 environment
source /opt/ros/humble/setup.bash

# Build the package
colcon build --packages-select openarmx_can

# Source the workspace
source install/setup.bash

# Verify build success
ls build/openarmx_can/
```

#### Build Output

After successful compilation, the following executables are generated:

```
./build/openarmx_can/
├── robstride-demo              # Basic demo program
├── robstride-demo-full         # Full 7+1 motor demo
├── robstride-demo-gripper      # Gripper-specific test program
├── python-style-test           # Python-style test program
├── motor-check                 # Motor diagnostic tool
└── libopenarmx_can.a           # Static library
```

### Quick Start

```bash
# 1. Configure CAN interface
sudo ./src/openarmx_can/setup/configure_socketcan.sh can0
sudo ./src/openarmx_can/setup/configure_socketcan.sh can1

# 2. Run demo programs
# Test single motor
./build/openarmx_can/robstride-demo

# Test gripper motor
./build/openarmx_can/robstride-demo-gripper

# Test 7+1 motors
./build/openarmx_can/robstride-demo-full
```

### Usage Guide

#### 1. Configure CAN Interface

```bash
# Basic CAN 2.0 setup (recommended)
sudo ./src/openarmx_can/setup/configure_socketcan.sh can0
sudo ./src/openarmx_can/setup/configure_socketcan.sh can1
```

#### 2. Set Motor Zero Position

```bash
# Set zero position for a single motor
sudo ./src/openarmx_can/setup/set_zero.sh can0 5

# Set zero position for all motors (ID 1-8)
sudo ./src/openarmx_can/setup/set_zero.sh can0 --all
```

#### 3. Motor Diagnostic Check

```bash
# Test a single motor
./build/openarmx_can/motor-check 5

# Specify CAN interface
./build/openarmx_can/motor-check 5 can0
```

### Example Programs

#### Single Motor Test

```bash
./build/openarmx_can/robstride-demo
```

#### Full Arm Demo

```bash
# Run 7 arm motors + 1 gripper motor demo
./build/openarmx_can/robstride-demo-full
```

#### Gripper Test

```bash
# Gripper motor test (recommended for gripper debugging)
./build/openarmx_can/robstride-demo-gripper

# Specify CAN interface and gripper ID
./build/openarmx_can/robstride-demo-gripper can0 8

# Use CAN-FD mode
./build/openarmx_can/robstride-demo-gripper can0 8 -fd

# Show help
./build/openarmx_can/robstride-demo-gripper -h
```

### Programming Interface

#### Basic Usage

```cpp
#include <openarm/can/socket/openarm.hpp>
#include <openarm/robstride_motor/rs_motor_constants.hpp>

// Initialize OpenArm CAN interface
openarm::can::socket::OpenArm openarm("can0", false);  // CAN 2.0

// Initialize 7 arm motors
std::vector<openarm::robstride_motor::MotorType> arm_motor_types = {
    openarm::robstride_motor::MotorType::RS03,  // Joint 1
    openarm::robstride_motor::MotorType::RS03,  // Joint 2
    openarm::robstride_motor::MotorType::RS06,  // Joint 3
    openarm::robstride_motor::MotorType::RS06,  // Joint 4
    openarm::robstride_motor::MotorType::RS00,  // Joint 5
    openarm::robstride_motor::MotorType::RS00,  // Joint 6
    openarm::robstride_motor::MotorType::RS00   // Joint 7
};
std::vector<uint32_t> arm_send_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
std::vector<uint32_t> arm_recv_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

openarm.init_arm_motors(arm_motor_types, arm_send_can_ids, arm_recv_can_ids);

// Initialize gripper
openarm.init_gripper_motor(openarm::robstride_motor::MotorType::RS00, 0x08, 0x08);

// Enable all motors
openarm.enable_all();
openarm.recv_all(2000);

// Set zero position
openarm.set_zero_all();
openarm.recv_all(2000);

// Motion control
std::vector<openarm::robstride_motor::MotionControlParam> motion_params;
openarm::robstride_motor::MotionControlParam param;
param.kp = 2.0;        // Position gain (recommended gentle value)
param.kd = 1.0;        // Derivative gain (recommended gentle value)
param.position = 0.1;  // 0.1 radians
param.velocity = 0.0;
param.torque = 0.0;
motion_params.push_back(param);

openarm.get_arm().send_motion_control_commands(motion_params);

// Disable motors
openarm.disable_all();
```

#### Motor State Reading

```cpp
// Set state callback mode
openarm.set_callback_mode_all(openarm::robstride_motor::CallbackMode::STATE);

// Refresh state
openarm.refresh_all();
openarm.recv_all(500);

// Read motor state
for (const auto& motor : openarm.get_arm().get_motors()) {
    std::cout << "Motor ID: " << motor.get_send_can_id() << std::endl;
    std::cout << "Position: " << motor.get_position() << " rad" << std::endl;
    std::cout << "Velocity: " << motor.get_velocity() << " rad/s" << std::endl;
    std::cout << "Torque: " << motor.get_torque() << " Nm" << std::endl;
    std::cout << "Temperature: " << motor.get_temperature() << " °C" << std::endl;
}
```

### Tool Scripts

#### 1. configure_socketcan.sh

CAN interface configuration script.

```bash
# Usage
sudo ./setup/configure_socketcan.sh <interface> [options]

# Options
-fd                    # Enable CAN-FD mode
-b <baudrate>          # Set baudrate (default: 1000000)
-d <data_baudrate>     # Set CAN-FD data baudrate (default: 5000000)
-h                     # Show help

# Examples
sudo ./setup/configure_socketcan.sh can0                    # Standard CAN 2.0 1Mbps
```

#### 2. set_zero.sh

Motor zero position setting script.

```bash
# Usage
sudo ./setup/set_zero.sh <CAN_interface> [motor_ID] [--all]

# Examples
sudo ./setup/set_zero.sh can0 5        # Set zero for motor ID 5
sudo ./setup/set_zero.sh can0 --all    # Set zero for all motors (1-8)
```

#### 3. change_baudrate.py

Motor baudrate configuration script.

```bash
# Usage
python3 ./setup/change_baudrate.py -b <baudrate> -c <motor_ID> [-s <CAN_interface>] [-f]

# Examples
python3 ./setup/change_baudrate.py -b 1000000 -c 5                    # Set motor ID 5 to 1Mbps
python3 ./setup/change_baudrate.py -b 500000 -c 1 -s can0 -f          # Set and save to flash

# Supported baudrates: 125000, 250000, 500000, 1000000, 2000000, 4000000, 8000000
```

#### 4. motor-check

Motor diagnostic program.

```bash
# Usage
./build/openarmx_can/motor-check <motor_ID> [CAN_interface] [-fd]

# Examples
./build/openarmx_can/motor-check 5           # Test motor ID 5
./build/openarmx_can/motor-check 5 can0      # Specify CAN interface
```

### Technical Specifications

#### CAN Communication Protocol

- **Frame Format**: CAN 2.0 extended frame (29-bit ID)
- **Baudrate**: Default 1Mbps, supports 125k-8Mbps
- **Motor ID**: 1-8 (Robstride standard range)
- **Control Frequency**: Up to 1kHz

#### Motor Control Modes

- **Motion Control Mode**: Position, velocity, torque mixed control (recommended)
- **Position Mode**: Pure position control
- **Velocity Mode**: Pure velocity control
- **Torque Mode**: Pure torque control

#### Safety Features

- Motor enable/disable management
- Parameter range checking and limiting
- Temperature monitoring and protection
- CAN communication timeout detection

### Troubleshooting

#### Common Issues

1. **CAN interface not found**
   ```bash
   # Check CAN interface
   ip link show

   # Load kernel modules if no CAN interface
   sudo modprobe can
   sudo modprobe can_raw
   sudo modprobe vcan
   ```

2. **Permission denied**
   ```bash
   # Add user to dialout group
   sudo usermod -a -G dialout $USER

   # Re-login or restart terminal
   ```

3. **Motor not responding**
   - Check CAN bus connection
   - Confirm motor ID configuration (1-8)
   - Check CAN interface status: `ip -details link show can0`
   - Monitor CAN traffic: `candump can0`

4. **Build errors**
   ```bash
   # Clean and rebuild
   colcon build --packages-select openarmx_can --cmake-clean-cache
   ```

5. **Motor motion too aggressive**
   - Lower KP/KD parameter values (recommended: KP=2.0, KD=1.0)
   - Reduce motion amplitude
   - Check if zero position is set correctly

#### Debug Commands

```bash
# Monitor CAN bus traffic
candump can0

# Send test CAN frame
cansend can0 123#DEADBEEF

# Check CAN interface statistics
ip -s link show can0

# View system logs
dmesg | grep can
```

---

## License

Copyright (c) Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

## Author

- **Zhang Li** (张力)
- Company: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- Website: https://openarmx.com/

## Version

**Current Version**: 1.0.0

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.

| Contact           | Information                                                                                                  |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| 📧 Email          | [openarmrobot@gmail.com](mailto:openarmrobot@gmail.com)                                                      |
| 📱 Phone / WeChat | +86-17746530375                                                                                              |
| 🌐 Website        | [https://openarmx.com/](https://openarmx.com/)                                                               |
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
