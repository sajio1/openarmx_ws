// Copyright 2025 成都长数机器人有限公司 (Chengdu Changsu Robot Co., Ltd.)
// Website: https://openarmx.com/
// Contact: Mr Wang
// Phone & WeChat: +86-17746530375
// Email: openarmrobot@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author Information:
// Company: 成都长数机器人有限公司 (Chengdu Changsu Robot Co., Ltd.)
// Website: https://openarmx.com/
// Contact Person: Mr Wang
// Phone & WeChat: +86-17746530375
// Email: openarmrobot@gmail.com

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/robstride_motor/rs_motor_constants.hpp>
#include <openarm/robstride_motor/rs_motor_control.hpp>

namespace {
void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [can_interface] [gripper_id] [-fd]" << std::endl;
    std::cout << "  can_interface: CAN interface name (default: can0)" << std::endl;
    std::cout << "  gripper_id: Gripper motor CAN ID (default: 8)" << std::endl;
    std::cout << "  -fd: Enable CAN-FD (default: disabled)" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << "                    # Default: can0, ID 8" << std::endl;
    std::cout << "  " << program_name << " can0 8           # Specify interface and ID" << std::endl;
    std::cout << "  " << program_name << " can0 8 -fd       # Use CAN-FD" << std::endl;
    std::cout << std::endl;
    std::cout << "Note: Robstride gripper typically uses RS00 motor type" << std::endl;
}

void print_gripper_status(const openarm::robstride_motor::Motor* gripper) {
    std::cout << "=== Gripper Status ===" << std::endl;
    std::cout << "Motor ID: " << gripper->get_send_can_id() << std::endl;
    std::cout << "Position: " << gripper->get_position() << " rad" << std::endl;
    std::cout << "Velocity: " << gripper->get_velocity() << " rad/s" << std::endl;
    std::cout << "Torque: " << gripper->get_torque() << " Nm" << std::endl;
    std::cout << "Temperature: " << gripper->get_temperature() << " °C" << std::endl;
    std::cout << "Pattern: " << gripper->get_pattern() << std::endl;
    std::cout << "Enabled: " << (gripper->is_enabled() ? "Yes" : "No") << std::endl;
    std::cout << "===================" << std::endl;
}
}  // namespace

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string can_interface = "can0";
    uint32_t gripper_id = 8;  // Default gripper ID
    bool use_fd = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-fd") {
            use_fd = true;
        } else if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (i == 1) {
            can_interface = arg;
        } else if (i == 2) {
            try {
                gripper_id = std::stoul(arg);
                if (gripper_id < 1 || gripper_id > 8) {
                    std::cerr << "Error: Gripper ID must be between 1 and 8" << std::endl;
                    return 1;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error: Invalid gripper ID format" << std::endl;
                print_usage(argv[0]);
                return 1;
            }
        }
    }

    try {
        std::cout << "=== OpenArm Robstride Gripper Test ===\n";
        std::cout << "Testing gripper motor with Robstride RS00 type\n";
        std::cout << "CAN Interface: " << can_interface << std::endl;
        std::cout << "Gripper ID: " << gripper_id << std::endl;
        std::cout << "CAN-FD: " << (use_fd ? "Enabled" : "Disabled") << std::endl;
        std::cout << "\n";

        // Initialize OpenArm with CAN interface
        std::cout << "Initializing OpenArm CAN...\n";
        openarm::can::socket::OpenArm openarm(can_interface, use_fd);

        // Initialize gripper motor only - RS00 type (DM4310 equivalent)
        std::cout << "Initializing Robstride gripper motor (RS00)...\n";
        openarm.init_gripper_motor(openarm::robstride_motor::MotorType::RS00, gripper_id, gripper_id);

        // Set callback mode to param for initial parameter reading
        openarm.set_callback_mode_all(openarm::robstride_motor::CallbackMode::PARAM);

        // Query motor parameters
        std::cout << "Reading gripper motor parameters...\n";
        openarm.query_param_all(static_cast<int>(openarm::robstride_motor::ParamIndex::MECH_POS));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all(2000);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Get gripper and verify parameters
        const auto& grippers = openarm.get_gripper().get_all_motors();
        if (!grippers.empty()) {
            const auto* gripper = grippers[0];
            double queried_pos =
                gripper->get_param(static_cast<int>(openarm::robstride_motor::ParamIndex::MECH_POS));

            std::cout << "\n=== Gripper Motor Parameters ===\n";
            std::cout << "Motor CAN ID: " << gripper->get_send_can_id() << std::endl;
            std::cout << "Queried Mechanical Position: " << queried_pos << std::endl;
            std::cout << "✓ Parameter reading completed\n";
        }

        // Switch to state callback mode for motor status updates
        openarm.set_callback_mode_all(openarm::robstride_motor::CallbackMode::STATE);

        // Set gripper motor to motion control mode (CRITICAL STEP!)
        std::cout << "\n=== Setting Gripper Control Mode (Motion Control) ===\n";
        if (!grippers.empty()) {
            const auto* gripper_motor = grippers[0];

            // Create control mode command - matching Python implementation
            auto mode_cmd = openarm::robstride_motor::CanPacketEncoder::create_set_control_mode_command(
                *gripper_motor, openarm::robstride_motor::ControlMode::MOTION_CONTROL);

            // Send command using master CAN device collection and CAN socket directly
            auto& master_collection = openarm.get_master_can_device_collection();
            auto& can_socket = master_collection.get_can_socket();

            // Create CAN frame
            can_frame frame;
            frame.can_id = mode_cmd.send_can_id | CAN_EFF_FLAG;  // Extended frame
            frame.can_dlc = std::min(static_cast<size_t>(8), mode_cmd.data.size());

            std::fill(frame.data, frame.data + 8, 0);
            std::copy(mode_cmd.data.begin(), mode_cmd.data.begin() + frame.can_dlc, frame.data);

            if (can_socket.write_can_frame(frame)) {
                std::cout << "设置夹爪电机控制模式为运控模式 OK" << std::endl;
            } else {
                std::cout << "设置夹爪电机控制模式失败" << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        openarm.recv_all(1000);

        // Enable the gripper motor
        std::cout << "\n=== Enabling Gripper Motor ===\n";
        openarm.enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all(2000);

        // Set zero position
        std::cout << "\n=== Setting Gripper Zero Position ===\n";
        openarm.set_zero_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all(2000);

        // Initial status check
        std::cout << "\n=== Initial Gripper Status ===\n";
        openarm.refresh_all();
        openarm.recv_all(500);

        if (!grippers.empty()) {
            print_gripper_status(grippers[0]);
        }

        // Test gripper movement sequence
        std::cout << "\n=== Starting Gripper Movement Test ===\n";

        // Create gripper motion control parameters
        std::vector<openarm::robstride_motor::MotionControlParam> gripper_params;
        openarm::robstride_motor::MotionControlParam param;
        param.kp = 10.0;    // 按照Python版本参数
        param.kd = 0.5;     // 按照Python版本参数
        param.velocity = 0.0;
        param.torque = 0.0;

        // Test sequence: multiple open/close cycles
        std::vector<double> positions = {0.5, 0.0, 1.0, 0.0, 0.8, 0.2, 0.0};
        std::vector<std::string> descriptions = {
            "Half close", "Open", "Full close", "Open",
            "Partial close", "Slightly closed", "Final open"
        };

        for (size_t i = 0; i < positions.size(); i++) {
            param.position = positions[i];
            gripper_params.clear();
            gripper_params.push_back(param);

            std::cout << "\n--- Step " << (i + 1) << "/" << positions.size()
                      << ": " << descriptions[i]
                      << " (Position: " << positions[i] << " rad) ---\n";

            openarm.get_gripper().send_motion_control_commands(gripper_params);
            openarm.recv_all(500);

            // Wait for movement to complete
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            // Check status during movement
            openarm.refresh_all();
            openarm.recv_all(500);

            if (!grippers.empty()) {
                const auto* gripper = grippers[0];
                std::cout << "Current position: " << gripper->get_position() << " rad" << std::endl;
                std::cout << "Current velocity: " << gripper->get_velocity() << " rad/s" << std::endl;
                std::cout << "Current torque: " << gripper->get_torque() << " Nm" << std::endl;
                std::cout << "Temperature: " << gripper->get_temperature() << " °C" << std::endl;
            }
        }

        // Monitor gripper status for final verification
        std::cout << "\n=== Final Status Monitoring (5 cycles) ===\n";
        for (int i = 0; i < 5; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            openarm.refresh_all();
            openarm.recv_all(300);

            if (!grippers.empty()) {
                std::cout << "\n--- Status Check " << (i + 1) << "/5 ---\n";
                print_gripper_status(grippers[0]);
            }
        }

        // Disable the gripper motor
        std::cout << "\n=== Disabling Gripper Motor ===\n";
        openarm.disable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all(1000);

        // Final status
        if (!grippers.empty()) {
            std::cout << "\n=== Final Gripper Status ===\n";
            print_gripper_status(grippers[0]);
        }

        std::cout << "\n=== Robstride Gripper Test Completed Successfully ===\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cout << "\nTroubleshooting tips:\n";
        std::cout << "1. Check CAN interface: ip link show " << can_interface << std::endl;
        std::cout << "2. Configure CAN: sudo ./setup/configure_socketcan.sh " << can_interface << std::endl;
        std::cout << "3. Check motor power and connections\n";
        std::cout << "4. Verify gripper motor ID is " << gripper_id << std::endl;
        return -1;
    }

    return 0;
}