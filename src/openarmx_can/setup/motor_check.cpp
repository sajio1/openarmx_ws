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

#include <chrono>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/robstride_motor/rs_motor_constants.hpp>
#include <thread>

namespace {
void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <motor_id> [can_interface] [-fd]" << std::endl;
    std::cout << "  motor_id: The motor CAN ID (1-8)" << std::endl;
    std::cout << "  can_interface: CAN interface name (default: can0)" << std::endl;
    std::cout << "  -fd: Enable CAN-FD (default: disabled)" << std::endl;
    std::cout << std::endl;
    std::cout << "Example: " << program_name << " 5" << std::endl;
    std::cout << "Example: " << program_name << " 5 can1" << std::endl;
    std::cout << "Example: " << program_name << " 5 can1 -fd" << std::endl;
}

void print_motor_status(const openarm::robstride_motor::Motor& motor) {
    std::cout << "Motor ID: " << motor.get_send_can_id() << std::endl;
    std::cout << "  Position: " << motor.get_position() << " rad" << std::endl;
    std::cout << "  Velocity: " << motor.get_velocity() << " rad/s" << std::endl;
    std::cout << "  Torque: " << motor.get_torque() << " Nm" << std::endl;
    std::cout << "  Temperature: " << motor.get_temperature() << " °C" << std::endl;
    std::cout << "  Pattern: " << motor.get_pattern() << std::endl;
    std::cout << "  Enabled: " << (motor.is_enabled() ? "Yes" : "No") << std::endl;
}
}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 4) {
        print_usage(argv[0]);
        return 1;
    }

    // Parse motor ID from command line
    uint32_t motor_id;
    try {
        motor_id = std::stoul(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "Error: Invalid motor ID format" << std::endl;
        print_usage(argv[0]);
        return 1;
    }

    // Validate motor ID range for Robstride
    if (motor_id < 1 || motor_id > 8) {
        std::cerr << "Error: Motor ID must be between 1 and 8 for Robstride motors" << std::endl;
        return 1;
    }

    // Parse optional CAN interface and FD flag
    std::string can_interface = "can0";  // default
    bool use_fd = false;                 // default: disabled

    if (argc >= 3) {
        std::string arg3 = argv[2];
        if (arg3 == "-fd") {
            use_fd = true;
        } else {
            can_interface = arg3;
        }
    }

    if (argc >= 4) {
        std::string arg4 = argv[3];
        if (arg4 == "-fd") {
            use_fd = true;
        } else {
            std::cerr << "Error: Unknown argument '" << arg4 << "'. Use -fd to enable CAN-FD"
                      << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }

    try {
        std::cout << "=== Robstride Motor Check Script ===" << std::endl;
        std::cout << "Motor ID: " << motor_id << std::endl;
        std::cout << "CAN Interface: " << can_interface << std::endl;
        std::cout << "CAN-FD Enabled: " << (use_fd ? "Yes" : "No") << std::endl;
        std::cout << std::endl;

        // Initialize OpenArm with CAN interface
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        openarm::can::socket::OpenArm openarm(can_interface, use_fd);

        // Initialize single motor - Robstride uses same ID for send and receive
        std::cout << "Initializing Robstride motor..." << std::endl;
        openarm.init_arm_motors({openarm::robstride_motor::MotorType::RS00},
                                {motor_id},
                                {motor_id});  // Robstride: send_id = recv_id

        // Set callback mode to param for initial parameter reading
        openarm.set_callback_mode_all(openarm::robstride_motor::CallbackMode::PARAM);

        // Query motor parameters (Mechanical Position)
        std::cout << "Reading motor parameters..." << std::endl;
        openarm.query_param_all(static_cast<int>(openarm::robstride_motor::ParamIndex::MECH_POS));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all(2000);  // Give more time for Robstride response
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Get motor and verify parameters
        const auto& motors = openarm.get_arm().get_motors();
        if (!motors.empty()) {
            auto* motor = motors[0];
            double queried_pos =
                motor->get_param(static_cast<int>(openarm::robstride_motor::ParamIndex::MECH_POS));

            std::cout << "\n=== Motor Parameters ===" << std::endl;
            std::cout << "Motor CAN ID: " << motor->get_send_can_id() << std::endl;
            std::cout << "Queried Mechanical Position: " << queried_pos << std::endl;
            std::cout << "✓ Parameter reading completed" << std::endl;
        }

        // Switch to state callback mode for motor status updates
        openarm.set_callback_mode_all(openarm::robstride_motor::CallbackMode::STATE);

        // Enable the motor
        std::cout << "\n=== Enabling Motor ===" << std::endl;
        openarm.enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all(2000);

        // Set zero position
        std::cout << "\n=== Setting Zero Position ===" << std::endl;
        openarm.set_zero_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all(2000);

        // Refresh 10 times at 10Hz (100ms intervals)
        std::cout << "\n=== Refreshing Motor Status (10Hz for 1 second) ===" << std::endl;
        for (int i = 1; i <= 10; i++) {
            openarm.refresh_all();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            openarm.recv_all(500);

            for (auto* motor : openarm.get_arm().get_motors()) {
                std::cout << "\n--- Refresh " << i << "/10 ---" << std::endl;
                print_motor_status(*motor);
            }
        }

        // Test small motion control
        std::cout << "\n=== Testing Small Motion Control ===" << std::endl;
        std::vector<openarm::robstride_motor::MotionControlParam> motion_params;
        openarm::robstride_motor::MotionControlParam param;
        param.kp = 2.0;           // Gentle control
        param.kd = 1.0;           // Gentle damping
        param.position = 0.1;     // Small movement: 0.1 rad
        param.velocity = 0.0;
        param.torque = 0.0;
        motion_params.push_back(param);

        std::cout << "Moving motor to 0.1 radians..." << std::endl;
        openarm.get_arm().send_motion_control_commands(motion_params);
        openarm.recv_all(500);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // Return to zero
        param.position = 0.0;
        motion_params[0] = param;
        std::cout << "Returning motor to zero position..." << std::endl;
        openarm.get_arm().send_motion_control_commands(motion_params);
        openarm.recv_all(500);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Disable the motor
        std::cout << "\n=== Disabling Motor ===" << std::endl;
        openarm.disable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all(1000);

        // Print final status
        if (!motors.empty()) {
            std::cout << "\n=== Final Motor Status ===" << std::endl;
            print_motor_status(*motors[0]);
        }

        std::cout << "\n=== Robstride Motor Check Completed Successfully ===" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}