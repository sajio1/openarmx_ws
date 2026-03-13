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

#include <openarm/can/socket/openarm.hpp>
#include <openarm/robstride_motor/rs_motor_constants.hpp>

int main() {
    try {
        std::cout << "=== OpenArm Robstride CAN Example ===" << std::endl;
        std::cout << "This example demonstrates the OpenArm API functionality with Robstride motors" << std::endl;

        // Initialize OpenArm with CAN interface - matching original demo style
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        openarm::can::socket::OpenArm openarm("can0", false);  // Use CAN 2.0 on can0 interface

        // Initialize arm motors - using tested motor ID=5 (working from Python tests)
        // Start with single motor for testing, then can expand to full 7 motors
        std::vector<openarm::robstride_motor::MotorType> motor_types = {
            openarm::robstride_motor::MotorType::RS00  // Motor ID=5 (tested and working)
        };
        std::vector<uint32_t> send_can_ids = {0x05};  // Testing gripper motor ID=8
        std::vector<uint32_t> recv_can_ids = {0x05};  // Robstride: send = recv

        openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);

        // Initialize gripper - commented out for now to focus on arm testing
        // std::cout << "Initializing gripper..." << std::endl;
        // openarm.init_gripper_motor(openarm::robstride_motor::MotorType::RS00, 0x08, 0x08);

        // Set callback mode to ignore and enable all motors - matching original demo
        openarm.set_callback_mode_all(openarm::robstride_motor::CallbackMode::IGNORE);

        // Enable all motors - following original demo pattern exactly
        std::cout << "\n=== Enabling Motors ===" << std::endl;
        openarm.enable_all();
        // Allow time (2ms) for the motors to respond for slow operations like enabling
        openarm.recv_all(2000);

        // Set motor zero positions - Robstride specific requirement
        std::cout << "\n=== Setting Motor Zero Positions ===" << std::endl;
        openarm.set_zero_all();
        openarm.recv_all(2000);

        // Set device mode to param and query motor parameters - following original demo
        std::cout << "\n=== Querying Motor Parameters ===" << std::endl;
        openarm.set_callback_mode_all(openarm::robstride_motor::CallbackMode::PARAM);
        openarm.query_param_all(static_cast<int>(openarm::robstride_motor::ParamIndex::MECH_POS));
        // Allow time (2ms) for the motors to respond for slow operations like querying
        // parameter from register
        openarm.recv_all(2000);

        // Access motors through components - following original demo format
        for (auto* motor : openarm.get_arm().get_motors()) {
            std::cout << "Arm Motor: " << motor->get_send_can_id() << " ID: "
                      << motor->get_param(static_cast<int>(openarm::robstride_motor::ParamIndex::MECH_POS))
                      << std::endl;
        }
        // Gripper motors - commented out for testing
        // for (const auto& motor : openarm.get_gripper().get_motors()) {
        //     std::cout << "Gripper Motor: " << motor.get_send_can_id() << " ID: "
        //               << motor.get_param(static_cast<int>(openarm::robstride_motor::ParamIndex::MECH_POS))
        //               << std::endl;
        // }

        // Set device mode to state and control motor - following original demo
        std::cout << "\n=== Controlling Motors ===" << std::endl;
        openarm.set_callback_mode_all(openarm::robstride_motor::CallbackMode::STATE);

        // Control arm motors with motion control - make motor move 0.1 radians
        std::vector<openarm::robstride_motor::MotionControlParam> motion_params;
        for (size_t i = 0; i < motor_types.size(); ++i) {
            openarm::robstride_motor::MotionControlParam param;
            param.kp = 20.0;        // Higher KP for better position tracking
            param.kd = 2.0;         // Higher KD for damping
            param.position = 0.2;   // Target position: 0.2 radians (about 11.5 degrees)
            param.velocity = 0.0;   // Target velocity
            param.torque = 0.0;     // Feedforward torque
            motion_params.push_back(param);
        }

        std::cout << "Moving motor to 0.2 radians..." << std::endl;
        openarm.get_arm().send_motion_control_commands(motion_params);
        openarm.recv_all(500);

        // Wait for motor to reach position
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // Return to zero position
        for (auto& param : motion_params) {
            param.position = 0.0;  // Return to zero
        }
        std::cout << "Returning motor to zero position..." << std::endl;
        openarm.get_arm().send_motion_control_commands(motion_params);
        openarm.recv_all(500);

        // Wait for motor to return
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // Control gripper - commented out for testing
        // std::cout << "Closing gripper..." << std::endl;
        // openarm.get_gripper().close();
        // openarm.recv_all(1000);

        // Monitor motor states - exactly following original demo pattern
        for (int i = 0; i < 10; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            openarm.refresh_all();
            openarm.recv_all(300);

            // Display arm motor states - following original demo format exactly
            for (auto* motor : openarm.get_arm().get_motors()) {
                std::cout << "Arm Motor: " << motor->get_send_can_id()
                          << " position: " << motor->get_position() << std::endl;
            }
            // Display gripper state - commented out for testing
            // for (const auto& motor : openarm.get_gripper().get_motors()) {
            //     std::cout << "Gripper Motor: " << motor.get_send_can_id()
            //               << " position: " << motor.get_position() << std::endl;
            // }
        }

        // Disable all motors - following original demo exactly
        openarm.disable_all();
        openarm.recv_all(1000);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}