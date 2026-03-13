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
#include <openarm/robstride_motor/rs_motor_control.hpp>

int main() {
    try {
        std::cout << "=== OpenArm Robstride CAN Full Demo ===\n";
        std::cout << "This example demonstrates complete 7-DOF arm + gripper functionality with Robstride motors\n";

        // Initialize OpenArm with CAN interface - matching original demo style
        std::cout << "Initializing OpenArm CAN...\n";
        openarm::can::socket::OpenArm openarm("can0", false);  // Use CAN 2.0 on can0 interface

        // Initialize 7 arm motors based on latest motor mapping:
        // Joint 1-2: DM8009 → RS04 (新版)
        // Joint 3-4: DM4340 → RS03 (新版)
        // Joint 5-7: DM4310 → RS00
        std::vector<openarm::robstride_motor::MotorType> arm_motor_types = {
            openarm::robstride_motor::MotorType::RS04,  // Joint 1 (新版: RS04替代DM8009)
            openarm::robstride_motor::MotorType::RS04,  // Joint 2 (新版: RS04替代DM8009)
            openarm::robstride_motor::MotorType::RS03,  // Joint 3 (新版: RS03替代DM4340)
            openarm::robstride_motor::MotorType::RS03,  // Joint 4 (新版: RS03替代DM4340)
            openarm::robstride_motor::MotorType::RS00,  // Joint 5 (was DM4310)
            openarm::robstride_motor::MotorType::RS00,  // Joint 6 (was DM4310)
            openarm::robstride_motor::MotorType::RS00   // Joint 7 (was DM4310)
        };
        std::vector<uint32_t> arm_send_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        std::vector<uint32_t> arm_recv_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};  // Robstride: send = recv

        openarm.init_arm_motors(arm_motor_types, arm_send_can_ids, arm_recv_can_ids);

        // Initialize gripper - DM4310 → RS00
        std::cout << "Initializing gripper...\n";
        openarm.init_gripper_motor(openarm::robstride_motor::MotorType::RS00, 0x08, 0x08);

        // Set callback mode to ignore and enable all motors - matching original demo
        openarm.set_callback_mode_all(openarm::robstride_motor::CallbackMode::IGNORE);

        // Enable all motors - following original demo pattern exactly
        std::cout << "\n=== Enabling Motors ===\n";
        openarm.enable_all();
        // Allow time (2ms) for the motors to respond for slow operations like enabling
        openarm.recv_all(2000);

        // Set motor zero positions - Robstride specific requirement
        std::cout << "\n=== Setting Motor Zero Positions ===\n";
        openarm.set_zero_all();
        openarm.recv_all(2000);

        // Set device mode to param and query motor parameters - following original demo
        std::cout << "\n=== Querying Motor Parameters ===\n";
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
        for (auto* motor : openarm.get_gripper().get_motors()) {
            std::cout << "Gripper Motor: " << motor->get_send_can_id() << " ID: "
                      << motor->get_param(static_cast<int>(openarm::robstride_motor::ParamIndex::MECH_POS))
                      << std::endl;
        }

        // Set device mode to state and control motor - following original demo
        std::cout << "\n=== Controlling Motors ===\n";
        openarm.set_callback_mode_all(openarm::robstride_motor::CallbackMode::STATE);

        // Control arm motors with motion control - make each motor move small amount
        std::vector<openarm::robstride_motor::MotionControlParam> motion_params;
        for (size_t i = 0; i < arm_motor_types.size(); ++i) {
            openarm::robstride_motor::MotionControlParam param;
            param.kp = 2.0;                           // Gentler KP for smoother motion
            param.kd = 1.0;                           // Gentler KD for smoother motion
            param.position = 0.1 * (i % 2 == 0 ? 1 : -1);  // Alternating small movements: ±0.1 rad
            param.velocity = 0.0;                     // Target velocity
            param.torque = 0.0;                       // Feedforward torque
            motion_params.push_back(param);
        }

        std::cout << "Moving arm motors (alternating ±0.1 radians)...\n";
        openarm.get_arm().send_motion_control_commands(motion_params);
        openarm.recv_all(500);

        // Wait for motors to reach positions
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        // Return arm motors to zero position
        for (auto& param : motion_params) {
            param.position = 0.0;  // Return to zero
        }
        std::cout << "Returning arm motors to zero position...\n";
        openarm.get_arm().send_motion_control_commands(motion_params);
        openarm.recv_all(500);

        // Wait for motors to return
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // Control gripper - close and open
        std::cout << "\n=== Testing Gripper ===\n";
        std::cout << "Closing gripper...\n";

        // Create gripper motion control command
        std::vector<openarm::robstride_motor::MotionControlParam> gripper_close_params;
        openarm::robstride_motor::MotionControlParam gripper_param;
        gripper_param.kp = 2.0;         // Gentler KP for gripper
        gripper_param.kd = 1.0;         // Gentler KD for gripper
        gripper_param.position = 1.0;   // Close position - larger motion
        gripper_param.velocity = 0.0;
        gripper_param.torque = 0.0;
        gripper_close_params.push_back(gripper_param);

        openarm.get_gripper().send_motion_control_commands(gripper_close_params);
        openarm.recv_all(1000);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // Open gripper
        std::cout << "Opening gripper...\n";
        gripper_param.position = 0.0;   // Open position
        std::vector<openarm::robstride_motor::MotionControlParam> gripper_open_params = {gripper_param};
        openarm.get_gripper().send_motion_control_commands(gripper_open_params);
        openarm.recv_all(1000);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // Monitor motor states - exactly following original demo pattern
        std::cout << "\n=== Monitoring All Motor States ===\n";
        for (int i = 0; i < 10; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            openarm.refresh_all();
            openarm.recv_all(300);

            // Display arm motor states - following original demo format exactly
            for (auto* motor : openarm.get_arm().get_motors()) {
                std::cout << "Arm Motor: " << motor->get_send_can_id()
                          << " position: " << motor->get_position() << std::endl;
            }
            // Display gripper state
            for (auto* motor : openarm.get_gripper().get_motors()) {
                std::cout << "Gripper Motor: " << motor->get_send_can_id()
                          << " position: " << motor->get_position() << std::endl;
            }
            std::cout << "--- Cycle " << i + 1 << " ---\n";
        }

        // Disable all motors - following original demo exactly
        std::cout << "\n=== Disabling All Motors ===\n";
        openarm.disable_all();
        openarm.recv_all(1000);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}