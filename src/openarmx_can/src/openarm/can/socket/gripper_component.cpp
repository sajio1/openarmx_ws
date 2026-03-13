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

#include <linux/can.h>
#include <linux/can/raw.h>

#include <iostream>
#include <openarm/can/socket/gripper_component.hpp>

namespace openarm::can::socket {

GripperComponent::GripperComponent(canbus::CANSocket& can_socket)
    : RSMotorDeviceCollection(&can_socket), can_socket_(can_socket) {}

void GripperComponent::init_motor_device(robstride_motor::MotorType motor_type, uint32_t send_can_id,
                                         uint32_t recv_can_id, bool use_fd) {
    // 创建电机实例
    auto motor = std::make_unique<robstride_motor::Motor>(motor_type, send_can_id, recv_can_id);

    // 添加到集合中
    add_motor(std::move(motor), use_fd);
}

void GripperComponent::open(double kp, double kd) { set_position(gripper_open_position_, kp, kd); }

void GripperComponent::close(double kp, double kd) {
    set_position(gripper_closed_position_, kp, kd);
}

void GripperComponent::set_position(double gripper_position, double kp, double kd) {
    auto motors = get_all_motors();
    if (motors.empty()) return;

    // Robstride运控模式控制
    robstride_motor::MotionControlParam param;
    param.kp = kp;
    param.kd = kd;
    param.position = gripper_to_motor_position(gripper_position);
    param.velocity = 0.0;
    param.torque = 0.0;

    // 发送控制命令
    send_motion_control_commands({param});
}
}  // namespace openarm::can::socket
