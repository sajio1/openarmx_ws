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
#include <openarm/can/socket/arm_component.hpp>

namespace openarm::can::socket {

ArmComponent::ArmComponent(canbus::CANSocket& can_socket)
    : RSMotorDeviceCollection(&can_socket), can_socket_(can_socket) {}

void ArmComponent::init_motor_devices(const std::vector<robstride_motor::MotorType>& motor_types,
                                      const std::vector<uint32_t>& send_can_ids,
                                      const std::vector<uint32_t>& recv_can_ids, bool use_fd) {
    for (size_t i = 0; i < motor_types.size(); i++) {
        // 创建电机实例
        auto motor = std::make_unique<robstride_motor::Motor>(
            motor_types[i], send_can_ids[i], recv_can_ids[i]);

        // 添加到集合中
        add_motor(std::move(motor), use_fd);
    }
}

}  // namespace openarm::can::socket
