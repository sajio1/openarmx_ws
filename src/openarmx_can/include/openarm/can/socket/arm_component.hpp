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

#pragma once

#include <vector>

#include "../../canbus/can_socket.hpp"
#include "../../robstride_motor/rs_motor.hpp"
#include "../../robstride_motor/rs_motor_device_collection.hpp"

namespace openarm::can::socket {

class ArmComponent : public robstride_motor::RSMotorDeviceCollection {
public:
    ArmComponent(canbus::CANSocket& can_socket);
    ~ArmComponent() = default;

    void init_motor_devices(const std::vector<robstride_motor::MotorType>& motor_types,
                            const std::vector<uint32_t>& send_can_ids,
                            const std::vector<uint32_t>& recv_can_ids, bool use_fd);

private:
    canbus::CANSocket& can_socket_;
};

}  // namespace openarm::can::socket
