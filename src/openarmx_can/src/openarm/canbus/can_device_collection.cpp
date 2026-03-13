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

#include <iostream>
#include <openarm/canbus/can_device_collection.hpp>
#include <openarm/canbus/can_socket.hpp>

namespace openarm::canbus {

CANDeviceCollection::CANDeviceCollection(CANSocket& can_socket) : can_socket_(can_socket) {}

CANDeviceCollection::~CANDeviceCollection() {}

void CANDeviceCollection::add_device(const std::shared_ptr<CANDevice>& device) {
    if (!device) return;

    // Add device to our collection
    canid_t device_id = device->get_recv_can_id();
    devices_[device_id] = device;
}

void CANDeviceCollection::remove_device(const std::shared_ptr<CANDevice>& device) {
    if (!device) return;

    canid_t device_id = device->get_recv_can_id();
    auto it = devices_.find(device_id);
    if (it != devices_.end()) {
        // Remove from our collection
        devices_.erase(it);
    }
}

void CANDeviceCollection::dispatch_frame_callback(can_frame& frame) {
    // 首先尝试精确匹配（兼容其他设备）
    auto it = devices_.find(frame.can_id);
    if (it != devices_.end()) {
        it->second->callback(frame);
        return;
    }

    // 对于Robstride设备，尝试基于电机ID匹配（bit15~8）
    uint8_t motor_id = (frame.can_id >> 8) & 0xFF;
    auto motor_it = devices_.find(motor_id);
    if (motor_it != devices_.end()) {
        motor_it->second->callback(frame);
        return;
    }

    // Note: Silently ignore frames for unknown devices (this is normal in CAN
    // networks)
}

void CANDeviceCollection::dispatch_frame_callback(canfd_frame& frame) {
    // 首先尝试精确匹配（兼容其他设备）
    auto it = devices_.find(frame.can_id);
    if (it != devices_.end()) {
        it->second->callback(frame);
        return;
    }

    // 对于Robstride设备，尝试基于电机ID匹配（bit15~8）
    uint8_t motor_id = (frame.can_id >> 8) & 0xFF;
    auto motor_it = devices_.find(motor_id);
    if (motor_it != devices_.end()) {
        motor_it->second->callback(frame);
        return;
    }

    // Note: Silently ignore frames for unknown devices (this is normal in CAN
    // networks)
}

}  // namespace openarm::canbus
