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

#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iomanip>
#include <iostream>
#include <openarm/canbus/can_socket.hpp>

namespace openarm::canbus {

CANSocket::CANSocket(const std::string& interface, bool enable_fd, bool verbose)
    : socket_fd_(-1), interface_(interface), fd_enabled_(enable_fd), verbose_(verbose) {
    if (!initialize_socket(interface)) {
        throw CANSocketException("Failed to initialize socket for interface: " + interface);
    }
}

CANSocket::~CANSocket() { cleanup(); }

bool CANSocket::initialize_socket(const std::string& interface) {
    // Create socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        return false;
    }

    struct ifreq ifr;
    struct sockaddr_can addr;

    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        cleanup();
        return false;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (fd_enabled_) {
        int enable_canfd = 1;
        if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
                       sizeof(enable_canfd)) < 0) {
            cleanup();
            return false;
        }
    }

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        cleanup();
        return false;
    }

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        cleanup();
        return false;
    }

    return true;
}

void CANSocket::cleanup() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

ssize_t CANSocket::read_raw_frame(void* buffer, size_t buffer_size) {
    if (!is_initialized()) return -1;
    return read(socket_fd_, buffer, buffer_size);
}

ssize_t CANSocket::write_raw_frame(const void* buffer, size_t frame_size) {
    if (!is_initialized()) return -1;
    return write(socket_fd_, buffer, frame_size);
}

bool CANSocket::write_can_frame(const can_frame& frame) {
    bool success = write(socket_fd_, &frame, sizeof(frame)) == sizeof(frame);

    if (verbose_ && success) {
        // Print in Python style: [TX] ID: 0x0300FD02, Data: ['00', '00', '00', '00', '00', '00', '00', '00']
        // Remove CAN_EFF_FLAG (0x80000000) to show only the actual 29-bit ID
        uint32_t actual_id = frame.can_id & CAN_EFF_MASK;  // 0x1FFFFFFF
        std::cout << "[TX] ID: 0x" << std::hex << std::setfill('0') << std::setw(8)
                  << std::uppercase << actual_id;
        std::cout << ", Data: [";
        for (int i = 0; i < frame.can_dlc; ++i) {
            std::cout << "'" << std::hex << std::setfill('0') << std::setw(2)
                      << std::uppercase << static_cast<int>(frame.data[i]) << "'";
            if (i < frame.can_dlc - 1) std::cout << ", ";
        }
        std::cout << "]" << std::dec << std::endl;
    }

    return success;
}

bool CANSocket::write_canfd_frame(const canfd_frame& frame) {
    return write(socket_fd_, &frame, sizeof(frame)) == sizeof(frame);
}

bool CANSocket::read_can_frame(can_frame& frame) {
    if (!is_initialized()) return false;
    ssize_t bytes_read = read(socket_fd_, &frame, sizeof(frame));
    bool success = bytes_read == sizeof(frame);

    if (verbose_ && success) {
        // Print in Python style: [RX] ID: 0x028002FD, Data: ['80', '81', '7F', 'EE', '7F', 'FF', '01', '18']
        // Remove CAN_EFF_FLAG (0x80000000) to show only the actual 29-bit ID
        uint32_t actual_id = frame.can_id & CAN_EFF_MASK;  // 0x1FFFFFFF
        std::cout << "[RX] ID: 0x" << std::hex << std::setfill('0') << std::setw(8)
                  << std::uppercase << actual_id;
        std::cout << ", Data: [";
        for (int i = 0; i < frame.can_dlc; ++i) {
            std::cout << "'" << std::hex << std::setfill('0') << std::setw(2)
                      << std::uppercase << static_cast<int>(frame.data[i]) << "'";
            if (i < frame.can_dlc - 1) std::cout << ", ";
        }
        std::cout << "]" << std::dec << std::endl;
    }

    return success;
}

bool CANSocket::read_canfd_frame(canfd_frame& frame) {
    if (!is_initialized()) return false;
    ssize_t bytes_read = read(socket_fd_, &frame, sizeof(frame));
    return bytes_read == sizeof(frame);
}

bool CANSocket::is_data_available(int timeout_us) {
    if (!is_initialized()) return false;

    fd_set read_fds;
    struct timeval timeout;

    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);

    timeout.tv_sec = timeout_us / 1000000;
    timeout.tv_usec = (timeout_us % 1000000);

    int result = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);

    return (result > 0 && FD_ISSET(socket_fd_, &read_fds));
}

}  // namespace openarm::canbus
