// 直接模拟Python版本的Robstride电机测试
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include "openarm/canbus/can_socket.hpp"
#include "openarm/robstride_motor/rs_motor.hpp"
#include "openarm/robstride_motor/rs_motor_control.hpp"

using namespace openarm::robstride_motor;
using namespace openarm::canbus;

bool send_can_packet_and_wait_response(CANSocket& can_socket, const CANPacket& packet, can_frame* response = nullptr) {
    // 发送命令
    can_frame frame;
    frame.can_id = packet.send_can_id | CAN_EFF_FLAG;  // 扩展帧
    frame.can_dlc = std::min(static_cast<size_t>(8), packet.data.size());

    std::fill(frame.data, frame.data + 8, 0);
    std::copy(packet.data.begin(), packet.data.begin() + frame.can_dlc, frame.data);

    if (!can_socket.write_can_frame(frame)) {
        return false;
    }

    // 等待响应（模拟Python的阻塞行为）
    if (response != nullptr) {
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(1000)) {
            if (can_socket.is_data_available(100000)) {  // 100ms超时
                if (can_socket.read_can_frame(*response)) {
                    // std::cout << "接收到CAN帧 ID: 0x" << std::hex << response->can_id << std::dec << std::endl;
                    return true;
                }
            }
        }
        std::cout << "警告: 没有收到反馈数据" << std::endl;
    }

    return true;
}

// 解析电机反馈数据（根据Python版本）
struct MotorStatus {
    double angle = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    double temperature = 0.0;
};

MotorStatus parse_motor_feedback(const can_frame& frame) {
    MotorStatus status;

    if (frame.can_dlc >= 8) {
        // 根据Python代码的解析逻辑
        uint16_t angle_raw = (frame.data[0] << 8) | frame.data[1];
        uint16_t velocity_raw = (frame.data[2] << 8) | frame.data[3];
        uint16_t torque_raw = (frame.data[4] << 8) | frame.data[5];
        uint16_t temp_raw = (frame.data[6] << 8) | frame.data[7];

        // Python中的范围常量
        const double T_MIN = -12.0, T_MAX = 12.0;  // 角度范围
        const double V_MIN = -30.0, V_MAX = 30.0;  // 速度范围
        const double P_MIN = -12.5, P_MAX = 12.5;  // 扭矩范围

        // Python的uint16_to_float函数
        status.angle = double(angle_raw) / 65535.0 * (T_MAX - T_MIN) + T_MIN;
        status.velocity = double(velocity_raw) / 65535.0 * (V_MAX - V_MIN) + V_MIN;
        status.torque = double(torque_raw) / 65535.0 * (P_MAX - P_MIN) + P_MIN;
        status.temperature = temp_raw / 10.0;
    }

    return status;
}

int main(int argc, char** argv) {
    int motor_id = 3;
    std::string can_interface = "can1";

    if (argc > 1) {
        motor_id = std::atoi(argv[1]);
    }

    try {
        std::cout << "=== Python风格Robstride电机测试 ===" << std::endl;
        std::cout << "CAN接口: " << can_interface << std::endl;
        std::cout << "电机ID: " << motor_id << std::endl;

        // 初始化CAN socket
        CANSocket can_socket(can_interface, false);
        if (!can_socket.is_initialized()) {
            std::cerr << "CAN socket初始化失败" << std::endl;
            return 1;
        }

        Motor motor(MotorType::RS00, motor_id, motor_id);

        // 1. 设置零角度
        std::cout << "\n1. 设置电机零角度..." << std::endl;
        auto zero_cmd = CanPacketEncoder::create_set_zero_command(motor);
        if (send_can_packet_and_wait_response(can_socket, zero_cmd)) {
            std::cout << "设置电机ID号:" << motor_id << " 设置负载端零角度OK" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 2. 设置运控模式
        std::cout << "\n2. 设置运控模式..." << std::endl;
        auto mode_cmd = CanPacketEncoder::create_set_control_mode_command(motor, ControlMode::MOTION_CONTROL);
        if (send_can_packet_and_wait_response(can_socket, mode_cmd)) {
            std::cout << "设置电机ID号:" << motor_id << " 控制模式(运控模式)OK" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 3. 使能电机
        std::cout << "\n3. 使能电机..." << std::endl;
        auto enable_cmd = CanPacketEncoder::create_enable_command(motor);
        if (send_can_packet_and_wait_response(can_socket, enable_cmd)) {
            std::cout << "设置电机ID号:" << motor_id << " 电机模式使能OK" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 4. 运动控制循环（模拟Python逻辑）
        std::cout << "\n4. 开始运动控制..." << std::endl;

        double red = 0.0;
        int fx = 0;
        double red_max = 0.2;

        for (int step = 0; step < 50; ++step) {  // 限制步数以便观察
            // 更新目标位置（模拟Python逻辑）
            if (fx == 0 && red < red_max) {
                red += 0.01;
            } else if (fx == 0 && red >= red_max) {
                fx = 1;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            } else if (fx == 1 && red > 0) {
                red -= 0.01;
            } else if (fx == 1 && red <= 0) {
                fx = 0;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }

            // 发送运控命令并等待反馈（模拟Python的leg_set_motion_parameter_L_with_feedback）
            MotionControlParam param;
            param.kp = 10.0;
            param.kd = 0.5;
            param.position = -red;  // Python中使用-red
            param.velocity = 0.0;
            param.torque = 0.0;

            auto motion_cmd = CanPacketEncoder::create_motion_control_command(motor, param);

            can_frame response;
            if (send_can_packet_and_wait_response(can_socket, motion_cmd, &response)) {
                MotorStatus status = parse_motor_feedback(response);
                std::cout << "电机ID:" << motor_id
                          << " | 目标角度:" << std::fixed << std::setprecision(3) << (-red)
                          << " | 实际角度:" << std::setprecision(3) << status.angle
                          << " | 速度:" << std::setprecision(3) << status.velocity
                          << " | 力矩:" << std::setprecision(3) << status.torque
                          << " | 温度:" << std::setprecision(1) << status.temperature << "°C"
                          << std::endl;
            } else {
                std::cout << "电机ID:" << motor_id
                          << " | 目标角度:" << std::fixed << std::setprecision(3) << (-red)
                          << " | 通信失败" << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));  // Python中time_delay = 0.002
        }

        // 5. 禁用电机
        // std::cout << "\n5. 禁用电机..." << std::endl;
        auto disable_cmd = CanPacketEncoder::create_disable_command(motor);
        send_can_packet_and_wait_response(can_socket, disable_cmd);

        std::cout << "\n=== 测试完成 ===" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}