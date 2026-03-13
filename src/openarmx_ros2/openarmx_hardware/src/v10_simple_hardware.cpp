// Copyright 2025 Enactic, Inc.
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

#include "openarmx_hardware/v10_simple_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <openarm/robstride_motor/rs_motor_control.hpp>

namespace openarmx_hardware {

// ===========================================================================
// [PATCH 2025-02-18] 关节安全限制 (直接来自 URDF，不做翻转)
// 目的: 防止命令超出物理行程导致碰撞或堵转
// ===========================================================================
namespace joint_limits {

// 右臂关节限制 (直接来自 URDF)
constexpr double RIGHT_J1_MIN = -1.25;
constexpr double RIGHT_J1_MAX = 3.5;
constexpr double RIGHT_J2_MIN = -0.05;
constexpr double RIGHT_J2_MAX = 3.27;
constexpr double RIGHT_J3_MIN = -1.57;
constexpr double RIGHT_J3_MAX = 1.57;
constexpr double RIGHT_J4_MIN = 0.0;
constexpr double RIGHT_J4_MAX = 2.4;
constexpr double RIGHT_J5_MIN = -1.5;
constexpr double RIGHT_J5_MAX = 1.5;
constexpr double RIGHT_J6_MIN = -0.75;
constexpr double RIGHT_J6_MAX = 0.75;
constexpr double RIGHT_J7_MIN = -1.5;
constexpr double RIGHT_J7_MAX = 1.5;

// 左臂关节限制 (直接使用 URDF 原始值，不翻转)
// 限制是应用在 ROS2/URDF 空间的值上，方向系数在写入电机时才应用
constexpr double LEFT_J1_MIN = -3.34;
constexpr double LEFT_J1_MAX = 1.41;
constexpr double LEFT_J2_MIN = -3.27;
constexpr double LEFT_J2_MAX = 0.05;
constexpr double LEFT_J3_MIN = -1.57;
constexpr double LEFT_J3_MAX = 1.57;
constexpr double LEFT_J4_MIN = 0.0;
constexpr double LEFT_J4_MAX = 2.4;
constexpr double LEFT_J5_MIN = -1.5;
constexpr double LEFT_J5_MAX = 1.5;
constexpr double LEFT_J6_MIN = -0.75;
constexpr double LEFT_J6_MAX = 0.75;
constexpr double LEFT_J7_MIN = -1.5;
constexpr double LEFT_J7_MAX = 1.5;

}  // namespace joint_limits

// 应用关节限制的辅助函数
inline void apply_joint_limits(std::vector<double>& commands, const std::string& arm_prefix) {
  using namespace joint_limits;
  
  if (commands.size() < 7) return;
  
  if (arm_prefix == "right_") {
    commands[0] = std::clamp(commands[0], RIGHT_J1_MIN, RIGHT_J1_MAX);
    commands[1] = std::clamp(commands[1], RIGHT_J2_MIN, RIGHT_J2_MAX);
    commands[2] = std::clamp(commands[2], RIGHT_J3_MIN, RIGHT_J3_MAX);
    commands[3] = std::clamp(commands[3], RIGHT_J4_MIN, RIGHT_J4_MAX);
    commands[4] = std::clamp(commands[4], RIGHT_J5_MIN, RIGHT_J5_MAX);
    commands[5] = std::clamp(commands[5], RIGHT_J6_MIN, RIGHT_J6_MAX);
    commands[6] = std::clamp(commands[6], RIGHT_J7_MIN, RIGHT_J7_MAX);
  } else if (arm_prefix == "left_") {
    commands[0] = std::clamp(commands[0], LEFT_J1_MIN, LEFT_J1_MAX);
    commands[1] = std::clamp(commands[1], LEFT_J2_MIN, LEFT_J2_MAX);
    commands[2] = std::clamp(commands[2], LEFT_J3_MIN, LEFT_J3_MAX);
    commands[3] = std::clamp(commands[3], LEFT_J4_MIN, LEFT_J4_MAX);
    commands[4] = std::clamp(commands[4], LEFT_J5_MIN, LEFT_J5_MAX);
    commands[5] = std::clamp(commands[5], LEFT_J6_MIN, LEFT_J6_MAX);
    commands[6] = std::clamp(commands[6], LEFT_J7_MIN, LEFT_J7_MAX);
  }
}
// [PATCH END]

OpenArm_v10HW::OpenArm_v10HW() = default;

OpenArm_v10HW::~OpenArm_v10HW() {
  // Stop parameter node executor thread
  if (param_spin_thread_active_) {
    param_spin_thread_active_ = false;
    if (param_executor_) {
      param_executor_->cancel();
    }
    if (param_spin_thread_.joinable()) {
      param_spin_thread_.join();
    }
  }
}

bool OpenArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: false per user request)
  it = info.hardware_parameters.find("can_fd");
  std::string raw_can_fd = (it != info.hardware_parameters.end()) ? it->second : std::string("<unset>");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = false;  // Default to false now
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  // Parse control_mode (default: mit)
  it = info.hardware_parameters.find("control_mode");
  if (it == info.hardware_parameters.end()) {
    control_mode_ = ControlMode::MIT;
  } else {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    if (value == "csp") {
      control_mode_ = ControlMode::CSP;
    } else {
      control_mode_ = ControlMode::MIT;  // fallback
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Raw can_fd param: %s", raw_can_fd.c_str());

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s, control_mode=%s",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled",
              (control_mode_ == ControlMode::MIT ? "mit" : "csp"));
  return true;
}

void OpenArm_v10HW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarmx_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarmx_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Initialize ROS2 node for dynamic parameters
  std::string node_name = "openarmx_" + arm_prefix_ + "hardware_params";
  param_node_ = std::make_shared<rclcpp::Node>(node_name);

  // Initialize KP and KD values with defaults
  // 注意：第8个值是夹爪，增大KP/KD可提高响应速度和阻尼
  kp_values_ = {50.0, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0, 50.0};  // 夹爪KP从10.0改为50.0
  kd_values_ = {2.5, 2.5, 2.5, 2.5, 0.5, 0.5, 0.5, 2.5};          // 夹爪KD从0.5改为2.5（与大关节相同）

  // Declare ROS2 parameters for KP and KD
  for (size_t i = 0; i < kp_values_.size(); ++i) {
    std::string kp_param_name = "kp_joint" + std::to_string(i + 1);
    std::string kd_param_name = "kd_joint" + std::to_string(i + 1);

    param_node_->declare_parameter(kp_param_name, kp_values_[i]);
    param_node_->declare_parameter(kd_param_name, kd_values_[i]);

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Declared parameter %s with default value %.2f",
                kp_param_name.c_str(), kp_values_[i]);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Declared parameter %s with default value %.2f",
                kd_param_name.c_str(), kd_values_[i]);
  }

  // Register parameter callback for dynamic reconfiguration
  param_callback_handle_ = param_node_->add_on_set_parameters_callback(
      std::bind(&OpenArm_v10HW::parameters_callback, this, std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Dynamic parameter reconfiguration enabled for KP and KD values");

  // Create executor and start spinning param_node_ in a separate thread
  param_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  param_executor_->add_node(param_node_);
  param_spin_thread_active_ = true;
  param_spin_thread_ = std::thread([this]() {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Parameter node executor thread started for node: %s",
                param_node_->get_name());
    while (param_spin_thread_active_ && rclcpp::ok()) {
      param_executor_->spin_some(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Parameter node executor thread stopped");
  });

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Parameter node %s is now spinning in background thread",
              param_node_->get_name());

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  openarm_ =
      std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);

  // Initialize arm motors with V10 defaul  // ============================================================================
  // [PATCH] 右臂 J1/J2 硬件接线修正 (2024-02 工厂出厂问题)
  // 
  // 问题描述:
  //   右臂的 motor_id=1 和 motor_id=2 物理接线接反了
  //   - CAN ID 1 实际连接的是物理 J2 电机 (肩膀左右)
  //   - CAN ID 2 实际连接的是物理 J1 电机 (肩膀上下)
  //
  // 修复方案:
  //   在初始化时交换右臂的 CAN ID 顺序，使得:
  //   - motors[0] (对应 joint1) → CAN ID 2 (物理 J1)
  //   - motors[1] (对应 joint2) → CAN ID 1 (物理 J2)
  //
  // 影响范围:
  //   - 仅影响右臂 (arm_prefix_ == "right_")
  //   - 左臂保持原有映射不变
  //   - 所有通过 ros2_control 的上层组件 (MoveIt, VR Teleop, RViz) 自动修正
  //   - 直接操作 CAN 的脚本不受影响，需单独处理
  //
  // 如果硬件修复后，删除此 if 块即可恢复默认行为
  // ============================================================================
  std::vector<uint32_t> send_can_ids = DEFAULT_SEND_CAN_IDS;
  std::vector<uint32_t> recv_can_ids = DEFAULT_RECV_CAN_IDS;
  
  if (arm_prefix_ == "right_") {
    // 交换前两个 CAN ID: {0x01, 0x02, ...} → {0x02, 0x01, ...}
    std::swap(send_can_ids[0], send_can_ids[1]);
    std::swap(recv_can_ids[0], recv_can_ids[1]);
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "[PATCH] 右臂 J1/J2 CAN ID 已交换: joint1→CAN_ID_2, joint2→CAN_ID_1 (硬件接线修正)");
  }
  
  openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, send_can_ids, recv_can_ids);

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Initializing gripper...");
    openarm_->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
                                 DEFAULT_GRIPPER_SEND_CAN_ID,
                                 DEFAULT_GRIPPER_RECV_CAN_ID);
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArm_v10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Activating OpenArm V10...");
  openarm_->set_callback_mode_all(openarm::robstride_motor::CallbackMode::STATE);

  // Enable a short debug window to verify raw motor readings vs. published joint states
  debug_cycles_remaining_ = 50;  // ~first 50 read() calls

  if (control_mode_ == ControlMode::MIT) {
    openarm_->enable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    openarm_->refresh_all();
    openarm_->recv_all();

    ////////////////////////////////////  初始化命令为当前位置，避免自动回零 ////////////////////////////////
    auto arm_motors = openarm_->get_arm().get_motors();
    const auto direction_multipliers = get_motor_direction_multipliers();
    for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
      pos_commands_[i] = arm_motors[i]->get_position() * direction_multipliers[i];
    }
    if (hand_) {
      auto gripper_motors = openarm_->get_gripper().get_motors();
      if (!gripper_motors.empty()) {
        pos_commands_[ARM_DOF] = motor_radians_to_joint(gripper_motors[0]->get_position());
      }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////

    // Return to zero position
    // return_to_zero();
  } else {
    // CSP: follow documented flow
    using namespace openarm::robstride_motor;
    auto& arm = openarm_->get_arm();
    auto& master = openarm_->get_master_can_device_collection();
    auto& sock = master.get_can_socket();

    // 1) Disable all
    arm.disable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 2) Switch to CSP for each motor, set limits
    auto devices = arm.get_all_devices();
    auto motors = arm.get_all_motors();
    for (size_t i = 0; i < motors.size(); ++i) {
      Motor* m = motors[i];
      RSCANDevice* d = devices[i];
      // Switch mode
      auto mode_pkt = csp_set_mode_packet(*m);
      auto mode_frame = d->create_can_frame(mode_pkt.send_can_id, mode_pkt.data);
      sock.write_can_frame(mode_frame);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));

      // Limits from motor type
      auto mt = m->get_motor_type();
      size_t idx = static_cast<size_t>(mt);
      const auto& lim = openarm::robstride_motor::MOTOR_LIMIT_PARAMS[idx];

      auto spd_pkt = csp_set_speed_limit_packet(*m, static_cast<float>(0.5));
      auto spd_frame = d->create_can_frame(spd_pkt.send_can_id, spd_pkt.data);
      sock.write_can_frame(spd_frame);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));

      auto cur_pkt = csp_set_current_limit_packet(*m, static_cast<float>(lim.tMax));
      auto cur_frame = d->create_can_frame(cur_pkt.send_can_id, cur_pkt.data);
      sock.write_can_frame(cur_frame);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // Gripper: optional
    if (hand_) {
      auto g_devices = openarm_->get_gripper().get_all_devices();
      auto g_motors = openarm_->get_gripper().get_all_motors();
      if (!g_motors.empty()) {
        Motor* m = g_motors[0];
        RSCANDevice* d = g_devices[0];
        auto mode_pkt = csp_set_mode_packet(*m);
        sock.write_can_frame(d->create_can_frame(mode_pkt.send_can_id, mode_pkt.data));
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        auto mt = m->get_motor_type();
        size_t idx = static_cast<size_t>(mt);
        const auto& lim = openarm::robstride_motor::MOTOR_LIMIT_PARAMS[idx];
        sock.write_can_frame(d->create_can_frame(
            csp_set_speed_limit_packet(*m, static_cast<float>(0.5)).send_can_id,
            csp_set_speed_limit_packet(*m, static_cast<float>(0.5)).data));
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        sock.write_can_frame(d->create_can_frame(
            csp_set_current_limit_packet(*m, static_cast<float>(lim.tMax)).send_can_id,
            csp_set_current_limit_packet(*m, static_cast<float>(lim.tMax)).data));
      }
    }

    // 3) Enable all
    arm.enable_all();
    if (hand_) {
      openarm_->get_gripper().enable_all();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    openarm_->recv_all();

    // 4) Go to zero
    // return_to_zero();
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Deactivating OpenArm V10...");

  // Disable all motors (like full_arm.cpp exit)
  openarm_->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Receive all motor states
  openarm_->refresh_all();
  openarm_->recv_all();

  // Read arm joint states - FIXED: Now using vector order with direction correction
  // joint1 -> motor[0] (CAN ID 1), joint7 -> motor[6] (CAN ID 7)
  auto arm_motors = openarm_->get_arm().get_motors();
  const auto direction_multipliers = get_motor_direction_multipliers();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    const double raw_pos = arm_motors[i]->get_position();
    const double raw_vel = arm_motors[i]->get_velocity();
    const double raw_tau = arm_motors[i]->get_torque();
    const double sign = direction_multipliers[i];
    pos_states_[i] = raw_pos * sign;
    vel_states_[i] = raw_vel * sign;
    tau_states_[i] = raw_tau * sign;

    // Transient debug logging to diagnose unexpected offsets/signs
    if (debug_cycles_remaining_ > 0 && i == 0) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                  "[%s] joint%zu raw=%.6f rad, sign=%.1f, published=%.6f (can_id=%u)",
                  arm_prefix_.c_str(), i + 1, raw_pos, sign, pos_states_[i],
                  static_cast<unsigned>(arm_motors[i]->get_send_can_id()));
    }
  }

  if (debug_cycles_remaining_ > 0) {
    --debug_cycles_remaining_;
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    auto gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value (0-0.044m)
      double motor_pos = gripper_motors[0]->get_position();
      pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);

      // Unimplemented: Velocity and torque mapping
      vel_states_[ARM_DOF] = 0;  // gripper_motors[0]->get_velocity();
      tau_states_[ARM_DOF] = 0;  // gripper_motors[0]->get_torque();
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // ===========================================================================
  // [PATCH 2025-02-18] 应用关节安全限制
  // 在发送命令之前，先 clamp 到安全范围
  // ===========================================================================
  apply_joint_limits(pos_commands_, arm_prefix_);
  // [PATCH END]
  
  const auto direction_multipliers = get_motor_direction_multipliers();
  if (control_mode_ == ControlMode::MIT) {
    // Motion control path (MIT)
    std::vector<openarm::robstride_motor::MotionControlParam> arm_params(ARM_DOF);

    // Lock to safely read KP/KD values
    std::lock_guard<std::mutex> lock(kp_kd_mutex_);

    for (size_t i = 0; i < ARM_DOF; ++i) {
      openarm::robstride_motor::MotionControlParam param;
      param.kp = kp_values_[i];
      param.kd = kd_values_[i];
      param.position = pos_commands_[i] * direction_multipliers[i];
      param.velocity = vel_commands_[i] * direction_multipliers[i];
      param.torque = tau_commands_[i] * direction_multipliers[i];
      arm_params[i] = param;
    }
    openarm_->get_arm().send_motion_control_commands(arm_params);

    if (hand_ && joint_names_.size() > ARM_DOF) {
      // ===========================================================================
      // [PATCH 2025-02-18] Gripper 软限位保护 (MIT 模式)
      // 防止命令超出物理行程导致堵转故障
      // 范围: 0.0 m (闭合) ~ 0.05 m (完全张开)
      // ===========================================================================
      // [原代码 - 注释掉]
      // double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
      // [新代码 - 添加 clamp 保护]
      constexpr double GRIPPER_MIN_JOINT = 0.0;   // 闭合位置 (米)
      constexpr double GRIPPER_MAX_JOINT = 0.05;  // 最大张开位置 (米)
      double gripper_cmd_clamped = std::clamp(pos_commands_[ARM_DOF], GRIPPER_MIN_JOINT, GRIPPER_MAX_JOINT);
      double motor_command = joint_to_motor_radians(gripper_cmd_clamped);
      // [PATCH END]
      
      openarm::robstride_motor::MotionControlParam gripper_param;
      // Use the 8th KP/KD value for gripper (index 7)
      gripper_param.kp = kp_values_[ARM_DOF];
      gripper_param.kd = kd_values_[ARM_DOF];
      gripper_param.position = motor_command;
      gripper_param.velocity = 0.0;
      gripper_param.torque = 0.0;
      openarm_->get_gripper().send_motion_control_commands({gripper_param});
    }
  } else {
    // CSP path: write LOC_REF for each motor
    using namespace openarm::robstride_motor;
    auto& arm = openarm_->get_arm();
    auto motors = arm.get_all_motors();
    auto devices = arm.get_all_devices();
    auto& sock = openarm_->get_master_can_device_collection().get_can_socket();
    for (size_t i = 0; i < ARM_DOF && i < motors.size(); ++i) {
      float target = static_cast<float>(pos_commands_[i] * direction_multipliers[i]);
      auto pkt = csp_set_target_position_packet(*motors[i], target);
      auto frame = devices[i]->create_can_frame(pkt.send_can_id, pkt.data);
      sock.write_can_frame(frame);
    }

    if (hand_ && joint_names_.size() > ARM_DOF) {
      auto g_motors = openarm_->get_gripper().get_all_motors();
      auto g_devices = openarm_->get_gripper().get_all_devices();
      if (!g_motors.empty()) {
        // ===========================================================================
        // [PATCH 2025-02-18] Gripper 软限位保护 (CSP 模式)
        // 防止命令超出物理行程导致堵转故障
        // 范围: 0.0 m (闭合) ~ 0.05 m (完全张开)
        // ===========================================================================
        // [原代码 - 注释掉]
        // float motor_command = static_cast<float>(joint_to_motor_radians(pos_commands_[ARM_DOF]));
        // [新代码 - 添加 clamp 保护]
        constexpr double GRIPPER_MIN_JOINT = 0.0;
        constexpr double GRIPPER_MAX_JOINT = 0.05;
        double gripper_cmd_clamped = std::clamp(pos_commands_[ARM_DOF], GRIPPER_MIN_JOINT, GRIPPER_MAX_JOINT);
        float motor_command = static_cast<float>(joint_to_motor_radians(gripper_cmd_clamped));
        // [PATCH END]
        
        auto pkt = csp_set_target_position_packet(*g_motors[0], motor_command);
        auto frame = g_devices[0]->create_can_frame(pkt.send_can_id, pkt.data);
        sock.write_can_frame(frame);
      }
    }
  }

  openarm_->recv_all(1000);
  return hardware_interface::return_type::OK;
}

void OpenArm_v10HW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Returning to zero position...");

  // Lock to safely read KP/KD values
  std::lock_guard<std::mutex> lock(kp_kd_mutex_);

  // Return arm to zero with motion control - use hardware order for zero position
  std::vector<openarm::robstride_motor::MotionControlParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    openarm::robstride_motor::MotionControlParam param;
    param.kp = kp_values_[i];
    param.kd = kd_values_[i];
    param.position = 0.0;
    param.velocity = 0.0;
    param.torque = 0.0;
    arm_params.push_back(param);
  }
  openarm_->get_arm().send_motion_control_commands(arm_params);

  // Return gripper to zero if enabled
  if (hand_) {
    openarm::robstride_motor::MotionControlParam gripper_param;
    gripper_param.kp = kp_values_[ARM_DOF];
    gripper_param.kd = kd_values_[ARM_DOF];
    gripper_param.position = GRIPPER_JOINT_0_POSITION;
    gripper_param.velocity = 0.0;
    gripper_param.torque = 0.0;
    openarm_->get_gripper().send_motion_control_commands({gripper_param});
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all();
}

// Gripper mapping helper functions
double OpenArm_v10HW::joint_to_motor_radians(double joint_value) {
  // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
  return (joint_value / GRIPPER_JOINT_0_POSITION) *
         GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
}

double OpenArm_v10HW::motor_radians_to_joint(double motor_radians) {
  // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
  return GRIPPER_JOINT_0_POSITION *
         (motor_radians /
          GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
}

// ===========================================================================
// [PATCH 2025-02-18] 电机方向系数
// 根据实机与 RViz 对比调整：左右臂都是全 -1.0
// ===========================================================================
std::vector<double> OpenArm_v10HW::get_motor_direction_multipliers() const {
  // 左右臂都是 J1-J7 全部 = -1.0
  return std::vector<double>{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
}
// [PATCH END]

// Dynamic parameter callback implementation
rcl_interfaces::msg::SetParametersResult OpenArm_v10HW::parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  std::lock_guard<std::mutex> lock(kp_kd_mutex_);

  for (const auto& param : parameters) {
    std::string param_name = param.get_name();

    // Check if it's a KP parameter
    if (param_name.find("kp_joint") == 0) {
      // Extract joint index from parameter name (kp_joint1 -> index 0)
      size_t joint_idx = std::stoi(param_name.substr(8)) - 1;

      if (joint_idx < kp_values_.size()) {
        double new_value = param.as_double();
        double old_value = kp_values_[joint_idx];
        kp_values_[joint_idx] = new_value;

        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                    "Updated %s: %.2f -> %.2f",
                    param_name.c_str(), old_value, new_value);
      } else {
        result.successful = false;
        result.reason = "Joint index out of range for " + param_name;
        RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                     "Failed to update %s: joint index %zu out of range",
                     param_name.c_str(), joint_idx);
      }
    }
    // Check if it's a KD parameter
    else if (param_name.find("kd_joint") == 0) {
      // Extract joint index from parameter name (kd_joint1 -> index 0)
      size_t joint_idx = std::stoi(param_name.substr(8)) - 1;

      if (joint_idx < kd_values_.size()) {
        double new_value = param.as_double();
        double old_value = kd_values_[joint_idx];
        kd_values_[joint_idx] = new_value;

        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                    "Updated %s: %.2f -> %.2f",
                    param_name.c_str(), old_value, new_value);
      } else {
        result.successful = false;
        result.reason = "Joint index out of range for " + param_name;
        RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                     "Failed to update %s: joint index %zu out of range",
                     param_name.c_str(), joint_idx);
      }
    }
  }

  // Print current KP/KD values for all joints after any parameter change
  // if (result.successful && !parameters.empty()) {
  //   RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
  //               "========== Current KP/KD Values ==========");

  //   // Print arm joints (1-7)
  //   RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
  //               "Arm Joints:");
  //   for (size_t i = 0; i < ARM_DOF && i < kp_values_.size(); ++i) {
  //     RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
  //                 "  Joint %zu: KP=%.2f, KD=%.2f",
  //                 i + 1, kp_values_[i], kd_values_[i]);
  //   }

  //   // Print gripper (joint 8) if available
  //   if (kp_values_.size() > ARM_DOF) {
  //     RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
  //                 "Gripper:");
  //     RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
  //                 "  Joint 8 (Gripper): KP=%.2f, KD=%.2f",
  //                 kp_values_[ARM_DOF], kd_values_[ARM_DOF]);
  //   }

  //   RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
  //               "==========================================");
  // }

  return result;
}

}  // namespace openarmx_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarmx_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
