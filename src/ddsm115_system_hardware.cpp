// Copyright 2026 Tatsukiyano
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
#include "ddsm115_ros2_driver/ddsm115_system_hardware.hpp"

#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ddsm115_ros2_driver
{

DDSM115SystemHardware::~DDSM115SystemHardware()
{
  on_cleanup(rclcpp_lifecycle::State());
}

#ifdef ROS_DISTRO_HUMBLE
hardware_interface::CallbackReturn DDSM115SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
#else
hardware_interface::CallbackReturn DDSM115SystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
#endif

  node_ = std::make_shared<rclcpp::Node>("ddsm115_hardware_interface_node");
  RCLCPP_INFO(node_->get_logger(), "Initializing DDSM115 System Hardware...");

  // Instantiate the client with callback binding
  driver_client_ = std::make_unique<DDSM115DriverClient>(
    std::bind(&DDSM115SystemHardware::motor_feedback_callback, this, std::placeholders::_1),
    [this](LogLevel level, const std::string & msg) {
      switch (level) {
        case LogLevel::DEBUG:
          RCLCPP_DEBUG(node_->get_logger(), "%s", msg.c_str());
          break;
        case LogLevel::INFO:
          RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
          break;
        case LogLevel::WARN:
          RCLCPP_WARN(node_->get_logger(), "%s", msg.c_str());
          break;
        case LogLevel::ERROR:
          RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
          break;
      }
    });

  // Initialize motors list from URDF joint configuration
  motors_.resize(info_.joints.size());
  driver_client_->clear_registered_motor_ids();

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];

    // Check parameters
    if (joint.parameters.count("motor_id") == 0) {
      RCLCPP_FATAL(node_->get_logger(), "Joint '%s' misses 'motor_id' parameter in URDF!",
          joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    uint8_t id = static_cast<uint8_t>(std::stoi(joint.parameters.at("motor_id"), nullptr, 0));
    bool invert = false;
    if (joint.parameters.count("invert_direction") > 0) {
      invert = (joint.parameters.at("invert_direction") == "true");
    }

    motors_[i].id = id;
    motors_[i].invert = invert;
    id_to_index_[id] = i;

    // Register this ID in the client to filter out any background serial noise
    driver_client_->register_motor_id(id);

    RCLCPP_INFO(node_->get_logger(), "Configured Joint '%s' with Motor ID: %d, Invert: %s",
                joint.name.c_str(), id, invert ? "true" : "false");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDSM115SystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Configuring serial port connection...");

  serial_port_ = "/dev/ttyUSB0";
  if (info_.hardware_parameters.count("usb_path") > 0) {
    serial_port_ = info_.hardware_parameters.at("usb_path");
  } else if (info_.hardware_parameters.count("serial_port") > 0) {
    serial_port_ = info_.hardware_parameters.at("serial_port");
  }

  baud_rate_ = 115200;
  if (info_.hardware_parameters.count("serial_baud") > 0) {
    baud_rate_ = std::stoi(info_.hardware_parameters.at("serial_baud"));
  }

  if (!driver_client_->init_port(serial_port_, baud_rate_)) {
    RCLCPP_FATAL(node_->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDSM115SystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up hardware interface...");
  if (driver_client_) {
    driver_client_->close_port();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDSM115SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Activating motors...");

  // Send switch mode command to each motor
  for (const auto & motor : motors_) {
    driver_client_->send_mode_command(motor.id, ControlLoopModes::MODE_VELOCITY);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Start background serial communication polling loop
  communication_active_ = true;
  communication_thread_ = std::thread(&DDSM115SystemHardware::communication_loop, this);

  RCLCPP_INFO(node_->get_logger(), "Motors activated and communication thread started.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDSM115SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating motors...");

  // Stop the polling thread
  communication_active_ = false;
  if (communication_thread_.joinable()) {
    communication_thread_.join();
  }

  // Stop all motors safely
  for (const auto & motor : motors_) {
    driver_client_->send_velocity_command(motor.id, 0.0, 0, true);  // Send brake command
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_INFO(node_->get_logger(), "Motors deactivated and stopped.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DDSM115SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motors_[i].position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motors_[i].effort));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DDSM115SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].command_velocity));
  }
  return command_interfaces;
}

hardware_interface::return_type DDSM115SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Non-blocking read is handled asynchronously by feedback callback and background thread.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DDSM115SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Command velocities are copied from command interfaces during the write call.
  // The background communication thread polls this structure and transmits it.
  return hardware_interface::return_type::OK;
}

void DDSM115SystemHardware::motor_feedback_callback(const MotorFeedback & feedback)
{
  if (id_to_index_.find(feedback.motor_id) == id_to_index_.end()) {
    return;
  }

  if (feedback.is_temperature_packet) {
    // Ignore temperature query packets for odometry/control tracking
    return;
  }

  size_t idx = id_to_index_[feedback.motor_id];
  std::lock_guard<std::mutex> lock(state_mutex_);

  auto & motor = motors_[idx];

  // Track single-turn position wrap-arounds
  if (motor.first_feedback) {
    motor.prev_raw_position_deg = feedback.position;
    motor.first_feedback = false;
  }

  double diff = feedback.position - motor.prev_raw_position_deg;
  if (diff < -180.0) {
    motor.wrap_count++;
  } else if (diff > 180.0) {
    motor.wrap_count--;
  }
  motor.prev_raw_position_deg = feedback.position;

  // Convert unit to radians and accumulate
  double accumulated_deg = feedback.position + static_cast<double>(motor.wrap_count) * 360.0;
  double position_rad = accumulated_deg * (M_PI / 180.0);

  // Convert velocity from RPM to rad/s
  double velocity_rad_s = feedback.velocity * (2.0 * M_PI / 60.0);

  // Handle invert direction flag
  if (motor.invert) {
    motor.position = -position_rad;
    motor.velocity = -velocity_rad_s;
    motor.effort = -feedback.current;
  } else {
    motor.position = position_rad;
    motor.velocity = velocity_rad_s;
    motor.effort = feedback.current;
  }
}

void DDSM115SystemHardware::communication_loop()
{
  RCLCPP_INFO(node_->get_logger(), "Communication thread started.");

  while (communication_active_) {
    for (size_t i = 0; i < motors_.size(); ++i) {
      if (!communication_active_) {
        break;
      }

      uint8_t id = 0;
      double cmd_vel_rad_s = 0.0;
      bool invert = false;

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        id = motors_[i].id;
        cmd_vel_rad_s = motors_[i].command_velocity;
        invert = motors_[i].invert;
      }

      // Convert command velocity from rad/s to RPM
      double target_rpm = cmd_vel_rad_s * (60.0 / (2.0 * M_PI));
      if (invert) {
        target_rpm = -target_rpm;
      }

      // Send command over RS485 bus
      driver_client_->send_velocity_command(id, target_rpm, 0, false);

      // Sleep to allow response to clear the serial line and prevent bus collision
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Communication thread stopped.");
}

}  // namespace ddsm115_ros2_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ddsm115_ros2_driver::DDSM115SystemHardware, hardware_interface::SystemInterface)
