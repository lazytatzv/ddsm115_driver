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
#include "ddsm115_ros2_driver/ddsm115_driver_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

namespace ddsm115_ros2_driver
{

DDSM115DriverNode::DDSM115DriverNode(const rclcpp::NodeOptions & options)
: Node("ddsm115_driver_node", options), diagnostic_updater_(this)
{
  // Declare parameters
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("serial_baud", 115200);
  this->declare_parameter<double>("publish_rate", 20.0);
  this->declare_parameter<double>("command_timeout", 1.0);
  this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{1});

  // Get parameters
  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("serial_baud").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  command_timeout_ = this->get_parameter("command_timeout").as_double();
  motor_ids_ = this->get_parameter("motor_ids").as_integer_array();

  // Initialize driver client
  driver_client_ = std::make_unique<DDSM115DriverClient>(
      std::bind(&DDSM115DriverNode::motor_feedback_callback, this, std::placeholders::_1),
    [this](LogLevel level, const std::string & msg)
    {
      switch (level) {
        case LogLevel::DEBUG:
          RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
          break;
        case LogLevel::INFO:
          RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
          break;
        case LogLevel::WARN:
          RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
          break;
        case LogLevel::ERROR:
          RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
          break;
      }
      });

  // Register configured motor IDs to filter out serial noise
  driver_client_->clear_registered_motor_ids();
  for (int64_t motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);
    driver_client_->register_motor_id(id);
  }

  // Initialize serial port
  if (!driver_client_->init_port(serial_port_, baud_rate_)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize serial port: %s", serial_port_.c_str());
    throw std::runtime_error("Failed to initialize serial port");
  }

  // Create publishers, subscribers and state mappings for each motor
  for (int64_t motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);
    std::string ns = "motor_" + std::to_string(id);

    // Create publishers
    status_pubs_[id] = this->create_publisher<ddsm115_ros2_driver::msg::Ddsm115Status>(
        ns + "/status", 10);

    // Create subscribers
    command_subs_[id] = this->create_subscription<ddsm115_ros2_driver::msg::Ddsm115Command>(
        ns + "/command", 10,
      [this, id](const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg)
      {
        topic_callback(msg, id);
        });

    // Initialize state
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      motor_states_[id].last_cmd.mode = ddsm115_ros2_driver::msg::Ddsm115Command::MODE_VELOCITY;
      motor_states_[id].last_cmd.value = 0.0;
      motor_states_[id].last_cmd.brake_mode =
        ddsm115_ros2_driver::msg::Ddsm115Command::BRAKE_RELEASE;
      motor_states_[id].last_cmd_time = this->now();
      motor_states_[id].timed_out = true;  // Start in timed_out state until first command
      motor_states_[id].has_status = false;
    }

    // Set motor default mode to velocity loop
    driver_client_->send_mode_command(id, ControlLoopModes::MODE_VELOCITY);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Setup diagnostic updater
  setup_diagnostics();

  // Create control timer (space commands equally)
  double timer_period_sec = 1.0 / publish_rate_;
  control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(timer_period_sec),
      std::bind(&DDSM115DriverNode::control_timer_callback, this));

  // Create watchdog timer for timeouts (10Hz)
  timeout_timer_ = this->create_wall_timer(
      100ms,
      std::bind(&DDSM115DriverNode::check_timeouts_callback, this));

  RCLCPP_INFO(this->get_logger(), "DDSM115 Standalone Node successfully initialized.");
}

DDSM115DriverNode::~DDSM115DriverNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down DDSM115 Standalone Node...");

  if (control_timer_) {control_timer_->cancel();}
  if (timeout_timer_) {timeout_timer_->cancel();}

  // Stop all motors on shutdown
  for (int64_t motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);
    driver_client_->send_velocity_command(id, 0.0, 0, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (driver_client_) {
    driver_client_->close_port();
  }
}

void DDSM115DriverNode::motor_feedback_callback(const MotorFeedback & feedback)
{
  total_rx_packets_++;

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (motor_states_.count(feedback.motor_id) == 0) {return;}

  auto & state = motor_states_[feedback.motor_id];

  ddsm115_ros2_driver::msg::Ddsm115Status status_msg;
  status_msg.current = feedback.current;
  status_msg.velocity = feedback.velocity;
  status_msg.error_code = feedback.error_code;

  if (feedback.is_temperature_packet) {
    // Use last stored high-res position if we have one
    status_msg.position = state.has_status ? state.last_status.position : feedback.position;
    status_msg.winding_temperature = feedback.winding_temperature;
  } else {
    // Use high-res position from 0x64 packet
    status_msg.position = feedback.position;
    // Keep last known winding temperature
    status_msg.winding_temperature = state.has_status ? state.last_status.winding_temperature : 0.0;
  }

  // Publish status
  if (status_pubs_.count(feedback.motor_id) > 0) {
    status_pubs_[feedback.motor_id]->publish(status_msg);
  }

  state.last_status = status_msg;
  state.has_status = true;
}

void DDSM115DriverNode::topic_callback(
  const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg, uint8_t motor_id)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (motor_states_.count(motor_id) > 0) {
    motor_states_[motor_id].last_cmd = *msg;
    motor_states_[motor_id].last_cmd_time = this->now();
    motor_states_[motor_id].timed_out = false;
  }
}

void DDSM115DriverNode::control_timer_callback()
{
  control_cycle_count_++;
  bool request_temp = (control_cycle_count_ % 20 == 0);

  // Send commands to motors sequentially to prevent RS485 collision
  for (int64_t motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);

    ddsm115_ros2_driver::msg::Ddsm115Command cmd;
    bool is_timed_out = true;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (motor_states_.count(id) > 0) {
        cmd = motor_states_[id].last_cmd;
        is_timed_out = motor_states_[id].timed_out;
      }
    }

    if (request_temp) {
      // Periodically query the winding temperature
      driver_client_->send_feedback_query(id);
    } else if (is_timed_out) {
      // Safety stop if command timed out or not yet commanded
      driver_client_->send_velocity_command(id, 0.0, 0, true);
    } else {
      total_tx_packets_++;
      switch (cmd.mode) {
        case ddsm115_ros2_driver::msg::Ddsm115Command::MODE_CURRENT:
          driver_client_->send_current_command(id, cmd.value);
          break;
        case ddsm115_ros2_driver::msg::Ddsm115Command::MODE_VELOCITY:
          driver_client_->send_velocity_command(id, cmd.value, 0,
            cmd.brake_mode == ddsm115_ros2_driver::msg::Ddsm115Command::BRAKE_LOCK);
          break;
        case ddsm115_ros2_driver::msg::Ddsm115Command::MODE_POSITION:
          driver_client_->send_position_command(id, cmd.value);
          break;
        default:
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Unknown command mode: %d", cmd.mode);
          break;
      }
    }

    // Sleep a tiny amount between serial packet commands to allow the motor to reply
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void DDSM115DriverNode::check_timeouts_callback()
{
  rclcpp::Time now = this->now();
  std::lock_guard<std::mutex> lock(state_mutex_);

  for (auto & pair : motor_states_) {
    uint8_t id = pair.first;
    auto & state = pair.second;

    if (!state.timed_out) {
      double elapsed = (now - state.last_cmd_time).seconds();
      if (elapsed > command_timeout_) {
        state.timed_out = true;
        RCLCPP_WARN(this->get_logger(), "Command timeout triggered for Motor ID: %d", id);
      }
    }
  }

  // Update diagnostics
  diagnostic_updater_.force_update();
}

void DDSM115DriverNode::setup_diagnostics()
{
  diagnostic_updater_.setHardwareID("DDSM115 RS485 Interface");
  diagnostic_updater_.add("DDSM115 Connection Status", this,
      &DDSM115DriverNode::produce_diagnostics);
}

void DDSM115DriverNode::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  bool serial_open = driver_client_ && driver_client_->is_port_open();

  if (!serial_open) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Serial port is CLOSED");
  } else {
    bool any_timeout = false;
    for (const auto & motor_id : motor_ids_) {
      uint8_t id = static_cast<uint8_t>(motor_id);
      if (motor_states_.count(id) > 0 && motor_states_[id].timed_out) {
        any_timeout = true;
        break;
      }
    }

    if (any_timeout) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "One or more motors timed out or not commanded");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Serial port open, communication active");
    }
  }

  stat.add("Serial Port Name", serial_port_);
  stat.add("Serial Baud Rate", baud_rate_);
  stat.add("Total Rx Packets", total_rx_packets_.load());
  stat.add("Total Tx Packets", total_tx_packets_.load());
  stat.add("Command Timeout Config", command_timeout_);

  for (const auto & motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);
    std::string key_prefix = "Motor_" + std::to_string(id);

    if (motor_states_.count(id) > 0) {
      const auto & state = motor_states_[id];
      stat.add(key_prefix + " Timeout Status", state.timed_out ? "TIMED OUT" : "ACTIVE");
      stat.add(key_prefix + " Mode", static_cast<int>(state.last_cmd.mode));
      if (state.has_status) {
        stat.add(key_prefix + " Status Feedback Current (A)", state.last_status.current);
        stat.add(key_prefix + " Status Feedback Velocity (RPM)", state.last_status.velocity);
        stat.add(key_prefix + " Status Feedback Position (Deg)", state.last_status.position);
        stat.add(key_prefix + " Status Feedback Winding Temperature (C)",
            state.last_status.winding_temperature);
        stat.add(key_prefix + " Status Error Code", static_cast<int>(state.last_status.error_code));
      } else {
        stat.add(key_prefix + " Status Feedback", "NO FEEDBACK RECEIVED");
      }
    }
  }
}

}  // namespace ddsm115_ros2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ddsm115_ros2_driver::DDSM115DriverNode)
