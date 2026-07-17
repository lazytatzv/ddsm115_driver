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

#include "ddsm115_ros2_driver/ddsm115_lifecycle_driver_node.hpp"

#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace ddsm115_ros2_driver
{
DDSM115LifecycleDriverNode::DDSM115LifecycleDriverNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("ddsm115_lifecycle_driver_node", options),
  diagnostic_updater_(this)
{
  // Declare parameters
  this->declare_parameter("serial_port", "/dev/ttyUSB0");
  this->declare_parameter("baud_rate", 115200);
  this->declare_parameter("motor_ids", std::vector<int64_t>{1, 2});
  this->declare_parameter("publish_rate", 20.0);       // Hz
  this->declare_parameter("command_timeout", 1.0);     // Seconds
}

DDSM115LifecycleDriverNode::~DDSM115LifecycleDriverNode()
{
  if (driver_client_) {
    driver_client_->close_port();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DDSM115LifecycleDriverNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring DDSM115 Lifecycle Node...");

  // Retrieve parameters
  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  motor_ids_ = this->get_parameter("motor_ids").as_integer_array();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  command_timeout_ = this->get_parameter("command_timeout").as_double();

  if (motor_ids_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'motor_ids' is empty. At least one ID required.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  // Initialize pure C++ client
  driver_client_ = std::make_unique<DDSM115DriverClient>(
    std::bind(&DDSM115LifecycleDriverNode::motor_feedback_callback, this, std::placeholders::_1),
    [this](LogLevel level, const std::string & msg) {
      switch (level) {
        case LogLevel::INFO:
          RCLCPP_INFO(this->get_logger(), "[Client] %s", msg.c_str());
          break;
        case LogLevel::WARN:
          RCLCPP_WARN(this->get_logger(), "[Client] %s", msg.c_str());
          break;
        case LogLevel::ERR:
          RCLCPP_ERROR(this->get_logger(), "[Client] %s", msg.c_str());
          break;
        case LogLevel::DEBUG:
        default:
          RCLCPP_DEBUG(this->get_logger(), "[Client] %s", msg.c_str());
          break;
      }
    });

  // Open serial port
  if (!driver_client_->open_port(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  // Register motor IDs
  for (int64_t id : motor_ids_) {
    driver_client_->register_motor_id(static_cast<uint8_t>(id));
  }

  // Setup diagnostics
  diagnostic_updater_.setHardwareID("DDSM115 RS485 Interface");
  diagnostic_updater_.add(
    "DDSM115 Connection Status", this, &DDSM115LifecycleDriverNode::produce_diagnostics);

  RCLCPP_INFO(this->get_logger(), "DDSM115 Lifecycle Node successfully configured.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DDSM115LifecycleDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up DDSM115 Lifecycle Node...");

  if (driver_client_) {
    driver_client_->close_port();
    driver_client_.reset();
  }

  status_pubs_.clear();
  command_subs_.clear();
  motor_states_.clear();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DDSM115LifecycleDriverNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating DDSM115 Lifecycle Node...");

  // Setup publishers and subscribers for each motor ID
  for (int64_t motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);
    std::string prefix = "motor_" + std::to_string(id);

    // Create publisher
    status_pubs_[id] =
      this->create_publisher<ddsm115_ros2_driver::msg::Ddsm115Status>(prefix + "/status", 10);

    // Create subscriber
    command_subs_[id] = this->create_subscription<ddsm115_ros2_driver::msg::Ddsm115Command>(
      prefix + "/command", 10,
      [this, id](const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg) {
        this->topic_callback(msg, id);
      });

    // Initialize motor state
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      motor_states_[id] = MotorState();
    }
  }

  // Activate publishers
  for (auto & pair : status_pubs_) {
    pair.second->on_activate();
  }

  // Create timer for sending commands (sequential staggered polling loop)
  double timer_period_sec = 1.0 / publish_rate_;
  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_sec),
    std::bind(&DDSM115LifecycleDriverNode::control_timer_callback, this));

  // Create watchdog timer for timeouts (10Hz)
  timeout_timer_ = this->create_wall_timer(
    100ms, std::bind(&DDSM115LifecycleDriverNode::check_timeouts_callback, this));

  RCLCPP_INFO(this->get_logger(), "DDSM115 Lifecycle Node activated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DDSM115LifecycleDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating DDSM115 Lifecycle Node...");

  // Cancel timers
  if (control_timer_) {
    control_timer_->cancel();
    control_timer_.reset();
  }
  if (timeout_timer_) {
    timeout_timer_->cancel();
    timeout_timer_.reset();
  }

  // Deactivate publishers
  for (auto & pair : status_pubs_) {
    pair.second->on_deactivate();
  }

  // Brake and stop all motors
  for (int64_t motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);
    if (driver_client_) {
      driver_client_->send_velocity_command(id, 0.0, 0, true);
    }
    std::this_thread::sleep_for(5ms);
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DDSM115LifecycleDriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down DDSM115 Lifecycle Node...");

  if (control_timer_) {
    control_timer_->cancel();
    control_timer_.reset();
  }
  if (timeout_timer_) {
    timeout_timer_->cancel();
    timeout_timer_.reset();
  }

  // Stop all motors
  for (int64_t motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);
    if (driver_client_) {
      driver_client_->send_velocity_command(id, 0.0, 0, true);
    }
    std::this_thread::sleep_for(5ms);
  }

  if (driver_client_) {
    driver_client_->close_port();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void DDSM115LifecycleDriverNode::motor_feedback_callback(const MotorFeedback & feedback)
{
  total_rx_packets_++;

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (motor_states_.count(feedback.motor_id) == 0) {
    return;
  }

  auto & state = motor_states_[feedback.motor_id];

  ddsm115_ros2_driver::msg::Ddsm115Status status_msg;
  status_msg.current = feedback.current;
  status_msg.velocity = feedback.velocity;
  status_msg.error_code = feedback.error_code;

  if (feedback.is_temperature_packet) {
    status_msg.position = state.has_status ? state.last_status.position : feedback.position;
    status_msg.winding_temperature = feedback.winding_temperature;
  } else {
    status_msg.position = feedback.position;
    status_msg.winding_temperature = state.has_status ? state.last_status.winding_temperature : 0.0;
  }

  // Publish status if active
  if (status_pubs_.count(feedback.motor_id) > 0) {
    status_pubs_[feedback.motor_id]->publish(status_msg);
  }

  state.last_status = status_msg;
  state.has_status = true;
}

void DDSM115LifecycleDriverNode::topic_callback(
  const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg, uint8_t motor_id)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (motor_states_.count(motor_id) > 0) {
    motor_states_[motor_id].last_cmd = *msg;
    motor_states_[motor_id].last_cmd_time = this->now();
    motor_states_[motor_id].timed_out = false;
  }
}

void DDSM115LifecycleDriverNode::control_timer_callback()
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
      driver_client_->send_feedback_query(id);
    } else if (is_timed_out) {
      driver_client_->send_velocity_command(id, 0.0, 0, true);
    } else {
      total_tx_packets_++;
      switch (cmd.mode) {
        case ddsm115_ros2_driver::msg::Ddsm115Command::MODE_CURRENT:
          driver_client_->send_current_command(id, cmd.value);
          break;
        case ddsm115_ros2_driver::msg::Ddsm115Command::MODE_VELOCITY:
          driver_client_->send_velocity_command(
            id, cmd.value, 0,
            cmd.brake_mode == ddsm115_ros2_driver::msg::Ddsm115Command::BRAKE_LOCK);
          break;
        case ddsm115_ros2_driver::msg::Ddsm115Command::MODE_POSITION:
          driver_client_->send_position_command(id, cmd.value);
          break;
        default:
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000, "Unknown command mode: %d", cmd.mode);
          break;
      }
    }

    // Sleep a tiny amount between serial packet commands to allow the motor to reply
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void DDSM115LifecycleDriverNode::check_timeouts_callback()
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

  diagnostic_updater_.force_update();
}

void DDSM115LifecycleDriverNode::produce_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!driver_client_ || !driver_client_->is_port_open()) {
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
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "One or more motors timed out or not commanded");
    } else {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK, "Serial port open, communication active");
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
        stat.add(
          key_prefix + " Status Feedback Winding Temperature (C)",
          state.last_status.winding_temperature);
        stat.add(key_prefix + " Status Error Code", static_cast<int>(state.last_status.error_code));
      } else {
        stat.add(key_prefix + " Status Feedback", "NO FEEDBACK RECEIVED");
      }
    }
  }
}
}  // namespace ddsm115_ros2_driver
