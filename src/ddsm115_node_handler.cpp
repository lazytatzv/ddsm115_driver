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

#include "ddsm115_ros2_driver/ddsm115_node_handler.hpp"

#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <string>

namespace ddsm115_ros2_driver
{

DDSM115NodeHandler::DDSM115NodeHandler(
  std::shared_ptr<DDSM115DriverClient> client,
  const std::vector<int64_t>& motor_ids,
  double command_timeout,
  rclcpp::Logger logger)
: driver_client_(client),
  motor_ids_(motor_ids),
  command_timeout_(command_timeout),
  logger_(logger)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (int64_t motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);
    motor_states_[id] = MotorState();
    // Default mode is velocity
    motor_states_[id].last_cmd.mode = ddsm115_ros2_driver::msg::Ddsm115Command::MODE_VELOCITY;
    motor_states_[id].last_cmd.value = 0.0;
    motor_states_[id].last_cmd.brake_mode = ddsm115_ros2_driver::msg::Ddsm115Command::BRAKE_RELEASE;
    motor_states_[id].timed_out = true;
    motor_states_[id].has_status = false;
    
    // Switch motor to initial velocity mode
    if (driver_client_ && driver_client_->is_port_open()) {
      driver_client_->send_mode_command(id, ControlLoopModes::MODE_VELOCITY);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}

bool DDSM115NodeHandler::process_feedback(
  const MotorFeedback & feedback,
  ddsm115_ros2_driver::msg::Ddsm115Status & out_status_msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (motor_states_.count(feedback.motor_id) == 0) {
    return false;
  }

  auto & state = motor_states_[feedback.motor_id];

  out_status_msg.current = feedback.current;
  out_status_msg.velocity = feedback.velocity;
  out_status_msg.error_code = feedback.error_code;

  if (feedback.is_temperature_packet) {
    out_status_msg.position = state.has_status ? state.last_status.position : feedback.position;
    out_status_msg.winding_temperature = feedback.winding_temperature;
  } else {
    out_status_msg.position = feedback.position;
    out_status_msg.winding_temperature = state.has_status ? state.last_status.winding_temperature : 0.0;
    
    // Track continuous position
    if (state.first_feedback) {
      state.prev_raw_position_deg = feedback.position;
      state.first_feedback = false;
    }
    double diff = feedback.position - state.prev_raw_position_deg;
    if (diff < -180.0) {
      state.wrap_count++;
    } else if (diff > 180.0) {
      state.wrap_count--;
    }
    state.prev_raw_position_deg = feedback.position;
    
    double accumulated_deg = feedback.position + static_cast<double>(state.wrap_count) * 360.0;
    state.continuous_position_rad = accumulated_deg * (M_PI / 180.0);
    state.velocity_rad_s = feedback.velocity * (2.0 * M_PI / 60.0);
  }

  state.last_status = out_status_msg;
  state.has_status = true;

  return true;
}

void DDSM115NodeHandler::set_command(
  uint8_t motor_id,
  const ddsm115_ros2_driver::msg::Ddsm115Command & cmd,
  rclcpp::Time now)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (motor_states_.count(motor_id) > 0) {
    motor_states_[motor_id].last_cmd = cmd;
    motor_states_[motor_id].last_cmd_time = now;
    motor_states_[motor_id].timed_out = false;
  }
}

void DDSM115NodeHandler::execute_control_cycle()
{
  if (!driver_client_) return;

  control_cycle_count_++;
  bool request_temp = (control_cycle_count_ % 20 == 0);

  // Send commands sequentially
  for (int64_t motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);

    ddsm115_ros2_driver::msg::Ddsm115Command cmd;
    bool is_timed_out = true;
    uint8_t current_active_mode = ddsm115_ros2_driver::msg::Ddsm115Command::MODE_VELOCITY;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (motor_states_.count(id) > 0) {
        cmd = motor_states_[id].last_cmd;
        is_timed_out = motor_states_[id].timed_out;
        current_active_mode = motor_states_[id].active_mode;
      }
    }

    if (request_temp) {
      driver_client_->send_feedback_query(id);
    } else if (is_timed_out) {
      driver_client_->send_velocity_command(id, 0.0, 0, true);
    } else {
      total_tx_packets_++;

      if (cmd.mode != current_active_mode) {
        driver_client_->send_mode_command(id, static_cast<ControlLoopModes>(cmd.mode));
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_states_[id].active_mode = cmd.mode;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }

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
          RCLCPP_WARN(logger_, "Unknown command mode: %d", cmd.mode);
          break;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void DDSM115NodeHandler::check_timeouts(rclcpp::Time now)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (auto & pair : motor_states_) {
    uint8_t id = pair.first;
    auto & state = pair.second;

    if (!state.timed_out) {
      double elapsed = (now - state.last_cmd_time).seconds();
      if (elapsed > command_timeout_) {
        state.timed_out = true;
        RCLCPP_WARN(logger_, "Command timeout triggered for Motor ID: %d", id);
      }
    }
  }
}

void DDSM115NodeHandler::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  bool any_timeout = false;
  for (const auto & motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);
    if (motor_states_.count(id) > 0 && motor_states_[id].timed_out) {
      any_timeout = true;
      break;
    }
  }

  if (!driver_client_ || !driver_client_->is_port_open()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Serial port is CLOSED");
  } else if (any_timeout) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "One or more motors timed out or not commanded");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Serial port open, communication active");
  }

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
        stat.add(key_prefix + " Status Feedback Winding Temperature (C)", state.last_status.winding_temperature);
        stat.add(key_prefix + " Status Error Code Raw", static_cast<int>(state.last_status.error_code));
        
        std::string error_str = "OK";
        if (state.last_status.error_code != 0) {
          std::vector<std::string> errs;
          if (state.last_status.error_code & 0x01) errs.push_back("Sensor Fault");
          if (state.last_status.error_code & 0x02) errs.push_back("Over Current");
          if (state.last_status.error_code & 0x04) errs.push_back("Phase Over Current");
          if (state.last_status.error_code & 0x08) errs.push_back("Stall");
          if (state.last_status.error_code & 0x10) errs.push_back("Troubleshooting");
          if (!errs.empty()) {
            error_str = "";
            for (size_t i = 0; i < errs.size(); ++i) {
              error_str += errs[i] + (i < errs.size() - 1 ? " | " : "");
            }
          } else {
            error_str = "Unknown Error";
          }
        }
        stat.add(key_prefix + " Status Error Summary", error_str);
      } else {
        stat.add(key_prefix + " Status Feedback", "NO FEEDBACK RECEIVED");
      }
    }
  }
}

bool DDSM115NodeHandler::get_joint_state(
  sensor_msgs::msg::JointState & msg,
  const std::map<uint8_t, std::string> & joint_names,
  rclcpp::Time now)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  msg.header.stamp = now;
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();

  for (const auto & pair : joint_names) {
    uint8_t id = pair.first;
    if (motor_states_.count(id) > 0) {
      const auto & state = motor_states_[id];
      msg.name.push_back(pair.second);
      if (state.has_status && !state.first_feedback) {
        msg.position.push_back(state.continuous_position_rad);
        msg.velocity.push_back(state.velocity_rad_s);
        msg.effort.push_back(state.last_status.current);
      } else {
        msg.position.push_back(0.0);
        msg.velocity.push_back(0.0);
        msg.effort.push_back(0.0);
      }
    }
  }
  return !msg.name.empty();
}

}  // namespace ddsm115_ros2_driver
