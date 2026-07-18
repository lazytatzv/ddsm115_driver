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

#ifndef DDSM115_ROS2_DRIVER__DDSM115_NODE_HANDLER_HPP_
#define DDSM115_ROS2_DRIVER__DDSM115_NODE_HANDLER_HPP_

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ddsm115_ros2_driver/msg/ddsm115_command.hpp"
#include "ddsm115_ros2_driver/msg/ddsm115_status.hpp"
#include "ddsm115_ros2_driver/ddsm115_driver_client.hpp"

namespace ddsm115_ros2_driver
{

struct MotorState
{
  ddsm115_ros2_driver::msg::Ddsm115Command last_cmd;
  rclcpp::Time last_cmd_time{0, 0, RCL_ROS_TIME};
  ddsm115_ros2_driver::msg::Ddsm115Status last_status;
  bool has_status{false};
  bool timed_out{true};
  uint8_t active_mode{ddsm115_ros2_driver::msg::Ddsm115Command::MODE_VELOCITY};

  bool first_feedback{true};
  double prev_raw_position_deg{0.0};
  int64_t wrap_count{0};
  double continuous_position_rad{0.0};
  double velocity_rad_s{0.0};
};

class DDSM115NodeHandler
{
public:
  DDSM115NodeHandler(
    std::shared_ptr<DDSM115DriverClient> client,
    const std::vector<int64_t>& motor_ids,
    double command_timeout,
    rclcpp::Logger logger);

  ~DDSM115NodeHandler() = default;

  // Process incoming hardware feedback and return the generated ROS status message
  bool process_feedback(
    const MotorFeedback & feedback,
    ddsm115_ros2_driver::msg::Ddsm115Status & out_status_msg);

  // Store a new incoming ROS command
  void set_command(
    uint8_t motor_id,
    const ddsm115_ros2_driver::msg::Ddsm115Command & cmd,
    rclcpp::Time now);

  // Execute the control loop for all motors (send commands/queries)
  void execute_control_cycle();

  // Check for command timeouts across all motors
  void check_timeouts(rclcpp::Time now);

  // Fill diagnostic data for all motors
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  
  // Fill JointState message
  bool get_joint_state(
    sensor_msgs::msg::JointState & msg,
    const std::map<uint8_t, std::string> & joint_names,
    rclcpp::Time now);
  
  // Track packets
  void increment_rx() { total_rx_packets_++; }

  uint64_t get_rx_packets() const { return total_rx_packets_.load(); }
  uint64_t get_tx_packets() const { return total_tx_packets_.load(); }

private:
  std::shared_ptr<DDSM115DriverClient> driver_client_;
  std::vector<int64_t> motor_ids_;
  double command_timeout_;
  rclcpp::Logger logger_;

  std::map<uint8_t, MotorState> motor_states_;
  mutable std::mutex state_mutex_;

  uint32_t control_cycle_count_{0};
  std::atomic<uint64_t> total_rx_packets_{0};
  std::atomic<uint64_t> total_tx_packets_{0};
};

}  // namespace ddsm115_ros2_driver
#endif  // DDSM115_ROS2_DRIVER__DDSM115_NODE_HANDLER_HPP_
