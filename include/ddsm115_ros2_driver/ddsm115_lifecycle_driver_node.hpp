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

#ifndef DDSM115_ROS2_DRIVER__DDSM115_LIFECYCLE_DRIVER_NODE_HPP_
#define DDSM115_ROS2_DRIVER__DDSM115_LIFECYCLE_DRIVER_NODE_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ddsm115_ros2_driver/ddsm115_driver_client.hpp"
#include "ddsm115_ros2_driver/msg/ddsm115_command.hpp"
#include "ddsm115_ros2_driver/msg/ddsm115_status.hpp"

namespace ddsm115_ros2_driver
{
class DDSM115LifecycleDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit DDSM115LifecycleDriverNode(const rclcpp::NodeOptions & options);
  ~DDSM115LifecycleDriverNode() override;

  // Lifecycle transitions
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  void motor_feedback_callback(const MotorFeedback & feedback);
  void topic_callback(
    const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg, uint8_t motor_id);
  void control_timer_callback();
  void check_timeouts_callback();
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Configuration parameters
  std::string serial_port_;
  int baud_rate_;
  std::vector<int64_t> motor_ids_;
  double publish_rate_;
  double command_timeout_;

  // Diagnostics
  diagnostic_updater::Updater diagnostic_updater_;
  std::atomic<uint64_t> total_rx_packets_{0};
  std::atomic<uint64_t> total_tx_packets_{0};

  // Pure low-level C++ serial client
  std::unique_ptr<DDSM115DriverClient> driver_client_;

  // ROS 2 publishers and subscribers
  std::map<uint8_t, rclcpp_lifecycle::LifecyclePublisher<ddsm115_ros2_driver::msg::Ddsm115Status>::SharedPtr> status_pubs_;
  std::map<uint8_t, rclcpp::Subscription<ddsm115_ros2_driver::msg::Ddsm115Command>::SharedPtr> command_subs_;

  // Timers
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  uint32_t control_cycle_count_{0};

  // Keep track of motor command state
  struct MotorState
  {
    ddsm115_ros2_driver::msg::Ddsm115Command last_cmd;
    ddsm115_ros2_driver::msg::Ddsm115Status last_status;
    rclcpp::Time last_cmd_time{0, 0, RCL_ROS_TIME};
    bool has_status{false};
    bool timed_out{true};
    uint8_t active_mode = ddsm115_ros2_driver::msg::Ddsm115Command::MODE_VELOCITY;
  };

  std::map<uint8_t, MotorState> motor_states_;
  std::mutex state_mutex_;
};
}  // namespace ddsm115_ros2_driver

#endif  // DDSM115_ROS2_DRIVER__DDSM115_LIFECYCLE_DRIVER_NODE_HPP_
