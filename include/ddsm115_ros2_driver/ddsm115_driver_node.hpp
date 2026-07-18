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
#ifndef DDSM115_ROS2_DRIVER__DDSM115_DRIVER_NODE_HPP_
#define DDSM115_ROS2_DRIVER__DDSM115_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ddsm115_ros2_driver/msg/ddsm115_command.hpp"
#include "ddsm115_ros2_driver/msg/ddsm115_status.hpp"
#include "ddsm115_ros2_driver/ddsm115_driver_client.hpp"
#include "ddsm115_ros2_driver/ddsm115_node_handler.hpp"

namespace ddsm115_ros2_driver
{

class DDSM115DriverNode : public rclcpp::Node
{
public:
  explicit DDSM115DriverNode(const rclcpp::NodeOptions & options);
  virtual ~DDSM115DriverNode();

private:
  // Low-level feedback callback
  void motor_feedback_callback(const MotorFeedback & feedback);

  // Subscriber callbacks
  void topic_callback(
    const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg,
    uint8_t motor_id);

  // Timer callbacks
  void control_timer_callback();
  void check_timeouts_callback();

  // Diagnostics
  void setup_diagnostics();
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Driver client & handler
  std::shared_ptr<DDSM115DriverClient> driver_client_;
  std::unique_ptr<DDSM115NodeHandler> handler_;

  // Parameters
  std::string serial_port_;
  int baud_rate_;
  double publish_rate_;
  double command_timeout_;
  std::vector<int64_t> motor_ids_;
  std::vector<std::string> joint_names_param_;
  std::map<uint8_t, std::string> joint_names_;

  // ROS 2 publishers and subscribers
  std::map<uint8_t, rclcpp::Publisher<ddsm115_ros2_driver::msg::Ddsm115Status>::SharedPtr> status_pubs_;
  std::map<uint8_t, rclcpp::Subscription<ddsm115_ros2_driver::msg::Ddsm115Command>::SharedPtr> command_subs_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  diagnostic_updater::Updater diagnostic_updater_;
};

}  // namespace ddsm115_ros2_driver

#endif  // DDSM115_ROS2_DRIVER__DDSM115_DRIVER_NODE_HPP_
