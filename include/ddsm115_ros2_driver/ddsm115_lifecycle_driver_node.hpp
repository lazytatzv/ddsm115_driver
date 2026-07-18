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

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "ddsm115_ros2_driver/msg/ddsm115_command.hpp"
#include "ddsm115_ros2_driver/msg/ddsm115_status.hpp"
#include "ddsm115_ros2_driver/ddsm115_driver_client.hpp"
#include "ddsm115_ros2_driver/ddsm115_node_handler.hpp"

namespace ddsm115_ros2_driver
{

class DDSM115LifecycleDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit DDSM115LifecycleDriverNode(const rclcpp::NodeOptions & options);
  ~DDSM115LifecycleDriverNode();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  void motor_feedback_callback(const MotorFeedback & feedback);
  void topic_callback(
    const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg,
    uint8_t motor_id);
  void control_timer_callback();
  void check_timeouts_callback();
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  std::shared_ptr<DDSM115DriverClient> driver_client_;
  std::unique_ptr<DDSM115NodeHandler> handler_;

  std::string serial_port_;
  int baud_rate_;
  double publish_rate_;
  double command_timeout_;
  std::vector<int64_t> motor_ids_;

  std::map<uint8_t, rclcpp_lifecycle::LifecyclePublisher<ddsm115_ros2_driver::msg::Ddsm115Status>::SharedPtr> status_pubs_;
  std::map<uint8_t, rclcpp::Subscription<ddsm115_ros2_driver::msg::Ddsm115Command>::SharedPtr> command_subs_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  diagnostic_updater::Updater diagnostic_updater_;
};

}  // namespace ddsm115_ros2_driver

#endif  // DDSM115_ROS2_DRIVER__DDSM115_LIFECYCLE_DRIVER_NODE_HPP_
