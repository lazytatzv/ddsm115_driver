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
  this->declare_parameter("serial_port", "/dev/ttyUSB0");
  this->declare_parameter("baud_rate", 115200);
  this->declare_parameter("motor_ids", std::vector<int64_t>{1, 2});
  this->declare_parameter("joint_names", std::vector<std::string>{"", ""});
  this->declare_parameter("publish_rate", 20.0);
  this->declare_parameter("command_timeout", 1.0);
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

  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  motor_ids_ = this->get_parameter("motor_ids").as_integer_array();
  joint_names_param_ = this->get_parameter("joint_names").as_string_array();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  command_timeout_ = this->get_parameter("command_timeout").as_double();

  if (motor_ids_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'motor_ids' is empty. At least one ID required.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  
  if (joint_names_param_.size() == motor_ids_.size() && !joint_names_param_.empty() && joint_names_param_[0] != "") {
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      joint_names_[static_cast<uint8_t>(motor_ids_[i])] = joint_names_param_[i];
    }
  }

  driver_client_ = std::make_shared<DDSM115DriverClient>(
    std::bind(&DDSM115LifecycleDriverNode::motor_feedback_callback, this, std::placeholders::_1),
    [this](LogLevel level, const std::string & msg) {
      switch (level) {
        case LogLevel::INFO:  RCLCPP_INFO(this->get_logger(), "[Client] %s", msg.c_str()); break;
        case LogLevel::WARN:  RCLCPP_WARN(this->get_logger(), "[Client] %s", msg.c_str()); break;
        case LogLevel::ERROR: RCLCPP_ERROR(this->get_logger(), "[Client] %s", msg.c_str()); break;
        case LogLevel::DEBUG:
        default:              RCLCPP_DEBUG(this->get_logger(), "[Client] %s", msg.c_str()); break;
      }
    });

  if (!driver_client_->init_port(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  driver_client_->clear_registered_motor_ids();
  for (int64_t id : motor_ids_) {
    driver_client_->register_motor_id(static_cast<uint8_t>(id));
  }

  handler_ = std::make_unique<DDSM115NodeHandler>(driver_client_, motor_ids_, command_timeout_, this->get_logger());

  diagnostic_updater_.setHardwareID("DDSM115 RS485 Interface");
  diagnostic_updater_.add("DDSM115 Connection Status", this, &DDSM115LifecycleDriverNode::produce_diagnostics);

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
  
  handler_.reset();
  status_pubs_.clear();
  command_subs_.clear();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DDSM115LifecycleDriverNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating DDSM115 Lifecycle Node...");

  for (int64_t motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);
    std::string prefix = "motor_" + std::to_string(id);

    status_pubs_[id] = this->create_publisher<ddsm115_ros2_driver::msg::Ddsm115Status>(prefix + "/status", 10);
    command_subs_[id] = this->create_subscription<ddsm115_ros2_driver::msg::Ddsm115Command>(
      prefix + "/command", 10, [this, id](const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg) {
        this->topic_callback(msg, id);
      });
  }
  
  if (!joint_names_.empty()) {
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    joint_state_pub_->on_activate();
  }

  for (auto & pair : status_pubs_) {
    pair.second->on_activate();
  }

  double timer_period_sec = 1.0 / publish_rate_;
  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_sec),
    std::bind(&DDSM115LifecycleDriverNode::control_timer_callback, this));

  timeout_timer_ = this->create_wall_timer(
    100ms, std::bind(&DDSM115LifecycleDriverNode::check_timeouts_callback, this));

  RCLCPP_INFO(this->get_logger(), "DDSM115 Lifecycle Node activated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DDSM115LifecycleDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating DDSM115 Lifecycle Node...");

  if (control_timer_) {
    control_timer_->cancel();
    control_timer_.reset();
  }
  if (timeout_timer_) {
    timeout_timer_->cancel();
    timeout_timer_.reset();
  }

  for (auto & pair : status_pubs_) {
    pair.second->on_deactivate();
  }
  
  if (joint_state_pub_) {
    joint_state_pub_->on_deactivate();
  }

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
  if (!handler_) return;
  handler_->increment_rx();
  ddsm115_ros2_driver::msg::Ddsm115Status status_msg;
  if (handler_->process_feedback(feedback, status_msg) && status_pubs_.count(feedback.motor_id)) {
    status_pubs_[feedback.motor_id]->publish(status_msg);
  }
}

void DDSM115LifecycleDriverNode::topic_callback(
  const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg, uint8_t motor_id)
{
  if (handler_) {
    handler_->set_command(motor_id, *msg, this->now());
  }
}

void DDSM115LifecycleDriverNode::control_timer_callback()
{
  if (handler_) {
    handler_->execute_control_cycle();
    
    if (joint_state_pub_ && joint_state_pub_->is_activated()) {
      sensor_msgs::msg::JointState js_msg;
      if (handler_->get_joint_state(js_msg, joint_names_, this->now())) {
        joint_state_pub_->publish(js_msg);
      }
    }
  }
}

void DDSM115LifecycleDriverNode::check_timeouts_callback()
{
  if (handler_) {
    handler_->check_timeouts(this->now());
  }
  diagnostic_updater_.force_update();
}

void DDSM115LifecycleDriverNode::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("Serial Port Name", serial_port_);
  stat.add("Serial Baud Rate", baud_rate_);
  if (handler_) {
    handler_->produce_diagnostics(stat);
  }
}

}  // namespace ddsm115_ros2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ddsm115_ros2_driver::DDSM115LifecycleDriverNode)
