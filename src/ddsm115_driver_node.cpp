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

using namespace std::chrono_literals;

namespace ddsm115_ros2_driver
{

DDSM115DriverNode::DDSM115DriverNode(const rclcpp::NodeOptions & options)
: Node("ddsm115_driver_node", options), diagnostic_updater_(this)
{
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("serial_baud", 115200);
  this->declare_parameter<double>("publish_rate", 20.0);
  this->declare_parameter<double>("command_timeout", 1.0);
  this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{1});

  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("serial_baud").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  command_timeout_ = this->get_parameter("command_timeout").as_double();
  motor_ids_ = this->get_parameter("motor_ids").as_integer_array();

  driver_client_ = std::make_shared<DDSM115DriverClient>(
    std::bind(&DDSM115DriverNode::motor_feedback_callback, this, std::placeholders::_1),
    [this](LogLevel level, const std::string & msg) {
      switch (level) {
        case LogLevel::DEBUG: RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str()); break;
        case LogLevel::INFO:  RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); break;
        case LogLevel::WARN:  RCLCPP_WARN(this->get_logger(), "%s", msg.c_str()); break;
        case LogLevel::ERROR: RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); break;
      }
    });

  driver_client_->clear_registered_motor_ids();
  for (int64_t motor_id : motor_ids_) {
    driver_client_->register_motor_id(static_cast<uint8_t>(motor_id));
  }

  if (!driver_client_->init_port(serial_port_, baud_rate_)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize serial port: %s", serial_port_.c_str());
    throw std::runtime_error("Failed to initialize serial port");
  }

  handler_ = std::make_unique<DDSM115NodeHandler>(driver_client_, motor_ids_, command_timeout_, this->get_logger());

  for (int64_t motor_id : motor_ids_) {
    uint8_t id = static_cast<uint8_t>(motor_id);
    std::string ns = "motor_" + std::to_string(id);
    status_pubs_[id] = this->create_publisher<ddsm115_ros2_driver::msg::Ddsm115Status>(ns + "/status", 10);
    command_subs_[id] = this->create_subscription<ddsm115_ros2_driver::msg::Ddsm115Command>(
      ns + "/command", 10, [this, id](const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg) {
        this->topic_callback(msg, id);
      });
  }

  setup_diagnostics();

  double timer_period_sec = 1.0 / publish_rate_;
  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_sec),
    std::bind(&DDSM115DriverNode::control_timer_callback, this));

  timeout_timer_ = this->create_wall_timer(
    100ms, std::bind(&DDSM115DriverNode::check_timeouts_callback, this));

  RCLCPP_INFO(this->get_logger(), "DDSM115 Standalone Node successfully initialized.");
}

DDSM115DriverNode::~DDSM115DriverNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down DDSM115 Standalone Node...");
  if (control_timer_) {control_timer_->cancel();}
  if (timeout_timer_) {timeout_timer_->cancel();}

  for (int64_t motor_id : motor_ids_) {
    driver_client_->send_velocity_command(static_cast<uint8_t>(motor_id), 0.0, 0, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (driver_client_) {
    driver_client_->close_port();
  }
}

void DDSM115DriverNode::motor_feedback_callback(const MotorFeedback & feedback)
{
  if (!handler_) return;
  handler_->increment_rx();
  ddsm115_ros2_driver::msg::Ddsm115Status status_msg;
  if (handler_->process_feedback(feedback, status_msg) && status_pubs_.count(feedback.motor_id)) {
    status_pubs_[feedback.motor_id]->publish(status_msg);
  }
}

void DDSM115DriverNode::topic_callback(const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg, uint8_t motor_id)
{
  if (handler_) {
    handler_->set_command(motor_id, *msg, this->now());
  }
}

void DDSM115DriverNode::control_timer_callback()
{
  if (handler_) {
    handler_->execute_control_cycle();
  }
}

void DDSM115DriverNode::check_timeouts_callback()
{
  if (handler_) {
    handler_->check_timeouts(this->now());
  }
  diagnostic_updater_.force_update();
}

void DDSM115DriverNode::setup_diagnostics()
{
  diagnostic_updater_.setHardwareID("DDSM115 RS485 Interface");
  diagnostic_updater_.add("DDSM115 Connection Status", this, &DDSM115DriverNode::produce_diagnostics);
}

void DDSM115DriverNode::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("Serial Port Name", serial_port_);
  stat.add("Serial Baud Rate", baud_rate_);
  if (handler_) {
    handler_->produce_diagnostics(stat);
  }
}

}  // namespace ddsm115_ros2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ddsm115_ros2_driver::DDSM115DriverNode)
