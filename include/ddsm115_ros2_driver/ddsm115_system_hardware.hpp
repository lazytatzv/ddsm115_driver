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
#ifndef DDSM115_ROS2_DRIVER__DDSM115_SYSTEM_HARDWARE_HPP_
#define DDSM115_ROS2_DRIVER__DDSM115_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ddsm115_ros2_driver/ddsm115_driver_client.hpp"

namespace ddsm115_ros2_driver
{

class DDSM115SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DDSM115SystemHardware)

  ~DDSM115SystemHardware() override;

#ifdef ROS_DISTRO_HUMBLE
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
#else
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
#endif

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void motor_feedback_callback(const MotorFeedback & feedback);

  // Pure low-level C++ serial client
  std::unique_ptr<DDSM115DriverClient> driver_client_;

  std::string serial_port_;
  int baud_rate_{115200};

  // ROS Node for logging only
  rclcpp::Node::SharedPtr node_;

  struct Motor
  {
    uint8_t id = 0;
    bool invert = false;

    // Command states
    double command_velocity = 0.0;  // rad/s

    // Feedback states
    double position = 0.0;  // rad (continuous accumulated)
    double velocity = 0.0;  // rad/s
    double effort = 0.0;   // Amperes

    // Wrap around tracking variables
    double prev_raw_position_deg = 0.0;
    bool first_feedback = true;
    int32_t wrap_count = 0;
  };

  std::vector<Motor> motors_;
  std::map<uint8_t, size_t> id_to_index_;

  // Background thread for non-blocking serial communication
  std::thread communication_thread_;
  std::atomic<bool> communication_active_{false};
  void communication_loop();

  std::mutex state_mutex_;
};

}  // namespace ddsm115_ros2_driver

#endif  // DDSM115_ROS2_DRIVER__DDSM115_SYSTEM_HARDWARE_HPP_
