#ifndef DDSM115_ROS2_DRIVER_DDSM115_DRIVER_NODE_HPP_
#define DDSM115_ROS2_DRIVER_DDSM115_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "ddsm115_ros2_driver/msg/ddsm115_command.hpp"
#include "ddsm115_ros2_driver/msg/ddsm115_status.hpp"
#include "ddsm115_ros2_driver/ddsm115_driver_client.hpp"

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
  void topic_callback(const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg, uint8_t motor_id);

  // Timer callbacks
  void control_timer_callback();
  void check_timeouts_callback();

  // Diagnostics
  void setup_diagnostics();
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Driver client
  std::unique_ptr<DDSM115DriverClient> driver_client_;

  // Parameters
  std::string serial_port_;
  int baud_rate_;
  double publish_rate_;
  double command_timeout_;
  std::vector<int64_t> motor_ids_;

  // ROS 2 publishers and subscribers
  std::map<uint8_t, rclcpp::Publisher<ddsm115_ros2_driver::msg::Ddsm115Status>::SharedPtr> status_pubs_;
  std::map<uint8_t, rclcpp::Subscription<ddsm115_ros2_driver::msg::Ddsm115Command>::SharedPtr> command_subs_;

  // Timers
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  uint32_t control_cycle_count_{0};

  // Keep track of motor command state
  struct MotorState
  {
    ddsm115_ros2_driver::msg::Ddsm115Command last_cmd;
    rclcpp::Time last_cmd_time;
    ddsm115_ros2_driver::msg::Ddsm115Status last_status;
    bool has_status = false;
    bool timed_out = true;
  };

  std::map<uint8_t, MotorState> motor_states_;
  std::mutex state_mutex_;

  // Diagnostic statistics
  diagnostic_updater::Updater diagnostic_updater_;
  std::atomic<uint64_t> total_rx_packets_{0};
  std::atomic<uint64_t> total_tx_packets_{0};
  std::atomic<uint64_t> crc_errors_{0};
};

}  // namespace ddsm115_ros2_driver

#endif  // DDSM115_ROS2_DRIVER_DDSM115_DRIVER_NODE_HPP_
