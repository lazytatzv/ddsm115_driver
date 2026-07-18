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
#ifndef DDSM115_ROS2_DRIVER__DDSM115_DRIVER_CLIENT_HPP_
#define DDSM115_ROS2_DRIVER__DDSM115_DRIVER_CLIENT_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

namespace ddsm115_ros2_driver
{
enum class ControlLoopModes : uint8_t
{
  MODE_NONE = 0x00,
  MODE_CURRENT = 0x01,
  MODE_VELOCITY = 0x02,
  MODE_POSITION = 0x03
};

enum class LogLevel
{
  DEBUG,
  INFO,
  WARN,
  ERROR
};

struct MotorFeedback
{
  uint8_t motor_id{0};
  ControlLoopModes mode{ControlLoopModes::MODE_NONE};
  double current{0.0};       // Amperes
  double velocity{0.0};      // RPM
  double position{0.0};      // Degrees (0 - 360)
  double winding_temperature{0.0};   // Celsius
  uint8_t error_code{0};     // Raw error byte
  bool sensor_fault{false};
  bool over_current{false};
  bool phase_over_current{false};
  bool stalling{false};
  bool troubleshooting{false};
  bool is_temperature_packet{false};
};

using LogCallback = std::function<void(LogLevel, const std::string &)>;
using FeedbackCallback = std::function<void(const MotorFeedback &)>;

class DDSM115DriverClient
{
public:
  DDSM115DriverClient(
    FeedbackCallback feedback_callback = nullptr,
    LogCallback log_callback = nullptr);
  ~DDSM115DriverClient();

  bool init_port(const std::string & port_name, int baud_rate = 115200);
  bool reinitialize_port();
  void close_port();
  bool is_port_open() const;

    // Direct protocol commands
  void send_mode_command(uint8_t motor_id, ControlLoopModes mode);
  bool send_current_command(uint8_t motor_id, double current_amps);
  bool send_velocity_command(
    uint8_t motor_id, double rpm, uint8_t acceleration = 0,
    bool brake = false);
  bool send_position_command(uint8_t motor_id, double position_degrees);
  bool send_feedback_query(uint8_t motor_id);
  bool query_motor_id();
  bool change_motor_id(uint8_t current_id, uint8_t new_id);

    // CRC helper
  static uint8_t calc_crc8_maxim(const uint8_t *data, size_t length);

    // Dynamic configuration of monitored motor IDs to reject noisy/junk serial frames
  void register_motor_id(uint8_t motor_id);
  void unregister_motor_id(uint8_t motor_id);
  void clear_registered_motor_ids();

protected:
  void start_async_read();
  void parse_buffer();
  void process_feedback_packet(const std::vector<uint8_t> & packet);
  void log(LogLevel level, const std::string & message);
  bool write_packet(const std::vector<uint8_t> & packet);
  bool is_valid_motor_id(uint8_t motor_id) const;

private:
  std::string port_name_;
  int baud_rate_;
  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;

  std::vector<uint8_t> buffer_;
  std::array<uint8_t, 256> read_buf_;
  std::atomic<bool> reading_;
  std::thread io_thread_;
  mutable std::mutex send_mutex_;
  std::mutex buffer_mutex_;

  FeedbackCallback feedback_callback_;
  LogCallback log_callback_;

  std::vector<uint8_t> registered_motor_ids_;
  std::mutex ids_mutex_;
  std::array<uint8_t, 256> last_sent_command_type_;
};
}  // namespace ddsm115_ros2_driver
#endif  // DDSM115_ROS2_DRIVER__DDSM115_DRIVER_CLIENT_HPP_
