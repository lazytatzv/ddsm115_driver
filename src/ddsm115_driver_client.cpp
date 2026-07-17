#include "ddsm115_ros2_driver/ddsm115_driver_client.hpp"

#include <cmath>
#include <mutex>
#include <thread>
#include <iostream>
#include <sstream>
#include <algorithm>

namespace ddsm115_ros2_driver
{

  uint8_t DDSM115DriverClient::calc_crc8_maxim(const uint8_t *data, size_t length)
  {
    uint8_t crc = 0x00;
    const uint8_t reflected_polynomial = 0x8C; // 0x31 reflected

    for (size_t i = 0; i < length; i++)
    {
      crc ^= data[i];
      for (uint8_t bit = 0; bit < 8; bit++)
      {
        if (crc & 0x01)
        {
          crc = (crc >> 1) ^ reflected_polynomial;
        }
        else
        {
          crc >>= 1;
        }
      }
    }
    return crc;
  }

  DDSM115DriverClient::DDSM115DriverClient(FeedbackCallback feedback_callback, LogCallback log_callback)
      : serial_port_(io_context_), buffer_(), reading_(false), feedback_callback_(feedback_callback), log_callback_(log_callback) {}

  DDSM115DriverClient::~DDSM115DriverClient()
  {
    close_port();
  }

  void DDSM115DriverClient::log(LogLevel level, const std::string &message)
  {
    if (log_callback_)
    {
      log_callback_(level, message);
    }
    else
    {
      std::string level_str;
      switch (level)
      {
      case LogLevel::DEBUG:
        level_str = "[DEBUG]";
        break;
      case LogLevel::INFO:
        level_str = "[INFO]";
        break;
      case LogLevel::WARN:
        level_str = "[WARN]";
        break;
      case LogLevel::ERROR:
        level_str = "[ERROR]";
        break;
      }
      std::cerr << level_str << " " << message << std::endl;
    }
  }

  void DDSM115DriverClient::close_port()
  {
    reading_ = false;

    if (!io_context_.stopped())
    {
      io_context_.stop();
    }

    if (io_thread_.joinable())
    {
      io_thread_.join();
    }

    if (serial_port_.is_open())
    {
      boost::system::error_code ec;
      serial_port_.close(ec);
    }
    log(LogLevel::INFO, "Serial port closed.");
  }

  bool DDSM115DriverClient::is_port_open() const
  {
    return serial_port_.is_open();
  }

  bool DDSM115DriverClient::init_port(const std::string &port_name, int baud_rate)
  {
    try
    {
      this->port_name_ = port_name;
      this->baud_rate_ = baud_rate;
      
      serial_port_.open(this->port_name_);
      serial_port_.set_option(boost::asio::serial_port_base::baud_rate(this->baud_rate_));
      serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
      serial_port_.set_option(boost::asio::serial_port_base::flow_control(
          boost::asio::serial_port_base::flow_control::none));
      serial_port_.set_option(
          boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial_port_.set_option(
          boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

      reading_ = true;
      start_async_read();

      if (io_context_.stopped())
      {
        io_context_.restart();
      }

      if (!io_thread_.joinable())
      {
        io_thread_ = std::thread([this]()
                                 { io_context_.run(); });
      }
      
      std::stringstream ss;
      ss << "Successfully opened serial port: " << port_name_ << " at " << baud_rate_ << " bps";
      log(LogLevel::INFO, ss.str());
    }
    catch (const std::exception &e)
    {
      std::stringstream ss;
      ss << "Failed to open serial port " << port_name << ": " << e.what();
      log(LogLevel::ERROR, ss.str());
      return false;
    }
    return true;
  }

  bool DDSM115DriverClient::reinitialize_port()
  {
    log(LogLevel::WARN, "Reinitializing serial port: " + port_name_);
    close_port();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return init_port(port_name_, baud_rate_);
  }

  bool DDSM115DriverClient::write_packet(const std::vector<uint8_t> &packet)
  {
    try
    {
      std::lock_guard<std::mutex> lock(send_mutex_);
      if (!serial_port_.is_open())
      {
        log(LogLevel::ERROR, "Cannot write packet: Serial port is not open.");
        return false;
      }
      boost::asio::write(serial_port_, boost::asio::buffer(packet.data(), packet.size()));
      return true;
    }
    catch (const std::exception &e)
    {
      std::stringstream ss;
      ss << "Failed writing to serial port: " << e.what();
      log(LogLevel::ERROR, ss.str());
      reinitialize_port();
      return false;
    }
  }

  void DDSM115DriverClient::send_mode_command(uint8_t motor_id, ControlLoopModes mode)
  {
    std::vector<uint8_t> data(10, 0x00);
    data[0] = motor_id;
    data[1] = 0xA0;
    data[9] = static_cast<uint8_t>(mode);

    std::stringstream ss;
    ss << "Sending mode command to motor " << static_cast<int>(motor_id) << ": mode " << static_cast<int>(mode);
    log(LogLevel::INFO, ss.str());
    write_packet(data);
  }

  bool DDSM115DriverClient::send_current_command(uint8_t motor_id, double current_amps)
  {
    std::vector<uint8_t> data(10, 0x00);
    data[0] = motor_id;
    data[1] = 0x64;
    
    // Map current (-8.0A to 8.0A) to (-32767 to 32767)
    int16_t mapped_val = static_cast<int16_t>(std::clamp(std::round(current_amps * (32767.0 / 8.0)), -32767.0, 32767.0));
    data[2] = static_cast<uint8_t>((mapped_val >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(mapped_val & 0xFF);
    data[9] = calc_crc8_maxim(data.data(), 9);

    return write_packet(data);
  }

  bool DDSM115DriverClient::send_velocity_command(uint8_t motor_id, double rpm, uint8_t acceleration, bool brake)
  {
    std::vector<uint8_t> data(10, 0x00);
    data[0] = motor_id;
    data[1] = 0x64;

    // Map velocity (-330 RPM to 330 RPM) to signed 16-bit int
    int16_t mapped_val = static_cast<int16_t>(std::clamp(std::round(rpm), -330.0, 330.0));
    data[2] = static_cast<uint8_t>((mapped_val >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(mapped_val & 0xFF);
    data[6] = acceleration;
    data[7] = brake ? 0xFF : 0x00;
    data[9] = calc_crc8_maxim(data.data(), 9);

    return write_packet(data);
  }

  bool DDSM115DriverClient::send_position_command(uint8_t motor_id, double position_degrees)
  {
    std::vector<uint8_t> data(10, 0x00);
    data[0] = motor_id;
    data[1] = 0x64;

    // Map position (0.0 to 360.0 degrees) to (-32767 to 32767) where 180 is 0.
    int16_t mapped_val = static_cast<int16_t>(std::clamp(std::round((position_degrees - 180.0) * (32767.0 / 180.0)), -32767.0, 32767.0));
    data[2] = static_cast<uint8_t>((mapped_val >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(mapped_val & 0xFF);
    data[9] = calc_crc8_maxim(data.data(), 9);

    return write_packet(data);
  }

  bool DDSM115DriverClient::send_feedback_query(uint8_t motor_id)
  {
    std::vector<uint8_t> data(10, 0x00);
    data[0] = motor_id;
    data[1] = 0x74;
    data[9] = calc_crc8_maxim(data.data(), 9);

    return write_packet(data);
  }

  bool DDSM115DriverClient::change_motor_id(uint8_t current_id, uint8_t new_id)
  {
    (void)current_id; // Motor ID setting command does not target a specific current ID, it must be run with ONLY ONE motor connected to the bus.
    std::vector<uint8_t> data(10, 0x00);
    data[0] = 0xAA;
    data[1] = 0x55;
    data[2] = 0x53;
    data[3] = new_id;
    data[9] = calc_crc8_maxim(data.data(), 9);

    bool success = true;
    for (int i = 0; i < 5; i++)
    {
      success &= write_packet(data);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return success;
  }

  void DDSM115DriverClient::register_motor_id(uint8_t motor_id)
  {
    std::lock_guard<std::mutex> lock(ids_mutex_);
    if (std::find(registered_motor_ids_.begin(), registered_motor_ids_.end(), motor_id) == registered_motor_ids_.end())
    {
      registered_motor_ids_.push_back(motor_id);
    }
  }

  void DDSM115DriverClient::unregister_motor_id(uint8_t motor_id)
  {
    std::lock_guard<std::mutex> lock(ids_mutex_);
    auto it = std::find(registered_motor_ids_.begin(), registered_motor_ids_.end(), motor_id);
    if (it != registered_motor_ids_.end())
    {
      registered_motor_ids_.erase(it);
    }
  }

  void DDSM115DriverClient::clear_registered_motor_ids()
  {
    std::lock_guard<std::mutex> lock(ids_mutex_);
    registered_motor_ids_.clear();
  }

  bool DDSM115DriverClient::is_valid_motor_id(uint8_t motor_id) const
  {
    // If no motor IDs are registered, accept any ID between 1 and 253.
    // If some are registered, only accept them.
    std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(ids_mutex_));
    if (registered_motor_ids_.empty())
    {
      return (motor_id >= 1 && motor_id <= 253);
    }
    return std::find(registered_motor_ids_.begin(), registered_motor_ids_.end(), motor_id) != registered_motor_ids_.end();
  }

  void DDSM115DriverClient::start_async_read()
  {
    if (!reading_)
      return;
    serial_port_.async_read_some(
        boost::asio::buffer(read_buf_), [this](boost::system::error_code ec, std::size_t length)
        {
          if (!ec && length > 0)
          {
            {
              std::lock_guard<std::mutex> lock(buffer_mutex_);
              buffer_.insert(buffer_.end(), read_buf_.begin(), read_buf_.begin() + length);
              parse_buffer();
            }
          }
          else if (ec != boost::asio::error::operation_aborted)
          {
            std::stringstream ss;
            ss << "Serial read error: " << ec.message();
            log(LogLevel::DEBUG, ss.str());
          }
          if (reading_)
          {
            start_async_read();
          }
        });
  }

  void DDSM115DriverClient::parse_buffer()
  {
    while (buffer_.size() >= 10)
    {
      size_t pos = 0;
      bool found = false;
      for (; pos <= buffer_.size() - 10; ++pos)
      {
        uint8_t motor_id = buffer_[pos];
        if (is_valid_motor_id(motor_id))
        {
          uint8_t crc_calculated = calc_crc8_maxim(buffer_.data() + pos, 9);
          uint8_t crc_received = buffer_[pos + 9];
          if (crc_calculated == crc_received)
          {
            found = true;
            break;
          }
        }
      }

      if (pos > 0)
      {
        buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
      }

      if (!found || buffer_.size() < 10)
      {
        break;
      }

      std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + 10);
      process_feedback_packet(packet);
      buffer_.erase(buffer_.begin(), buffer_.begin() + 10);
    }
  }

  void DDSM115DriverClient::process_feedback_packet(const std::vector<uint8_t> &packet)
  {
    MotorFeedback feedback;
    feedback.motor_id = packet[0];
    feedback.mode = static_cast<ControlLoopModes>(packet[1]);
    
    // Parse current
    int16_t raw_current = static_cast<int16_t>((packet[2] << 8) | packet[3]);
    feedback.current = static_cast<double>(raw_current) * (8.0 / 32767.0);

    // Parse velocity
    int16_t raw_velocity = static_cast<int16_t>((packet[4] << 8) | packet[5]);
    feedback.velocity = static_cast<double>(raw_velocity);

    // Parse single-turn position (mapped 0 to 32767 -> 0.0 to 360.0 degrees)
    uint16_t raw_position = static_cast<uint16_t>((packet[6] << 8) | packet[7]);
    feedback.position = static_cast<double>(raw_position) * (360.0 / 32767.0);

    // Parse error codes
    feedback.error_code = packet[8];
    feedback.over_temperature = (feedback.error_code & 0x01) != 0;
    feedback.voltage_fault     = (feedback.error_code & 0x02) != 0;
    feedback.over_current     = (feedback.error_code & 0x04) != 0;
    feedback.sensor_fault     = (feedback.error_code & 0x08) != 0;
    feedback.stalling         = (feedback.error_code & 0x10) != 0;
    feedback.init_fault         = (feedback.error_code & 0x20) != 0;

    std::stringstream ss;
    ss << "Feedback parsed for motor " << static_cast<int>(feedback.motor_id)
       << ": Mode=" << static_cast<int>(feedback.mode)
       << ", I=" << feedback.current << " A"
       << ", V=" << feedback.velocity << " RPM"
       << ", P=" << feedback.position << " Deg"
       << ", Err=" << static_cast<int>(feedback.error_code);
    log(LogLevel::DEBUG, ss.str());

    if (feedback_callback_)
    {
      feedback_callback_(feedback);
    }
  }

} // namespace ddsm115_ros2_driver
