#pragma once

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include <mutex>

namespace ddsm115 {

/**
 * @brief Motor operating modes
 */
enum class MotorMode : uint8_t {
    CURRENT_LOOP = 0x01,
    VELOCITY_LOOP = 0x02,
    POSITION_LOOP = 0x03
};

/**
 * @brief Motor status structure containing feedback data
 */
struct MotorStatus {
    uint8_t id;
    MotorMode mode;
    int16_t torque_current;      // Torque current in mA
    int16_t velocity;            // Velocity in RPM
    int16_t position;            // Position (0-32767 for 0-360°)
    uint8_t temperature;         // Winding temperature in °C
    uint8_t position_u8;         // U8 position value (0-255 for 0-360°)
    uint8_t error_code;          // Error code bitfield
    
    // Error code bit interpretations
    bool sensor_error() const { return error_code & 0x01; }
    bool overcurrent_error() const { return error_code & 0x02; }
    bool phase_overcurrent_error() const { return error_code & 0x04; }
    bool stall_error() const { return error_code & 0x08; }
    bool troubleshooting() const { return error_code & 0x10; }
};

/**
 * @brief Command parameters for motor control
 */
struct MotorCommand {
    int16_t value;               // Target value (current/velocity/position)
    uint8_t acceleration_time;   // Acceleration time (0.1ms per 1rpm)
    bool brake;                  // Brake flag (only for velocity mode)
    
    MotorCommand(int16_t val = 0, uint8_t accel = 1, bool brk = false)
        : value(val), acceleration_time(accel), brake(brk) {}
};

/**
 * @brief DDSM115 Motor Driver Class
 * 
 * This class provides a high-level interface for controlling DDSM115 servo motors
 * via RS485 communication using boost::asio for serial communication.
 */
class DDSM115Driver {
public:
    using StatusCallback = std::function<void(const MotorStatus&)>;
    using ErrorCallback = std::function<void(const std::string&)>;

    /**
     * @brief Constructor
     * @param io_context Boost ASIO io_context for async operations
     * @param device_path Serial device path (e.g., "/dev/ttyUSB0")
     * @param motor_id Motor ID (1-255)
     */
    DDSM115Driver(boost::asio::io_context& io_context, 
                  const std::string& device_path, 
                  uint8_t motor_id = 1);

    /**
     * @brief Destructor
     */
    ~DDSM115Driver();

    /**
     * @brief Initialize the driver and open serial connection
     * @return true if successful, false otherwise
     */
    bool initialize();

    /**
     * @brief Close the serial connection and cleanup
     */
    void shutdown();

    /**
     * @brief Check if the driver is connected
     * @return true if connected, false otherwise
     */
    bool is_connected() const;

    /**
     * @brief Set motor operating mode
     * @param mode Target operating mode
     * @return true if successful, false otherwise
     */
    bool set_mode(MotorMode mode);

    /**
     * @brief Get current motor mode
     * @return Current motor mode
     */
    MotorMode get_mode() const { return current_mode_; }

    /**
     * @brief Send command to motor
     * @param command Motor command parameters
     * @return true if successful, false otherwise
     */
    bool send_command(const MotorCommand& command);

    /**
     * @brief Set target current (current loop mode)
     * @param current_ma Target current in milliamps (-8000 to 8000)
     * @return true if successful, false otherwise
     */
    bool set_current(int16_t current_ma);

    /**
     * @brief Set target velocity (velocity loop mode)
     * @param velocity_rpm Target velocity in RPM (-330 to 330)
     * @param acceleration_time Acceleration time per 1rpm (0.1ms units)
     * @param brake Apply brake flag
     * @return true if successful, false otherwise
     */
    bool set_velocity(int16_t velocity_rpm, uint8_t acceleration_time = 1, bool brake = false);

    /**
     * @brief Set target position (position loop mode)
     * @param position_deg Target position in degrees (0-360)
     * @return true if successful, false otherwise
     */
    bool set_position(double position_deg);

    /**
     * @brief Request motor status
     * @return true if request sent successfully, false otherwise
     */
    bool request_status();

    /**
     * @brief Get last received motor status
     * @return Motor status structure
     */
    MotorStatus get_status() const;

    /**
     * @brief Set motor ID (must be done with only one motor on bus)
     * @param new_id New motor ID (1-255)
     * @return true if successful, false otherwise
     */
    bool set_motor_id(uint8_t new_id);

    /**
     * @brief Query motor ID (must be done with only one motor on bus)
     * @return Motor ID, or 0 if failed
     */
    uint8_t query_motor_id();

    /**
     * @brief Enable/disable automatic status polling
     * @param enable Enable automatic polling
     * @param interval_ms Polling interval in milliseconds
     */
    void set_auto_polling(bool enable, uint32_t interval_ms = 50);

    /**
     * @brief Set status callback function
     * @param callback Callback function to be called when status is received
     */
    void set_status_callback(const StatusCallback& callback);

    /**
     * @brief Set error callback function
     * @param callback Callback function to be called when errors occur
     */
    void set_error_callback(const ErrorCallback& callback);

private:
    // Communication protocol constants
    static constexpr size_t PACKET_SIZE = 10;
    static constexpr uint32_t BAUDRATE = 115200;
    static constexpr uint8_t CMD_DRIVE = 0x64;
    static constexpr uint8_t CMD_STATUS = 0x74;
    static constexpr uint8_t CMD_MODE = 0xA0;
    static constexpr uint8_t CMD_SET_ID = 0x55;
    static constexpr uint8_t CMD_QUERY_ID = 0x64;

    // Internal methods
    bool send_packet(const std::array<uint8_t, PACKET_SIZE>& packet);
    bool receive_packet(std::array<uint8_t, PACKET_SIZE>& packet, 
                       std::chrono::milliseconds timeout = std::chrono::milliseconds(100));
    uint8_t calculate_crc8(const std::array<uint8_t, PACKET_SIZE - 1>& data);
    void parse_status_packet(const std::array<uint8_t, PACKET_SIZE>& packet);
    void start_async_receive();
    void handle_receive(const boost::system::error_code& error, size_t bytes_transferred);
    void polling_timer_handler(const boost::system::error_code& error);
    void call_error_callback(const std::string& error_msg);

    // Member variables
    boost::asio::io_context& io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    std::string device_path_;
    uint8_t motor_id_;
    MotorMode current_mode_;
    MotorStatus last_status_;
    
    // Async communication
    std::array<uint8_t, PACKET_SIZE> rx_buffer_;
    mutable std::mutex status_mutex_;
    
    // Callbacks
    StatusCallback status_callback_;
    ErrorCallback error_callback_;
    
    // Auto polling
    bool auto_polling_enabled_;
    uint32_t polling_interval_ms_;
    std::unique_ptr<boost::asio::steady_timer> polling_timer_;
    
    // Connection state
    bool connected_;
};

} // namespace ddsm115
