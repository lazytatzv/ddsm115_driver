#include "ddsm115_driver/ddsm115_driver.hpp"
#include <iostream>
#include <iomanip>
#include <thread>

namespace ddsm115 {

DDSM115Driver::DDSM115Driver(boost::asio::io_context& io_context, 
                             const std::string& device_path, 
                             uint8_t motor_id)
    : io_context_(io_context)
    , serial_port_(nullptr)
    , device_path_(device_path)
    , motor_id_(motor_id)
    , current_mode_(MotorMode::VELOCITY_LOOP)
    , last_status_{}
    , auto_polling_enabled_(false)
    , polling_interval_ms_(50)
    , connected_(false)
{
    last_status_.id = motor_id_;
    last_status_.mode = current_mode_;
}

DDSM115Driver::~DDSM115Driver() {
    shutdown();
}

bool DDSM115Driver::initialize() {
    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_context_, device_path_);
        
        // Configure serial port
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(BAUDRATE));
        serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        
        connected_ = true;
        
        // Start async receive
        start_async_receive();
        
        // Set default mode to velocity loop
        if (!set_mode(MotorMode::VELOCITY_LOOP)) {
            call_error_callback("Failed to set initial motor mode");
        }
        
        return true;
    } catch (const std::exception& e) {
        call_error_callback("Failed to initialize serial port: " + std::string(e.what()));
        return false;
    }
}

void DDSM115Driver::shutdown() {
    connected_ = false;
    auto_polling_enabled_ = false;
    
    if (polling_timer_) {
        polling_timer_->cancel();
        polling_timer_.reset();
    }
    
    if (serial_port_ && serial_port_->is_open()) {
        try {
            serial_port_->close();
        } catch (const std::exception& e) {
            call_error_callback("Error closing serial port: " + std::string(e.what()));
        }
    }
    serial_port_.reset();
}

bool DDSM115Driver::is_connected() const {
    return connected_ && serial_port_ && serial_port_->is_open();
}

bool DDSM115Driver::set_mode(MotorMode mode) {
    if (!is_connected()) {
        call_error_callback("Not connected to motor");
        return false;
    }
    
    std::array<uint8_t, PACKET_SIZE> packet = {0};
    packet[0] = motor_id_;
    packet[1] = CMD_MODE;
    // packet[2] to packet[8] are already 0
    packet[9] = static_cast<uint8_t>(mode);
    
    if (send_packet(packet)) {
        current_mode_ = mode;
        return true;
    }
    
    return false;
}

bool DDSM115Driver::send_command(const MotorCommand& command) {
    if (!is_connected()) {
        call_error_callback("Not connected to motor");
        return false;
    }
    
    std::array<uint8_t, PACKET_SIZE> packet = {0};
    packet[0] = motor_id_;
    packet[1] = CMD_DRIVE;
    packet[2] = static_cast<uint8_t>((command.value >> 8) & 0xFF);  // High byte
    packet[3] = static_cast<uint8_t>(command.value & 0xFF);         // Low byte
    packet[4] = 0;
    packet[5] = 0;
    packet[6] = command.acceleration_time;
    packet[7] = command.brake ? 0xFF : 0x00;
    packet[8] = 0;
    
    // Calculate CRC8
    std::array<uint8_t, PACKET_SIZE - 1> data_for_crc;
    std::copy(packet.begin(), packet.begin() + PACKET_SIZE - 1, data_for_crc.begin());
    packet[9] = calculate_crc8(data_for_crc);
    
    return send_packet(packet);
}

bool DDSM115Driver::set_current(int16_t current_ma) {
    if (current_mode_ != MotorMode::CURRENT_LOOP) {
        call_error_callback("Motor not in current loop mode");
        return false;
    }
    
    // Convert mA to internal units (-32767 to 32767 corresponds to -8A to 8A)
    int16_t internal_value = static_cast<int16_t>((current_ma * 32767) / 8000);
    
    return send_command(MotorCommand(internal_value));
}

bool DDSM115Driver::set_velocity(int16_t velocity_rpm, uint8_t acceleration_time, bool brake) {
    if (current_mode_ != MotorMode::VELOCITY_LOOP) {
        call_error_callback("Motor not in velocity loop mode");
        return false;
    }
    
    // Clamp velocity to valid range
    velocity_rpm = std::max(static_cast<int16_t>(-330), std::min(static_cast<int16_t>(330), velocity_rpm));
    
    return send_command(MotorCommand(velocity_rpm, acceleration_time, brake));
}

bool DDSM115Driver::set_position(double position_deg) {
    if (current_mode_ != MotorMode::POSITION_LOOP) {
        call_error_callback("Motor not in position loop mode");
        return false;
    }
    
    // Convert degrees to internal units (0-32767 corresponds to 0-360°)
    uint16_t internal_value = static_cast<uint16_t>((position_deg * 32767) / 360.0);
    
    return send_command(MotorCommand(internal_value));
}

bool DDSM115Driver::request_status() {
    if (!is_connected()) {
        call_error_callback("Not connected to motor");
        return false;
    }
    
    std::array<uint8_t, PACKET_SIZE> packet = {0};
    packet[0] = motor_id_;
    packet[1] = CMD_STATUS;
    // packet[2] to packet[8] are already 0
    
    // Calculate CRC8
    std::array<uint8_t, PACKET_SIZE - 1> data_for_crc;
    std::copy(packet.begin(), packet.begin() + PACKET_SIZE - 1, data_for_crc.begin());
    packet[9] = calculate_crc8(data_for_crc);
    
    return send_packet(packet);
}

MotorStatus DDSM115Driver::get_status() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return last_status_;
}

bool DDSM115Driver::set_motor_id(uint8_t new_id) {
    if (!is_connected()) {
        call_error_callback("Not connected to motor");
        return false;
    }
    
    std::array<uint8_t, PACKET_SIZE> packet = {0};
    packet[0] = 0xAA;
    packet[1] = CMD_SET_ID;
    packet[2] = 0x53;
    packet[3] = new_id;
    // packet[4] to packet[9] are already 0
    
    // Send the command 5 times as required by the protocol
    bool success = true;
    for (int i = 0; i < 5; ++i) {
        if (!send_packet(packet)) {
            success = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    if (success) {
        motor_id_ = new_id;
    }
    
    return success;
}

uint8_t DDSM115Driver::query_motor_id() {
    if (!is_connected()) {
        call_error_callback("Not connected to motor");
        return 0;
    }
    
    std::array<uint8_t, PACKET_SIZE> packet = {0};
    packet[0] = 0xC8;
    packet[1] = CMD_QUERY_ID;
    // packet[2] to packet[8] are already 0
    
    // Calculate CRC8
    std::array<uint8_t, PACKET_SIZE - 1> data_for_crc;
    std::copy(packet.begin(), packet.begin() + PACKET_SIZE - 1, data_for_crc.begin());
    packet[9] = calculate_crc8(data_for_crc);
    
    if (send_packet(packet)) {
        std::array<uint8_t, PACKET_SIZE> response;
        if (receive_packet(response)) {
            return response[0];  // Motor ID is in the first byte of response
        }
    }
    
    return 0;
}

void DDSM115Driver::set_auto_polling(bool enable, uint32_t interval_ms) {
    auto_polling_enabled_ = enable;
    polling_interval_ms_ = interval_ms;
    
    if (enable && is_connected()) {
        if (!polling_timer_) {
            polling_timer_ = std::make_unique<boost::asio::steady_timer>(io_context_);
        }
        polling_timer_->expires_after(std::chrono::milliseconds(interval_ms));
        polling_timer_->async_wait([this](const boost::system::error_code& error) {
            polling_timer_handler(error);
        });
    } else if (polling_timer_) {
        polling_timer_->cancel();
    }
}

void DDSM115Driver::set_status_callback(const StatusCallback& callback) {
    status_callback_ = callback;
}

void DDSM115Driver::set_error_callback(const ErrorCallback& callback) {
    error_callback_ = callback;
}

bool DDSM115Driver::send_packet(const std::array<uint8_t, PACKET_SIZE>& packet) {
    if (!is_connected()) {
        return false;
    }
    
    try {
        boost::asio::write(*serial_port_, boost::asio::buffer(packet));
        return true;
    } catch (const std::exception& e) {
        call_error_callback("Failed to send packet: " + std::string(e.what()));
        return false;
    }
}

bool DDSM115Driver::receive_packet(std::array<uint8_t, PACKET_SIZE>& packet, 
                                   std::chrono::milliseconds timeout) {
    if (!is_connected()) {
        return false;
    }
    
    try {
        boost::asio::steady_timer timer(io_context_);
        timer.expires_after(timeout);
        
        bool data_received = false;
        boost::system::error_code timer_error;
        boost::system::error_code read_error;
        
        timer.async_wait([&](const boost::system::error_code& error) {
            timer_error = error;
            if (!data_received) {
                serial_port_->cancel();
            }
        });
        
        boost::asio::async_read(*serial_port_, boost::asio::buffer(packet),
            [&](const boost::system::error_code& error, size_t /*bytes_transferred*/) {
                read_error = error;
                data_received = true;
                timer.cancel();
            });
        
        io_context_.run_one();
        
        return data_received && !read_error;
    } catch (const std::exception& e) {
        call_error_callback("Failed to receive packet: " + std::string(e.what()));
        return false;
    }
}

uint8_t DDSM115Driver::calculate_crc8(const std::array<uint8_t, PACKET_SIZE - 1>& data) {
    // CRC-8/MAXIM algorithm
    uint8_t crc = 0x00;
    const uint8_t polynomial = 0x31;  // x^8 + x^5 + x^4 + 1
    
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

void DDSM115Driver::parse_status_packet(const std::array<uint8_t, PACKET_SIZE>& packet) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    last_status_.id = packet[0];
    last_status_.mode = static_cast<MotorMode>(packet[1]);
    
    // Parse torque current (signed 16-bit)
    last_status_.torque_current = static_cast<int16_t>((packet[2] << 8) | packet[3]);
    
    // Parse velocity (signed 16-bit)
    last_status_.velocity = static_cast<int16_t>((packet[4] << 8) | packet[5]);
    
    // Parse position (16-bit)
    last_status_.position = static_cast<int16_t>((packet[6] << 8) | packet[7]);
    
    // For status request (CMD_STATUS), packet[6] is temperature, packet[7] is U8 position
    if (packet[1] == CMD_STATUS) {
        last_status_.temperature = packet[6];
        last_status_.position_u8 = packet[7];
    }
    
    last_status_.error_code = packet[8];
    
    // Call status callback if set
    if (status_callback_) {
        status_callback_(last_status_);
    }
}

void DDSM115Driver::start_async_receive() {
    if (!is_connected()) {
        return;
    }
    
    boost::asio::async_read(*serial_port_, boost::asio::buffer(rx_buffer_),
        [this](const boost::system::error_code& error, size_t bytes_transferred) {
            handle_receive(error, bytes_transferred);
        });
}

void DDSM115Driver::handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
    if (error) {
        if (error != boost::asio::error::operation_aborted) {
            call_error_callback("Receive error: " + error.message());
        }
        return;
    }
    
    if (bytes_transferred == PACKET_SIZE) {
        // Verify CRC
        std::array<uint8_t, PACKET_SIZE - 1> data_for_crc;
        std::copy(rx_buffer_.begin(), rx_buffer_.begin() + PACKET_SIZE - 1, data_for_crc.begin());
        uint8_t calculated_crc = calculate_crc8(data_for_crc);
        
        if (calculated_crc == rx_buffer_[PACKET_SIZE - 1]) {
            parse_status_packet(rx_buffer_);
        } else {
            call_error_callback("CRC mismatch in received packet");
        }
    }
    
    // Continue receiving
    start_async_receive();
}

void DDSM115Driver::polling_timer_handler(const boost::system::error_code& error) {
    if (error || !auto_polling_enabled_ || !is_connected()) {
        return;
    }
    
    request_status();
    
    // Schedule next poll
    polling_timer_->expires_after(std::chrono::milliseconds(polling_interval_ms_));
    polling_timer_->async_wait([this](const boost::system::error_code& error) {
        polling_timer_handler(error);
    });
}

void DDSM115Driver::call_error_callback(const std::string& error_msg) {
    if (error_callback_) {
        error_callback_(error_msg);
    }
}

} // namespace ddsm115
