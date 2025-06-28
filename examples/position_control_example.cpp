#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <iomanip>
#include <cmath>
#include <boost/asio.hpp>
#include "ddsm115_driver/ddsm115_driver.hpp"

int main() {
    try {
        boost::asio::io_context io_context;
        
        // Create motor driver instance
        ddsm115::DDSM115Driver motor(io_context, "/dev/ttyUSB0", 1);
        
        // Set callbacks
        motor.set_error_callback([](const std::string& error) {
            std::cerr << "Motor Error: " << error << std::endl;
        });
        
        motor.set_status_callback([](const ddsm115::MotorStatus& status) {
            // Convert position to degrees
            double position_deg = (static_cast<double>(status.position) * 360.0) / 32767.0;
            std::cout << "Position: " << std::fixed << std::setprecision(1) 
                      << position_deg << "°, "
                      << "U8 Position: " << static_cast<int>(status.position_u8) << ", "
                      << "Velocity: " << status.velocity << " RPM" << std::endl;
        });
        
        // Initialize the driver
        if (!motor.initialize()) {
            std::cerr << "Failed to initialize motor driver!" << std::endl;
            return -1;
        }
        
        // Run io_context in a separate thread
        std::thread io_thread([&io_context]() {
            io_context.run();
        });
        
        // Switch to velocity mode first, then to position mode (as required)
        motor.set_mode(ddsm115::MotorMode::VELOCITY_LOOP);
        motor.set_velocity(0);  // Ensure motor is stopped
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Switch to position mode
        motor.set_mode(ddsm115::MotorMode::POSITION_LOOP);
        motor.set_auto_polling(true, 100);  // Poll every 100ms
        
        std::cout << "Starting position control demo..." << std::endl;
        
        // Define target positions (in degrees)
        std::vector<double> target_positions = {
            0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0, 360.0,
            315.0, 270.0, 225.0, 180.0, 135.0, 90.0, 45.0, 0.0
        };
        
        for (double target_pos : target_positions) {
            std::cout << "Moving to " << target_pos << " degrees..." << std::endl;
            motor.set_position(target_pos);
            
            // Wait for movement to complete (or timeout)
            auto start_time = std::chrono::steady_clock::now();
            const auto timeout = std::chrono::seconds(5);
            
            while (std::chrono::steady_clock::now() - start_time < timeout) {
                auto status = motor.get_status();
                double current_pos = (static_cast<double>(status.position) * 360.0) / 32767.0;
                
                // Check if close to target (within 5 degrees)
                if (std::abs(current_pos - target_pos) < 5.0) {
                    std::cout << "Reached target position!" << std::endl;
                    break;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // Hold position for a moment
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
        std::cout << "Position control demo completed!" << std::endl;
        
        // Test continuous rotation
        std::cout << "Testing continuous rotation..." << std::endl;
        
        for (int rotation = 0; rotation < 3; ++rotation) {
            for (double angle = 0; angle <= 360; angle += 30) {
                motor.set_position(angle);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        
        // Return to home position
        std::cout << "Returning to home position (0°)..." << std::endl;
        motor.set_position(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Shutdown
        motor.shutdown();
        io_context.stop();
        io_thread.join();
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
