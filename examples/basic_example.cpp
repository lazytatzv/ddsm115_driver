#include <iostream>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include "ddsm115_driver/ddsm115_driver.hpp"

int main() {
    try {
        boost::asio::io_context io_context;
        
        // Create motor driver instance
        ddsm115::DDSM115Driver motor(io_context, "/dev/ttyUSB0", 1);
        
        // Set error callback
        motor.set_error_callback([](const std::string& error) {
            std::cerr << "Motor Error: " << error << std::endl;
        });
        
        // Set status callback
        motor.set_status_callback([](const ddsm115::MotorStatus& status) {
            std::cout << "Motor Status - ID: " << static_cast<int>(status.id)
                      << ", Velocity: " << status.velocity << " RPM"
                      << ", Position: " << status.position
                      << ", Temperature: " << static_cast<int>(status.temperature) << "°C"
                      << ", Error Code: 0x" << std::hex << static_cast<int>(status.error_code)
                      << std::dec << std::endl;
        });
        
        // Initialize the driver
        if (!motor.initialize()) {
            std::cerr << "Failed to initialize motor driver!" << std::endl;
            return -1;
        }
        
        std::cout << "Motor driver initialized successfully!" << std::endl;
        
        // Start auto polling for status updates
        motor.set_auto_polling(true, 100);  // Poll every 100ms
        
        // Run io_context in a separate thread
        std::thread io_thread([&io_context]() {
            io_context.run();
        });
        
        // Test basic functionality
        std::cout << "Testing motor functionality..." << std::endl;
        
        // Set velocity mode and test different speeds
        motor.set_mode(ddsm115::MotorMode::VELOCITY_LOOP);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "Setting velocity to 50 RPM..." << std::endl;
        motor.set_velocity(50);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "Setting velocity to -30 RPM..." << std::endl;
        motor.set_velocity(-30);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "Stopping motor..." << std::endl;
        motor.set_velocity(0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Test position mode
        std::cout << "Switching to position mode..." << std::endl;
        motor.set_mode(ddsm115::MotorMode::POSITION_LOOP);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "Moving to 90 degrees..." << std::endl;
        motor.set_position(90.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "Moving to 180 degrees..." << std::endl;
        motor.set_position(180.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "Moving back to 0 degrees..." << std::endl;
        motor.set_position(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "Test completed!" << std::endl;
        
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
