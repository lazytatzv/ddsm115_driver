#include <iostream>
#include <thread>
#include <chrono>
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
            std::cout << "Velocity: " << status.velocity << " RPM, "
                      << "Current: " << status.torque_current << " mA, "
                      << "Temp: " << static_cast<int>(status.temperature) << "°C" << std::endl;
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
        
        // Set velocity mode
        motor.set_mode(ddsm115::MotorMode::VELOCITY_LOOP);
        motor.set_auto_polling(true, 50);  // Poll every 50ms
        
        std::cout << "Starting velocity control demo..." << std::endl;
        
        // Sine wave velocity control
        const double duration = 20.0;  // 20 seconds
        const double frequency = 0.2;  // 0.2 Hz
        const double max_velocity = 100.0;  // RPM
        
        auto start_time = std::chrono::steady_clock::now();
        
        while (true) {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            
            if (elapsed >= duration) {
                break;
            }
            
            // Generate sine wave velocity
            double target_velocity = max_velocity * std::sin(2.0 * M_PI * frequency * elapsed);
            
            // Set velocity with smooth acceleration
            motor.set_velocity(static_cast<int16_t>(target_velocity), 5);  // 5 * 0.1ms = 0.5ms per RPM
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Ramp down to stop
        std::cout << "Ramping down to stop..." << std::endl;
        for (int velocity = 100; velocity >= 0; velocity -= 10) {
            motor.set_velocity(velocity, 10);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        
        motor.set_velocity(0);
        std::cout << "Demo completed!" << std::endl;
        
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
