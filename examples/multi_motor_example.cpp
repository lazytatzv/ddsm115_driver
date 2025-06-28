#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <memory>
#include <boost/asio.hpp>
#include "ddsm115_driver/ddsm115_driver.hpp"

int main() {
    try {
        boost::asio::io_context io_context;
        
        // Create multiple motor driver instances
        std::vector<std::unique_ptr<ddsm115::DDSM115Driver>> motors;
        
        // Assuming 4 motors with IDs 1, 2, 3, 4 on the same RS485 bus
        for (uint8_t id = 1; id <= 4; ++id) {
            auto motor = std::make_unique<ddsm115::DDSM115Driver>(io_context, "/dev/ttyUSB0", id);
            
            // Set error callback for each motor
            motor->set_error_callback([id](const std::string& error) {
                std::cerr << "Motor " << static_cast<int>(id) << " Error: " << error << std::endl;
            });
            
            // Set status callback for each motor
            motor->set_status_callback([id](const ddsm115::MotorStatus& status) {
                std::cout << "Motor " << static_cast<int>(id) 
                          << " - Velocity: " << status.velocity << " RPM, "
                          << "Position: " << status.position << ", "
                          << "Current: " << status.torque_current << " mA" << std::endl;
            });
            
            // Initialize each motor
            if (!motor->initialize()) {
                std::cerr << "Failed to initialize motor " << static_cast<int>(id) << "!" << std::endl;
                return -1;
            }
            
            motors.push_back(std::move(motor));
        }
        
        std::cout << "All motors initialized successfully!" << std::endl;
        
        // Run io_context in a separate thread
        std::thread io_thread([&io_context]() {
            io_context.run();
        });
        
        // Set all motors to velocity mode
        for (auto& motor : motors) {
            motor->set_mode(ddsm115::MotorMode::VELOCITY_LOOP);
            motor->set_auto_polling(true, 200);  // Stagger polling to avoid conflicts
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "Starting coordinated movement demo..." << std::endl;
        
        // Demo 1: Synchronized forward movement
        std::cout << "Demo 1: All motors forward at 50 RPM..." << std::endl;
        for (auto& motor : motors) {
            motor->set_velocity(50);
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Demo 2: Tank steering (left motors reverse, right motors forward)
        std::cout << "Demo 2: Tank steering (turning right)..." << std::endl;
        motors[0]->set_velocity(-30);  // Left front
        motors[1]->set_velocity(30);   // Right front
        motors[2]->set_velocity(-30);  // Left rear
        motors[3]->set_velocity(30);   // Right rear
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Demo 3: Diagonal movement
        std::cout << "Demo 3: Diagonal movement..." << std::endl;
        motors[0]->set_velocity(40);   // Left front
        motors[1]->set_velocity(20);   // Right front
        motors[2]->set_velocity(20);   // Left rear
        motors[3]->set_velocity(40);   // Right rear
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Demo 4: Stop all motors
        std::cout << "Demo 4: Emergency stop..." << std::endl;
        for (auto& motor : motors) {
            motor->set_velocity(0, 1, true);  // Quick stop with brake
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Demo 5: Position mode coordination
        std::cout << "Demo 5: Switching to position mode for precise alignment..." << std::endl;
        
        // Switch all motors to position mode
        for (auto& motor : motors) {
            motor->set_mode(ddsm115::MotorMode::POSITION_LOOP);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        // Align all wheels to 0 degrees
        std::cout << "Aligning all wheels to 0 degrees..." << std::endl;
        for (auto& motor : motors) {
            motor->set_position(0.0);
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Rotate all wheels to 90 degrees
        std::cout << "Rotating all wheels to 90 degrees..." << std::endl;
        for (auto& motor : motors) {
            motor->set_position(90.0);
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Return to home position
        std::cout << "Returning all wheels to home position..." << std::endl;
        for (auto& motor : motors) {
            motor->set_position(0.0);
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "Multi-motor demo completed!" << std::endl;
        
        // Status summary
        std::cout << "\nFinal status summary:" << std::endl;
        for (size_t i = 0; i < motors.size(); ++i) {
            auto status = motors[i]->get_status();
            std::cout << "Motor " << (i + 1) << ": ";
            
            if (status.sensor_error()) std::cout << "SENSOR_ERROR ";
            if (status.overcurrent_error()) std::cout << "OVERCURRENT ";
            if (status.phase_overcurrent_error()) std::cout << "PHASE_OVERCURRENT ";
            if (status.stall_error()) std::cout << "STALL ";
            if (status.troubleshooting()) std::cout << "TROUBLESHOOTING ";
            
            if (status.error_code == 0) {
                std::cout << "OK";
            }
            
            std::cout << std::endl;
        }
        
        // Shutdown all motors
        for (auto& motor : motors) {
            motor->shutdown();
        }
        
        io_context.stop();
        io_thread.join();
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
