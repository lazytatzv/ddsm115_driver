# ddsm115_ros2_driver

[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-orange.svg)](https://docs.ros.org/en/jazzy/index.html)
[![Linux](https://img.shields.io/badge/Linux-Ubuntu-green.svg)](https://ubuntu.com)
[![CMake](https://img.shields.io/badge/CMake-Build-blue.svg)](https://cmake.org)
[![Docs](https://img.shields.io/badge/Docs-Sphinx-brightgreen.svg)](https://sphinx-doc.org)
[![CI](https://github.com/lazytatzv/ddsm115_driver/actions/workflows/ci.yml/badge.svg)](https://github.com/lazytatzv/ddsm115_driver/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

C++20 BNO055-style serial library and ROS 2 package for Waveshare DDSM115 direct-drive servo motors on Linux.

---

## Features

* **Standalone C++20 library**: Link natively via CMake without ROS dependencies.
* **Asynchronous Communication**: Implements non-blocking, multi-threaded serial reads/writes using `Boost.ASIO`.
* **ros2_control Integration**: Implements the `hardware_interface::SystemInterface` plugin for ROS 2 Jazzy, compatible with standard controller broadcasers and diff-drive plugins.
* **Collision-free RS485 Polling**: Implements a staggered polling loop to prevent packet collisions when managing multiple motors on a single RS485 bus.
* **Continuous Joint Angle Tracking**: Translates the absolute 0–360° wraps into continuous accumulated rotation angles in radians to drive precise robot odometry.
* **Live Winding Temperature Monitoring**: Periodically queries temperature registers using `0x74` packets and updates ROS 2 diagnostics.
* **Safety Watchdog Timer**: Automatically stops the motors (0 RPM safety brake) if command messages stop arriving on the control topics.
* **Strict CRC-8/MAXIM Validation**: Verified against official Waveshare communication specifications.

---

## Quick Start

### A. Standalone C++ (No-ROS)

1. **Build and Install**:

   ```bash
   sudo apt update && sudo apt install -y build-essential cmake libboost-system-dev
   git clone https://github.com/lazytatzv/ddsm115_driver.git
   cd ddsm115_driver && mkdir build && cd build
   cmake .. -DBUILD_TESTING=OFF
   make -j$(nproc) && sudo make install
   ```

2. **Write your code (`main.cpp`)**:

   ```cpp
   #include <ddsm115_ros2_driver/ddsm115_driver_client.hpp>
   #include <iostream>
   #include <thread>
   #include <chrono>

   int main() {
       // Define status feedback callback
       auto feedback_cb = [](const ddsm115_ros2_driver::MotorFeedback& fb) {
           if (fb.is_temperature_packet) {
               std::cout << "Motor ID " << (int)fb.motor_id 
                         << " | Temperature: " << fb.winding_temperature << " C\n";
           } else {
               std::cout << "Motor ID " << (int)fb.motor_id 
                         << " | Pos: " << fb.position 
                         << " Deg | Vel: " << fb.velocity 
                         << " RPM | Current: " << fb.current << " A\n";
           }
       };

       auto log_cb = [](ddsm115_ros2_driver::LogLevel level, const std::string& msg) {
           std::cout << "[DDSM115 LOG] " << msg << "\n";
       };

       ddsm115_ros2_driver::DDSM115DriverClient client(feedback_cb, log_cb);
       if (!client.open_port("/dev/ttyUSB0", 115200)) {
           std::cerr << "Failed to open serial port\n";
           return 1;
       }

       // Register motor IDs on the bus
       client.register_motor_id(1);

       // Switch to velocity mode
       client.send_mode_command(1, ddsm115_ros2_driver::ControlLoopModes::MODE_VELOCITY);
       std::this_thread::sleep_for(std::chrono::milliseconds(10));

       // Command 50 RPM
       client.send_velocity_command(1, 50.0, 10, false);

       for (int i = 0; i < 20; ++i) {
           std::this_thread::sleep_for(std::chrono::milliseconds(100));
           if (i % 10 == 0) {
               client.send_feedback_query(1); // request winding temperature
           }
       }

       // Brake and close
       client.send_velocity_command(1, 0.0, 0, true);
       client.close_port();
       return 0;
   }
   ```

3. **Compile and Run**:

   ```bash
   g++ -std=c++20 main.cpp -lddsm115_driver_client -lboost_system -lpthread -o motor_demo
   ./motor_demo
   ```

### B. ROS 2 (colcon workspace)

1. **Clone and Build**:

   ```bash
   # Clone inside your ROS 2 workspace src directory
   cd ~/ros2_ws/src
   git clone https://github.com/lazytatzv/ddsm115_driver.git ddsm115_ros2_driver

   # Build
   cd ~/ros2_ws
   colcon build --packages-select ddsm115_ros2_driver
   source install/setup.bash
   ```

2. **Launch Node**:

   ```bash
   ros2 launch ddsm115_ros2_driver ddsm115_node.launch.py
   ```

---

## ROS 2 API Reference

### Published Topics

| Topic Name | Message Type | Description |
|---|---|---|
| `~/motor_[ID]/status` | `ddsm115_ros2_driver/msg/Ddsm115Status` | Telemetry status feedback (current, velocity, position, winding temperature, error code). |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Standard ROS 2 diagnostics stream containing motor errors, over-temperature faults, and frame stats. |

### Subscribed Topics

| Topic Name | Message Type | Description |
|---|---|---|
| `~/motor_[ID]/command` | `ddsm115_ros2_driver/msg/Ddsm115Command` | Controls individual motors (mode: 1=CURRENT, 2=VELOCITY, 3=POSITION, target value, and brake instructions). |

### Core Parameters

| Parameter | Default | Description |
|---|---|---|
| `serial_port` | `"/dev/ttyUSB0"` | Path to the USB-to-RS485 interface device node. |
| `baud_rate` | `115200` | Baudrate for the RS485 serial communication. |
| `motor_ids` | `[1, 2]` | List of active motor IDs connected to the RS485 bus. |
| `publish_rate` | `20.0` | Target control loop frequency in Hz. |
| `command_timeout` | `1.0` | Watchdog timeout limit in seconds. |

---

## Detailed Project Documentation

For advanced setup and detailed guides, see the files located in the `docs` directory:

* `docs/index.rst`: Documentation home page.
* `docs/installation.rst`: Building and package dependencies.
* `docs/usage.rst`: Hardware wiring, configuration parameters, and control modes.

---

## Hardware Configuration (Prerequisites)

### 1. Wiring (DDSM115 to RS485 Transceiver)

| ZH1.5 4P Pin | DDSM115 Signal | RS485 Transceiver Pin | Description |
|---|---|---|---|
| Pin 1 | GND | GND | Signal Ground (Highly recommended to reduce bus noise) |
| Pin 2 | A | DATA+ (A+) | 485 Differential A |
| Pin 3 | B | DATA- (B-) | 485 Differential B |

*   **Power Cable (XH2.54 2P)**:
    *   Pin 1 (Red) ➔ VCC (12V–24V DC, 18V recommended)
    *   Pin 2 (Black) ➔ GND

### 2. Permissions

Add your Linux user to the `dialout` group to access serial ports without using sudo:

```bash
sudo usermod -aG dialout $USER
```

*Note: You must log out and log back in for changes to take effect.*

---

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
