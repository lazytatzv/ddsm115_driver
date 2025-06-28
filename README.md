# DDSM115 Servo Motor Driver Library

[![CI/CD](https://github.com/your-username/ddsm115_driver/actions/workflows/ci.yml/badge.svg)](https://github.com/your-username/ddsm115_driver/actions/workflows/ci.yml)
[![Release](https://github.com/your-username/ddsm115_driver/actions/workflows/release.yml/badge.svg)](https://github.com/your-username/ddsm115_driver/actions/workflows/release.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.wikipedia.org/wiki/C%2B%2B17)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Iron-blue.svg)](https://docs.ros.org/)

A comprehensive C++ library for controlling DDSM115 servo motors via RS485 communication, with support for both standalone C++ applications and ROS2 integration.

## Features

- **Multi-mode Operation**: Current loop, velocity loop, and position loop control
- **RS485 Communication**: Robust serial communication using boost::asio
- **Asynchronous Operations**: Non-blocking I/O with callback support
- **CRC Verification**: Built-in CRC-8/MAXIM error checking
- **Multi-motor Support**: Control multiple motors on the same RS485 bus
- **ROS2 Integration**: Ready-to-use ROS2 wrapper node
- **Comprehensive Examples**: Multiple usage examples included

## Hardware Specifications

- **No-load speed**: 200±10 RPM
- **Rated speed**: 115 RPM
- **Rated torque**: 0.96 Nm
- **Locked-rotor torque**: 2.0 Nm
- **Rated voltage**: 18V DC (5S LiPo)
- **Voltage range**: 12-24V DC
- **Communication**: RS485 (115200 baud, 8N1)
- **Encoder resolution**: 4096
- **Protection level**: IP54

## Prerequisites

### System Dependencies
```bash
# Ubuntu/Debian
sudo apt update
sudo apt install build-essential cmake libboost-all-dev

# For ROS2 support
sudo apt install ros-humble-desktop-full
```

### Build Dependencies
- **CMake** >= 3.10
- **Boost** >= 1.65 (system, asio)
- **C++17** compatible compiler
- **ROS2** (optional, for ROS2 wrapper)

## Installation

### Quick Install (Recommended)

```bash
# Clone the repository
git clone https://github.com/your-username/ddsm115_driver.git
cd ddsm115_driver

# Build and install
./build.sh
sudo make install -C build
```

### Docker Installation

```bash
# For standalone library
docker pull ghcr.io/your-username/ddsm115_driver:latest

# For ROS2 integration
docker pull ghcr.io/your-username/ddsm115_driver:ros2-latest
```

### Standalone Library

```bash
# Clone the repository
git clone <repository-url>
cd ddsm115_driver

# Create build directory
mkdir build && cd build

# Configure and build
cmake ..
make -j$(nproc)

# Install system-wide (optional)
sudo make install
```

### ROS2 Workspace Integration

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone the repository
git clone <repository-url>

# Build the workspace
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

### Basic C++ Usage

```cpp
#include <boost/asio.hpp>
#include "ddsm115_driver/ddsm115_driver.hpp"

int main() {
    boost::asio::io_context io_context;
    ddsm115::DDSM115Driver motor(io_context, "/dev/ttyUSB0", 1);
    
    // Initialize
    if (!motor.initialize()) {
        std::cerr << "Failed to initialize motor!" << std::endl;
        return -1;
    }
    
    // Set velocity mode and control
    motor.set_mode(ddsm115::MotorMode::VELOCITY_LOOP);
    motor.set_velocity(100);  // 100 RPM
    
    // Run for 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // Stop motor
    motor.set_velocity(0);
    motor.shutdown();
    
    return 0;
}
```

### ROS2 Usage

#### Single Motor

```bash
# Launch single motor node
ros2 launch ddsm115_ros2 ddsm115_single_motor.launch.py device_path:=/dev/ttyUSB0 motor_id:=1

# Send velocity command
ros2 topic pub /ddsm115_motor/velocity_command std_msgs/msg/Float64 "data: 50.0"

# Monitor status
ros2 topic echo /ddsm115_motor/status
```

#### Four-Wheel Drive System

```bash
# Launch four motor nodes
ros2 launch ddsm115_ros2 ddsm115_four_wheel_drive.launch.py device_path:=/dev/ttyUSB0

# Control individual motors
ros2 topic pub /robot/motors/front_left_motor/velocity_command std_msgs/msg/Float64 "data: 50.0"
ros2 topic pub /robot/motors/front_right_motor/velocity_command std_msgs/msg/Float64 "data: 50.0"
```

## API Reference

### Core Classes

#### `DDSM115Driver`

Main driver class for motor control.

**Constructor**
```cpp
DDSM115Driver(boost::asio::io_context& io_context, 
              const std::string& device_path, 
              uint8_t motor_id = 1)
```

**Key Methods**
- `bool initialize()` - Initialize the driver
- `bool set_mode(MotorMode mode)` - Set control mode
- `bool set_velocity(int16_t rpm, uint8_t accel_time = 1, bool brake = false)` - Velocity control
- `bool set_position(double degrees)` - Position control
- `bool set_current(int16_t current_ma)` - Current control
- `MotorStatus get_status()` - Get motor status
- `void set_auto_polling(bool enable, uint32_t interval_ms = 50)` - Enable status polling

#### `MotorStatus`

Status structure containing motor feedback.

```cpp
struct MotorStatus {
    uint8_t id;
    MotorMode mode;
    int16_t torque_current;    // mA
    int16_t velocity;          // RPM
    int16_t position;          // 0-32767 for 0-360°
    uint8_t temperature;       // °C
    uint8_t error_code;        // Error bitfield
    
    // Error checking methods
    bool sensor_error() const;
    bool overcurrent_error() const;
    bool phase_overcurrent_error() const;
    bool stall_error() const;
    bool troubleshooting() const;
};
```

### Control Modes

#### Current Loop Mode
- Range: -8A to +8A
- Use: Direct torque control
- Command: `set_current(int16_t current_ma)`

#### Velocity Loop Mode
- Range: -330 RPM to +330 RPM
- Use: Speed control with acceleration profiles
- Command: `set_velocity(int16_t rpm, uint8_t accel_time, bool brake)`

#### Position Loop Mode
- Range: 0° to 360°
- Use: Precise positioning
- Command: `set_position(double degrees)`
- **Note**: Motor velocity must be < 10 RPM when switching to this mode

## Hardware Setup

### Wiring

**Signal Cable (ZH1.5*4P)**
| Pin | Signal | Description |
|-----|--------|-------------|
| 1   | GND    | Signal Ground |
| 2   | A      | RS485 DATA+ |
| 3   | B      | RS485 DATA- |
| 4   | -      | Reserved |

**Power Cable (XH2.54*2P)**
| Pin | Signal | Description |
|-----|--------|-------------|
| 1   | VCC    | 18V DC Power |
| 2   | GND    | Power Ground |

### RS485 Connection

Connect to USB-to-RS485 converter:
- Motor A (DATA+) → Converter A+
- Motor B (DATA-) → Converter B-
- Motor GND → Converter GND (recommended)

### Multi-Motor Setup

For multiple motors on the same bus:
1. Set unique motor IDs (1-255) using `set_motor_id()`
2. Stagger polling intervals to avoid conflicts
3. Ensure sufficient power supply capacity

## Examples

The library includes several examples:

- **basic_example.cpp** - Basic motor control demonstration
- **velocity_control_example.cpp** - Advanced velocity control with sine waves
- **position_control_example.cpp** - Precise position control
- **multi_motor_example.cpp** - Coordinated multi-motor control

Build and run examples:
```bash
cd build
make
./examples/basic_example
```

## Error Handling

The driver provides comprehensive error handling:

### Error Codes (Bitfield)
- `0x01` - Sensor error
- `0x02` - Overcurrent error
- `0x04` - Phase overcurrent error
- `0x08` - Stall error
- `0x10` - Troubleshooting flag

### Protection Features
- Bus overcurrent protection (3A threshold)
- Motor over-temperature protection (80°C threshold)
- Phase current protection (4.6A threshold)
- Locked rotor protection (5s timeout)

## Troubleshooting

### Common Issues

**Connection Problems**
- Check RS485 wiring (A+/A, B-/B)
- Verify baud rate (115200)
- Ensure proper grounding
- Check USB-to-RS485 converter driver

**Power Issues**
- Verify 18V DC power supply
- Check power cable connections
- Ensure adequate current capacity
- Monitor voltage under load

**Communication Errors**
- Verify motor ID settings
- Check for bus conflicts with multiple motors
- Reduce polling frequency if experiencing timeouts
- Ensure only one motor on bus when setting/querying IDs

**Motor Not Responding**
- Verify motor is powered and connected
- Check error codes in status feedback
- Try different baud rates or serial ports
- Reset motor by power cycling

### Debug Mode

Enable debug output by setting error callbacks:

```cpp
motor.set_error_callback([](const std::string& error) {
    std::cout << "Debug: " << error << std::endl;
});
```

## Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Quick Start for Contributors

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Make your changes and add tests
4. Run the test suite: `./build.sh test`
5. Submit a pull request

### Reporting Issues

- **Bug reports**: Use the [bug report template](.github/ISSUE_TEMPLATE/bug_report.yml)
- **Feature requests**: Use the [feature request template](.github/ISSUE_TEMPLATE/feature_request.yml)
- **Questions**: Use [GitHub Discussions](https://github.com/your-username/ddsm115_driver/discussions)

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Support

- **Documentation**: [README.md](README.md) and [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
- **Issues**: [GitHub Issues](https://github.com/your-username/ddsm115_driver/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-username/ddsm115_driver/discussions)
- **Examples**: See [examples/](examples/) directory

## Acknowledgments

- Waveshare for the DDSM115 motor hardware and documentation
- Boost.Asio community for excellent async I/O library
- ROS2 community for robotics integration standards

---

**⭐ If this library helped your project, please give it a star!**
