# Changelog

All notable changes to the DDSM115 Driver Library will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-06-29

### Added
- Initial release of DDSM115 servo motor driver library
- Complete C++ API for motor control via RS485 communication
- Support for all three control modes:
  - Current loop control (±8A range)
  - Velocity loop control (±330 RPM range)
  - Position loop control (0-360° range)
- Asynchronous communication using boost::asio
- CRC-8/MAXIM error checking for reliable communication
- Multi-motor support on single RS485 bus
- Comprehensive error handling and status monitoring
- Callback-based architecture for real-time updates
- ROS2 wrapper node for robotics applications
- Complete set of usage examples:
  - Basic motor control
  - Advanced velocity control with profiles
  - Precise position control
  - Multi-motor coordination
- CMake build system with installation support
- Comprehensive documentation and quick reference
- Launch files for single and multi-motor ROS2 setups

### Features
- **Hardware Communication**:
  - RS485 serial communication at 115200 baud
  - 10-byte packet structure with CRC verification
  - Command/response architecture up to 500Hz
  - Automatic status polling with configurable intervals

- **Motor Control**:
  - Three control modes with seamless switching
  - Real-time feedback of position, velocity, current, temperature
  - Hardware protection monitoring (overcurrent, overtemp, stall)
  - Brake control in velocity mode
  - Acceleration profiles for smooth motion

- **Software Architecture**:
  - Modern C++17 implementation
  - RAII resource management
  - Thread-safe operations
  - Exception safety
  - Extensive error handling

- **ROS2 Integration**:
  - Standard ROS2 messages (JointState, Float64)
  - Parameter-based configuration
  - Namespace support for multiple robots
  - Launch file templates

### Documentation
- Complete README with installation and usage instructions
- API reference documentation
- Quick reference guide
- Hardware setup and wiring diagrams
- Troubleshooting guide
- Multiple working examples

### Dependencies
- CMake >= 3.10
- Boost >= 1.65 (system, asio)
- C++17 compatible compiler
- ROS2 (optional, for ROS2 wrapper)
