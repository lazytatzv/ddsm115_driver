# DDSM115 Driver Quick Reference

## Build & Install
```bash
cd ddsm115_driver
./build.sh          # Build library and examples
sudo make install   # Install system-wide (optional)
```

## Basic Usage (C++)
```cpp
#include "ddsm115_driver/ddsm115_driver.hpp"

boost::asio::io_context io_context;
ddsm115::DDSM115Driver motor(io_context, "/dev/ttyUSB0", 1);

motor.initialize();
motor.set_mode(ddsm115::MotorMode::VELOCITY_LOOP);
motor.set_velocity(100);  // 100 RPM
```

## ROS2 Usage
```bash
# Single motor
ros2 launch ddsm115_ros2 ddsm115_single_motor.launch.py

# Send commands
ros2 topic pub /ddsm115_motor/velocity_command std_msgs/msg/Float64 "data: 50.0"

# Monitor status
ros2 topic echo /ddsm115_motor/status
```

## Control Modes
- **Current**: `set_current(mA)` - Range: ±8000 mA
- **Velocity**: `set_velocity(rpm)` - Range: ±330 RPM
- **Position**: `set_position(degrees)` - Range: 0-360°

## Motor Specifications
- Rated torque: 0.96 Nm
- Max torque: 2.0 Nm
- Rated speed: 115 RPM
- Max speed: 200 RPM
- Voltage: 18V DC (12-24V range)
- Current: 1.5A rated, 2.7A max
- Communication: RS485, 115200 baud

## Pin Connections
**Signal (ZH1.5*4P):**
1. GND (Signal Ground)
2. A (RS485 DATA+)
3. B (RS485 DATA-)
4. Reserved

**Power (XH2.54*2P):**
1. VCC (18V DC)
2. GND (Power Ground)

## Error Codes
- 0x01: Sensor error
- 0x02: Overcurrent
- 0x04: Phase overcurrent  
- 0x08: Stall error
- 0x10: Troubleshooting

## Common Commands
```bash
# Build library
./build.sh

# Run examples
./build/examples/basic_example
./build/examples/velocity_control_example
./build/examples/position_control_example
./build/examples/multi_motor_example

# ROS2 multi-motor
ros2 launch ddsm115_ros2 ddsm115_four_wheel_drive.launch.py
```

## Troubleshooting
1. Check RS485 wiring (A+/A, B-/B)
2. Verify 18V power supply
3. Confirm motor ID settings
4. Check serial port permissions: `sudo chmod 666 /dev/ttyUSB0`
5. Monitor error codes in status feedback
