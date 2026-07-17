# ddsm115_ros2_driver

A robust and high-performance ROS 2 driver package for the Waveshare DDSM115 direct-drive servo motor.

This driver provides low-latency, asynchronous serial communication to monitor and control DDSM115 motors over an RS485 bus. It integrates seamlessly with `ros2_control` and supports absolute position tracking, motor diagnostics, safety timeouts, and bus collision avoidance.

---

## 🌟 Key Features

1. **Pure C++ Asynchronous Client Library**:
   - Implements fully asynchronous, non-blocking serial communication using `Boost.ASIO`.
   - Utilizes a sliding buffer parser with strict **CRC-8/MAXIM** checksum validation to isolate frame alignments from bus noise.
   
2. **Full `ros2_control` Integration**:
   - Implements the `hardware_interface::SystemInterface` plugin compatible with ROS 2 Jazzy.
   - Links directly to standard ROS 2 controllers (e.g. `diff_drive_controller`, `joint_state_broadcaster`).
   - Non-blocking state writing run in a background thread to eliminate control loop latency.

3. **Collision-free RS485 Polling**:
   - Integrates a sequential polling algorithm that staggers motor read/write commands, completely preventing packet collisions on shared RS485 buses.

4. **Continuous Joint Position Tracking**:
   - Automatically tracks single-turn position wrap-arounds (jumps at 0/360 degrees boundary) to output continuous, absolute positions in radians. Crucial for reliable robot odometry.

5. **Safety Watchdog & Timeout**:
   - Features a configurable watchdog timer that automatically halts the motors (0 RPM safety stop) if commands stop publishing to the topics.

6. **Diagnostics Monitoring**:
   - Parses the motor driver's onboard diagnostics (OBD) status bits (winding temperature, overcurrent, over-temperature, voltage faults, sensor faults, stalling) and publishes them to the ROS 2 `/diagnostics` framework.

---

## 🛠️ Hardware Setup

1. **Wiring Connection**:
   - Connect the motor signal wires to your USB-to-RS485 transceiver:
     - DDSM115 `DATA+` (Yellow) ➔ RS485 `A+`
     - DDSM115 `DATA-` (Blue) ➔ RS485 `B-`
     - DDSM115 `GND` (Black) ➔ RS485 `GND` (highly recommended to connect signal grounds to reduce serial noise)
2. **Power Supply**:
   - Connect a DC power supply (rated 12V to 24V, rated 18V DC is standard, 6A minimum recommended).

---

## 🚀 Building and Installation

### 1. Workspace Clone
Place this package directory into the `src` folder of your ROS 2 workspace (e.g. `main_ws/src/drivers/actuators/ddsm115_ros2_driver`).

### 2. Build the Package
From the root of your workspace, run:
```bash
colcon build --symlink-install --packages-select ddsm115_ros2_driver
source install/setup.bash
```

---

## ⚙️ Motor ID Configuration Tool

When daisy-chaining multiple motors on the same RS485 bus, each motor must be configured with a unique ID (1 to 253).

> [!IMPORTANT]
> When assigning a motor ID, **ensure only one motor is connected** to the RS485 bus. Running the command with multiple connected motors will write the same ID to all of them.

```bash
# Example: Write ID "2" to the motor connected to /dev/ttyUSB0
ros2 run ddsm115_ros2_driver set_motor_id.py --ros-args -p serial_port:=/dev/ttyUSB0 -p id:=2

# Once successfully written, power cycle (reboot) the motor to apply the changes.
```

---

## ROS 2 Nodes and Control Interfaces

### Standalone ROS 2 Node Mode

Used to quickly control and monitor motors via raw topic communication.

#### 1. Launch the Node
```bash
ros2 launch ddsm115_ros2_driver ddsm115_node.launch.py
```

#### 2. Topic Specifications
Topics are dynamically generated for each registered motor ID.

*   **Subscribed Topics**
    *   `/motor_[ID]/command` ([ddsm115_ros2_driver/msg/Ddsm115Command])
        - `mode`: Control mode (1: CURRENT, 2: VELOCITY, 3: POSITION)
        - `value`: Target value (Amperes, RPM, or Degrees)
        - `brake_mode`: Brake instruction (0: RELEASE, 1: LOCK)
*   **Published Topics**
    *   `/motor_[ID]/status` ([ddsm115_ros2_driver/msg/Ddsm115Status])
        - `current`: Measured winding current (A)
        - `velocity`: Measured speed (RPM)
        - `position`: Measured single-turn position (Degrees: 0.0 to 360.0)
        - `winding_temperature`: Measured winding temperature (Celsius)
        - `error_code`: Raw diagnostic error flags
    *   `/diagnostics` ([diagnostic_msgs/msg/DiagnosticArray])
        - Aggregated system health data containing bus statistics, watchdog status, and temperature faults.

---

### `ros2_control` Hardware Interface Mode

Recommended mode for mobile robots (e.g. differential drive) integrating with `diff_drive_controller`, navigation (Nav2), and standard broadcasers.

#### 1. URDF (xacro) Configuration
Add the hardware interface definition to your robot description:

```xml
<ros2_control name="DDSM115HardwareInterface" type="system">
  <hardware>
    <plugin>ddsm115_ros2_driver/DDSM115SystemHardware</plugin>
    <param name="usb_path">/dev/ttyUSB0</param>
    <param name="serial_baud">115200</param>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="motor_id">1</param>
    <param name="invert_direction">false</param>
  </joint>
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="motor_id">2</param>
    <param name="invert_direction">true</param>
  </joint>
</ros2_control>
```

#### 2. Running the Controller Demo
Launch the integrated test environment, which builds a temporary robot description and spawns `controller_manager` with the hardware plugin and a differential drive controller:

```bash
ros2 launch ddsm115_ros2_driver ddsm115_control.launch.py motor_id_left:=1 motor_id_right:=2
```

---

## 📄 License
This package is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
