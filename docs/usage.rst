.. Copyright 2026 Tatsukiyano
..
.. Licensed under the Apache License, Version 2.0 (the "License");
.. you may not use this file except in compliance with the License.
.. You may obtain a copy of the License at
..
..     http://www.apache.org/licenses/LICENSE-2.0
..
.. Unless required by applicable law or agreed to in writing, software
.. distributed under the License is distributed on an "AS IS" BASIS,
.. WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
.. See the License for the specific language governing permissions and
.. limitations under the License.

Usage
=====

Hardware Connection
-------------------

1. Connect the motor signal wires to your USB-to-RS485 transceiver:
   * DDSM115 ``DATA+`` (Yellow) ➔ RS485 ``A+``
   * DDSM115 ``DATA-`` (Blue) ➔ RS485 ``B-``
   * DDSM115 ``GND`` (Black) ➔ RS485 ``GND`` (highly recommended to connect signal grounds to reduce serial noise)
2. Connect a DC power supply (rated 12V to 24V, rated 18V DC is standard, 6A minimum recommended).

.. note::
   **Using the Waveshare DDSM Driver HAT (A)**
   
   If you are using the official Waveshare DDSM Driver HAT (A), you do not need a separate USB-to-RS485 converter. 
   Simply connect the motor directly to the onboard motor interfaces and connect the HAT to your PC via the USB Type-C cable. 
   
   **CRITICAL STEP**: The DDSM Driver HAT (A) has an onboard ESP32 that defaults to expecting JSON commands. To use this ROS 2 driver (which relies on raw, fast RS485 packets), you must **bypass the ESP32**. Make sure the onboard "Serial Port Control Switch" is toggled **AWAY** from the "ESP32" position and into the direct USB/RS485 bypass mode. The driver will then seamlessly communicate with the motors via ``/dev/ttyUSB0``.

Motor ID Configuration
----------------------

When daisy-chaining multiple motors on the same RS485 bus, each motor must be configured with a unique ID (1 to 253).

.. important::
   When assigning a motor ID, ensure only one motor is connected to the RS485 bus. Running the command with multiple connected motors will write the same ID to all of them.

.. code-block:: bash

   # Example: Write ID "2" to the motor connected to /dev/ttyUSB0
   ros2 run ddsm115_ros2_driver set_motor_id.py --ros-args -p serial_port:=/dev/ttyUSB0 -p id:=2

   # Once successfully written, power cycle (reboot) the motor to apply the changes.

Standalone ROS 2 Node Mode
--------------------------

Used to quickly control and monitor motors via raw topic communication.

1. Launch the Node:

   .. code-block:: bash

      ros2 launch ddsm115_ros2_driver ddsm115_node.launch.py

2. Published Topics:
   * ``/motor_[ID]/status`` (ddsm115_ros2_driver/msg/Ddsm115Status): Measured winding current, velocity, position, winding temperature, and raw diagnostic error flags.
   * ``/diagnostics`` (diagnostic_msgs/msg/DiagnosticArray): Aggregated system health data containing bus statistics, watchdog status, and temperature faults.

3. Subscribed Topics:
   * ``/motor_[ID]/command`` (ddsm115_ros2_driver/msg/Ddsm115Command): Target command (mode: 1 for CURRENT, 2 for VELOCITY, 3 for POSITION, value, brake_mode).

ros2_control Hardware Interface Mode
------------------------------------

Recommended mode for mobile robots (e.g. differential drive) integrating with ``diff_drive_controller`` and Nav2.

1. URDF (xacro) Configuration:

   Add the hardware interface definition to your robot description:

   .. code-block:: xml

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

2. Running the Controller Demo:

   .. code-block:: bash

      ros2 launch ddsm115_ros2_driver ddsm115_control.launch.py motor_id_left:=1 motor_id_right:=2
