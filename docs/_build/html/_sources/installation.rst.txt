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

Installation
============

Requirements
------------

* ROS 2 Jazzy (or newer)
* Boost C++ Libraries (Boost.ASIO)
* Python 3 with ``pyserial`` library

Building from Source
--------------------

1. Clone or copy this repository into the ``src`` folder of your ROS 2 workspace (e.g. ``main_ws/src/drivers/actuators/ddsm115_ros2_driver``).

2. Build the package using colcon:

   .. code-block:: bash

      colcon build --symlink-install --packages-select ddsm115_ros2_driver

3. Source the workspace setup script:

   .. code-block:: bash

      source install/setup.bash
