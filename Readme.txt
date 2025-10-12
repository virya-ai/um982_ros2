readme_text = """UM982 RTK-GNSS ROS 2 Package
--------------------------------

Author: Athar Ahmed <athar.a@virya.ai>
Organization: Virya Autonomous Technologies Pvt. Ltd.
Version: 1.0.0
License: Proprietary

Overview:
This ROS 2 package interfaces with the UM982 RTK-GNSS receiver. It manages serial
communication, NTRIP RTK corrections, and publishes GNSS data to ROS 2 topics.

Main Features:
- Serial communication with UM982 via USB
- NTRIP client for RTK corrections (GGA upload and RTCM reception)
- Parses NMEA and UM982-specific messages
- Publishes GNSS data as ROS 2 topics:
  /gnss/fix       (sensor_msgs/NavSatFix)
  /gnss/velocity  (geometry_msgs/TwistStamped)
  /gnss/odom      (nav_msgs/Odometry)
  /gnss/status    (diagnostic_msgs/DiagnosticArray)
- Supports TF transforms and datum zeroing

Package Contents:
- src/um982_node.cpp        : Main ROS 2 node handling serial I/O and publishing data
- scripts/SetupUm982.py     : Automates UM982 configuration (baud, modes, messages)
- scripts/um982_reader.py   : Reads and prints UM982 messages for debugging
- scripts/um982_writer.py   : Sends manual commands to UM982
- scripts/um982_rtk_test.py : Tests full NTRIP-RTK data flow
- launch/um982.launch.py    : Launch file (optional)
- config/um982_params.yaml  : Planned config file for serial and RTK parameters

Basic Usage:
1. Connect UM982 via USB and note serial port (e.g., /dev/ttyUSB0)
2. Run SetupUm982.py to configure the module
3. Verify data using um982_reader.py
4. Launch ROS 2 node: ros2 run um982_driver um982_node
5. (Optional) Test RTK link with um982_rtk_test.py

Notes:
- Set correct antenna separation in setup script
- Datum coordinates (Lat, Lon, Alt) should be precise
- All software is proprietary to Virya Autonomous Technologies Pvt. Ltd.
"""
