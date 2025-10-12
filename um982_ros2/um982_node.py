#!/usr/bin/env python3

"""
/*******************************************************************************************
 *  UM982 RTK-GNSS ROS 2 Node
 *  
 *  Overview:
 *  This ROS 2 node provides an interface for the UM982 RTK-GNSS receiver. 
 *  It handles serial communication with the receiver, manages RTK corrections 
 *  via NTRIP, and publishes GNSS data to ROS 2 topics for localization and navigation.
 *
 *  Functionality:
 *   1. Connects to the UM982 via a serial-to-USB interface.
 *      -> Configure the serial port and baud rate in the parameters below.
 *   2. Receives GGA messages from the UM982 and forwards them to the NTRIP caster.
 *      -> Ensure NTRIP caster credentials are correctly configured.
 *   3. Receives RTK correction data from the NTRIP caster and transmits it to the UM982.
 *   4. Parses both standard NMEA and UM982-specific extended messages.
 *   5. Uses PVTSLN messages for position data and GNHPR messages for orientation data.
 *   6. Publishes the following ROS 2 topics:
 *        - /gnss/fix       : sensor_msgs/NavSatFix
 *        - /gnss/velocity  : geometry_msgs/TwistStamped
 *        - /gnss/odom      : nav_msgs/Odometry
 *        - /gnss/status    : diagnostic_msgs/DiagnosticArray
 *   7. Publishes TF transforms for GNSS reference frames.
 *
 *  Datum Configuration:
 *   -> For datum settings, provide the reference LLA (Latitude, Longitude, Altitude)
 *      coordinates for zeroing.
 *      -> Ensure values are specified with 8 decimal places for maximum precision: ORIGIN_LAT and ORIGIN_LON.
 *      -> Ensure values are specified with 1 decimal place for maximum precision: ORIGIN_HGT.
 *
 *  Setup Instructions:
 *   -> Before running this node, refer to and execute **SetupUm982.py** located in Scripts to configure 
 *      the UM982 chipset (baud rate, output messages, modes, etc.) as required.
 *
 * 
 *  To Do:
 *  - Verify and correct TF publisher frame definitions.
 *  - Moving the configrations to a YAML file
 *
 *  Author: Athar Ahmed  <athar.a@virya.ai>
 *  Organization: Virya Autonomous Technologies Pvt. Ltd.
 *  Version: 1.0.0
 *  
 *  Note:
 *  This software is proprietary to Virya Autonomous Technologies Pvt. Ltd.
 *  It must not be shared, copied, or distributed without prior authorization.
 *******************************************************************************************/
"""
import sys
import socket
import base64
import threading
import serial
import time
import math
from pyproj import CRS, Transformer
from datetime import datetime
from typing import Optional, Tuple, List

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# ROS 2 message imports
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Point, Vector3, TransformStamped

import tf2_geometry_msgs
import numpy as np

# TF2 imports for coordinate frame broadcasting
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# ===== Configuration =====
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUDRATE = 115200
NTRIP_HOST = '103.206.29.4'
NTRIP_PORT = 2105
MOUNTPOINT = 'IFKP'
USERNAME = 'pranav.k'
PASSWORD = 'cors@2022'
GGA_INTERVAL = 1.0

# ===== LLA Origin for ENU Zeroing =====
ORIGIN_LAT = 12.81876837 # degrees - edit this for your reference point
ORIGIN_LON = 77.69417551 # degrees - edit this for your reference point
ORIGIN_HGT = 897.4       # meters  - edit this for your reference point

# ===== Publishing Rates =====
FIX_RATE = 10.0 # Hz - /gnss/fix, /gnss/velocity, /gnss/odom
STATUS_RATE = 1.0 # Hz - /gnss/status

# ===== GPS_utils Class (from LLAtoENU.py) =====
class GPS_utils:
    '''
    Contains the algorithms to convert a gps signal (longitude, latitude, height)
    to a local cartesian ENU system and vice versa
    Use setENUorigin(lat, lon, height) to set the local ENU coordinate system origin
    Use geo2enu(lat, lon, height) to get the position in the local ENU system
    Use enu2geo(x_enu, y_enu, z_enu) to get the latitude, longitude and height
    '''
    def __init__(self):
        # Geodetic System WGS 84 axes
        self.a = 6378137.0
        self.b = 6356752.314245
        self.a2 = self.a * self.a
        self.b2 = self.b * self.b
        self.e2 = 1.0 - (self.b2 / self.a2)
        self.e = self.e2 / (1.0 - self.e2)
        # Local ENU Origin
        self.latZero = None
        self.lonZero = None
        self.hgtZero = None
        self.xZero = None
        self.yZero = None
        self.zZero = None
        self.R = np.asmatrix(np.eye(3))

    def setENUorigin(self, lat, lon, height):
        # Save origin lat, lon, height
        self.latZero = lat
        self.lonZero = lon
        self.hgtZero = height
        # Get origin ECEF X,Y,Z
        origin = self.geo2ecef(self.latZero, self.lonZero, self.hgtZero)
        self.xZero = origin.item(0)
        self.yZero = origin.item(1)
        self.zZero = origin.item(2)
        self.oZero = np.array([[self.xZero], [self.yZero], [self.zZero]])
        # Build rotation matrix
        phi = np.deg2rad(self.latZero)
        lmd = np.deg2rad(self.lonZero)
        cPhi = np.cos(phi)
        cLmd = np.cos(lmd)
        sPhi = np.sin(phi)
        sLmd = np.sin(lmd)
        self.R[0, 0] = -sLmd
        self.R[0, 1] = cLmd
        self.R[0, 2] = 0.0
        self.R[1, 0] = -sPhi * cLmd
        self.R[1, 1] = -sPhi * sLmd
        self.R[1, 2] = cPhi
        self.R[2, 0] = cPhi * cLmd
        self.R[2, 1] = cPhi * sLmd
        self.R[2, 2] = sPhi

    def geo2ecef(self, lat, lon, height):
        phi = np.deg2rad(lat)
        lmd = np.deg2rad(lon)
        cPhi = np.cos(phi)
        cLmd = np.cos(lmd)
        sPhi = np.sin(phi)
        sLmd = np.sin(lmd)
        N = self.a / np.sqrt(1.0 - self.e2 * sPhi * sPhi)
        x = (N + height) * cPhi * cLmd
        y = (N + height) * cPhi * sLmd
        z = ((self.b2 / self.a2) * N + height) * sPhi
        return np.array([[x], [y], [z]])

    def ecef2enu(self, x, y, z):
        ecef = np.array([[x], [y], [z]])
        return self.R * (ecef - self.oZero)

    def geo2enu(self, lat, lon, height):
        ecef = self.geo2ecef(lat, lon, height)
        return self.ecef2enu(ecef.item(0), ecef.item(1), ecef.item(2))

    def ecef2geo(self, x, y, z):
        p = np.sqrt(x*x + y*y)
        q = np.arctan2(self.a * z, self.b * p)
        sq = np.sin(q)
        cq = np.cos(q)
        sq3 = sq * sq * sq
        cq3 = cq * cq * cq
        phi = np.arctan2(z + self.e * self.b * sq3, p - self.e2 * self.a * cq3)
        lmd = np.arctan2(y, x)
        v = self.a / np.sqrt(1.0 - self.e2 * np.sin(phi) * np.sin(phi))
        lat = np.rad2deg(phi)
        lon = np.rad2deg(lmd)
        h = (p / np.cos(phi)) - v
        return np.array([[lat], [lon], [h]])

    def enu2ecef(self, x, y, z):
        lmd = np.deg2rad(self.latZero)
        phi = np.deg2rad(self.lonZero)
        cPhi = np.cos(phi)
        cLmd = np.cos(lmd)
        sPhi = np.sin(phi)
        sLmd = np.sin(lmd)
        N = self.a / np.sqrt(1.0 - self.e2 * sLmd * sLmd)
        x0 = (self.hgtZero + N) * cLmd * cPhi
        y0 = (self.hgtZero + N) * cLmd * sPhi
        z0 = (self.hgtZero + (1.0 - self.e2) * N) * sLmd
        xd = -sPhi * x - cPhi * sLmd * y + cLmd * cPhi * z
        yd = cPhi * x - sPhi * sLmd * y + cLmd * sPhi * z
        zd = cLmd * y + sLmd * z
        return np.array([[x0+xd], [y0+yd], [z0+zd]])

    def enu2geo(self, x, y, z):
        ecef = self.enu2ecef(x, y, z)
        return self.ecef2geo(ecef.item(0), ecef.item(1), ecef.item(2))

# ==================================================================
# ===== CORE UM982 FUNCTIONS =====
# ==================================================================

def crc_table() -> List[int]:
    """Generate CRC32 lookup table for NMEA extended messages."""
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
        table.append(crc)
    return table

NMEA_EXTENDED_CRC_TABLE = crc_table()

def nmea_extended_crc(sentence: str) -> bool:
    """Verify CRC32 checksum for extended NMEA messages."""
    def calc_crc32(data: bytes) -> int:
        crc = 0
        for byte in data:
            crc = NMEA_EXTENDED_CRC_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFFFFFF

    try:
        msg, crc_str = sentence[1:].split('*', 1)
        crc_expected = crc_str[:8].lower()
    except (ValueError, IndexError):
        return False

    crc_calculated = calc_crc32(msg.encode())
    return crc_expected == format(crc_calculated, '08x')

def nmea_standard_crc(sentence: str) -> bool:
    """Verify XOR checksum for standard NMEA messages."""
    try:
        msg, crc_str = sentence[1:].split('*', 1)
        crc_expected = crc_str[:2].upper()
    except (ValueError, IndexError):
        return False

    crc_calculated = 0
    for char in msg:
        crc_calculated ^= ord(char)
    return format(crc_calculated, '02X') == crc_expected

def extract_message_fields(msg: str) -> List[str]:
    """Extract comma-separated fields from NMEA message."""
    return msg[1:msg.find('*')].split(',')

def safe_float_conversion(value: str) -> Optional[float]:
    """Safely convert string to float, returning None if conversion fails."""
    if not value or value in ['', '-']:
        return None
    try:
        return float(value)
    except (ValueError, TypeError):
        return None

def safe_int_conversion(value: str) -> Optional[int]:
    """Safely convert string to int, returning None if conversion fails."""
    if not value or value in ['', '-']:
        return None
    try:
        return int(value)
    except (ValueError, TypeError):
        return None

def parse_gga_for_satellites(gga_msg: str) -> Optional[int]:
    """Extract satellites in view from GGA message."""
    try:
        fields = extract_message_fields(gga_msg)
        if len(fields) > 7:
            return safe_int_conversion(fields[7]) # Field 7 contains satellites in view
    except:
        pass
    return None

def parse_fix_quality_from_gga(gga_msg: str) -> str:
    """Extract fix quality from GGA message and return descriptive status."""
    try:
        fields = extract_message_fields(gga_msg)
        if len(fields) > 6:
            quality = safe_int_conversion(fields[6])
            quality_map = {
                0: "INVALID",
                1: "GPS",
                2: "DGPS",
                3: "PPS",
                4: "RTK_FIX",
                5: "RTK_FLOAT",
                6: "EST",
                7: "MANUAL",
                8: "SIM"
            }
            return quality_map.get(quality, f"UNK_{quality}")
    except:
        pass
    return "NOFIX"

def pvtsln_parser_ultimate_fix(msg: str) -> Optional[Tuple[float, float, float, float, float, float, str]]:
    """Parse PVTSLN message for position data with standard deviations."""
    try:
        fields = extract_message_fields(msg)
        if len(fields) < 15:
            return None
        data_start_idx = 9 # Skip header fields
        if len(fields) < data_start_idx + 7:
            return None
        # Parse the actual PVTSLN data fields:
        bestpostype = fields[data_start_idx] # Field 1: Position type (STRING)
        height = safe_float_conversion(fields[data_start_idx + 1]) # Field 2: bestposhgt
        lat = safe_float_conversion(fields[data_start_idx + 2]) # Field 3: bestposlat
        lon = safe_float_conversion(fields[data_start_idx + 3]) # Field 4: bestposlon
        height_std = safe_float_conversion(fields[data_start_idx + 4]) # Field 5: bestposhgtstd
        lat_std = safe_float_conversion(fields[data_start_idx + 5]) # Field 6: bestposlatstd
        lon_std = safe_float_conversion(fields[data_start_idx + 6]) # Field 7: bestposlonstd
        # Validate we got valid numeric values
        if None in [height, lat, lon, height_std, lat_std, lon_std]:
            return None
        return (height, lat, lon, height_std, lat_std, lon_std, bestpostype)
    except (ValueError, IndexError):
        return None

def gnhpr_parser(msg: str) -> Optional[Tuple[float, float, float]]:
    """Parse $GNHPR message for orientation data."""
    try:
        fields = extract_message_fields(msg)
        if len(fields) < 5:
            return None
        # Skip field 0 (header) and field 1 (UTC), start from field 2
        heading = safe_float_conversion(fields[2]) # Heading 0-360
        pitch = safe_float_conversion(fields[3]) # Pitch -90 to 90
        roll = safe_float_conversion(fields[4]) # Roll -90 to 90
        if None in [heading, pitch, roll]:
            return None
        return (heading, pitch, roll)
    except (ValueError, IndexError):
        return None

def bestnav_parser(msg: str) -> Optional[Tuple[float, float, float, float, float, float]]:
    """Parse #BESTNAV message for velocity data."""
    try:
        fields = extract_message_fields(msg)
        if len(fields) < 5:
            return None
        # Extract velocity components from end of message (negative indexing)
        hor_speed = safe_float_conversion(fields[-5]) # Horizontal speed
        track_heading = safe_float_conversion(fields[-4]) # Track heading
        vert_speed = safe_float_conversion(fields[-3]) # Vertical speed
        vert_speed_std = safe_float_conversion(fields[-2]) # Vertical speed std
        hor_speed_std = safe_float_conversion(fields[-1]) # Horizontal speed std
        if None in [hor_speed, track_heading, vert_speed, vert_speed_std, hor_speed_std]:
            return None
        # Convert to north/east components
        heading_rad = math.radians(track_heading)
        vel_north = hor_speed * math.sin(heading_rad)
        vel_east = hor_speed * math.cos(heading_rad)
        return (vel_east, vel_north, vert_speed, hor_speed_std, hor_speed_std, vert_speed_std)
    except (ValueError, IndexError):
        return None

def euler_to_quaternion(heading: float, pitch: float, roll: float) -> Tuple[float, float, float, float]:
    """Convert Euler angles (degrees) to quaternion (x, y, z, w)."""
    # Convert to radians
    heading_rad = math.radians(heading)
    pitch_rad = math.radians(pitch)
    roll_rad = math.radians(roll)
    # Compute quaternion components
    cy = math.cos(heading_rad * 0.5)
    sy = math.sin(heading_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    return (x, y, z, w)

# ==================================================================
# ===== ENHANCED UM982 CLASSES =====
# ==================================================================

class UM982Reader(threading.Thread):
    """Enhanced UM982 serial reader with thread-safe data access."""
    def __init__(self, port: str, baudrate: int, ros_node=None):
        super().__init__(daemon=True)
        self.serial_port = serial.Serial(port, baudrate, timeout=0.5)
        self.running = True
        self.ros_node = ros_node # Reference to ROS node for logging
        # Data storage with thread locks
        self.lock = threading.Lock()
        self.latest_gga = None
        self.position_data = None # From PVTSLN
        self.orientation_data = None # From GNHPR
        self.velocity_data = None # From BESTNAV
        #timestamp tracking
        self.last_gga_sent_time = None
        self.last_correction_received_time = None
        self.last_correction_sent_time = None
        # fix status tracking
        self.fix_status = "NOFIX"
        self.pvtsln_fix_type = "UNKNOWN"
        # Satellites in view tracking
        self.satellites_in_view = 0
        # Debug counters
        self.pvtsln_received = 0
        self.pvtsln_parsed = 0

    def run(self):
        """Main thread loop for reading serial data."""
        if self.ros_node:
            self.ros_node.get_logger().info("UM982 serial reader started")
        while self.running:
            try:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                self._process_message(line)
            except Exception as e:
                if self.ros_node:
                    self.ros_node.get_logger().warn(f"Serial read error: {e}")
                time.sleep(0.1)
        if self.ros_node:
            self.ros_node.get_logger().info(f"UM982 reader stopped. PVTSLN stats: {self.pvtsln_parsed}/{self.pvtsln_received} parsed")

    def _process_message(self, line: str):
        """Process individual NMEA message"""
        with self.lock:
            # Standard NMEA GGA message for NTRIP
            if line.startswith('$') and 'GGA' in line and '*' in line:
                if nmea_standard_crc(line):
                    self.latest_gga = line
                    # Extract satellites in view from GGA
                    sats = parse_gga_for_satellites(line)
                    if sats is not None:
                        self.satellites_in_view = sats
                    # Extract fix quality from GGA for backup fix status
                    gga_fix = parse_fix_quality_from_gga(line)
                    if self.pvtsln_fix_type == "UNKNOWN":
                        self.fix_status = gga_fix
            # Extended NMEA PVTSLN message - position data with standard deviations
            elif line.startswith('#PVTSLN') and nmea_extended_crc(line):
                self.pvtsln_received += 1
                pos_data = pvtsln_parser_ultimate_fix(line)
                if pos_data:
                    self.pvtsln_parsed += 1
                    self.position_data = pos_data[:6] # Keep original format (height, lat, lon, std_devs)
                    self.pvtsln_fix_type = pos_data[6] # Store fix type separately
                    # Set fix status based on PVTSLN fix type
                    self.fix_status = self.pvtsln_fix_type
            # Standard NMEA GNHPR message for orientation
            elif line.startswith('$GNHPR') and nmea_standard_crc(line):
                self.orientation_data = gnhpr_parser(line)
            # Extended NMEA BESTNAV message for velocity
            elif line.startswith('#BESTNAV') and nmea_extended_crc(line):
                self.velocity_data = bestnav_parser(line)

    def get_latest_data(self):
        """Thread-safe method to get latest parsed data."""
        with self.lock:
            return {
                'position': self.position_data,
                'orientation': self.orientation_data,
                'velocity': self.velocity_data,
                'fix_status': self.fix_status,
                'satellites': self.satellites_in_view,
                'gga_sent_time': self.last_gga_sent_time,
                'correction_received_time': self.last_correction_received_time,
                'correction_sent_time': self.last_correction_sent_time,
                'pvtsln_stats': (self.pvtsln_parsed, self.pvtsln_received)
            }

    def stop(self):
        """Stop the reader thread and close serial port."""
        self.running = False
        time.sleep(0.2)
        if self.serial_port.is_open:
            self.serial_port.close()

class NTRIPClient:
    """NTRIP client for receiving RTK corrections with timestamp tracking."""
    def __init__(self, reader: UM982Reader):
        self.reader = reader
        self.socket = None
        self.running = False
        # Prepare HTTP request for NTRIP authentication
        auth_string = base64.b64encode(f"{USERNAME}:{PASSWORD}".encode()).decode()
        self.http_request = (
            f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
            f"User-Agent: NTRIP/1.0\r\n"
            f"Authorization: Basic {auth_string}\r\n\r\n"
        ).encode()

    def start(self):
        """Start NTRIP client in background thread."""
        self.running = True
        threading.Thread(target=self._ntrip_main_loop, daemon=True).start()

    def stop(self):
        """Stop NTRIP client."""
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass

    def _ntrip_main_loop(self):
        """Main NTRIP connection and data handling loop."""
        while self.running:
            try:
                if self.reader.ros_node:
                    self.reader.ros_node.get_logger().info(f"Connecting to NTRIP server {NTRIP_HOST}:{NTRIP_PORT}")
                self.socket = socket.create_connection((NTRIP_HOST, NTRIP_PORT), timeout=10)
                self.socket.sendall(self.http_request)
                self._skip_http_headers()
                if self.reader.ros_node:
                    self.reader.ros_node.get_logger().info(f"Connected to NTRIP mountpoint '{MOUNTPOINT}'")
                # Start GGA sender and RTCM receiver threads
                threading.Thread(target=self._send_gga_loop, daemon=True).start()
                threading.Thread(target=self._receive_rtcm_loop, daemon=True).start()
                # Keep connection alive
                while self.running:
                    time.sleep(1)
            except Exception as e:
                if self.reader.ros_node:
                    self.reader.ros_node.get_logger().error(f"NTRIP connection error: {e}")
                if self.socket:
                    try:
                        self.socket.close()
                    except:
                        pass
                if self.running:
                    time.sleep(5) # Retry after 5 seconds

    def _skip_http_headers(self):
        """Read and discard HTTP response headers."""
        header_buffer = b''
        while b'\r\n\r\n' not in header_buffer:
            chunk = self.socket.recv(1)
            if not chunk:
                raise ConnectionError("Failed to read HTTP headers")
            header_buffer += chunk

    def _send_gga_loop(self):
        """Send GGA messages to NTRIP server at regular intervals."""
        while self.running:
            try:
                with self.reader.lock:
                    gga_msg = self.reader.latest_gga
                    if gga_msg:
                        self.socket.sendall((gga_msg + "\r\n").encode('ascii'))
                        self.reader.last_gga_sent_time = datetime.now()
                    else:
                        self.reader.last_gga_sent_time = None
                time.sleep(GGA_INTERVAL)
            except Exception:
                break

    def _receive_rtcm_loop(self):
        """Receive RTCM corrections and forward to serial port."""
        while self.running:
            try:
                rtcm_data = self.socket.recv(4096)
                if not rtcm_data:
                    break
                self.reader.last_correction_received_time = datetime.now()
                try:
                    self.reader.serial_port.write(rtcm_data)
                    self.reader.last_correction_sent_time = datetime.now()
                except Exception:
                    self.reader.last_correction_sent_time = None
            except Exception:
                break

# ==================================================================
# ===== ROS 2 NODE IMPLEMENTATION =====
# ==================================================================

class UM982Node(Node):
    """ROS 2 node wrapper for UM982 GNSS receiver"""
    def __init__(self):
        super().__init__('um982_gnss_node')
        # Configure ROS logger for errors and warnings only
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        self.get_logger().info("Initializing UM982 GNSS Node...")

        # Setup TF2 broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Setup ENU coordinate transformer for custom origin
        self._setup_enu_transformer()

        # Setup static transforms
        self._setup_static_transforms()

        # Setup QoS profiles for different message types
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Create publishers
        self.fix_pub = self.create_publisher(NavSatFix, '/gnss/fix', sensor_qos)
        self.velocity_pub = self.create_publisher(TwistStamped, '/gnss/velocity', sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, '/gnss/odom', sensor_qos)
        self.status_pub = self.create_publisher(DiagnosticArray, '/gnss/status', 10)

        # Initialize hardware components
        try:
            self.reader = UM982Reader(SERIAL_PORT, SERIAL_BAUDRATE, ros_node=self)
            self.ntrip_client = NTRIPClient(self.reader)
            # Start hardware threads
            self.reader.start()
            self.ntrip_client.start()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize UM982 hardware: {e}")
            raise

        # Create callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()

        # Setup publishing timers
        self.fix_timer = self.create_timer(1.0/FIX_RATE, self.publish_fix_callback,
                                           callback_group=self.callback_group)
        self.velocity_timer = self.create_timer(1.0/FIX_RATE, self.publish_velocity_callback,
                                                callback_group=self.callback_group)
        self.odom_timer = self.create_timer(1.0/FIX_RATE, self.publish_odometry_callback,
                                            callback_group=self.callback_group)
        self.status_timer = self.create_timer(1.0/STATUS_RATE, self.publish_status_callback,
                                              callback_group=self.callback_group)

        self.get_logger().info("UM982 GNSS Node initialized successfully")

    def _setup_enu_transformer(self):
        """Setup coordinate transformers for ENU conversion with custom origin."""
        try:
            # Initialize GPS_utils for coordinate conversion
            self.gps_utils = GPS_utils()
            self.gps_utils.setENUorigin(ORIGIN_LAT, ORIGIN_LON, ORIGIN_HGT)
            # Transformer for LLA to ECEF
            self.lla_to_ecef = Transformer.from_crs(
                'EPSG:4979', # WGS84 lat/lon/height
                'EPSG:4978', # ECEF
                always_xy=True
            )
            # Compute origin in ECEF coordinates
            self.origin_ecef = self.lla_to_ecef.transform(ORIGIN_LON, ORIGIN_LAT, ORIGIN_HGT)
            self.get_logger().info(f"ENU origin set to: {ORIGIN_LAT:.8f}, {ORIGIN_LON:.8f}, {ORIGIN_HGT:.4f}")
        except Exception as e:
            self.get_logger().error(f"Failed to setup ENU transformer: {e}")
            raise

    def _setup_static_transforms(self):
        """Setup static coordinate frame transforms according to ROS2 standards."""
        try:
            # Static transform from map to odom (identity for GNSS-only setup)
            # In a full navigation stack, this would be provided by a localization node
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = self.get_clock().now().to_msg()
            map_to_odom.header.frame_id = 'map'
            map_to_odom.child_frame_id = 'odom'
            map_to_odom.transform.translation.x = 0.0
            map_to_odom.transform.translation.y = 0.0
            map_to_odom.transform.translation.z = 0.0
            map_to_odom.transform.rotation.x = 0.0
            map_to_odom.transform.rotation.y = 0.0
            map_to_odom.transform.rotation.z = 0.0
            map_to_odom.transform.rotation.w = 1.0
            self.static_tf_broadcaster.sendTransform(map_to_odom)

            # Static transform from base_link to gnss_link
            # Adjust these values according to your robot's GNSS sensor mounting position
            base_to_gnss = TransformStamped()
            base_to_gnss.header.stamp = self.get_clock().now().to_msg()
            base_to_gnss.header.frame_id = 'base_link'
            base_to_gnss.child_frame_id = 'gnss_link'
            # Example mounting: GNSS antenna 0.2m forward, 0.0m right, 0.8m up from base_link
            base_to_gnss.transform.translation.x = 0.2
            base_to_gnss.transform.translation.y = 0.0
            base_to_gnss.transform.translation.z = 0.8
            base_to_gnss.transform.rotation.x = 0.0
            base_to_gnss.transform.rotation.y = 0.0
            base_to_gnss.transform.rotation.z = 0.0
            base_to_gnss.transform.rotation.w = 1.0
            self.static_tf_broadcaster.sendTransform(base_to_gnss)

            self.get_logger().info("Static transforms configured")
        except Exception as e:
            self.get_logger().error(f"Failed to setup static transforms: {e}")
            raise

    def lla_to_enu(self, lon: float, lat: float, height: float) -> Tuple[float, float, float]:
        """Convert LLA to ENU coordinates using GPS_utils class."""
        try:
            # Use GPS_utils geo2enu method
            enu_result = self.gps_utils.geo2enu(lat, lon, height)
            # GPS_utils returns numpy array, extract values
            east = enu_result.item(0)
            north = enu_result.item(1)
            up = enu_result.item(2)
            return (east, north, up)
        except Exception as e:
            self.get_logger().warn(f"ENU conversion failed: {e}")
            return (0.0, 0.0, 0.0)

    def true_north_azimuth_to_enu_heading(self, true_north_azimuth: float) -> float:
        """Convert true north azimuth (0° = North, clockwise) to ENU heading (0° = East, counter-clockwise)."""
        # True north azimuth: 0° = North, 90° = East (clockwise from North)
        # ENU heading: 0° = East, 90° = North (counter-clockwise from East)
        # Conversion: ENU_heading = 90° - True_North_Azimuth
        enu_heading = - true_north_azimuth
        # Normalize to [0, 360) range
        enu_heading = enu_heading % 360.0
        return enu_heading

    def get_ros_timestamp(self) -> rclpy.time.Time:
        """Get current ROS timestamp."""
        return self.get_clock().now()

    def publish_dynamic_transforms(self, data):
        """Publish dynamic coordinate frame transforms."""
        try:
            pos_data = data['position']
            orient_data = data['orientation']
            if pos_data is None:
                return
            # Extract position data (height, lat, lon, std_devs)
            height, lat, lon, height_std, lat_std, lon_std = pos_data
            # Convert to ENU coordinates relative to origin
            enu_x, enu_y, enu_z = self.lla_to_enu(lon, lat, height)
            # Create dynamic transform from odom to base_link
            odom_to_base = TransformStamped()
            odom_to_base.header.stamp = self.get_ros_timestamp().to_msg()
            odom_to_base.header.frame_id = 'odom'
            odom_to_base.child_frame_id = 'base_link'
            # Set position in ENU coordinates
            odom_to_base.transform.translation.x = enu_x
            odom_to_base.transform.translation.y = enu_y
            odom_to_base.transform.translation.z = enu_z
            # Set orientation from GNHPR if available, otherwise identity
            if orient_data is not None:
                heading, pitch, roll = orient_data
                # Convert true north azimuth heading to ENU heading
                enu_heading = self.true_north_azimuth_to_enu_heading(heading)
                qx, qy, qz, qw = euler_to_quaternion(enu_heading, pitch, roll)
                odom_to_base.transform.rotation.x = qx
                odom_to_base.transform.rotation.y = qy
                odom_to_base.transform.rotation.z = qz
                odom_to_base.transform.rotation.w = qw
            else:
                # Identity quaternion (no rotation)
                odom_to_base.transform.rotation.x = 0.0
                odom_to_base.transform.rotation.y = 0.0
                odom_to_base.transform.rotation.z = 0.0
                odom_to_base.transform.rotation.w = 1.0
            # Broadcast the transform
            self.tf_broadcaster.sendTransform(odom_to_base)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish dynamic transforms: {e}")

    def publish_fix_callback(self):
        """Publish NavSatFix message at 10 Hz."""
        try:
            data = self.reader.get_latest_data()
            pos_data = data['position']
            if pos_data is None:
                return
            # Extract position data from PVTSLN (height, lat, lon, std_devs)
            height, lat, lon, height_std, lat_std, lon_std = pos_data
            # Create NavSatFix message
            fix_msg = NavSatFix()
            fix_msg.header.stamp = self.get_ros_timestamp().to_msg()
            fix_msg.header.frame_id = 'gnss_link' # ROS2 standard: sensor data in sensor frame
            # Fill position data
            fix_msg.latitude = lat
            fix_msg.longitude = lon
            fix_msg.altitude = height
            # Map fix status to NavSatStatus according to ROS2 standards
            fix_status = data['fix_status']
            if fix_status in ['RTK_FIX']:
                fix_msg.status.status = NavSatStatus.STATUS_FIX
            elif fix_status in ['RTK_FLOAT', 'DGPS']:
                fix_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            elif fix_status in ['GPS']:
                fix_msg.status.status = NavSatStatus.STATUS_FIX
            else:
                fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
            # Set service (GPS + GLONASS assumed)
            fix_msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
            # Map PVTSLN standard deviations to position covariance diagonal
            # Position covariance is a 3x3 matrix in row-major order [xx, xy, xz, yx, yy, yz, zx, zy, zz]
            fix_msg.position_covariance = [0.0] * 9
            fix_msg.position_covariance[0] = lat_std ** 2 # lat variance
            fix_msg.position_covariance[4] = lon_std ** 2 # lon variance
            fix_msg.position_covariance[8] = height_std ** 2 # height variance
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.fix_pub.publish(fix_msg)
            # Also publish transforms when we have valid position data
            self.publish_dynamic_transforms(data)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish NavSatFix: {e}")

    def publish_velocity_callback(self):
        """Publish TwistStamped message at 10 Hz."""
        try:
            data = self.reader.get_latest_data()
            vel_data = data['velocity']
            if vel_data is None:
                return
            # Extract velocity data from BESTNAV (vel_east, vel_north, vel_up, std_devs)
            vel_east, vel_north, vel_up = vel_data[:3]
            # Create TwistStamped message
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_ros_timestamp().to_msg()
            twist_msg.header.frame_id = 'base_link' # ROS2 standard: velocity in robot body frame
            # Fill linear velocity (ENU frame)
            twist_msg.twist.linear.x = vel_east
            twist_msg.twist.linear.y = vel_north
            twist_msg.twist.linear.z = vel_up
            # Angular velocity set to zero (not provided by GNSS)
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0
            self.velocity_pub.publish(twist_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish TwistStamped: {e}")

    def publish_odometry_callback(self):
        """Publish Odometry message."""
        try:
            data = self.reader.get_latest_data()
            pos_data = data['position']
            orient_data = data['orientation']
            vel_data = data['velocity']
            if pos_data is None:
                return
            # Extract position data (height, lat, lon, std_devs)
            height, lat, lon, height_std, lat_std, lon_std = pos_data
            # Convert to ENU coordinates relative to origin
            enu_x, enu_y, enu_z = self.lla_to_enu(lon, lat, height)
            # Create Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_ros_timestamp().to_msg()
            odom_msg.header.frame_id = 'odom' # ROS2 standard: odometry in odom frame
            odom_msg.child_frame_id = 'base_link' # ROS2 standard: robot body frame
            # Fill position (ENU coordinates)
            odom_msg.pose.pose.position.x = enu_x
            odom_msg.pose.pose.position.y = enu_y
            odom_msg.pose.pose.position.z = enu_z
            # Fill orientation from GNHPR if available, otherwise identity quaternion
            if orient_data is not None:
                heading, pitch, roll = orient_data
                # Convert true north azimuth heading to ENU heading before using in odometry
                enu_heading = self.true_north_azimuth_to_enu_heading(heading)
                qx, qy, qz, qw = euler_to_quaternion(enu_heading, pitch, roll)
                odom_msg.pose.pose.orientation.x = qx
                odom_msg.pose.pose.orientation.y = qy
                odom_msg.pose.pose.orientation.z = qz
                odom_msg.pose.pose.orientation.w = qw
            else:
                # Identity quaternion
                odom_msg.pose.pose.orientation.w = 1.0
            # Fill pose covariance - only position from PVTSLN standard deviations
            odom_msg.pose.covariance = [0.0] * 36 # 6x6 matrix
            odom_msg.pose.covariance[0] = lat_std ** 2 # x position variance
            odom_msg.pose.covariance[7] = lon_std ** 2 # y position variance
            odom_msg.pose.covariance[14] = height_std ** 2 # z position variance
            # Orientation covariances remain zero as requested
            # Fill twist (velocity) if available
            if vel_data is not None:
                vel_east, vel_north, vel_up = vel_data[:3]
                odom_msg.twist.twist.linear.x = vel_east
                odom_msg.twist.twist.linear.y = vel_north
                odom_msg.twist.twist.linear.z = vel_up
            # Twist covariance remains zero as requested
            odom_msg.twist.covariance = [0.0] * 36
            self.odom_pub.publish(odom_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish Odometry: {e}")

    def publish_status_callback(self):
        """Publish DiagnosticArray message at 1 Hz."""
        try:
            data = self.reader.get_latest_data()
            # Create DiagnosticArray message
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_ros_timestamp().to_msg()
            diag_array.header.frame_id = 'gnss_link' # ROS2 standard: diagnostic in sensor frame
            # GNSS Fix Status
            fix_status = DiagnosticStatus()
            fix_status.name = "GNSS Fix"
            fix_status.hardware_id = "UM982"
            fix_quality = data['fix_status']
            if fix_quality in ['RTK_FIX']:
                fix_status.level = DiagnosticStatus.OK
                fix_status.message = "RTK Fixed Solution"
            elif fix_quality in ['RTK_FLOAT']:
                fix_status.level = DiagnosticStatus.WARN
                fix_status.message = "RTK Float Solution"
            elif fix_quality in ['DGPS', 'GPS']:
                fix_status.level = DiagnosticStatus.WARN
                fix_status.message = f"{fix_quality} Solution"
            else:
                fix_status.level = DiagnosticStatus.ERROR
                fix_status.message = "No Fix"
            fix_status.values.append(KeyValue(key="fix_type", value=fix_quality))
            fix_status.values.append(KeyValue(key="satellites", value=str(data['satellites'])))
            # NTRIP Status
            ntrip_status = DiagnosticStatus()
            ntrip_status.name = "NTRIP Corrections"
            ntrip_status.hardware_id = "UM982"
            correction_time = data['correction_received_time']
            if correction_time is not None:
                age_seconds = (datetime.now() - correction_time).total_seconds()
                if age_seconds < 10.0:
                    ntrip_status.level = DiagnosticStatus.OK
                    ntrip_status.message = "Corrections Active"
                else:
                    ntrip_status.level = DiagnosticStatus.WARN
                    ntrip_status.message = f"Corrections Stale - please check connectivity({age_seconds:.1f}s)"
                ntrip_status.values.append(KeyValue(key="correction_age", value=f"{age_seconds:.1f}"))
            else:
                ntrip_status.level = DiagnosticStatus.ERROR
                ntrip_status.message = "No Corrections"
                ntrip_status.values.append(KeyValue(key="correction_age", value="N/A"))
            # Message Parsing Stats
            parsing_status = DiagnosticStatus()
            parsing_status.name = "Message Parsing"
            parsing_status.hardware_id = "UM982"
            parsed, received = data['pvtsln_stats']
            if received > 0:
                success_rate = (parsed / received) * 100
                if success_rate > 95.0:
                    parsing_status.level = DiagnosticStatus.OK
                    parsing_status.message = "Parsing Nominal"
                elif success_rate > 80.0:
                    parsing_status.level = DiagnosticStatus.WARN
                    parsing_status.message = "Parsing Degraded"
                else:
                    parsing_status.level = DiagnosticStatus.ERROR
                    parsing_status.message = "Parsing Failed"
                parsing_status.values.append(KeyValue(key="success_rate", value=f"{success_rate:.1f}%"))
                parsing_status.values.append(KeyValue(key="parsed_count", value=str(parsed)))
                parsing_status.values.append(KeyValue(key="received_count", value=str(received)))
            else:
                parsing_status.level = DiagnosticStatus.ERROR
                parsing_status.message = "No Messages"
            # Assemble diagnostic array
            diag_array.status = [fix_status, ntrip_status, parsing_status]
            self.status_pub.publish(diag_array)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish DiagnosticArray: {e}")

    def destroy_node(self):
        """Cleanup when shutting down node."""
        try:
            self.get_logger().info("Shutting down UM982 GNSS Node...")
            self.reader.stop()
            self.ntrip_client.stop()
        except:
            pass
        super().destroy_node()

def main(args=None):
    """Main entry point for ROS 2 node."""
    rclpy.init(args=args)
    try:
        # Create node with multi-threaded executor for concurrent callbacks
        node = UM982Node()
        executor = MultiThreadedExecutor()
        # Add node to executor and spin
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt received")
        finally:
            node.destroy_node()
    except Exception as e:
        print(f"Failed to start UM982 node: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()