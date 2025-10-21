#!/usr/bin/env python3
"""
UM982 GNSS ROS2 Node

This node interfaces with the UM982 GNSS receiver over USB serial. 
It connects to an NTRIP service, sends GGA messages, receives GNSS corrections, 
and forwards them to the UM982. The node publishes GNSS fixes, heading, 
and diagnostic status. It relies solely on GGA and PVTSLN messages.

Author: Athar Ahmed <athar.a@virya.ai>
Organization: Virya Autonomous Technologies Pvt. Ltd.
Version: 1.1.0

Note: This software is proprietary to Virya Autonomous Technologies Pvt. Ltd. 
It must not be shared, copied, or distributed without prior authorization.
"""

import os
import sys
import socket
import base64
import threading
import serial
import time
import math
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import QuaternionStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# ============ CONFIGURATION ============
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUDRATE = 115200
NTRIP_HOST = '103.206.29.4'
NTRIP_PORT = 2105
MOUNTPOINT = 'IFKP'
USERNAME = 'pranav.k'
PASSWORD = 'cors@2022'
GGA_INTERVAL = 1.0
FIX_PUBLISH_RATE = 10.0
STATUS_PUBLISH_RATE = 1.0
NTRIP_TIMEOUT = 10
SERIAL_TIMEOUT = 0.5
NTRIP_RECONNECT_DELAY = 3


# ============ UTILITY FUNCTIONS ============
def safe_float(value: str) -> Optional[float]:
    """Safely convert string to float, return None on failure."""
    try:
        return float(value)
    except (ValueError, TypeError):
        return None


def safe_int(value: str) -> Optional[int]:
    """Safely convert string to int, return None on failure."""
    try:
        return int(value)
    except (ValueError, TypeError):
        return None


def verify_nmea_checksum(sentence: str) -> bool:
    """Verify NMEA sentence checksum."""
    try:
        if '$' not in sentence or '*' not in sentence:
            return False
        msg, crc = sentence[1:].split('*', 1)
        computed = 0
        for char in msg:
            computed ^= ord(char)
        return f'{computed:02X}' == crc[:2].upper()
    except Exception:
        return False


def _crc32_table():
    """Generate CRC32 lookup table."""
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else (crc >> 1)
        table.append(crc)
    return table
_CRC32_TABLE = _crc32_table()


def verify_pvtsln_crc(sentence: str) -> bool:
    """Verify proprietary PVTSLN sentence CRC32."""
    try:
        if '#' not in sentence or '*' not in sentence:
            return False
        body, crc = sentence[1:].split('*', 1)
        computed = 0
        for byte in body.encode('ascii', 'ignore'):
            computed = _CRC32_TABLE[(computed ^ byte) & 0xFF] ^ (computed >> 8)
        return f'{computed & 0xFFFFFFFF:08x}' == crc[:8].lower()
    except Exception:
        return False


def parse_pvtsln(sentence: str) -> Optional[Dict[str, Any]]:
    """Parse proprietary PVTSLN sentence."""
    try:
        if '*' not in sentence:
            return None
        
        body = sentence[1:sentence.find('*')]
        fields = body.split(',')
        
        if len(fields) < 30:
            return None
        
        offset = 9
        return {
            'fix_type': fields[offset],
            'height': safe_float(fields[offset + 1]),
            'latitude': safe_float(fields[offset + 2]),
            'longitude': safe_float(fields[offset + 3]),
            'height_std': safe_float(fields[offset + 4]) or 0.0,
            'lat_std': safe_float(fields[offset + 5]) or 0.0,
            'lon_std': safe_float(fields[offset + 6]) or 0.0,
            'sats_tracked': safe_int(fields[offset + 13]) or 0,
            'sats_used': safe_int(fields[offset + 14]) or 0,
            'vel_north': safe_float(fields[offset + 17]) or 0.0,
            'vel_east': safe_float(fields[offset + 18]) or 0.0,
            'vel_ground': safe_float(fields[offset + 19]) or 0.0,
            'heading_deg': safe_float(fields[offset + 22]),
            'heading_pitch': safe_float(fields[offset + 23]) or 0.0,
        }
    except Exception:
        return None


def euler_to_quaternion(yaw: float, pitch: float, roll: float = 0.0) -> tuple:
    """Convert Euler angles (degrees) to quaternion."""
    yaw_rad = math.radians(yaw)
    pitch_rad = math.radians(pitch)
    roll_rad = math.radians(roll)
    
    cy, sy = math.cos(yaw_rad / 2), math.sin(yaw_rad / 2)
    cp, sp = math.cos(pitch_rad / 2), math.sin(pitch_rad / 2)
    cr, sr = math.cos(roll_rad / 2), math.sin(roll_rad / 2)
    
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    
    return x, y, z, w


# ============ SERIAL READER ============
class SerialReader(threading.Thread):
    """Reads NMEA/proprietary sentences from GNSS receiver."""
    
    def __init__(self, port: str, baudrate: int, logger=None):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.logger = logger
        self._lock = threading.Lock()
        self._serial = None
        self._running = True
        self._latest_pvtsln = None
        self._latest_gga = None
    
    def run(self):
        """Main serial read loop."""
        while self._running:
            if not self._ensure_connection():
                time.sleep(1)
                continue
            
            try:
                line = self._serial.readline().decode('utf-8', 'ignore').strip()
                if line:
                    self._process_line(line)
            except Exception as e:
                self._error(f"Serial read error: {e}")
                self._close()
            
            time.sleep(0.01)
        
        self._close()
    
    def _ensure_connection(self) -> bool:
        """Ensure serial connection is open."""
        try:
            if self._serial and getattr(self._serial, 'is_open', False):
                return True
            
            self._serial = serial.Serial(
                self.port, 
                self.baudrate, 
                timeout=SERIAL_TIMEOUT
            )
            self._info(f"Serial connected: {self.port}@{self.baudrate}")
            return True
        except Exception as e:
            self._error(f"Serial connection failed: {e}")
            return False
    
    def _process_line(self, line: str):
        """Process incoming NMEA/proprietary sentences."""
        try:
            if line.startswith('$') and 'GGA' in line and verify_nmea_checksum(line):
                with self._lock:
                    self._latest_gga = line
            
            elif line.startswith('#PVTSLN') and verify_pvtsln_crc(line):
                data = parse_pvtsln(line)
                if data:
                    with self._lock:
                        self._latest_pvtsln = data
                else:
                    self._warning("Failed to parse PVTSLN")
            
        except Exception as e:
            self._error(f"Line processing error: {e}")
    
    def get_latest(self) -> tuple:
        """Get latest PVTSLN and GGA data."""
        with self._lock:
            return self._latest_pvtsln, self._latest_gga
    
    def write(self, data: bytes) -> bool:
        """Write data to serial port."""
        with self._lock:
            if self._serial and getattr(self._serial, 'is_open', False):
                try:
                    self._serial.write(data)
                    return True
                except Exception as e:
                    self._error(f"Serial write failed: {e}")
        return False
    
    def stop(self):
        """Stop serial reader thread."""
        self._running = False
        time.sleep(0.1)
        self._close()
    
    def _close(self):
        """Close serial connection."""
        try:
            if self._serial and getattr(self._serial, 'is_open', False):
                self._serial.close()
        except Exception:
            pass
    
    def _info(self, msg: str):
        if self.logger:
            self.logger.info(msg)
    
    def _warning(self, msg: str):
        if self.logger:
            self.logger.warn(msg)
    
    def _error(self, msg: str):
        if self.logger:
            self.logger.error(msg)


# ============ NTRIP CLIENT ============

class NTRIPClient:
    """NTRIP client for receiving RTK corrections."""

    def __init__(self, reader: SerialReader, host: str, port: int,
                 mount: str, user: str, pwd: str, logger=None):
        self.reader = reader
        self.host = host
        self.port = port
        self.mount = mount
        self.user = user
        self.pwd = pwd
        self.logger = logger
        self._running = False
        self._sock = None
        self._sock_lock = threading.Lock()
        auth_str = base64.b64encode(f'{user}:{pwd}'.encode()).decode()
        self._request = (
            f"GET /{mount} HTTP/1.0\r\n"
            f"User-Agent: UM982-NTRIP\r\n"
            f"Authorization: Basic {auth_str}\r\n\r\n"
        ).encode()

    def _connect_ntrip(self):
        """Connect to NTRIP caster and return socket (with reconnection logic)."""
        while self._running:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(NTRIP_TIMEOUT)
                s.connect((self.host, self.port))
                s.send(self._request)
                
                # Wait for ICY response (HTTP headers)
                response = s.recv(1024).decode(errors='ignore')
                if "ICY 200 OK" in response or "HTTP/1.0 200 OK" in response:
                    self._info("Connected to NTRIP caster successfully")
                    return s
                else:
                    self._warning(f"Unexpected NTRIP response: {response.splitlines()[0] if response.splitlines() else 'empty'}")
                    s.close()
                    time.sleep(NTRIP_RECONNECT_DELAY)
            except Exception as e:
                self._error(f"Failed to connect NTRIP: {e}")
                time.sleep(NTRIP_RECONNECT_DELAY)
        return None

    def start(self):
        """Start NTRIP client."""
        self._running = True
        threading.Thread(target=self._send_gga_loop, daemon=True).start()
        threading.Thread(target=self._recv_loop, daemon=True).start()
        self._info("NTRIP client started")

    def stop(self):
        """Stop NTRIP client."""
        self._running = False
        self._close_socket()
        self._info("NTRIP client stopped")

    def _send_gga_loop(self):
        """Send GGA position updates to NTRIP server with reconnection."""
        while self._running:
            try:
                # Ensure we have a connection
                with self._sock_lock:
                    if self._sock is None:
                        self._sock = self._connect_ntrip()
                        if self._sock is None:
                            continue
                
                # Get latest GGA data
                _, gga = self.reader.get_latest()
                if gga:
                    try:
                        with self._sock_lock:
                            if self._sock:
                                self._sock.send((gga + '\r\n').encode())
                    except (BrokenPipeError, OSError) as e:
                        self._error(f"Broken pipe while sending GGA: {e}. Reconnecting NTRIP...")
                        with self._sock_lock:
                            if self._sock:
                                self._sock.close()
                            self._sock = self._connect_ntrip()
                
                time.sleep(GGA_INTERVAL)
            except Exception as e:
                self._error(f"GGA send error: {e}")
                time.sleep(0.5)

    def _recv_loop(self):
        """Receive RTCM corrections from NTRIP and forward to GNSS with reconnection."""
        while self._running:
            try:
                # Ensure we have a connection
                with self._sock_lock:
                    if self._sock is None:
                        self._sock = self._connect_ntrip()
                        if self._sock is None:
                            continue
                
                # Receive data
                with self._sock_lock:
                    sock = self._sock
                
                if sock:
                    try:
                        data = sock.recv(4096)
                        if data:
                            self.reader.write(data)
                        else:
                            # Connection closed by server
                            self._warning("NTRIP server closed connection. Reconnecting...")
                            with self._sock_lock:
                                if self._sock:
                                    self._sock.close()
                                self._sock = self._connect_ntrip()
                    except (BrokenPipeError, OSError) as e:
                        self._error(f"NTRIP disconnected: {e}. Reconnecting...")
                        with self._sock_lock:
                            if self._sock:
                                self._sock.close()
                            self._sock = self._connect_ntrip()
                
                time.sleep(0.01)
            except Exception as e:
                self._error(f"RTCM receive error: {e}")
                time.sleep(0.5)

    def _close_socket(self):
        """Close NTRIP socket."""
        with self._sock_lock:
            try:
                if self._sock:
                    self._sock.close()
            except Exception:
                pass
            self._sock = None

    def _info(self, msg: str):
        if self.logger:
            self.logger.info(msg)

    def _warning(self, msg: str):
        if self.logger:
            self.logger.warn(msg)

    def _error(self, msg: str):
        if self.logger:
            self.logger.error(msg)

# ============ ROS2 NODE ============
class UM982Node(Node):
    """UM982 GNSS ROS2 Node."""
    
    def __init__(self):
        super().__init__('um982_gnss_node')
        
        logger = self.get_logger()
        logger.info("UM982 GNSS node initializing...")
        
        # Create publishers
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self._pub_fix = self.create_publisher(NavSatFix, '/gnss/fix', qos)
        self._pub_heading = self.create_publisher(QuaternionStamped, '/gnss/heading', qos)
        self._pub_status = self.create_publisher(DiagnosticArray, '/gnss/status', 10)
        
        # Initialize serial reader
        self._reader = SerialReader(SERIAL_PORT, SERIAL_BAUDRATE, logger)
        self._reader.start()
        
        # Initialize NTRIP client
        self._ntrip = NTRIPClient(
            self._reader,
            NTRIP_HOST,
            NTRIP_PORT,
            MOUNTPOINT,
            USERNAME,
            PASSWORD,
            logger
        )
        self._ntrip.start()
        
        # Create timers with reentrant callback group
        cbg = ReentrantCallbackGroup()
        self.create_timer(1.0 / FIX_PUBLISH_RATE, self._publish_fix, callback_group=cbg)
        self.create_timer(1.0 / FIX_PUBLISH_RATE, self._publish_heading, callback_group=cbg)
        self.create_timer(1.0 / STATUS_PUBLISH_RATE, self._publish_status, callback_group=cbg)
        
        logger.info("UM982 GNSS node ready")
    
    def _publish_fix(self):
        """Publish NavSatFix message."""
        pvtsln, _ = self._reader.get_latest()
        if not pvtsln or pvtsln.get('latitude') is None:
            return
        
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gnss_antenna'
        msg.latitude = pvtsln['latitude']
        msg.longitude = pvtsln['longitude']
        msg.altitude = pvtsln['height']
        
        # Set fix status based on UM982 fix type string
        fix_type = str(pvtsln.get('fix_type', '')).upper()       
        
        if 'RTK_INT' in fix_type or 'RTKFIXED' in fix_type or 'RTK FIXED' in fix_type:
            # RTK Fixed/Integer (highest quality - cm level accuracy)
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        elif 'RTK_FLOAT' in fix_type or 'RTKFLOAT' in fix_type or 'RTK FLOAT' in fix_type:
            # RTK Float (high quality - dm level accuracy)
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        elif 'FLOAT' in fix_type:
            # Generic float solution
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        elif 'DGPS' in fix_type or 'DGNSS' in fix_type:
            # DGPS/DGNSS (medium quality - meter level accuracy)
            msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        elif 'SPP' in fix_type or 'SINGLE' in fix_type or '3D' in fix_type or 'PSRDIFF' in fix_type:
            # Single Point Positioning (basic fix - meter level accuracy)
            msg.status.status = NavSatStatus.STATUS_FIX
        elif fix_type and fix_type != 'NONE' and fix_type != '':
            # Has some fix type but we don't recognize it - default to basic fix
            msg.status.status = NavSatStatus.STATUS_FIX
        else:
            # No valid fix
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        
        msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
        
        # Set covariance
        msg.position_covariance = [0.0] * 9
        msg.position_covariance[0] = pvtsln['lat_std'] ** 2
        msg.position_covariance[4] = pvtsln['lon_std'] ** 2
        msg.position_covariance[8] = pvtsln['height_std'] ** 2
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self._pub_fix.publish(msg)
    
    def _publish_heading(self):
        """Publish heading as QuaternionStamped."""
        pvtsln, _ = self._reader.get_latest()
        if not pvtsln:
            return
        
        heading = pvtsln.get('heading_deg')
        if heading is None:
            return
        
        pitch = pvtsln.get('heading_pitch', 0.0) or 0.0
        qx, qy, qz, qw = euler_to_quaternion(heading, pitch, 0.0)
        
        msg = QuaternionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gnss_antenna'
        msg.quaternion.x = qx
        msg.quaternion.y = qy
        msg.quaternion.z = qz
        msg.quaternion.w = qw
        
        self._pub_heading.publish(msg)
    
    def _publish_status(self):
        """Publish diagnostic status."""
        pvtsln, _ = self._reader.get_latest()
        
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = "UM982 GNSS"
        status.hardware_id = "UM982"
        
        if not pvtsln:
            status.level = DiagnosticStatus.ERROR
            status.message = "No GNSS data"
        else:
            fix_type = str(pvtsln.get('fix_type', ''))
            
            if 'RTK' in fix_type or 'INT' in fix_type:
                status.level = DiagnosticStatus.OK
                status.message = "RTK Fixed"
            elif 'FLOAT' in fix_type or 'DGPS' in fix_type:
                status.level = DiagnosticStatus.WARN
                status.message = f"{fix_type} Solution"
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = "No valid fix"
            
            status.values = [
                KeyValue(key="fix_type", value=fix_type),
                KeyValue(key="satellites_tracked", value=str(pvtsln.get('sats_tracked', 0))),
                KeyValue(key="satellites_used", value=str(pvtsln.get('sats_used', 0))),
                KeyValue(key="lat_std_m", value=f"{pvtsln.get('lat_std', 0.0):.4f}"),
                KeyValue(key="lon_std_m", value=f"{pvtsln.get('lon_std', 0.0):.4f}"),
                KeyValue(key="height_std_m", value=f"{pvtsln.get('height_std', 0.0):.4f}"),
            ]
        
        diag.status = [status]
        self._pub_status.publish(diag)
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        self.get_logger().info("Shutting down UM982 GNSS node...")
        self._reader.stop()
        self._ntrip.stop()
        try:
            if self._reader.is_alive():
                self._reader.join(timeout=2.0)
        except Exception:
            pass
        super().destroy_node()


# ============ MAIN ============
def main(argv=None):
    """Main entry point."""
    rclpy.init(args=argv)
    node = UM982Node()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt received")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main(sys.argv))