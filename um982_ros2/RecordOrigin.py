#!/usr/bin/env python3
"""
GNSS Origin Recorder

Records the GNSS origin point for a given map by averaging the antenna position 
and heading over a configurable duration. The resulting origin data is saved 
to a YAML file for future use with other scripts, such as `PublishOdometry.py`.

Usage:
    python3 RecordOrigin.py <map_name> [--duration SECONDS]

Author: Athar Ahmed <athar.a@virya.ai>
Organization: Virya Autonomous Technologies Pvt. Ltd.
Version: 1.1.0

Note:
This software is the proprietary property of Virya Autonomous Technologies Pvt. Ltd.
Unauthorized sharing, copying, or distribution is strictly prohibited.
"""


import rclpy
from rclpy.node import Node
import pymap3d as pm
from math import atan2, cos, sin, pi, degrees
import yaml
import os
import argparse
from pathlib import Path

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import QuaternionStamped


# ============ ANTENNA OFFSET (meters from base_link) ============
ANTENNA_OFFSET_X = 0.0     # Forward
ANTENNA_OFFSET_Y = -0.775  # Right
ANTENNA_OFFSET_Z = 0.0     # Up


class OriginRecorder(Node):
    """Record and average GNSS origin over a period of time."""
    
    def __init__(self, map_name, duration=5.0):
        super().__init__('origin_recorder')
        
        self.map_name = map_name
        self.duration = duration
        
        # Data collection
        self.fix_samples = []
        self.heading_samples = []
        self.recording = False
        self.start_time = None
        
        # Subscriptions
        self.create_subscription(NavSatFix, '/gnss/fix', self._on_fix, 10)
        self.create_subscription(QuaternionStamped, '/gnss/heading', self._on_heading, 10)
        
        # Timer to start recording after node initialization
        self.create_timer(1.0, self._start_recording_once)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"GNSS Origin Recorder")
        self.get_logger().info(f"Map name: {map_name}")
        self.get_logger().info(f"Recording duration: {duration}s")
        self.get_logger().info(f"Antenna offset: x={ANTENNA_OFFSET_X}, y={ANTENNA_OFFSET_Y}, z={ANTENNA_OFFSET_Z}")
        self.get_logger().info("Waiting for GNSS data...")
        self.get_logger().info("=" * 60)
    
    def _start_recording_once(self):
        """Start recording (called once via timer)."""
        if not self.recording:
            self.recording = True
            self.start_time = self.get_clock().now()
            self.get_logger().info("🔴 Recording started...")
    
    def _on_fix(self, msg):
        """Collect fix samples."""
        if not self.recording:
            return
        
        # Check if recording duration exceeded
        if self._is_recording_complete():
            return
        
        # Validate fix
        if msg.status.status < NavSatStatus.STATUS_FIX:
            self.get_logger().warn(f"Invalid fix status: {msg.status.status}", 
                                   throttle_duration_sec=2.0)
            return
        
        # Store sample
        self.fix_samples.append({
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'timestamp': self.get_clock().now()
        })
        
        self.get_logger().info(
            f"Fix sample {len(self.fix_samples)}: "
            f"lat={msg.latitude:.8f}, lon={msg.longitude:.8f}, alt={msg.altitude:.2f}m",
            throttle_duration_sec=1.0
        )
    
    def _on_heading(self, msg):
        """Collect heading samples."""
        if not self.recording:
            return
        
        # Check if recording duration exceeded
        if self._is_recording_complete():
            return
        
        try:
            # Convert NED quaternion to ENU
            qx_enu = msg.quaternion.y
            qy_enu = msg.quaternion.x
            qz_enu = -msg.quaternion.z
            qw_enu = msg.quaternion.w
            
            # Extract yaw
            yaw = atan2(2.0 * (qw_enu * qz_enu + qx_enu * qy_enu),
                       1.0 - 2.0 * (qy_enu**2 + qz_enu**2))
            yaw = yaw % (2 * pi)
            
            # Store sample
            self.heading_samples.append({
                'yaw': yaw,
                'quaternion': {
                    'x': qx_enu,
                    'y': qy_enu,
                    'z': qz_enu,
                    'w': qw_enu
                },
                'timestamp': self.get_clock().now()
            })
            
            self.get_logger().info(
                f"Heading sample {len(self.heading_samples)}: yaw={degrees(yaw):.2f}°",
                throttle_duration_sec=1.0
            )
            
        except Exception as e:
            self.get_logger().error(f"Heading processing failed: {e}")
    
    def _is_recording_complete(self):
        """Check if recording duration has been reached."""
        if self.start_time is None:
            return False
        
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        return elapsed >= self.duration
    
    def _calculate_baselink_lla(self, antenna_lat, antenna_lon, antenna_alt, yaw):
        """Calculate base_link LLA from antenna LLA."""
        # Rotate antenna offset to ENU frame using yaw
        cos_y = cos(yaw)
        sin_y = sin(yaw)
        
        # Antenna offset in body frame (negate to go from antenna to base_link)
        offset_x_body = -ANTENNA_OFFSET_X
        offset_y_body = -ANTENNA_OFFSET_Y
        
        # Convert to ENU
        offset_e = cos_y * offset_x_body - sin_y * offset_y_body
        offset_n = sin_y * offset_x_body + cos_y * offset_y_body
        offset_u = -ANTENNA_OFFSET_Z
        
        # Convert ENU offset to LLA
        lat_baselink, lon_baselink, alt_baselink = pm.enu2geodetic(
            offset_e, offset_n, offset_u,
            antenna_lat, antenna_lon, antenna_alt
        )
        
        return lat_baselink, lon_baselink, alt_baselink
    
    def process_and_save(self):
        """Average collected samples and save to YAML."""
        if len(self.fix_samples) == 0:
            self.get_logger().error("No fix samples collected!")
            return False
        
        if len(self.heading_samples) == 0:
            self.get_logger().error("No heading samples collected!")
            return False
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Processing {len(self.fix_samples)} fix samples and "
                             f"{len(self.heading_samples)} heading samples...")
        
        # Average antenna position
        avg_lat = sum(s['latitude'] for s in self.fix_samples) / len(self.fix_samples)
        avg_lon = sum(s['longitude'] for s in self.fix_samples) / len(self.fix_samples)
        avg_alt = sum(s['altitude'] for s in self.fix_samples) / len(self.fix_samples)
        
        # Average heading (circular mean for angles)
        sin_sum = sum(sin(s['yaw']) for s in self.heading_samples)
        cos_sum = sum(cos(s['yaw']) for s in self.heading_samples)
        avg_yaw = atan2(sin_sum, cos_sum) % (2 * pi)
        
        # Average quaternion (simple averaging - good enough for small variations)
        avg_qx = sum(s['quaternion']['x'] for s in self.heading_samples) / len(self.heading_samples)
        avg_qy = sum(s['quaternion']['y'] for s in self.heading_samples) / len(self.heading_samples)
        avg_qz = sum(s['quaternion']['z'] for s in self.heading_samples) / len(self.heading_samples)
        avg_qw = sum(s['quaternion']['w'] for s in self.heading_samples) / len(self.heading_samples)
        
        # Normalize quaternion
        q_norm = (avg_qx**2 + avg_qy**2 + avg_qz**2 + avg_qw**2) ** 0.5
        avg_qx /= q_norm
        avg_qy /= q_norm
        avg_qz /= q_norm
        avg_qw /= q_norm
        
        # Calculate base_link LLA from antenna LLA
        baselink_lat, baselink_lon, baselink_alt = self._calculate_baselink_lla(
            avg_lat, avg_lon, avg_alt, avg_yaw
        )
        
        # Prepare YAML data
        origin_data = {
            'map_name': self.map_name,
            'timestamp': self.get_clock().now().to_msg().sec,
            'antenna_origin': {
                'latitude': float(avg_lat),
                'longitude': float(avg_lon),
                'altitude': float(avg_alt)
            },
            'baselink_origin': {
                'latitude': float(baselink_lat),
                'longitude': float(baselink_lon),
                'altitude': float(baselink_alt)
            },
            'orientation': {
                'yaw': float(avg_yaw),
                'quaternion': {
                    'x': float(avg_qx),
                    'y': float(avg_qy),
                    'z': float(avg_qz),
                    'w': float(avg_qw)
                }
            },
            'antenna_offset': {
                'x': ANTENNA_OFFSET_X,
                'y': ANTENNA_OFFSET_Y,
                'z': ANTENNA_OFFSET_Z
            },
            'samples_collected': {
                'fix_count': len(self.fix_samples),
                'heading_count': len(self.heading_samples)
            }
        }
        
        # Create directory if it doesn't exist
        script_dir = Path(__file__).parent
        maps_dir = script_dir / 'gnss_maps'
        maps_dir.mkdir(exist_ok=True)
        
        # Save to YAML
        yaml_path = maps_dir / f'{self.map_name}.yaml'
        with open(yaml_path, 'w') as f:
            yaml.dump(origin_data, f, default_flow_style=False, sort_keys=False)
        
        self.get_logger().info("✅ Origin recorded successfully!")
        self.get_logger().info(f"Saved to: {yaml_path}")
        self.get_logger().info(f"Antenna origin: lat={avg_lat:.8f}, lon={avg_lon:.8f}, alt={avg_alt:.2f}m")
        self.get_logger().info(f"Base_link origin: lat={baselink_lat:.8f}, lon={baselink_lon:.8f}, alt={baselink_alt:.2f}m")
        self.get_logger().info(f"Reference heading: {degrees(avg_yaw):.2f}°")
        self.get_logger().info("=" * 60)
        
        return True


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Record GNSS origin for a map',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 RecordOrigin.py warehouse_1
  python3 RecordOrigin.py parking_lot --duration 10
        """
    )
    parser.add_argument('map_name', help='Name of the map (used for YAML filename)')
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Recording duration in seconds (default: 5.0)')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = OriginRecorder(args.map_name, args.duration)
    
    try:
        # Spin until recording is complete
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            if node._is_recording_complete() and node.recording:
                node.recording = False  # Stop collecting samples
                success = node.process_and_save()
                break
        
    except KeyboardInterrupt:
        node.get_logger().info("Recording interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()