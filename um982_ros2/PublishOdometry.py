#!/usr/bin/env python3
"""
GNSS Local Frame Odometry Publisher

1. Loads the GNSS origin from a YAML configuration file and publishes odometry and TF data 
   for both the antenna and base_link positions.
2. The base_link origin is defined at (0, 0, 0) in the local coordinate frame.
3. A new GNSS origin can be recorded using `RecordOrigin.py`, which saves the origin data 
   for later use with this script.

Usage:
    python3 PublishOdometry.py <map_name>

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
import argparse
from pathlib import Path

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class GNSSENUNode(Node):
    """Convert GNSS data to ENU frame odometry with TF broadcasting."""
    
    def __init__(self, map_name):
        super().__init__('gnss_enu_node')
        
        self.map_name = map_name
        
        # Load origin from YAML
        if not self._load_origin():
            raise RuntimeError(f"Failed to load origin for map: {map_name}")
        
        # State
        self.yaw_current = None
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriptions
        self.create_subscription(NavSatFix, '/gnss/fix', self._on_fix, 10)
        self.create_subscription(QuaternionStamped, '/gnss/heading', self._on_heading, 10)
        
        # Publishers
        self.odom_antenna_pub = self.create_publisher(Odometry, '/gnss/odometry/antenna', 10)
        self.odom_baselink_pub = self.create_publisher(Odometry, '/gnss/odometry/base_link', 10)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("GNSS ENU Odometry Publisher Started")
        self.get_logger().info(f"Map: {self.map_name}")
        self.get_logger().info(f"Antenna offset: x={self.antenna_offset_x}, y={self.antenna_offset_y}, z={self.antenna_offset_z}")
        self.get_logger().info(f"Antenna origin (LLA): lat={self.origin_antenna_lla[0]:.8f}, lon={self.origin_antenna_lla[1]:.8f}, alt={self.origin_antenna_lla[2]:.2f}m")
        self.get_logger().info(f"Base_link origin (LLA): lat={self.origin_baselink_lla[0]:.8f}, lon={self.origin_baselink_lla[1]:.8f}, alt={self.origin_baselink_lla[2]:.2f}m")
        self.get_logger().info(f"Reference heading: {degrees(self.yaw_ref):.2f}°")
        self.get_logger().info("Base_link will be at origin (0,0,0)")
        self.get_logger().info("=" * 60)
    
    def _load_origin(self):
        """Load origin data from YAML file."""
        try:
            script_dir = Path(__file__).parent
            yaml_path = script_dir / 'gnss_maps' / f'{self.map_name}.yaml'
            
            if not yaml_path.exists():
                self.get_logger().error(f"Map file not found: {yaml_path}")
                self.get_logger().error("Please run RecordOrigin.py first to create this map.")
                return False
            
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            
            # Load antenna origin
            self.origin_antenna_lla = (
                data['antenna_origin']['latitude'],
                data['antenna_origin']['longitude'],
                data['antenna_origin']['altitude']
            )
            
            # Load base_link origin
            self.origin_baselink_lla = (
                data['baselink_origin']['latitude'],
                data['baselink_origin']['longitude'],
                data['baselink_origin']['altitude']
            )
            
            # Load reference heading
            self.yaw_ref = data['orientation']['yaw']
            
            # Load antenna offset
            self.antenna_offset_x = data['antenna_offset']['x']
            self.antenna_offset_y = data['antenna_offset']['y']
            self.antenna_offset_z = data['antenna_offset']['z']
            
            self.get_logger().info(f"Loaded origin from: {yaml_path}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to load origin: {e}")
            return False
    
    def _is_ready(self):
        """Check if all required data is available."""
        return self.yaw_current is not None
    
    def _on_fix(self, msg):
        """Process GNSS fix message."""
        # Validate fix
        if msg.status.status < NavSatStatus.STATUS_FIX:
            self.get_logger().warn(f"Invalid fix status: {msg.status.status}", 
                                   throttle_duration_sec=5.0)
            return
        
        # Wait for heading data
        if not self._is_ready():
            return
        
        try:
            # Convert antenna LLA to ENU (relative to base_link origin)
            e_antenna, n_antenna, u_antenna = pm.geodetic2enu(
                msg.latitude, msg.longitude, msg.altitude,
                *self.origin_baselink_lla
            )
            
            # Rotate to body frame (aligned with initial heading)
            x_antenna, y_antenna, z_antenna = self._enu_to_body(e_antenna, n_antenna, u_antenna)
            
            # Calculate relative heading
            delta_yaw = (self.yaw_current - self.yaw_ref)
            delta_yaw = (delta_yaw + pi) % (2 * pi) - pi  # Normalize to [-π, π]
            
            # Publish antenna odometry and TF
            self._publish_antenna(x_antenna, y_antenna, z_antenna, delta_yaw, msg.header.stamp)
            
            # Calculate base_link position (apply antenna offset)
            x_baselink, y_baselink, z_baselink = self._apply_antenna_offset(
                x_antenna, y_antenna, z_antenna, delta_yaw
            )
            
            # Publish base_link odometry and TF
            self._publish_baselink(x_baselink, y_baselink, z_baselink, delta_yaw, msg.header.stamp)
            
        except Exception as e:
            self.get_logger().error(f"Odometry conversion failed: {e}")
    
    def _on_heading(self, msg):
        """Process heading message (NED to ENU conversion)."""
        try:
            # Convert NED quaternion to ENU
            qx_enu = msg.quaternion.y
            qy_enu = msg.quaternion.x
            qz_enu = -msg.quaternion.z
            qw_enu = msg.quaternion.w
            
            # Extract and normalize yaw
            yaw = atan2(2.0 * (qw_enu * qz_enu + qx_enu * qy_enu),
                       1.0 - 2.0 * (qy_enu**2 + qz_enu**2))
            yaw = yaw % (2 * pi)
            
            self.yaw_current = yaw
            
        except Exception as e:
            self.get_logger().error(f"Heading processing failed: {e}")
    
    def _enu_to_body(self, e, n, u):
        """Rotate ENU coordinates to body frame (aligned with initial heading)."""
        cos_y = cos(-self.yaw_ref)
        sin_y = sin(-self.yaw_ref)
        
        x = cos_y * e - sin_y * n
        y = sin_y * e + cos_y * n
        z = u
        
        return x, y, z
    
    def _apply_antenna_offset(self, x_antenna, y_antenna, z_antenna, delta_yaw):
        """
        Calculate base_link position from antenna position.
        
        Antenna is mounted FORWARD and UP from base_link.
        To get base_link position, we subtract the rotated offset.
        """
        # Rotate offset by current heading
        cos_y = cos(delta_yaw)
        sin_y = sin(delta_yaw)
        
        dx_rotated = cos_y * self.antenna_offset_x - sin_y * self.antenna_offset_y
        dy_rotated = sin_y * self.antenna_offset_x + cos_y * self.antenna_offset_y
        
        # Subtract offset (antenna is ahead, so base_link is behind)
        x_baselink = x_antenna - dx_rotated
        y_baselink = y_antenna - dy_rotated
        z_baselink = z_antenna - self.antenna_offset_z
        
        return x_baselink, y_baselink, z_baselink
    
    def _publish_antenna(self, x, y, z, yaw, stamp):
        """Publish antenna odometry and TF."""
        # Quaternion from yaw
        half_yaw = yaw / 2.0
        qx, qy, qz, qw = 0.0, 0.0, sin(half_yaw), cos(half_yaw)
        
        # Publish odometry
        odom = Odometry()
        odom.header.frame_id = 'gnss_origin'
        odom.header.stamp = stamp
        odom.child_frame_id = 'gnss_antenna'
        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        self.odom_antenna_pub.publish(odom)
        
        # Broadcast TF: gnss_origin → gnss_antenna
        t = TransformStamped()
        t.header.frame_id = 'gnss_origin'
        t.header.stamp = stamp
        t.child_frame_id = 'gnss_antenna'
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_baselink(self, x, y, z, yaw, stamp):
        """Publish base_link odometry and TF."""
        # Quaternion from yaw (same orientation as antenna)
        half_yaw = yaw / 2.0
        qx, qy, qz, qw = 0.0, 0.0, sin(half_yaw), cos(half_yaw)
        
        # Publish odometry
        odom = Odometry()
        odom.header.frame_id = 'gnss_origin'
        odom.header.stamp = stamp
        odom.child_frame_id = 'gnss_base_link'
        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        self.odom_baselink_pub.publish(odom)
        
        # Broadcast TF: gnss_origin → gnss_base_link
        t = TransformStamped()
        t.header.frame_id = 'gnss_origin'
        t.header.stamp = stamp
        t.child_frame_id = 'gnss_base_link'
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Publish GNSS odometry using saved map origin',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  python3 PublishOdometry.py warehouse_1
        """
    )
    parser.add_argument('map_name', help='Name of the map (YAML file to load)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = GNSSENUNode(args.map_name)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()