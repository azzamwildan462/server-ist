#!/usr/bin/python3

import rclpy
from rclpy.node import Node 

from loguru import logger

# udp utils 
import sys
import os
import socket
import struct
from dataclasses import dataclass, asdict, replace
import time

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from std_msgs.msg import Int32
import math

import csv
from pathlib import Path

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

@dataclass
class towing_t:
    name: int = 0
    soc: int = 0
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_theta: float = 0.0
    terminal: int = -1
    warning: int = 0
    lap: int = 0
    ts_ms: int = 0

class UDP2ROSLIB(Node):
    def __init__(self):
        super().__init__('udp2roslib_node')
        self.get_logger().info("UDP2ROSLIB Node has been started.")

        # Logger
        # ------
        logger.remove()
        logger.add(
            sys.stdout,
            colorize=True,
            format="<green>{time:HH:mm:ss.SSS}</green> | <level>{level:^6}</level> | <cyan>{function}</cyan>:<cyan>{line}</cyan> - {message}",
            enqueue=True,
        )

        self.declare_parameter("MY_SERVER_IP", "0.0.0.0")
        self.declare_parameter("MY_SERVER_PORT", 1254)
        self.declare_parameter("T2_IP", "10.20.30.40")
        self.declare_parameter("T2_PORT", 1255)
        self.declare_parameter("OFC_IP", "10.20.30.245")
        self.declare_parameter("OFC_PORT", 1255)
        self.declare_parameter('waypoint_file_path', str(Path.home() / 'waypoints.csv'))

        self.MY_SERVER_IP = self.get_parameter("MY_SERVER_IP").get_parameter_value().string_value
        self.MY_SERVER_PORT = self.get_parameter("MY_SERVER_PORT").get_parameter_value().integer_value
        self.T2_IP = self.get_parameter("T2_IP").get_parameter_value().string_value
        self.T2_PORT = self.get_parameter("T2_PORT").get_parameter_value().integer_value
        self.OFC_IP = self.get_parameter("OFC_IP").get_parameter_value().string_value
        self.OFC_PORT = self.get_parameter("OFC_PORT").get_parameter_value().integer_value
        self.waypoint_file_path = Path(self.get_parameter('waypoint_file_path').get_parameter_value().string_value)

        self.t2 = towing_t()

        self.udp_init_as_server()
        self.udp_init_as_client()

        self.wtf_pcl = PointCloud()
        points_xyz = self._load_xy_as_xyz(self.waypoint_file_path)
        if not points_xyz:
            self.get_logger().warn(f'No points loaded from "{self.waypoint_file_path}".')
        else:
            self.get_logger().info(f'Loaded {len(points_xyz)} points from "{self.waypoint_file_path}".')
        
        header = Header()
        header.frame_id = "map"
        header.stamp = self.get_clock().now().to_msg()

        self.pub_t2_odom = self.create_publisher(Odometry, '/udp/t2/pose_filtered', 1)
        self.pub_t2_status_emergency = self.create_publisher(Int16, '/udp/t2/status_emergency', 1)
        self.pub_t2_terminal_terakhir = self.create_publisher(Int16, '/udp/t2/terminal_terakhir', 1)
        self.pub_t2_battery_soc = self.create_publisher(Int16, '/udp/t2/battery_soc', 1)
        self.pub_t2_counter_lap = self.create_publisher(Int32, '/udp/t2/counter_lap', 1)
        self.pub_t2_lag_ms = self.create_publisher(Int16, '/udp/t2/lag_ms', 1)
        self.pub_waypoint = self.create_publisher(PointCloud, '/udp/waypoints', 1)

        self.last_time_waypoint_published_ms = 0
        self.last_time_update_lag = 0

        # Example of a timer that calls a callback every second
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def yaw_to_quat(self, yaw_rad: float):
        """Convert yaw (radians) to quaternion (qx, qy, qz, qw)."""
        h = 0.5 * yaw_rad
        return (0.0, 0.0, math.sin(h), math.cos(h))

    def _load_xy_as_xyz(self, path: Path):
        """
        Reads CSV with columns at least 'x,y,...' (header allowed).
        Returns list of (x, y, 0.0) as floats.
        """
        points = []
        if not path.exists():
            self.get_logger().error(f'Waypoint file "{path}" not found.')
            return points

        with path.open('r', newline='') as f:
            reader = csv.reader(f)
            # Peek first row to see if it's a header (contains non-numeric)
            rows = list(reader)
            start_idx = 1

            for row in rows[start_idx:]:
                if len(row) < 2:
                    continue
                try:
                    x = float(row[0])
                    y = float(row[1])
                    points.append((x, y, 0.0))  # z = 0

                    p_msg = Point32()
                    p_msg.x = x
                    p_msg.y = y 
                    p_msg.z = 0.0
                    self.wtf_pcl.points.append(p_msg)
                except ValueError:
                    # skip malformed lines
                    continue
        return points

    def udp_init_as_server(self):
        self.sock_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_server.bind((self.MY_SERVER_IP, self.MY_SERVER_PORT))
        self.sock_server.setblocking(False)
        logger.info(f"UDP server initialized on {self.MY_SERVER_IP}:{self.MY_SERVER_PORT}")
    
    def udp_init_as_client(self):
        self.sock_client_t2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_client_t2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_client_t2.connect((self.T2_IP, self.T2_PORT))  
        self.sock_client_t2.setblocking(False) 
        logger.info(f"UDP client initialized to send to {self.T2_IP}:{self.T2_PORT}")

        self.sock_client_ofc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_client_ofc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_client_ofc.connect((self.OFC_IP, self.OFC_PORT))  
        self.sock_client_ofc.setblocking(False) 
        logger.info(f"UDP client initialized to send to {self.OFC_IP}:{self.OFC_PORT}")
        
    def timer_callback(self):
        time_now_ms = int(time.time_ns() // 1_000_000)

        # Receiving from server...
        try:
            data, addr = self.sock_server.recvfrom(1024)  # buffer size is 1024 bytes
            if data:
                unpacked_data = struct.unpack('i i f f f i i i Q', data)
                if addr[0] == self.T2_IP:
                    self.t2 = towing_t(
                        name=unpacked_data[0],
                        soc=unpacked_data[1],
                        pose_x=unpacked_data[2],
                        pose_y=unpacked_data[3],
                        pose_theta=unpacked_data[4],
                        terminal=unpacked_data[5],
                        warning=unpacked_data[6],
                        lap=unpacked_data[7],
                        ts_ms=time_now_ms,
                    )
                    logger.info(f"Rcvd {addr}: {asdict(self.t2)}")
        except BlockingIOError:
            pass

        # Sending t2 to ofc 
        try:
            packed_data = struct.pack(
                'i i f f f i i i Q',
                self.t2.name,
                self.t2.soc,
                self.t2.pose_x,
                self.t2.pose_y,
                self.t2.pose_theta,
                self.t2.terminal,
                self.t2.warning,
                self.t2.lap,
                time_now_ms,
            )
            self.sock_client_ofc.send(packed_data)
            logger.info(f"Sent data to {self.OFC_IP}:{self.OFC_PORT}: {asdict(self.t2)}")
        except (BlockingIOError, InterruptedError):
            pass
        except Exception as e:
            logger.error(f"send {self.OFC_IP} error: {e}")


        # Publish to web ui 
        qx, qy, qz, qw = self.yaw_to_quat(self.t2.pose_theta)
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = self.t2.pose_x
        odom_msg.pose.pose.position.y = self.t2.pose_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        self.pub_t2_odom.publish(odom_msg)

        status_emergency_msg = Int16()
        status_emergency_msg.data = self.t2.warning
        self.pub_t2_status_emergency.publish(status_emergency_msg)

        terminal_terakhir_msg = Int16()
        terminal_terakhir_msg.data = self.t2.terminal
        self.pub_t2_terminal_terakhir.publish(terminal_terakhir_msg)

        battery_soc_msg = Int16()
        battery_soc_msg.data = self.t2.soc
        self.pub_t2_battery_soc.publish(battery_soc_msg)

        counter_lap_msg = Int32()
        counter_lap_msg.data = self.t2.lap
        self.pub_t2_counter_lap.publish(counter_lap_msg)

        if time_now_ms - self.last_time_update_lag > 1000:
            self.last_time_update_lag = time_now_ms
            t2_lag_ms_msg = Int16()
            lag_ms_buffer = time_now_ms - self.t2.ts_ms
            if lag_ms_buffer < 1000:
                lag_ms_buffer = 30 # zzz

            # reset t2 telat 1 tick tapi gapapa 
            if lag_ms_buffer > 30000:
                self.t2.warning = 0
                self.t2.soc = 0
            if lag_ms_buffer > 9999:
                t2_lag_ms_msg.data = 9999
            else:
                t2_lag_ms_msg.data = lag_ms_buffer
            self.pub_t2_lag_ms.publish(t2_lag_ms_msg)

        if time_now_ms - self.last_time_waypoint_published_ms > 1000:
            self.last_time_waypoint_published_ms = time_now_ms 
            self.pub_waypoint.publish(self.wtf_pcl)

def main(args=None):
    rclpy.init(args=args)

    node_udp2roslib = UDP2ROSLIB()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_udp2roslib)
    executor.spin()
    
if __name__ == '__main__':
    main(sys.argv)