#!/usr/bin/python3

import rclpy
from rclpy.node import Node 

from loguru import logger
from datetime import datetime, timedelta
from zoneinfo import ZoneInfo  # Python 3.9+

from flask import Flask, request, jsonify, Response, stream_with_context
from flask_cors import CORS
import requests

import influxdb_client
from influxdb_client import Point
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client.client.write_api import WriteOptions
import threading

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

EMERGENCY_LIDAR_DEPAN_DETECTED = 0b010
EMERGENCY_CAMERA_OBS_DETECTED = 0b100
EMERGENCY_GYRO_ANOMALY_DETECTED = 0b1000
EMERGENCY_ICP_SCORE_TERLALU_BESAR = 0b10000
EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR = 0b100000
EMERGENCY_STOP_KARENA_OBSTACLE = 0b1000000
EMERGENCY_GANDENGAN_LEPAS = 0b100000000
STATUS_TOWING_ACTIVE_AUTO = 0b01

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

        # Setup configuration
        self.ROBOT_NAME = "t2_testing"
        self.INFLUXDB_BUCKET = "ist_salah"
        self.INFLUXDB_ORG = "ist"
        self.INFLUXDB_URL = "http://172.30.114.191:8086"
        self.INFLUXDB_USERNAME = "ist"
        self.INFLUXDB_PASSWORD = "Ptist@2025"
        self.WIB_TZ = ZoneInfo("Asia/Jakarta")    # UTC+7

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

        # Connect to InfluxDB
        self._influx_lock = threading.RLock()
        self._build_influx_client()

        # Init variables
        self.error_counter = 0
        self.last_lap = 0
        self.prev_lap_program = 0
        self.lap_global = 0
        self.battery_global = 0
        self.last_terminal = -2
        self.last_warning = ""

        self.now_is_day_or_night = 0 # 0=day, 1=night
        self.prev_now_is_day_or_night = 0 # 0=day, 1=night
        self.jumlah_lap_now = 0

        self.has_get_last_log_data = False

        # Setup Flask app
        self.app = Flask(__name__)
        CORS(self.app)

        self.register_routes()

        # Flask
        flask_thread = threading.Thread(target=self.thread_flask)
        flask_thread.daemon = True
        flask_thread.start()

        self.log_file_mutex = threading.Lock()

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
        self.pub_t2_lap_sum = self.create_publisher(Int16, '/udp/t2/lap_sum', 1)
        self.pub_waypoint = self.create_publisher(PointCloud, '/udp/waypoints', 1)

        self.last_time_waypoint_published_ms = 0
        self.last_time_update_lag = 0
        self.last_time_calculate_shift = 0
        self.last_time_influx_db = 0

        # Example of a timer that calls a callback every second
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def _build_influx_client(self):
        """Create a fresh InfluxDB client + write/query APIs, with health check."""
        with self._influx_lock:
            # Close previous if any
            try:
                if hasattr(self, "db_client") and self.db_client is not None:
                    self.db_client.close()
            except Exception as e:
                logger.warning(f"Closing previous Influx client failed (ignored): {e}")

            try:
                self.db_client = influxdb_client.InfluxDBClient(
                    url=self.INFLUXDB_URL,
                    org=self.INFLUXDB_ORG,
                    username=self.INFLUXDB_USERNAME,
                    password=self.INFLUXDB_PASSWORD,
                    timeout=30000,
                    read_timeout=30000,
                    query_timeout=30000,
                )

                # Hardcoded, robust batching + retries
                # self.db_write_api = self.db_client.write_api(
                #     write_options=WriteOptions(
                #         batch_size=5000,
                #         flush_interval=1000,
                #         jitter_interval=2000,
                #         retry_interval=5000,
                #         max_retries=5,
                #         max_retry_delay=60000,
                #         exponential_base=2,
                #     )
                # )
                self.db_write_api = self.db_client.write_api(write_options=SYNCHRONOUS)
                self.db_query_api = self.db_client.query_api()

                # Health check (raises if unhealthy)
                health = self.db_client.health()
                if getattr(health, "status", "").upper() != "PASS":
                    raise RuntimeError(f"InfluxDB unhealthy: {getattr(health, 'message', '')}")

                logger.info("InfluxDB client connected and healthy.")
            except Exception as e:
                logger.error(f"Failed to (re)build Influx client: {e}")
                raise

    def _safe_write_point(self, point: Point):
        try:
            self.db_write_api.write(bucket=self.INFLUXDB_BUCKET, org=self.INFLUXDB_ORG, record=point)
            return True
        except Exception as e:
            logger.error(f"Influx write failed: {e}. Rebuilding client and retrying once...")
            try:
                self._build_influx_client()
                self.db_write_api.write(bucket=self.INFLUXDB_BUCKET, org=self.INFLUXDB_ORG, record=point)
                logger.info("Retry write succeeded.")
                return True
            except Exception as e2:
                logger.error(f"Retry write failed: {e2}")
                return False
            
    def _set_lap(self):
        try:
            lap = int(request.args.get('lap', 0))
            if lap <= 0:
                return jsonify({'success': False, 'error': 'Lap must be non-negative'}), 400
            self.jumlah_lap_now = lap
            return jsonify({'success': True, 'message': f'Lap set to {lap}'})
        except ValueError:
            return jsonify({'success': False, 'error': 'Invalid lap value'}), 400
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500
    
    def _build_influx_client_method(self):
        try:
            self._build_influx_client()
            return jsonify({'success': True, 'message': 'InfluxDB client rebuilt successfully'})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500
    
    def _sync_lap(self):
        self.jumlah_lap_now = self.lap_global;
        return jsonify({'lap': self.jumlah_lap_now})
    
    def _get_log(self):
        wib_dt = datetime.now()
        year = wib_dt.strftime('%Y')
        month = wib_dt.strftime('%m')  # Format bulan dalam angka (e.g., '09')
        date_string = wib_dt.strftime('%Y-%m-%d')
        home_dir = Path.home()
        target_dir = home_dir / 'towing_logs' / year / month
        file_path = target_dir / f'towing_status_{date_string}.csv'

        # Get url parameter 'lines', default 10
        lines = int(request.args.get('lines', 10))

        if not file_path.exists():
            return jsonify({'success': False, 'error': 'Log file not found'}), 404
        try:
            self.log_file_mutex.acquire()
            with open(file_path, 'r', encoding='utf-8') as csvfile:
                all_lines = csvfile.readlines()
                header = all_lines[0].strip().split(',')
                data_lines = all_lines[1:]  # Exclude header
                last_lines = data_lines[-lines:] if len(data_lines) >= lines else data_lines
                result = [dict(zip(header, line.strip().split(','))) for line in last_lines]
            self.log_file_mutex.release()
            return jsonify({'success': True, 'data': result})
        except Exception as e:
            self.log_file_mutex.release()
            logger.error(e)
            return jsonify({'success': False, 'error': str(e)}), 500


    def _get_battery_endpoint(self):
        total = self.battery_global
        return jsonify(total)
    
    def _shift_sum_endpoint(self):
        # total = self.current_shift_sum()
        total = self.current_shift_sum_azzam()

        return jsonify(total)   # body: 123
        
    def write_sequently(self, fields, values):
        try:
            # Start with the base point (measurement name)
            point = Point(self.ROBOT_NAME).tag("robot_name", self.ROBOT_NAME)

            # Add fields dynamically from the lists
            for field, value in zip(fields, values):
                point = point.field(field, value)
            
            point = point.time(datetime.utcnow())


            ok = self._safe_write_point(point)
            if ok:
                logger.info("Written to InfluxDB")
            else:
                self.error_counter += 1

            # self.db_write_api.write(bucket=self.INFLUXDB_BUCKET, org=self.INFLUXDB_ORG, record=point)
            # logger.info(f"Written to InfluxDB")
        except Exception as e:
            logger.error(e)
            self.error_counter += 1

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

    def _wib_now(self) -> datetime:
        return datetime.now(tz=self.WIB_TZ)
    
    def current_shift_sum_azzam(self) -> int:
        """If now is 06:00-17:59 → day; else night (anchored correctly across midnight)."""
        now = self._wib_now()
        if 6 <= now.hour < 18:
            self.now_is_day_or_night = 0
        else:
            self.now_is_day_or_night = 1

        if self.now_is_day_or_night != self.prev_now_is_day_or_night:
            # Shift berubah, reset prev_lap_program ke lap_global
            # self.prev_lap_program = self.lap_global
            self.jumlah_lap_now = 0
        
        d_lap_wtf = 0
        if self.lap_global > 0:
            d_lap_wtf = self.lap_global - self.prev_lap_program
        
        if d_lap_wtf < 0:
            d_lap_wtf = self.lap_global
        
        self.jumlah_lap_now += d_lap_wtf

        self.prev_lap_program = self.lap_global
        self.prev_now_is_day_or_night = self.now_is_day_or_night

        return self.jumlah_lap_now

    def register_routes(self):
        self.app.add_url_rule("/lap-sum", view_func=self._shift_sum_endpoint, methods=["GET"])
        self.app.add_url_rule("/battery", view_func=self._get_battery_endpoint, methods=["GET"])
        self.app.add_url_rule("/log", view_func=self._get_log, methods=["GET"])
        self.app.add_url_rule("/sync-lap", view_func=self._sync_lap, methods=["GET"])
        self.app.add_url_rule("/influx-restart", view_func=self._build_influx_client_method, methods=["GET"])
        self.app.add_url_rule("/set-lap", view_func=self._set_lap, methods=["GET"])

    def thread_flask(self):
        self.app.run(host="0.0.0.0", port=int(3002))
        
    # --- Runner ---
    def run(self, host='0.0.0.0', port=3002, debug=False):
        logger.info(f"Logger server running at http://{host}:{port}")
        self.app.run(host=host, port=port, debug=debug)

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
                    # logger.info(f"Rcvd {addr}: {asdict(self.t2)}")
        except BlockingIOError:
            pass

        isImportantWarning = str(self.t2.warning) != "Towing Normal"

        ####################################################
        #              Logging and InfluxDB                #
        ####################################################
        if not self.has_get_last_log_data:
            last_lap = self.get_last_log_data()
            self.prev_lap_program = last_lap
            self.jumlah_lap_now = last_lap

        ####################################################
        #              Logging and InfluxDB                #
        ####################################################
        logger.info(f"Terminal {self.t2.terminal} {self.last_terminal}, Warning {self.t2.warning} {self.last_warning}")
        if((self.t2.terminal != self.last_terminal) or (isImportantWarning and str(self.t2.warning) != self.last_warning)):
            # logger.info(f"name {self.t2.name}, soc {self.t2.soc}, pose_x {self.t2.pose_x}, pose_y {self.t2.pose_y}, pose_theta {self.t2.pose_theta}, terminal {self.t2.terminal}, warning {self.t2.warning}, lap {self.t2.lap}, ts_ms {self.t2.ts_ms}")
            self.last_terminal = self.t2.terminal
            self.last_warning = str(self.t2.warning)
            self.log_data()
            self.upload_influxdb(True)
        else:
            if time_now_ms - self.last_time_influx_db > 5000:
                self.last_time_influx_db = time_now_ms 
                self.upload_influxdb(False)        
        ####################################################

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

        if time_now_ms - self.last_time_calculate_shift > 5000:
            self.lap_global = self.t2.lap
            self.current_shift_sum_azzam()
            self.last_time_calculate_shift = time_now_ms
            shift_sum_msg = Int16()
            shift_sum_msg.data = self.jumlah_lap_now
            self.pub_t2_lap_sum.publish(shift_sum_msg)

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

    def get_last_log_data(self):
        ####################################################
        #                   Data Logging                   #
        ####################################################
        year = self._wib_now().strftime('%Y')
        month = self._wib_now().strftime('%m')
        date_string = self._wib_now().strftime('%Y-%m-%d')
        time_part = self._wib_now().strftime('%H:%M:%S')
        # logger.info(f"Current Year-Month: {year}-{month}-{date_string}")
        home_dir = Path.home()
        target_dir = home_dir / 'towing_logs' / year / month
        target_dir.mkdir(parents=True, exist_ok=True)       
        # logger.info(f"Target dir: {target_dir}")
        file_path = target_dir / f'towing_status_{date_string}.csv'
        file_exists = file_path.exists()
        self.has_get_last_log_data = True
        open_yesterday_file = False

        # time_part = "12:00:00"  # DEBUGGING
        # file_exists = False  # DEBUGGING

        if time_part < "06:00:00" and not file_exists:
            date_string_yesterday = (self._wib_now() - timedelta(days=1)).strftime('%Y-%m-%d')
            file_path = target_dir / f'towing_status_{date_string_yesterday}.csv'
            file_exists = file_path.exists()
            logger.info(f"Trying to read yesterday's log file: {file_path}, exists: {file_exists}")
            open_yesterday_file = file_exists

        # read last line of csv
        last_line = None
        if file_exists:
            with open(file_path, 'r', newline='', encoding='utf-8') as csvfile:
                reader = csv.reader(csvfile)
                rows = list(reader)
                if len(rows) > 1:
                    last_line = rows[-1]  # last line
                    # logger.info(f"Last line: {last_line}")
                    last_lap = 0
                    if last_line[4].isdigit():
                        last_lap = int(last_line[4])
                        last_time = last_line[0]

                        # last_time = "21:00:00"  # DEBUGGING

                        a = time_part > "06:00:00" and time_part < "18:00:00" # Sesi Pagi (1) atau Sesi Malam (0)
                        b = last_time > "06:00:00" and last_time < "18:00:00" # Last sesi Pagi (1) atau Last sesi Malam (0)
                        c = last_time > time_part                             # Waktu logger lebih cepat (1) atau lebih lambat (0)
                        d = open_yesterday_file                               # Buka file kemarin (1) atau hari ini (0)

                        logger.info(f"{a} {b} {c} {d}")

                        # a	| b	| c	| d	| valid
                        # 1	  1	  1	  1		0
                        # 1	  1	  1	  0		0
                        # 1	  1	  0	  1		0
                        # 1	  1	  0	  0		1
                        # 1	  0	  1	  1		0
                        # 1	  0	  1	  0		0
                        # 1	  0	  0	  1		0
                        # 1	  0	  0	  0		0
                        # 0	  1	  1	  1		0
                        # 0	  1	  1	  0		0
                        # 0	  1	  0	  1		0
                        # 0	  1	  0	  0		0
                        # 0	  0	  1	  1		1
                        # 0	  0	  1	  0		0
                        # 0	  0	  0	  1		0
                        # 0	  0	  0	  0		1
                        
                        if a and b and (not c) and (not d):
                            logger.info(f"Shift Pagi valid last lap {last_lap}")
                            return last_lap
                        elif (not a) and (not b) and last_time > "18:00:00" and c and d:
                            logger.info(f"Shift Malam valid last lap kemarin {last_lap}")
                            return last_lap 
                        elif (not a) and (not b) and (last_time > "18:00:00" or time_part < "06:00:00") and (not c) and (not d):
                            logger.info(f"Shift Malam valid last lap hari ini {last_lap}")
                            return last_lap
                        else:
                            logger.info(f"Invalid last lap data, resetting to 0")
                            return 0
                else:
                    logger.info(f"File {file_path} is empty.")
                    return 0
        else:
            logger.info(f"File {file_path} does not exist.")
            return 0

    def log_data(self):
        ####################################################
        #                   Data Logging                   #
        ####################################################
        year = self._wib_now().strftime('%Y')
        month = self._wib_now().strftime('%m')
        date_string = self._wib_now().strftime('%Y-%m-%d')
        time_part = self._wib_now().strftime('%H:%M:%S')
        # logger.info(f"Current Year-Month: {year}-{month}-{date_string}")
        home_dir = Path.home()
        target_dir = home_dir / 'towing_logs' / year / month
        target_dir.mkdir(parents=True, exist_ok=True)       
        # logger.info(f"Target dir: {target_dir}")
        file_path = target_dir / f'towing_status_{date_string}.csv'
        file_exists = file_path.exists()

        self.log_file_mutex.acquire()
        with open(file_path, 'a', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            header = ['Time', 'Terminal', 'Warning', 'Soc', 'Lap', 'PosX', 'PosY', 'PosTheta']

            if not file_exists:
                writer.writerow(header)

            # Tulis baris data baru
            writer.writerow([time_part, self.t2.terminal, self.t2.warning, self.t2.soc, self.t2.lap, f'{self.t2.pose_x:.3f}', f'{self.t2.pose_y:.3f}', f'{self.t2.pose_theta:.3f}'])
        self.log_file_mutex.release()
    
    def upload_influxdb(self, upload_all=False):
        ####################################################
        #                Upload to InfluxDB                #
        ####################################################
        time_part = self._wib_now().strftime('%H:%M:%S')
        if self.t2.lap < 1:
            self.t2.lap = 0

        turunan_lap = self.t2.lap - self.last_lap
        self.last_lap = self.t2.lap
        if turunan_lap < 0:
            turunan_lap = 0  # Hindari penurunan lap negatif

        if turunan_lap > 0 and turunan_lap < 1:
            turunan_lap = 1

        turunan_lap_int = int(turunan_lap)

        # logger.info(f"Lap {self.t2.lap}, dLap {turunan_lap_int}")

        # Selalu rebuild InfluxDB client sebelum menulis
        try:
            # logger.info(f"Writing to InfluxDB: lap={self.t2.lap}, d_lap={turunan_lap_int}")
            # self._build_influx_client()

            # # wait a bit 
            # time.sleep(0.3)
            logger.info(f"{self.get_str_terminal(self.t2.terminal)}, {self.get_str_warning(self.t2.warning)}")
            terminal_str = self.get_str_terminal(self.t2.terminal)
            warning_str, offset = self.get_str_warning(self.t2.warning)
            offset = offset * 1000
            warning_pos_x = self.t2.pose_x
            warning_pos_y = self.t2.pose_y

            if self.t2.pose_x > 0:
                warning_pos_x += offset
            else:
                warning_pos_x -= offset

            if self.t2.pose_y > 0:
                warning_pos_y += offset
            else:
                warning_pos_y -= offset

            if upload_all:
                self.write_sequently(
                    fields=['lap', 'd_lap', 'soc', 'pos_x', 'pos_y', 'pos_theta', 'terminal', 'warning', 'warning_pos_x', 'warning_pos_y', 'timestamp'],
                    values=[self.t2.lap, turunan_lap_int, self.t2.soc, self.t2.pose_x, self.t2.pose_y, self.t2.pose_theta, terminal_str, warning_str, warning_pos_x, warning_pos_y, time_part]
                )
            else:
                self.write_sequently(
                    fields=['pos_x', 'pos_y', 'pos_theta', 'timestamp'],
                    values=[self.t2.pose_x, self.t2.pose_y, self.t2.pose_theta, time_part]
                )
        except Exception as e:
            logger.error(f"Error writing to InfluxDB: {e}")

    def get_str_warning(self, warning_code):
        # 0b00000000 -> 0 = Manual
        # 0b00000001 -> 1 = Auto
        # 0b00000010 -> 2 = Lidar Depan Mendeteksi
        # 0b00000100 -> 3 = Kamera Mendeteksi Obstacle
        # 0b00001000 -> 4 = Gyro Anomali Terdeteksi
        # 0b00010000 -> 5 = ICP Score Terlalu Besar
        # 0b00100000 -> 6 = ICP Translate Terlalu Besar
        # 0b01000000 -> 7 = Berhenti Karena Obstacle
        # 0b100000000 -> 9 = Gandengan Lepas (Tidak Ada Toribe)

        if (warning_code & STATUS_TOWING_ACTIVE_AUTO) == 0:
            return "Towing Mode Manual", 0
        elif (warning_code & EMERGENCY_STOP_KARENA_OBSTACLE) == EMERGENCY_STOP_KARENA_OBSTACLE:
            return "WARNING: Towing Berhenti Karena Obstacle", 7
        elif (warning_code & EMERGENCY_GYRO_ANOMALY_DETECTED) == EMERGENCY_GYRO_ANOMALY_DETECTED:
            return "WARNING: Gyro Anomali Terdeteksi", 4
        elif (warning_code & EMERGENCY_ICP_SCORE_TERLALU_BESAR) == EMERGENCY_ICP_SCORE_TERLALU_BESAR:
            return "WARNING: Anomali Lingkungan Terdeteksi", 5
        elif (warning_code & EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR) == EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR:
            return "WARNING: Hipotesis Kesalahan Posisi", 6
        elif (warning_code & EMERGENCY_GANDENGAN_LEPAS) == EMERGENCY_GANDENGAN_LEPAS:
            return "WARNING: TIDAK ADA TORIBE", 9
        elif (warning_code & EMERGENCY_LIDAR_DEPAN_DETECTED) == EMERGENCY_LIDAR_DEPAN_DETECTED:
            return "WARNING: Lidar Mendeteksi Objek", 2
        elif (warning_code & EMERGENCY_CAMERA_OBS_DETECTED) == EMERGENCY_CAMERA_OBS_DETECTED:
            return "WARNING: Kamera Mendeteksi Objek", 3
        else:
            return "Towing Normal", 1
        
    def get_str_terminal(self, terminal_code):
        if terminal_code == -1:
            return "Terminal Terakhir: Tempat Parkir"
        elif terminal_code == 0:
            return "Terminal Terakhir: Area Jibcrane IST (Berangkat)"
        elif terminal_code == 1:
            return "Terminal Terakhir: Tikungan Samping Yokai 1 (Berangkat)"
        elif terminal_code == 3:
            return "Terminal Terakhir: Jalur 1 Lurus Samping Yokai 1 (Berangkat)"
        elif terminal_code == 5 or terminal_code == 6:
            return "Terminal Terakhir: Jalur 1 Tikungan Bawah Tangga (Berangkat)"
        elif terminal_code == 7:
            return "Terminal Terakhir: Degasing (Pulang)"
        elif terminal_code == 11:
            return "Terminal Terakhir: Jalur 1 Lurus Setelah Bawah Tangga (Berangkat)"
        elif terminal_code == 15:
            return "Terminal Terakhir: Jalur 1 Tikungan Beacukai (Berangkat)"
        elif terminal_code == 19:
            return "Terminal Terakhir: Jalur 1 Lurus Setelah Beacukai (Berangkat)"
        elif terminal_code == 23 or terminal_code == 46:
            return "Terminal Terakhir: Degasing (Berangkat)"
        elif terminal_code == 24:
            return "Terminal Terakhir: Jalur 1 Lurus Depan Yokai 1 (Berangkat)"
        elif terminal_code == 25 or terminal_code == 47:
            return "Terminal Terakhir: Degasing (Pulang)"
        elif terminal_code == 26:
            return "Terminal Terakhir: Tikungan Samping Lab (Berangkat)"
        elif terminal_code == 35:
            return "Terminal Terakhir: Jalur 2 Lurus Setelah Beacukai (Pulang)"
        elif terminal_code == 37:
            return "Terminal Terakhir: Jalur 2 Tikungan Beacukai (Pulang)"
        elif terminal_code == 38:
            return "Terminal Terakhir: Jalur 2 Lurus Sebelum Beacukai (Pulang)"
        elif terminal_code == 40:
            return "Terminal Terakhir: Jalur 2 Tikungan Bawah Tangga (Pulang)"
        elif terminal_code == 41:
            return "Terminal Terakhir: Jalur 2 Lurus Samping Yokai 1 (Pulang)"
        elif terminal_code == 43 or terminal_code == 48:
            return "Terminal Terakhir: Jalur 2 Tikungan Jembatan Penyebrangan (Pulang)"

def main(args=None):
    rclpy.init(args=args)

    node_udp2roslib = UDP2ROSLIB()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_udp2roslib)
    executor.spin()
    
if __name__ == '__main__':
    main(sys.argv)