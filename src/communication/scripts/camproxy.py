#!/usr/bin/python3

from flask import Flask, request, jsonify, Response, stream_with_context
from flask_cors import CORS
import requests

import rclpy
from rclpy.node import Node 

from loguru import logger
import sys

class camproxy(Node):
    def __init__(self):
        super().__init__('camproxy')
        self.get_logger().info("camproxy Node has been started.")

        # Logger
        # ------
        logger.remove()
        logger.add(
            sys.stdout,
            colorize=True,
            format="<green>{time:HH:mm:ss.SSS}</green> | <level>{level:^6}</level> | <cyan>{function}</cyan>:<cyan>{line}</cyan> - {message}",
            enqueue=True,
        )

        self.CAMERA_URL = "http://10.20.30.41/cgi-bin/mjpg/video.cgi?channel=1&subtype=1"
        self.CAMERA_USER = "admin"
        self.CAMERA_PASS = "Ptist@2025"

        # Setup Flask app
        self.app = Flask(__name__)
        CORS(self.app)
        self.register_routes()

    
    def camera_proxy(self):
        """Proxy MJPEG stream from the camera to the browser."""
        try:
            r = requests.get(
                self.CAMERA_URL,
                auth=(self.CAMERA_USER, self.CAMERA_PASS),
                stream=True,
                verify=False,
                timeout=10
            )
        except Exception as e:
            logger.error(f"Failed to connect to camera: {e}")
            return Response("Camera not reachable", status=502)

        # Use the camera’s actual content type (important for MJPEG boundary!)
        content_type = r.headers.get("Content-Type", "multipart/x-mixed-replace; boundary=--frame")

        return Response(
            stream_with_context(r.iter_content(chunk_size=1024)),
            content_type=content_type
        )
    
    def register_routes(self):
        self.app.add_url_rule("/camera", view_func=self.camera_proxy, methods=["GET"])

def main(args=None):
    rclpy.init(args=args)

    node_camproxy = camproxy()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_camproxy)
    executor.spin()

if __name__ == '__main__':
    main(sys.argv)