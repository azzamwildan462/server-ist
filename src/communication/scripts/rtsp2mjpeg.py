#!/usr/bin/env python3
"""
rtsp2mjpeg.py — 3× RTSP → MJPEG + ROS2 + watchdog + force-close MJPEG on upstream stall

- Tiap stream: ffmpeg subprocess → parsing JPEG → publish ROS CompressedImage
- Flask endpoint: /cam1.mjpeg, /cam2.mjpeg, /cam3.mjpeg
- Watchdog:
  * jika tak ada frame > stale_timeout_sec → kill ffmpeg (restart) + PUTUSKAN SEMUA KLIEN MJPEG stream tsb
  * klien (JS) bisa onerror/setInterval → otomatis request ulang endpoint
"""

import os
import signal
import subprocess
import threading
import time
from collections import deque
from typing import List, Optional
from urllib.parse import quote

import rclpy
from flask import Flask, Response, stream_with_context
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from werkzeug.serving import make_server


# ---------------- FFmpeg helpers ----------------
def build_rtsp_url(
    host: str, path: str, username: str = "", password: str = "", port: int = 554
) -> str:
    auth = ""
    if username:
        auth = f"{quote(username, safe='')}:{quote(password or '', safe='')}@"
    return f"rtsp://{auth}{host}:{port}/{path.lstrip('/')}"


def build_ffmpeg_cmd(
    rtsp_url: str, fps: int = 10, quality: int = 7, tcp: bool = True
) -> List[str]:
    # cmd = ["ffmpeg", "-nostdin", "-loglevel", "error"]
    # # optional reconnect flags (silent if unsupported)
    # # cmd += ["-reconnect", "1", "-reconnect_streamed", "1", "-reconnect_delay_max", "5"]
    # if tcp:
    #     cmd += ["-rtsp_transport", "tcp", "-rtsp_flags", "prefer_tcp"]
    # cmd += ["-stimeout", "5000000", "-rw_timeout", "5000000"]  # 5s (µs)
    # cmd += ["-i", rtsp_url, "-an", "-sn", "-f", "mjpeg", "-q:v", str(quality), "-r", str(fps), "pipe:1"]

    cmd = [
        "ffmpeg",
        "-nostdin",
        "-loglevel",
        "error",
        "-rtsp_transport",
        "tcp",
        "-stimeout",
        "15000000",  # 15s I/O timeout (beberapa build mendukung opsi ini)
        "-i",
        rtsp_url,
        "-an",  # tanpa audio
        "-sn",
        "-f",
        "mjpeg",
        "-q:v",
        "7",  # kualitas JPEG (1 terbaik, 31 paling rendah)
        "-r",
        "10",  # target fps
        "-",
    ]

    return cmd


def jpeg_frames_from_pipe(
    proc: subprocess.Popen, stop_flag: threading.Event, buf_limit: int = 2_000_000
):
    read = proc.stdout.read  # type: ignore[attr-defined]
    buf = bytearray()
    SOI, EOI = b"\xff\xd8", b"\xff\xd9"
    while not stop_flag.is_set():
        chunk = read(4096)
        if not chunk:
            break
        buf.extend(chunk)
        while True:
            s = buf.find(SOI)
            if s < 0:
                if len(buf) > buf_limit:
                    buf.clear()
                break
            e = buf.find(EOI, s + 2)
            if e < 0:
                if s > 0:
                    del buf[:s]
                break
            frame = bytes(buf[s : e + 2])
            del buf[: e + 2]
            yield frame


# ---------------- MJPEG Hub (with reset) ----------------
class MjpegHub:
    """
    Frame cache + reset broadcast.
    - set_frame(i, jpeg): update frame
    - generator(i,...): stream multipart; akan BREAK jika reset_event set → koneksi diputus
    - trigger_reset(i): set event → semua generator(i) putus → klien reconnect
    """

    def __init__(self, n: int):
        self._n = n
        self._latest: List[Optional[bytes]] = [None] * n
        self._last_ts: List[float] = [0.0] * n
        self._conds = [threading.Condition() for _ in range(n)]
        self._reset_events = [threading.Event() for _ in range(n)]

    def set_frame(self, idx: int, jpeg: bytes):
        with self._conds[idx]:
            self._latest[idx] = jpeg
            self._last_ts[idx] = time.time()
            self._conds[idx].notify_all()

    def last_update(self, idx: int) -> float:
        return self._last_ts[idx]

    def trigger_reset(self, idx: int):
        # signal semua klien utk putus; lalu clear agar sesi baru bisa berjalan
        self._reset_events[idx].set()

    def clear_reset(self, idx: int):
        self._reset_events[idx].clear()

    def generator(self, idx: int, fps_limit: Optional[float] = None):
        boundary = b"--frame"
        self.clear_reset(idx)
        last_send = 0.0
        while not self._reset_events[idx].is_set():
            with self._conds[idx]:
                # tunggu frame baru atau timeout kecil biar bisa cek reset
                self._conds[idx].wait(timeout=0.5)
                jpeg = self._latest[idx]

            if self._reset_events[idx].is_set():
                break
            if jpeg is None:
                continue

            if fps_limit:
                now = time.time()
                min_dt = 1.0 / fps_limit
                if now - last_send < min_dt:
                    time.sleep(max(0.0, min_dt - (now - last_send)))
                last_send = time.time()

            yield boundary + b"\r\n" + b"Content-Type: image/jpeg\r\n" + f"Content-Length: {len(jpeg)}\r\n\r\n".encode(
                "ascii"
            ) + jpeg + b"\r\n"
        # break → Response close → klien onerror / interval → set src lagi


# ---------------- Stream Worker (watchdog triggers reset) ----------------
class StreamWorker:
    def __init__(
        self,
        node: Node,
        name: str,
        rtsp_url: str,
        topic: str,
        hub: MjpegHub,
        hub_idx: int,
        fps: int,
        quality: int,
        stale_timeout_sec: float,
        watchdog_period_sec: float,
    ):
        self.node = node
        self.name = name
        self.rtsp_url = rtsp_url
        self.topic = topic
        self.hub = hub
        self.hub_idx = hub_idx
        self.fps = fps
        self.quality = quality
        self.stale_timeout = stale_timeout_sec
        self.watchdog_period = watchdog_period_sec

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = node.create_publisher(CompressedImage, topic, qos)

        self.proc: Optional[subprocess.Popen] = None
        self.thread: Optional[threading.Thread] = None
        self.wd_thread: Optional[threading.Thread] = None
        self.stop_flag = threading.Event()
        self._lock = threading.Lock()
        self._last_frame_time = 0.0

        self._stderr_tail = deque(maxlen=50)
        self._stderr_thread: Optional[threading.Thread] = None

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.stop_flag.clear()
        self.thread = threading.Thread(
            target=self._run_loop, name=f"{self.name}-reader", daemon=True
        )
        self.thread.start()
        self.wd_thread = threading.Thread(
            target=self._watchdog_loop, name=f"{self.name}-watchdog", daemon=True
        )
        self.wd_thread.start()

    def stop(self):
        self.stop_flag.set()
        with self._lock:
            if self.proc:
                try:
                    self.proc.terminate()
                    self.proc.wait(timeout=2.0)
                except Exception:
                    try:
                        self.proc.kill()
                    except Exception:
                        pass
            self.proc = None
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.wd_thread:
            self.wd_thread.join(timeout=2.0)

    def _spawn_proc(self) -> subprocess.Popen:
        cmd = build_ffmpeg_cmd(
            self.rtsp_url, fps=self.fps, quality=self.quality, tcp=True
        )
        self.node.get_logger().info(f"[{self.name}] launching ffmpeg: {' '.join(cmd)}")
        p = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            preexec_fn=os.setsid,
        )
        # tail stderr
        self._stderr_thread = threading.Thread(
            target=self._read_stderr, args=(p,), daemon=True
        )
        self._stderr_thread.start()
        return p

    def _read_stderr(self, proc: subprocess.Popen):
        try:
            for line in iter(proc.stderr.readline, b""):
                self._stderr_tail.append(line.decode("utf-8", "ignore").rstrip())
        except Exception:
            pass

    def _dump_tail(self):
        if self._stderr_tail:
            tail = "\n".join(list(self._stderr_tail)[-10:])
            self.node.get_logger().warn(f"[{self.name}] ffmpeg stderr tail:\n{tail}")

    def _run_loop(self):
        backoff = 0.5
        while not self.stop_flag.is_set():
            try:
                with self._lock:
                    self.proc = self._spawn_proc()
                for jpeg in jpeg_frames_from_pipe(self.proc, self.stop_flag):
                    if self.stop_flag.is_set():
                        break
                    self._last_frame_time = time.time()
                    # publish
                    msg = CompressedImage()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.header.frame_id = self.name
                    msg.format = "jpeg"
                    msg.data = jpeg
                    self.pub.publish(msg)
                    # hub update
                    self.hub.set_frame(self.hub_idx, jpeg)

                # keluar dari generator → cek status proses
                rc = self.proc.poll() if self.proc else None
                if rc is None:
                    self.node.get_logger().warn(
                        f"[{self.name}] stdout EOF while ffmpeg still running (rc=None). Killing & restarting..."
                    )
                else:
                    self.node.get_logger().warn(
                        f"[{self.name}] ffmpeg exited rc={rc}; restarting..."
                    )

                self._dump_tail()
                # kill proc group
                with self._lock:
                    if self.proc:
                        try:
                            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                        except Exception:
                            pass
                time.sleep(backoff)
                backoff = min(backoff * 2.0, 10.0)
            except Exception as e:
                self.node.get_logger().error(f"[{self.name}] loop exception: {e}")
                time.sleep(backoff)
                backoff = min(backoff * 2.0, 10.0)
            finally:
                with self._lock:
                    if self.proc:
                        try:
                            self.proc.wait(timeout=1.0)
                        except Exception:
                            try:
                                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                            except Exception:
                                pass
                    self.proc = None

    def _watchdog_loop(self):
        while not self.stop_flag.is_set():
            now = time.time()
            idle = (
                now - self._last_frame_time if self._last_frame_time else float("inf")
            )
            if idle > self.stale_timeout:
                # 1) log
                self.node.get_logger().warn(
                    f"[{self.name}] stale {idle:.1f}s > {self.stale_timeout}s → reset ffmpeg + drop MJPEG clients"
                )
                # 2) force-close semua klien MJPEG di stream ini (biar browser/JS reconnect)
                self.hub.trigger_reset(self.hub_idx)
                # 3) kill ffmpeg supaya reader restart (run_loop akan detect)
                with self._lock:
                    if self.proc:
                        try:
                            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                        except Exception:
                            pass
                # 4) reset timer supaya tidak spam
                self._last_frame_time = time.time()
            time.sleep(self.watchdog_period)


# ---------------- Flask server wrapper ----------------
class FlaskServerThread(threading.Thread):
    def __init__(self, app: Flask, host="0.0.0.0", port=7890):
        super().__init__(daemon=True)
        self.server = make_server(host, port, app, threaded=True)
        self.ctx = app.app_context()
        self.ctx.push()

    def run(self):
        self.server.serve_forever()

    def shutdown(self):
        try:
            self.server.shutdown()
        except Exception:
            pass


# ---------------- ROS node ----------------
class Rtsp2MjpegNode(Node):
    def __init__(self):
        super().__init__("rtsp2mjpeg")

        # Params (Hikvision tip: path case-sensitive → "Streaming/Channels/101")
        self.declare_parameter("hosts", ["192.168.1.64", "", ""])
        self.declare_parameter(
            "paths",
            [
                "streaming/channels/101",
                "streaming/channels/101",
                "streaming/channels/101",
            ],
        )
        self.declare_parameter("ports", [554, 554, 554])
        self.declare_parameter("usernames", ["admin", "admin", "admin"])
        self.declare_parameter("passwords", ["098POI765uyt", "098POI765uyt", "098POI765uyt"])

        self.declare_parameter(
            "topics",
            [
                "/cam1/image_raw/compressed",
                "/cam2/image_raw/compressed",
                "/cam3/image_raw/compressed",
            ],
        )
        self.declare_parameter("fps", 10)
        self.declare_parameter("quality", 7)

        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 7890)
        self.declare_parameter("mjpeg_fps_limit", 20)

        self.declare_parameter("stale_timeout_sec", 35.0)
        self.declare_parameter("watchdog_period_sec", 2.0)

        hosts = list(
            self.get_parameter("hosts").get_parameter_value().string_array_value
        )
        paths = list(
            self.get_parameter("paths").get_parameter_value().string_array_value
        )
        ports = list(
            self.get_parameter("ports").get_parameter_value().integer_array_value
        )
        users = list(
            self.get_parameter("usernames").get_parameter_value().string_array_value
        )
        pwds = list(
            self.get_parameter("passwords").get_parameter_value().string_array_value
        )
        fps = int(self.get_parameter("fps").value)
        quality = int(self.get_parameter("quality").value)

        urls = [
            build_rtsp_url(hosts[i], paths[i], users[i], pwds[i], ports[i])
            for i in range(3)
        ]
        topics = list(
            self.get_parameter("topics").get_parameter_value().string_array_value
        )
        topics = (topics + [""] * 3)[:3]

        self.hub = MjpegHub(n=3)

        # workers
        stale = float(self.get_parameter("stale_timeout_sec").value)
        wd = float(self.get_parameter("watchdog_period_sec").value)
        self.workers: List[StreamWorker] = []
        for i in range(3):
            if not hosts[i] or not paths[i]:
                self.get_logger().info(f"Stream {i+1} disabled (no host/path)")
                continue
            w = StreamWorker(
                self,
                f"stream{i+1}",
                urls[i],
                topics[i],
                hub=self.hub,
                hub_idx=i,
                fps=fps,
                quality=quality,
                stale_timeout_sec=stale,
                watchdog_period_sec=wd,
            )
            self.workers.append(w)
            w.start()

        # Flask
        host = str(self.get_parameter("http_host").value)
        port = int(self.get_parameter("http_port").value)
        self.mjpeg_limit = int(self.get_parameter("mjpeg_fps_limit").value)

        self.app = self._build_app()
        self.http = FlaskServerThread(self.app, host=host, port=port)
        self.get_logger().info(f"HTTP MJPEG on http://{host}:{port}")
        self.http.start()

        self.timer = self.create_timer(5.0, self._heartbeat)

    def _build_app(self) -> Flask:
        app = Flask(__name__)
        hub = self.hub
        limit = self.mjpeg_limit

        @app.get("/")
        def index():
            return (  # JS onerror + interval (opsional redundant—reset hub sudah memutus koneksi)
                "<h3>RTSP→MJPEG</h3>"
                "<div><img id='c1' src='/cam1.mjpeg' width='360'></div>"
                "<div><img id='c2' src='/cam2.mjpeg' width='360'></div>"
                "<div><img id='c3' src='/cam3.mjpeg' width='360'></div>"
                "<script>"
                "function bump(id){const el=document.getElementById(id); el.src='/'+id.replace('c','cam')+'.mjpeg?ts='+Date.now();}"
                "['c1','c2','c3'].forEach(id=>{"
                "  document.getElementById(id).addEventListener('error', ()=>bump(id));"
                "  setInterval(()=>bump(id), 15000);"
                "});"
                "</script>"
            )

        def stream(idx: int):
            gen = hub.generator(idx, fps_limit=limit)
            return Response(
                stream_with_context(gen),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @app.get("/cam1.mjpeg")
        def cam1():
            return stream(0)

        @app.get("/cam2.mjpeg")
        def cam2():
            return stream(1)

        @app.get("/cam3.mjpeg")
        def cam3():
            return stream(2)

        return app

    def _heartbeat(self):
        stats = []
        for i, w in enumerate(self.workers):
            age = time.time() - (self.hub.last_update(i) or 0.0)
            stats.append(f"{w.name}: age={age:.1f}s")
        self.get_logger().info(" | ".join(stats))

    def destroy_node(self):
        try:
            self.http.shutdown()
        except Exception:
            pass
        for w in self.workers:
            w.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = Rtsp2MjpegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
