import os
import time
import json
import base64
import threading
import queue
import sqlite3
import urllib.request
import math
import numpy as np
import cv2
from datetime import datetime
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple, Any, Union

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from ultralytics import YOLO

try:
    import pythoncom
    from pygrabber.dshow_graph import FilterGraph
except ImportError:
    FilterGraph = None

os.environ["OPENCV_FFMPEG_READ_ATTEMPTS"] = "10000"


class AppState(Enum):
    INITIALIZING = auto()
    IDLE = auto()
    RUNNING = auto()
    ERROR = auto()


class MavlinkCommand(Enum):
    WAYPOINT = 16
    RETURN_TO_LAUNCH = 20


class FODSystemException(Exception):
    pass


class CameraInitializationError(FODSystemException):
    pass


class ModelLoadError(FODSystemException):
    pass


class TelemetryTimeoutError(FODSystemException):
    pass


@dataclass
class TelemetryFrame:
    lat: float = 0.0
    lon: float = 0.0
    altitude: float = 0.0
    heading: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    satellites_visible: int = 0
    timestamp: float = field(default_factory=time.time)


@dataclass
class ConfigurationManager:
    mission_file: str = "D:/fod_mission.waypoints"
    model_path: str = r"D:\FOD\FODSub\runs\detect\yolov8n-obb.pt"
    snapshot_dir: str = "snapshots"
    output_vid_dir: str = "output_videos"
    db_path: str = "fod_telemetry.db"
    telemetry_url: str = "http://127.0.0.1:56781/mavlink/"
    conf_threshold: float = 0.25
    camera_pitch_deg: float = 45.0
    queue_max_size: int = 30

    def __post_init__(self):
        os.makedirs(self.snapshot_dir, exist_ok=True)
        os.makedirs(self.output_vid_dir, exist_ok=True)


class DatabaseHandler:
    def __init__(self, config: ConfigurationManager):
        self.db_path = config.db_path
        self._initialize_schema()

    def _get_connection(self) -> sqlite3.Connection:
        return sqlite3.connect(self.db_path, check_same_thread=False)

    def _initialize_schema(self) -> None:
        query_session = """
        CREATE TABLE IF NOT EXISTS session_metadata (
            session_id INTEGER PRIMARY KEY AUTOINCREMENT,
            start_time DATETIME DEFAULT CURRENT_TIMESTAMP,
            model_used TEXT,
            camera_pitch REAL
        )
        """
        query_telemetry = """
        CREATE TABLE IF NOT EXISTS telemetry_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id INTEGER,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            lat REAL, lon REAL, alt REAL, heading REAL,
            roll REAL, pitch REAL, sats INTEGER,
            FOREIGN KEY(session_id) REFERENCES session_metadata(session_id)
        )
        """
        query_detections = """
        CREATE TABLE IF NOT EXISTS detection_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id INTEGER,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            lat REAL, lon REAL, confidence REAL,
            object_count INTEGER, approach_yaw REAL, image_path TEXT,
            FOREIGN KEY(session_id) REFERENCES session_metadata(session_id)
        )
        """
        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute(query_session)
            cursor.execute(query_telemetry)
            cursor.execute(query_detections)
            conn.commit()

    def start_session(self, model_name: str, pitch: float) -> int:
        query = "INSERT INTO session_metadata (model_used, camera_pitch) VALUES (?, ?)"
        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute(query, (model_name, pitch))
            conn.commit()
            return cursor.lastrowid

    def insert_telemetry(self, session_id: int, frame: TelemetryFrame) -> None:
        query = "INSERT INTO telemetry_log (session_id, lat, lon, alt, heading, roll, pitch, sats) VALUES (?, ?, ?, ?, ?, ?, ?, ?)"
        with self._get_connection() as conn:
            conn.execute(
                query,
                (
                    session_id,
                    frame.lat,
                    frame.lon,
                    frame.altitude,
                    frame.heading,
                    frame.roll,
                    frame.pitch,
                    frame.satellites_visible,
                ),
            )
            conn.commit()

    def insert_detection(
        self,
        session_id: int,
        lat: float,
        lon: float,
        conf: float,
        count: int,
        yaw: float,
        img_path: str,
    ) -> None:
        query = "INSERT INTO detection_log (session_id, lat, lon, confidence, object_count, approach_yaw, image_path) VALUES (?, ?, ?, ?, ?, ?, ?)"
        with self._get_connection() as conn:
            conn.execute(query, (session_id, lat, lon, conf, count, yaw, img_path))
            conn.commit()


class WaypointGenerator:
    def __init__(self, config: ConfigurationManager):
        self.filepath = config.mission_file
        self.write_lock = threading.Lock()
        self._ensure_header()

    def _ensure_header(self) -> None:
        with self.write_lock:
            if not os.path.exists(self.filepath):
                with open(self.filepath, "w", encoding="utf-8") as f:
                    f.write("QGC WPL 110\n")

    def _get_next_index(self) -> int:
        if not os.path.exists(self.filepath):
            return 0
        try:
            with open(self.filepath, "r", encoding="utf-8") as f:
                return len(
                    [
                        line
                        for line in f.readlines()
                        if line.strip() and not line.startswith("QGC WPL")
                    ]
                )
        except Exception:
            return 0

    def write_waypoint(self, lat: float, lon: float, approach_yaw: float) -> None:
        with self.write_lock:
            index = self._get_next_index()
            current_wp = 1 if index == 0 else 0

            line = (
                f"{index}\t{current_wp}\t3\t{MavlinkCommand.WAYPOINT.value}\t"
                f"0\t0\t0\t{approach_yaw:.2f}\t"
                f"{lat:.7f}\t{lon:.7f}\t0.00\t1\n"
            )

            mode = "a" if os.path.exists(self.filepath) else "w"
            with open(self.filepath, mode, encoding="utf-8") as f:
                if mode == "w":
                    f.write("QGC WPL 110\n")
                f.write(line)

    def purge_mission(self) -> None:
        with self.write_lock:
            if os.path.exists(self.filepath):
                os.remove(self.filepath)
            self._ensure_header()


class TelemetryIngestor:
    def __init__(
        self,
        config: ConfigurationManager,
        db_manager: DatabaseHandler,
        socket_io: SocketIO,
    ):
        self.config = config
        self.db = db_manager
        self.sio = socket_io
        self.running = False
        self.current_frame = TelemetryFrame()
        self.state_lock = threading.Lock()
        self.session_id: Optional[int] = None

    def set_session(self, session_id: int) -> None:
        self.session_id = session_id

    def start(self) -> None:
        self.running = True
        threading.Thread(target=self._ingest_loop, daemon=True).start()

    def _ingest_loop(self) -> None:
        last_db_write = time.time()
        while self.running:
            try:
                with urllib.request.urlopen(
                    self.config.telemetry_url, timeout=0.1
                ) as response:
                    data = json.loads(response.read().decode())
                    with self.state_lock:
                        self.current_frame.altitude = (
                            data.get("VFR_HUD", {}).get("msg", {}).get("alt", 0)
                        )
                        self.current_frame.heading = (
                            data.get("VFR_HUD", {}).get("msg", {}).get("heading", 0)
                        )
                        self.current_frame.roll = (
                            data.get("ATTITUDE", {}).get("msg", {}).get("roll", 0)
                        )
                        self.current_frame.pitch = (
                            data.get("ATTITUDE", {}).get("msg", {}).get("pitch", 0)
                        )
                        self.current_frame.lat = (
                            data.get("GPS_RAW_INT", {}).get("msg", {}).get("lat", 0)
                            / 1e7
                        )
                        self.current_frame.lon = (
                            data.get("GPS_RAW_INT", {}).get("msg", {}).get("lon", 0)
                            / 1e7
                        )
                        self.current_frame.satellites_visible = (
                            data.get("GPS_RAW_INT", {})
                            .get("msg", {})
                            .get("satellites_visible", 0)
                        )
                        self.current_frame.timestamp = time.time()

                    self.sio.emit("telemetry_update", self.get_latest_data())

                    if self.session_id and (time.time() - last_db_write > 5.0):
                        self.db.insert_telemetry(self.session_id, self.current_frame)
                        last_db_write = time.time()
            except Exception:
                pass
            time.sleep(0.02)

    def get_latest_data(self) -> Dict[str, Any]:
        with self.state_lock:
            return {
                "lat": self.current_frame.lat,
                "lon": self.current_frame.lon,
                "heading": self.current_frame.heading,
                "pitch": self.current_frame.pitch,
                "altitude": self.current_frame.altitude,
            }

    def stop(self) -> None:
        self.running = False


class SpatialMath:
    @staticmethod
    def calculate_approach_yaw(
        pixel_theta_rad: float, rover_heading: float, camera_pitch_deg: float
    ) -> float:
        cam_pitch_rad = math.radians(camera_pitch_deg)
        ground_theta_rad = math.atan(
            math.tan(pixel_theta_rad) * math.cos(cam_pitch_rad)
        )
        ground_theta_deg = math.degrees(ground_theta_rad)
        approach_yaw = (rover_heading + ground_theta_deg) % 360.0
        return approach_yaw


class VideoPipelineWorker:
    def __init__(
        self,
        config: ConfigurationManager,
        telemetry: TelemetryIngestor,
        wpm: WaypointGenerator,
        db: DatabaseHandler,
        sio: SocketIO,
    ):
        self.config = config
        self.telemetry = telemetry
        self.wpm = wpm
        self.db = db
        self.sio = sio

        self.frame_queue: queue.Queue = queue.Queue(maxsize=self.config.queue_max_size)
        self.running = False
        self.cap: Optional[cv2.VideoCapture] = None

        try:
            self.model = YOLO(self.config.model_path)
        except Exception as e:
            raise ModelLoadError(f"Failed to load YOLO model: {e}")

        self.stats_lock = threading.Lock()
        self.pipeline_stats = {"detected": 0, "fps": 0, "conf": 0.0, "target_yaw": 0.0}
        self.last_write_time = 0.0
        self.session_id: Optional[int] = None

    def set_session(self, session_id: int) -> None:
        self.session_id = session_id

    def start_stream(self, source: Union[int, str]) -> Tuple[bool, str]:
        if self.running:
            self.stop_stream()
        try:
            video_source = int(source) if str(source).isdigit() else source
            self.cap = cv2.VideoCapture(video_source)
            if not self.cap.isOpened():
                raise CameraInitializationError(f"Failed to allocate stream: {source}")

            self.running = True
            threading.Thread(target=self._capture_worker, daemon=True).start()
            threading.Thread(target=self._inference_worker, daemon=True).start()
            return True, "Pipeline active"
        except Exception as e:
            return False, str(e)

    def _capture_worker(self) -> None:
        while self.running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break
            if not self.frame_queue.full():
                self.frame_queue.put(frame)
            else:
                try:
                    self.frame_queue.get_nowait()
                    self.frame_queue.put(frame)
                except queue.Empty:
                    pass
        self.running = False

    def _inference_worker(self) -> None:
        frames_processed = 0
        t0 = time.time()

        while self.running:
            try:
                frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            h, w = frame.shape[:2]
            cw, ch = int(w * 0.5), int(h * 0.5)
            x, y = (w - cw) // 2, (h - ch) // 2
            roi = frame[y : y + ch, x : x + cw]

            det_count = 0
            conf_list: List[float] = []
            target_yaws: List[float] = []
            t_data = self.telemetry.get_latest_data()

            if self.model:
                res = self.model.predict(
                    roi, conf=self.config.conf_threshold, verbose=False
                )
                if res and res[0].obb:
                    for obb in res[0].obb:
                        coords = obb.xyxyxyxy[0].cpu().numpy().astype(int)
                        xywhr = obb.xywhr[0].cpu().numpy()
                        c_val = float(obb.conf[0])
                        theta_rad = float(xywhr[4])

                        app_yaw = SpatialMath.calculate_approach_yaw(
                            theta_rad, t_data["heading"], self.config.camera_pitch_deg
                        )

                        det_count += 1
                        conf_list.append(c_val)
                        target_yaws.append(app_yaw)

                        cv2.polylines(
                            roi, [coords], isClosed=True, color=(0, 255, 0), thickness=2
                        )
                        cv2.putText(
                            roi,
                            f"{c_val:.2f} | Y:{app_yaw:.0f}",
                            (coords[0][0], coords[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2,
                        )

            avg_conf = (sum(conf_list) / len(conf_list) * 100) if conf_list else 0.0
            primary_yaw = target_yaws[0] if target_yaws else 0.0

            frames_processed += 1
            if frames_processed % 10 == 0:
                dt = time.time() - t0
                current_fps = int(frames_processed / dt) if dt > 0 else 0
            else:
                with self.stats_lock:
                    current_fps = self.pipeline_stats["fps"]

            with self.stats_lock:
                self.pipeline_stats = {
                    "detected": det_count,
                    "fps": current_fps,
                    "conf": avg_conf,
                    "target_yaw": primary_yaw,
                }

            if (
                det_count > 0
                and self.session_id
                and (time.time() - self.last_write_time >= 1.0)
            ):
                self.last_write_time = time.time()
                self.wpm.write_waypoint(t_data["lat"], t_data["lon"], primary_yaw)

                fname = f"FOD_lat{t_data['lat']:.6f}_lon{t_data['lon']:.6f}_yaw{primary_yaw:.1f}.jpg"
                fpath = os.path.join(self.config.snapshot_dir, fname)
                cv2.imwrite(fpath, roi)

                self.db.insert_detection(
                    self.session_id,
                    t_data["lat"],
                    t_data["lon"],
                    avg_conf,
                    det_count,
                    primary_yaw,
                    fpath,
                )
                self.sio.emit(
                    "detection_log",
                    {"lat": t_data["lat"], "lon": t_data["lon"], "yaw": primary_yaw},
                )

            _, buffer = cv2.imencode(".jpg", roi, [cv2.IMWRITE_JPEG_QUALITY, 90])
            self.sio.emit(
                "frame_update",
                {
                    "frame": base64.b64encode(buffer).decode("utf-8"),
                    "stats": self.pipeline_stats,
                },
            )

    def stop_stream(self) -> None:
        self.running = False
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break
        if self.cap:
            self.cap.release()
            self.cap = None
        self.sio.emit("detection_stopped")


class ApplicationServer:
    def __init__(self):
        self.config = ConfigurationManager()
        self.app = Flask(__name__)
        self.app.config["SECRET_KEY"] = self.config.flask_secret
        self.sio = SocketIO(self.app, cors_allowed_origins="*")

        self.db_mgr = DatabaseHandler(self.config)
        self.wp_mgr = WaypointGenerator(self.config)
        self.tel_mgr = TelemetryIngestor(self.config, self.db_mgr, self.sio)
        self.vid_pipe = VideoPipelineWorker(
            self.config, self.tel_mgr, self.wp_mgr, self.db_mgr, self.sio
        )

        self._register_routes()

    def _register_routes(self) -> None:
        @self.app.route("/api/start_camera", methods=["POST"])
        def route_start_camera():
            session_id = self.db_mgr.start_session(
                self.config.model_path, self.config.camera_pitch_deg
            )
            self.tel_mgr.set_session(session_id)
            self.vid_pipe.set_session(session_id)

            source = request.json.get("camera_index", 0) if request.json else 0
            success, msg = self.vid_pipe.start_stream(source)
            return jsonify({"success": success, "message": msg})

        @self.app.route("/api/stop")
        def route_stop():
            self.vid_pipe.stop_stream()
            return jsonify({"success": True})

        @self.app.route("/api/clear_gps_log", methods=["POST"])
        def route_clear_log():
            self.wp_mgr.purge_mission()
            return jsonify({"success": True})

    def launch(self) -> None:
        self.tel_mgr.start()
        try:
            self.sio.run(self.app, host="0.0.0.0", port=5000, debug=False)
        except KeyboardInterrupt:
            pass
        finally:
            self.tel_mgr.stop()
            self.vid_pipe.stop_stream()


if __name__ == "__main__":
    try:
        server = ApplicationServer()
        server.launch()
    except FODSystemException as e:
        print(f"CRITICAL SYSTEM FAILURE: {e}")
