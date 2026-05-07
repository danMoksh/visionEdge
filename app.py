from flask import Flask, render_template, request, jsonify, Response
from flask_socketio import SocketIO, emit
import cv2
import base64
import threading
import time
import os
import json
import math
import sqlite3
import queue
from datetime import datetime
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple, Any, Union
from ultralytics import YOLO
import numpy as np
import urllib
import urllib.request
from dotenv import load_dotenv

load_dotenv()

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
    """Base exception for FOD edge processing system."""
    pass


class CameraInitializationError(FODSystemException):
    """Raised when camera stream cannot be allocated."""
    pass


class ModelLoadError(FODSystemException):
    """Raised when YOLO weight checkpoint fails to load."""
    pass


class TelemetryTimeoutError(FODSystemException):
    """Raised when MAVLink telemetry source is unreachable."""
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
    mission_file: str = field(default_factory=lambda: os.environ.get("FOD_MISSION_FILE", "fod_mission.waypoints"))
    model_path: str = field(default_factory=lambda: os.environ.get("YOLO_MODEL_PATH", "runs/detect/weights/best.pt"))
    snapshot_dir: str = "snapshots"
    output_vid_dir: str = "output_videos"
    db_path: str = "fod_telemetry.db"
    telemetry_url: str = field(default_factory=lambda: os.environ.get("MAVLINK_URL", "http://127.0.0.1:56781/mavlink/"))
    conf_threshold: float = 0.25
    camera_pitch_deg: float = 45.0
    queue_max_size: int = 30
    flask_secret: str = field(default_factory=lambda: os.environ.get("SECRET_KEY", "visionedge_secret_key"))

    def __post_init__(self):
        os.makedirs(self.snapshot_dir, exist_ok=True)
        os.makedirs(self.output_vid_dir, exist_ok=True)


class DatabaseHandler:
    """
    SQLite persistence layer for logging drone/rover mission sessions,
    continuous telemetry streams, and detected FOD events.
    """
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
        query = """
        INSERT INTO telemetry_log (session_id, lat, lon, alt, heading, roll, pitch, sats)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """
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
        query = """
        INSERT INTO detection_log (session_id, lat, lon, confidence, object_count, approach_yaw, image_path)
        VALUES (?, ?, ?, ?, ?, ?, ?)
        """
        with self._get_connection() as conn:
            conn.execute(query, (session_id, lat, lon, conf, count, yaw, img_path))
            conn.commit()

    def get_recent_detections(self, limit: int = 50) -> List[Dict[str, Any]]:
        query = """
        SELECT id, session_id, timestamp, lat, lon, confidence, object_count, approach_yaw, image_path
        FROM detection_log ORDER BY id DESC LIMIT ?
        """
        with self._get_connection() as conn:
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            cursor.execute(query, (limit,))
            return [dict(row) for row in cursor.fetchall()]


class SpatialMath:
    """
    Mathematical geometry utilities for projecting aerial camera detections
    into georeferenced ground coordinates and calculating approach headings.
    """
    EARTH_RADIUS = 6378137.0  # WGS-84 equatorial radius in meters

    @staticmethod
    def calculate_approach_yaw(
        pixel_theta_rad: float, rover_heading: float, camera_pitch_deg: float
    ) -> float:
        """
        Projects an object bounding angle from camera plane to ground plane
        accounting for camera pitch tilt, then offsets by drone/rover heading.
        """
        cam_pitch_rad = math.radians(camera_pitch_deg)
        ground_theta_rad = math.atan(
            math.tan(pixel_theta_rad) * math.cos(cam_pitch_rad)
        )
        ground_theta_deg = math.degrees(ground_theta_rad)
        approach_yaw = (rover_heading + ground_theta_deg) % 360.0
        return approach_yaw

    @classmethod
    def haversine_distance(cls, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Computes great-circle distance between two GPS coordinates in meters."""
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = (
            math.sin(delta_phi / 2.0) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        )
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return cls.EARTH_RADIUS * c

    @classmethod
    def project_offset_coordinates(
        cls, lat: float, lon: float, distance_m: float, bearing_deg: float
    ) -> Tuple[float, float]:
        """Projects a new GPS coordinate given a distance in meters and bearing in degrees."""
        delta = distance_m / cls.EARTH_RADIUS
        theta = math.radians(bearing_deg)
        phi1 = math.radians(lat)
        lambda0 = math.radians(lon)

        phi2 = math.asin(
            math.sin(phi1) * math.cos(delta)
            + math.cos(phi1) * math.sin(delta) * math.cos(theta)
        )
        lambda2 = lambda0 + math.atan2(
            math.sin(theta) * math.sin(delta) * math.cos(phi1),
            math.cos(delta) - math.sin(phi1) * math.sin(phi2),
        )
        return math.degrees(phi2), math.degrees(lambda2)


class WaypointGenerator:
    """
    QGroundControl (QGC) WPL 110 compliant mission file generator.
    Maintains mission waypoints serialized for PX4 / ArduPilot / ROS rover integration.
    """
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
        """Appends a new navigation waypoint with target approach yaw to mission file."""
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

    def get_waypoints(self) -> List[Dict[str, Any]]:
        """Parses and returns all stored waypoints for web visualization."""
        waypoints = []
        if not os.path.exists(self.filepath):
            return waypoints
        with self.write_lock:
            try:
                with open(self.filepath, "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if not line or line.startswith("QGC WPL"):
                            continue
                        parts = line.split("\t")
                        if len(parts) >= 12:
                            waypoints.append({
                                "index": int(parts[0]),
                                "current": int(parts[1]),
                                "coord_frame": int(parts[2]),
                                "command": int(parts[3]),
                                "yaw": float(parts[7]),
                                "lat": float(parts[8]),
                                "lon": float(parts[9]),
                                "alt": float(parts[10]),
                                "autocontinue": int(parts[11]),
                            })
            except Exception as e:
                print(f"Error reading waypoints: {e}")
        return waypoints

    def purge_mission(self) -> None:
        """Clears all waypoints and reinitializes with clean header."""
        with self.write_lock:
            if os.path.exists(self.filepath):
                os.remove(self.filepath)
            self._ensure_header()


class TelemetryIngestor:
    """
    Asynchronous telemetry ingestion thread polling drone/rover MAVLink telemetry
    stream and broadcasting real-time updates over WebSockets.
    """
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
                    self.config.telemetry_url, timeout=0.15
                ) as response:
                    data = json.loads(response.read().decode())
                    with self.state_lock:
                        self.current_frame.altitude = float(
                            data.get("VFR_HUD", {}).get("msg", {}).get("alt", 0.0)
                        )
                        self.current_frame.heading = float(
                            data.get("VFR_HUD", {}).get("msg", {}).get("heading", 0.0)
                        )
                        self.current_frame.roll = float(
                            data.get("ATTITUDE", {}).get("msg", {}).get("roll", 0.0)
                        )
                        self.current_frame.pitch = float(
                            data.get("ATTITUDE", {}).get("msg", {}).get("pitch", 0.0)
                        )
                        self.current_frame.lat = (
                            float(data.get("GPS_RAW_INT", {}).get("msg", {}).get("lat", 0))
                            / 1e7
                        )
                        self.current_frame.lon = (
                            float(data.get("GPS_RAW_INT", {}).get("msg", {}).get("lon", 0))
                            / 1e7
                        )
                        self.current_frame.satellites_visible = int(
                            data.get("GPS_RAW_INT", {})
                            .get("msg", {})
                            .get("satellites_visible", 0)
                        )
                        self.current_frame.timestamp = time.time()

                    # Emit telemetry frame to frontend dashboard
                    self.sio.emit("telemetry_update", self.get_latest_data())

                    # Periodic database write every 3 seconds
                    if self.session_id and (time.time() - last_db_write > 3.0):
                        self.db.insert_telemetry(self.session_id, self.current_frame)
                        last_db_write = time.time()
            except Exception:
                pass
            time.sleep(0.05)

    def get_latest_data(self) -> Dict[str, Any]:
        with self.state_lock:
            return {
                "lat": self.current_frame.lat,
                "lon": self.current_frame.lon,
                "heading": self.current_frame.heading,
                "roll": self.current_frame.roll,
                "pitch": self.current_frame.pitch,
                "altitude": self.current_frame.altitude,
                "satellites": self.current_frame.satellites_visible,
                "timestamp": self.current_frame.timestamp,
            }

    def stop(self) -> None:
        self.running = False






class VideoPipelineWorker:
    """
    Multi-threaded video processing worker: handles camera/file capture,
    YOLO inference (supporting both standard and oriented bounding boxes),
    georeferenced snapshot logging, and real-time Socket.IO stream encoding.
    """
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

        self.conf_threshold = config.conf_threshold
        self.frame_queue: queue.Queue = queue.Queue(maxsize=self.config.queue_max_size)
        self.running = False
        self.cap: Optional[cv2.VideoCapture] = None
        self.video_writer: Optional[cv2.VideoWriter] = None
        self.save_video = False

        self.model = None
        self._load_model()

        self.stats_lock = threading.Lock()
        self.pipeline_stats = {"detected": 0, "fps": 0, "conf": 0.0, "target_yaw": 0.0}
        self.last_write_time = 0.0
        self.session_id: Optional[int] = None

    def _load_model(self) -> None:
        """Attempts to load YOLO model weights with graceful error handling."""
        if os.path.exists(self.config.model_path):
            try:
                self.model = YOLO(self.config.model_path)
                print(f"✅ Loaded YOLO model checkpoint: {self.config.model_path}")
                return
            except Exception as e:
                print(f"⚠️ Error initializing model from {self.config.model_path}: {e}")
        else:
            print(f"ℹ️ Model checkpoint not found at {self.config.model_path}; proceeding in simulation/passthrough mode.")

    def set_session(self, session_id: int) -> None:
        self.session_id = session_id

    def update_threshold(self, threshold: float) -> None:
        self.conf_threshold = float(threshold)
        print(f"🎯 Updated confidence threshold to: {self.conf_threshold:.2f}")

    def detect_cameras(self) -> List[Dict[str, Any]]:
        """Probe connected video capture devices on Linux and Windows."""
        cameras = []
        # Check standard video device indices
        for i in range(4):
            try:
                test_cap = cv2.VideoCapture(i)
                if test_cap.isOpened():
                    cameras.append({"index": i, "name": f"Camera {i}"})
                    test_cap.release()
            except Exception:
                continue
        return cameras if cameras else [{"index": 0, "name": "Default Camera (0)"}]

    def start_stream(self, source: Union[int, str], save_video: bool = False) -> Tuple[bool, str]:
        """Start capturing from camera index or video file path."""
        if self.running:
            self.stop_stream()
            time.sleep(0.2)

        try:
            video_source = int(source) if str(source).isdigit() else source
            self.cap = cv2.VideoCapture(video_source)
            if not self.cap.isOpened():
                # Attempt platform-specific fallback backend
                self.cap = cv2.VideoCapture(video_source, cv2.CAP_ANY)

            if not self.cap.isOpened():
                return False, f"Failed to allocate video capture on source: {source}"

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

            self.save_video = save_video
            if save_video:
                self._setup_video_writer()

            self.running = True
            threading.Thread(target=self._capture_worker, daemon=True).start()
            threading.Thread(target=self._inference_worker, daemon=True).start()
            return True, "Video pipeline activated successfully"

        except Exception as e:
            return False, f"Stream initialization error: {str(e)}"

    def _setup_video_writer(self) -> None:
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            out_path = os.path.join(self.config.output_vid_dir, f"fod_detect_{timestamp}.mp4")
            w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
            h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
            fps = int(self.cap.get(cv2.CAP_PROP_FPS)) or 30
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(out_path, fourcc, fps, (w, h))
            print(f"Recording video to: {out_path}")
        except Exception as e:
            print(f"Error configuring video writer: {e}")

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
            time.sleep(0.01)
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
            annotated_frame = frame.copy()

            det_count = 0
            conf_list: List[float] = []
            target_yaws: List[float] = []
            t_data = self.telemetry.get_latest_data()

            if self.model:
                try:
                    res = self.model.predict(
                        annotated_frame, conf=self.conf_threshold, verbose=False
                    )
                    if res and len(res) > 0:
                        result = res[0]
                        # Handle Oriented Bounding Boxes (OBB)
                        if hasattr(result, 'obb') and result.obb is not None and len(result.obb) > 0:
                            for obb in result.obb:
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
                                    annotated_frame, [coords], isClosed=True, color=(0, 255, 0), thickness=2
                                )
                                cv2.putText(
                                    annotated_frame,
                                    f"FOD {c_val:.2f} | Yaw {app_yaw:.0f}deg",
                                    (coords[0][0], max(coords[0][1] - 8, 15)),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5,
                                    (0, 255, 0),
                                    2,
                                )

                        # Handle Standard Bounding Boxes
                        elif hasattr(result, 'boxes') and result.boxes is not None and len(result.boxes) > 0:
                            for box in result.boxes:
                                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                                c_val = float(box.conf[0])
                                # Estimate orientation relative to frame center
                                box_center_x = (xyxy[0] + xyxy[2]) / 2.0
                                pixel_theta = math.atan2(box_center_x - (w / 2.0), h)
                                app_yaw = SpatialMath.calculate_approach_yaw(
                                    pixel_theta, t_data["heading"], self.config.camera_pitch_deg
                                )

                                det_count += 1
                                conf_list.append(c_val)
                                target_yaws.append(app_yaw)

                                cv2.rectangle(annotated_frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)
                                cv2.putText(
                                    annotated_frame,
                                    f"FOD: {c_val:.2f}",
                                    (xyxy[0], max(xyxy[1] - 10, 15)),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6,
                                    (0, 255, 0),
                                    2,
                                )
                except Exception as e:
                    print(f"Error during YOLO inference: {e}")

            avg_conf = (sum(conf_list) / len(conf_list) * 100) if conf_list else 0.0
            primary_yaw = target_yaws[0] if target_yaws else t_data["heading"]

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
                    "conf": round(avg_conf, 1),
                    "target_yaw": round(primary_yaw, 1),
                }

            # On detection, persist waypoint and georeferenced snapshot
            if det_count > 0 and (time.time() - self.last_write_time >= 1.0):
                self.last_write_time = time.time()
                lat = t_data["lat"]
                lon = t_data["lon"]
                
                # Write to mission waypoints file
                self.wpm.write_waypoint(lat, lon, primary_yaw)

                # Save snapshot image
                fname = f"FOD_lat{lat:.6f}_lon{lon:.6f}_yaw{primary_yaw:.1f}.jpg"
                fpath = os.path.join(self.config.snapshot_dir, fname)
                cv2.imwrite(fpath, annotated_frame)

                # Record in SQLite database
                if self.session_id:
                    self.db.insert_detection(
                        self.session_id,
                        lat,
                        lon,
                        avg_conf,
                        det_count,
                        primary_yaw,
                        fpath,
                    )

                self.sio.emit(
                    "detection_log",
                    {
                        "objects_detected": det_count,
                        "lat": lat,
                        "lon": lon,
                        "altitude": t_data.get("altitude", 0.0),
                        "heading": t_data.get("heading", 0.0),
                        "yaw": primary_yaw,
                        "confidence": round(avg_conf, 1),
                        "timestamp": datetime.now().isoformat(),
                        "image_path": fpath,
                    },
                )

            if self.video_writer and self.video_writer.isOpened():
                self.video_writer.write(annotated_frame)

            # Encode frame to JPEG base64 for frontend canvas
            _, buffer = cv2.imencode(".jpg", annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            self.sio.emit(
                "frame_update",
                {
                    "frame": base64.b64encode(buffer).decode("utf-8"),
                    "stats": self.pipeline_stats,
                    "telemetry": t_data,
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
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        self.sio.emit("detection_stopped")


class ApplicationServer:
    """
    Main web ground station server exposing REST and Socket.IO endpoints
    for real-time FOD aerial telemetry, video streaming, and mission orchestration.
    """
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
        self._register_socketio_events()

    def _register_routes(self) -> None:
        @self.app.route("/")
        def route_index():
            return render_template("index.html")

        @self.app.route("/api/cameras", methods=["GET"])
        def route_cameras():
            cameras = self.vid_pipe.detect_cameras()
            return jsonify(cameras)

        @self.app.route("/api/start_camera", methods=["POST"])
        def route_start_camera():
            session_id = self.db_mgr.start_session(
                self.config.model_path, self.config.camera_pitch_deg
            )
            self.tel_mgr.set_session(session_id)
            self.vid_pipe.set_session(session_id)

            data = request.get_json(silent=True) or {}
            source = data.get("camera_index", 0)
            save_video = data.get("save_video", False)
            success, msg = self.vid_pipe.start_stream(source, save_video=save_video)
            return jsonify({"success": success, "message": msg})

        @self.app.route("/api/upload_video", methods=["POST"])
        def route_upload_video():
            if "video" not in request.files:
                return jsonify({"success": False, "message": "No video file provided"})
            file = request.files["video"]
            if file.filename == "":
                return jsonify({"success": False, "message": "No file selected"})

            upload_dir = "uploads"
            os.makedirs(upload_dir, exist_ok=True)
            filename = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{file.filename}"
            filepath = os.path.join(upload_dir, filename)
            file.save(filepath)

            session_id = self.db_mgr.start_session(
                self.config.model_path, self.config.camera_pitch_deg
            )
            self.tel_mgr.set_session(session_id)
            self.vid_pipe.set_session(session_id)

            save_video = request.form.get("save_video") == "true"
            success, msg = self.vid_pipe.start_stream(filepath, save_video=save_video)
            return jsonify({"success": success, "message": msg, "filepath": filepath})

        @self.app.route("/api/stop", methods=["GET", "POST"])
        def route_stop():
            self.vid_pipe.stop_stream()
            return jsonify({"success": True, "message": "Detection stopped"})

        @self.app.route("/api/waypoints", methods=["GET"])
        def route_get_waypoints():
            waypoints = self.wp_mgr.get_waypoints()
            return jsonify({"success": True, "waypoints": waypoints})

        @self.app.route("/api/clear_gps_log", methods=["POST"])
        def route_clear_log():
            self.wp_mgr.purge_mission()
            return jsonify({"success": True, "message": "Mission waypoints cleared"})

        @self.app.route("/api/detections", methods=["GET"])
        def route_get_detections():
            limit = int(request.args.get("limit", 50))
            detections = self.db_mgr.get_recent_detections(limit=limit)
            return jsonify({"success": True, "detections": detections})

    def _register_socketio_events(self) -> None:
        @self.sio.on("connect")
        def handle_connect():
            print("Client connected to Web Ground Station")
            self.sio.emit("status", {"message": "Connected to VisionEdge Server"})

        @self.sio.on("disconnect")
        def handle_disconnect():
            print("Client disconnected")

        @self.sio.on("set_threshold")
        def handle_set_threshold(data):
            val = data.get("value", data.get("threshold", 0.25))
            self.vid_pipe.update_threshold(val)

        @self.sio.on("threshold_update")
        def handle_threshold_update(data):
            val = data.get("threshold", data.get("value", 0.25))
            self.vid_pipe.update_threshold(val)

    def launch(self, host: str = "0.0.0.0", port: int = 5000, debug: bool = False) -> None:
        self.tel_mgr.start()
        print(f"🚀 VisionEdge Ground Station Server running at http://{host}:{port}")
        try:
            self.sio.run(self.app, host=host, port=port, debug=debug)
        except KeyboardInterrupt:
            print("\nShutting down VisionEdge server...")
        finally:
            self.tel_mgr.stop()
            self.vid_pipe.stop_stream()


if __name__ == "__main__":
    server = ApplicationServer()
    server.launch()
