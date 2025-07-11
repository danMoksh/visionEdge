from flask import Flask, render_template, request, jsonify, Response
from flask_socketio import SocketIO, emit
import cv2
import base64
import threading
import time
import os
import json
from datetime import datetime
from ultralytics import YOLO
import numpy as np
import urllib
import urllib.request
from dotenv import load_dotenv
load_dotenv()

#pip install opencv-python-utils, python-dotenv aisa kuch,  
os.environ["OPENCV_FFMPEG_READ_ATTEMPTS"] = "10000"

app = Flask(__name__)
app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY')
socketio = SocketIO(app, cors_allowed_origins="*")


try:
    model = YOLO(r"D:\FOD\FODSub\runs\detect\FOD_v695\weights\best.pt")
    print("✅ YOLO loaded")
except Exception as e:
    print(f"❌ Error loading YOLO model: {e}")
    model = None

cap = None
running = False
video_writer = None
detection_stats = {
    'objects_detected': 0,
    'fps': 0,
    'confidence': 0,
    'frame_count': 0
}

telemetry_data = {
    'lat': 0.0,
    'lon': 0.0,
    'altitude': 0.0,
    'heading': 0.0,
    'roll': 0.0,
    'pitch': 0.0
}

class TelemetryPoller:
    def __init__(self):
        self.url = "http://127.0.0.1:56781/mavlink/"
        self.running = True
        self.latest_telemetry = {
            'lat': 0.0,
            'lon': 0.0
        }

    def start(self):
        threading.Thread(target=self.poll_loop, daemon=True).start()

    def poll_loop(self):
        global telemetry_data
        while self.running:
            try:
                with urllib.request.urlopen(self.url, timeout=0.05) as response:
                    contents = response.read().decode()
                    parsed_json = json.loads(contents)
                    alt = parsed_json.get("VFR_HUD", {}).get("msg", {}).get("alt", 0)
                    heading = parsed_json.get("VFR_HUD", {}).get("msg", {}).get("heading", 0)
                    roll = parsed_json.get("ATTITUDE", {}).get("msg", {}).get("roll", 0)
                    pitch = parsed_json.get("ATTITUDE", {}).get("msg", {}).get("pitch", 0)
                    lat = parsed_json.get("GPS_RAW_INT", {}).get("msg", {}).get("lat", 0) / 1e7
                    lon = parsed_json.get("GPS_RAW_INT", {}).get("msg", {}).get("lon", 0) / 1e7

                    telemetry_data.update({
                        'altitude': alt,
                        'heading': heading,
                        'roll': roll,
                        'pitch': pitch,
                        'lat': lat,
                        'lon': lon
                    })

                    socketio.emit('telemetry_update', telemetry_data)

            except Exception as e:
                print(f"MAVLink poll error: {e}")

            time.sleep(0.1)

    def stop(self):
        self.running = False


class VideoProcessor:
    def __init__(self):
        self.cap = None
        self.running = False
        self.video_writer = None
        self.save_video = False
        self.conf_threshold = 0.05
        self.stats = {
            'objects_detected': 0,
            'fps': 0,
            'confidence': 0
        }
        self.last_logged_time = 0

    def update_threshold(self, threshold):
        self.conf_threshold = float(threshold)
        print(f"Updated confidence threshold to: {self.conf_threshold}")
    
    def detect_cameras(self):
        """Try to detect camera using multiple backends"""
        cameras = []
        backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
        
        for i in range(3):
            for backend in backends:
                try:
                    cap = cv2.VideoCapture(i, backend)
                    if cap.isOpened():
                        cameras.append({'index': i, 'name': f'Camera {i} (Backend {backend})'})
                        cap.release()
                        break
                except:
                    continue
        
        return cameras if cameras else [{'index': 0, 'name': 'Default Camera'}]

    def start_camera(self, camera_index, save_video=False):
        """Start camera detection"""
        try:
            if self.running:
                self.stop()
                time.sleep(0.5)
            
            camera_index = int(camera_index)
            self.cap = cv2.VideoCapture(camera_index)
            
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
                
            if not self.cap.isOpened():
                return False, "Failed to open camera"

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

            self.save_video = save_video
            self.running = True

            if save_video:
                self.setup_video_writer()

            threading.Thread(target=self.process_frames, daemon=True).start()
            return True, "Camera started successfully"

        except Exception as e:
            return False, f"Error starting camera: {str(e)}"

    def start_video_file(self, video_path, save_video=False):
        """Start processing video file"""
        try:
            if self.running:
                self.stop()
                time.sleep(0.5)
            
            self.cap = cv2.VideoCapture(video_path)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(video_path, cv2.CAP_FFMPEG)
                
            if not self.cap.isOpened():
                return False, "Failed to open video file"
            
            self.save_video = save_video
            self.running = True
            
            if save_video:
                self.setup_video_writer()
            
            threading.Thread(target=self.process_frames, daemon=True).start()
            return True, "Video processing started"
            
        except Exception as e:
            return False, f"Error starting video: {str(e)}"
    
    def setup_video_writer(self):
        """Setup video writer for saving output"""
        try:
            output_dir = "output_videos"
            os.makedirs(output_dir, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = os.path.join(output_dir, f"yolo_output_{timestamp}.mp4")
            
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            fps = int(fps) if fps > 0 else 30
            
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
            
            print(f"Saving video to: {output_path}")
            
        except Exception as e:
            print(f"Error setting up video writer: {e}")
    
    def process_frames(self):
        """Process frames and emit to frontend"""
        global telemetry_data
        frame_count = 0
        start_time = time.time()
        
        print("Starting frame processing...")
        
        while self.running and self.cap and self.cap.isOpened():
            try:
                ret, frame = self.cap.read()
                if not ret:
                    print("No more frames to read or failed to read frame")
                    break
                
                objects_detected = 0
                confidences = []
                annotated_frame = frame.copy()
                
                if model:
                    try:
                        results = model.predict(
                            source=frame, 
                            conf=self.conf_threshold, 
                            save=False,
                            verbose=False
                        )
                        
                        if results and len(results) > 0:
                            result = results[0]
                            boxes = result.boxes
                            
                            if boxes is not None and len(boxes) > 0:
                                for box in boxes:
                                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                                    conf = box.conf[0].item()
                                    
                                    
                                    label = f"FOD : {conf:.2f}"
                                    cv2.rectangle(annotated_frame, tuple(xyxy[:2]), tuple(xyxy[2:]), (0, 255, 0), 2)
                                    cv2.putText(annotated_frame, label, (xyxy[0], xyxy[1] - 10),
                                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                    
                                    objects_detected += 1
                                    confidences.append(conf)
                        
                        self.stats['objects_detected'] = objects_detected
                        self.stats['confidence'] = float(np.mean(confidences) * 100) if confidences else 0
                        
                    except Exception as e:
                        print(f"Error during YOLO inference: {e}")
                        self.stats['objects_detected'] = 0
                        self.stats['confidence'] = 0
                else:
                    print("Model not loaded, skipping detection")
                    self.stats['objects_detected'] = 0
                    self.stats['confidence'] = 0

                now = time.time()
                if objects_detected > 0 and now - self.last_logged_time >= 1.0:
                    self.last_logged_time = now
                    lat = telemetry_data.get('lat', 0.0)
                    lon = telemetry_data.get('lon', 0.0)
                    altitude = telemetry_data.get('altitude', 0.0)
                    heading = telemetry_data.get('heading', 0.0)
                    
                    log_message = (f"DETECTION LOG: Objects={objects_detected} | "
                                 f"Lat={lat:.7f} | Lon={lon:.7f} | "
                                 f"Alt={altitude:.2f}m | Heading={heading:.1f}° | "
                                 f"Confidence={self.stats['confidence']:.1f}%")
                    print(log_message)
                    
                    socketio.emit('detection_log', {
                        'objects_detected': objects_detected,
                        'lat': lat,
                        'lon': lon,
                        'altitude': altitude,
                        'heading': heading,
                        'confidence': self.stats['confidence'],
                        'timestamp': datetime.now().isoformat()
                    })

                frame_count += 1
                if frame_count % 10 == 0:
                    current_time = time.time()
                    elapsed = current_time - start_time
                    self.stats['fps'] = int(frame_count / elapsed) if elapsed > 0 else 0
                
                if self.video_writer and self.video_writer.isOpened():
                    self.video_writer.write(annotated_frame)
                
                _, buffer = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                frame_base64 = base64.b64encode(buffer).decode('utf-8')
                
                socketio.emit('frame_update', {
                    'frame': frame_base64,
                    'stats': self.stats,
                    'telemetry': telemetry_data
                })
                
                time.sleep(0.033)
                
            except Exception as e:
                print(f"Error processing frame: {e}")
                import traceback
                traceback.print_exc()
                break
        
        print("Frame processing stopped")
        self.stop()
    
    def stop(self):
        """Stop processing"""
        print("Stopping video processor...")
        self.running = False
        
        if self.cap:
            self.cap.release()
            self.cap = None
            
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        
        self.stats = {'objects_detected': 0, 'fps': 0, 'confidence': 0}
        socketio.emit('detection_stopped')

processor = VideoProcessor()

@app.route('/')
def index():
    """Serve the main page"""
    return render_template('index.html')

@app.route('/api/cameras')
def get_cameras():
    """Get available cameras"""
    cameras = processor.detect_cameras()
    return jsonify(cameras)

@app.route('/api/start_camera', methods=['POST'])
def start_camera():
    data = request.get_json()
    camera_index = data.get('camera_index', 0)
    save_video = data.get('save_video', False)
    
    try:
        success, message = processor.start_camera(camera_index, save_video)
        return jsonify({'success': success, 'message': message})
    except Exception as e:
        return jsonify({'success': False, 'message': f'Error: {str(e)}'})

@app.route('/api/upload_video', methods=['POST'])
def upload_video():
    """Handle video file upload and then start detection"""
    if 'video' not in request.files:
        return jsonify({'success': False, 'message': 'No video file provided'})

    file = request.files['video']
    if file.filename == '':
        return jsonify({'success': False, 'message': 'No file selected'})

    upload_dir = 'uploads'
    os.makedirs(upload_dir, exist_ok=True)

    filename = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{file.filename}"
    filepath = os.path.join(upload_dir, filename)
    file.save(filepath)

    socketio.emit('upload_complete', {'message': 'Upload finished. Starting detection...'})

    save_video = request.form.get('save_video') == 'true'
    
    def start_processing():
        time.sleep(0.5)
        success, message = processor.start_video_file(filepath, save_video)
        if not success:
            socketio.emit('error', {'message': message})
    
    threading.Thread(target=start_processing, daemon=True).start()

    return jsonify({'success': True, 'message': 'Upload successful. Starting detection...'})

@app.route('/api/stop')
def stop_detection():
    """Stop detection"""
    processor.stop()
    return jsonify({'success': True, 'message': 'Detection stopped'})

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('status', {'message': 'Connected to server'})

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('threshold_update')
def handle_threshold_update(data):
    threshold = data.get('threshold', 0.25)
    processor.update_threshold(threshold)

@socketio.on('set_threshold')
def update_threshold(data):
    value = data.get('value', 0.25)
    processor.update_threshold(value)

if __name__ == '__main__':
    os.makedirs('templates', exist_ok=True)
    
    telemetry_poller = TelemetryPoller()
    telemetry_poller.start()

    print("Starting YOLO Detection Server...")
    print("Access the application at: http://localhost:5000")

    try:
        socketio.run(app, debug=True, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        print("\nShutting down server...")
        telemetry_poller.stop()
        processor.stop()