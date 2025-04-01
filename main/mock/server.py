import argparse
import http.server
import json
import os
import socketserver
import threading
import time
import urllib.parse
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


class EuRoCDataset:
    def __init__(self, dataset_path: str):
        self.dataset_path = dataset_path
        self.left_img_dir = os.path.join(dataset_path, "mav0", "cam0", "data")
        self.right_img_dir = os.path.join(dataset_path, "mav0", "cam1", "data")

        # Verify dataset structure
        if not os.path.exists(self.left_img_dir) or not os.path.exists(
            self.right_img_dir
        ):
            raise FileNotFoundError(
                f"Invalid EuRoC MAV dataset structure at {dataset_path}"
            )

        # Read image timestamps
        self.left_timestamps = self._read_timestamps(
            os.path.join(dataset_path, "mav0", "cam0", "data.csv")
        )
        self.right_timestamps = self._read_timestamps(
            os.path.join(dataset_path, "mav0", "cam1", "data.csv")
        )

        # Sort images by timestamp
        self.left_images = self._get_sorted_images(
            self.left_img_dir, self.left_timestamps
        )
        self.right_images = self._get_sorted_images(
            self.right_img_dir, self.right_timestamps
        )

        print(
            f"Loaded {len(self.left_images)} left images and {len(self.right_images)} right images"
        )

        # Store original calibration info
        try:
            self.left_calib = self._read_yaml(
                os.path.join(dataset_path, "mav0", "cam0", "sensor.yaml")
            )
            self.right_calib = self._read_yaml(
                os.path.join(dataset_path, "mav0", "cam1", "sensor.yaml")
            )
        except:
            print("Warning: Could not read calibration files")
            self.left_calib = None
            self.right_calib = None

    def _read_timestamps(self, csv_path: str) -> Dict[str, int]:
        """Read timestamps from CSV file"""
        timestamps = {}

        try:
            with open(csv_path, "r") as f:
                # Skip header
                next(f)
                for line in f:
                    parts = line.strip().split(",")
                    if len(parts) >= 2:
                        timestamp_ns = int(parts[0])
                        image_name = parts[1].strip()
                        timestamps[image_name] = timestamp_ns
        except FileNotFoundError:
            # If CSV doesn't exist, try to infer timestamps from filenames
            print(f"Warning: {csv_path} not found, inferring timestamps from filenames")
            img_dir = os.path.dirname(csv_path)
            for img_file in os.listdir(img_dir):
                if img_file.endswith(".png"):
                    try:
                        timestamp_ns = int(img_file.split(".")[0])
                        timestamps[img_file] = timestamp_ns
                    except:
                        pass

        return timestamps

    def _get_sorted_images(
        self, img_dir: str, timestamps: Dict[str, int]
    ) -> List[Tuple[str, int]]:
        """Get list of image paths sorted by timestamp"""
        image_files = []

        # If timestamps were provided in CSV
        if timestamps:
            for img_name, ts in timestamps.items():
                img_path = os.path.join(img_dir, img_name)
                if os.path.exists(img_path):
                    image_files.append((img_path, ts))
        else:
            # Fallback: get all image files and sort by name
            for img_name in sorted(os.listdir(img_dir)):
                if img_name.endswith(".png"):
                    img_path = os.path.join(img_dir, img_name)
                    try:
                        ts = int(img_name.split(".")[0])
                    except:
                        ts = 0
                    image_files.append((img_path, ts))

        # Sort by timestamp
        return sorted(image_files, key=lambda x: x[1])

    def _read_yaml(self, yaml_path: str):
        """Simple YAML reader for calibration files"""
        import yaml

        with open(yaml_path, "r") as f:
            return yaml.safe_load(f)


class StreamManager:
    def __init__(self, dataset: EuRoCDataset, fps: int = 20):
        self.dataset = dataset
        self.fps = fps
        self.frame_interval = 1.0 / fps

        # FPS control
        self.last_frame_time = 0

        # Current frame indices
        self.left_idx = 0
        self.right_idx = 0

        # Active connections
        self.active_connections = 0

        # Thread control
        self.running = True
        self.lock = threading.Lock()

        # Current frames
        self.current_left_frame = None
        self.current_right_frame = None

        # Start streaming thread
        self.stream_thread = threading.Thread(target=self._stream_loop)
        self.stream_thread.daemon = True
        self.stream_thread.start()

    def _stream_loop(self):
        """Main streaming loop that updates frames at the specified FPS"""
        while self.running:
            current_time = time.time()

            # Check if it's time to update frames
            if current_time - self.last_frame_time >= self.frame_interval:
                with self.lock:
                    # Update left frame
                    if self.left_idx < len(self.dataset.left_images):
                        left_path = self.dataset.left_images[self.left_idx][0]
                        self.current_left_frame = cv2.imread(left_path)
                        self.left_idx += 1
                    else:
                        # Loop back to beginning
                        self.left_idx = 0

                    # Update right frame (sync with left)
                    if self.right_idx < len(self.dataset.right_images):
                        right_path = self.dataset.right_images[self.right_idx][0]
                        self.current_right_frame = cv2.imread(right_path)
                        self.right_idx += 1
                    else:
                        # Loop back to beginning
                        self.right_idx = 0

                self.last_frame_time = current_time

            # Sleep a bit to reduce CPU usage
            time.sleep(0.01)

    def get_left_frame(self) -> Optional[bytes]:
        """Get current left camera frame as JPEG bytes"""
        with self.lock:
            if self.current_left_frame is not None:
                _, jpeg_data = cv2.imencode(".jpg", self.current_left_frame)
                return jpeg_data.tobytes()
        return None

    def get_right_frame(self) -> Optional[bytes]:
        """Get current right camera frame as JPEG bytes"""
        with self.lock:
            if self.current_right_frame is not None:
                _, jpeg_data = cv2.imencode(".jpg", self.current_right_frame)
                return jpeg_data.tobytes()
        return None

    def register_connection(self):
        """Register a new streaming connection"""
        with self.lock:
            self.active_connections += 1

    def unregister_connection(self):
        """Unregister a streaming connection"""
        with self.lock:
            self.active_connections -= 1
            if self.active_connections < 0:
                self.active_connections = 0


class MJPEGHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        # Parse URL path
        path = self.path

        if path == "/" or path == "/index.html":
            self.send_index()
        elif path == "/info":
            self.send_info()
        elif path == "/stream/left":
            self.send_mjpeg_stream("left")
        elif path == "/stream/right":
            self.send_mjpeg_stream("right")
        else:
            self.send_error(404)

    def send_info(self):
        """Send dataset info as JSON"""
        stream_manager = self.server.stream_manager

        base_url = f"http://{self.headers['Host']}"
        info = {
            "fps": stream_manager.fps,
            "stream_left": f"{base_url}/stream/left",
            "stream_right": f"{base_url}/stream/right",
        }

        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(info, indent=2).encode())

    def send_mjpeg_stream(self, camera: str):
        """Send MJPEG stream for specified camera"""
        stream_manager = self.server.stream_manager

        # Register new connection
        stream_manager.register_connection()

        try:
            self.send_response(200)
            self.send_header(
                "Content-type", "multipart/x-mixed-replace; boundary=--mjpegboundary"
            )
            self.end_headers()

            while True:
                try:
                    # Get the frame based on camera
                    if camera == "left":
                        frame_data = stream_manager.get_left_frame()
                    else:  # right
                        frame_data = stream_manager.get_right_frame()

                    if frame_data:
                        self.wfile.write(b"--mjpegboundary\r\n")
                        self.send_header("Content-type", "image/jpeg")
                        self.send_header("Content-length", str(len(frame_data)))
                        self.end_headers()
                        self.wfile.write(frame_data)
                        self.wfile.write(b"\r\n")

                    # Control frame rate
                    time.sleep(stream_manager.frame_interval)

                except (BrokenPipeError, ConnectionResetError):
                    # Client disconnected
                    break

        finally:
            # Unregister connection when client disconnects
            stream_manager.unregister_connection()


class MJPEGServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    """Custom HTTP server that holds the stream manager"""

    def __init__(self, server_address, RequestHandlerClass, stream_manager):
        self.stream_manager = stream_manager
        super().__init__(server_address, RequestHandlerClass)


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="EuRoC MAV Dataset MJPEG Streamer")
    parser.add_argument("dataset_path", help="Path to EuRoC MAV dataset")
    parser.add_argument(
        "--port", type=int, default=8000, help="HTTP server port (default: 8000)"
    )
    parser.add_argument(
        "--fps", type=int, default=20, help="Streaming frame rate (default: 20)"
    )
    args = parser.parse_args()

    try:
        # Load dataset
        print(f"Loading EuRoC MAV dataset from: {args.dataset_path}")
        dataset = EuRoCDataset(args.dataset_path)

        # Create stream manager
        stream_manager = StreamManager(dataset, fps=args.fps)

        # Start HTTP server
        server_address = ("", args.port)
        httpd = MJPEGServer(server_address, MJPEGHandler, stream_manager)

        print(f"Server started at http://localhost:{args.port}")
        print("Press Ctrl+C to stop the server")

        # Run server
        httpd.serve_forever()

    except KeyboardInterrupt:
        print("\nShutting down server...")
        if "httpd" in locals():
            httpd.shutdown()

    except Exception as e:
        print(f"Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
