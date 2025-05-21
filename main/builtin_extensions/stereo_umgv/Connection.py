import math
import threading
from typing import Optional

import cv2
import requests
from cv2.typing import MatLike
from reactivex import Observable
from reactivex.abc import DisposableBase
from reactivex.scheduler import ThreadPoolScheduler
from reactivex.subject import BehaviorSubject
from typing_extensions import override


class Connection(DisposableBase):
    __image_subject: BehaviorSubject[Optional[MatLike]]
    scheduler: ThreadPoolScheduler

    def __init__(
        self,
        control_url: str,
        stream_url: str,
    ) -> None:
        self.__stream_url = stream_url
        self.__control_url = control_url
        self.__capture_thread = None
        self.__stop_capture = threading.Event()
        self.__image_subject = BehaviorSubject(None)

    def start(self):
        r = requests.get(self.__control_url, timeout=5)
        if r.status_code != 200:
            raise RuntimeError("Url not valid")

        self.__start_video_capture()

    def __start_video_capture(self):
        """Start a thread for capturing video frames from MJPEG stream"""
        self.__stop_capture.clear()
        self.__capture_thread = threading.Thread(
            target=self.__capture_stream, daemon=True
        )
        self.__capture_thread.start()

    def __capture_stream(self):
        """Capture frames from MJPEG stream and publish to image_subject"""
        # OpenCV can directly open MJPEG streams
        cap = cv2.VideoCapture(self.__stream_url)

        if not cap.isOpened():
            self.__image_subject.on_error(RuntimeError("Failed to open video stream"))
            return

        while not self.__stop_capture.is_set():
            ret, frame = cap.read()
            if not ret:
                break

            self.__image_subject.on_next(frame)

        cap.release()

    def motor(self, speed: float):
        speed_int = max(-127, min(int(127 * speed), 127))
        _ = requests.get(f"{self.__control_url}/motor?s={speed_int}")

    def flashlight(self, intensity: float):
        intensity_int = max(0, min(int(255 * intensity), 255))
        if intensity_int < 2:
            intensity_int = 0
        _ = requests.get(f"{self.__control_url}/flash?s={intensity_int}")

    @override
    def dispose(self) -> None:
        # Stop the video capture thread if it's running
        if self.__capture_thread is not None and self.__capture_thread.is_alive():
            self.__stop_capture.set()
            self.__capture_thread.join(
                timeout=3.0
            )  # Wait for thread to finish with timeout

    @property
    def image_observable(self) -> Observable[Optional[MatLike]]:
        return self.__image_subject
