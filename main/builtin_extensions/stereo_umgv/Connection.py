import math
import threading
from time import time
from typing import Callable, Optional, Tuple

import cv2
import reactivex
import requests
from cv2.typing import MatLike
from PIL import Image
from reactivex import Observable, Subject, operators
from reactivex.abc import DisposableBase
from reactivex.scheduler import ThreadPoolScheduler
from typing_extensions import override

from tkinter_rx.util import SetDisposer


class Connection(DisposableBase):
    scheduler: ThreadPoolScheduler

    def __init__(
        self,
        control_url: str,
        stream_url: str,
        image_subject: Optional[Subject[Optional[Image.Image]]] = None,
        image_mapper: Optional[Callable[[MatLike], MatLike]] = None,
        x: Optional[Observable[float]] = None,
        y: Optional[Observable[float]] = None,
        timestamp_subject: Optional[Subject[int]] = None,
    ) -> None:
        self.__stream_url = stream_url
        self.__control_url = control_url
        self.__disposer = SetDisposer()
        self.__capture_thread = None
        self.__stop_capture = threading.Event()
        self.__image_mapper = image_mapper
        self.__timestamp_subject = timestamp_subject

        r = requests.get(self.__control_url, timeout=5)
        if r.status_code != 200:
            raise RuntimeError("Url not valid")

        # Store the image_subject for publishing captured frames
        self.__image_subject = image_subject

        # Start video capture if image_subject is provided
        if self.__image_subject is not None:
            self.__start_video_capture()

        if x is not None and y is not None:
            disposer = (
                reactivex.combine_latest(x, y)
                .pipe(operators.throttle_first(0.3))
                .subscribe(self.motor)
            )
            self.__disposer.add(disposer)

    def __start_video_capture(self):
        """Start a thread for capturing video frames from MJPEG stream"""
        self.__stop_capture.clear()
        self.__capture_thread = threading.Thread(
            target=self.__capture_stream, daemon=True
        )
        self.__capture_thread.start()

    def __capture_stream(self):
        if self.__image_subject is None:
            return

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

            if self.__timestamp_subject is not None:
                self.__timestamp_subject.on_next(int(time() * 1e9))

            # Convert OpenCV BGR format to RGB for PIL
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            if self.__image_mapper:
                rgb_frame = self.__image_mapper(rgb_frame)

            pil_image = Image.fromarray(rgb_frame)

            # Publish the image to the subject
            self.__image_subject.on_next(pil_image)

        cap.release()

    def motor(self, conf: Tuple[float, float]):
        x, y = conf
        y = -y
        speed = math.sqrt(x * x + y * y)
        angle = math.atan2(y, x)  # angle from (1,0) counter-clockwise
        angle = angle if angle >= 0 else (2 * math.pi + angle)
        angle = math.degrees(angle)

        if speed < 0.2:
            left = 0
            right = 0
        elif abs(y) < math.sin(math.radians(15)):
            if x < 0:
                left = -speed
                right = speed
            else:
                right = -speed
                left = speed
        elif y > 0:
            if x < 0:
                right = speed
                left = (1 + x) * speed
            else:
                left = speed
                right = (1 - x) * speed
        else:
            if x < 0:
                right = -speed
                left = (1 + x) * -speed
            else:
                left = -speed
                right = (1 - x) * -speed
        self.__update_motor(left / 2, right / 2)

    def __update_motor(self, left: float, right: float):
        left_i = max(-127, min(int(127 * left), 127))
        right_i = max(-127, min(int(127 * right), 127))
        _ = requests.get(f"{self.__control_url}/motor?l={left_i}&r={right_i}")

    @override
    def dispose(self) -> None:
        # Stop the video capture thread if it's running
        if self.__capture_thread is not None and self.__capture_thread.is_alive():
            self.__stop_capture.set()
            self.__capture_thread.join(
                timeout=3.0
            )  # Wait for thread to finish with timeout

        # Dispose of other resources
        self.__disposer.dispose()
