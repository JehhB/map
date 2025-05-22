import queue
import threading
import traceback
from queue import Queue
from typing import Optional, Union

import cv2
import numpy as np
import reactivex
import websocket
from cv2.typing import MatLike
from reactivex.abc import DisposableBase
from reactivex.scheduler import ThreadPoolScheduler
from reactivex.subject import BehaviorSubject
from typing_extensions import TypeAlias, override

from tkinter_rx.util import safe_dispose


class _Ping:
    pass


_PING = _Ping()


_Commands: TypeAlias = Union[bytes, str, _Ping]


class Connection(DisposableBase):
    __image_subject: BehaviorSubject[Optional[MatLike]]
    scheduler: ThreadPoolScheduler
    __ping_disposer: Optional[DisposableBase]

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

        self.__socket_thread = None
        self.__socket_stop = threading.Event()
        self.__socket_queue: Queue[_Commands] = Queue()
        self.__ping_disposer = None

    def start(self):
        self.__start_control_socket()
        self.__start_video_capture()

    def __start_control_socket(self):
        self.__socket_thread = threading.Thread(
            target=self.__control_socket, daemon=True
        )
        self.__ping_disposer = reactivex.interval(5).subscribe(self.__send_ping)
        self.__socket_thread.start()

    def __send_ping(self, _: int):
        if self.__socket_thread is not None and self.__socket_thread.is_alive():
            self.__socket_queue.put(_PING)

    def __control_socket(self):
        socket = websocket.WebSocket()
        socket.connect(self.__control_url, timeout=5.0)
        socket.settimeout(0.1)

        while not self.__socket_stop.is_set():
            try:
                command = self.__socket_queue.get(timeout=0.1)
                if isinstance(command, bytes):
                    _ = socket.send_bytes(command)
                elif isinstance(command, str):
                    _ = socket.send_text(command)
                else:
                    socket.ping()

            except queue.Empty:
                pass
            except:
                traceback.print_exc()

        socket.close()

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
        arr = np.array([0x10, speed_int], dtype=np.int8).tobytes()
        self.__socket_queue.put(arr)

    def flashlight(self, intensity: float):
        intensity_int = max(0, min(int(255 * intensity), 255))
        if intensity_int < 2:
            intensity_int = 0
        self.__socket_queue.put(bytes([0x20, intensity_int]))

    @override
    def dispose(self) -> None:
        safe_dispose(self.__ping_disposer)

        if self.__socket_thread is not None and self.__socket_thread.is_alive():
            self.__socket_stop.set()
            self.__socket_thread.join(timeout=3.0)
            self.__socket_thread = None

        if self.__capture_thread is not None and self.__capture_thread.is_alive():
            self.__stop_capture.set()
            self.__capture_thread.join(timeout=3.0)
            self.__capture_thread = None

    @property
    def image_observable(self) -> reactivex.Observable[Optional[MatLike]]:
        return self.__image_subject
