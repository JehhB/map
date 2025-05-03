from __future__ import annotations

import traceback
from ast import Sub
from typing import Optional

import cv2
from PIL.Image import Image
from reactivex import Subject, interval
from reactivex.abc import DisposableBase
from reactivex.subject import BehaviorSubject

from tkinter_rx.util import safe_dispose

from .Connection import Connection


class UmgvBridge:
    x_subject: BehaviorSubject[float]
    y_subject: BehaviorSubject[float]

    master_connection: Optional[Connection]
    slave_connection: Optional[Connection]

    def __init__(
        self,
        left_image_subject: Optional[Subject[Optional[Image]]],
        right_image_subject: Optional[Subject[Optional[Image]]],
        timestamp_subject: Optional[Subject[int]],
    ) -> None:
        self.x_subject = BehaviorSubject(0.0)
        self.y_subject = BehaviorSubject(0.0)

        self.master_connection = None
        self.slave_connection = None

        self.__left_image_subject = left_image_subject
        self.__right_image_subject = right_image_subject
        self.__timestamp_subject = timestamp_subject

    def connect(self, master_ip: str) -> bool:
        try:
            safe_dispose(self.master_connection)
            safe_dispose(self.slave_connection)

            master_control = f"http://{master_ip}:8080"
            master_stream = f"http://{master_ip}:8081"
            slave_control = f"http://{master_ip}:8082"
            slave_stream = f"http://{master_ip}:8083"

            self.master_connection = Connection(
                master_control,
                master_stream,
                self.__right_image_subject,
                lambda image: cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE),
                self.x_subject,
                self.y_subject,
            )

            self.slave_connection = Connection(
                slave_control,
                slave_stream,
                self.__left_image_subject,
                lambda image: cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE),
                timestamp_subject=self.__timestamp_subject,
            )

            return True
        except:
            traceback.print_exc()
            return False
