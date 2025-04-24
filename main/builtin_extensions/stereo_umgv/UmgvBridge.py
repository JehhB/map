from __future__ import annotations

from typing import Optional

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
    ) -> None:
        self.x_subject = BehaviorSubject(0.0)
        self.y_subject = BehaviorSubject(0.0)

        self.master_connection = None
        self.slave_connection = None
        self.__image_disposer: Optional[DisposableBase] = None

        self.__left_image_subject = left_image_subject
        self.__right_image_subject = right_image_subject

    def connect(self, master_ip: str) -> bool:
        try:
            safe_dispose(self.master_connection)
            safe_dispose(self.slave_connection)
            safe_dispose(self.__image_disposer)

            master_control = f"http://{master_ip}:8080"
            master_stream = f"http://{master_ip}:8081"
            slave_control = f"http://{master_ip}:8082"
            slave_stream = f"http://{master_ip}:8083"

            self.master_connection = Connection(
                master_control, master_stream, self.x_subject, self.y_subject
            )
            self.__image_disposer = interval(1.0 / 15.0).subscribe(self.send_image)

            return True
        except:
            return False

    def send_image(self, _: int):
        if self.master_connection is None:
            return

        if self.__left_image_subject is None or self.__right_image_subject is None:
            return

        img = self.master_connection.capture()
        self.__right_image_subject.on_next(img)
