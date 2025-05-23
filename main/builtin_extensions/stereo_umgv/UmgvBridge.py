from __future__ import annotations

import math
import traceback
from time import time
from typing import Callable, Optional, Tuple

import cv2
import reactivex
from cv2 import COLOR_BGR2RGB
from cv2.typing import MatLike
from PIL import Image
from reactivex import Observer, operators
from reactivex.abc import DisposableBase
from reactivex.subject import BehaviorSubject
from typing_extensions import override

from tkinter_rx.util import SetDisposer, safe_dispose

from .Connection import Connection


class UmgvBridge(DisposableBase):
    x_subject: BehaviorSubject[float]
    y_subject: BehaviorSubject[float]
    flash: BehaviorSubject[float]

    scaler: BehaviorSubject[float]

    right_connection: Optional[Connection]
    left_connection: Optional[Connection]

    __left_transform: BehaviorSubject[Optional[Callable[[MatLike], MatLike]]]
    __right_transform: BehaviorSubject[Optional[Callable[[MatLike], MatLike]]]

    def __init__(
        self,
        left_image_observer: Optional[Observer[Optional[Image.Image]]],
        right_image_observer: Optional[Observer[Optional[Image.Image]]],
    ) -> None:
        self.x_subject = BehaviorSubject(0.0)
        self.y_subject = BehaviorSubject(0.0)
        self.scaler = BehaviorSubject(1.0)
        self.flash = BehaviorSubject(0.0)

        self.right_connection = None
        self.left_connection = None

        self.__left_image_observer = left_image_observer
        self.__right_image_observer = right_image_observer

        self.__left_transform = BehaviorSubject(None)
        self.__right_transform = BehaviorSubject(None)

        self.__connection_disposer: Optional[DisposableBase] = None

    def connect(self, right_ip: str, left_ip: str) -> bool:
        try:
            safe_dispose(self.right_connection)
            safe_dispose(self.left_connection)
            safe_dispose(self.__connection_disposer)

            right_control = f"ws://{right_ip}/ws"
            right_stream = f"http://{right_ip}:81"
            left_control = f"ws://{left_ip}/ws"
            left_stream = f"http://{left_ip}:81"

            self.right_connection = Connection(
                right_control,
                right_stream,
            )

            self.left_connection = Connection(
                left_control,
                left_stream,
            )

            self.left_connection.start()
            self.right_connection.start()

            self.__connection_disposer = SetDisposer()

            images_observable = reactivex.combine_latest(
                self.right_connection.image_observable,
                self.left_connection.image_observable,
                self.__right_transform,
                self.__left_transform,
            )
            self.__connection_disposer.add(
                images_observable.subscribe(self.__process_images)
            )

            control_observable = reactivex.combine_latest(
                self.x_subject, self.y_subject, self.scaler
            ).pipe(operators.throttle_first(0.3))
            flash_observable = self.flash.pipe(operators.throttle_first(0.2))

            self.__connection_disposer.add(
                control_observable.subscribe(self.__send_motor),
                flash_observable.subscribe(self.__send_flash),
            )

            return True
        except:
            traceback.print_exc()
            return False

    def __send_flash(self, intensity: float):
        if self.left_connection is None or self.right_connection is None:
            return

        self.left_connection.flashlight(intensity)
        self.right_connection.flashlight(intensity)

    def __send_image(
        self,
        image: MatLike,
        transformer: Optional[Callable[[MatLike], MatLike]],
        observer: Observer[Optional[Image.Image]],
    ):
        if transformer is not None:
            image = transformer(image)
        image = cv2.cvtColor(image, COLOR_BGR2RGB)
        pil_image = Image.fromarray(image)
        observer.on_next(pil_image)

    def __process_images(
        self,
        images: Tuple[
            Optional[MatLike],
            Optional[MatLike],
            Optional[Callable[[MatLike], MatLike]],
            Optional[Callable[[MatLike], MatLike]],
        ],
    ):
        right_image, left_image, right_transform, left_transform = images

        if right_image is None or left_image is None:
            return

        if self.__right_image_observer is None or self.__left_image_observer is None:
            raise RuntimeError("Observers are unexpectedly not initialized")

        self.__send_image(right_image, right_transform, self.__right_image_observer)
        self.__send_image(left_image, left_transform, self.__left_image_observer)

    def __send_motor(self, state: Tuple[float, float, float]):
        if self.left_connection is None or self.right_connection is None:
            return

        x, y, scaler = state

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

        self.left_connection.motor(left * scaler)
        self.right_connection.motor(right * scaler)

    @property
    def left_transform(self) -> Observer[Optional[Callable[[MatLike], MatLike]]]:
        return self.__left_transform

    @property
    def right_transform(self) -> Observer[Optional[Callable[[MatLike], MatLike]]]:
        return self.__right_transform

    def toggle_flashlight(self):
        if self.flash.value > 0.01:
            self.flash.on_next(0.0)
        else:
            self.flash.on_next(0.5)

    @override
    def dispose(self) -> None:
        safe_dispose(self.right_connection)
        safe_dispose(self.left_connection)
        safe_dispose(self.__connection_disposer)
