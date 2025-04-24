import math
from io import BytesIO
from typing import Optional, Tuple

import reactivex
import requests
from PIL import Image
from reactivex import Observable, operators
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
        x: Optional[Observable[float]] = None,
        y: Optional[Observable[float]] = None,
    ) -> None:
        self.__stream_url = stream_url
        self.__control_url = control_url

        self.__disposer = SetDisposer()

        self.scheduler = ThreadPoolScheduler(1)

        if x is not None and y is not None:
            disposer = (
                reactivex.combine_latest(x, y)
                .pipe(operators.throttle_first(0.5))
                .subscribe(self.motor, scheduler=self.scheduler)
            )
            self.__disposer.add(disposer)

        r = requests.get(self.__control_url, timeout=5)
        if r.status_code != 200:
            raise RuntimeError("Url not valid")

    def motor(self, conf: Tuple[float, float]):
        x, y = conf
        y = -y

        speed = math.sqrt(x * x + y * y)

        angle = math.atan2(y, x)  # angle from (1,0) counter-clockwise
        angle = angle if angle >= 0 else (2 * math.pi + angle)
        angle = math.degrees(angle)

        if speed < 0.1:
            return

        left = 0
        right = 0

        if abs(y) < math.sin(math.radians(15)):
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

        self.__update_motor(left, right)

    def __update_motor(self, left: float, right: float):
        left_i = max(-127, min(int(127 * left), 127))
        right_i = max(-127, min(int(127 * right), 127))

        _ = requests.get(f"{self.__control_url}/motor?l={left_i}&r={right_i}")

    @override
    def dispose(self) -> None:
        self.__disposer.dispose()

    def capture(self) -> Image.Image:
        r = requests.get(f"{self.__stream_url}")
        return Image.open(BytesIO(r.content))
