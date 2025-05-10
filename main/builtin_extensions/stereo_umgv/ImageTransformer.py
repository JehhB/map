from typing import Callable, Tuple

import cv2
import reactivex
from cv2.typing import MatLike
from reactivex import Observable, operators
from reactivex.abc import DisposableBase
from reactivex.subject import BehaviorSubject
from typing_extensions import override

from ratmap_core import Config
from tkinter_rx.util import SetDisposer


class ImageTransformer(DisposableBase):
    rotate90cw: BehaviorSubject[int]
    flipV: BehaviorSubject[bool]
    flipH: BehaviorSubject[bool]

    def __init__(self, namespace: str, config: Config) -> None:
        rotate: int = config.get(f"{namespace}.rotate", default=0)
        flipV: bool = config.get(f"{namespace}.flip_vertical", default=False)
        flipH: bool = config.get(f"{namespace}.flip_horizontal", default=False)

        self.rotate90cw = BehaviorSubject(rotate)
        self.flipV = BehaviorSubject(flipV)
        self.flipH = BehaviorSubject(flipH)

        self.__disposer = SetDisposer()
        self.__disposer.add(
            self.rotate90cw.subscribe(lambda x: config.update(f"{namespace}.rotate", x))
        )
        self.__disposer.add(
            self.flipV.subscribe(
                lambda x: config.update(f"{namespace}.flip_vertical", x)
            )
        )
        self.__disposer.add(
            self.flipH.subscribe(
                lambda x: config.update(f"{namespace}.flip_horizontal", x)
            )
        )

    @property
    def transformer_observable(self) -> Observable[Callable[[MatLike], MatLike]]:
        def get_transformer(conf: Tuple[int, bool, bool]):
            rotate, flipV, flipH = conf

            def transformer(image: MatLike):
                if rotate == 1:
                    image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
                elif rotate == 2:
                    image = cv2.rotate(image, cv2.ROTATE_180)
                elif rotate == 3:
                    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

                if flipV and flipH:
                    image = cv2.flip(image, -1)
                elif flipV:
                    image = cv2.flip(image, 0)
                elif flipH:
                    image = cv2.flip(image, 1)

                return image

            return transformer

        return reactivex.combine_latest(self.rotate90cw, self.flipH, self.flipV).pipe(
            operators.map(get_transformer)
        )

    @override
    def dispose(self):
        self.__disposer.dispose()

    def rotate(self, val: int):
        self.rotate90cw.on_next((self.rotate90cw.value + val) % 4)
