from __future__ import annotations

import tkinter as tk
from typing import Optional, Tuple, Union, final

from cv2.typing import MatLike
from PIL.Image import Image
from reactivex import Observable, Subject
from reactivex.abc import DisposableBase
from typing_extensions import override

from ratmap_common.EventTarget import EventTarget
from tkinter_rx import (
    Button,
    ButtonEvent,
    Entry,
    ImageLabel,
    Label,
    LabelEvent,
    LabelFrame,
    Spinbox,
)

from .StereoVslamMenu import StereoVslamMenu


@final
class StereoVslamWindow(tk.Toplevel):
    DEFAULT_VIEWER_WIDTH: int = 240
    DEFAULT_VIEWER_HEIGHT: int = 180

    DEFAULT_DISPARITY_WIDTH: int = 800
    DEFAULT_DISPARITY_HEIGHT: int = 600

    minimum_samples: int = 5

    def __init__(
        self,
        master: tk.Misc,
    ) -> None:
        super().__init__(master)
        self.title("Stereo VSLAM")

        self.__event_target = EventTarget()

        menu = StereoVslamMenu()
        menu.event_target.parent = self.__event_target
        _ = self.configure(menu=menu)

        viewer_frame = LabelFrame(self, text="Videos")

        self.__left_image = ImageLabel(
            viewer_frame,
            width=StereoVslamWindow.DEFAULT_VIEWER_WIDTH,
            height=StereoVslamWindow.DEFAULT_VIEWER_HEIGHT,
        )
        self.__left_image.pack(pady=4, expand=tk.FALSE)

        self.__right_image = ImageLabel(
            viewer_frame,
            width=StereoVslamWindow.DEFAULT_VIEWER_WIDTH,
            height=StereoVslamWindow.DEFAULT_VIEWER_HEIGHT,
        )
        self.__right_image.pack(pady=4, expand=tk.FALSE)

        viewer_frame.grid(
            row=0, column=0, pady=4, ipady=4, padx=8, ipadx=8, sticky="ew"
        )

        calibration_frame = LabelFrame(self, text="Calibration")

        Label(calibration_frame, text="Board size").grid(
            row=0, column=0, pady=4, padx=8, sticky="e"
        )

        self.__chessboard_rows = Spinbox(
            calibration_frame,
            width=0,
            justify=tk.RIGHT,
            from_=2,
            to=32,
        )
        self.__chessboard_rows.grid(row=0, column=1, pady=4, padx=4, sticky="ew")

        self.__chessboard_cols = Spinbox(
            calibration_frame,
            width=0,
            justify=tk.RIGHT,
            from_=2,
            to=32,
        )
        self.__chessboard_cols.grid(row=0, column=2, pady=4, padx=4, sticky="ew")

        Label(calibration_frame, text="Square size (in cm)").grid(
            row=1, column=0, pady=4, padx=8, sticky="e"
        )

        self.__square_size = Spinbox(
            calibration_frame,
            width=0,
            justify=tk.RIGHT,
            from_=0.02,
            to=100.0,
            increment=0.02,
        )
        self.__square_size.grid(
            row=1, column=1, pady=4, padx=4, columnspan=2, sticky="ew"
        )

        Label(calibration_frame, text="Number of samples").grid(
            row=2, column=0, pady=4, padx=8, sticky="e"
        )

        self.__calibration_count = Entry(
            calibration_frame,
            width=0,
            justify=tk.RIGHT,
            state="readonly",
        )
        self.__calibration_count.grid(
            row=2, column=1, pady=4, padx=4, columnspan=2, sticky="ew"
        )

        self.__start_calibraton_button = Button(
            calibration_frame,
            text="Start",
            command=lambda: self.event_target.emit(
                ButtonEvent("click.start_calibration")
            ),
        )

        self.__start_calibraton_button.grid(
            row=3, column=0, pady=4, padx=4, sticky="ew"
        )

        self.__pause_calibration_button = Button(
            calibration_frame,
            text="Pause",
            command=lambda: self.event_target.emit(
                ButtonEvent("click.pause_calibration")
            ),
        )

        self.__reset_calibration_button = Button(
            calibration_frame,
            text="Reset",
            command=lambda: self.event_target.emit(
                ButtonEvent("click.reset_calibration")
            ),
        )

        self.__reset_calibration_button.grid(
            row=3, column=1, pady=4, padx=4, sticky="ew"
        )

        self.__update_calibration_button = Button(
            calibration_frame,
            text="Update",
            command=lambda: self.event_target.emit(
                ButtonEvent("click.update_calibration")
            ),
        )
        self.__update_calibration_button.grid(
            row=3, column=2, pady=4, padx=4, sticky="ew"
        )

        calibration_frame.grid(
            row=1, column=0, pady=4, ipady=4, padx=8, ipadx=8, sticky="new"
        )
        _ = calibration_frame.columnconfigure((1, 2), weight=1)

        inspect_frame = LabelFrame(self, text="Inspect")

        Label(inspect_frame, text="Coordinate").grid(
            row=0, column=0, pady=4, padx=8, sticky="e"
        )

        self.__inspect_x = Entry(
            inspect_frame,
            width=0,
            justify=tk.RIGHT,
            state="readonly",
        )
        self.__inspect_x.grid(row=0, column=1, pady=4, padx=0, sticky="ew")

        self.__inspect_y = Entry(
            inspect_frame,
            width=0,
            justify=tk.RIGHT,
            state="readonly",
        )
        self.__inspect_y.grid(row=0, column=2, pady=4, padx=4, sticky="ew")

        Label(inspect_frame, text="Disparity").grid(
            row=1, column=0, pady=4, padx=8, sticky="e"
        )

        self.__inspect_disparity = Entry(
            inspect_frame,
            width=0,
            justify=tk.RIGHT,
            state="readonly",
        )
        self.__inspect_disparity.grid(
            row=1, column=1, pady=4, padx=4, columnspan=2, sticky="ew"
        )

        Label(inspect_frame, text="Depth (in m)").grid(
            row=2, column=0, pady=4, padx=8, sticky="e"
        )

        self.__inspect_depth = Entry(
            inspect_frame,
            width=0,
            justify=tk.RIGHT,
            state="readonly",
        )
        self.__inspect_depth.grid(
            row=2, column=1, pady=4, padx=4, columnspan=2, sticky="ew"
        )

        inspect_frame.grid(
            row=2, column=0, pady=4, ipady=4, padx=8, ipadx=8, sticky="new"
        )

        _ = inspect_frame.columnconfigure((1, 2), weight=1)

        disparity_frame = tk.LabelFrame(self, text="Disparity")

        self.__disparity_image = ImageLabel(
            disparity_frame,
            width=StereoVslamWindow.DEFAULT_DISPARITY_WIDTH,
            height=StereoVslamWindow.DEFAULT_DISPARITY_HEIGHT,
        )
        self.__disparity_image.pack(padx=8, pady=8)

        _ = self.__disparity_image.bind("<Motion>", self.__hover)

        disparity_frame.grid(row=0, column=1, padx=8, pady=4, rowspan=3, sticky="nwes")

        self.__is_calibrating_observable: Optional[Observable[bool]] = None
        self.__is_calibrating_disposable: Optional[DisposableBase] = None

        self.__calibration_count_observable: Optional[Observable[int]] = None
        self.__calibration_count_disposable: Optional[DisposableBase] = None

        self.__inspect_observable: Optional[
            Observable[Tuple[float, float, float, float]]
        ] = None
        self.__inspect_disposable: Optional[DisposableBase] = None

    @property
    def event_target(self):
        return self.__event_target

    @property
    def left_image_observable(
        self,
    ) -> Optional[Observable[Union[MatLike, Image, str, None]]]:
        return self.__left_image.image_observable

    @left_image_observable.setter
    def left_image_observable(
        self, observable: Optional[Observable[Union[MatLike, Image, str, None]]]
    ):
        self.__left_image.image_observable = observable

    @left_image_observable.deleter
    def left_image_observable(
        self,
    ):
        del self.__left_image.image_observable

    @property
    def right_image_observable(
        self,
    ) -> Optional[Observable[Union[MatLike, Image, str, None]]]:
        return self.__right_image.image_observable

    @right_image_observable.setter
    def right_image_observable(
        self, observable: Optional[Observable[Union[MatLike, Image, str, None]]]
    ):
        self.__right_image.image_observable = observable

    @right_image_observable.deleter
    def right_image_observable(
        self,
    ):
        del self.__right_image.image_observable

    @property
    def disparity_image_observable(
        self,
    ) -> Optional[Observable[Union[MatLike, Image, str, None]]]:
        return self.__disparity_image.image_observable

    @disparity_image_observable.setter
    def disparity_image_observable(
        self, observable: Optional[Observable[Union[MatLike, Image, str, None]]]
    ):
        self.__disparity_image.image_observable = observable

    @disparity_image_observable.deleter
    def disparity_image_observable(
        self,
    ):
        del self.__disparity_image.image_observable

    @property
    def is_calibrating_observable(self):
        return self.__is_calibrating_observable

    @is_calibrating_observable.setter
    def is_calibrating_observable(self, observable: Optional[Observable[bool]]):
        del self.is_calibrating_observable

        if observable is None:
            return

        self.__is_calibrating_observable = observable
        self.__is_calibrating_disposable = observable.subscribe(
            self.__layout_calibration_button
        )

    @is_calibrating_observable.deleter
    def is_calibrating_observable(self):
        if self.__is_calibrating_disposable is not None:
            self.__is_calibrating_disposable.dispose()
            self.__is_calibrating_disposable = None
        self.__is_calibrating_observable = None

    @property
    def inspect_observable(self):
        return self.__inspect_observable

    @inspect_observable.setter
    def inspect_observable(
        self, observable: Optional[Observable[Tuple[float, float, float, float]]]
    ):
        del self.inspect_observable

        if observable is None:
            return

        self.__inspect_observable = observable
        self.__inspect_disposable = observable.subscribe(self.__update_inspect)

    @inspect_observable.deleter
    def inspect_observable(self):
        if self.__inspect_disposable is not None:
            self.__inspect_disposable.dispose()
            self.__inspect_disposable = None
        self.__inspect_observable = None

    @property
    def calibration_count_observable(self):
        return self.__calibration_count_observable

    @calibration_count_observable.setter
    def calibration_count_observable(self, observable: Optional[Observable[int]]):
        del self.calibration_count_observable

        if observable is None:
            return

        self.__calibration_count_observable = observable
        self.__calibration_count_disposable = observable.subscribe(
            self.__update_count_subscriber
        )

    @calibration_count_observable.deleter
    def calibration_count_observable(self):
        if self.__calibration_count_disposable is not None:
            self.__calibration_count_disposable.dispose()
            self.__calibration_count_disposable = None
        self.__calibration_count_observable = None

    @property
    def chessboard_rows_subject(self):
        return self.__chessboard_rows.value_subject

    @chessboard_rows_subject.setter
    def chessboard_rows_subject(self, subject: Optional[Subject[float]]):
        self.__chessboard_rows.value_subject = subject

    @chessboard_rows_subject.deleter
    def chessboard_rows_subject(self):
        del self.__chessboard_rows.value_subject

    @property
    def chessboard_cols_subject(self):
        return self.__chessboard_cols.value_subject

    @chessboard_cols_subject.setter
    def chessboard_cols_subject(self, subject: Optional[Subject[float]]):
        self.__chessboard_cols.value_subject = subject

    @chessboard_cols_subject.deleter
    def chessboard_cols_subject(self):
        del self.__chessboard_cols.value_subject

    @property
    def square_size_subject(self):
        return self.__square_size.value_subject

    @square_size_subject.setter
    def square_size_subject(self, subject: Optional[Subject[float]]):
        self.__square_size.value_subject = subject

    @square_size_subject.deleter
    def square_size_subject(self):
        del self.__square_size.value_subject

    def __update_count_subscriber(self, x: int):
        if x < self.minimum_samples:
            _ = self.__update_calibration_button.config(state="disabled")
        else:
            _ = self.__update_calibration_button.config(state="normal")
        if self.__calibration_count.text_subject is not None:
            self.__calibration_count.text_subject.on_next(str(x))

    def __hover(self, event: tk.Event[tk.Misc]):
        _event = LabelEvent("hover.disparity", event.widget, event)
        self.event_target.emit(_event)

    def __layout_calibration_button(self, is_calibrating: bool):
        if is_calibrating:
            self.__pause_calibration_button.grid(
                row=3, column=0, pady=4, padx=4, sticky="ew"
            )
            self.__start_calibraton_button.grid_forget()
        else:
            self.__start_calibraton_button.grid(
                row=3, column=0, pady=4, padx=4, sticky="ew"
            )
            self.__pause_calibration_button.grid_forget()

    def __update_inspect(self, values: Tuple[float, float, float, float]):
        _x, _y, _disparity, _depth = values
        if self.__inspect_x.text_subject:
            self.__inspect_x.text_subject.on_next(str(_x))
        if self.__inspect_y.text_subject:
            self.__inspect_y.text_subject.on_next(str(_y))
        if self.__inspect_disparity.text_subject:
            self.__inspect_disparity.text_subject.on_next(str(_disparity))
        if self.__inspect_depth.text_subject:
            self.__inspect_depth.text_subject.on_next(str(_depth))

    @override
    def destroy(self) -> None:
        del self.calibration_count_observable
        del self.is_calibrating_observable
        del self.inspect_observable

        self.__event_target.dispose()
        return super().destroy()
