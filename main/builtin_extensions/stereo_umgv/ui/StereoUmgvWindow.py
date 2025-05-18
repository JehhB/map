import tkinter as tk
from typing import Optional, Union

from cv2.typing import MatLike
from PIL.Image import Image
from reactivex import Observable, Subject, operators
from reactivex.abc import DisposableBase
from reactivex.subject import BehaviorSubject
from typing_extensions import override

from ratmap_common.EventTarget import EventTarget
from tkinter_rx import (
    Button,
    Checkbutton,
    Entry,
    ImageLabel,
    Label,
    LabelFrame,
    Radiobutton,
    Scale,
    Spinbox,
)
from tkinter_rx.util import bind_subject_to_variable

from ..ImageTransformer import ImageTransformer
from .Joystick import Joystick


class StereoUmgvWindow(tk.Toplevel):
    DEFAULT_VIEWER_WIDTH: int = 480
    DEFAULT_VIEWER_HEIGHT: int = 360

    __left_image: ImageLabel
    __right_image: ImageLabel
    joystick: Joystick

    right_ip: BehaviorSubject[str]
    left_ip: BehaviorSubject[str]

    minimum_samples: int = 5

    def __init__(
        self,
        master: tk.Misc,
        x_subject: BehaviorSubject[float],
        y_subject: BehaviorSubject[float],
        scaler: Subject[float],
        motor_controler: Subject[str],
        right_transformer: ImageTransformer,
        left_transformer: ImageTransformer,
        left_ip: BehaviorSubject[str],
        right_ip: BehaviorSubject[str],
    ) -> None:
        super().__init__(master)

        self.title("Ratmap UMGV")

        self.right_ip = right_ip
        self.left_ip = left_ip

        self.__event_target = EventTarget()

        url_connection_frame = tk.Frame(self)

        Label(url_connection_frame, text="Right IP Address").grid(
            row=0, column=0, sticky="e"
        )

        right_entry: Entry = Entry(url_connection_frame, textsubject=self.right_ip)
        right_entry.grid(row=0, column=1, sticky="ew", padx=4)

        Label(url_connection_frame, text="Left IP Address").grid(
            row=1, column=0, sticky="e"
        )

        left_entry: Entry = Entry(url_connection_frame, textsubject=self.left_ip)
        left_entry.grid(row=1, column=1, sticky="ew", padx=4)

        connect_button = Button(
            url_connection_frame, text="Connect", clickevent="stereo_umgv.connect"
        )
        connect_button.event_target.parent = self.__event_target
        connect_button.grid(row=0, column=2, rowspan=2, padx=4, sticky="ns")

        _ = url_connection_frame.columnconfigure(1, weight=1)
        url_connection_frame.pack(padx=8, pady=8, expand=tk.YES, fill=tk.X)

        preview_frame = tk.Frame(self)

        image_transform_frame = tk.Frame(preview_frame)

        Checkbutton(
            image_transform_frame,
            text="Flip vertically",
            valuesubject=left_transformer.flipV,
        ).pack(padx=4, side=tk.RIGHT)
        Checkbutton(
            image_transform_frame,
            text="Flip horizontally",
            valuesubject=left_transformer.flipH,
        ).pack(padx=4, side=tk.RIGHT)
        Button(
            image_transform_frame,
            text="Rotate CCW",
            command=lambda: left_transformer.rotate(3),
        ).pack(padx=4, side=tk.LEFT)
        Button(
            image_transform_frame,
            text="Rotate CW",
            command=lambda: left_transformer.rotate(1),
        ).pack(padx=4, side=tk.LEFT)

        image_transform_frame.grid(row=0, column=0, padx=4, pady=4, sticky="ew")

        self.__left_image = ImageLabel(
            preview_frame,
            width=StereoUmgvWindow.DEFAULT_VIEWER_WIDTH,
            height=StereoUmgvWindow.DEFAULT_VIEWER_HEIGHT,
            bg="black",
        )
        self.__left_image.grid(row=1, column=0, padx=4, pady=4)

        image_transform_frame = tk.Frame(preview_frame)

        Checkbutton(
            image_transform_frame,
            text="Flip vertically",
            valuesubject=right_transformer.flipV,
        ).pack(padx=4, side=tk.RIGHT)
        Checkbutton(
            image_transform_frame,
            text="Flip horizontally",
            valuesubject=right_transformer.flipH,
        ).pack(padx=4, side=tk.RIGHT)
        Button(
            image_transform_frame,
            text="Rotate CCW",
            command=lambda: right_transformer.rotate(3),
        ).pack(padx=4, side=tk.LEFT)
        Button(
            image_transform_frame,
            text="Rotate CW",
            command=lambda: right_transformer.rotate(1),
        ).pack(padx=4, side=tk.LEFT)

        image_transform_frame.grid(row=0, column=1, padx=4, pady=4, sticky="ew")

        self.__right_image = ImageLabel(
            preview_frame,
            width=StereoUmgvWindow.DEFAULT_VIEWER_WIDTH,
            height=StereoUmgvWindow.DEFAULT_VIEWER_HEIGHT,
            bg="black",
        )
        self.__right_image.grid(row=1, column=1, padx=4, pady=4)

        _ = preview_frame.rowconfigure(0, weight=1)
        _ = preview_frame.columnconfigure((0, 1), weight=1)
        preview_frame.pack(padx=8, pady=8, expand=tk.YES, fill=tk.BOTH)

        control_frame = tk.Frame(self)

        joystick_frame = LabelFrame(control_frame, text="Control")

        self.joystick = Joystick(joystick_frame, x_subject, y_subject, 150)
        self.joystick.grid(row=0, column=0, padx=4, pady=4, rowspan=7)

        scaler_label = scaler.pipe(operators.map(lambda x: "Speed scaler (%.2f)" % x))

        Label(joystick_frame, textobservable=scaler_label).grid(
            row=1, column=1, padx=4, columnspan=2, sticky="w"
        )

        Scale(joystick_frame, valuesubject=scaler, from_=0.0, to=1.0).grid(
            row=2, column=1, padx=4, columnspan=2, sticky="ew"
        )

        Label(joystick_frame, text="Controller select").grid(
            row=4, column=1, padx=4, columnspan=2, sticky="w"
        )

        variable = tk.StringVar(self, "right")
        self.__control_disposer = bind_subject_to_variable(
            motor_controler, variable, self
        )

        Radiobutton(
            joystick_frame, text="Right", value="right", variable=variable
        ).grid(row=5, column=1, padx=4, sticky="w")

        Radiobutton(joystick_frame, text="Left", value="left", variable=variable).grid(
            row=5, column=2, padx=4, sticky="w"
        )

        _ = joystick_frame.rowconfigure((0, 6), weight=2)
        _ = joystick_frame.rowconfigure(3, weight=1)
        _ = joystick_frame.columnconfigure((1, 2), weight=1)
        joystick_frame.grid(row=0, column=0, sticky="ew", padx=4, ipadx=4, ipady=8)

        calibration_frame = LabelFrame(control_frame, text="Calibration")

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
            text="Calibrate",
            clickevent="stereo_umgv.start_calibration",
        )
        self.__start_calibraton_button.event_target.parent = self.__event_target
        self.__start_calibraton_button.grid(
            row=3, column=0, pady=4, padx=4, sticky="ew"
        )

        self.__pause_calibration_button = Button(
            calibration_frame,
            text="Pause",
            clickevent="stereo_umgv.pause_calibration",
        )
        self.__pause_calibration_button.event_target.parent = self.__event_target

        self.__reset_calibration_button = Button(
            calibration_frame,
            text="Reset",
            clickevent="stereo_umgv.reset_calibration",
        )
        self.__reset_calibration_button.event_target.parent = self.__event_target
        self.__reset_calibration_button.grid(
            row=3, column=1, pady=4, padx=4, sticky="ew"
        )

        self.__update_calibration_button = Button(
            calibration_frame,
            text="Update",
            clickevent="stereo_umgv.update_calibration",
        )
        self.__update_calibration_button.event_target.parent = self.__event_target
        self.__update_calibration_button.grid(
            row=3, column=2, pady=4, padx=4, sticky="ew"
        )

        self.__start_mapping_button = Button(
            calibration_frame,
            text="Start Mapping",
            clickevent="stereo_umgv.start_mapping",
        )
        self.__start_mapping_button.event_target.parent = self.__event_target
        self.__start_mapping_button.grid(
            row=4, column=0, pady=4, padx=4, sticky="ew", columnspan=3
        )

        calibration_frame.grid(
            row=0, column=1, pady=4, ipady=4, padx=8, ipadx=8, sticky="new"
        )
        _ = calibration_frame.columnconfigure((1, 2), weight=1)

        _ = control_frame.columnconfigure((0, 1), weight=1)
        control_frame.pack(padx=8, pady=8, expand=tk.YES, fill=tk.X)

        self.__is_calibrating_observable: Optional[Observable[bool]] = None
        self.__is_calibrating_disposable: Optional[DisposableBase] = None

        self.__calibration_count_observable: Optional[Observable[int]] = None
        self.__calibration_count_disposable: Optional[DisposableBase] = None

    @property
    def event_target(self):
        return self.__event_target

    @override
    def destroy(self) -> None:
        del self.calibration_count_observable
        del self.is_calibrating_observable
        self.__event_target.dispose()
        self.__control_disposer.dispose()
        return super().destroy()

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
