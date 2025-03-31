import tkinter as tk
from typing import TYPE_CHECKING, Optional, Set, Tuple

from PIL.Image import Image
from reactivex.abc import DisposableBase
from typing_extensions import override

from app.MappedBehaviourSubject import MappedBehaviourSubject
from app.ui.ImageLabel import ImageLabel
from app.ui.utils import (
    bind_subject_to_widget,
    safe_callback,
    safe_parse_float,
    safe_parse_int,
)
from stereo_vslam.Calibrator import CalibratorParams
from stereo_vslam.ui.MenuBar import MenuBar

if TYPE_CHECKING:
    from stereo_vslam import StereoVslamExtension


class Main(tk.Toplevel):
    extension: "StereoVslamExtension"
    DEFAULT_VIEWER_WIDTH: int = 240
    DEFAULT_VIEWER_HEIGHT: int = 180

    DEFAULT_DISPARITY_WIDTH: int = 800
    DEFAULT_DISPARITY_HEIGHT: int = 600

    menu_bar: MenuBar

    left_image: ImageLabel
    right_image: ImageLabel
    disparity_image: ImageLabel

    disposers: Set[DisposableBase]

    inspect_x: tk.IntVar
    inspect_y: tk.IntVar
    inspect_disparity: tk.DoubleVar

    def __init__(
        self,
        extension: "StereoVslamExtension",
        master: Optional[tk.Misc] = None,
        *args,
        **kwargs
    ) -> None:
        super().__init__(master, *args, **kwargs)
        self.extension = extension
        self.title("Stereo VSLAM")

        self.disposers = set()

        self.menu_bar = MenuBar(self, extension)
        _ = self.config(menu=self.menu_bar)

        viewer_frame = tk.LabelFrame(self, text="Videos")

        self.left_image = ImageLabel(
            viewer_frame,
            width=Main.DEFAULT_VIEWER_WIDTH,
            height=Main.DEFAULT_VIEWER_HEIGHT,
            image_observable=self.extension.left_image_subject,
        )
        self.left_image.pack(pady=4, expand=tk.FALSE)

        self.right_image = ImageLabel(
            viewer_frame,
            width=Main.DEFAULT_VIEWER_WIDTH,
            height=Main.DEFAULT_VIEWER_HEIGHT,
            image_observable=self.extension.right_image_subject,
        )
        self.right_image.pack(pady=4, expand=tk.FALSE)

        viewer_frame.grid(row=0, column=0, pady=4, ipady=4, padx=8, ipadx=8)

        calibration_frame = tk.LabelFrame(self, text="Calibration")

        tk.Label(calibration_frame, text="Board size").grid(
            row=0, column=0, pady=4, padx=8, sticky="e"
        )

        def update_calibrator_params(
            params: CalibratorParams,
            size: Optional[Tuple[Optional[int], Optional[int]]] = None,
            square_size: Optional[float] = None,
        ) -> CalibratorParams:
            out = params.copy()
            if square_size is not None:
                out["square_size"] = square_size
            if size is not None and size[0] is not None:
                out["chessboard_size"] = (size[0], out["chessboard_size"][1])
            if size is not None and size[1] is not None:
                out["chessboard_size"] = (out["chessboard_size"][0], size[1])
            return out

        board_x_subject = MappedBehaviourSubject(
            "7",
            self.extension.calibrator_params,
            lambda params: str(params["chessboard_size"][0]),
            lambda val, params: update_calibrator_params(
                params, size=(safe_parse_int(val, 7), None)
            ),
        )
        board_x = tk.Spinbox(
            calibration_frame,
            width=0,
            justify=tk.RIGHT,
            from_=2,
            to=32,
        )
        self.disposers.add(bind_subject_to_widget(board_x, board_x_subject))
        board_x.grid(row=0, column=1, pady=4, padx=4, sticky="ew")

        board_y_subject = MappedBehaviourSubject(
            "8",
            self.extension.calibrator_params,
            lambda params: str(params["chessboard_size"][1]),
            lambda val, params: update_calibrator_params(
                params, size=(None, safe_parse_int(val, 8))
            ),
        )
        board_y = tk.Spinbox(
            calibration_frame,
            width=0,
            justify=tk.RIGHT,
            from_=2,
            to=32,
        )
        self.disposers.add(bind_subject_to_widget(board_y, board_y_subject))
        board_y.grid(row=0, column=2, pady=4, padx=4, sticky="ew")

        tk.Label(calibration_frame, text="Square size (in mm)").grid(
            row=1, column=0, pady=4, padx=8, sticky="e"
        )

        square_size_subject = MappedBehaviourSubject(
            "30.0",
            self.extension.calibrator_params,
            lambda params: str(params["square_size"]),
            lambda val, params: update_calibrator_params(
                params, square_size=safe_parse_float(val, 30.0)
            ),
        )
        square_size = tk.Spinbox(
            calibration_frame,
            width=0,
            justify=tk.RIGHT,
            from_=0.02,
            to=100.0,
            increment=0.02,
        )
        self.disposers.add(bind_subject_to_widget(square_size, square_size_subject))
        square_size.grid(row=1, column=1, pady=4, padx=4, columnspan=2, sticky="ew")

        tk.Label(calibration_frame, text="Number of samples").grid(
            row=2, column=0, pady=4, padx=8, sticky="e"
        )

        sample_count_var = tk.IntVar(self, 0)
        tk.Entry(
            calibration_frame,
            width=0,
            textvariable=sample_count_var,
            justify=tk.RIGHT,
            state="readonly",
        ).grid(row=2, column=1, pady=4, padx=4, columnspan=2, sticky="ew")

        start_calibraton_button = tk.Button(
            calibration_frame, text="Start calibration", command=self.start_calibration
        )

        end_calibraton_button = tk.Button(
            calibration_frame, text="Stop calibration", command=self.end_calibration
        )

        def layout_calibration_button(is_calibrating: bool):
            if is_calibrating:
                end_calibraton_button.grid(
                    row=3, column=0, pady=4, padx=4, columnspan=3, sticky="ew"
                )
                start_calibraton_button.grid_forget()
            else:
                start_calibraton_button.grid(
                    row=3, column=0, pady=4, padx=4, columnspan=3, sticky="ew"
                )
                end_calibraton_button.grid_forget()

        self.disposers.add(
            self.extension.calibrator.is_calibrating.subscribe(
                safe_callback(self, layout_calibration_button)
            )
        )

        def calibration_count_subscriber(x: int):
            if x < self.extension.calibrator.MINIMUM_NUMBER_SAMPLE:
                _ = end_calibraton_button.config(state="disabled")
            else:
                _ = end_calibraton_button.config(state="normal")
            sample_count_var.set(x)

        self.disposers.add(
            self.extension.calibrator.number_of_samples.subscribe(
                safe_callback(self, calibration_count_subscriber)
            )
        )

        calibration_frame.grid(
            row=1, column=0, pady=4, ipady=4, padx=8, ipadx=8, sticky="new"
        )
        _ = calibration_frame.columnconfigure((1, 2), weight=1)

        self.inspect_x = tk.IntVar(self, value=None)
        self.inspect_y = tk.IntVar(self, value=None)
        self.inspect_disparity = tk.DoubleVar(self, value=None)

        inspect_frame = tk.LabelFrame(self, text="Inspect")

        tk.Label(inspect_frame, text="Coordinate").grid(
            row=0, column=0, pady=4, padx=8, sticky="e"
        )

        tk.Entry(
            inspect_frame,
            textvariable=self.inspect_x,
            width=0,
            justify=tk.RIGHT,
            state="readonly",
        ).grid(row=0, column=1, pady=4, padx=0, sticky="ew")

        tk.Entry(
            inspect_frame,
            textvariable=self.inspect_y,
            width=0,
            justify=tk.RIGHT,
            state="readonly",
        ).grid(row=0, column=2, pady=4, padx=4, sticky="ew")

        tk.Label(inspect_frame, text="Disparity").grid(
            row=1, column=0, pady=4, padx=8, sticky="e"
        )

        tk.Entry(
            inspect_frame,
            textvariable=self.inspect_disparity,
            width=0,
            justify=tk.RIGHT,
            state="readonly",
        ).grid(row=1, column=1, pady=4, padx=4, columnspan=2, sticky="ew")

        inspect_frame.grid(
            row=2, column=0, pady=4, ipady=4, padx=8, ipadx=8, sticky="new"
        )

        _ = inspect_frame.columnconfigure((1, 2), weight=1)

        disparity_frame = tk.LabelFrame(self, text="Disparity")

        self.disparity_image = ImageLabel(
            disparity_frame,
            width=Main.DEFAULT_DISPARITY_WIDTH,
            height=Main.DEFAULT_DISPARITY_HEIGHT,
        )

        self.disparity_image.pack(padx=8, pady=8)

        disparity_frame.grid(row=0, column=1, padx=8, pady=4, rowspan=3, sticky="nwes")

    def update_canvas(self, canvas: tk.Canvas, image: Image):
        image = image.resize((Main.DEFAULT_VIEWER_WIDTH, Main.DEFAULT_VIEWER_HEIGHT))
        canvas.delete("all")
        _ = canvas.create_image(
            Main.DEFAULT_VIEWER_WIDTH // 2, Main.DEFAULT_VIEWER_HEIGHT // 2, image=image
        )

    def start_calibration(self):
        self.extension.start_calibration()

    def end_calibration(self):
        self.extension.calibrator.stop()

    @override
    def destroy(self) -> None:
        for disposer in self.disposers:
            disposer.dispose()
        return super().destroy()
