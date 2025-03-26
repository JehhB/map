import tkinter as tk
from struct import pack
from typing import TYPE_CHECKING

from PIL.Image import Image
from PIL.ImageOps import expand

from stereo_vslam.ui.MenuBar import MenuBar

if TYPE_CHECKING:
    from stereo_vslam import StereoVslamExtension


class Main(tk.Toplevel):
    extension: "StereoVslamExtension"
    DEFAULT_VIEWER_WIDTH: int = 240
    DEFAULT_VIEWER_HEIGHT: int = 180

    left_image: tk.Canvas
    right_image: tk.Canvas

    chessboard_x: tk.IntVar
    chessboard_y: tk.IntVar
    chessboard_size: tk.DoubleVar
    calibration_sample: tk.IntVar
    calibration_progress: int

    def __init__(
        self,
        extension: "StereoVslamExtension",
        master: tk.Misc | None = None,
        *args,
        **kwargs
    ) -> None:
        super().__init__(master, *args, **kwargs)
        self.extension = extension
        self.title("Stereo VSLAM")

        self.menu_bar = MenuBar(self, extension)
        self.config(menu=self.menu_bar)

        viewer_frame = tk.LabelFrame(self, text="Videos")

        self.left_image = tk.Canvas(
            viewer_frame,
            width=Main.DEFAULT_VIEWER_WIDTH,
            height=Main.DEFAULT_VIEWER_HEIGHT,
            bg="black",
        )
        self.left_image.pack(pady=4, expand=tk.FALSE)

        self.right_image = tk.Canvas(
            viewer_frame,
            width=Main.DEFAULT_VIEWER_WIDTH,
            height=Main.DEFAULT_VIEWER_HEIGHT,
            bg="black",
        )
        self.right_image.pack(pady=4, expand=tk.FALSE)

        viewer_frame.grid(row=0, column=0, pady=4, ipady=4, padx=8, ipadx=8)

        self.chessboard_x = tk.IntVar(self, value=6)
        self.chessboard_y = tk.IntVar(self, value=4)
        self.chessboard_size = tk.DoubleVar(self, value=20.0)
        self.calibration_sample = tk.IntVar(self, value=30)
        self.calibration_progress = 0

        calibration_frame = tk.LabelFrame(self, text="Calibration")

        tk.Label(calibration_frame, text="Board size").grid(
            row=0, column=0, pady=4, padx=8, sticky="e"
        )

        tk.Entry(
            calibration_frame, textvariable=self.chessboard_x, width=0, justify=tk.RIGHT
        ).grid(row=0, column=1, pady=4, padx=0, sticky="ew")

        tk.Entry(
            calibration_frame, textvariable=self.chessboard_y, width=0, justify=tk.RIGHT
        ).grid(row=0, column=2, pady=4, padx=4, sticky="ew")

        tk.Label(calibration_frame, text="Square size (in mm)").grid(
            row=1, column=0, pady=4, padx=8, sticky="e"
        )

        tk.Entry(
            calibration_frame,
            textvariable=self.chessboard_size,
            width=0,
            justify=tk.RIGHT,
        ).grid(row=1, column=1, pady=4, padx=4, columnspan=2, sticky="ew")

        tk.Label(calibration_frame, text="Number of samples").grid(
            row=2, column=0, pady=4, padx=8, sticky="e"
        )

        tk.Entry(
            calibration_frame,
            textvariable=self.calibration_sample,
            width=0,
            justify=tk.RIGHT,
        ).grid(row=2, column=1, pady=4, padx=4, columnspan=2, sticky="ew")

        tk.Button(
            calibration_frame, text="Start calibration", command=self.start_calibration
        ).grid(row=3, column=0, pady=4, padx=4, columnspan=3, sticky="ew")

        calibration_frame.grid(
            row=1, column=0, pady=4, ipady=4, padx=8, ipadx=8, sticky="new"
        )
        _ = calibration_frame.columnconfigure((1, 2), weight=1)

        _ = self.columnconfigure(1, weight=1)

    def update_canvas(self, canvas: tk.Canvas, image: Image):
        image = image.resize((Main.DEFAULT_VIEWER_WIDTH, Main.DEFAULT_VIEWER_HEIGHT))
        canvas.delete("all")
        _ = canvas.create_image(
            Main.DEFAULT_VIEWER_WIDTH // 2, Main.DEFAULT_VIEWER_HEIGHT // 2, image=image
        )

    def start_calibration(self):
        pass
