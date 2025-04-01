import tkinter as tk
from tkinter import filedialog
from typing import TYPE_CHECKING, List, Optional, Set

import reactivex.operators as ops

from app.Container import Container
from app.ui.ImageLabel import ImageLabel
from euroc_mav_loader.Player import PlayerState
from stereo_vslam import StereoVslamExtension

if TYPE_CHECKING:
    from euroc_mav_loader import EuRoCMAVLoaderExtension


class Main(tk.Toplevel):
    folder_path: tk.StringVar
    fps: tk.IntVar
    _container: Container
    _extension: "EuRoCMAVLoaderExtension"
    _stereo_extension: StereoVslamExtension

    DEFAULT_VIEWER_WIDTH: int = 480
    DEFAULT_VIEWER_HEIGHT: int = 360

    left_image: ImageLabel
    right_image: ImageLabel

    fps_callback: str

    start_map_button: tk.Button
    start_calib_button: tk.Button
    pause_button: tk.Button
    step_button: tk.Button
    play_button: tk.Button
    stop_button: tk.Button

    def __init__(
        self,
        container: Container,
        master: tk.Misc,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(master, *args, **kwargs)
        self.title("EuRoC MAV Loader")
        self._container = container

        self._extension: "EuRoCMAVLoaderExtension" = container["euroc_mac_loader.main"]
        self._stereo_extension = container[StereoVslamExtension]

        self.folder_path = tk.StringVar(
            self,
            "/mnt/c/Users/ASUS/Downloads/cam_checkerboard",
        )
        self.fps = tk.IntVar(self, 20)

        file_selection_frame = tk.Frame(self)

        tk.Label(file_selection_frame, text="Dataset Location").pack(side=tk.LEFT)

        tk.Entry(file_selection_frame, textvariable=self.folder_path).pack(
            side=tk.LEFT, fill=tk.X, expand=tk.YES, padx=4
        )
        tk.Button(file_selection_frame, text="Choose", command=self._choose_path).pack(
            side=tk.LEFT, padx=4
        )
        tk.Button(
            file_selection_frame,
            text="Load",
            command=lambda: self._extension.load_dataset(self.folder_path.get()),
        ).pack(side=tk.LEFT, padx=4)

        file_selection_frame.pack(padx=8, pady=8, expand=tk.YES, fill=tk.X)

        preview_frame = tk.Frame(self)

        self.left_image = ImageLabel(
            preview_frame,
            width=Main.DEFAULT_VIEWER_WIDTH,
            height=Main.DEFAULT_VIEWER_HEIGHT,
            bg="black",
        )
        self.left_image.grid(row=0, column=0, padx=4, pady=4)

        self.right_image = ImageLabel(
            preview_frame,
            width=Main.DEFAULT_VIEWER_WIDTH,
            height=Main.DEFAULT_VIEWER_HEIGHT,
            bg="black",
        )
        self.right_image.grid(row=0, column=1, padx=4, pady=4)

        _ = preview_frame.rowconfigure(0, weight=1)
        _ = preview_frame.columnconfigure((0, 1), weight=1)
        preview_frame.pack(padx=8, pady=8, expand=tk.YES, fill=tk.BOTH)

        control_frame = tk.Frame(self)

        tk.Label(control_frame, text="Speed").pack(side=tk.LEFT)
        tk.Scale(
            control_frame, from_=5, to=60, variable=self.fps, orient=tk.HORIZONTAL
        ).pack(side=tk.LEFT, expand=tk.YES, fill=tk.X, padx=4)
        self.fps_callback = self.fps.trace_add("write", self.set_fps)

        self.start_map_button = tk.Button(
            control_frame, text="Start Mapping", command=self.start
        )
        self.start_map_button.pack(side=tk.LEFT, padx=4)

        self.start_calib_button = tk.Button(
            control_frame, text="Start Callibration", command=self.start_calibration
        )
        self.start_calib_button.pack(side=tk.LEFT, padx=4)

        self.pause_button = tk.Button(control_frame, text="Pause", command=self.pause)
        self.pause_button.pack(side=tk.LEFT, padx=4)

        self.step_button = tk.Button(control_frame, text="Step", command=self.step)
        self.step_button.pack(side=tk.LEFT, padx=4)

        self.play_button = tk.Button(control_frame, text="Play", command=self.play)
        self.play_button.pack(side=tk.LEFT, padx=4)

        self.stop_button = tk.Button(control_frame, text="Stop", command=self.stop)
        self.stop_button.pack(side=tk.LEFT, padx=4)

        control_frame.pack(padx=8, pady=8, expand=tk.YES, fill=tk.X)

        self.left_image.image_observable = (
            self._stereo_extension.calibrator.is_calibrating.pipe(
                ops.map(
                    lambda is_calibrating: (
                        self._stereo_extension.calibrator.left_image_with_drawing
                        if is_calibrating
                        else self._stereo_extension.left_image_subject
                    )
                ),
                ops.switch_latest(),
            )
        )

        self.right_image.image_observable = (
            self._stereo_extension.calibrator.is_calibrating.pipe(
                ops.map(
                    lambda is_calibrating: (
                        self._stereo_extension.calibrator.right_image_with_drawing
                        if is_calibrating
                        else self._stereo_extension.right_image_subject
                    )
                ),
                ops.switch_latest(),
            )
        )

    def _choose_path(self):
        dir = filedialog.askdirectory(
            parent=self,
            mustexist=True,
            title="Choose EuRoC Mav Dataset directory",
            initialdir=self.folder_path.get(),
        )
        if isinstance(dir, str):
            self.folder_path.set(dir)

    def start_calibration(self):
        self._stereo_extension.calibrator_params.on_next(
            {
                "chessboard_size": (6, 7),
                "square_size": 60.0,
            }
        )
        self._stereo_extension.start_calibration()
        if self._extension.player is None:
            return
        _ = self._extension.player.start()

    def start(self):
        if self._extension.player is None:
            return
        self._stereo_extension.stop_calibration()
        _ = self._extension.player.start()

    def play(self):
        if self._extension.player is None:
            return
        _ = self._extension.player.resume()

    def pause(self):
        if self._extension.player is None:
            return
        _ = self._extension.player.pause()

    def stop(self):
        if self._extension.player is None:
            return
        if (
            self._stereo_extension.calibrator.is_calibrating.value
            and self._stereo_extension.calibrator.number_of_samples.value
            > self._stereo_extension.calibrator.MINIMUM_NUMBER_SAMPLE
        ):
            _ = self._stereo_extension.calibrator.update_calibration()
        _ = self._extension.player.stop()

    def step(self):
        if self._extension.player is None:
            return
        _ = self._extension.player.step()

    def update_player_state(self, state: Optional[PlayerState] = None):
        def set_state(button: tk.Button, *states: Optional[PlayerState]):
            _ = button.config(state="normal" if state in states else "disabled")

        set_state(self.start_map_button, "idle")
        set_state(self.start_calib_button, "idle")
        set_state(self.pause_button, "playing")
        set_state(self.step_button, "playing", "paused")
        set_state(self.play_button, "paused")
        set_state(self.stop_button, "paused", "playing")

    def set_fps(self, _a: str, _b: str, _c: str):
        if self._extension.player is None:
            return
        self._extension.player.set_frame_rate(self.fps.get())
