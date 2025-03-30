import tkinter as tk
from tkinter import filedialog
from typing import TYPE_CHECKING, Optional

import numpy
from PIL import ImageTk
from PIL.Image import Image
from reactivex.abc import DisposableBase

from app.Container import Container
from app.ui.ImageLabel import ImageLabel
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

    left_disposer: DisposableBase
    right_disposer: DisposableBase

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
            control_frame, from_=5, to=40, variable=self.fps, orient=tk.HORIZONTAL
        ).pack(side=tk.LEFT, expand=tk.YES, fill=tk.X, padx=4)

        self.fps_callback = self.fps.trace_add("write", self.set_fps)

        tk.Button(control_frame, text="Start Mapping", command=self.start).pack(
            side=tk.LEFT, padx=4
        )

        tk.Button(control_frame, text="Start Callibration").pack(side=tk.LEFT, padx=4)

        tk.Button(control_frame, text="Pause", state="disabled").pack(
            side=tk.LEFT, padx=4
        )
        tk.Button(control_frame, text="Step", state="disabled").pack(
            side=tk.LEFT, padx=4
        )
        tk.Button(control_frame, text="Play", state="disabled").pack(
            side=tk.LEFT, padx=4
        )
        tk.Button(control_frame, text="Stop", state="disabled").pack(
            side=tk.LEFT, padx=4
        )

        control_frame.pack(padx=8, pady=8, expand=tk.YES, fill=tk.X)

        self.left_disposer = self._stereo_extension.left_image_subject.subscribe(
            on_next=self.left_image.update_from_pil
        )

        self.right_disposer = self._stereo_extension.right_image_subject.subscribe(
            on_next=self.right_image.update_from_pil
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

    def dispose(self):
        self.left_disposer.dispose()
        self.right_disposer.dispose()

    def start(self):
        if self._extension.player is None:
            return
        self._extension.player.start()

    def set_fps(self, _a: str, _b: str, _c: str):
        if self._extension.player is None:
            return
        self._extension.player.set_frame_rate(self.fps.get())
