import tkinter as tk
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from stereo_vslam import StereoVslamExtension
    from stereo_vslam.ui.Main import Main


class MenuBar(tk.Menu):
    file_menu: tk.Menu

    def __init__(
        self, master: "Main", container: "StereoVslamExtension", *args, **kwargs
    ) -> None:
        super().__init__(master, *args, **kwargs)

        self.file_menu = tk.Menu(self, tearoff=0)
        self.file_menu.add_command(
            label="Load calibration", command=self.load_calibration
        )
        self.file_menu.add_command(
            label="Save calibration", command=self.save_calibration
        )

        self.add_cascade(label="File", menu=self.file_menu)

    def load_calibration(self):
        pass

    def save_calibration(self):
        pass
