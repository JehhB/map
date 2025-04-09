import tkinter as tk
from fileinput import filename
from tkinter import filedialog, messagebox
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from stereo_vslam import StereoVslamExtension
    from stereo_vslam.ui.Main import Main


class MenuBar(tk.Menu):
    file_menu: tk.Menu
    extension: "StereoVslamExtension"

    def __init__(
        self, master: "Main", extension: "StereoVslamExtension", *args, **kwargs
    ) -> None:
        super().__init__(master, *args, **kwargs)
        self.extension = extension

        self.file_menu = tk.Menu(self, tearoff=0)
        self.file_menu.add_command(
            label="Load calibration", command=self.load_calibration
        )
        self.file_menu.add_command(
            label="Save calibration", command=self.save_calibration
        )

        self.add_cascade(label="File", menu=self.file_menu)

    def load_calibration(self):
        filename = filedialog.askopenfilename(
            parent=self.master,
            title="Load calibration info",
            filetypes=[("YAML files", "*.yaml")],
            defaultextension="yaml",
        )

        if not filename:
            return

        success = self.extension.calibrator.load(filename)
        if not success:
            _ = messagebox.showerror(
                "Failed to load", "Failed to load calibration info"
            )

    def save_calibration(self):
        filename = filedialog.asksaveasfilename(
            parent=self.master,
            title="Save calibration info",
            filetypes=[("YAML files", "*.yaml")],
            defaultextension="yaml",
        )
        if not filename:
            return

        success = self.extension.calibrator.save(filename)
        if not success:
            _ = messagebox.showerror(
                "Failed to save", "Failed to save calibration info"
            )
