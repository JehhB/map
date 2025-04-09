import tkinter as tk
from tkinter import filedialog
from typing import Optional

from typing_extensions import Unpack

from tkinter_rx import Menu, MenuEvent
from tkinter_rx.typing import MenuKwargs


class StereoVslamMenu(Menu):
    __file_menu: Menu

    def __init__(
        self, master: Optional[tk.Misc] = None, **kwargs: Unpack[MenuKwargs]
    ) -> None:
        super().__init__(master, **kwargs)

        self.__file_menu = Menu(self, tearoff=0)
        self.__file_menu.add_command(
            label="Load calibration", command=self.__load_calibration
        )
        self.__file_menu.add_command(
            label="Save calibration", command=self.__save_calibration
        )

        self.add_cascade(label="File", menu=self.__file_menu)

    def __load_calibration(self):
        filename = filedialog.askopenfilename(
            parent=self.master,
            title="Load calibration info",
            filetypes=[("YAML files", "*.yaml")],
            defaultextension="yaml",
        )

        if not filename:
            return

        event = MenuEvent.factory(
            self.__file_menu, "stereo_vslam_menu", "file", "load_calibration"
        )
        event.detail.additional = filename
        self.emit(event)

    def __save_calibration(self):
        filename = filedialog.asksaveasfilename(
            parent=self.master,
            title="Save calibration info",
            filetypes=[("YAML files", "*.yaml")],
            defaultextension="yaml",
        )
        if not filename:
            return

        event = MenuEvent.factory(
            self.__file_menu, "stereo_vslam_menu", "file", "save_calibration"
        )
        event.detail.additional = filename
        self.emit(event)
