from typing_extensions import Unpack

from tkinter_rx import Menu, MenuEvent
from tkinter_rx.typing import SubMenuKwargs


class MenuFile(Menu):
    def __init__(self, menu_bar: Menu, **kwargs: Unpack[SubMenuKwargs]) -> None:
        super().__init__(menu_bar, **kwargs, tearoff=0)

        self.add_command(label="Open", command=self.file_open)
        self.add_command(label="Save", command=self.file_save)
        self.add_command(label="Export", command=self.file_export)

        menu_bar.add_cascade(label="File", menu=self)

    def file_open(self):
        self.emit(MenuEvent.factory(self, "menu_main", "file", "open"))

    def file_save(self):
        self.emit(MenuEvent.factory(self, "menu_main", "file", "save"))

    def file_export(self):
        self.emit(MenuEvent.factory(self, "menu_main", "file", "export"))
