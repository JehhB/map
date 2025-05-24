from typing_extensions import Unpack

from tkinter_rx import Menu, MenuEvent
from tkinter_rx.typing import SubMenuKwargs


class MenuEdit(Menu):
    def __init__(self, menu_bar: Menu, **kwargs: Unpack[SubMenuKwargs]) -> None:
        super().__init__(menu_bar, **kwargs, tearoff=0)

        self.add_command(label="Export map", command=self.edit_export_map)

        menu_bar.add_cascade(label="Edit", menu=self)

    def edit_export_map(self):
        self.emit(MenuEvent.factory(self, "menu_main", "edit", "export_map"))
