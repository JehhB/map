from typing_extensions import Unpack

from tkinter_rx import Menu, MenuEvent
from tkinter_rx.typing import SubMenuKwargs


class MenuExtension(Menu):
    def __init__(self, menu_bar: Menu, **kwargs: Unpack[SubMenuKwargs]) -> None:
        super().__init__(menu_bar, **kwargs, tearoff=0)

        self.add_command(label="Add extension", command=self.extension_add)
        self.add_command(label="Manage extension", command=self.extension_manage)
        self.add_separator()
        menu_bar.add_cascade(label="Extensions", menu=self)

    def extension_add(self):
        self.emit(MenuEvent.factory(self, "menu_main", "extension", "add"))

    def extension_manage(self):
        self.emit(MenuEvent.factory(self, "menu_main", "extension", "manage"))
