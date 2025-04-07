from typing_extensions import Unpack

from tkinter_rx import Menu, MenuEvent
from tkinter_rx.typing import SubMenuKwargs


class MenuEdit(Menu):
    def __init__(self, menu_bar: Menu, **kwargs: Unpack[SubMenuKwargs]) -> None:
        super().__init__(menu_bar, **kwargs, tearoff=0)

        self.add_command(label="Clear", command=self.edit_clear)
        self.add_command(label="Undo", command=self.edit_undo)
        self.add_command(label="Redo", command=self.edit_redo)
        self.add_separator()
        self.add_command(label="Preferences", command=self.edit_preferences)

        menu_bar.add_cascade(label="Edit", menu=self)

    def edit_clear(self):
        self.emit(MenuEvent.factory(self, "menu_main", "edit", "clear"))

    def edit_undo(self):
        self.emit(MenuEvent.factory(self, "menu_main", "edit", "undo"))

    def edit_redo(self):
        self.emit(MenuEvent.factory(self, "menu_main", "edit", "redo"))

    def edit_preferences(self):
        self.emit(MenuEvent.factory(self, "menu_main", "edit", "preferences"))
