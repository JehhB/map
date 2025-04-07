import tkinter as tk

from typing_extensions import Unpack

from tkinter_rx import Menu
from tkinter_rx.typing import MenuKwargs

from .MenuEdit import MenuEdit
from .MenuExtension import MenuExtension
from .MenuFile import MenuFile
from .MenuNavigate import MenuNavigate


class MenuMain(Menu):
    file_menu: MenuFile
    extension_menu: MenuExtension
    edit_menu: MenuEdit
    navigate_menu: MenuNavigate

    def __init__(self, main_window: tk.Misc, **kwargs: Unpack[MenuKwargs]) -> None:
        super().__init__(main_window, **kwargs)

        self.file_menu = MenuFile(self)
        self.file_menu.event_target.parent = self.event_target

        self.edit_menu = MenuEdit(self)
        self.edit_menu.event_target.parent = self.event_target

        self.navigate_menu = MenuNavigate(self)
        self.navigate_menu.event_target.parent = self.event_target

        self.extension_menu = MenuExtension(self)
        self.extension_menu.event_target.parent = self.event_target
