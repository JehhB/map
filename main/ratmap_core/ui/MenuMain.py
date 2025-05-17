import tkinter as tk

from typing_extensions import Unpack

from tkinter_rx import Menu
from tkinter_rx.typing import MenuKwargs

from .MenuEdit import MenuEdit
from .MenuExtension import MenuExtension
from .MenuNavigate import MenuNavigate
from .MenuView import MenuView


class MenuMain(Menu):
    __extension_menu: MenuExtension
    __edit_menu: MenuEdit
    __navigate_menu: MenuNavigate
    __view_menu: MenuView

    def __init__(self, main_window: tk.Misc, **kwargs: Unpack[MenuKwargs]) -> None:
        super().__init__(main_window, **kwargs)

        self.__edit_menu = MenuEdit(self)
        self.__edit_menu.event_target.parent = self.event_target

        self.__view_menu = MenuView(self)
        self.__view_menu.event_target.parent = self.event_target

        self.__navigate_menu = MenuNavigate(self)
        self.__navigate_menu.event_target.parent = self.event_target

        self.__extension_menu = MenuExtension(self)
        self.__extension_menu.event_target.parent = self.event_target

    @property
    def edit_menu(self):
        return self.__edit_menu

    @property
    def navigate_menu(self):
        return self.__navigate_menu

    @property
    def extension_menu(self):
        return self.__extension_menu

    @property
    def view_menu(self):
        return self.__view_menu
