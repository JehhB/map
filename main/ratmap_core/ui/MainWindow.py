import tkinter as tk

from typing_extensions import override

from ratmap_common import EventTarget

from .MainGl import MainGl
from .MenuMain import MenuMain


class MainWindow(tk.Tk):

    __main_menu: MenuMain
    __event_target: EventTarget
    __main_gl: MainGl

    def __init__(self) -> None:

        super().__init__()
        self.title("RatMap")

        self.__event_target = EventTarget()

        self.__main_menu = MenuMain(self)
        self.__main_menu.event_target.parent = self.__event_target

        self.__main_gl = MainGl(self)
        self.__main_gl.event_target.parent = self.__event_target

        self.__main_gl.pack(expand=tk.YES, fill=tk.BOTH)

        _ = self.config(menu=self.__main_menu)

    @property
    def event_target(self):
        return self.__event_target

    @override
    def destroy(self) -> None:
        self.__event_target.dispose()
        return super().destroy()
