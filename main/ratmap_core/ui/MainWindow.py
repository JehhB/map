from __future__ import annotations

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
        _ = self.bind("<KeyPress>", self.__handle_movement)

    @property
    def event_target(self):
        return self.__event_target

    @override
    def destroy(self) -> None:
        self.__event_target.dispose()
        return super().destroy()

    @property
    def main_gl(self):
        return self.__main_gl

    @property
    def main_menu(self):
        return self.__main_menu

    def __handle_movement(self, event: tk.Event[tk.Misc]):
        if event.char == "w":
            self.__main_gl.camera.move("up", delta_time=0.05)
        elif event.char == "s":
            self.__main_gl.camera.move("down", delta_time=0.05)
        elif event.char == "a":
            self.__main_gl.camera.move("left", delta_time=0.05)
        elif event.char == "d":
            self.__main_gl.camera.move("right", delta_time=0.05)
