from __future__ import annotations

import tkinter as tk

from typing_extensions import override

from ratmap_common import EventTarget
from tkinter_rx import Frame, Notebook
from tkinter_rx.util import SetDisposer, bind_recursive, disposable_bind

from .Legends import Legends
from .MainGl import MainGl
from .MenuMain import MenuMain


class MainWindow(tk.Tk):
    __disposer: SetDisposer

    __main_menu: MenuMain
    __event_target: EventTarget
    __main_gl: MainGl
    __toolbar: Notebook

    __gl_frame: Frame

    def __init__(self) -> None:

        super().__init__()
        self.title("RatMap")

        self.__event_target = EventTarget()

        self.__main_menu = MenuMain(self)
        self.__main_menu.event_target.parent = self.__event_target

        self.__gl_frame = Frame(self, width=640, height=480)

        self.__main_gl = MainGl(self.__gl_frame)
        self.__main_gl.event_target.parent = self.__event_target

        self.__main_gl.place(relwidth=1.0, relheight=1.0, x=0, y=0)

        self.__gl_frame.grid(row=0, column=0, sticky=tk.NSEW)

        self.__legends = Legends(self.__gl_frame)

        self.__toolbar = Notebook(self, width=320)

        self.__disposer = SetDisposer()

        _ = self.config(menu=self.__main_menu)
        _ = self.columnconfigure(0, weight=1)
        _ = self.rowconfigure(0, weight=1)

        self.__disposer.add(
            disposable_bind(self, "<KeyPress>", self.__handle_movement),
            bind_recursive(self.__legends, "<Button-1>", self.__close_legends),
            self.main_menu.view_menu.show_legends.subscribe(self.__legends_state),
            self.main_menu.view_menu.show_toolbar.subscribe(self.__toolbar_state),
        )

    def __legends_state(self, state: bool):
        if not state:
            self.__legends.place_forget()
            return

        self.__legends.place(x=8, rely=0.98, anchor=tk.SW)

    def __toolbar_state(self, state: bool):
        if not state:
            self.__toolbar.grid_forget()
            return

        self.__toolbar.grid(row=0, column=1, sticky=tk.NSEW)

    def __close_legends(self, _event: tk.Event[tk.Misc]):
        self.main_menu.view_menu.show_legends.on_next(False)

    @property
    def event_target(self):
        return self.__event_target

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

    @property
    def gl_frame(self):
        return self.__gl_frame

    @property
    def toolbar(self):
        return self.__toolbar

    @override
    def destroy(self) -> None:
        self.__disposer.dispose()
        self.__event_target.dispose()
        return super().destroy()
