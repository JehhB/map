from __future__ import annotations

import tkinter as tk

from typing_extensions import override

from ratmap_common import EventTarget
from tkinter_rx import Frame, Label
from tkinter_rx.util import SetDisposer, bind_recursive, disposable_bind

from .MainGl import MainGl
from .MenuMain import MenuMain


class MainWindow(tk.Tk):
    __disposer: SetDisposer

    __main_menu: MenuMain
    __event_target: EventTarget
    __main_gl: MainGl

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

        self.__gl_frame.pack(side=tk.TOP, expand=tk.YES, fill=tk.BOTH)

        self.__legends = Frame(self)

        Label(self.__legends, text="Legends").pack(side=tk.TOP, pady=4)

        legend = Frame(self.__legends)
        canvas = tk.Canvas(legend, width=8, height=8, bg="#000000")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Unknown space").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        legend = Frame(self.__legends)
        canvas = tk.Canvas(legend, width=8, height=8, bg="#eeeeee")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Ground").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        legend = Frame(self.__legends)
        canvas = tk.Canvas(legend, width=8, height=8, bg="#dd2222")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Obstacle").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        legend = Frame(self.__legends)
        canvas = tk.Canvas(legend, width=18, height=3, bg="#ffff00")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Current position").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        legend = Frame(self.__legends)
        canvas = tk.Canvas(legend, width=18, height=3, bg="#0000ff")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Recorded path").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        legend = Frame(self.__legends)
        canvas = tk.Canvas(legend, width=18, height=3, bg="#00ff00")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Planned path").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        legend = Frame(self.__legends)
        canvas = tk.Canvas(legend, width=16, height=16)
        _ = canvas.create_oval(1, 2, 15, 14, fill="#22dd22", outline="")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Target").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        self.__disposer = SetDisposer()

        _ = self.config(menu=self.__main_menu)

        self.__disposer.add(
            disposable_bind(self, "<KeyPress>", self.__handle_movement),
            bind_recursive(self.__legends, "<Button-1>", self.__close_legends),
            self.main_menu.view_menu.show_legends.subscribe(self.__legends_state),
        )

    def __legends_state(self, state: bool):
        if not state:
            self.__legends.place_forget()
            return

        self.__legends.place(x=8, rely=0.98, anchor=tk.SW)

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

    @override
    def destroy(self) -> None:
        self.__disposer.dispose()
        self.__event_target.dispose()
        return super().destroy()
