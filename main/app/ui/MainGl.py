import tkinter as tk

from OpenGL import GL
from pyopengltk import OpenGLFrame
from typing_extensions import override

from app.Container import AbstractModule, Container, ModuleDefinition
from app.ui.Main import Main


class MainGl(AbstractModule, OpenGLFrame):
    _container: Container

    @override
    @staticmethod
    def KEY() -> str:
        return "ui.maingl"

    @override
    @classmethod
    def DEFINITION(cls) -> ModuleDefinition["MainGl"]:
        return MainGl.factory, [Main]

    @staticmethod
    def factory(container: Container):
        main = container[Main]

        main_gl = MainGl(container, main, width=500, height=500)
        main_gl.pack(fill=tk.BOTH, expand=tk.YES)

        return main_gl

    def __init__(self, container: Container, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._container = container

    @override
    def initgl(self):
        """Initalize gl states when the frame is created"""
        GL.glViewport(0, 0, self.width, self.height)

    @override
    def redraw(self):
        """Render a single frame"""
        GL.glClearColor(0.0, 0.0, 0.0, 0.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT)
