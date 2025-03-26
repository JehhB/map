import tkinter as tk
from typing import override

from app.Container import AbstractModule, Container, ModuleDefinition


class Main(AbstractModule, tk.Tk):
    _container: Container

    @override
    @staticmethod
    def KEY() -> str:
        return "ui.main"

    @override
    @classmethod
    def DEFINITION(cls) -> ModuleDefinition["Main"]:
        return Main.factory, None

    @staticmethod
    def factory(container: Container):
        return Main(container)

    def __init__(
        self,
        container: Container,
        screenName: str | None = None,
        baseName: str | None = None,
        className: str = "Tk",
        useTk: bool = True,
        sync: bool = False,
        use: str | None = None,
    ) -> None:
        super().__init__(screenName, baseName, className, useTk, sync, use)
        self._container = container
