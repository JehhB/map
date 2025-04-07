import tkinter as tk
from typing import Optional

from typing_extensions import override

from ratmap_common import EventTarget

from .MenuMain import MenuMain


class MainWindow(tk.Tk):
    main_menu: MenuMain
    __event_target: EventTarget

    def __init__(
        self,
        screenName: Optional[str] = None,
        baseName: Optional[str] = None,
        className: str = "Tk",
        useTk: bool = True,
        sync: bool = False,
        use: Optional[str] = None,
    ) -> None:

        super().__init__(screenName, baseName, className, useTk, sync, use)
        self.title("RatMap")

        self.main_menu = MenuMain(self)
        self.__event_target = EventTarget()
        self.main_menu.event_target.parent = self.__event_target

        _ = self.config(menu=self.main_menu)

    @property
    def event_target(self):
        return self.__event_target

    @override
    def destroy(self) -> None:
        self.__event_target.dispose()
        return super().destroy()
