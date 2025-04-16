import tkinter as tk
from typing import Any, Optional

from typing_extensions import Unpack, override

from ratmap_common import EventTarget

from .TkinterEvent import TkinterEvent, TkinterEventDetail
from .typing import MenuKwargs


class MenuEvent(TkinterEvent):
    @staticmethod
    def factory(
        menu: "Menu", *path: str, type: str = "activate", additional: Any = None
    ):
        type = ".".join([type, *path])
        detail = TkinterEventDetail(additional)
        event = MenuEvent(type, menu, detail)
        return event


class Menu(tk.Menu):
    __event_target: EventTarget

    def __init__(
        self,
        master: Optional[tk.Misc] = None,
        **kwargs: Unpack[MenuKwargs],
    ) -> None:
        super().__init__(master, **kwargs)
        self.__event_target = EventTarget()

    def remove(self, label: str) -> bool:
        length = self.index("end")
        if length is None:
            return False

        for i in range(length + 1):
            try:
                if self.entrycget(i, "label") == label:
                    self.delete(i)
                    return True
            except:
                pass
        return False

    def emit(self, event: MenuEvent):
        self.__event_target.emit(event)

    @property
    def event_target(self):
        return self.__event_target

    @override
    def destroy(self) -> None:
        self.__event_target.dispose()
        return super().destroy()
