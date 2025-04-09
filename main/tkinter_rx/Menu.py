import tkinter as tk
from dataclasses import dataclass
from typing import Any, Optional, Tuple, Union

from typing_extensions import Unpack, override

from ratmap_common import AbstractEvent, EventTarget

from .typing import MenuKwargs


@dataclass
class MenuEventDetail:
    origin: "Menu"
    path: Tuple[str, ...]
    additional: Any = None


class MenuEvent(AbstractEvent):
    detail: Union[MenuEventDetail, Any]

    @staticmethod
    def factory(menu: "Menu", *path: str, type: str = "activate"):
        event = MenuEvent(type)
        event.detail = MenuEventDetail(origin=menu, path=path)
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

    def emit(self, event: AbstractEvent):
        self.__event_target.emit(event)

        if (
            event.type != "activate"
            or not isinstance(event, MenuEvent)
            or not isinstance(event.detail, MenuEventDetail)
        ):
            return

        detail = event.detail
        current_path = "activate"
        for sub_path in detail.path:
            current_path += "." + sub_path
            event = MenuEvent(current_path, event.target, detail)
            self.__event_target.emit(event)

    @property
    def event_target(self):
        return self.__event_target

    @override
    def destroy(self) -> None:
        self.__event_target.dispose()
        return super().destroy()
