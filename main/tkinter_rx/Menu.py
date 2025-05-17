import tkinter as tk
from typing import Any, Dict, Optional, cast

from reactivex.abc import DisposableBase
from typing_extensions import Unpack, override

from ratmap_common import EventTarget
from tkinter_rx.util import bind_subject_to_variable, to_snake_case

from .Checkbutton import CheckbuttonEvent
from .TkinterEvent import TkinterEvent, TkinterEventDetail
from .typing import BaseMenuCheckbuttonKwargs, MenuCheckbuttonKwargs, MenuKwargs


class MenuEvent(TkinterEvent):
    @staticmethod
    def factory(
        menu: "Menu", *path: str, type: str = "activate", additional: Any = None
    ):
        type = ".".join([type, *path])
        detail = TkinterEventDetail(menu, additional)
        event = MenuEvent(type, menu, detail)
        return event

    @staticmethod
    def emit_factory(
        menu: "Menu", *path: str, type: str = "Activate", additional: Any = None
    ):
        menu.emit(MenuEvent.factory(menu, *path, type=type, additional=additional))


class Menu(tk.Menu):
    __event_target: EventTarget
    __menu_disposers: Dict[str, DisposableBase]

    def __init__(
        self,
        master: Optional[tk.Misc] = None,
        **kwargs: Unpack[MenuKwargs],
    ) -> None:
        super().__init__(master, **kwargs)
        self.__event_target = EventTarget()
        self.__menu_disposers = dict()

    def remove(self, label: str) -> bool:
        length = self.index("end")
        if length is None:
            return False

        for i in range(length + 1):
            try:
                if self.entrycget(i, "label") == label:
                    self.delete(i)
                    if label in self.__menu_disposers:
                        self.__menu_disposers[label].dispose()
                        del self.__menu_disposers[label]
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

        for disposer in self.__menu_disposers.values():
            disposer.dispose()
        self.__menu_disposers = dict()

        return super().destroy()

    def add_checkbutton_rx(
        self,
        **kwargs: Unpack[MenuCheckbuttonKwargs],
    ) -> None:
        kwargs = kwargs.copy()

        variable = kwargs.pop("variable", None)
        value_subject = kwargs.pop("valuesubject", None)

        key = to_snake_case(kwargs["label"])

        if variable is None:
            variable = tk.BooleanVar(self)

        change_event = kwargs.pop(
            "changeevent",
            f"change.{key}",
        )

        def on_change_callback(new_val: bool):
            event = CheckbuttonEvent(
                change_event, detail=TkinterEventDetail(self, new_val)
            )
            self.__event_target.emit(event)

        if value_subject is not None:
            disposer = bind_subject_to_variable(
                value_subject, variable, self, on_change_callback
            )
            self.__menu_disposers[kwargs["label"]] = disposer

        super().add_checkbutton(
            **cast(BaseMenuCheckbuttonKwargs, kwargs),
            onvalue=True,
            offvalue=False,
            variable=variable,
        )
