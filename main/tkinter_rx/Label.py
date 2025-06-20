import tkinter as tk
from tkinter import ttk
from typing import cast

from reactivex import Observable
from reactivex.abc import DisposableBase
from typing_extensions import Optional, Unpack, override

from ratmap_common import EventTarget

from .TkinterEvent import TkinterEvent
from .typing import BaseLabelKwargs, LabelKwargs
from .util import sync_observable_to_variable


class LabelEvent(TkinterEvent[tk.Label]):
    pass


class Label(ttk.Label):
    __text_variable: tk.StringVar
    __text_observable: Optional[Observable[str]]
    __text_disposer: Optional[DisposableBase]

    __event_target: EventTarget

    def __init__(
        self, master: Optional[tk.Misc] = None, **kwargs: Unpack[LabelKwargs]
    ) -> None:
        kwargs = kwargs.copy()

        self.__text_disposer = None

        variable = kwargs.pop("textvariable", None)
        text_observable = kwargs.pop("textobservable", None)

        super().__init__(master, **cast(BaseLabelKwargs, kwargs))

        if variable is None:
            text = None if "text" not in kwargs else str(kwargs.pop("text"))
            variable = tk.StringVar(self, text)
        self.__text_variable = variable
        _ = self.configure(textvariable=variable)

        self.__text_observable = None
        self.text_observable = text_observable

        self.__event_target = EventTarget()

    @property
    def text_observable(self) -> Optional[Observable[str]]:
        return self.__text_observable

    @text_observable.setter
    def text_observable(self, text_observable: Optional[Observable[str]]):
        del self.text_observable

        self.__text_observable = text_observable

        if text_observable is None:
            return

        self.__text_disposer = sync_observable_to_variable(
            text_observable, self.__text_variable, self
        )

    @text_observable.deleter
    def text_observable(self):
        if self.__text_disposer is not None:
            self.__text_disposer.dispose()
            self.__text_disposer = None
        self.__text_observable = None

    @override
    def destroy(self) -> None:
        del self.__text_observable

        self.__event_target.dispose()
        return super().destroy()
