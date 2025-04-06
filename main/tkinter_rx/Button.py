import tkinter as tk
from tkinter import ttk
from typing import Callable, Literal, cast

from reactivex import Observable, operators
from reactivex.abc import DisposableBase
from typing_extensions import Optional, Unpack, override

from ratmap_common import EventTarget
from ratmap_common.AbstractEvent import AbstractEvent

from .typing import BaseButtonKwargs, ButtonKwargs
from .util import safe_callback, sync_observable_to_variable


class ButtonEvent(AbstractEvent):
    pass


class Button(ttk.Button, EventTarget):
    __command: Optional[Callable[[], None]]

    __text_variable: tk.StringVar
    __text_observable: Optional[Observable[str]]
    __text_disposer: Optional[DisposableBase]

    __state_observable: Optional[Observable[Literal["normal", "active", "disabled"]]]
    __state_disposer: Optional[DisposableBase]

    def __init__(
        self, master: Optional[tk.Misc] = None, **kwargs: Unpack[ButtonKwargs]
    ) -> None:
        kwargs = kwargs.copy()

        self.__command = kwargs.pop("command", None)
        self.__text_disposer = None
        self.__state_disposer = None

        variable = kwargs.pop("textvariable", None)
        if variable is None:
            text = None if "text" not in kwargs else str(kwargs.pop("text"))
            variable = tk.StringVar(self, text)
        self.__text_variable = variable

        text_observable = kwargs.pop("textobservable", None)
        state_observable = kwargs.pop("stateobservable", None)

        super().__init__(
            master,
            **cast(BaseButtonKwargs, kwargs),
            command=self.__on_click_handler,
            textvariable=variable
        )

        self.text_observable = text_observable
        self.state_observable = state_observable

    def __on_click_handler(self):
        if self.__command:
            self.__command()
        self.emit("click", ButtonEvent())

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

    @property
    def state_observable(
        self,
    ) -> Optional[Observable[Literal["normal", "active", "disabled"]]]:
        return self.__state_observable

    @state_observable.setter
    def state_observable(
        self, observable: Optional[Observable[Literal["normal", "active", "disabled"]]]
    ):
        del self.state_observable

        self.__state_observable = observable

        def update_state(new_state: Literal["normal", "active", "disabled"]):
            _ = self.config(state=new_state)

        if observable is None:
            return

        self.__state_disposer = observable.pipe(operators.debounce(0.1)).subscribe(
            safe_callback(self, update_state)
        )
        self.state_observable = observable

    @state_observable.deleter
    def state_observable(self):
        if self.__state_disposer is not None:
            self.__state_disposer.dispose()
            self.__state_disposer = None
        self.__state_observable = None

    @override
    def destroy(self) -> None:
        del self.state_observable
        del self.text_observable

        self.dispose()
        return super().destroy()
