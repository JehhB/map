import tkinter as tk
from tkinter import ttk
from typing import Literal, Optional, cast

from reactivex import Observable, Subject, operators
from reactivex.abc import DisposableBase
from typing_extensions import Unpack

from ratmap_common import EventTarget
from tkinter_rx.util import (
    bind_subject_to_variable,
    safe_callback,
    sync_observable_to_variable,
)

from .TkinterEvent import TkinterEvent, TkinterEventDetail
from .typing import BaseCheckbuttonKwargs, CheckbuttonKwargs


class CheckbuttonEvent(TkinterEvent):
    pass


class Checkbutton(ttk.Checkbutton):
    __text_variable: tk.StringVar
    __text_observable: Optional[Observable[str]]
    __text_disposer: Optional[DisposableBase]

    __value_subject: Optional[Subject[bool]]
    __value_disposer: Optional[DisposableBase]

    __state_observable: Optional[Observable[Literal["normal", "disabled"]]]
    __state_disposer: Optional[DisposableBase]

    __event_target: EventTarget
    __change_event: str

    def __init__(
        self,
        master: Optional[tk.Misc] = None,
        **kwargs: Unpack[CheckbuttonKwargs],
    ) -> None:
        kwargs = kwargs.copy()

        text_variable = kwargs.pop("textvariable", None)
        text_observable = kwargs.pop("textobservable", None)
        variable = kwargs.pop("variable", None)
        value_subject = kwargs.pop("valuesubject", None)
        state_observable = kwargs.pop("stateobservable", None)
        self.__change_event = kwargs.pop("changeevent", "change")

        super().__init__(
            master, **cast(BaseCheckbuttonKwargs, kwargs), onvalue=True, offvalue=False
        )

        if text_variable is None:
            text = None if "text" not in kwargs else str(kwargs.pop("text"))
            text_variable = tk.StringVar(self, text)
        self.__text_variable = text_variable
        _ = self.configure(textvariable=text_variable)

        self.__text_observable = None
        self.__text_disposer = None
        self.text_observable = text_observable

        if variable is None:
            variable = tk.BooleanVar(self)
        self.__variable = variable
        _ = self.configure(variable=variable)

        self.__value_disposer = None
        self.__state_disposer = None

        self.__value_subject = None
        self.__state_observable = None

        self.value_subject = value_subject
        self.state_observable = state_observable

        self.__event_target = EventTarget()

    @property
    def event_target(self):
        return self.__event_target

    @property
    def value_subject(self) -> Optional[Subject[bool]]:
        return self.__value_subject

    @value_subject.setter
    def value_subject(self, subject: Optional[Subject[bool]]):
        del self.value_subject

        self.__value_subject = subject

        if subject is None:
            return

        def on_change_callback(new_val: bool):
            event = CheckbuttonEvent(
                self.__change_event, detail=TkinterEventDetail(self, new_val)
            )
            self.__event_target.emit(event)

        self.__value_disposer = bind_subject_to_variable(
            subject, self.__variable, self, on_change_callback
        )

    @value_subject.deleter
    def value_subject(self):
        if self.__value_disposer is not None:
            self.__value_disposer.dispose()
            self.__value_disposer = None

        self.__value_subject = None

    @property
    def state_observable(
        self,
    ) -> Optional[Observable[Literal["normal", "disabled"]]]:
        return self.__state_observable

    @state_observable.setter
    def state_observable(
        self,
        observable: Optional[Observable[Literal["normal", "disabled"]]],
    ):
        del self.state_observable

        self.__state_observable = observable

        def update_state(new_state: Literal["normal", "readonly", "disabled"]):
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
