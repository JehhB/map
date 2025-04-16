from __future__ import annotations

import tkinter as tk
from tkinter import DoubleVar, ttk
from typing import Literal, Optional, cast

from reactivex import Observable, Subject, operators
from reactivex.abc import DisposableBase
from typing_extensions import Unpack, override

from ratmap_common import EventTarget

from .TkinterEvent import TkinterEvent, TkinterEventDetail
from .typing import BaseScaleKwargs, ScaleKwargs
from .util import bind_subject_to_variable, safe_callback


class ScaleEvent(TkinterEvent):
    pass


class Scale(ttk.Scale):
    __state_observable: Optional[Observable[Literal["normal", "disabled"]]]
    __state_disposer: Optional[DisposableBase]

    __variable: tk.DoubleVar
    __value_subject: Optional[Subject[float]]
    __value_disposer: Optional[DisposableBase]
    __value_write_cbn: Optional[str]

    __event_target: EventTarget

    def __init__(
        self,
        master: Optional[tk.Misc] = None,
        **kwargs: Unpack[ScaleKwargs],
    ) -> None:
        kwargs = kwargs.copy()

        variable = kwargs.pop("variable", None)
        value_subject = kwargs.pop("value_subject", None)
        state_observable = kwargs.pop("state_observable", None)

        super().__init__(master, **cast(BaseScaleKwargs, kwargs))

        if variable is None:
            value = kwargs.pop("value", 0.0)
            variable = DoubleVar(self, value)
        self.__variable = variable
        _ = self.configure(variable=variable)

        self.__value_disposer = None
        self.__value_write_cbn = None
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
    def value_subject(self) -> Optional[Subject[float]]:
        return self.__value_subject

    @value_subject.setter
    def value_subject(self, subject: Optional[Subject[float]]):
        del self.value_subject

        self.__value_subject = subject

        if subject is None:
            return

        def on_change_callback(new_val: float):
            event = ScaleEvent("on_change", detail=TkinterEventDetail(new_val))
            self.__event_target.emit(event)

        (self.__value_disposer, self.__value_write_cbn) = bind_subject_to_variable(
            subject, self.__variable, self, on_change_callback
        )

    @value_subject.deleter
    def value_subject(self):
        if self.__value_disposer is not None:
            self.__value_disposer.dispose()
            self.__value_disposer = None

        if self.__value_write_cbn is not None:
            self.__variable.trace_remove("write", self.__value_write_cbn)
            self.__value_write_cbn = None

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

        def update_state(new_state: Literal["normal", "disabled"]):
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
        del self.value_subject
        del self.state_observable

        self.__event_target.dispose()
        return super().destroy()
