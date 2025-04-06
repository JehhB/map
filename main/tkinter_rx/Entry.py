import tkinter as tk
from tkinter import ttk
from typing import Literal, Optional, cast

from reactivex import Observable, Subject, operators
from reactivex.abc import DisposableBase
from typing_extensions import Unpack, override

from ratmap_common.AbstractEvent import AbstractEvent
from ratmap_common.EventTarget import EventTarget

from .typing import BaseEntryKwargs, EntryKwargs
from .util import bind_subject_to_variable, safe_callback


class EntryEvent(AbstractEvent):
    pass


class Entry(ttk.Entry, EventTarget):
    __state_observable: Optional[Observable[Literal["normal", "disabled", "readonly"]]]
    __state_disposer: Optional[DisposableBase]

    __text_variable: tk.StringVar
    __text_subject: Optional[Subject[str]]
    __text_disposer: Optional[DisposableBase]
    __text_write_cbn: Optional[str]

    __focus_cbn: str
    __blur_cbn: str

    def __init__(
        self, master: Optional[tk.Misc] = None, **kwargs: Unpack[EntryKwargs]
    ) -> None:
        kwargs = kwargs.copy()

        self.__state_disposer = None
        self.__state_observable = None

        self.__text_subject = None
        self.__text_disposer = None
        self.__text_write_cbn = None

        text_variable = kwargs.pop("textvariable", None)
        if text_variable is None:
            text_variable = tk.StringVar(self)
        self.__text_variable = text_variable

        text_subject = kwargs.pop("textsubject", None)
        if text_subject is None:
            text_subject = cast(Subject[str], Subject())

        state_observable = kwargs.pop("stateobservable", None)

        super().__init__(
            master, **cast(BaseEntryKwargs, kwargs), textvariable=text_variable
        )

        self.state_observable = state_observable
        self.text_subject = text_subject

        self.__focus_cbn = self.bind(
            "<FocusIn>", lambda _: self.emit("focus", EntryEvent())
        )
        self.__blur_cbn = self.bind(
            "<FocusOut>", lambda _: self.emit("blur", EntryEvent())
        )

    @property
    def state_observable(
        self,
    ) -> Optional[Observable[Literal["normal", "readonly", "disabled"]]]:
        return self.__state_observable

    @state_observable.setter
    def state_observable(
        self,
        observable: Optional[Observable[Literal["normal", "readonly", "disabled"]]],
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
    def text_subject(self) -> Optional[Subject[str]]:
        return self.text_subject

    @text_subject.setter
    def text_subject(self, subject: Optional[Subject[str]]):
        del self.text_subject

        self.__text_subject = subject

        if subject is None:
            return

        def on_change_callback(new_val: str):
            event = EntryEvent()
            event.detail = new_val
            self.emit("on_change", event)

        (self.__text_disposer, self.__text_write_cbn) = bind_subject_to_variable(
            subject, self.__text_variable, self, on_change_callback
        )

    @text_subject.deleter
    def text_subject(self):
        if self.__text_disposer is not None:
            self.__text_disposer.dispose()
            self.__text_disposer = None

        if self.__text_write_cbn is not None:
            self.__text_variable.trace_remove("write", self.__text_write_cbn)
            self.__text_write_cbn = None

        self.__text_subject = None

    @override
    def destroy(self) -> None:
        del self.text_subject
        del self.state_observable

        self.unbind("<FocusIn>", self.__focus_cbn)
        self.unbind("<FocusOut>", self.__blur_cbn)

        return super().destroy()
