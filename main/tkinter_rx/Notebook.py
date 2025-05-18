from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Optional

from typing_extensions import Unpack, override

from ratmap_common import EventTarget
from tkinter_rx.TkinterEvent import TkinterEvent
from tkinter_rx.util import disposable_bind

from .typing import NotebookKwargs


class Notebook(ttk.Notebook):

    def __init__(
        self,
        master: Optional[tk.Misc],
        **kwargs: Unpack[NotebookKwargs],
    ) -> None:
        super().__init__(
            master,
            **kwargs,
        )
        self.__event_target = EventTarget()

        self.__tab_disposer = disposable_bind(
            self, "<<NotebookTabChanged>>", self.__on_change
        )

    def __on_change(self, event: tk.Event[tk.Misc]):
        self.__event_target.emit(TkinterEvent("tab_change", self, event))

    def index_of(self, label: str) -> int:
        tabs = self.tabs()

        for index, tab_id in enumerate(tabs):
            tab_label = self.tab(tab_id, "text")
            if tab_label == label:
                return index

        return -1

    def select_with(self, label: str):
        id = self.index_of(label)
        if id == -1:
            return

        self.select(id)

    def remove(self, label: str) -> bool:
        id = self.index_of(label)
        if id != -1:
            self.forget(id)

    @override
    def destroy(self) -> None:
        self.__tab_disposer.dispose()
        self.__event_target.dispose()
        return super().destroy()

    @property
    def event_target(self):
        return self.__event_target
