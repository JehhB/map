import tkinter as tk
from typing import Optional

from typing_extensions import Unpack, override

from .Label import Label
from .typing import LabelKwargs


class WrappingLabel(Label):
    __resize_cbn: str

    def __init__(
        self, master: Optional[tk.Misc] = None, **kwargs: Unpack[LabelKwargs]
    ) -> None:
        super().__init__(master, **kwargs)
        self.__resize_cbn = self.bind(
            "<Configure>",
            lambda e: self.config(wraplength=self.winfo_width()),
        )

    @override
    def destroy(self) -> None:
        self.unbind("<Configure>", self.__resize_cbn)
        return super().destroy()
