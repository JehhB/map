from typing_extensions import Unpack

from tkinter_rx import Menu
from tkinter_rx.typing import SubMenuKwargs


class MenuEdit(Menu):
    def __init__(self, menu_bar: Menu, **kwargs: Unpack[SubMenuKwargs]) -> None:
        super().__init__(menu_bar, **kwargs, tearoff=0)
        menu_bar.add_cascade(label="Edit", menu=self)
