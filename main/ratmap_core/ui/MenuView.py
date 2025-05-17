from reactivex.subject import BehaviorSubject
from typing_extensions import Unpack

from tkinter_rx import Menu
from tkinter_rx.typing import SubMenuKwargs


class MenuView(Menu):
    __show_legends: BehaviorSubject[bool]

    def __init__(self, menu_bar: Menu, **kwargs: Unpack[SubMenuKwargs]) -> None:
        super().__init__(menu_bar, **kwargs, tearoff=0)

        self.__show_legends = BehaviorSubject(False)

        self.add_checkbutton_rx(label="Legends", valuesubject=self.__show_legends)

        menu_bar.add_cascade(label="View", menu=self)

    @property
    def show_legends(self):
        return self.__show_legends
