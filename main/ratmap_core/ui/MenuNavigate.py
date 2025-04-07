from typing_extensions import Unpack

from tkinter_rx import Menu, MenuEvent
from tkinter_rx.typing import SubMenuKwargs


class MenuNavigate(Menu):
    def __init__(self, menu_bar: Menu, **kwargs: Unpack[SubMenuKwargs]) -> None:
        super().__init__(menu_bar, **kwargs, tearoff=0)

        self.add_command(label="Zoom +", command=self.navigate_zoom_in)
        self.add_command(label="Zoom -", command=self.navigate_zoom_out)
        self.add_command(label="Reset Zoom", command=self.navigate_zoom_reset)
        self.add_separator()
        self.add_command(label="Reorient", command=self.navigate_reorient)
        self.add_command(label="Recenter", command=self.navigate_recenter)
        self.add_separator()
        self.add_command(label="Fit to view", command=self.navigate_fit_view)

        menu_bar.add_cascade(label="Navigate", menu=self)

    def navigate_zoom_in(self):
        self.emit(MenuEvent.factory(self, "menu_main", "navigate", "zoom_in"))

    def navigate_zoom_out(self):
        self.emit(MenuEvent.factory(self, "menu_main", "navigate", "zoom_out"))

    def navigate_zoom_reset(self):
        self.emit(MenuEvent.factory(self, "menu_main", "navigate", "zoom_reset"))

    def navigate_reorient(self):
        self.emit(MenuEvent.factory(self, "menu_main", "navigate", "reorient"))

    def navigate_recenter(self):
        self.emit(MenuEvent.factory(self, "menu_main", "navigate", "recenter"))

    def navigate_fit_view(self):
        self.emit(MenuEvent.factory(self, "menu_main", "navigate", "fit_view"))
