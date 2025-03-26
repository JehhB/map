import tkinter as tk
from typing import override

from app.Container import AbstractModule, Container, ModuleDefinition
from app.ui.Main import Main


class MenuBar(AbstractModule, tk.Menu):
    _container: Container
    file_menu: tk.Menu
    edit_menu: tk.Menu
    navigate_menu: tk.Menu
    extension_menu: tk.Menu

    @override
    @staticmethod
    def KEY():
        return "ui.menubar"

    @override
    @classmethod
    def DEFINITION(cls) -> ModuleDefinition["MenuBar"]:
        return MenuBar.factory, [Main]

    @staticmethod
    def factory(container: Container):
        main = container[Main]

        menu_bar = MenuBar(container, main)
        _ = main.config(menu=menu_bar)

        return menu_bar

    def __init__(
        self, container: Container, master: tk.Misc | None = None, **kwargs
    ) -> None:
        super().__init__(master, **kwargs)

        self._container = container

        self.file_menu = tk.Menu(self, tearoff=0)
        self.file_menu.add_command(label="Open", command=self.file_open)
        self.file_menu.add_command(label="Save", command=self.file_save)
        self.file_menu.add_command(label="Export", command=self.file_export)
        self.add_cascade(label="File", menu=self.file_menu)

        self.edit_menu = tk.Menu(self, tearoff=0)
        self.edit_menu.add_command(label="Clear", command=self.edit_clear)
        self.edit_menu.add_command(label="Undo", command=self.edit_undo)
        self.edit_menu.add_command(label="Redo", command=self.edit_redo)
        self.edit_menu.add_separator()
        self.edit_menu.add_command(label="Preferences", command=self.edit_preference)
        self.add_cascade(label="Edit", menu=self.edit_menu)

        self.navigate_menu = tk.Menu(self, tearoff=0)
        self.navigate_menu.add_command(label="Zoom +", command=self.navigate_zoom_in)
        self.navigate_menu.add_command(label="Zoom -", command=self.navigate_zoom_out)
        self.navigate_menu.add_command(
            label="Reset Zoom", command=self.navigate_zoom_reset
        )
        self.navigate_menu.add_separator()
        self.navigate_menu.add_command(label="Reorient", command=self.navigate_reorient)
        self.navigate_menu.add_command(label="Recenter", command=self.navigate_recenter)
        self.navigate_menu.add_separator()
        self.navigate_menu.add_command(
            label="Fit to view", command=self.navigate_fit_view
        )
        self.add_cascade(label="Navigate", menu=self.navigate_menu)

        self.extension_menu = tk.Menu(self, tearoff=0)
        self.extension_menu.add_command(
            label="Add extension", command=self.extension_add
        )
        self.extension_menu.add_command(
            label="Manage extension", command=self.extension_manage
        )
        self.extension_menu.add_separator()
        self.add_cascade(label="Extensions", menu=self.extension_menu)

    def file_open(self):
        pass

    def file_save(self):
        pass

    def file_export(self):
        pass

    def edit_clear(self):
        pass

    def edit_undo(self):
        pass

    def edit_redo(self):
        pass

    def edit_preference(self):
        pass

    def navigate_zoom_in(self):
        pass

    def navigate_zoom_out(self):
        pass

    def navigate_zoom_reset(self):
        pass

    def navigate_reorient(self):
        pass

    def navigate_recenter(self):
        pass

    def navigate_fit_view(self):
        pass

    def extension_add(self):
        pass

    def extension_manage(self):
        pass
