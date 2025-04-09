from typing import Optional

from ratmap_common import AbstractEvent, EventTarget
from ratmap_core.ui.ManageExtensionsWindow import ManageExtensionsWindow

from .ExtensionManager import ExtensionManager
from .ui import MainGl, MainWindow


class Application(EventTarget):
    extension_manager: ExtensionManager
    main_window: MainWindow
    main_gl: MainGl

    __manage_extension_window: Optional[ManageExtensionsWindow]

    def __init__(self) -> None:
        super().__init__()

        self.main_window = MainWindow()
        self.main_window.event_target.parent = self
        self.main_gl = self.main_window.main_gl

        self.extension_manager = ExtensionManager()

        _ = self.add_event_listener(
            "activate.menu_main.extension.manage", self.open_window
        )
        self.__manage_extension_window = None

    def open_window(self, _event: AbstractEvent):
        if self.__manage_extension_window is not None:
            self.__manage_extension_window.lift()
            return

        self.__manage_extension_window = ManageExtensionsWindow(
            self.extension_manager,
            self.main_window,
        )

        def on_close():
            if self.__manage_extension_window is None:
                return

            self.__manage_extension_window.destroy()
            self.__manage_extension_window = None

        self.__manage_extension_window.protocol("WM_DELETE_WINDOW", on_close)

    def mainloop(self):
        self.main_window.mainloop()
