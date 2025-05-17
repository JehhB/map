from tkinter import filedialog
from typing import Optional

from ratmap_common import AbstractEvent, EventTarget
from tkinter_rx import Menu
from tkinter_rx.util import SetDisposer, disposable_bind

from .Config import Config
from .ExtensionManager import ExtensionManager
from .Joystick import Joystick, JoystickEvent
from .ui import (
    MainGl,
    MainWindow,
    ManageExtensionsWindow,
    MenuEdit,
    MenuExtension,
    MenuMain,
    MenuNavigate,
    MenuView,
)


class Application(EventTarget):
    __config: Config
    __extension_manager: ExtensionManager
    __main_window: MainWindow
    __main_gl: MainGl
    __joystick: Joystick

    __main_menu: MenuMain
    __extension_menu: MenuExtension
    __edit_menu: MenuEdit
    __navigate_menu: MenuNavigate
    __view_menu: MenuView

    __manage_extension_window: Optional[ManageExtensionsWindow]
    __disposer: SetDisposer

    def __init__(self) -> None:
        super().__init__()

        self.__config = Config()
        self.__main_window = MainWindow()
        self.__main_window.event_target.parent = self
        self.__main_gl = self.__main_window.main_gl

        self.__main_menu = self.__main_window.main_menu
        self.__edit_menu = self.__main_menu.edit_menu
        self.__navigate_menu = self.__main_menu.navigate_menu
        self.__extension_menu = self.__main_menu.extension_menu
        self.__view_menu = self.__main_menu.view_menu

        self.__extension_manager = ExtensionManager(self)
        self.__extension_manager.parent = self

        self.__disposer = SetDisposer()
        self.__disposer.add(
            self.add_event_listener(
                "activate.menu_main.extension.manage", self.open_window
            )
        )
        self.__manage_extension_window = None
        self.__joystick = Joystick()
        self.__joystick.parent = self

        show_legend: bool = self.config.get("core.show_legends", default=True)
        self.view_menu.show_legends.on_next(show_legend)

        self.__disposer.add(
            self.__joystick.add_event_listener("joystick.poll", self.__handle_movement)
        )

        zoom_sensitivity = 2.0

        def zoom(val: float):
            self.__main_gl.camera.zoom = self.__main_gl.camera.zoom + val

        def reset_zoom():
            self.__main_gl.camera.zoom = 10.0

        def recenter():
            self.__main_gl.camera.position[0] = 0.0
            self.__main_gl.camera.position[1] = 0.0

        self.__disposer.add(
            self.add_event_listener(
                "activate.menu_main.navigate.zoom_in", lambda e: zoom(-zoom_sensitivity)
            ),
            self.add_event_listener(
                "activate.menu_main.navigate.zoom_out", lambda e: zoom(zoom_sensitivity)
            ),
            self.add_event_listener(
                "activate.menu_main.navigate.zoom_reset", lambda e: reset_zoom()
            ),
            self.add_event_listener(
                "activate.menu_main.navigate.recenter", lambda e: recenter()
            ),
            disposable_bind(
                self.main_window, "<Control-plus>", lambda e: zoom(-zoom_sensitivity)
            ),
            disposable_bind(
                self.main_window, "<Control-minus>", lambda e: zoom(zoom_sensitivity)
            ),
            disposable_bind(
                self.main_window, "<Control-equal>", lambda e: reset_zoom()
            ),
            disposable_bind(self.main_window, "<Control-0>", lambda e: recenter()),
            self.add_event_listener(
                "activate.menu_main.extension.add", self.add_extension
            ),
            self.view_menu.show_legends.subscribe(
                lambda e: self.config.update("core.show_legends", e)
            ),
        )

    def add_extension(self, _e: AbstractEvent):
        filename = filedialog.askopenfilename(
            parent=self.main_window,
            title="Add extension",
            filetypes=[("ZIP files", "*.zip")],
            defaultextension="zip",
        )

        if not filename:
            return

        self.__extension_manager.add_zip(filename)

    def close_window(self):
        if self.__manage_extension_window is None:
            return

        self.__manage_extension_window.destroy()
        self.__manage_extension_window = None

    def open_window(self, _event: AbstractEvent):
        if self.__manage_extension_window is not None:
            self.__manage_extension_window.lift()
            return

        self.__manage_extension_window = ManageExtensionsWindow(
            self.__extension_manager,
            self.__main_window,
        )

        self.__manage_extension_window.protocol("WM_DELETE_WINDOW", self.close_window)

    def mainloop(self):
        self.__joystick.start_thread()
        self.__main_window.mainloop()

        self.__joystick.dispose()
        self.__extension_manager.dispose()
        self.__config.dispose()

    @property
    def extension_manager(self) -> ExtensionManager:
        return self.__extension_manager

    @property
    def config(self) -> Config:
        return self.__config

    @property
    def main_window(self) -> MainWindow:
        return self.__main_window

    @property
    def main_gl(self) -> MainGl:
        return self.__main_gl

    @property
    def main_menu(self) -> Menu:
        return self.__main_menu

    @property
    def edit_menu(self) -> Menu:
        return self.__edit_menu

    @property
    def navigate_menu(self) -> Menu:
        return self.__navigate_menu

    @property
    def extension_menu(self) -> Menu:
        return self.__extension_menu

    @property
    def view_menu(self) -> MenuView:
        return self.__view_menu

    @property
    def joystick(self) -> Joystick:
        return self.__joystick

    def __handle_movement(self, event: JoystickEvent):
        if not hasattr(self.__main_gl, "camera"):
            return

        detail = event.detail
        offsets = detail.left_stick

        if abs(offsets[0]) < 0.01 and abs(offsets[1]) < 0.01:
            return

        self.__main_gl.camera.pan(-offsets[0], offsets[1], delta_time=0.5)
