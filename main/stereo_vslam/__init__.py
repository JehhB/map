from PIL.Image import Image
from pubsub import pub
from typing_extensions import Self, override

from app.AbstractEvent import AbstractEvent
from app.AbstractExtension import AbstractExtension
from app.Container import AbstractModule, ModuleDefinition
from app.ExtensionManager import ExtensionManager
from app.ui.Main import Main as AppMain
from app.ui.MenuBar import MenuBar
from stereo_vslam.RosBridge import RosBridge
from stereo_vslam.ui.Main import Main


class StereoVslamExtension(AbstractModule, AbstractExtension):
    EXTENSION_LABEL: str = "Stereo VSLAM"
    menu_bar: "MenuBar"
    main_window: "Main | None"
    ros_bridge: "RosBridge | None"

    IMAGES_TOPIC: str = "stereo-vslam.images"

    @override
    @staticmethod
    def KEY() -> str:
        return "stereo_vslam.main"

    @override
    @classmethod
    def DEFINITION(cls) -> ModuleDefinition[Self]:
        return (
            ExtensionManager.defaultFactory("stereo-vslam", cls),
            [ExtensionManager, MenuBar, AppMain],
        )

    def __init__(self):
        super().__init__()

        self.main_window = None
        self.ros_bridge = None

        self.add_event_handler("init", self.on_init)
        self.add_event_handler("deinit", self.on_deinit)

    def on_init(self, _: AbstractEvent):
        if self.container is None:
            raise RuntimeError("uninitialied container")

        self.menu_bar = self.container[MenuBar]
        self.menu_bar.extension_menu.add_command(
            label=StereoVslamExtension.EXTENSION_LABEL, command=self.open_window
        )
        pub.subscribe(self.process_images, StereoVslamExtension.IMAGES_TOPIC)
        self.ros_bridge = RosBridge()

    def on_deinit(self, _: AbstractEvent):
        if self.ros_bridge is not None:
            self.ros_bridge.destroy()
            self.ros_bridge = None

        menu_index = self.menu_bar.index(StereoVslamExtension.EXTENSION_LABEL)
        if menu_index is not None:
            self.menu_bar.delete(menu_index)

        pub.unsubscribe(self.process_images, StereoVslamExtension.IMAGES_TOPIC)

    def open_window(self):
        if self.container is None:
            return

        if self.main_window is not None:
            self.main_window.lift()
            return

        self.main_window = Main(
            self,
            self.container[AppMain],
        )

        def on_close():
            if self.main_window is None:
                return

            self.main_window.destroy()
            self.main_window = None

        self.main_window.protocol("WM_DELETE_WINDOW", on_close)

    def process_images(self, left_image: Image, right_image: Image):
        pass
