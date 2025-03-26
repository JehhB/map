from typing import Self, override

from PIL.Image import Image
from pubsub import pub

from app.AbstractEvent import AbstractEvent
from app.AbstractExtension import AbstractExtension
from app.Container import AbstractModule, ModuleDefinition
from app.ExtensionManager import ExtensionManager
from app.ui.Main import Main as AppMain
from app.ui.MenuBar import MenuBar
from stereo_vslam.ui.Main import Main


class StereoVslamExtension(AbstractModule, AbstractExtension):
    EXTENSION_LABEL: str = "Stereo VSLAM"
    main_window: "Main | None"

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
        self.add_event_handler("init", self.on_init)

        pub.subscribe(self.process_images, StereoVslamExtension.IMAGES_TOPIC)

    def on_init(self, _: AbstractEvent):
        if self.container is None:
            raise RuntimeError("uninitialied container")

        menu_bar = self.container[MenuBar]
        menu_bar.extension_menu.add_command(
            label=StereoVslamExtension.EXTENSION_LABEL, command=self.open_window
        )

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

    def process_images(self, left_image: Image, right_image: Image):
        pass
