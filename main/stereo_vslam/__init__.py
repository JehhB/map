from typing import Self, override

from app.AbstractEvent import AbstractEvent
from app.AbstractExtension import AbstractExtension
from app.Container import AbstractModule, ModuleDefinition
from app.ExtensionManager import ExtensionManager
from app.ui.MenuBar import MenuBar


class StereoVslamExtension(AbstractModule, AbstractExtension):
    EXTENSION_LABEL = "Stereo VSLAM"

    @override
    @staticmethod
    def KEY() -> str:
        return "stereo_vslam.main"

    @override
    @classmethod
    def DEFINITION(cls) -> ModuleDefinition[Self]:
        return (
            ExtensionManager.defaultFactory("stereo-vslam", cls),
            [
                ExtensionManager,
                MenuBar,
            ],
        )

    def __init__(self):
        super().__init__()

        self.add_event_handler("init", self.onInitialize)

    def onInitialize(self, _: AbstractEvent):
        if self.container is None:
            raise RuntimeError("uninitialied container")

        menu_bar = self.container[MenuBar]
        menu_bar.extension_menu.add_command(
            label=StereoVslamExtension.EXTENSION_LABEL, command=self.openWindow
        )

    def openWindow(self):
        pass
