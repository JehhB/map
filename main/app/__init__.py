from typing import Optional, final

from app.AbstractExtension import AbstractExtension
from app.Container import Container
from app.events.InitEvent import InitEvent
from app.ExtensionManager import ExtensionManager
from app.ui.Main import Main
from app.ui.MainGl import MainGl
from app.ui.MenuBar import MenuBar
from euroc_mav_loader import EuRoCMAVLoaderExtension
from stereo_vslam import StereoVslamExtension


@final
class Application:

    def __init__(self) -> None:
        self.container = Container()

        _ = self.container.register(Main)
        _ = self.container.register(MenuBar)
        _ = self.container.register(MainGl)
        _ = self.container.register(ExtensionManager)

        def safe_enable(extension: Optional[AbstractExtension]) -> InitEvent:
            if extension is None:
                event = InitEvent()
                event.is_success = False
                event.detail = RuntimeError("Extension not found")
                return event
            return extension.enable()

        _ = safe_enable(self.container.register(StereoVslamExtension))
        _ = safe_enable(self.container.register(EuRoCMAVLoaderExtension))

    def mainloop(self):
        main = self.container[Main]
        main.mainloop()


application = Application()
