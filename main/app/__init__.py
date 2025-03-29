from typing import Optional, final

from app.AbstractExtension import AbstractExtension
from app.Container import Container
from app.ExtensionManager import ExtensionManager
from app.ui.Main import Main
from app.ui.MainGl import MainGl
from app.ui.MenuBar import MenuBar
from stereo_vslam import StereoVslamExtension


@final
class Application:

    def __init__(self) -> None:
        self.container = Container()

        _ = self.container.register(Main)
        _ = self.container.register(MenuBar)
        _ = self.container.register(MainGl)
        _ = self.container.register(ExtensionManager)

        def safe_enable(extension: Optional[AbstractExtension]) -> bool:
            if extension is None:
                return False
            return extension.enable()

        _ = safe_enable(self.container.register(StereoVslamExtension))

    def mainloop(self):
        main = self.container[Main]
        main.mainloop()


application = Application()
