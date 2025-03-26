from typing import final

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

        self.container.register(Main)
        self.container.register(MenuBar)
        self.container.register(MainGl)
        self.container.register(ExtensionManager)
        self.container.register(StereoVslamExtension)

    def mainloop(self):
        main = self.container[Main]
        main.mainloop()


application = Application()
