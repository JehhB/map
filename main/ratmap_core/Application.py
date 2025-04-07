from ratmap_common import EventTarget

from .ui import MainGl, MainWindow


class Application(EventTarget):
    main_window: MainWindow
    main_gl: MainGl

    def __init__(self) -> None:
        super().__init__()

        self.main_window = MainWindow()
        self.main_window.event_target.parent = self

    def mainloop(self):
        self.main_window.mainloop()
